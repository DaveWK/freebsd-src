/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Daniel Shue <dgshue@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * phy(9) driver for the SpacemiT K1 (Ky X1) DEDICATED PCIe PHYs
 * (compatible: "spacemit,k1-pcie-phy") at 0xc0c10000 (pcie1) and 0xc0d10000
 * (pcie2).
 *
 * IMPORTANT SCOPE / SAFETY: this driver deliberately matches ONLY the
 * dedicated PCIe PHYs, NOT the combo PHY ("spacemit,k1-combo-phy" at
 * 0xc0b10000), which is owned by spacemit_combphy.c and is REQUIRED for the
 * working USB3 port.  The only interaction with the combo PHY here is a
 * strictly READ-ONLY access to its RX/TX termination calibration result
 * register (PCIE_RCAL_RESULT); we never write the combo PHY or flip its
 * PCIe/USB3 mux, so USB3 is never disturbed.  If the combo PHY has not been
 * calibrated (e.g. never brought up by U-Boot / the USB3 path), we decline to
 * enable the PCIe PHY rather than force a calibration that would break USB3.
 *
 * Register sequence reimplemented from the mainline Linux driver
 * drivers/phy/spacemit/phy-k1-pcie.c (GPL-2.0), NOT copied.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <machine/bus.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/phy/phy.h>

/* Base of the combo PHY (port A), which holds the shared calibration result. */
#define	K1_COMBO_PHY_BASE	0xc0b10000
#define	K1_COMBO_PHY_SIZE	0x1000

/* PHY register offsets (per-lane; second lane at +PHY_LANE_OFFSET). */
#define	PHY_LANE_OFFSET		0x0400

#define	PCIE_PU_ADDR_CLK_CFG	0x0008
#define	 PLL_READY		(1u << 0)
#define	 CFG_INTERNAL_TIMER_ADJ_M (0xfu << 7)
#define	 TIMER_ADJ_PCIE		(0x6u << 7)
#define	 CFG_SW_PHY_INIT_DONE	(1u << 11)

#define	PCIE_RC_DONE_STATUS	0x0018
#define	 CFG_FORCE_RCV_RETRY	(1u << 10)

#define	PCIE_RC_CAL_REG2	0x0020
#define	 RC_CAL_TOGGLE		(1u << 22)
#define	 CLKSEL_M		(0x7u << 29)
#define	 CLKSEL_24M		(0x3u << 29)

#define	PCIE_PU_PLL_1		0x0048
#define	 REF_100_WSSC		(1u << 12)
#define	 FREF_SEL_M		(0x7u << 13)
#define	 FREF_24M		(0x1u << 13)
#define	 SSC_DEP_SEL_M		(0xfu << 16)

#define	PCIE_PU_PLL_2		0x004c
#define	 GEN_REF100		(1u << 4)

#define	PCIE_RX_REG1		0x0050
#define	 EN_RTERM		(1u << 3)
#define	 AFE_RTERM_REG_M	(0xfu << 8)
#define	 AFE_RTERM_REG_S	8

#define	PCIE_RX_REG2		0x0054
#define	 RX_RTERM_SEL		(1u << 5)

#define	PCIE_LTSSM_DIS_ENTRY	0x005c
#define	 CFG_REFCLK_MODE_M	(0x3u << 8)
#define	 RFCLK_MODE_DRIVER	(0x1u << 8)
#define	 OVRD_REFCLK_MODE	(1u << 10)

#define	PCIE_TX_REG1		0x0064
#define	 TX_RTERM_REG_M		(0xfu << 12)
#define	 TX_RTERM_REG_S		12
#define	 TX_RTERM_SEL		(1u << 25)

/* Combo-PHY-only: RX/TX termination calibration result. */
#define	PCIE_RCAL_RESULT	0x0084
#define	 RTERM_VALUE_RX_M	0xfu
#define	 RTERM_VALUE_TX_M	0xf0u
#define	 R_TUNE_DONE		(1u << 10)

#define	PLL_LOCK_TIMEOUT_US	500000
#define	POLL_DELAY_US		500

struct sppcie_phy_softc {
	device_t	dev;
	struct resource	*mem;
	int		mem_rid;
	clk_t		refclk;
	hwreset_t	rst_phy;	/* "phy" (global) reset; deasserted for life */
	int		nlanes;
};

static struct ofw_compat_data compat_data[] = {
	{ "spacemit,k1-pcie-phy",	1 },
	{ NULL,				0 }
};

#define	RD4(sc, r)	bus_read_4((sc)->mem, (r))
#define	WR4(sc, r, v)	bus_write_4((sc)->mem, (r), (v))

/*
 * Read the shared RX/TX termination calibration from the combo PHY's result
 * register.  READ-ONLY: we map the combo PHY's register window, read the one
 * result register, and unmap -- we never write it, so USB3 is untouched.
 * Returns 0 and fills *rterm on success; ENXIO if calibration is not done.
 */
static int
sppcie_phy_get_rterm(device_t dev, uint32_t *rterm)
{
	char *va;
	uint32_t val;

	va = pmap_mapdev(K1_COMBO_PHY_BASE, K1_COMBO_PHY_SIZE);
	if (va == NULL)
		return (ENXIO);
	val = *(volatile uint32_t *)(va + PCIE_RCAL_RESULT);	/* read-only */
	pmap_unmapdev(va, K1_COMBO_PHY_SIZE);

	if ((val & R_TUNE_DONE) == 0)
		return (ENXIO);		/* combo PHY not calibrated yet */

	*rterm = val & (RTERM_VALUE_RX_M | RTERM_VALUE_TX_M);
	return (0);
}

/* Per-lane PCIe analog init using the calibrated termination values. */
static void
sppcie_phy_init_lanes(struct sppcie_phy_softc *sc, uint32_t rterm)
{
	uint32_t rx = rterm & RTERM_VALUE_RX_M;
	uint32_t tx = (rterm & RTERM_VALUE_TX_M) >> 4;
	uint32_t off, val;
	int i;

	for (i = 0; i < sc->nlanes; i++) {
		off = i * PHY_LANE_OFFSET;

		/* RX termination value + enable refclock RX termination. */
		val = RD4(sc, off + PCIE_RX_REG1);
		val &= ~AFE_RTERM_REG_M;
		val |= (rx << AFE_RTERM_REG_S) & AFE_RTERM_REG_M;
		val |= EN_RTERM;
		WR4(sc, off + PCIE_RX_REG1, val);

		val = RD4(sc, off + PCIE_RX_REG2);
		val &= ~RX_RTERM_SEL;	/* use RX_REG1 value */
		WR4(sc, off + PCIE_RX_REG2, val);

		/* TX driver termination value. */
		val = RD4(sc, off + PCIE_TX_REG1);
		val &= ~TX_RTERM_REG_M;
		val |= (tx << TX_RTERM_REG_S) & TX_RTERM_REG_M;
		val |= TX_RTERM_SEL;	/* use TX_REG1 value */
		WR4(sc, off + PCIE_TX_REG1, val);

		/* 24 MHz input clock; toggle RC calibration. */
		val = RD4(sc, off + PCIE_RC_CAL_REG2);
		val &= CLKSEL_M;
		val |= CLKSEL_24M;
		val &= ~RC_CAL_TOGGLE;
		WR4(sc, off + PCIE_RC_CAL_REG2, val);
		val |= RC_CAL_TOGGLE;
		WR4(sc, off + PCIE_RC_CAL_REG2, val);

		/* Force refclk driver mode. */
		val = RD4(sc, off + PCIE_LTSSM_DIS_ENTRY);
		val |= OVRD_REFCLK_MODE;
		val &= ~CFG_REFCLK_MODE_M;
		val |= RFCLK_MODE_DRIVER;
		WR4(sc, off + PCIE_LTSSM_DIS_ENTRY, val);
	}
}

/* PLL config + start; poll for PLL_READY. */
static int
sppcie_phy_pll_start(struct sppcie_phy_softc *sc)
{
	uint32_t off, val;
	int i, us;

	for (i = 0; i < sc->nlanes; i++) {
		off = i * PHY_LANE_OFFSET;
		val = RD4(sc, off + PCIE_PU_ADDR_CLK_CFG);
		val &= ~CFG_INTERNAL_TIMER_ADJ_M;
		val |= TIMER_ADJ_PCIE;
		WR4(sc, off + PCIE_PU_ADDR_CLK_CFG, val);
	}

	val = RD4(sc, PCIE_RC_DONE_STATUS);
	val |= CFG_FORCE_RCV_RETRY;
	WR4(sc, PCIE_RC_DONE_STATUS, val);

	val = RD4(sc, PCIE_PU_PLL_1);
	val &= ~SSC_DEP_SEL_M;			/* SSC_DEP_NONE */
	WR4(sc, PCIE_PU_PLL_1, val);

	val = RD4(sc, PCIE_PU_PLL_2);
	val |= GEN_REF100;			/* 100 MHz PLL output */
	WR4(sc, PCIE_PU_PLL_2, val);

	/* 24 MHz reference, no 100 MHz-SSC input. */
	val = RD4(sc, PCIE_PU_PLL_1);
	val &= ~REF_100_WSSC;
	val &= ~FREF_SEL_M;
	val |= FREF_24M;
	WR4(sc, PCIE_PU_PLL_1, val);

	/* Mark PLL config done on all lanes. */
	for (i = 0; i < sc->nlanes; i++) {
		off = i * PHY_LANE_OFFSET;
		val = RD4(sc, off + PCIE_PU_ADDR_CLK_CFG);
		val |= CFG_SW_PHY_INIT_DONE;
		WR4(sc, off + PCIE_PU_ADDR_CLK_CFG, val);
	}

	/* Lanes share a PLL; sampling lane 0 is sufficient. */
	for (us = 0; us < PLL_LOCK_TIMEOUT_US; us += POLL_DELAY_US) {
		if (RD4(sc, PCIE_PU_ADDR_CLK_CFG) & PLL_READY)
			return (0);
		DELAY(POLL_DELAY_US);
	}
	return (ETIMEDOUT);
}

static int
sppcie_phy_enable(struct phynode *phynode, bool enable)
{
	struct sppcie_phy_softc *sc;
	device_t dev;
	uint32_t rterm;
	int error;

	dev = phynode_get_device(phynode);
	sc = device_get_softc(dev);

	if (!enable)
		return (0);

	/* Consume the combo PHY's calibration (read-only; USB3 untouched). */
	error = sppcie_phy_get_rterm(dev, &rterm);
	if (error != 0) {
		device_printf(dev, "PCIe PHY termination not calibrated "
		    "(combo PHY not up); declining to enable\n");
		return (error);
	}
	sppcie_phy_init_lanes(sc, rterm);
	error = sppcie_phy_pll_start(sc);
	if (error != 0)
		device_printf(dev, "PCIe PHY PLL failed to lock\n");
	return (error);
}

static phynode_method_t sppcie_phy_methods[] = {
	PHYNODEMETHOD(phynode_enable,	sppcie_phy_enable),
	PHYNODEMETHOD_END
};
DEFINE_CLASS_1(sppcie_phynode, sppcie_phynode_class, sppcie_phy_methods, 0,
    phynode_class);

static int
sppcie_phy_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "SpacemiT K1 PCIe PHY");
	return (BUS_PROBE_DEFAULT);
}

static int
sppcie_phy_attach(device_t dev)
{
	struct sppcie_phy_softc *sc;
	struct phynode_init_def initdef;
	struct phynode *phynode;
	phandle_t node;
	pcell_t lanes;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	sc->mem_rid = 0;
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid,
	    RF_ACTIVE);
	if (sc->mem == NULL) {
		device_printf(dev, "cannot allocate memory resource\n");
		return (ENXIO);
	}

	sc->nlanes = 2;	/* port B/C dedicated PHYs are 2-lane; vendor/mainline init both. DT num-lanes overrides. */
	if (OF_getencprop(node, "num-lanes", &lanes, sizeof(lanes)) > 0 &&
	    lanes >= 1 && lanes <= 2)
		sc->nlanes = lanes;

	/* Optional reference clock. */
	if (clk_get_by_ofw_name(dev, 0, "refclk", &sc->refclk) == 0)
		(void)clk_enable(sc->refclk);
	else
		sc->refclk = NULL;

	/*
	 * Deassert the PHY's own "global"/"phy" reset and leave it
	 * deasserted for the PHY's lifetime.  Without this the PHY block
	 * stays in reset and its PLL never locks (mainline deasserts the
	 * per-PHY "phy" reset at probe for exactly this reason).
	 */
	if (hwreset_get_by_ofw_name(dev, node, "phy", &sc->rst_phy) == 0) {
		if (hwreset_deassert(sc->rst_phy) != 0)
			device_printf(dev, "cannot deassert 'phy' reset\n");
	} else {
		sc->rst_phy = NULL;
		device_printf(dev, "warning: no 'phy' reset; PLL may not lock\n");
	}

	memset(&initdef, 0, sizeof(initdef));
	initdef.id = 0;
	initdef.ofw_node = node;
	phynode = phynode_create(dev, &sppcie_phynode_class, &initdef);
	if (phynode == NULL) {
		device_printf(dev, "failed to create PCIe PHY node\n");
		goto fail;
	}
	if (phynode_register(phynode) == NULL) {
		device_printf(dev, "failed to register PCIe PHY node\n");
		goto fail;
	}

	return (0);

fail:
	if (sc->refclk != NULL)
		clk_release(sc->refclk);
	bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem);
	return (ENXIO);
}

static device_method_t sppcie_phy_dev_methods[] = {
	DEVMETHOD(device_probe,		sppcie_phy_probe),
	DEVMETHOD(device_attach,	sppcie_phy_attach),

	DEVMETHOD_END
};

static driver_t sppcie_phy_driver = {
	"spacemit_pcie_phy",
	sppcie_phy_dev_methods,
	sizeof(struct sppcie_phy_softc)
};

EARLY_DRIVER_MODULE(spacemit_pcie_phy, simplebus, sppcie_phy_driver, NULL, NULL,
    BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(spacemit_pcie_phy, 1);
