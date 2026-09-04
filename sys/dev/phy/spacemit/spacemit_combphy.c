/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Daniel Shue <dgshue@gmail.com>
 *
 * USB3 combo PHY driver for the SpacemiT K1 (Ky X1) RISC-V SoC
 * (compatible: spacemit,k1-combo-phy).  Only USB3 mode is supported here
 * (the PHY is also usable for PCIe/SATA).  Init sequence and register
 * values from the vendor Linux driver
 * (drivers/phy/spacemit/phy-spacemit-k1x-combphy.c); clocks/resets are
 * left as the boot firmware programmed them.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/phy/phy.h>

#define	PHY_TYPE_USB3			4

#define	COMBPHY_USB_REG1		0x68
#define	COMBPHY_USB_REG1_VAL		0x0
#define	COMBPHY_USB_REG2		(0x12 << 2)
#define	COMBPHY_USB_REG2_VAL		0x603a2276
#define	COMBPHY_USB_REG3		(0x02 << 2)
#define	COMBPHY_USB_REG3_VAL		0x97c
#define	COMBPHY_USB_REG4		(0x06 << 2)
#define	COMBPHY_USB_REG4_VAL		0x0
#define	COMBPHY_USB_PLL_REG		0x8
#define	COMBPHY_USB_PLL_MASK		0x1

/*
 * This PHY is shared: it drives either PCIe port A or the USB3 controller,
 * never both.  Which consumer the lane is muxed to is selected outside the
 * PHY's own register window, in the APMU block referenced by the node's
 * "spacemit,apmu" phandle.  The reset value is 0 = PCIe, so a board using it
 * for USB3 must set COMBO_PHY_SEL explicitly -- Linux does this in
 * k1_combo_phy_sel() (drivers/phy/spacemit/phy-k1-pcie.c).
 */
#define	APMU_USB_PHY_CTRL0		0x110
#define	 APMU_COMBO_PHY_SEL		(1u << 3)	/* 0: PCIe, 1: USB3 */
#define	APMU_PCIE_CLK_RES_CTRL_0	0x3cc
#define	 APMU_PCIE_APP_HOLD_PHY_RST	(1u << 30)

struct spcomb_softc {
	device_t	dev;
	struct resource	*mem_res;
	bool		apmu_valid;
	bus_space_tag_t	apmu_bst;
	bus_space_handle_t apmu_bsh;
};

static struct ofw_compat_data compat_data[] = {
	{ "spacemit,k1-combo-phy",	1 },
	{ NULL,				0 }
};

#define	RD4(sc, r)	bus_read_4((sc)->mem_res, (r))
#define	WR4(sc, r, v)	bus_write_4((sc)->mem_res, (r), (v))

/*
 * Map the APMU window named by the node's "spacemit,apmu" phandle.  Failure is
 * not fatal: on a board that uses this PHY for PCIe the mux is already in its
 * reset state and nothing needs writing.
 */
static void
spcomb_map_apmu(struct spcomb_softc *sc, phandle_t node)
{
	phandle_t apmu;
	pcell_t xref;

	sc->apmu_valid = false;
	if (OF_getencprop(node, "spacemit,apmu", &xref, sizeof(xref)) <= 0)
		return;
	apmu = OF_node_from_xref(xref);
	if (apmu == 0)
		return;
	if (OF_decode_addr(apmu, 0, &sc->apmu_bst, &sc->apmu_bsh, NULL) != 0) {
		device_printf(sc->dev, "cannot map APMU for PHY mode select\n");
		return;
	}
	sc->apmu_valid = true;
}

static void
spcomb_select_usb3(struct spcomb_softc *sc)
{
	uint32_t val;

	if (!sc->apmu_valid) {
		device_printf(sc->dev,
		    "no APMU mapping: cannot select USB3 mode\n");
		return;
	}

	/*
	 * Point the shared lane at the USB3 controller.  Without this the DWC3
	 * core never receives a PIPE clock: its own registers keep answering
	 * (they sit on the AXI clock) but USBCMD.HCRST never self-clears and
	 * the XHCI layer fails with "Controller reset timeout".
	 */
	val = bus_space_read_4(sc->apmu_bst, sc->apmu_bsh, APMU_USB_PHY_CTRL0);
	if ((val & APMU_COMBO_PHY_SEL) == 0)
		bus_space_write_4(sc->apmu_bst, sc->apmu_bsh,
		    APMU_USB_PHY_CTRL0, val | APMU_COMBO_PHY_SEL);

	/* Linux clears APP_HOLD_PHY_RST for the combo PHY and leaves it clear. */
	val = bus_space_read_4(sc->apmu_bst, sc->apmu_bsh,
	    APMU_PCIE_CLK_RES_CTRL_0);
	if ((val & APMU_PCIE_APP_HOLD_PHY_RST) != 0)
		bus_space_write_4(sc->apmu_bst, sc->apmu_bsh,
		    APMU_PCIE_CLK_RES_CTRL_0,
		    val & ~APMU_PCIE_APP_HOLD_PHY_RST);
}

static int
spcomb_enable(struct phynode *phynode, bool enable)
{
	struct spcomb_softc *sc;
	device_t dev;
	int i;

	dev = phynode_get_device(phynode);
	sc = device_get_softc(dev);

	/* Only USB3 mode is handled. */
	if (!enable || phynode_get_id(phynode) != PHY_TYPE_USB3)
		return (0);

	spcomb_select_usb3(sc);

	WR4(sc, COMBPHY_USB_REG1, COMBPHY_USB_REG1_VAL);
	WR4(sc, COMBPHY_USB_REG2, COMBPHY_USB_REG2_VAL);
	WR4(sc, COMBPHY_USB_REG3, COMBPHY_USB_REG3_VAL);
	WR4(sc, COMBPHY_USB_REG4, COMBPHY_USB_REG4_VAL);

	/* Wait for the PLL to lock. */
	for (i = 0; i < 1000; i++) {
		if (RD4(sc, COMBPHY_USB_PLL_REG) & COMBPHY_USB_PLL_MASK)
			break;
		DELAY(100);
	}
	if (i == 1000)
		device_printf(dev, "warning: USB3 PHY PLL not locked "
		    "(combo PHY mode select wrong?)\n");

	return (0);
}

static phynode_method_t spcomb_phynode_methods[] = {
	PHYNODEMETHOD(phynode_enable,	spcomb_enable),
	PHYNODEMETHOD_END
};
DEFINE_CLASS_1(spcomb_phynode, spcomb_phynode_class, spcomb_phynode_methods,
    0, phynode_class);

static int
spcomb_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "SpacemiT K1 combo PHY (USB3)");
	return (BUS_PROBE_DEFAULT);
}

static int
spcomb_attach(device_t dev)
{
	struct spcomb_softc *sc;
	struct phynode_init_def phy_init;
	struct phynode *phynode;
	int rid;

	sc = device_get_softc(dev);
	sc->dev = dev;

	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "cannot allocate registers\n");
		return (ENXIO);
	}

	/*
	 * Enable the combo-PHY's clocks and release its resets so the USB3
	 * PLL can lock.  These live in the shared PCIe0 control register.
	 */
	{
		clk_t clk;
		hwreset_t rst;
		int i;

		for (i = 0; clk_get_by_ofw_index(dev, 0, i, &clk) == 0; i++)
			(void)clk_enable(clk);
		for (i = 0; hwreset_get_by_ofw_idx(dev, 0, i, &rst) == 0; i++)
			hwreset_deassert(rst);
	}

	spcomb_map_apmu(sc, ofw_bus_get_node(dev));

	bzero(&phy_init, sizeof(phy_init));
	phy_init.id = PHY_TYPE_USB3;
	phy_init.ofw_node = ofw_bus_get_node(dev);
	phynode = phynode_create(dev, &spcomb_phynode_class, &phy_init);
	if (phynode == NULL) {
		device_printf(dev, "failed to create combo PHY\n");
		return (ENXIO);
	}
	if (phynode_register(phynode) == NULL) {
		device_printf(dev, "failed to register combo PHY\n");
		return (ENXIO);
	}
	return (0);
}

static device_method_t spcomb_methods[] = {
	DEVMETHOD(device_probe,		spcomb_probe),
	DEVMETHOD(device_attach,	spcomb_attach),
	DEVMETHOD_END
};

static driver_t spcomb_driver = {
	"spacemit_combphy",
	spcomb_methods,
	sizeof(struct spcomb_softc),
};

EARLY_DRIVER_MODULE(spacemit_combphy, simplebus, spcomb_driver, 0, 0,
    BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_EARLY);
