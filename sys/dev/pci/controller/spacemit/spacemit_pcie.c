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
 * DesignWare PCIe host (root complex) front-end for the SpacemiT K1 (Ky X1)
 * (compatible: "spacemit,k1-pcie").  Subclasses FreeBSD's generic pci_dw
 * DesignWare core (dev/pci/pci_dw.c), providing the K1-specific glue:
 *   - app clocks (dbi/mstr/slv) + resets via the CCU,
 *   - the per-controller APMU control window (reached via the DT
 *     "spacemit,apmu = <&syscon_apmu offset>" property) for soft reset,
 *     PERST#, RC-mode select, aux-power detect and LTSSM enable,
 *   - the dedicated PCIe PHY (our spacemit_pcie_phy driver), and
 *   - the link-status region for link-up detection.
 *
 * This drives ONLY pcie1/pcie2 (the board's dedicated PCIe slots).  It does
 * NOT touch the USB3 combo PHY.  Sequence reimplemented from the mainline
 * Linux driver drivers/pci/controller/dwc/pcie-spacemit-k1.c (GPL-2.0), NOT
 * copied; front-end modeled on dev/pci/pci_dw_mv.c.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/proc.h>
#include <sys/mutex.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <machine/bus.h>
#include <machine/intr.h>
#include <machine/resource.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_pci.h>
#include <dev/ofw/ofwpci.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/phy/phy.h>
#include <dev/regulator/regulator.h>
#include <dev/syscon/syscon.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcib_private.h>
#include <dev/pci/pci_dw.h>

#include "pcib_if.h"
#include "syscon_if.h"
#include "pci_dw_if.h"
#include "msi_if.h"
#include "pic_if.h"

/* APMU per-controller control registers (relative to the DT-supplied offset). */
#define	PCIE_CLK_RESET_CONTROL	0x0000
#define	 LTSSM_EN		(1u << 6)
#define	 PCIE_AUX_PWR_DET	(1u << 9)
#define	 PCIE_RC_PERST		(1u << 12)	/* 1 = assert PERST# */
#define	 APP_HOLD_PHY_RST	(1u << 30)
#define	 DEVICE_TYPE_RC		(1u << 31)
#define	PCIE_CONTROL_LOGIC	0x0004
#define	 PCIE_SOFT_RESET	(1u << 0)

/* Link-status region ("link" reg, offset 0x04). */
#define	K1_PHY_AHB_IRQ_EN	0x0000
#define	 PCIE_INTERRUPT_EN	(1u << 0)
#define	K1_PHY_AHB_LINK_STS	0x0004
#define	 SMLH_LINK_UP		(1u << 1)
#define	 RDLH_LINK_UP		(1u << 12)
#define	INTR_ENABLE		0x0014
#define	 MSI_CTRL_INT		(1u << 11)

#define	PCIE_T_PVPERL_MS	100	/* PERL# deassert delay (CEM spec) */

/*
 * Bounded link-training wait.  With no card in the slot the link never comes
 * up; we must NOT block attach forever (that hangs the whole boot before
 * mountroot).  100 ms is generous for a present card to reach L0.
 */
#define	LINK_TRAIN_TIMEOUT_US	1000000
#define	LINK_POLL_DELAY_US	200

#define	PCI_VENDOR_ID_SPACEMIT		0x201f
#define	PCI_DEVICE_ID_SPACEMIT_K1	0x0001

/* ---- DesignWare integrated MSI controller (K1 has no GIC ITS; MSIs raise the
 * single "msi" PLIC IRQ and we demux in software).  Modeled on the FreeBSD
 * plumbing in riscv/starfive/jh7110_pcie.c, using the standard DWC MSI DBI
 * registers.  rge(4) and most modern NICs need MSI (K1 PCIe has no INTx). */
#define	PCIE_MSI_ADDR_LO	0x820
#define	PCIE_MSI_ADDR_HI	0x824
#define	PCIE_MSI_INTR0_ENABLE	0x828
#define	PCIE_MSI_INTR0_MASK	0x82c
#define	PCIE_MSI_INTR0_STATUS	0x830
#define	SPM_MSI_COUNT		32
#define	SPM_MSI_USED		0x1

struct spacemit_pcie_irqsrc {
	struct intr_irqsrc	isrc;
	u_int			irq;
	u_int			is_used;
};

struct spacemit_pcie_softc {
	struct pci_dw_softc	dw_sc;		/* Must be first. */
	device_t		dev;
	phandle_t		node;
	struct resource		*link_res;
	int			link_rid;
	struct syscon		*apmu;
	bus_size_t		apmu_off;
	clk_t			clk_dbi;
	clk_t			clk_mstr;
	clk_t			clk_slv;
	hwreset_t		rst_dbi;
	hwreset_t		rst_mstr;
	hwreset_t		rst_slv;
	phy_t			phy;
	bool			phy_up;		/* PHY PLL locked => DBI reachable */
	regulator_t		vpcie3v3;	/* endpoint 3.3V slot power */
	/* DWC integrated MSI */
	struct resource		*irq_res;
	void			*irq_cookie;
	struct mtx		msi_mtx;
	struct spacemit_pcie_irqsrc *isrcs;
	void			*msi_page;
	bus_addr_t		msi_target;
	bus_dma_tag_t		dma_tag;
};

static struct ofw_compat_data compat_data[] = {
	{ "spacemit,k1-pcie",	1 },
	{ NULL,			0 }
};

#define	LINK_RD4(sc, r)		bus_read_4((sc)->link_res, (r))
#define	LINK_WR4(sc, r, v)	bus_write_4((sc)->link_res, (r), (v))
#define	APMU_MODIFY(sc, r, clr, set) \
	SYSCON_MODIFY_4((sc)->apmu, (sc)->apmu_off + (r), (clr), (set))

static void
spacemit_pcie_toggle_soft_reset(struct spacemit_pcie_softc *sc)
{

	APMU_MODIFY(sc, PCIE_CONTROL_LOGIC, 0, PCIE_SOFT_RESET);
	(void)SYSCON_READ_4(sc->apmu, sc->apmu_off + PCIE_CONTROL_LOGIC);
	DELAY(2000);
	APMU_MODIFY(sc, PCIE_CONTROL_LOGIC, PCIE_SOFT_RESET, 0);
}

static int
spacemit_pcie_enable_resources(struct spacemit_pcie_softc *sc)
{

	if (clk_enable(sc->clk_dbi) != 0 || clk_enable(sc->clk_mstr) != 0 ||
	    clk_enable(sc->clk_slv) != 0)
		return (ENXIO);
	(void)hwreset_deassert(sc->rst_dbi);
	(void)hwreset_deassert(sc->rst_mstr);
	(void)hwreset_deassert(sc->rst_slv);
	return (0);
}

/* pci_dw_if: report link-up status. */
static int
spacemit_pcie_get_link(device_t dev, bool *status)
{
	struct spacemit_pcie_softc *sc;
	uint32_t val;

	sc = device_get_softc(dev);
	val = LINK_RD4(sc, K1_PHY_AHB_LINK_STS);
	*status = (val & RDLH_LINK_UP) && (val & SMLH_LINK_UP);
	return (0);
}

/*
 * Root-complex bring-up (mirrors mainline k1_pcie_init).  Returns 0 on success
 * (clocks/resets up and the controller reachable), or an error if the fabric
 * could not be brought up at all.  A DOWN link (empty slot) is NOT an error --
 * *linkup is set to reflect it, and the caller decides whether to enumerate.
 * Every poll here is bounded so an empty slot can never hang attach/boot.
 */
static int
spacemit_pcie_init_rc(struct spacemit_pcie_softc *sc, bool *linkup)
{
	bool status;
	int error, us;

	*linkup = false;
	sc->phy_up = false;

	/*
	 * Power the endpoint slot BEFORE bring-up so the CEM T_PVPERL
	 * delay below (PERST# held asserted) doubles as the power-settle
	 * time.  Non-fatal if absent.
	 */
	if (sc->vpcie3v3 != NULL) {
		if (regulator_enable(sc->vpcie3v3) != 0)
			device_printf(sc->dev, "cannot enable vpcie3v3 supply\n");
		else
			DELAY(100000);	/* 100ms: let the cold endpoint power up */
	}

	spacemit_pcie_toggle_soft_reset(sc);

	error = spacemit_pcie_enable_resources(sc);
	if (error != 0)
		return (error);

	/*
	 * Program the vendor/device ID over DBI HERE, BEFORE PERST# is
	 * asserted -- exactly as mainline does.  The K1 PCIe wrapper gates the
	 * DBI clock domain while PERST# is asserted, so any DBI access with
	 * PERST# low (once the pad is muxed so PERST# actually reaches the
	 * controller) stalls the AXI bus and hard-hangs the hart.  The core is
	 * already out of reset here: the PHY driver deasserted the PHY global
	 * reset at attach and enable_resources() released dbi/mstr/slv.
	 */
	pci_dw_dbi_wr2(sc->dev, PCIR_VENDOR, PCI_VENDOR_ID_SPACEMIT);
	pci_dw_dbi_wr2(sc->dev, PCIR_DEVICE, PCI_DEVICE_ID_SPACEMIT_K1);

	/* Assert PERST#, wait the CEM-spec power-stable delay. */
	APMU_MODIFY(sc, PCIE_CLK_RESET_CONTROL, 0, PCIE_RC_PERST);
	(void)SYSCON_READ_4(sc->apmu, sc->apmu_off + PCIE_CLK_RESET_CONTROL);
	DELAY(PCIE_T_PVPERL_MS * 1000);

	/* Root-complex mode + aux power present. */
	APMU_MODIFY(sc, PCIE_CLK_RESET_CONTROL, 0,
	    DEVICE_TYPE_RC | PCIE_AUX_PWR_DET);

	/*
	 * Hold the PHY application (PIPE) interface in reset THROUGH PHY init,
	 * exactly as mainline does (it sets APP_HOLD_PHY_RST in probe and only
	 * clears it in start_link, together with enabling the LTSSM).  Bringing
	 * the PIPE interface out of reset before the PHY analog is ready leaves
	 * the LTSSM stuck in Detect (no receiver detected), so keep it ASSERTED
	 * here; it is cleared below at LTSSM enable.
	 */
	APMU_MODIFY(sc, PCIE_CLK_RESET_CONTROL, 0, APP_HOLD_PHY_RST);

	/*
	 * Bring up the PHY (dedicated PCIe PHY; USB3 combo PHY untouched).
	 * The PHY's PLL-lock poll is internally bounded.  Treat a PHY that
	 * fails to lock as "no link" (typical for an EMPTY SLOT) -- NON-FATAL:
	 * leave the link down and let the caller skip enumeration cleanly
	 * rather than proceeding into DBI/link-dependent init that can hang.
	 */
	if (sc->phy != NULL) {
		error = phy_enable(sc->phy);
		if (error != 0) {
			device_printf(sc->dev,
			    "PCIe PHY did not come up (no card?); link down\n");
			return (0);	/* non-fatal: *linkup stays false */
		}
	}

	sc->phy_up = true;	/* PHY PLL locked (or no PHY node) */
	/* Deassert PERST#. */
	APMU_MODIFY(sc, PCIE_CLK_RESET_CONTROL, PCIE_RC_PERST, 0);

	/* Stop holding the PHY in reset and enable link training (LTSSM). */
	APMU_MODIFY(sc, PCIE_CLK_RESET_CONTROL, APP_HOLD_PHY_RST, LTSSM_EN);

	/* Enable link/MSI interrupts in the link region. */
	LINK_WR4(sc, INTR_ENABLE, MSI_CTRL_INT);
	LINK_WR4(sc, K1_PHY_AHB_IRQ_EN,
	    LINK_RD4(sc, K1_PHY_AHB_IRQ_EN) | PCIE_INTERRUPT_EN);

	/*
	 * Wait (bounded) for link training to reach L0.  With no card the link
	 * never comes up; we simply time out and report link-down -- we do NOT
	 * block boot.  A present card normally links within a few ms.
	 */
	for (us = 0; us < LINK_TRAIN_TIMEOUT_US; us += LINK_POLL_DELAY_US) {
		if (spacemit_pcie_get_link(sc->dev, &status) == 0 && status) {
			*linkup = true;
			break;
		}
		DELAY(LINK_POLL_DELAY_US);
	}
	if (!*linkup)
		device_printf(sc->dev, "PCIe link down (no device in slot)\n");

	return (0);
}

static int
spacemit_pcie_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "SpacemiT K1 PCIe");
	return (BUS_PROBE_DEFAULT);
}

static int spacemit_pcie_msi_attach(struct spacemit_pcie_softc *sc);

/*
 * Register the parent bus's dma-ranges as busdma translation windows, so
 * memory above the 2 GiB identity window that a range covers is reached
 * directly instead of through bounce pages.  Returns the windows added.
 */
static int
spacemit_pcie_dma_windows(struct spacemit_pcie_softc *sc)
{
	phandle_t bus, node;
	pcell_t *cells;
	ssize_t len;
	int acells, pacells, scells, entry, i, n, error;
	uint64_t addr[3];

	node = ofw_bus_get_node(sc->dev);
	bus = OF_parent(node);
	if (bus == 0 || OF_parent(bus) == 0)
		return (0);
	if (OF_getencprop(bus, "#address-cells", &acells,
	    sizeof(acells)) <= 0)
		acells = 2;
	if (OF_getencprop(bus, "#size-cells", &scells, sizeof(scells)) <= 0)
		scells = 1;
	if (OF_getencprop(OF_parent(bus), "#address-cells", &pacells,
	    sizeof(pacells)) <= 0)
		pacells = 2;
	if (acells < 1 || acells > 2 || pacells < 1 || pacells > 2 ||
	    scells < 1 || scells > 2)
		return (0);
	len = OF_getencprop_alloc_multi(bus, "dma-ranges", sizeof(pcell_t),
	    (void **)&cells);
	if (len <= 0)
		return (0);
	entry = acells + pacells + scells;
	n = 0;
	for (i = 0; i + entry <= len; i += entry) {
		addr[0] = cells[i];
		if (acells == 2)
			addr[0] = (addr[0] << 32) | cells[i + 1];
		addr[1] = cells[i + acells];
		if (pacells == 2)
			addr[1] = (addr[1] << 32) | cells[i + acells + 1];
		addr[2] = cells[i + acells + pacells];
		if (scells == 2)
			addr[2] = (addr[2] << 32) |
			    cells[i + acells + pacells + 1];
		/* bus address addr[0] reaches CPU physical addr[1]. */
		error = bus_dma_tag_add_window(sc->dma_tag, addr[1], addr[2],
		    addr[0]);
		if (error != 0) {
			device_printf(sc->dev,
			    "cannot add DMA window %#jx-%#jx (%d)\n",
			    (uintmax_t)addr[1], (uintmax_t)(addr[1] + addr[2] - 1),
			    error);
			continue;
		}
		if (bootverbose || addr[0] != addr[1])
			device_printf(sc->dev,
			    "DMA window: CPU %#jx-%#jx at bus %#jx\n",
			    (uintmax_t)addr[1],
			    (uintmax_t)(addr[1] + addr[2] - 1), (uintmax_t)addr[0]);
		n++;
	}
	OF_prop_free(cells);
	return (n);
}

static int
spacemit_pcie_attach(device_t dev)
{
	struct resource_map_request req;
	struct resource_map map;
	struct spacemit_pcie_softc *sc;
	pcell_t apmu_prop[2];
	phandle_t node;
	bool linkup;
	int rid, error, dma_windows;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);
	sc->node = node;

	/* DBI region (reg index 0), mapped non-prefetchable device memory. */
	rid = 0;
	sc->dw_sc.dbi_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE | RF_UNMAPPED);
	if (sc->dw_sc.dbi_res == NULL) {
		device_printf(dev, "cannot allocate DBI memory\n");
		return (ENXIO);
	}
	resource_init_map_request(&req);
	req.memattr = VM_MEMATTR_DEVICE;
	error = bus_map_resource(dev, SYS_RES_MEMORY, sc->dw_sc.dbi_res, &req,
	    &map);
	if (error != 0) {
		device_printf(dev, "cannot map DBI memory\n");
		return (error);
	}
	rman_set_mapping(sc->dw_sc.dbi_res, &map);

	/* Link-status region (reg index 3, reg-name "link"). */
	sc->link_rid = 3;
	sc->link_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->link_rid, RF_ACTIVE);
	if (sc->link_res == NULL) {
		device_printf(dev, "cannot allocate link region\n");
		return (ENXIO);
	}

	/* APMU control window via the "spacemit,apmu" phandle+offset. */
	if (OF_getencprop(node, "spacemit,apmu", apmu_prop,
	    sizeof(apmu_prop)) != sizeof(apmu_prop)) {
		device_printf(dev, "missing 'spacemit,apmu' property\n");
		return (ENXIO);
	}
	if (syscon_get_by_ofw_node(dev, OF_node_from_xref(apmu_prop[0]),
	    &sc->apmu) != 0) {
		device_printf(dev, "cannot get APMU syscon\n");
		return (ENXIO);
	}
	sc->apmu_off = apmu_prop[1];

	/* App clocks + resets. */
	if (clk_get_by_ofw_name(dev, 0, "dbi", &sc->clk_dbi) != 0 ||
	    clk_get_by_ofw_name(dev, 0, "mstr", &sc->clk_mstr) != 0 ||
	    clk_get_by_ofw_name(dev, 0, "slv", &sc->clk_slv) != 0) {
		device_printf(dev, "cannot get app clocks\n");
		return (ENXIO);
	}
	if (hwreset_get_by_ofw_name(dev, 0, "dbi", &sc->rst_dbi) != 0 ||
	    hwreset_get_by_ofw_name(dev, 0, "mstr", &sc->rst_mstr) != 0 ||
	    hwreset_get_by_ofw_name(dev, 0, "slv", &sc->rst_slv) != 0) {
		device_printf(dev, "cannot get app resets\n");
		return (ENXIO);
	}

	/*
	 * The dedicated PCIe PHY is referenced (unnamed, index 0) from the
	 * root-port child node (`pcie@0`), matching mainline's parse_port.
	 */
	sc->phy = NULL;
	sc->vpcie3v3 = NULL;
	{
		phandle_t port;

		for (port = OF_child(node); port != 0; port = OF_peer(port)) {
			if (!ofw_bus_node_status_okay(port))
				continue;
			if (phy_get_by_ofw_idx(dev, port, 0, &sc->phy) == 0) {
				/*
				 * The endpoint slot power ("vpcie3v3-supply")
				 * lives on the same root-port child node.  On the
				 * R2S it is a GPIO-gated fixed regulator that
				 * powers the on-board RTL8125; without enabling it
				 * the endpoint has no power and the link never
				 * trains (PLL locks but "no device in slot").
				 */
				(void)regulator_get_by_ofw_property(dev, port,
				    "vpcie3v3-supply", &sc->vpcie3v3);
				break;
			}
		}
	}

	/*
	 * K1-specific root-complex bring-up before the generic DWC init.
	 * A down link (empty slot) is non-fatal and reported via linkup; the
	 * bring-up itself only fails if the fabric clocks/resets can't come up.
	 */
	error = spacemit_pcie_init_rc(sc, &linkup);
	if (error != 0) {
		device_printf(dev, "root-complex bring-up failed\n");
		return (error);
	}

	/*
	 * Bring up the generic DesignWare core (DBI is clock-gated, not
	 * link-gated, so this is safe with the link down).  Config-space
	 * enumeration of downstream buses is gated on link-up by
	 * spacemit_pcie_get_link(), so an empty slot simply enumerates nothing
	 * rather than hanging.
	 */
	if (!sc->phy_up) {
		device_printf(dev,
		    "PCIe PHY down; DBI unreachable, skipping DWC init "
		    "(controller idle)\n");
		return (0);	/* attach as an idle stub; board still boots */
	}

	error = pci_dw_init(dev);
	if (error != 0)
		return (error);

	/*
	 * K1 PCIe has a 2-GiB identity DMA window; the parent bus's
	 * dma-ranges maps an upper window to different CPU addresses.  Passing
	 * an untranslated high physical address to an endpoint silently
	 * targets the wrong memory, including a high bounce page.  Constrain
	 * all descendant tags (payloads, queues and PRP lists) to the identity
	 * window, then register the dma-ranges as translation windows so the
	 * memory they cover is reached directly; the rest still bounces.
	 * hw.spacemit_pcie.dma_windows=0 keeps every high address bouncing.
	 */
	error = bus_dma_tag_create(sc->dw_sc.dmat, 1, 0,
	    0x7fffffffULL, BUS_SPACE_MAXADDR, NULL, NULL,
	    BUS_SPACE_MAXSIZE, BUS_SPACE_UNRESTRICTED, BUS_SPACE_MAXSIZE,
	    0, NULL, NULL, &sc->dma_tag);
	if (error != 0) {
		device_printf(dev, "cannot create reachable DMA tag (%d)\n",
		    error);
		return (error);
	}
	dma_windows = 1;
	TUNABLE_INT_FETCH("hw.spacemit_pcie.dma_windows", &dma_windows);
	if (dma_windows == 0 || spacemit_pcie_dma_windows(sc) == 0)
		device_printf(dev,
		    "DMA restricted to identity window below 2 GiB\n");

	/* Stand up the DWC integrated MSI controller before children attach. */
	error = spacemit_pcie_msi_attach(sc);
	if (error != 0)
		device_printf(dev, "MSI setup failed (%d); "
		    "MSI-only devices will not attach\n", error);

	bus_attach_children(dev);
	return (0);
}

static int
spacemit_pcie_msi_intr(void *arg)
{
	struct spacemit_pcie_softc *sc = arg;
	struct spacemit_pcie_irqsrc *irq;
	struct trapframe *tf = curthread->td_intr_frame;
	uint32_t status;
	int i;

	status = pci_dw_dbi_rd4(sc->dev, PCIE_MSI_INTR0_STATUS);
	if (status == 0)
		return (FILTER_STRAY);
	/* Ack first, then dispatch each pending vector. */
	pci_dw_dbi_wr4(sc->dev, PCIE_MSI_INTR0_STATUS, status);
	for (i = 0; status != 0; i++) {
		if ((status & (1U << i)) != 0) {
			irq = &sc->isrcs[i];
			if (intr_isrc_dispatch(&irq->isrc, tf) != 0)
				device_printf(sc->dev,
				    "spurious MSI vector %d\n", i);
			status &= ~(1U << i);
		}
	}
	return (FILTER_HANDLED);
}

/* --- msi_if: allocate/release/map MSI vectors from our 32-vector pool --- */
static int
spacemit_pcie_msi_alloc_msi(device_t dev, device_t child, int count,
    int maxcount, device_t *pic, struct intr_irqsrc **srcs)
{
	struct spacemit_pcie_softc *sc = device_get_softc(dev);
	int i, beg;

	mtx_lock(&sc->msi_mtx);
	for (beg = 0; beg + count <= SPM_MSI_COUNT; ) {
		for (i = beg; i < beg + count; i++)
			if (sc->isrcs[i].is_used == SPM_MSI_USED)
				break;
		if (i == beg + count)
			goto found;
		beg = i + 1;
	}
	mtx_unlock(&sc->msi_mtx);
	device_printf(dev, "failed to allocate %d MSIs\n", count);
	return (ENXIO);
found:
	for (i = 0; i < count; i++) {
		sc->isrcs[i + beg].is_used = SPM_MSI_USED;
		srcs[i] = &sc->isrcs[i + beg].isrc;
	}
	mtx_unlock(&sc->msi_mtx);
	*pic = device_get_parent(dev);
	return (0);
}

static int
spacemit_pcie_msi_release_msi(device_t dev, device_t child, int count,
    struct intr_irqsrc **isrc)
{
	struct spacemit_pcie_softc *sc = device_get_softc(dev);
	struct spacemit_pcie_irqsrc *irq;
	int i;

	mtx_lock(&sc->msi_mtx);
	for (i = 0; i < count; i++) {
		irq = (struct spacemit_pcie_irqsrc *)isrc[i];
		irq->is_used = 0;
	}
	mtx_unlock(&sc->msi_mtx);
	return (0);
}

static int
spacemit_pcie_msi_map_msi(device_t dev, device_t child,
    struct intr_irqsrc *isrc, uint64_t *addr, uint32_t *data)
{
	struct spacemit_pcie_softc *sc = device_get_softc(dev);
	struct spacemit_pcie_irqsrc *spirq =
	    (struct spacemit_pcie_irqsrc *)isrc;

	*addr = sc->msi_target;
	*data = spirq->irq;
	return (0);
}

static int
spacemit_pcie_msi_alloc_msix(device_t dev, device_t child, device_t *pic,
    struct intr_irqsrc **isrcp)
{
	return (spacemit_pcie_msi_alloc_msi(dev, child, 1, 1, pic, isrcp));
}

static int
spacemit_pcie_msi_release_msix(device_t dev, device_t child,
    struct intr_irqsrc *isrc)
{
	return (spacemit_pcie_msi_release_msi(dev, child, 1, &isrc));
}

/* --- pcib_if: forward child MSI requests to ourselves as the MSI parent --- */
static int
spacemit_pcie_alloc_msi(device_t pci, device_t child, int count, int maxcount,
    int *irqs)
{
	phandle_t mp = OF_xref_from_node(ofw_bus_get_node(pci));

	return (intr_alloc_msi(pci, child, mp, count, maxcount, irqs));
}

static int
spacemit_pcie_release_msi(device_t pci, device_t child, int count, int *irqs)
{
	phandle_t mp = OF_xref_from_node(ofw_bus_get_node(pci));

	return (intr_release_msi(pci, child, mp, count, irqs));
}

static int
spacemit_pcie_map_msi(device_t pci, device_t child, int irq, uint64_t *addr,
    uint32_t *data)
{
	phandle_t mp = OF_xref_from_node(ofw_bus_get_node(pci));

	return (intr_map_msi(pci, child, mp, irq, addr, data));
}

static int
spacemit_pcie_alloc_msix(device_t pci, device_t child, int *irq)
{
	phandle_t mp = OF_xref_from_node(ofw_bus_get_node(pci));

	return (intr_alloc_msix(pci, child, mp, irq));
}

static int
spacemit_pcie_release_msix(device_t pci, device_t child, int irq)
{
	phandle_t mp = OF_xref_from_node(ofw_bus_get_node(pci));

	return (intr_release_msix(pci, child, mp, irq));
}

/* --- pic_if: per-vector mask/unmask via the DWC MSI mask register --- */
static void
spacemit_pcie_msi_mask(device_t dev, struct intr_irqsrc *isrc, bool mask)
{
	struct spacemit_pcie_softc *sc = device_get_softc(dev);
	struct spacemit_pcie_irqsrc *spirq =
	    (struct spacemit_pcie_irqsrc *)isrc;
	uint32_t reg;

	mtx_lock(&sc->msi_mtx);
	reg = pci_dw_dbi_rd4(dev, PCIE_MSI_INTR0_MASK);
	if (mask)
		reg |= (1U << spirq->irq);	/* DWC: 1 = masked */
	else
		reg &= ~(1U << spirq->irq);
	pci_dw_dbi_wr4(dev, PCIE_MSI_INTR0_MASK, reg);
	mtx_unlock(&sc->msi_mtx);
}

static void
spacemit_pcie_msi_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	spacemit_pcie_msi_mask(dev, isrc, false);
}

static void
spacemit_pcie_msi_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	spacemit_pcie_msi_mask(dev, isrc, true);
}

static void
spacemit_pcie_msi_post_filter(device_t dev, struct intr_irqsrc *isrc)
{
}

static void
spacemit_pcie_msi_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{
}

static void
spacemit_pcie_msi_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{
}

/* Register as an MSI controller and program the DWC MSI receiver. */
static int
spacemit_pcie_msi_attach(struct spacemit_pcie_softc *sc)
{
	device_t dev = sc->dev;
	const char *name;
	phandle_t xref;
	int error, i, rid;

	mtx_init(&sc->msi_mtx, "spacemit_pcie msi", NULL, MTX_DEF);

	rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "cannot allocate 'msi' IRQ\n");
		return (ENXIO);
	}
	error = bus_setup_intr(dev, sc->irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
	    spacemit_pcie_msi_intr, NULL, sc, &sc->irq_cookie);
	if (error != 0) {
		device_printf(dev, "cannot set up 'msi' IRQ\n");
		return (error);
	}

	/* One page used purely as the DWC MSI target; writes are intercepted
	 * by the controller's MSI receiver and never reach this memory. */
	sc->msi_page = contigmalloc(PAGE_SIZE, M_DEVBUF, M_WAITOK | M_ZERO,
	    0, BUS_SPACE_MAXADDR, PAGE_SIZE, 0);
	sc->msi_target = vtophys(sc->msi_page);

	sc->isrcs = mallocarray(SPM_MSI_COUNT, sizeof(*sc->isrcs), M_DEVBUF,
	    M_WAITOK | M_ZERO);
	name = device_get_nameunit(dev);
	for (i = 0; i < SPM_MSI_COUNT; i++) {
		sc->isrcs[i].irq = i;
		error = intr_isrc_register(&sc->isrcs[i].isrc, dev, 0,
		    "%s,%u", name, i);
		if (error != 0) {
			device_printf(dev, "intr_isrc_register %d failed\n", i);
			return (error);
		}
	}

	/* Program the MSI target address; enable all vectors, mask each until
	 * a consumer unmasks it via pic_enable_intr. */
	pci_dw_dbi_wr4(dev, PCIE_MSI_ADDR_LO, (uint32_t)sc->msi_target);
	pci_dw_dbi_wr4(dev, PCIE_MSI_ADDR_HI,
	    (uint32_t)(sc->msi_target >> 32));
	pci_dw_dbi_wr4(dev, PCIE_MSI_INTR0_ENABLE, 0xffffffff);
	pci_dw_dbi_wr4(dev, PCIE_MSI_INTR0_MASK, 0xffffffff);
	pci_dw_dbi_wr4(dev, PCIE_MSI_INTR0_STATUS, 0xffffffff);

	xref = OF_xref_from_node(sc->node);
	error = intr_msi_register(dev, xref);
	if (error != 0) {
		device_printf(dev, "intr_msi_register failed\n");
		return (error);
	}
	if (bootverbose)
		device_printf(dev, "DWC integrated MSI: %d vectors\n",
		    SPM_MSI_COUNT);
	return (0);
}
/* ---- end MSI controller ---- */

static bus_dma_tag_t
spacemit_pcie_get_dma_tag(device_t dev, device_t child __unused)
{
	struct spacemit_pcie_softc *sc;

	sc = device_get_softc(dev);
	return (sc->dma_tag);
}

static device_method_t spacemit_pcie_methods[] = {
	DEVMETHOD(device_probe,		spacemit_pcie_probe),
	DEVMETHOD(device_attach,	spacemit_pcie_attach),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),

	DEVMETHOD(pci_dw_get_link,	spacemit_pcie_get_link),
	DEVMETHOD(bus_get_dma_tag,	spacemit_pcie_get_dma_tag),

	/* MSI controller: override pci_dw's msimap-based versions with ours. */
	DEVMETHOD(pcib_alloc_msi,	spacemit_pcie_alloc_msi),
	DEVMETHOD(pcib_release_msi,	spacemit_pcie_release_msi),
	DEVMETHOD(pcib_map_msi,		spacemit_pcie_map_msi),
	DEVMETHOD(pcib_alloc_msix,	spacemit_pcie_alloc_msix),
	DEVMETHOD(pcib_release_msix,	spacemit_pcie_release_msix),

	DEVMETHOD(msi_alloc_msi,	spacemit_pcie_msi_alloc_msi),
	DEVMETHOD(msi_release_msi,	spacemit_pcie_msi_release_msi),
	DEVMETHOD(msi_map_msi,		spacemit_pcie_msi_map_msi),
	DEVMETHOD(msi_alloc_msix,	spacemit_pcie_msi_alloc_msix),
	DEVMETHOD(msi_release_msix,	spacemit_pcie_msi_release_msix),

	DEVMETHOD(pic_enable_intr,	spacemit_pcie_msi_enable_intr),
	DEVMETHOD(pic_disable_intr,	spacemit_pcie_msi_disable_intr),
	DEVMETHOD(pic_post_filter,	spacemit_pcie_msi_post_filter),
	DEVMETHOD(pic_pre_ithread,	spacemit_pcie_msi_pre_ithread),
	DEVMETHOD(pic_post_ithread,	spacemit_pcie_msi_post_ithread),

	DEVMETHOD_END
};

DEFINE_CLASS_1(spacemit_pcie, spacemit_pcie_driver, spacemit_pcie_methods,
    sizeof(struct spacemit_pcie_softc), pci_dw_driver);

DRIVER_MODULE(spacemit_pcie, simplebus, spacemit_pcie_driver, NULL, NULL);
MODULE_DEPEND(spacemit_pcie, pci_dw, 1, 1, 1);
MODULE_VERSION(spacemit_pcie, 1);
