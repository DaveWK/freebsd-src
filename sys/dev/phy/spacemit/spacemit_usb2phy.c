/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Daniel Shue <dgshue@gmail.com>
 *
 * USB 2.0 (UTMI) PHY driver for the SpacemiT K1 (Ky X1) RISC-V SoC
 * (compatible: spacemit,k1-usb2-phy).  Init sequence and register layout
 * from the vendor Linux driver (drivers/usb/phy/phy-k1x-ci-usb2.c).  The
 * dwc3 host driver calls phy_enable("usb2-phy") which runs the init here.
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

#include <dev/phy/phy.h>

#define	USB2_PHY_REG01			0x04	/* PHY_RST_MODE_CTRL */
#define	 USB2_PHY_REG01_PLL_IS_READY	(1u << 0)
#define	 USB2_PHY_REG01_CLK_CDR_EN	(1u << 1)
#define	 USB2_PHY_REG01_CLK_PLL_EN	(1u << 2)
#define	 USB2_PHY_REG01_CLK_MAC_EN	(1u << 3)
#define	 USB2_PHY_REG01_MAC_RSTN	(1u << 5)
#define	 USB2_PHY_REG01_CDR_RSTN	(1u << 6)
#define	 USB2_PHY_REG01_PLL_RSTN	(1u << 7)
#define	 USB2_PHY_REG01_HS_LINE_TX_MODE	(1u << 13)
#define	 USB2_PHY_REG01_FS_LINE_TX_MODE	(1u << 14)
#define	 USB2_PHY_REG01_CLK_ENABLE	(USB2_PHY_REG01_CLK_PLL_EN |	\
					 USB2_PHY_REG01_CLK_CDR_EN |	\
					 USB2_PHY_REG01_CLK_MAC_EN)
#define	 USB2_PHY_REG01_DEASSERT_RST	(USB2_PHY_REG01_PLL_RSTN |	\
					 USB2_PHY_REG01_CDR_RSTN |	\
					 USB2_PHY_REG01_MAC_RSTN)
#define	 USB2_PHY_REG01_INIT_MODE	(USB2_PHY_REG01_FS_LINE_TX_MODE | \
					 USB2_PHY_REG01_HS_LINE_TX_MODE)
#define	USB2_PHY_REG04			0x10
#define	 USB2_PHY_REG04_AUTO_CLEAR_DIS	(1u << 2)
#define	USB2_PHY_REG0D			0x34
/*
 * PHY_PLL_DIV_CFG.  The PHY PLL does not start until its divider has been
 * programmed for the board's reference clock.  0xbec4 = PHY_DIV_LOCAL_EN |
 * freq_sel(24 MHz) | the vendor's default fractional divider, i.e. exactly
 * what Linux's phy-k1-usb2.c writes for a 24 MHz input.
 */
#define	USB2_PHY_PLL_DIV_CFG		0x98
#define	 USB2_PHY_PLL_DIV_CFG_24M	0xbec4
#define	USB2_ANALOG_REG14_13		0xa4
#define	 USB2_ANALOG_HSDAC_IREG_EN	(1u << 4)
#define	 USB2_ANALOG_HSDAC_ISEL_MASK	0xf
#define	 USB2_ANALOG_HSDAC_ISEL_15_INC	0xc

struct spusb2_softc {
	device_t	dev;
	struct resource	*mem_res;
};

static struct ofw_compat_data compat_data[] = {
	{ "spacemit,k1-usb2-phy",	1 },
	{ NULL,				0 }
};

#define	RD4(sc, r)	bus_read_4((sc)->mem_res, (r))
#define	WR4(sc, r, v)	bus_write_4((sc)->mem_res, (r), (v))

static int
spusb2_enable(struct phynode *phynode, bool enable)
{
	struct spusb2_softc *sc;
	device_t dev;
	uint32_t val;
	int i;

	dev = phynode_get_device(phynode);
	sc = device_get_softc(dev);

	if (!enable)
		return (0);

	/*
	 * Let any controller reset settle before touching the PHY; Linux
	 * waits 150-200 us here for the same reason.
	 */
	DELAY(200);

	/*
	 * Program the PLL divider for the 24 MHz reference FIRST.  Until this
	 * is written the PLL never starts, and with a stopped PLL the clock
	 * enables in REG01 do not latch: writing 0x60ef reads back as 0x60e9
	 * (CLK_PLL_EN and CLK_CDR_EN stuck at 0), the UTMI clock never runs,
	 * and the DWC3 host block that this PHY feeds can no longer complete
	 * a reset -- USBCMD.HCRST is set and never self-clears, which the
	 * XHCI layer reports as "Controller reset timeout".  Measured on an
	 * Orange Pi R2S: without this write HCRST stays set indefinitely
	 * (>10 s); with it, it clears in under a millisecond.
	 */
	WR4(sc, USB2_PHY_PLL_DIV_CFG, USB2_PHY_PLL_DIV_CFG_24M);

	/* Now the PLL can lock. */
	for (i = 0; i < 1000; i++) {
		if (RD4(sc, USB2_PHY_REG01) & USB2_PHY_REG01_PLL_IS_READY)
			break;
		DELAY(100);
	}
	if (i == 1000)
		device_printf(dev, "warning: PHY PLL not ready\n");

	/* Release the PHY's internal resets and enable its clocks. */
	WR4(sc, USB2_PHY_REG01, USB2_PHY_REG01_INIT_MODE |
	    USB2_PHY_REG01_CLK_ENABLE | USB2_PHY_REG01_DEASSERT_RST);
	WR4(sc, USB2_PHY_REG0D, 0x1c);

	val = RD4(sc, USB2_PHY_REG01);
	if ((val & USB2_PHY_REG01_CLK_ENABLE) != USB2_PHY_REG01_CLK_ENABLE)
		device_printf(dev, "warning: PHY clock enables did not latch "
		    "(REG01 %#x); USB host reset will hang\n", val);

	val = RD4(sc, USB2_ANALOG_REG14_13);
	val &= ~USB2_ANALOG_HSDAC_ISEL_MASK;
	val |= USB2_ANALOG_HSDAC_ISEL_15_INC | USB2_ANALOG_HSDAC_IREG_EN;
	WR4(sc, USB2_ANALOG_REG14_13, val);

	val = RD4(sc, USB2_PHY_REG04);
	val |= USB2_PHY_REG04_AUTO_CLEAR_DIS;
	WR4(sc, USB2_PHY_REG04, val);

	return (0);
}

static phynode_method_t spusb2_phynode_methods[] = {
	PHYNODEMETHOD(phynode_enable,	spusb2_enable),
	PHYNODEMETHOD_END
};
DEFINE_CLASS_1(spusb2_phynode, spusb2_phynode_class, spusb2_phynode_methods,
    0, phynode_class);

static int
spusb2_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "SpacemiT K1 USB2 PHY");
	return (BUS_PROBE_DEFAULT);
}

static int
spusb2_attach(device_t dev)
{
	struct spusb2_softc *sc;
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

	bzero(&phy_init, sizeof(phy_init));
	phy_init.id = 0;
	phy_init.ofw_node = ofw_bus_get_node(dev);
	phynode = phynode_create(dev, &spusb2_phynode_class, &phy_init);
	if (phynode == NULL) {
		device_printf(dev, "failed to create USB2 PHY\n");
		return (ENXIO);
	}
	if (phynode_register(phynode) == NULL) {
		device_printf(dev, "failed to register USB2 PHY\n");
		return (ENXIO);
	}
	return (0);
}

static device_method_t spusb2_methods[] = {
	DEVMETHOD(device_probe,		spusb2_probe),
	DEVMETHOD(device_attach,	spusb2_attach),
	DEVMETHOD_END
};

static driver_t spusb2_driver = {
	"spacemit_usb2phy",
	spusb2_methods,
	sizeof(struct spusb2_softc),
};

EARLY_DRIVER_MODULE(spacemit_usb2phy, simplebus, spusb2_driver, 0, 0,
    BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_EARLY);
