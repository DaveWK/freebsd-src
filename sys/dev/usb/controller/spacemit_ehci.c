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
 * EHCI (USB 2.0 host) driver for the SpacemiT K1 (Ky X1) Marvell-derived
 * USB2 controllers (compatible: "spacemit,mv-ehci") at 0xc0900100 and
 * 0xc0980100.  These are extra USB2 host ports, separate from the DWC3/USB3
 * controller (which already provides working USB2+USB3).
 *
 * This is a self-contained variant of dev/usb/controller/ehci_mv.c: it uses
 * the same generic EHCI core (ehci_init/ehci_detach + the Marvell host
 * subregion + USBMODE post-reset workaround) but adds the K1-specific
 * clock/reset/PHY bring-up the plain ehci_mv driver does not do.  It is kept
 * separate so the shared ehci_mv.c (used by real Marvell SoCs) is not
 * touched.
 *
 * NOTE ON USB3: the DWC3/USB3 controller uses CLK_USB30 / RESET_USB30_* and
 * is unaffected by this driver, which only enables CLK_USB_AXI / CLK_USB_P1
 * and the USB2 PHY.  The DTB nodes ship "disabled" by default, so this driver
 * has no effect unless a board explicitly enables a USB2 port.
 */

#include <sys/stdint.h>
#include <sys/stddef.h>
#include <sys/param.h>
#include <sys/queue.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/condvar.h>
#include <sys/sx.h>
#include <sys/unistd.h>
#include <sys/callout.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/phy/phy.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usb_core.h>
#include <dev/usb/usb_busdma.h>
#include <dev/usb/usb_process.h>
#include <dev/usb/usb_util.h>
#include <dev/usb/usb_controller.h>
#include <dev/usb/usb_bus.h>
#include <dev/usb/controller/ehci.h>
#include <dev/usb/controller/ehcireg.h>

/* Marvell integrated USB: host controller sits 0x100 into the reg window. */
#define	MV_USB_HOST_OFST	0x0100
#define	USB_BRIDGE_INTR_MASK	0x214
#define	MV_USB_ADDR_DECODE_ERR	(1u << 0)
#define	MV_USB_HOST_UNDERFLOW	(1u << 1)
#define	MV_USB_HOST_OVERFLOW	(1u << 2)
#define	MV_USB_DEVICE_UNDERFLOW	(1u << 3)

#define	EHCI_HC_DEVSTR		"SpacemiT K1 USB 2.0 controller"

struct spacemit_ehci_softc {
	ehci_softc_t	base;		/* Must be first. */
	clk_t		clk;
	hwreset_t	reset;
	phy_t		phy;
};

static struct ofw_compat_data compat_data[] = {
	{ "spacemit,mv-ehci",	1 },
	{ NULL,			0 }
};

/*
 * Marvell post-reset workaround: an EHCI core reset clears USBMODE, leaving
 * the core in an undefined mode, so re-select host mode (errata GL USB-2).
 */
static void
spacemit_ehci_post_reset(struct ehci_softc *ehci_softc)
{
	uint32_t usbmode;

	usbmode = EOREAD4(ehci_softc, EHCI_USBMODE_NOLPM);
	usbmode &= ~EHCI_UM_CM;
	usbmode |= EHCI_UM_CM_HOST;
	EOWRITE4(ehci_softc, EHCI_USBMODE_NOLPM, usbmode);
}

static int
spacemit_ehci_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, EHCI_HC_DEVSTR);
	return (BUS_PROBE_DEFAULT);
}

static int
spacemit_ehci_attach(device_t dev)
{
	struct spacemit_ehci_softc *sc;
	ehci_softc_t *esc;
	bus_space_handle_t bsh;
	int rid, error;

	sc = device_get_softc(dev);
	esc = &sc->base;

	esc->sc_bus.parent = dev;
	esc->sc_bus.devices = esc->sc_devices;
	esc->sc_bus.devices_max = EHCI_MAX_DEVICES;
	esc->sc_bus.dma_bits = 32;

	/* Enable the controller's AXI clock and release its reset. */
	if (clk_get_by_ofw_index(dev, 0, 0, &sc->clk) == 0)
		(void)clk_enable(sc->clk);
	else
		sc->clk = NULL;
	if (hwreset_get_by_ofw_idx(dev, 0, 0, &sc->reset) == 0)
		(void)hwreset_deassert(sc->reset);
	else
		sc->reset = NULL;

	/* Bring up the USB2 PHY if one is described. */
	if (phy_get_by_ofw_idx(dev, ofw_bus_get_node(dev), 0, &sc->phy) == 0)
		(void)phy_enable(sc->phy);
	else
		sc->phy = NULL;

	if (usb_bus_mem_alloc_all(&esc->sc_bus, USB_GET_DMA_TAG(dev),
	    &ehci_iterate_hw_softc)) {
		error = ENOMEM;
		goto fail;
	}

	rid = 0;
	esc->sc_io_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (esc->sc_io_res == NULL) {
		device_printf(dev, "cannot map memory\n");
		error = ENXIO;
		goto fail;
	}
	esc->sc_io_tag = rman_get_bustag(esc->sc_io_res);
	bsh = rman_get_bushandle(esc->sc_io_res);
	esc->sc_io_size = rman_get_size(esc->sc_io_res) - MV_USB_HOST_OFST;

	/* The EHCI host registers begin MV_USB_HOST_OFST into the window. */
	if (bus_space_subregion(esc->sc_io_tag, bsh, MV_USB_HOST_OFST,
	    esc->sc_io_size, &esc->sc_io_hdl) != 0) {
		device_printf(dev, "cannot create host register subregion\n");
		error = ENXIO;
		goto fail;
	}

	rid = 0;
	esc->sc_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_SHAREABLE | RF_ACTIVE);
	if (esc->sc_irq_res == NULL) {
		device_printf(dev, "cannot allocate IRQ\n");
		error = ENXIO;
		goto fail;
	}

	esc->sc_bus.bdev = device_add_child(dev, "usbus", DEVICE_UNIT_ANY);
	if (esc->sc_bus.bdev == NULL) {
		device_printf(dev, "cannot add USB device\n");
		error = ENXIO;
		goto fail;
	}
	device_set_ivars(esc->sc_bus.bdev, &esc->sc_bus);
	device_set_desc(esc->sc_bus.bdev, EHCI_HC_DEVSTR);
	snprintf(esc->sc_vendor, sizeof(esc->sc_vendor), "SpacemiT");

	error = bus_setup_intr(dev, esc->sc_irq_res,
	    INTR_TYPE_BIO | INTR_MPSAFE, NULL,
	    (driver_intr_t *)ehci_interrupt, esc, &esc->sc_intr_hdl);
	if (error != 0) {
		device_printf(dev, "cannot setup IRQ, %d\n", error);
		esc->sc_intr_hdl = NULL;
		goto fail;
	}

	/* Mask the USB bridge error interrupts (we do not use the err IRQ). */
	bus_space_write_4(esc->sc_io_tag, bsh, USB_BRIDGE_INTR_MASK, 0);

	esc->sc_vendor_post_reset = spacemit_ehci_post_reset;
	esc->sc_vendor_get_port_speed = ehci_get_port_speed_portsc;
	esc->sc_flags |= EHCI_SCFLG_TT | EHCI_SCFLG_NORESTERM;

	error = ehci_init(esc);
	if (error == 0)
		error = device_probe_and_attach(esc->sc_bus.bdev);
	if (error != 0) {
		device_printf(dev, "USB init failed, %d\n", error);
		goto fail;
	}

	return (0);

fail:
	if (esc->sc_intr_hdl != NULL)
		bus_teardown_intr(dev, esc->sc_irq_res, esc->sc_intr_hdl);
	if (esc->sc_irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, esc->sc_irq_res);
	if (esc->sc_io_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, esc->sc_io_res);
	usb_bus_mem_free_all(&esc->sc_bus, &ehci_iterate_hw_softc);
	if (sc->phy != NULL)
		phy_release(sc->phy);
	if (sc->reset != NULL)
		hwreset_release(sc->reset);
	if (sc->clk != NULL)
		clk_release(sc->clk);
	return (error);
}

static int
spacemit_ehci_detach(device_t dev)
{
	struct spacemit_ehci_softc *sc;
	ehci_softc_t *esc;
	int error;

	sc = device_get_softc(dev);
	esc = &sc->base;

	error = bus_generic_detach(dev);
	if (error != 0)
		return (error);

	if (esc->sc_irq_res != NULL && esc->sc_intr_hdl != NULL) {
		ehci_detach(esc);
		bus_teardown_intr(dev, esc->sc_irq_res, esc->sc_intr_hdl);
		esc->sc_intr_hdl = NULL;
	}
	if (esc->sc_irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, esc->sc_irq_res);
	if (esc->sc_io_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, esc->sc_io_res);
	usb_bus_mem_free_all(&esc->sc_bus, &ehci_iterate_hw_softc);

	if (sc->phy != NULL)
		phy_release(sc->phy);
	if (sc->reset != NULL)
		hwreset_release(sc->reset);
	if (sc->clk != NULL)
		clk_release(sc->clk);
	return (0);
}

static device_method_t spacemit_ehci_methods[] = {
	DEVMETHOD(device_probe,		spacemit_ehci_probe),
	DEVMETHOD(device_attach,	spacemit_ehci_attach),
	DEVMETHOD(device_detach,	spacemit_ehci_detach),
	DEVMETHOD(device_suspend,	bus_generic_suspend),
	DEVMETHOD(device_resume,	bus_generic_resume),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),

	DEVMETHOD_END
};

static driver_t spacemit_ehci_driver = {
	"ehci",
	spacemit_ehci_methods,
	sizeof(struct spacemit_ehci_softc)
};

DRIVER_MODULE(spacemit_ehci, simplebus, spacemit_ehci_driver, NULL, NULL);
MODULE_DEPEND(spacemit_ehci, usb, 1, 1, 1);
MODULE_VERSION(spacemit_ehci, 1);
