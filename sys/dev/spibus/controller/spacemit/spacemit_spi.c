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
 * spibus(4) controller driver for the SpacemiT K1 (Ky X1) SSP-based SPI
 * (compatible: "spacemit,k1-spi").  This is a Marvell/PXA SSP-style port
 * (SSP_TOP_CTRL / SSP_STATUS / SSP_DATAR with a 32-entry FIFO).  We drive it
 * in polled, byte-at-a-time full-duplex mode, which is simple and robust for
 * the typical SPI peripherals on the header (sensors, small displays).
 *
 * The bit clock is the SSP functional clock as programmed by firmware/CCU
 * (our CCU exposes it read-only); per-transfer speed reprogramming is not
 * done here.  Register semantics reimplemented from the mainline Linux driver
 * drivers/spi/spi-spacemit-k1.c (GPL-2.0), NOT copied; wrapped in FreeBSD's
 * spibus framework modeled on dev/spibus/controller/allwinner/aw_spi.c.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>

#include <dev/spibus/spi.h>
#include <dev/spibus/spibusvar.h>

#include "spibus_if.h"

/* SSP registers. */
#define	SSP_TOP_CTRL	0x00
#define	 TOP_SSE	(1u << 0)	/* port enable */
#define	 TOP_FRF_MASK	(0x3u << 1)	/* frame format (0 = Motorola SPI) */
#define	 TOP_DSS_SHIFT	5		/* data size select (val = bits-1) */
#define	 TOP_DSS_MASK	(0x1fu << 5)
#define	 TOP_SPO	(1u << 10)	/* clock polarity */
#define	 TOP_SPH	(1u << 11)	/* clock phase */
#define	 TOP_HOLD_FRAME_LOW (1u << 14)	/* hold CS asserted */

#define	SSP_FIFO_CTRL	0x04
#define	SSP_STATUS	0x14
#define	 SSP_STATUS_BSY	(1u << 0)	/* busy */
#define	 SSP_STATUS_TNF	(1u << 6)	/* TX FIFO not full */
#define	 SSP_STATUS_RNE	(1u << 14)	/* RX FIFO not empty */
#define	SSP_DATAR	0x10

#define	SSP_POLL_USEC	100000

struct spacemit_spi_softc {
	device_t	dev;
	device_t	spibus;
	struct resource	*mem;
	int		mem_rid;
	struct mtx	mtx;
	clk_t		func_clk;
	clk_t		bus_clk;
	hwreset_t	reset;
};

static struct ofw_compat_data compat_data[] = {
	{ "spacemit,k1-spi",	1 },
	{ NULL,			0 }
};

#define	RD4(sc, r)	bus_read_4((sc)->mem, (r))
#define	WR4(sc, r, v)	bus_write_4((sc)->mem, (r), (v))

/* Configure port: 8-bit Motorola SPI with the requested mode, and enable. */
static void
spi_configure(struct spacemit_spi_softc *sc, uint32_t mode)
{
	uint32_t ctrl;

	ctrl = RD4(sc, SSP_TOP_CTRL);
	ctrl &= ~(TOP_FRF_MASK | TOP_DSS_MASK | TOP_SPO | TOP_SPH);
	ctrl |= (7u << TOP_DSS_SHIFT);		/* 8-bit words */
	if (mode & SPIBUS_MODE_CPOL)
		ctrl |= TOP_SPO;
	if (mode & SPIBUS_MODE_CPHA)
		ctrl |= TOP_SPH;
	ctrl |= TOP_SSE;
	WR4(sc, SSP_TOP_CTRL, ctrl);
}

static void
spi_disable(struct spacemit_spi_softc *sc)
{

	WR4(sc, SSP_TOP_CTRL, RD4(sc, SSP_TOP_CTRL) & ~TOP_SSE);
}

/* Full-duplex byte transfer: write one byte, read one byte. */
static int
spi_xfer_buf(struct spacemit_spi_softc *sc, uint8_t *rx, const uint8_t *tx,
    uint32_t len)
{
	uint32_t i;
	int t;

	for (i = 0; i < len; i++) {
		for (t = 0; t < SSP_POLL_USEC; t++) {
			if (RD4(sc, SSP_STATUS) & SSP_STATUS_TNF)
				break;
			DELAY(1);
		}
		if ((RD4(sc, SSP_STATUS) & SSP_STATUS_TNF) == 0)
			return (EIO);
		WR4(sc, SSP_DATAR, tx != NULL ? tx[i] : 0);

		for (t = 0; t < SSP_POLL_USEC; t++) {
			if (RD4(sc, SSP_STATUS) & SSP_STATUS_RNE)
				break;
			DELAY(1);
		}
		if ((RD4(sc, SSP_STATUS) & SSP_STATUS_RNE) == 0)
			return (EIO);
		if (rx != NULL)
			rx[i] = (uint8_t)RD4(sc, SSP_DATAR);
		else
			(void)RD4(sc, SSP_DATAR);
	}
	return (0);
}

static int
spacemit_spi_transfer(device_t dev, device_t child, struct spi_command *cmd)
{
	struct spacemit_spi_softc *sc;
	uint32_t cs, mode, clock;
	int error;

	sc = device_get_softc(dev);

	spibus_get_cs(child, &cs);
	spibus_get_clock(child, &clock);
	spibus_get_mode(child, &mode);
	cs &= ~SPIBUS_CS_HIGH;		/* K1 SSP drives CS in hardware */

	mtx_lock(&sc->mtx);

	spi_configure(sc, mode);
	/* Assert chip select (hold the frame low for the whole command). */
	WR4(sc, SSP_TOP_CTRL, RD4(sc, SSP_TOP_CTRL) | TOP_HOLD_FRAME_LOW);

	error = 0;
	if (cmd->tx_cmd_sz > 0)
		error = spi_xfer_buf(sc, cmd->rx_cmd, cmd->tx_cmd,
		    cmd->tx_cmd_sz);
	if (error == 0 && cmd->tx_data_sz > 0)
		error = spi_xfer_buf(sc, cmd->rx_data, cmd->tx_data,
		    cmd->tx_data_sz);

	/* Deassert chip select and disable the port. */
	WR4(sc, SSP_TOP_CTRL, RD4(sc, SSP_TOP_CTRL) & ~TOP_HOLD_FRAME_LOW);
	spi_disable(sc);

	mtx_unlock(&sc->mtx);
	return (error);
}

static int
spacemit_spi_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "SpacemiT K1 SPI");
	return (BUS_PROBE_DEFAULT);
}

static int
spacemit_spi_attach(device_t dev)
{
	struct spacemit_spi_softc *sc;

	sc = device_get_softc(dev);
	sc->dev = dev;

	sc->mem_rid = 0;
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid,
	    RF_ACTIVE);
	if (sc->mem == NULL) {
		device_printf(dev, "cannot allocate memory resource\n");
		return (ENXIO);
	}

	if (clk_get_by_ofw_name(dev, 0, "core", &sc->func_clk) != 0 ||
	    clk_enable(sc->func_clk) != 0) {
		device_printf(dev, "cannot enable core clock\n");
		goto fail;
	}
	if (clk_get_by_ofw_name(dev, 0, "bus", &sc->bus_clk) != 0 ||
	    clk_enable(sc->bus_clk) != 0) {
		device_printf(dev, "cannot enable bus clock\n");
		goto fail;
	}

	if (hwreset_get_by_ofw_idx(dev, 0, 0, &sc->reset) == 0)
		(void)hwreset_deassert(sc->reset);
	else
		sc->reset = NULL;

	mtx_init(&sc->mtx, "spacemit_spi", NULL, MTX_DEF);

	/* Start with the port disabled. */
	spi_disable(sc);

	sc->spibus = device_add_child(dev, "spibus", DEVICE_UNIT_ANY);
	if (sc->spibus == NULL) {
		device_printf(dev, "cannot add spibus child\n");
		mtx_destroy(&sc->mtx);
		goto fail;
	}

	bus_attach_children(dev);
	return (0);

fail:
	if (sc->reset != NULL)
		hwreset_release(sc->reset);
	if (sc->bus_clk != NULL)
		clk_release(sc->bus_clk);
	if (sc->func_clk != NULL)
		clk_release(sc->func_clk);
	bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem);
	return (ENXIO);
}

static int
spacemit_spi_detach(device_t dev)
{
	struct spacemit_spi_softc *sc;
	int error;

	sc = device_get_softc(dev);
	error = bus_generic_detach(dev);
	if (error != 0)
		return (error);

	if (sc->reset != NULL)
		hwreset_release(sc->reset);
	if (sc->bus_clk != NULL)
		clk_release(sc->bus_clk);
	if (sc->func_clk != NULL)
		clk_release(sc->func_clk);
	bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem);
	mtx_destroy(&sc->mtx);
	return (0);
}

static phandle_t
spacemit_spi_get_node(device_t bus, device_t dev)
{

	return (ofw_bus_get_node(bus));
}

static device_method_t spacemit_spi_methods[] = {
	DEVMETHOD(device_probe,		spacemit_spi_probe),
	DEVMETHOD(device_attach,	spacemit_spi_attach),
	DEVMETHOD(device_detach,	spacemit_spi_detach),

	DEVMETHOD(spibus_transfer,	spacemit_spi_transfer),

	DEVMETHOD(ofw_bus_get_node,	spacemit_spi_get_node),

	DEVMETHOD_END
};

static driver_t spacemit_spi_driver = {
	"spacemit_spi",
	spacemit_spi_methods,
	sizeof(struct spacemit_spi_softc)
};

DRIVER_MODULE(spacemit_spi, simplebus, spacemit_spi_driver, NULL, NULL);
DRIVER_MODULE(ofw_spibus, spacemit_spi, ofw_spibus_driver, NULL, NULL);
MODULE_DEPEND(spacemit_spi, spibus, 1, 1, 1);
MODULE_VERSION(spacemit_spi, 1);
