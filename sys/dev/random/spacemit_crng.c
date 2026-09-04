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
 * random_source(9) provider for the SpacemiT K1 (Ky X1) hardware CRNG/TRNG
 * (compatible: "spacemit,hw_crng"), at 0xf0703800.  The block shares the AES
 * engine's clock/reset domain (CLK_AES / RESET_AES in the APMU syscon).
 *
 * We use the CPU register-read path (no DMA): seed the generator from a
 * software seed, kick RNG_EN, poll RNG_CTRL for the VALID bit, then read
 * 32-bit words out of RNG_DATA, clearing the squeezer FIFO between reads and
 * rejecting a word identical to the previous one (a stuck-output guard).
 * Register semantics and the read sequence were derived from the vendor BSP
 * driver drivers/char/hw_random/spacemit-crng.c (GPL-2.0; reimplemented, not
 * copied).  This entropy is whitened by the kernel's Fortuna pool, so it is
 * registered as a RANDOM_PURE_* source rather than trusted directly.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/random.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/stdatomic.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>

#include <dev/random/randomdev.h>
#include <dev/random/random_harvestq.h>

/* Register offsets (from the block base). */
#define	RNG_SQU_CTRL		0x44
#define	RNG_CTRL		0xc0
#define	RNG_DATA		0xc4
#define	RNG_SEED_VAL		0xc8
#define	RNG_CTRL_EXT		0xd0

/* RNG_SQU_CTRL bits. */
#define	SQU_CTRL_FIFO_CLR	(1u << 30)

/* RNG_CTRL bits. */
#define	CTRL_RNG_EN		(1u << 0)
#define	CTRL_RNG_SEED_EN	(1u << 1)
#define	CTRL_RNG_SEED_VALID	(1u << 30)
#define	CTRL_RNG_VALID		(1u << 31)

/* RNG_CTRL_EXT bits (cleared to select the internal generator + power up). */
#define	RNG_REG_SEL		(1u << 4)
#define	RNG_REG_LDO_PU		(1u << 6)

/* Poll budget: the seed/valid handshake is normally microseconds. */
#define	CRNG_POLL_USEC		100000

struct spacemit_crng_softc {
	device_t		dev;
	struct resource		*mem;
	int			mem_rid;
	struct mtx		mtx;
	clk_t			clk;
	hwreset_t		reset;
};

/*
 * random_source(9) uses a single global read callback with no context arg,
 * so stash the instance (there is only one CRNG on the SoC).
 */
static _Atomic(struct spacemit_crng_softc *) g_crng_sc;

static unsigned	spacemit_crng_read(void *, unsigned);

static const struct random_source random_spacemit_crng = {
	.rs_ident = "SpacemiT K1 CRNG",
	.rs_source = RANDOM_PURE_SPACEMIT,
	.rs_read = spacemit_crng_read,
};

static struct ofw_compat_data compat_data[] = {
	{ "spacemit,hw_crng",	1 },
	{ NULL,			0 }
};

#define	CRNG_RD4(sc, off)	bus_read_4((sc)->mem, (off))
#define	CRNG_WR4(sc, off, v)	bus_write_4((sc)->mem, (off), (v))

static void
crng_fifo_clear(struct spacemit_crng_softc *sc)
{
	CRNG_WR4(sc, RNG_SQU_CTRL, CRNG_RD4(sc, RNG_SQU_CTRL) | SQU_CTRL_FIFO_CLR);
}

static void
crng_reset_status(struct spacemit_crng_softc *sc)
{
	uint32_t val;

	crng_fifo_clear(sc);
	val = CRNG_RD4(sc, RNG_CTRL);
	val &= ~(CTRL_RNG_EN | CTRL_RNG_VALID);
	val &= ~(CTRL_RNG_SEED_EN | CTRL_RNG_SEED_VALID);
	CRNG_WR4(sc, RNG_CTRL, val);
}

/*
 * Program a software seed and wait for the generator to accept it.  The seed
 * value is only a kick for the hardware entropy path; its quality is not
 * relied upon (the kernel pool whitens the output).  Uses the CPU cycle/time
 * counter as the seed, matching the vendor sequence.
 */
static int
crng_seed(struct spacemit_crng_softc *sc)
{
	uint32_t val;
	int i;

	CRNG_WR4(sc, RNG_SEED_VAL, (uint32_t)get_cyclecount());

	val = CRNG_RD4(sc, RNG_CTRL);
	val |= CTRL_RNG_SEED_EN;
	CRNG_WR4(sc, RNG_CTRL, val);

	for (i = 0; i < CRNG_POLL_USEC; i++) {
		if (CRNG_RD4(sc, RNG_CTRL) & CTRL_RNG_SEED_VALID)
			return (0);
		DELAY(1);
	}
	return (ETIMEDOUT);
}

/*
 * Fill up to *szp bytes of buf with hardware random data.  Returns the number
 * of bytes produced (may be 0 on hardware timeout).  Caller holds sc->mtx.
 */
static size_t
crng_harvest(struct spacemit_crng_softc *sc, uint8_t *buf, size_t sz)
{
	uint32_t val, prev;
	size_t done;
	int i;

	if (crng_seed(sc) != 0)
		return (0);

	/* Select internal generator, power up LDO, then enable. */
	val = CRNG_RD4(sc, RNG_CTRL_EXT);
	val &= ~(RNG_REG_SEL | RNG_REG_LDO_PU);
	CRNG_WR4(sc, RNG_CTRL_EXT, val);

	val = CRNG_RD4(sc, RNG_CTRL);
	val |= CTRL_RNG_EN;
	CRNG_WR4(sc, RNG_CTRL, val);

	for (i = 0; i < CRNG_POLL_USEC; i++) {
		if (CRNG_RD4(sc, RNG_CTRL) & CTRL_RNG_VALID)
			break;
		DELAY(1);
	}
	if ((CRNG_RD4(sc, RNG_CTRL) & CTRL_RNG_VALID) == 0)
		return (0);

	prev = 0;
	done = 0;
	while (done + sizeof(uint32_t) <= sz) {
		int tries;

		val = 0;
		for (tries = 0; tries < 16; tries++) {
			crng_fifo_clear(sc);
			val = CRNG_RD4(sc, RNG_DATA);
			if (val != prev)
				break;
		}
		if (val == prev)	/* stuck output: stop, report partial */
			break;
		memcpy(buf + done, &val, sizeof(uint32_t));
		done += sizeof(uint32_t);
		prev = val;
	}

	return (done);
}

static unsigned
spacemit_crng_read(void *buf, unsigned usz)
{
	struct spacemit_crng_softc *sc;
	size_t got, off, chunk;

	sc = atomic_load_explicit(&g_crng_sc, memory_order_acquire);
	if (sc == NULL)
		return (0);

	off = 0;
	mtx_lock(&sc->mtx);
	/* Work in <=16-byte chunks, resetting the block between them. */
	while (off < usz) {
		chunk = MIN((size_t)usz - off, 16);
		got = crng_harvest(sc, (uint8_t *)buf + off, chunk);
		crng_reset_status(sc);
		if (got == 0)
			break;
		off += got;
	}
	mtx_unlock(&sc->mtx);

	return ((unsigned)off);
}

static int
spacemit_crng_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "SpacemiT K1 hardware CRNG");
	return (BUS_PROBE_DEFAULT);
}

static int
spacemit_crng_attach(device_t dev)
{
	struct spacemit_crng_softc *sc, *exp;

	sc = device_get_softc(dev);
	sc->dev = dev;

	exp = NULL;
	if (!atomic_compare_exchange_strong_explicit(&g_crng_sc, &exp, sc,
	    memory_order_release, memory_order_acquire)) {
		device_printf(dev, "another CRNG instance already active\n");
		return (ENXIO);
	}

	/*
	 * Enable the AES/CRNG functional clock and release the shared reset.
	 * Both are best-effort: firmware normally leaves them on, and the
	 * block is shared with the (unused) AES engine, so treat absence or
	 * failure as non-fatal rather than refusing to provide entropy.
	 */
	if (clk_get_by_ofw_index(dev, 0, 0, &sc->clk) == 0) {
		if (clk_enable(sc->clk) != 0)
			device_printf(dev, "warning: could not enable clock\n");
	} else
		sc->clk = NULL;

	if (hwreset_get_by_ofw_idx(dev, 0, 0, &sc->reset) == 0) {
		if (hwreset_deassert(sc->reset) != 0)
			device_printf(dev, "warning: could not deassert reset\n");
	} else
		sc->reset = NULL;

	sc->mem_rid = 0;
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid,
	    RF_ACTIVE);
	if (sc->mem == NULL) {
		device_printf(dev, "cannot allocate memory resource\n");
		goto fail;
	}

	mtx_init(&sc->mtx, "spacemit_crng", NULL, MTX_DEF);

	/* Leave the block in a clean state before first use. */
	crng_reset_status(sc);

	random_source_register(&random_spacemit_crng);
	return (0);

fail:
	if (sc->reset != NULL)
		hwreset_release(sc->reset);
	if (sc->clk != NULL)
		clk_release(sc->clk);
	atomic_store_explicit(&g_crng_sc, NULL, memory_order_release);
	return (ENXIO);
}

static int
spacemit_crng_detach(device_t dev)
{
	struct spacemit_crng_softc *sc;

	sc = device_get_softc(dev);

	random_source_deregister(&random_spacemit_crng);
	if (sc->mem != NULL) {
		mtx_destroy(&sc->mtx);
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem);
	}
	if (sc->reset != NULL)
		hwreset_release(sc->reset);
	if (sc->clk != NULL)
		clk_release(sc->clk);
	atomic_store_explicit(&g_crng_sc, NULL, memory_order_release);
	return (0);
}

static device_method_t spacemit_crng_methods[] = {
	DEVMETHOD(device_probe,		spacemit_crng_probe),
	DEVMETHOD(device_attach,	spacemit_crng_attach),
	DEVMETHOD(device_detach,	spacemit_crng_detach),

	DEVMETHOD_END
};

static driver_t spacemit_crng_driver = {
	"spacemit_crng",
	spacemit_crng_methods,
	sizeof(struct spacemit_crng_softc)
};

DRIVER_MODULE(spacemit_crng, simplebus, spacemit_crng_driver, NULL, NULL);
MODULE_DEPEND(spacemit_crng, random_device, 1, 1, 1);
MODULE_VERSION(spacemit_crng, 1);
