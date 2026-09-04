/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Daniel Shue <dgshue@gmail.com>
 *
 * Real-time clock driver for the SpacemiT K1 (Ky X1) RISC-V SoC.  The K1
 * embeds a Marvell MMP-style RTC (compatible "mrvl,mmp-rtc") at 0xd4010000.
 * This IP keeps a simple 32-bit seconds counter (RCNR) that increments at
 * 1 Hz; reading it yields UTC seconds since the Unix epoch and writing it
 * sets the clock.  This is a minimal polled clock_if(9) provider: gettime
 * and settime only, no alarm/periodic interrupt (basic wall-clock time is
 * all a headless firewall needs, and skipping the IRQ avoids any conflict).
 *
 * The RTC counter is in the always-on RTC power domain, so it survives a
 * warm reboot (the SoC stays powered).  It does NOT necessarily survive a
 * full power-off on this board -- persistent-across-power-loss time is a
 * PMIC-RTC concern (spacemit,p1), which is deliberately not implemented.
 *
 * Register model from the mainline/vendor rtc-sa1100.c "mrvl,mmp-rtc" path.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/clock.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>

#include "clock_if.h"

/*
 * MMP RTC registers.  For "mrvl,mmp-rtc" the 32-bit seconds counter (RCNR)
 * is at offset 0; RTSR/RTAR/RTTR (status/alarm/trim) are unused here.
 */
#define	RTC_RCNR	0x00	/* 32-bit up-counter, seconds since epoch */
#define	RTC_RTTR	0x0c	/* clock divider/trim; sets the 1 Hz tick */
#define	 RTC_RTTR_DEF_DIVIDER	(32768 - 1)	/* 32.768 kHz -> 1 Hz */

struct sprtc_softc {
	device_t	dev;
	struct resource	*mem_res;
	struct mtx	mtx;
};

static struct ofw_compat_data compat_data[] = {
	{ "mrvl,mmp-rtc",	1 },
	{ NULL,			0 }
};

#define	RD4(sc, r)	bus_read_4((sc)->mem_res, (r))
#define	WR4(sc, r, v)	bus_write_4((sc)->mem_res, (r), (v))

static int
sprtc_gettime(device_t dev, struct timespec *ts)
{
	struct sprtc_softc *sc;
	uint32_t sec;

	sc = device_get_softc(dev);

	mtx_lock(&sc->mtx);
	sec = RD4(sc, RTC_RCNR);
	mtx_unlock(&sc->mtx);

	ts->tv_sec = sec;
	ts->tv_nsec = 0;

	return (0);
}

static int
sprtc_settime(device_t dev, struct timespec *ts)
{
	struct sprtc_softc *sc;

	sc = device_get_softc(dev);

	/*
	 * We register with CLOCKF_SETTIME_NO_ADJ, which also disables the
	 * UTC adjustment, so apply utc_offset() ourselves: the counter holds
	 * UTC seconds since the epoch.
	 */
	ts->tv_sec -= utc_offset();

	mtx_lock(&sc->mtx);
	WR4(sc, RTC_RCNR, (uint32_t)ts->tv_sec);
	mtx_unlock(&sc->mtx);

	return (0);
}

static int
sprtc_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "SpacemiT K1 RTC");
	return (BUS_PROBE_DEFAULT);
}

static int
sprtc_attach(device_t dev)
{
	struct sprtc_softc *sc;
	clk_t clk;
	hwreset_t rst;
	int i, rid;

	sc = device_get_softc(dev);
	sc->dev = dev;
	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "cannot allocate registers\n");
		mtx_destroy(&sc->mtx);
		return (ENXIO);
	}

	/* Deassert reset, then enable the RTC clocks (best effort). */
	if (hwreset_get_by_ofw_idx(dev, 0, 0, &rst) == 0)
		(void)hwreset_deassert(rst);
	for (i = 0; clk_get_by_ofw_index(dev, 0, i, &clk) == 0; i++)
		(void)clk_enable(clk);

	/*
	 * If the clock divider is uninitialized (RTTR == 0) the seconds
	 * counter does not tick.  Program the default 32.768 kHz -> 1 Hz
	 * divider (trim 0).  Per the vendor/mainline driver, an uninitialized
	 * RTTR also means the current RCNR is meaningless, so leave the time
	 * unset (0) until userland sets it (settime / ntpd).
	 */
	if (RD4(sc, RTC_RTTR) == 0)
		WR4(sc, RTC_RTTR, RTC_RTTR_DEF_DIVIDER);

	/*
	 * Register as a 1-second-resolution clock.  CLOCKF_SETTIME_NO_ADJ
	 * because the counter is whole seconds; we handle utc_offset in
	 * settime.
	 */
	clock_register_flags(dev, 1000000, CLOCKF_SETTIME_NO_ADJ);

	if (bootverbose)
		device_printf(dev, "seconds counter now %u\n",
		    RD4(sc, RTC_RCNR));

	return (0);
}

static device_method_t sprtc_methods[] = {
	DEVMETHOD(device_probe,		sprtc_probe),
	DEVMETHOD(device_attach,	sprtc_attach),

	/* clock interface */
	DEVMETHOD(clock_gettime,	sprtc_gettime),
	DEVMETHOD(clock_settime,	sprtc_settime),

	DEVMETHOD_END
};

static driver_t sprtc_driver = {
	"spacemit_rtc",
	sprtc_methods,
	sizeof(struct sprtc_softc),
};

DRIVER_MODULE(spacemit_rtc, simplebus, sprtc_driver, 0, 0);
