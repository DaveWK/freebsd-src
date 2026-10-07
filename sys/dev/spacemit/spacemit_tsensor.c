/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Daniel Shue <dgshue@gmail.com>
 * Copyright (c) 2026 Dave Klotz <DaveWK225@protonmail.com>
 *
 * Thermal sensor driver for the SpacemiT K1 (Ky X1) RISC-V SoC
 * (compatible: spacemit,k1-tsensor).  The block has five sensors (the
 * "#thermal-sensor-cells = <1>" index): 0 soc, 1 package, 2 gpu, 3 cpu
 * cluster 0 and 4 cpu cluster 1.  Each is exposed via sysctl in 0.1 Kelvin
 * (dev.spacemit_tsensor.<unit>.temperature remains sensor 0).
 * Register layout and conversion from the mainline Linux driver
 * (drivers/thermal/spacemit/k1_tsensor.c).
 *
 * The driver also protects the SoC.  It reads the passive and critical trip
 * points of the devicetree thermal zones that use this sensor (or, with no
 * zones, built-in ones equal to mainline k1.dtsi) and polls the sensors:
 *
 *  - at or above a passive trip, while the temperature is not falling, it
 *    lowers the CPU frequency one cpufreq(4) level at a time with kernel
 *    priority (as acpi_thermal(4) passive cooling does); once every zone is
 *    below its trip minus hysteresis it raises it again one level at a time
 *    and finally restores the level that was in force before throttling.
 *    A raise that is cut again within 10 s doubles the time before the next
 *    one (up to 30 s), so a large gap between two levels does not make the
 *    cap swing every few seconds;
 *  - at a critical trip it drops to the lowest level and shuts the system
 *    down (power off by default, see hw.spacemit_tsensor.critical_action).
 *
 * Interrupt-driven trips are left for later: the sensors are polled once a
 * second, and at the zone's passive polling interval while hot.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/limits.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/proc.h>
#include <sys/reboot.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>

#include "cpufreq_if.h"

#define	TSEN_PCTRL		0x00
#define	 TSEN_PCTRL_ENABLE	(1u << 0)
#define	 TSEN_PCTRL_TEMP_MODE	(1u << 3)
#define	 TSEN_PCTRL_RAW_SEL	(1u << 7)
#define	 TSEN_PCTRL_CTUNE	(0xfu << 8)
#define	 TSEN_PCTRL_SW_CTRL	(0xfu << 18)
#define	 TSEN_PCTRL_HW_AUTO	(1u << 23)
#define	TSEN_EN			0x08
#define	TSEN_TIME		0x0c
#define	 TSEN_TIME_WAIT_REF	(0xfu << 0)
#define	 TSEN_TIME_ADC_CNT_RST	(0xfu << 4)
#define	 TSEN_TIME_FILTER_PER	(0x3u << 20)
#define	 TSEN_TIME_MASK		(0xffffffu << 0)
#define	TSEN_INT_EN		0x14
#define	TSEN_DATA(n)		(0x20 + ((n) / 2) * 4)
#define	 TSEN_DATA_SHIFT(n)	(((n) % 2) * 16)
#define	 TSEN_DATA_MASK		0xffff

#define	TSEN_NSENSORS		5
#define	TSEN_TEMP_OFFSET	278	/* raw - 278 = degrees C */

/*
 * Readings outside this window are treated as "no sample" (an unclocked or
 * disabled sensor reads 0).
 */
#define	TSEN_TEMP_MIN_C		(-40)
#define	TSEN_TEMP_MAX_C		150

static const char *sptsen_names[TSEN_NSENSORS] = {
	"soc", "package", "gpu", "cluster0", "cluster1"
};

#define	TZ_MAX_ZONES		8
#define	TZ_NONE			INT_MAX		/* no such trip */
#define	TZ_MAX_LEVELS		16
#define	TZ_UP_MAX_MS		30000	/* longest adaptive raise interval */
#define	TZ_OVERSHOOT_MS		10000	/* a cut this soon after a raise */
#define	TZ_SETTLED_MS		60000	/* no cut this long: reset interval */
#define	TZ_LOG_MS		60000	/* report throttling at most this often */
#define	TZ_MSTOHZ(ms)		(((int64_t)(ms) * hz + 999) / 1000)

struct sptsen_zone {
	char		name[32];
	int		sensor;
	int		passive;	/* millicelsius, or TZ_NONE */
	int		hysteresis;	/* millicelsius */
	int		critical;	/* millicelsius, or TZ_NONE */
	int		passive_ms;	/* polling interval while hot */
	int		last;		/* last reading, millicelsius */
	bool		valid;
};

struct sptsen_softc {
	device_t		dev;
	struct resource		*mem_res;
	struct sptsen_zone	zones[TZ_MAX_ZONES];
	int			nzones;
	struct thread		*td;
	struct cf_level		*levels;
	int			throttle_mhz;	/* active cap, 0 = none */
	int			start_mhz;	/* level before throttling */
	int			throttle_events;
	uint64_t		throttle_steps;
	int			last_step;	/* ticks */
	int			last_up;	/* ticks of the last raise */
	int			last_down;	/* ticks of the last cut */
	int			up_ms;		/* adaptive raise interval */
	int			last_log;	/* ticks */
	int			unlogged;	/* events not yet reported */
	bool			raised;		/* last_up is valid */
	bool			logged;		/* last_log is valid */
	int			hottest;	/* millicelsius */
	bool			critical_fired;
	bool			nodata_warned;
};

/*
 * Mainline k1.dtsi zones, used when the devicetree has none for this
 * sensor: critical 115 C everywhere, passive 85 C (2 C hysteresis, 100 ms
 * polling) on the gpu and both CPU clusters.
 */
static const struct sptsen_zone sptsen_default_zones[] = {
	{ "soc",	0, TZ_NONE, 0,    115000, 100, 0, false },
	{ "package",	1, TZ_NONE, 0,    115000, 100, 0, false },
	{ "gpu",	2, 85000,   2000, 115000, 100, 0, false },
	{ "cluster0",	3, 85000,   2000, 115000, 100, 0, false },
	{ "cluster1",	4, 85000,   2000, 115000, 100, 0, false },
};

SYSCTL_NODE(_hw, OID_AUTO, spacemit_tsensor, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "SpacemiT K1 thermal protection");

static int sptsen_throttle = 1;
SYSCTL_INT(_hw_spacemit_tsensor, OID_AUTO, throttle, CTLFLAG_RWTUN,
    &sptsen_throttle, 0,
    "Lower the CPU frequency at passive trip points (0 = off)");

static int sptsen_critical_action = 0;
SYSCTL_INT(_hw_spacemit_tsensor, OID_AUTO, critical_action, CTLFLAG_RWTUN,
    &sptsen_critical_action, 0,
    "At a critical trip: 0 power off, 1 reboot, 2 halt, 3 log only");

static int sptsen_poll_ms = 1000;
SYSCTL_INT(_hw_spacemit_tsensor, OID_AUTO, poll_ms, CTLFLAG_RWTUN,
    &sptsen_poll_ms, 0, "Sensor polling interval while cool, ms");

static int sptsen_step_down_ms = 250;
SYSCTL_INT(_hw_spacemit_tsensor, OID_AUTO, step_down_ms, CTLFLAG_RWTUN,
    &sptsen_step_down_ms, 0, "Minimum time between frequency reductions, ms");

static int sptsen_step_up_ms = 1000;
SYSCTL_INT(_hw_spacemit_tsensor, OID_AUTO, step_up_ms, CTLFLAG_RWTUN,
    &sptsen_step_up_ms, 0,
    "Minimum time between frequency increases, ms; doubled (up to 30 s) "
    "while increases overshoot the trip");

static struct ofw_compat_data compat_data[] = {
	{ "spacemit,k1-tsensor",	1 },
	{ NULL,				0 }
};

#define	RD4(sc, r)	bus_read_4((sc)->mem_res, (r))
#define	WR4(sc, r, v)	bus_write_4((sc)->mem_res, (r), (v))

/*
 * Read sensor 'n' in degrees Celsius.  Returns false when the sensor has
 * not produced a plausible sample.
 */
static bool
sptsen_read_c(struct sptsen_softc *sc, int n, int *tempc)
{
	int val;

	val = (RD4(sc, TSEN_DATA(n)) >> TSEN_DATA_SHIFT(n)) & TSEN_DATA_MASK;
	if (val == 0)
		return (false);
	val -= TSEN_TEMP_OFFSET;
	if (val < TSEN_TEMP_MIN_C || val > TSEN_TEMP_MAX_C)
		return (false);
	*tempc = val;
	return (true);
}

static int
sptsen_temp_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct sptsen_softc *sc = arg1;
	int temp, tempc;

	/*
	 * Report absolute zero's sentinel (0 dK) when the sensor has no
	 * sample, so a monitoring tool can tell "no data" from a reading.
	 */
	if (sptsen_read_c(sc, arg2, &tempc))
		temp = tempc * 10 + 2732;	/* 0.1 Kelvin, IK format */
	else
		temp = 0;
	return (sysctl_handle_int(oidp, &temp, 0, req));
}

/*
 * Thermal zones.
 */
static int
sptsen_parse_zones(struct sptsen_softc *sc)
{
	phandle_t tz, zone, trips, trip;
	pcell_t cells[2], xref;
	char type[16];
	struct sptsen_zone *z;
	int32_t temp, hyst;
	uint32_t ms;
	int n;

	tz = OF_finddevice("/thermal-zones");
	if (tz == -1)
		return (0);
	xref = OF_xref_from_node(ofw_bus_get_node(sc->dev));
	n = 0;
	for (zone = OF_child(tz); zone != 0 && n < TZ_MAX_ZONES;
	    zone = OF_peer(zone)) {
		if (OF_getencprop(zone, "thermal-sensors", cells,
		    sizeof(cells)) != sizeof(cells) || cells[0] != xref ||
		    cells[1] >= TSEN_NSENSORS)
			continue;
		z = &sc->zones[n];
		memset(z, 0, sizeof(*z));
		if (OF_getprop(zone, "name", z->name, sizeof(z->name)) <= 0)
			snprintf(z->name, sizeof(z->name), "zone%d", n);
		z->name[sizeof(z->name) - 1] = '\0';
		z->sensor = cells[1];
		z->passive = z->critical = TZ_NONE;
		z->passive_ms = 100;
		if (OF_getencprop(zone, "polling-delay-passive", &ms,
		    sizeof(ms)) == sizeof(ms) && ms > 0)
			z->passive_ms = ms;
		trips = ofw_bus_find_child(zone, "trips");
		for (trip = (trips > 0) ? OF_child(trips) : 0; trip != 0;
		    trip = OF_peer(trip)) {
			if (OF_getprop(trip, "type", type, sizeof(type)) <= 0 ||
			    OF_getencprop(trip, "temperature", &temp,
			    sizeof(temp)) != sizeof(temp))
				continue;
			type[sizeof(type) - 1] = '\0';
			if (OF_getencprop(trip, "hysteresis", &hyst,
			    sizeof(hyst)) != sizeof(hyst))
				hyst = 0;
			/* The lowest passive (or hot) and critical trips. */
			if ((strcmp(type, "passive") == 0 ||
			    strcmp(type, "hot") == 0) && temp < z->passive) {
				z->passive = temp;
				z->hysteresis = hyst;
			} else if (strcmp(type, "critical") == 0 &&
			    temp < z->critical)
				z->critical = temp;
		}
		n++;
	}
	return (n);
}

static device_t
sptsen_cpufreq(void)
{
	devclass_t dc;

	if ((dc = devclass_find("cpufreq")) == NULL)
		return (NULL);
	return (devclass_get_device(dc, 0));
}

/*
 * Report throttling starting or ending, at most once a minute: close to a
 * trip the cap can start and end every few seconds.
 */
static void
sptsen_log_event(struct sptsen_softc *sc, const char *what)
{

	if (sc->logged && ticks - sc->last_log < TZ_MSTOHZ(TZ_LOG_MS)) {
		sc->unlogged++;
		return;
	}
	device_printf(sc->dev, "%d.%d C: %s", sc->hottest / 1000,
	    (sc->hottest % 1000) / 100, what);
	if (sc->unlogged > 0)
		printf(" (%d more changes since the last report)", sc->unlogged);
	printf("\n");
	sc->unlogged = 0;
	sc->last_log = ticks;
	sc->logged = true;
}

/*
 * End throttling: put the CPUs back on the level in force before it began,
 * then give up the kernel priority.  The level is set explicitly: when the
 * framework learnt the current level from an uncached driver it saved only
 * its frequency, and restoring that alone would not reach the driver.
 */
static int
sptsen_restore(struct sptsen_softc *sc, device_t cf, int n)
{
	int error, i;

	for (i = 0; i < n; i++) {
		if (sc->levels[i].total_set.freq != sc->start_mhz)
			continue;
		error = CPUFREQ_SET(cf, &sc->levels[i], CPUFREQ_PRIO_KERN);
		if (error != 0)
			return (error);
		break;
	}
	error = CPUFREQ_SET(cf, NULL, CPUFREQ_PRIO_KERN);
	return (error == ENXIO ? 0 : error);
}

/*
 * Move the cap one cpufreq level down (dir < 0) or up (dir > 0).  Raising
 * it back to the level in force before throttling releases the kernel
 * priority and restores that level.
 */
static void
sptsen_cool_step(struct sptsen_softc *sc, int dir)
{
	struct cf_level cur;
	device_t cf;
	int cap, error, i, n;

	if ((cf = sptsen_cpufreq()) == NULL)
		return;
	n = TZ_MAX_LEVELS;
	if (CPUFREQ_LEVELS(cf, sc->levels, &n) != 0 || n <= 0)
		return;

	if (sc->throttle_mhz == 0) {
		if (dir > 0)
			return;
		/* The current level is recorded, so it is restored later. */
		if (CPUFREQ_GET(cf, &cur) == 0 &&
		    cur.total_set.freq != CPUFREQ_VAL_UNKNOWN)
			sc->start_mhz = cur.total_set.freq;
		else
			sc->start_mhz = sc->levels[0].total_set.freq;
		cap = sc->start_mhz;
	} else
		cap = sc->throttle_mhz;

	if (dir < 0) {
		/* Levels are sorted fastest first. */
		for (i = 0; i < n; i++)
			if (sc->levels[i].total_set.freq < cap)
				break;
		if (i == n) {
			if (sc->throttle_mhz == 0) {
				/* Already at the lowest level: hold it. */
				i = n - 1;
			} else
				return;
		}
		error = CPUFREQ_SET(cf, &sc->levels[i], CPUFREQ_PRIO_KERN);
		if (error != 0)
			return;
		if (sc->throttle_mhz == 0) {
			sc->throttle_events++;
			sptsen_log_event(sc, "throttling CPUs");
		}
		sc->throttle_mhz = sc->levels[i].total_set.freq;
		/*
		 * A cut soon after a raise means the raise overshot: wait
		 * longer before the next one.
		 */
		if (sc->raised && ticks - sc->last_up <
		    TZ_MSTOHZ(TZ_OVERSHOOT_MS))
			sc->up_ms = MIN(sc->up_ms * 2, TZ_UP_MAX_MS);
		sc->last_down = ticks;
	} else {
		for (i = n - 1; i >= 0; i--)
			if (sc->levels[i].total_set.freq > cap)
				break;
		if (i < 0 || sc->levels[i].total_set.freq >= sc->start_mhz) {
			if (sptsen_restore(sc, cf, n) != 0)
				return;
			sptsen_log_event(sc, "throttling ended");
			sc->throttle_mhz = 0;
		} else {
			error = CPUFREQ_SET(cf, &sc->levels[i],
			    CPUFREQ_PRIO_KERN);
			if (error != 0)
				return;
			sc->throttle_mhz = sc->levels[i].total_set.freq;
		}
		sc->last_up = ticks;
		sc->raised = true;
	}
	sc->throttle_steps++;
	sc->last_step = ticks;
}

static void
sptsen_release(struct sptsen_softc *sc)
{
	device_t cf;
	int n;

	if ((cf = sptsen_cpufreq()) == NULL)
		return;
	n = TZ_MAX_LEVELS;
	if (CPUFREQ_LEVELS(cf, sc->levels, &n) != 0 || n <= 0 ||
	    sptsen_restore(sc, cf, n) != 0)
		return;
	sptsen_log_event(sc, "throttling released");
	sc->throttle_mhz = 0;
	sc->last_step = ticks;
}

static void
sptsen_critical(struct sptsen_softc *sc, struct sptsen_zone *z, int mc)
{
	device_t cf;
	int howto, n;

	if (sc->critical_fired)
		return;
	sc->critical_fired = true;
	device_printf(sc->dev, "%s: %d.%d C reached critical trip %d C\n",
	    z->name, mc / 1000, (mc % 1000) / 100, z->critical / 1000);

	switch (sptsen_critical_action) {
	case 1:
		howto = RB_AUTOBOOT;
		break;
	case 2:
		howto = RB_HALT;
		break;
	case 3:
		device_printf(sc->dev, "critical action disabled "
		    "(hw.spacemit_tsensor.critical_action=3)\n");
		return;
	default:
		howto = RB_POWEROFF;
		break;
	}

	/* Shed as much heat as possible before shutting down. */
	n = TZ_MAX_LEVELS;
	if ((cf = sptsen_cpufreq()) != NULL &&
	    CPUFREQ_LEVELS(cf, sc->levels, &n) == 0 && n > 0)
		(void)CPUFREQ_SET(cf, &sc->levels[n - 1], CPUFREQ_PRIO_HIGHEST);

	device_printf(sc->dev, "shutting down (%s)\n",
	    howto == RB_AUTOBOOT ? "reboot" :
	    howto == RB_HALT ? "halt" : "power off");
	shutdown_nice(howto);
}

static void
sptsen_thread(void *arg)
{
	struct sptsen_softc *sc;
	struct sptsen_zone *z;
	int c, hottest, i, interval, mc, nvalid, passive_ms;
	bool cool, hot, rising;

	sc = arg;
	for (;;) {
		hot = rising = false;
		cool = true;
		nvalid = 0;
		passive_ms = INT_MAX;
		hottest = INT_MIN;
		for (i = 0; i < sc->nzones; i++) {
			z = &sc->zones[i];
			if (!sptsen_read_c(sc, z->sensor, &c)) {
				z->valid = false;
				continue;
			}
			nvalid++;
			mc = c * 1000;
			hottest = MAX(hottest, mc);
			if (z->critical != TZ_NONE && mc >= z->critical)
				sptsen_critical(sc, z, mc);
			if (z->passive != TZ_NONE) {
				passive_ms = MIN(passive_ms, z->passive_ms);
				if (mc >= z->passive) {
					hot = true;
					if (!z->valid || mc >= z->last)
						rising = true;
				}
				if (mc >= z->passive - z->hysteresis)
					cool = false;
			}
			z->last = mc;
			z->valid = true;
		}
		sc->hottest = hottest;
		if (nvalid == 0 && sc->nzones > 0) {
			/* No sensor data at all: fail towards cooler. */
			if (!sc->nodata_warned)
				device_printf(sc->dev, "no sensor data; "
				    "holding the CPUs at a low frequency\n");
			sc->nodata_warned = true;
			hot = rising = true;
			cool = false;
		}

		if (sptsen_throttle == 0) {
			if (sc->throttle_mhz != 0)
				sptsen_release(sc);
		} else if (hot && rising) {
			if (sc->throttle_mhz == 0 || ticks - sc->last_step >=
			    TZ_MSTOHZ(sptsen_step_down_ms))
				sptsen_cool_step(sc, -1);
		} else if (cool && sc->throttle_mhz != 0 &&
		    ticks - sc->last_step >= TZ_MSTOHZ(sc->up_ms))
			sptsen_cool_step(sc, 1);

		/* Adapt the raise interval; settle back after a quiet minute. */
		if (sc->up_ms < sptsen_step_up_ms ||
		    ticks - sc->last_down >= TZ_MSTOHZ(TZ_SETTLED_MS))
			sc->up_ms = sptsen_step_up_ms;

		interval = sptsen_poll_ms;
		if ((hot || sc->throttle_mhz != 0) && passive_ms != INT_MAX)
			interval = MIN(interval, passive_ms);
		tsleep(sc, 0, "k1tz", MAX(1, TZ_MSTOHZ(MAX(interval, 10))));
	}
}

static int
sptsen_hottest_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct sptsen_softc *sc = arg1;
	int temp;

	temp = (sc->hottest == INT_MIN) ? 0 : sc->hottest / 100 + 2732;
	return (sysctl_handle_int(oidp, &temp, 0, req));
}

static void
sptsen_add_zone_sysctls(struct sptsen_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *zones, *node;
	struct sptsen_zone *z;
	int i;

	ctx = device_get_sysctl_ctx(sc->dev);
	zones = SYSCTL_ADD_NODE(ctx,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev)), OID_AUTO, "zone",
	    CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, "Thermal zones");
	for (i = 0; i < sc->nzones; i++) {
		z = &sc->zones[i];
		node = SYSCTL_ADD_NODE(ctx, SYSCTL_CHILDREN(zones), OID_AUTO,
		    z->name, CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, "Thermal zone");
		SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(node), OID_AUTO, "sensor",
		    CTLFLAG_RD, &z->sensor, 0, "Sensor index");
		SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(node), OID_AUTO, "passive",
		    CTLFLAG_RW, &z->passive, 0,
		    "Passive trip, millicelsius (2147483647 = none)");
		SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(node), OID_AUTO,
		    "hysteresis", CTLFLAG_RW, &z->hysteresis, 0,
		    "Passive trip hysteresis, millicelsius");
		SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(node), OID_AUTO,
		    "critical", CTLFLAG_RW, &z->critical, 0,
		    "Critical trip, millicelsius (2147483647 = none)");
		SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(node), OID_AUTO,
		    "passive_ms", CTLFLAG_RW, &z->passive_ms, 0,
		    "Polling interval while hot, ms");
	}
}

static int
sptsen_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "SpacemiT K1 thermal sensor");
	return (BUS_PROBE_DEFAULT);
}

static int
sptsen_attach(device_t dev)
{
	struct sptsen_softc *sc;
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;
	clk_t clk;
	hwreset_t rst;
	uint32_t val;
	int i, rid;

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
	 * Deassert the sensor's reset BEFORE touching its registers.  Without
	 * this the ADC never runs and the data registers read 0.  Mainline
	 * Linux does this first in probe (reset_control_get_exclusive_
	 * deasserted), so we do too.
	 */
	if (hwreset_get_by_ofw_idx(dev, 0, 0, &rst) == 0) {
		if (hwreset_deassert(rst) != 0)
			device_printf(dev, "warning: cannot deassert reset\n");
	} else if (bootverbose)
		device_printf(dev, "no reset control\n");

	/* Enable the sensor's clocks (core + bus; best effort). */
	for (i = 0; clk_get_by_ofw_index(dev, 0, i, &clk) == 0; i++) {
		if (clk_enable(clk) != 0)
			device_printf(dev, "warning: cannot enable clock %d\n",
			    i);
	}

	/* Disable interrupts (polled reads only). */
	WR4(sc, TSEN_INT_EN, 0xffffffff);

	/* ADC sampling time / filter period. */
	val = RD4(sc, TSEN_TIME) & ~TSEN_TIME_MASK;
	val |= TSEN_TIME_FILTER_PER | TSEN_TIME_ADC_CNT_RST |
	    TSEN_TIME_WAIT_REF;
	WR4(sc, TSEN_TIME, val);

	/* Hardware auto mode, temp mode, raw select, power up. */
	val = RD4(sc, TSEN_PCTRL);
	val &= ~(TSEN_PCTRL_SW_CTRL | TSEN_PCTRL_CTUNE);
	val |= TSEN_PCTRL_RAW_SEL | TSEN_PCTRL_TEMP_MODE |
	    TSEN_PCTRL_HW_AUTO | TSEN_PCTRL_ENABLE;
	WR4(sc, TSEN_PCTRL, val);

	/*
	 * Enable all five sensors.  The CPU cluster sensors (3, 4) are the
	 * ones SpacemiT's own kernel uses for its trip points; reading only
	 * sensor 0 hides the hottest part of the die.
	 */
	WR4(sc, TSEN_EN, RD4(sc, TSEN_EN) | ((1u << TSEN_NSENSORS) - 1));

	/*
	 * Give the hardware auto-mode conversion time to produce a first
	 * sample before anyone reads the data registers (a few ADC periods).
	 */
	DELAY(2000);

	ctx = device_get_sysctl_ctx(dev);
	tree = device_get_sysctl_tree(dev);
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "temperature", CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    sptsen_temp_sysctl, "IK", "SoC temperature (sensor 0)");
	for (i = 0; i < TSEN_NSENSORS; i++) {
		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    sptsen_names[i], CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE,
		    sc, i, sptsen_temp_sysctl, "IK", "Sensor temperature");
	}

	sc->nzones = sptsen_parse_zones(sc);
	if (sc->nzones == 0) {
		memcpy(sc->zones, sptsen_default_zones,
		    sizeof(sptsen_default_zones));
		sc->nzones = nitems(sptsen_default_zones);
	}
	for (i = 0; i < sc->nzones; i++)
		device_printf(dev, "zone %s: sensor %d, passive %d C, "
		    "critical %d C\n", sc->zones[i].name, sc->zones[i].sensor,
		    sc->zones[i].passive == TZ_NONE ? -1 :
		    sc->zones[i].passive / 1000,
		    sc->zones[i].critical == TZ_NONE ? -1 :
		    sc->zones[i].critical / 1000);
	sptsen_add_zone_sysctls(sc);
	sc->hottest = INT_MIN;
	sc->up_ms = sptsen_step_up_ms;
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "hottest",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    sptsen_hottest_sysctl, "IK", "Hottest zone at the last poll");
	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "throttle_mhz",
	    CTLFLAG_RD, &sc->throttle_mhz, 0,
	    "CPU frequency cap in force, MHz (0 = not throttling)");
	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "throttle_events", CTLFLAG_RD, &sc->throttle_events, 0,
	    "Times throttling started");
	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "raise_interval_ms", CTLFLAG_RD, &sc->up_ms, 0,
	    "Current adaptive interval between frequency increases, ms");
	SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "throttle_steps", CTLFLAG_RD, &sc->throttle_steps, 0,
	    "Frequency cap changes");

	sc->levels = malloc(TZ_MAX_LEVELS * sizeof(*sc->levels), M_DEVBUF,
	    M_WAITOK | M_ZERO);
	if (kthread_add(sptsen_thread, sc, NULL, &sc->td, 0, 0, "%s",
	    device_get_nameunit(dev)) != 0)
		device_printf(dev, "cannot start the thermal thread; "
		    "no thermal protection\n");
	return (0);
}

static int
sptsen_detach(device_t dev)
{

	return (EBUSY);
}

static device_method_t sptsen_methods[] = {
	DEVMETHOD(device_probe,		sptsen_probe),
	DEVMETHOD(device_attach,	sptsen_attach),
	DEVMETHOD(device_detach,	sptsen_detach),
	DEVMETHOD_END
};

static driver_t sptsen_driver = {
	"spacemit_tsensor",
	sptsen_methods,
	sizeof(struct sptsen_softc),
};

DRIVER_MODULE(spacemit_tsensor, simplebus, sptsen_driver, 0, 0);
