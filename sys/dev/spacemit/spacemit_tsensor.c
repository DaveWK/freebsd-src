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
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>

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

struct sptsen_softc {
	device_t	dev;
	struct resource	*mem_res;
};

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

	if (bootverbose)
		device_printf(dev, "temperature reporting enabled\n");
	return (0);
}

static device_method_t sptsen_methods[] = {
	DEVMETHOD(device_probe,		sptsen_probe),
	DEVMETHOD(device_attach,	sptsen_attach),
	DEVMETHOD_END
};

static driver_t sptsen_driver = {
	"spacemit_tsensor",
	sptsen_methods,
	sizeof(struct sptsen_softc),
};

DRIVER_MODULE(spacemit_tsensor, simplebus, sptsen_driver, 0, 0);
