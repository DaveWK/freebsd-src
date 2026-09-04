/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Daniel Shue <dgshue@gmail.com>
 *
 * Thermal sensor driver for the SpacemiT K1 (Ky X1) RISC-V SoC
 * (compatible: spacemit,k1-tsensor).  Exposes the on-die temperature via
 * a sysctl (dev.spacemit_tsensor.<unit>.temperature, in 0.1 Kelvin).
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
#define	TSEN_DATA0		0x20
#define	 TSEN_DATA_LOW_MASK	0xffff

#define	TSEN_TEMP_OFFSET	278	/* raw - 278 = degrees C */

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

static int
sptsen_temp_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct sptsen_softc *sc = arg1;
	int temp, val;

	/* Sensor 0 (primary) is in the low half-word of DATA0. */
	val = RD4(sc, TSEN_DATA0) & TSEN_DATA_LOW_MASK;
	/*
	 * A raw value of 0 means the ADC has not produced a sample (e.g. the
	 * sensor is held in reset or clock-gated).  Report absolute zero's
	 * sentinel (0 dK) rather than a nonsensical sub-freezing temperature
	 * so a monitoring tool can tell "no data" from a real reading.
	 */
	if (val == 0)
		temp = 0;
	else
		/* Convert to 0.1 Kelvin for the IK sysctl format. */
		temp = (val - TSEN_TEMP_OFFSET) * 10 + 2732;
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
	 * this the ADC never runs and TSEN_DATA0 reads 0 (the driver then
	 * reports a nonsensical constant temperature).  Mainline Linux does
	 * this first in probe (reset_control_get_exclusive_deasserted), so we
	 * do too.
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

	/* Enable sensor 0 (primary). */
	WR4(sc, TSEN_EN, RD4(sc, TSEN_EN) | 0x1);

	/*
	 * Give the hardware auto-mode conversion time to produce a first
	 * sample before anyone reads DATA0 (a few ADC periods).
	 */
	DELAY(2000);

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "temperature", CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    sptsen_temp_sysctl, "IK", "SoC temperature");

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
