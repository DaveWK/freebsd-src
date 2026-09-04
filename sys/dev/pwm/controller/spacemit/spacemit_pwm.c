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
 * pwmbus(4) controller driver for the SpacemiT K1 (Ky X1) PWM blocks
 * (compatible: "spacemit,k1-pwm", "marvell,pxa910-pwm").  Each block is a
 * single-channel Marvell PXA-style PWM with three registers: control
 * (prescale), duty, and period.  Useful for fan control, backlight and LEDs.
 *
 *   period_ns = 10^9 * (prescale + 1) * (period_val + 1) / clk_rate
 *   duty_ns   = 10^9 * (prescale + 1) * duty_val / clk_rate
 *
 * Register model from mainline Linux drivers/pwm/pwm-pxa.c (GPL-2.0),
 * reimplemented not copied; wrapped in FreeBSD's pwmbus framework modeled on
 * dev/pwm/controller/allwinner/aw_pwm.c.
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

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/pwm/pwmbus.h>

#include "pwmbus_if.h"

#define	PWMCR		0x00		/* control (prescale) */
#define	PWMDCR		0x04		/* duty cycle */
#define	PWMPCR		0x08		/* period */

#define	PWMCR_SD	(1u << 6)	/* stop on next period */
#define	PWMDCR_FD	(1u << 10)	/* full duty */

#define	PWM_PRESCALE_MAX	63
#define	PWM_PV_MAX	1023		/* 10-bit period value */

#define	NS_PER_SEC	1000000000ULL

struct spacemit_pwm_softc {
	device_t	dev;
	device_t	busdev;
	struct resource	*mem;
	int		mem_rid;
	clk_t		clk;
	hwreset_t	reset;
	uint64_t	clk_freq;
	u_int		period;		/* cached, ns */
	u_int		duty;		/* cached, ns */
	bool		enabled;
};

static struct ofw_compat_data compat_data[] = {
	{ "spacemit,k1-pwm",		1 },
	{ "marvell,pxa910-pwm",		1 },
	{ NULL,				0 }
};

#define	RD4(sc, r)	bus_read_4((sc)->mem, (r))
#define	WR4(sc, r, v)	bus_write_4((sc)->mem, (r), (v))

static int
spacemit_pwm_channel_count(device_t dev, u_int *nchannel)
{

	*nchannel = 1;
	return (0);
}

static int
spacemit_pwm_channel_config(device_t dev, u_int channel, u_int period_ns,
    u_int duty_ns)
{
	struct spacemit_pwm_softc *sc;
	uint64_t period_cycles, prescale, pv, dc;

	if (channel != 0)
		return (EINVAL);
	sc = device_get_softc(dev);

	/* period_cycles = clk_rate * period_ns / 1e9 */
	period_cycles = (sc->clk_freq * period_ns) / NS_PER_SEC;
	if (period_cycles < 1)
		period_cycles = 1;

	prescale = (period_cycles - 1) / (PWM_PV_MAX + 1);
	if (prescale > PWM_PRESCALE_MAX)
		return (EINVAL);

	pv = period_cycles / (prescale + 1);
	if (pv < 1)
		pv = 1;
	pv -= 1;

	if (duty_ns == period_ns)
		dc = PWMDCR_FD;
	else
		dc = ((pv + 1) * duty_ns) / period_ns;

	/*
	 * PWMCR holds the prescale; set SD so a stopped PWM ends cleanly at
	 * the next period boundary (matches mainline).
	 */
	WR4(sc, PWMCR, (uint32_t)(prescale | PWMCR_SD));
	WR4(sc, PWMDCR, (uint32_t)dc);
	WR4(sc, PWMPCR, (uint32_t)pv);

	sc->period = period_ns;
	sc->duty = duty_ns;
	return (0);
}

static int
spacemit_pwm_channel_get_config(device_t dev, u_int channel, u_int *period_ns,
    u_int *duty_ns)
{
	struct spacemit_pwm_softc *sc;

	if (channel != 0)
		return (EINVAL);
	sc = device_get_softc(dev);
	*period_ns = sc->period;
	*duty_ns = sc->duty;
	return (0);
}

static int
spacemit_pwm_channel_enable(device_t dev, u_int channel, bool enable)
{
	struct spacemit_pwm_softc *sc;

	if (channel != 0)
		return (EINVAL);
	sc = device_get_softc(dev);

	if (enable == sc->enabled)
		return (0);

	if (enable) {
		/* Re-apply cached config; clearing SD lets the PWM run. */
		WR4(sc, PWMCR, RD4(sc, PWMCR) & ~PWMCR_SD);
	} else {
		WR4(sc, PWMCR, RD4(sc, PWMCR) | PWMCR_SD);
	}
	sc->enabled = enable;
	return (0);
}

static int
spacemit_pwm_channel_is_enabled(device_t dev, u_int channel, bool *enabled)
{
	struct spacemit_pwm_softc *sc;

	if (channel != 0)
		return (EINVAL);
	sc = device_get_softc(dev);
	*enabled = sc->enabled;
	return (0);
}

static int
spacemit_pwm_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "SpacemiT K1 PWM");
	return (BUS_PROBE_DEFAULT);
}

static int
spacemit_pwm_attach(device_t dev)
{
	struct spacemit_pwm_softc *sc;

	sc = device_get_softc(dev);
	sc->dev = dev;

	sc->mem_rid = 0;
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid,
	    RF_ACTIVE);
	if (sc->mem == NULL) {
		device_printf(dev, "cannot allocate memory resource\n");
		return (ENXIO);
	}

	if (clk_get_by_ofw_index(dev, 0, 0, &sc->clk) != 0 ||
	    clk_enable(sc->clk) != 0) {
		device_printf(dev, "cannot enable clock\n");
		goto fail;
	}
	if (clk_get_freq(sc->clk, &sc->clk_freq) != 0 || sc->clk_freq == 0) {
		device_printf(dev, "cannot get clock frequency\n");
		goto fail;
	}

	if (hwreset_get_by_ofw_idx(dev, 0, 0, &sc->reset) == 0)
		(void)hwreset_deassert(sc->reset);
	else
		sc->reset = NULL;

	sc->busdev = device_add_child(dev, "pwmbus", DEVICE_UNIT_ANY);
	if (sc->busdev == NULL) {
		device_printf(dev, "cannot add pwmbus child\n");
		goto fail;
	}

	bus_attach_children(dev);
	return (0);

fail:
	if (sc->reset != NULL)
		hwreset_release(sc->reset);
	if (sc->clk != NULL)
		clk_release(sc->clk);
	bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem);
	return (ENXIO);
}

static int
spacemit_pwm_detach(device_t dev)
{
	struct spacemit_pwm_softc *sc;
	int error;

	sc = device_get_softc(dev);
	error = bus_generic_detach(dev);
	if (error != 0)
		return (error);

	if (sc->reset != NULL)
		hwreset_release(sc->reset);
	if (sc->clk != NULL)
		clk_release(sc->clk);
	bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem);
	return (0);
}

static phandle_t
spacemit_pwm_get_node(device_t bus, device_t dev)
{

	return (ofw_bus_get_node(bus));
}

static device_method_t spacemit_pwm_methods[] = {
	DEVMETHOD(device_probe,		spacemit_pwm_probe),
	DEVMETHOD(device_attach,	spacemit_pwm_attach),
	DEVMETHOD(device_detach,	spacemit_pwm_detach),

	/* OFW glue. */
	DEVMETHOD(ofw_bus_get_node,	spacemit_pwm_get_node),

	/* pwmbus interface. */
	DEVMETHOD(pwmbus_channel_count,		spacemit_pwm_channel_count),
	DEVMETHOD(pwmbus_channel_config,	spacemit_pwm_channel_config),
	DEVMETHOD(pwmbus_channel_get_config,	spacemit_pwm_channel_get_config),
	DEVMETHOD(pwmbus_channel_enable,	spacemit_pwm_channel_enable),
	DEVMETHOD(pwmbus_channel_is_enabled,	spacemit_pwm_channel_is_enabled),

	DEVMETHOD_END
};

static driver_t spacemit_pwm_driver = {
	"spacemit_pwm",
	spacemit_pwm_methods,
	sizeof(struct spacemit_pwm_softc)
};

DRIVER_MODULE(spacemit_pwm, simplebus, spacemit_pwm_driver, NULL, NULL);
MODULE_DEPEND(spacemit_pwm, pwmbus, 1, 1, 1);
MODULE_VERSION(spacemit_pwm, 1);
