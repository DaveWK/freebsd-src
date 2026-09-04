/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Daniel Shue <dgshue@gmail.com>
 *
 * Pin mux (pinctrl) driver for the SpacemiT K1 (Ky X1) RISC-V SoC
 * (compatible: spacemit,k1-pinctrl).  Register semantics and the pin->
 * register offset mapping follow the mainline Linux driver
 * (drivers/pinctrl/spacemit/pinctrl-k1.c).
 *
 * Each pad has a 32-bit MFPR register; the low 3 bits select the mux
 * function.  Devicetree pin groups carry a "pinmux" array where each
 * entry is (pin << 16) | function.  We apply the whole tree at attach so
 * that peripherals whose pins the bootloader did not configure (e.g. the
 * second EMAC) come up muxed before their drivers attach.
 *
 * Pad configuration (bias / drive strength) is normally left at the
 * reset/firmware values: most groups in our devicetree carry pinconf
 * properties inherited from the mainline k1.dtsi that have never been
 * applied on this port, and blanket-applying them would perturb pads the
 * firmware already configured for peripherals that demonstrably work
 * (root SD, EMACs).  A pin group node opts in to full pinconf with the
 * FreeBSD-local boolean property "spacemit,apply-pinconf"; for such
 * nodes the generic properties bias-pull-up / bias-pull-down /
 * bias-disable, drive-strength (mA) and power-source (1800/3300) are
 * translated to MFPR bits exactly like mainline Linux
 * drivers/pinctrl/spacemit/pinctrl-k1.c does (same drive-strength mA
 * tables), and the edge-detect field is normalized to "none" like the
 * vendor X1_PADCONF values do for every functional (non-GPIO-IRQ) pad.
 * First user: the mmc2 pads (GPIO_15..20) of the AP6256 SDIO WiFi
 * module, which the vendor pinctrl_mmc2 group sets to
 * MUX_MODE1 | EDGE_NONE | PULL_UP | PAD_1V8_DS2 (word 0xd041) while our
 * firmware does not initialize them at all.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sbuf.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/fdt/fdt_pinctrl.h>

#include <dev/clk/clk.h>

#include "fdt_pinctrl_if.h"

/*
 * MFPR bit layout (K1; identical in mainline pinctrl-k1.c and the vendor
 * dt-bindings/pinctrl/x1-pinctrl.h):
 *   [2:0]   mux function
 *   [3]     strong pull ("SPU")
 *   [5:4]   edge detect rise/fall
 *   [6]     edge detect none/clear
 *   [7]     slew rate enable
 *   [9:8]   schmitt trigger threshold
 *   [12:10] drive strength (1.8V pads effectively use [12:11])
 *   [15:13] pull: bit15 = pull enable, bit14 = pull-up, bit13 = pull-down
 */
#define	PAD_MUX_MASK	0x7u
#define	PAD_STRONG_PULL	(1u << 3)
#define	PAD_EDGE_RISE	(1u << 4)
#define	PAD_EDGE_FALL	(1u << 5)
#define	PAD_EDGE_NONE	(1u << 6)
#define	PAD_DRIVE_MASK	(0x7u << 10)
#define	PAD_DRIVE_SHIFT	10
#define	PAD_PULLDOWN	(1u << 13)
#define	PAD_PULLUP	(1u << 14)
#define	PAD_PULL_EN	(1u << 15)
#define	PAD_PULL_MASK	(PAD_PULL_EN | PAD_PULLUP | PAD_PULLDOWN)

/*
 * drive-strength (mA) -> MFPR drive-field value, K1 tables from mainline
 * pinctrl-k1.c (k1_drive_conf).  Selection rule is mainline's
 * spacemit_get_ds_value(): first entry whose mA is >= the requested value.
 */
struct sppinctrl_ds {
	uint8_t		value;
	uint8_t		mA;
};

static const struct sppinctrl_ds sppinctrl_ds_1v8[] = {
	{ 0, 11 }, { 2, 21 }, { 4, 32 }, { 6, 42 },
};

static const struct sppinctrl_ds sppinctrl_ds_3v3[] = {
	{ 0, 7 }, { 2, 10 }, { 4, 13 }, { 6, 16 },
	{ 1, 19 }, { 3, 23 }, { 5, 26 }, { 7, 29 },
};

struct sppinctrl_softc {
	device_t	dev;
	struct mtx	mtx;
	struct resource	*mem_res;
};

static struct ofw_compat_data compat_data[] = {
	{ "spacemit,k1-pinctrl",	1 },
	{ NULL,				0 }
};

#define	RD4(sc, r)	bus_read_4((sc)->mem_res, (r))
#define	WR4(sc, r, v)	bus_write_4((sc)->mem_res, (r), (v))

/* Map a pin number to its MFPR register offset (K1 layout). */
static unsigned int
k1_pin_to_offset(unsigned int pin)
{
	unsigned int off = 0;

	if (pin <= 85)
		off = pin + 1;
	else if (pin <= 92)
		off = pin + 37;
	else if (pin <= 97)
		off = pin + 24;
	else if (pin == 98)
		off = 93;
	else if (pin == 99)
		off = 92;
	else if (pin == 100)
		off = 91;
	else if (pin == 101)
		off = 90;
	else if (pin == 102)
		off = 95;
	else if (pin == 103)
		off = 94;
	else if (pin <= 110)
		off = pin + 6;
	else if (pin <= 127)
		off = pin + 20;
	return (off << 2);
}

static uint8_t
sppinctrl_ds_value(const struct sppinctrl_ds *tbl, int n, uint32_t mA)
{
	int i;

	for (i = 0; i < n; i++) {
		if (mA <= tbl[i].mA)
			return (tbl[i].value);
	}
	return (tbl[n - 1].value);
}

/*
 * Parse the generic pinconf properties of an opted-in pin group node into
 * a (mask, value) pair to fold into each pad's MFPR.  Returns the mask of
 * MFPR bits to replace (0 = nothing to apply).
 */
static uint32_t
sppinctrl_parse_pinconf(phandle_t node, uint32_t *valp)
{
	uint32_t mask, val, mA, mV, arg;
	const struct sppinctrl_ds *tbl;
	int ntbl;

	/*
	 * Always define *valp: the caller folds it into every pad write as
	 * (reg & ~mask) | val even when mask == 0.  Returning with *valp
	 * untouched here left it uninitialized stack garbage that got OR'd
	 * into the MFPR of EVERY non-opted-in pin group applied at attach
	 * (UART/EMAC/SD pads included), remuxing/reconfiguring random pads
	 * and wedging the boot right after the pinctrl announce line.
	 */
	*valp = 0;

	if (!OF_hasprop(node, "spacemit,apply-pinconf"))
		return (0);

	mask = 0;
	val = 0;

	if (OF_hasprop(node, "bias-pull-up")) {
		mask |= PAD_PULL_MASK | PAD_STRONG_PULL;
		val |= PAD_PULL_EN | PAD_PULLUP;
		/*
		 * Mainline semantics: bias-pull-up = <1> selects the
		 * strong (low-impedance) pull-up, any other argument the
		 * normal one.  An argument-less boolean counts as 1.
		 */
		arg = 1;
		(void)OF_getencprop(node, "bias-pull-up", &arg, sizeof(arg));
		if (arg == 1)
			val |= PAD_STRONG_PULL;
	} else if (OF_hasprop(node, "bias-pull-down")) {
		mask |= PAD_PULL_MASK | PAD_STRONG_PULL;
		val |= PAD_PULL_EN | PAD_PULLDOWN;
	} else if (OF_hasprop(node, "bias-disable")) {
		mask |= PAD_PULL_MASK | PAD_STRONG_PULL;
	}

	if (OF_getencprop(node, "drive-strength", &mA, sizeof(mA)) > 0) {
		/*
		 * The drive field encoding depends on the pad voltage
		 * class; power-source selects it (1800 or 3300), matching
		 * the mainline binding.  Without power-source assume 3.3V.
		 */
		mV = 3300;
		(void)OF_getencprop(node, "power-source", &mV, sizeof(mV));
		if (mV == 1800) {
			tbl = sppinctrl_ds_1v8;
			ntbl = nitems(sppinctrl_ds_1v8);
		} else {
			tbl = sppinctrl_ds_3v3;
			ntbl = nitems(sppinctrl_ds_3v3);
		}
		mask |= PAD_DRIVE_MASK;
		val |= (uint32_t)sppinctrl_ds_value(tbl, ntbl, mA) <<
		    PAD_DRIVE_SHIFT;
	}

	/*
	 * A group being configured for a peripheral function does not use
	 * the pad's GPIO edge detector; normalize it to "none" the way
	 * every vendor X1_PADCONF functional-pad value does (EDGE_NONE set,
	 * rise/fall clear).
	 */
	if (mask != 0) {
		mask |= PAD_EDGE_NONE | PAD_EDGE_RISE | PAD_EDGE_FALL;
		val |= PAD_EDGE_NONE;
	}

	*valp = val;
	return (mask);
}

static void
sppinctrl_apply_pinmux(struct sppinctrl_softc *sc, phandle_t node)
{
	pcell_t *pinmux;
	uint32_t reg, val, cfg_mask, cfg_val;
	unsigned int pin, mux, off;
	int i, n;

	n = OF_getencprop_alloc_multi(node, "pinmux", sizeof(pcell_t),
	    (void **)&pinmux);
	if (n <= 0)
		return;

	cfg_mask = sppinctrl_parse_pinconf(node, &cfg_val);

	mtx_lock(&sc->mtx);
	for (i = 0; i < n; i++) {
		pin = pinmux[i] >> 16;
		mux = pinmux[i] & PAD_MUX_MASK;
		if (pin > 127)
			continue;
		off = k1_pin_to_offset(pin);
		reg = RD4(sc, off);
		val = (reg & ~PAD_MUX_MASK) | mux;
		/* Only ever change bits covered by the pinconf mask. */
		val = (val & ~cfg_mask) | (cfg_val & cfg_mask);
		if (val != reg)
			WR4(sc, off, val);
	}
	mtx_unlock(&sc->mtx);
	OF_prop_free(pinmux);
}

/*
 * Debug sysctl: dev.spacemit_pinctrl.<unit>.pins
 *
 * Live dump of every pad's MFPR word so the applied mux *and* pad config
 * (pull/drive/edge) can be verified on hardware; e.g. the AP6256 SDIO
 * pads GPIO_15..20 must read 0xd041 (MUX1 | EDGE_NONE | PULL_UP |
 * 1.8V-DS2) after boot for the WiFi bring-up.
 */
static int
sppinctrl_sysctl_pins(SYSCTL_HANDLER_ARGS)
{
	struct sppinctrl_softc *sc = arg1;
	struct sbuf sb;
	int error, pin;

	sbuf_new_for_sysctl(&sb, NULL, 128 * 16, req);
	for (pin = 0; pin <= 127; pin++) {
		sbuf_printf(&sb, "%3d:%04x%s", pin,
		    RD4(sc, k1_pin_to_offset(pin)),
		    (pin % 8) == 7 ? "\n" : " ");
	}
	error = sbuf_finish(&sb);
	sbuf_delete(&sb);
	return (error);
}

static int
sppinctrl_configure(device_t dev, phandle_t cfgxref)
{
	struct sppinctrl_softc *sc = device_get_softc(dev);
	phandle_t node, child;

	node = OF_node_from_xref(cfgxref);

	/* The pinmux array may be on this node or on child pin subnodes. */
	if (OF_hasprop(node, "pinmux"))
		sppinctrl_apply_pinmux(sc, node);
	for (child = OF_child(node); child != 0; child = OF_peer(child)) {
		if (OF_hasprop(child, "pinmux"))
			sppinctrl_apply_pinmux(sc, child);
	}
	return (0);
}

static int
sppinctrl_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "SpacemiT K1 pinctrl");
	return (BUS_PROBE_DEFAULT);
}

static int
sppinctrl_attach(device_t dev)
{
	struct sppinctrl_softc *sc = device_get_softc(dev);
	clk_t clk;
	int i, rid;

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

	/* Enable the block's clocks if present (best effort). */
	for (i = 0; clk_get_by_ofw_index(dev, 0, i, &clk) == 0; i++)
		(void)clk_enable(clk);

	/*
	 * Register and apply the whole pin-config tree now, before the
	 * peripheral drivers (e.g. the EMACs) attach in the default pass.
	 */
	fdt_pinctrl_register(dev, NULL);
	fdt_pinctrl_configure_tree(dev);

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "pins", CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, sppinctrl_sysctl_pins, "A",
	    "Live MFPR (mux + pad config) dump for pins 0..127");
	return (0);
}

static device_method_t sppinctrl_methods[] = {
	DEVMETHOD(device_probe,		sppinctrl_probe),
	DEVMETHOD(device_attach,	sppinctrl_attach),

	/* fdt_pinctrl interface */
	DEVMETHOD(fdt_pinctrl_configure, sppinctrl_configure),

	DEVMETHOD_END
};

static driver_t sppinctrl_driver = {
	"spacemit_pinctrl",
	sppinctrl_methods,
	sizeof(struct sppinctrl_softc),
};

EARLY_DRIVER_MODULE(spacemit_pinctrl, simplebus, sppinctrl_driver, 0, 0,
    BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_EARLY);
