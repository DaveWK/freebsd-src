/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Daniel Shue <dgshue@gmail.com>
 *
 * GPIO controller driver for the SpacemiT K1 (Ky X1) RISC-V SoC
 * (compatible: spacemit,k1-gpio).  Register semantics from the OpenBSD
 * smtgpio(4) driver and the mainline Linux gpio-spacemit-k1 binding.
 *
 * The controller has 4 banks of 32 pins.  Each register type (level,
 * direction, set, clear) has one 32-bit word per bank; banks 0-2 are at
 * word offsets 0x0/0x4/0x8 and bank 3 is at a separate 0x100 window.
 * Pins are numbered linearly here as bank*32 + offset.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "gpio_if.h"

#define	SPGPIO_NBANKS	4
#define	SPGPIO_NPINS	(SPGPIO_NBANKS * 32)

#define	GPIO_PLR	0x00	/* pin level (read) */
#define	GPIO_PDR	0x0c	/* pin direction (1 = output) */
#define	GPIO_PSR	0x18	/* pin set (write 1 to drive high) */
#define	GPIO_PCR	0x24	/* pin clear (write 1 to drive low) */

#define	SPGPIO_CAPS	(GPIO_PIN_INPUT | GPIO_PIN_OUTPUT)

struct spgpio_softc {
	device_t		dev;
	device_t		busdev;
	struct mtx		mtx;
	struct resource		*mem_res;
	struct gpio_pin		pins[SPGPIO_NPINS];
};

static struct ofw_compat_data compat_data[] = {
	{ "spacemit,k1-gpio",	1 },
	{ NULL,			0 }
};

#define	SPGPIO_LOCK(sc)		mtx_lock_spin(&(sc)->mtx)
#define	SPGPIO_UNLOCK(sc)	mtx_unlock_spin(&(sc)->mtx)
#define	RD4(sc, r)		bus_read_4((sc)->mem_res, (r))
#define	WR4(sc, r, v)		bus_write_4((sc)->mem_res, (r), (v))

/* Byte offset of a bank's register block. */
static bus_size_t
spgpio_bank_off(uint32_t bank)
{
	switch (bank) {
	case 0: return (0x0000);
	case 1: return (0x0004);
	case 2: return (0x0008);
	case 3: return (0x0100);
	}
	return ((bus_size_t)-1);
}

static device_t
spgpio_get_bus(device_t dev)
{
	struct spgpio_softc *sc = device_get_softc(dev);

	return (sc->busdev);
}

static int
spgpio_pin_max(device_t dev, int *maxpin)
{

	*maxpin = SPGPIO_NPINS - 1;
	return (0);
}

static int
spgpio_pin_getname(device_t dev, uint32_t pin, char *name)
{

	if (pin >= SPGPIO_NPINS)
		return (EINVAL);
	snprintf(name, GPIOMAXNAME, "gpio%u_%u", pin / 32, pin % 32);
	return (0);
}

static int
spgpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{

	if (pin >= SPGPIO_NPINS)
		return (EINVAL);
	*caps = SPGPIO_CAPS;
	return (0);
}

static int
spgpio_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct spgpio_softc *sc = device_get_softc(dev);

	if (pin >= SPGPIO_NPINS)
		return (EINVAL);
	SPGPIO_LOCK(sc);
	*flags = sc->pins[pin].gp_flags;
	SPGPIO_UNLOCK(sc);
	return (0);
}

static int
spgpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct spgpio_softc *sc = device_get_softc(dev);
	bus_size_t off;
	uint32_t bit, dir;

	if (pin >= SPGPIO_NPINS)
		return (EINVAL);
	off = spgpio_bank_off(pin / 32);
	bit = 1u << (pin % 32);

	SPGPIO_LOCK(sc);
	dir = RD4(sc, off + GPIO_PDR);
	if (flags & GPIO_PIN_OUTPUT)
		dir |= bit;
	else if (flags & GPIO_PIN_INPUT)
		dir &= ~bit;
	WR4(sc, off + GPIO_PDR, dir);
	sc->pins[pin].gp_flags = flags & SPGPIO_CAPS;
	SPGPIO_UNLOCK(sc);
	return (0);
}

static int
spgpio_pin_get(device_t dev, uint32_t pin, unsigned int *val)
{
	struct spgpio_softc *sc = device_get_softc(dev);
	bus_size_t off;
	uint32_t bit, reg;

	if (pin >= SPGPIO_NPINS)
		return (EINVAL);
	off = spgpio_bank_off(pin / 32);
	bit = 1u << (pin % 32);

	SPGPIO_LOCK(sc);
	reg = RD4(sc, off + GPIO_PLR);
	*val = (reg & bit) ? 1 : 0;
	SPGPIO_UNLOCK(sc);
	return (0);
}

static int
spgpio_pin_set(device_t dev, uint32_t pin, unsigned int val)
{
	struct spgpio_softc *sc = device_get_softc(dev);
	bus_size_t off;
	uint32_t bit;

	if (pin >= SPGPIO_NPINS)
		return (EINVAL);
	off = spgpio_bank_off(pin / 32);
	bit = 1u << (pin % 32);

	SPGPIO_LOCK(sc);
	/* Set and clear are separate write-1 registers. */
	WR4(sc, off + (val ? GPIO_PSR : GPIO_PCR), bit);
	SPGPIO_UNLOCK(sc);
	return (0);
}

static int
spgpio_pin_toggle(device_t dev, uint32_t pin)
{
	struct spgpio_softc *sc = device_get_softc(dev);
	bus_size_t off;
	uint32_t bit, reg;

	if (pin >= SPGPIO_NPINS)
		return (EINVAL);
	off = spgpio_bank_off(pin / 32);
	bit = 1u << (pin % 32);

	SPGPIO_LOCK(sc);
	reg = RD4(sc, off + GPIO_PLR);
	WR4(sc, off + ((reg & bit) ? GPIO_PCR : GPIO_PSR), bit);
	SPGPIO_UNLOCK(sc);
	return (0);
}

/* Devicetree 3-cell mapping: <bank offset flags>; flag bit0 = active low. */
static int
spgpio_map_gpios(device_t bus, phandle_t dev, phandle_t gparent, int gcells,
    pcell_t *gpios, uint32_t *pin, uint32_t *flags)
{

	if (gcells != 3)
		return (EINVAL);
	if (gpios[0] >= SPGPIO_NBANKS || gpios[1] >= 32)
		return (EINVAL);
	*pin = gpios[0] * 32 + gpios[1];
	*flags = gpios[2];
	return (0);
}

static int
spgpio_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "SpacemiT K1 GPIO controller");
	return (BUS_PROBE_DEFAULT);
}

static int
spgpio_attach(device_t dev)
{
	struct spgpio_softc *sc = device_get_softc(dev);
	int i, rid;

	sc->dev = dev;
	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_SPIN);

	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "cannot allocate registers\n");
		mtx_destroy(&sc->mtx);
		return (ENXIO);
	}

	for (i = 0; i < SPGPIO_NPINS; i++) {
		sc->pins[i].gp_pin = i;
		sc->pins[i].gp_caps = SPGPIO_CAPS;
		sc->pins[i].gp_flags =
		    (RD4(sc, spgpio_bank_off(i / 32) + GPIO_PDR) &
		    (1u << (i % 32))) ? GPIO_PIN_OUTPUT : GPIO_PIN_INPUT;
	}

	/* Register the OFW xref so phandle lookups (reset-gpios) resolve. */
	OF_device_register_xref(OF_xref_from_node(ofw_bus_get_node(dev)), dev);

	sc->busdev = gpiobus_add_bus(dev);
	if (sc->busdev == NULL) {
		device_printf(dev, "cannot attach gpiobus\n");
		bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->mem_res);
		mtx_destroy(&sc->mtx);
		return (ENXIO);
	}
	bus_attach_children(dev);
	return (0);
}

static device_method_t spgpio_methods[] = {
	DEVMETHOD(device_probe,		spgpio_probe),
	DEVMETHOD(device_attach,	spgpio_attach),

	/* GPIO interface */
	DEVMETHOD(gpio_get_bus,		spgpio_get_bus),
	DEVMETHOD(gpio_pin_max,		spgpio_pin_max),
	DEVMETHOD(gpio_pin_getname,	spgpio_pin_getname),
	DEVMETHOD(gpio_pin_getcaps,	spgpio_pin_getcaps),
	DEVMETHOD(gpio_pin_getflags,	spgpio_pin_getflags),
	DEVMETHOD(gpio_pin_setflags,	spgpio_pin_setflags),
	DEVMETHOD(gpio_pin_get,		spgpio_pin_get),
	DEVMETHOD(gpio_pin_set,		spgpio_pin_set),
	DEVMETHOD(gpio_pin_toggle,	spgpio_pin_toggle),
	DEVMETHOD(gpio_map_gpios,	spgpio_map_gpios),

	DEVMETHOD_END
};

static driver_t spgpio_driver = {
	"gpio",
	spgpio_methods,
	sizeof(struct spgpio_softc),
};

/*
 * Attach before the default pass so the EMAC's PHY-reset lookup finds us.
 */
EARLY_DRIVER_MODULE(spacemit_gpio, simplebus, spgpio_driver, 0, 0,
    BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_LATE);
MODULE_DEPEND(spacemit_gpio, gpiobus, 1, 1, 1);
