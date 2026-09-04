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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Clock and reset controller driver for the SpacemiT K1 (aka Ky X1) SoC,
 * covering the APBC and APMU system-controller register banks.
 *
 * Register layout derived from the devicetree bindings
 * (dt-bindings/clock/spacemit,k1-syscon.h) and the documentation of the
 * mainline Linux drivers (drivers/clk/spacemit/ccu-k1.c,
 * drivers/reset/spacemit/reset-spacemit-k1.c); driver structure follows
 * FreeBSD's jh7110 clock drivers and OpenBSD's smtclock(4).
 *
 * This is a fixed-configuration provider: it reports rates as programmed
 * by the boot firmware and implements gate/reset control.  It does not
 * model the PLL tree or support reprogramming mux/dividers.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <machine/bus.h>

#include <dev/fdt/simplebus.h>
#include <dev/hwreset/hwreset.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/syscon/syscon.h>

#include "clkdev_if.h"
#include "hwreset_if.h"
#include "syscon_if.h"

/* Fixed parent rates (PLL1 VCO = 2457.6 MHz, dividers per mainline names). */
#define	K1_PLL1_D3	819200000UL	/* pll1_d3_819p2 */
#define	K1_PLL1_D4	614400000UL	/* pll1_d4_614p4 */
#define	K1_PLL1_D6	409600000UL	/* pll1_d6_409p6 */
#define	K1_PLL1_D11	(2457600000UL / 11)
#define	K1_PLL1_D13	(2457600000UL / 13)
#define	K1_PLL1_D23	(2457600000UL / 23)

/* PLL2 VCO = 3000 MHz; pll2_dN = 3000 MHz / N.  U-Boot parks the SDH
 * functional muxes on PLL2 parents, so they must be modelled here or the
 * SDH clock reads 0 Hz (eMMC / SDH2 then fails to enumerate). */
#define	K1_PLL2_D5	(3000000000UL / 5)	/* pll2_d5 = 600 MHz */
#define	K1_PLL2_D8	(3000000000UL / 8)	/* pll2_d8 = 375 MHz */

struct smccu_clk {
	int		id;		/* binding ID within this bank */
	const char	*name;
	uint32_t	reg;
	int8_t		gate_shift;	/* -1 = no gate */
	int8_t		mux_shift;	/* -1 = no mux */
	int8_t		mux_width;
	int8_t		div_shift;	/* -1 = no divider */
	int8_t		div_width;
	const uint64_t	*prates;	/* parent rates, indexed by mux */
	int		nrates;
	uint64_t	rate;		/* fixed rate when no mux (0 = unknown) */
	uint32_t	gate_mask;	/* if nonzero, full enable mask (overrides
					   gate_shift; some blocks need >1 bit) */
	bool		wo_reg;		/* register reads as 0; use the shadow */
	bool		settable;	/* clknode_set_freq may reprogram the mux */
};

struct smccu_reset {
	int		id;
	uint32_t	reg;
	uint32_t	assert_mask;
	uint32_t	deassert_mask;
};

struct smccu_bank {
	const struct smccu_clk		*clks;
	int				nclks;
	const struct smccu_reset	*resets;
	int				nresets;
};

/*
 * APBC bank (@0xd4015000).
 * UART functional clock mux (FNCLKSEL, bits [6:4]): 0 = 57.6 MHz,
 * 1 = 14.7456 MHz, 2 = 48 MHz.  Gate: bit 1 (func), bit 0 (bus).
 */
static const uint64_t k1_uart_rates[] = { 57600000, 14745600, 48000000 };

/*
 * TWSI (I2C) functional clock mux (bits [6:4], width 3): 0 = 31.5 MHz,
 * 1 = 51.2 MHz, 2 = 61.44 MHz (pll1 dividers).  Func gate bit 1, bus gate
 * bit 0, reset bit 2 -- all in the respective APBC_TWSIn_CLK_RST register.
 *
 * MEASURED ON HARDWARE (2026-08-28, R2S, /dev/mem sweep of APBC+0x38 and
 * APBC+0x20 with a userland i2c master): ONLY mux index 2 has a running
 * parent.  At index 0 and 1 the i2c block is fully register-accessible (the
 * APB *bus* clock is a separate gate, bit 0) but its state machine is frozen:
 * ICR/ILCR/IWCR read back exactly what was written, IBMR shows both lines
 * idle-high, yet CR_TB never self-clears and ISR stays 0x00000000 forever.
 * At index 2 the very same code ACKs the EEPROM at 0x50 and the PMIC at 0x41.
 * Setting every gate in PLL1_SWCR2 (0xd4090104, 0x58b98fff -> 0xffffffff) does
 * NOT revive index 0 or 1, so those two parents are not merely gated off --
 * they are not running on this board at all.  The mux therefore must be
 * programmed to 2; leaving it at its power-on value of 0 is why i2c attached
 * but could not transact.  Rate confirmed by timing a 256-byte EEPROM read at
 * three ILCR SLV values: f = 2322 bits * 2 * 352 / 26.634 ms = 61.4 MHz.
 */
static const uint64_t k1_twsi_rates[] = { 31500000, 51200000, 61440000 };

/*
 * PWM functional clock mux (bits [6:4], width 3): 0 = 12.8 MHz (pll1_d192),
 * 1 = 24 MHz (osc).  Func gate bit 1, bus gate bit 0.
 */
static const uint64_t k1_pwm_rates[] = { 12800000, 24000000 };

/*
 * SSP3 (SPI) functional clock mux (bits [6:4], width 3):
 * 0 = 6.4 MHz, 1 = 12.8 MHz, 2 = 25.6 MHz, 3 = 51.2 MHz (pll1 dividers).
 * Func gate bit 1, bus gate bit 0.
 */
static const uint64_t k1_ssp_rates[] = { 6400000, 12800000, 25600000, 51200000 };

static const struct smccu_clk k1_apbc_clks[] = {
	/* id, name,           reg,  gate, mux,w, div,w, parents */
	{ 0,  "k1_uart0",     0x00,  1,  4, 3, -1, 0, k1_uart_rates, 3, 0 },
	{ 1,  "k1_uart2",     0x04,  1,  4, 3, -1, 0, k1_uart_rates, 3, 0 },
	{ 2,  "k1_uart3",     0x24,  1,  4, 3, -1, 0, k1_uart_rates, 3, 0 },
	{ 3,  "k1_uart4",     0x70,  1,  4, 3, -1, 0, k1_uart_rates, 3, 0 },
	{ 4,  "k1_uart5",     0x74,  1,  4, 3, -1, 0, k1_uart_rates, 3, 0 },
	{ 5,  "k1_uart6",     0x78,  1,  4, 3, -1, 0, k1_uart_rates, 3, 0 },
	{ 6,  "k1_uart7",     0x94,  1,  4, 3, -1, 0, k1_uart_rates, 3, 0 },
	{ 7,  "k1_uart8",     0x98,  1,  4, 3, -1, 0, k1_uart_rates, 3, 0 },
	{ 8,  "k1_uart9",     0x9c,  1,  4, 3, -1, 0, k1_uart_rates, 3, 0 },
	{ 9,  "k1_gpio",      0x08,  1, -1, 0, -1, 0, NULL, 0, 24000000 },
	/* Thermal sensor: core func clock gate bit 1, bus clock gate bit 0. */
	{ 48, "k1_tsen",      0x6c,  1, -1, 0, -1, 0, NULL, 0, 0 },
	{ 98, "k1_tsen_bus",  0x6c,  0, -1, 0, -1, 0, NULL, 0, 0 },
	/*
	 * RTC: core func clock needs BOTH bit 7 and bit 1 set (full enable
	 * mask), bus clock gate bit 0.  Firmware leaves only bit 1 set, so the
	 * counter does not tick until we set bit 7 too.
	 */
	{ 31, "k1_rtc",       0x28, -1, -1, 0, -1, 0, NULL, 0, 0,
	    (1u << 7) | (1u << 1) },
	{ 83, "k1_rtc_bus",   0x28,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 52, "k1_uart0_bus", 0x00,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 53, "k1_uart2_bus", 0x04,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 54, "k1_uart3_bus", 0x24,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 55, "k1_uart4_bus", 0x70,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 56, "k1_uart5_bus", 0x74,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 57, "k1_uart6_bus", 0x78,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 58, "k1_uart7_bus", 0x94,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 59, "k1_uart8_bus", 0x98,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 60, "k1_uart9_bus", 0x9c,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 61, "k1_gpio_bus",  0x08,  0, -1, 0, -1, 0, NULL, 0, 0 },
	/*
	 * TWSI (I2C) func clocks: mux [6:4] w3, gate bit 1.  Register offsets
	 * per APBC_TWSIn_CLK_RST.  All are marked settable so the i2c driver's
	 * clk_set_freq() can pick the one mux index that has a live parent (2).
	 *
	 * TWSI8 (reg 0x20) has the hardware quirk that reads ALWAYS return 0
	 * (verified: write 0x23, read back 0x00000000, while the i2c8 block
	 * itself responds normally).  It is therefore marked wo_reg so gate and
	 * mux updates compose through the driver-side shadow instead of a
	 * read-modify-write that would read 0 and wipe the other field.  It has
	 * the SAME mux as every other TWSI -- mainline models it as a fixed
	 * 31.5 MHz clock, which is wrong for this SoC and left the PMIC bus
	 * unclocked.
	 */
	{ 32, "k1_twsi0",     0x2c,  1,  4, 3, -1, 0, k1_twsi_rates, 3, 0,
	    0, false, true },
	{ 33, "k1_twsi1",     0x30,  1,  4, 3, -1, 0, k1_twsi_rates, 3, 0,
	    0, false, true },
	{ 34, "k1_twsi2",     0x38,  1,  4, 3, -1, 0, k1_twsi_rates, 3, 0,
	    0, false, true },
	{ 35, "k1_twsi4",     0x40,  1,  4, 3, -1, 0, k1_twsi_rates, 3, 0,
	    0, false, true },
	{ 36, "k1_twsi5",     0x4c,  1,  4, 3, -1, 0, k1_twsi_rates, 3, 0,
	    0, false, true },
	{ 37, "k1_twsi6",     0x60,  1,  4, 3, -1, 0, k1_twsi_rates, 3, 0,
	    0, false, true },
	{ 38, "k1_twsi7",     0x68,  1,  4, 3, -1, 0, k1_twsi_rates, 3, 0,
	    0, false, true },
	{ 39, "k1_twsi8",     0x20,  1,  4, 3, -1, 0, k1_twsi_rates, 3, 0,
	    0, true,  true },
	/* TWSI bus clocks: gate bit 0. */
	{ 84, "k1_twsi0_bus", 0x2c,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 85, "k1_twsi1_bus", 0x30,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 86, "k1_twsi2_bus", 0x38,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 87, "k1_twsi4_bus", 0x40,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 88, "k1_twsi5_bus", 0x4c,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 89, "k1_twsi6_bus", 0x60,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 90, "k1_twsi7_bus", 0x68,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 91, "k1_twsi8_bus", 0x20,  0, -1, 0, -1, 0, NULL, 0, 0,
	    0, true,  false },
	/* PWM func clocks: mux [6:4] w3 (12.8/24 MHz), gate bit 1. */
	{ 10, "k1_pwm0",   0x0c, 1,  4, 3, -1, 0, k1_pwm_rates, 2, 0 },
	{ 11, "k1_pwm1",   0x10, 1,  4, 3, -1, 0, k1_pwm_rates, 2, 0 },
	{ 12, "k1_pwm2",   0x14, 1,  4, 3, -1, 0, k1_pwm_rates, 2, 0 },
	{ 13, "k1_pwm3",   0x18, 1,  4, 3, -1, 0, k1_pwm_rates, 2, 0 },
	{ 14, "k1_pwm4",   0xa8, 1,  4, 3, -1, 0, k1_pwm_rates, 2, 0 },
	{ 15, "k1_pwm5",   0xac, 1,  4, 3, -1, 0, k1_pwm_rates, 2, 0 },
	{ 16, "k1_pwm6",   0xb0, 1,  4, 3, -1, 0, k1_pwm_rates, 2, 0 },
	{ 17, "k1_pwm7",   0xb4, 1,  4, 3, -1, 0, k1_pwm_rates, 2, 0 },
	{ 18, "k1_pwm8",   0xb8, 1,  4, 3, -1, 0, k1_pwm_rates, 2, 0 },
	{ 19, "k1_pwm9",   0xbc, 1,  4, 3, -1, 0, k1_pwm_rates, 2, 0 },
	{ 20, "k1_pwm10",  0xc0, 1,  4, 3, -1, 0, k1_pwm_rates, 2, 0 },
	{ 21, "k1_pwm11",  0xc4, 1,  4, 3, -1, 0, k1_pwm_rates, 2, 0 },
	{ 22, "k1_pwm12",  0xc8, 1,  4, 3, -1, 0, k1_pwm_rates, 2, 0 },
	{ 23, "k1_pwm13",  0xcc, 1,  4, 3, -1, 0, k1_pwm_rates, 2, 0 },
	{ 24, "k1_pwm14",  0xd0, 1,  4, 3, -1, 0, k1_pwm_rates, 2, 0 },
	{ 25, "k1_pwm15",  0xd4, 1,  4, 3, -1, 0, k1_pwm_rates, 2, 0 },
	{ 26, "k1_pwm16",  0xd8, 1,  4, 3, -1, 0, k1_pwm_rates, 2, 0 },
	{ 27, "k1_pwm17",  0xdc, 1,  4, 3, -1, 0, k1_pwm_rates, 2, 0 },
	{ 28, "k1_pwm18",  0xe0, 1,  4, 3, -1, 0, k1_pwm_rates, 2, 0 },
	{ 29, "k1_pwm19",  0xe4, 1,  4, 3, -1, 0, k1_pwm_rates, 2, 0 },
	/* PWM bus clocks: gate bit 0. */
	{ 62, "k1_pwm0_bus",  0x0c, 0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 63, "k1_pwm1_bus",  0x10, 0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 64, "k1_pwm2_bus",  0x14, 0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 65, "k1_pwm3_bus",  0x18, 0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 66, "k1_pwm4_bus",  0xa8, 0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 67, "k1_pwm5_bus",  0xac, 0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 68, "k1_pwm6_bus",  0xb0, 0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 69, "k1_pwm7_bus",  0xb4, 0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 70, "k1_pwm8_bus",  0xb8, 0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 71, "k1_pwm9_bus",  0xbc, 0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 72, "k1_pwm10_bus", 0xc0, 0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 73, "k1_pwm11_bus", 0xc4, 0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 74, "k1_pwm12_bus", 0xc8, 0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 75, "k1_pwm13_bus", 0xcc, 0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 76, "k1_pwm14_bus", 0xd0, 0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 77, "k1_pwm15_bus", 0xd4, 0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 78, "k1_pwm16_bus", 0xd8, 0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 79, "k1_pwm17_bus", 0xdc, 0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 80, "k1_pwm18_bus", 0xe0, 0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 81, "k1_pwm19_bus", 0xe4, 0, -1, 0, -1, 0, NULL, 0, 0 },
	/* SSP3 (SPI): func mux [6:4] w3, gate bit 1; bus gate bit 0. */
	{ 30, "k1_ssp3",      0x7c, 1,  4, 3, -1, 0, k1_ssp_rates, 4, 0 },
	{ 82, "k1_ssp3_bus",  0x7c, 0, -1, 0, -1, 0, NULL, 0, 0 },
};

/* APBC resets: bit 2 is an active-high reset in each CLK_RST register. */
static const struct smccu_reset k1_apbc_resets[] = {
	{ 0, 0x00, (1u << 2), 0 },	/* RESET_UART0 */
	{ 1, 0x04, (1u << 2), 0 },	/* RESET_UART2 */
	{ 2, 0x24, (1u << 2), 0 },	/* RESET_UART3 */
	{ 3, 0x70, (1u << 2), 0 },	/* RESET_UART4 */
	{ 4, 0x74, (1u << 2), 0 },	/* RESET_UART5 */
	{ 5, 0x78, (1u << 2), 0 },	/* RESET_UART6 */
	{ 6, 0x94, (1u << 2), 0 },	/* RESET_UART7 */
	{ 7, 0x98, (1u << 2), 0 },	/* RESET_UART8 */
	{ 8, 0x9c, (1u << 2), 0 },	/* RESET_UART9 */
	{ 9, 0x08, (1u << 2), 0 },	/* RESET_GPIO */
	{ 48, 0x6c, (1u << 2), 0 },	/* RESET_TSEN (thermal sensor) */
	{ 31, 0x28, (1u << 2), 0 },	/* RESET_RTC */
	/* TWSI (I2C) resets: bit 2 active-high, per APBC_TWSIn_CLK_RST. */
	{ 32, 0x2c, (1u << 2), 0 },	/* RESET_TWSI0 */
	{ 33, 0x30, (1u << 2), 0 },	/* RESET_TWSI1 */
	{ 34, 0x38, (1u << 2), 0 },	/* RESET_TWSI2 */
	{ 35, 0x40, (1u << 2), 0 },	/* RESET_TWSI4 */
	{ 36, 0x4c, (1u << 2), 0 },	/* RESET_TWSI5 */
	{ 37, 0x60, (1u << 2), 0 },	/* RESET_TWSI6 */
	{ 38, 0x68, (1u << 2), 0 },	/* RESET_TWSI7 */
	/*
	 * RESET_TWSI8 (id 39, reg 0x20) is DELIBERATELY OMITTED.
	 *
	 * APBC_TWSI8_CLK_RST (0x20) has the hardware quirk that reads always
	 * return 0 (documented in mainline ccu-k1.c).  The reset assert/deassert
	 * does a read-modify-write: it reads 0x20 (gets 0), clears the reset
	 * mask, ORs the (de)assert mask, and writes back.  For the deassert
	 * (deassert_mask 0) that writes 0 to 0x20 -- which WIPES the func-clock
	 * gate bits (1|0) that clk_enable() just set.  So the i2c driver's
	 * hwreset_deassert() was ungating TWSI8 immediately after enabling it,
	 * leaving the whole i2c8 register block UNCLOCKED (ISR/IBMR read 0, every
	 * transfer times out).  Mainline has NO reset node for twsi8 for exactly
	 * this reason and treats 0x20 as gate-only; we do the same.  With no
	 * reset entry, hwreset_get in the i2c driver fails and the driver skips
	 * the reset pulse (sc->reset = NULL), leaving the clock gate intact.
	 */
	/* PWM resets: bit 2 active-high, per APBC_PWMn_CLK_RST. */
	{ 10, 0x0c, (1u << 2), 0 },	/* RESET_PWM0 */
	{ 11, 0x10, (1u << 2), 0 },	/* RESET_PWM1 */
	{ 12, 0x14, (1u << 2), 0 },	/* RESET_PWM2 */
	{ 13, 0x18, (1u << 2), 0 },	/* RESET_PWM3 */
	{ 14, 0xa8, (1u << 2), 0 },	/* RESET_PWM4 */
	{ 15, 0xac, (1u << 2), 0 },	/* RESET_PWM5 */
	{ 16, 0xb0, (1u << 2), 0 },	/* RESET_PWM6 */
	{ 17, 0xb4, (1u << 2), 0 },	/* RESET_PWM7 */
	{ 18, 0xb8, (1u << 2), 0 },	/* RESET_PWM8 */
	{ 19, 0xbc, (1u << 2), 0 },	/* RESET_PWM9 */
	{ 20, 0xc0, (1u << 2), 0 },	/* RESET_PWM10 */
	{ 21, 0xc4, (1u << 2), 0 },	/* RESET_PWM11 */
	{ 22, 0xc8, (1u << 2), 0 },	/* RESET_PWM12 */
	{ 23, 0xcc, (1u << 2), 0 },	/* RESET_PWM13 */
	{ 24, 0xd0, (1u << 2), 0 },	/* RESET_PWM14 */
	{ 25, 0xd4, (1u << 2), 0 },	/* RESET_PWM15 */
	{ 26, 0xd8, (1u << 2), 0 },	/* RESET_PWM16 */
	{ 27, 0xdc, (1u << 2), 0 },	/* RESET_PWM17 */
	{ 28, 0xe0, (1u << 2), 0 },	/* RESET_PWM18 */
	{ 29, 0xe4, (1u << 2), 0 },	/* RESET_PWM19 */
	{ 30, 0x7c, (1u << 2), 0 },	/* RESET_SSP3 (SPI) */
};

/*
 * APMU bank (@0xd4282800).
 * SDH0/1/2 functional clocks: divider bits [10:8] (divisor = value + 1),
 * mux bits [7:5], gate bit 4, in the respective CLK_RES_CTRL register.
 */
static const uint64_t k1_sdh01_rates[] = {
	K1_PLL1_D6, K1_PLL1_D4, K1_PLL2_D8, K1_PLL2_D5, K1_PLL1_D11, K1_PLL1_D13, K1_PLL1_D23
};
static const uint64_t k1_sdh2_rates[] = {
	K1_PLL1_D6, K1_PLL1_D4, K1_PLL2_D8, K1_PLL1_D3, K1_PLL1_D11, K1_PLL1_D13,
	K1_PLL1_D23
};

#define	APMU_SDH0_CLK_RES_CTRL	0x054
#define	APMU_SDH1_CLK_RES_CTRL	0x058
#define	APMU_USB_CLK_RES_CTRL	0x05c
#define	APMU_AES_CLK_RES_CTRL	0x068	/* AES engine + HW CRNG */
#define	APMU_SDH2_CLK_RES_CTRL	0x0e0
#define	APMU_EMAC0_CLK_RES_CTRL	0x3e4
#define	APMU_EMAC1_CLK_RES_CTRL	0x3ec
#define	APMU_PCIE_CLK_RES_CTRL_0 0x3cc	/* combo PHY 0 = USB3 / PCIe0 */
#define	APMU_PCIE_CLK_RES_CTRL_1 0x3d4	/* PCIe1 (expansion slot) */
#define	APMU_PCIE_CLK_RES_CTRL_2 0x3dc	/* PCIe2 (expansion slot) */

/* AES functional clock parents (mux bit 6): pll1_d12 = 204.8 MHz, d24 = 102.4 MHz. */
static const uint64_t k1_aes_rates[] = { 204800000, 102400000 };

static const struct smccu_clk k1_apmu_clks[] = {
	/* id, name,          reg,                    gate, mux,w, div,w */
	{ 10, "k1_sdh_axi",   APMU_SDH0_CLK_RES_CTRL,  3, -1, 0, -1, 0,
	    NULL, 0, 0 },
	{ 11, "k1_sdh0",      APMU_SDH0_CLK_RES_CTRL,  4,  5, 3,  8, 3,
	    k1_sdh01_rates, nitems(k1_sdh01_rates), 0 },
	{ 12, "k1_sdh1",      APMU_SDH1_CLK_RES_CTRL,  4,  5, 3,  8, 3,
	    k1_sdh01_rates, nitems(k1_sdh01_rates), 0 },
	{ 13, "k1_sdh2",      APMU_SDH2_CLK_RES_CTRL,  4,  5, 3,  8, 3,
	    k1_sdh2_rates, nitems(k1_sdh2_rates), 0 },
	{ 14, "k1_usb_p1",    APMU_USB_CLK_RES_CTRL,   5, -1, 0, -1, 0,
	    NULL, 0, 0 },
	{ 15, "k1_usb_axi",   APMU_USB_CLK_RES_CTRL,   1, -1, 0, -1, 0,
	    NULL, 0, 0 },
	{ 16, "k1_usb30",     APMU_USB_CLK_RES_CTRL,   8, -1, 0, -1, 0,
	    NULL, 0, 0 },
	{ 37, "k1_emac0_bus", APMU_EMAC0_CLK_RES_CTRL, 0, -1, 0, -1, 0,
	    NULL, 0, 0 },
	{ 39, "k1_emac1_bus", APMU_EMAC1_CLK_RES_CTRL, 0, -1, 0, -1, 0,
	    NULL, 0, 0 },
	/* PCIe0 / USB3 combo-PHY gates (dbi/slave/master). */
	{ 28, "k1_pcie0_master", APMU_PCIE_CLK_RES_CTRL_0, 2, -1, 0, -1, 0,
	    NULL, 0, 0 },
	{ 29, "k1_pcie0_slave",  APMU_PCIE_CLK_RES_CTRL_0, 1, -1, 0, -1, 0,
	    NULL, 0, 0 },
	{ 30, "k1_pcie0_dbi",    APMU_PCIE_CLK_RES_CTRL_0, 0, -1, 0, -1, 0,
	    NULL, 0, 0 },
	/* PCIe1 gates (master/slave/dbi = bits 2/1/0), same layout as PCIe0. */
	{ 31, "k1_pcie1_master", APMU_PCIE_CLK_RES_CTRL_1, 2, -1, 0, -1, 0,
	    NULL, 0, 0 },
	{ 32, "k1_pcie1_slave",  APMU_PCIE_CLK_RES_CTRL_1, 1, -1, 0, -1, 0,
	    NULL, 0, 0 },
	{ 33, "k1_pcie1_dbi",    APMU_PCIE_CLK_RES_CTRL_1, 0, -1, 0, -1, 0,
	    NULL, 0, 0 },
	/* PCIe2 gates. */
	{ 34, "k1_pcie2_master", APMU_PCIE_CLK_RES_CTRL_2, 2, -1, 0, -1, 0,
	    NULL, 0, 0 },
	{ 35, "k1_pcie2_slave",  APMU_PCIE_CLK_RES_CTRL_2, 1, -1, 0, -1, 0,
	    NULL, 0, 0 },
	{ 36, "k1_pcie2_dbi",    APMU_PCIE_CLK_RES_CTRL_2, 0, -1, 0, -1, 0,
	    NULL, 0, 0 },
	/* AES engine / HW CRNG functional clock: mux bit 6 (w1), gate bit 5. */
	{ 20, "k1_aes",       APMU_AES_CLK_RES_CTRL,   5,  6, 1, -1, 0,
	    k1_aes_rates, nitems(k1_aes_rates), 0 },
};

/* APMU resets: the listed bit releases the block (deassert = set bit). */
static const struct smccu_reset k1_apmu_resets[] = {
	{ 2,  APMU_SDH0_CLK_RES_CTRL,  0, (1u << 0) },	/* RESET_SDH_AXI */
	{ 3,  APMU_SDH0_CLK_RES_CTRL,  0, (1u << 1) },	/* RESET_SDH0 */
	{ 4,  APMU_SDH1_CLK_RES_CTRL,  0, (1u << 1) },	/* RESET_SDH1 */
	{ 5,  APMU_SDH2_CLK_RES_CTRL,  0, (1u << 1) },	/* RESET_SDH2 */
	{ 6,  APMU_USB_CLK_RES_CTRL,   0, (1u << 4) },	/* RESET_USBP1_AXI */
	{ 7,  APMU_USB_CLK_RES_CTRL,   0, (1u << 0) },	/* RESET_USB_AXI */
	{ 8,  APMU_USB_CLK_RES_CTRL,   0, (1u << 9) },	/* RESET_USB30_AHB */
	{ 9,  APMU_USB_CLK_RES_CTRL,   0, (1u << 10) },	/* RESET_USB30_VCC */
	{ 10, APMU_USB_CLK_RES_CTRL,   0, (1u << 11) },	/* RESET_USB30_PHY */
	{ 35, APMU_EMAC0_CLK_RES_CTRL, 0, (1u << 1) },	/* RESET_EMAC0 */
	{ 36, APMU_EMAC1_CLK_RES_CTRL, 0, (1u << 1) },	/* RESET_EMAC1 */
	{ 14, APMU_AES_CLK_RES_CTRL,   0, (1u << 4) },	/* RESET_AES (CRNG) */
	/* PCIe0 / USB3 combo-PHY resets (master/slave/dbi release by setting
	   the bit; GLOBAL is active on bit 8, released by clearing it). */
	{ 23, APMU_PCIE_CLK_RES_CTRL_0, 0, (1u << 5) },	/* RESET_PCIE0_MASTER */
	{ 24, APMU_PCIE_CLK_RES_CTRL_0, 0, (1u << 4) },	/* RESET_PCIE0_SLAVE */
	{ 25, APMU_PCIE_CLK_RES_CTRL_0, 0, (1u << 3) },	/* RESET_PCIE0_DBI */
	{ 26, APMU_PCIE_CLK_RES_CTRL_0, (1u << 8), 0 },	/* RESET_PCIE0_GLOBAL */
	/* PCIe1 resets: master/slave/dbi release on bits 5/4/3; global on bit 8. */
	{ 27, APMU_PCIE_CLK_RES_CTRL_1, 0, (1u << 5) },	/* RESET_PCIE1_MASTER */
	{ 28, APMU_PCIE_CLK_RES_CTRL_1, 0, (1u << 4) },	/* RESET_PCIE1_SLAVE */
	{ 29, APMU_PCIE_CLK_RES_CTRL_1, 0, (1u << 3) },	/* RESET_PCIE1_DBI */
	{ 30, APMU_PCIE_CLK_RES_CTRL_1, (1u << 8), 0 },	/* RESET_PCIE1_GLOBAL */
	/* PCIe2 resets. */
	{ 31, APMU_PCIE_CLK_RES_CTRL_2, 0, (1u << 5) },	/* RESET_PCIE2_MASTER */
	{ 32, APMU_PCIE_CLK_RES_CTRL_2, 0, (1u << 4) },	/* RESET_PCIE2_SLAVE */
	{ 33, APMU_PCIE_CLK_RES_CTRL_2, 0, (1u << 3) },	/* RESET_PCIE2_DBI */
	{ 34, APMU_PCIE_CLK_RES_CTRL_2, (1u << 8), 0 },	/* RESET_PCIE2_GLOBAL */
};

static const struct smccu_bank k1_apbc_bank = {
	.clks = k1_apbc_clks,
	.nclks = nitems(k1_apbc_clks),
	.resets = k1_apbc_resets,
	.nresets = nitems(k1_apbc_resets),
};

static const struct smccu_bank k1_apmu_bank = {
	.clks = k1_apmu_clks,
	.nclks = nitems(k1_apmu_clks),
	.resets = k1_apmu_resets,
	.nresets = nitems(k1_apmu_resets),
};

static struct ofw_compat_data compat_data[] = {
	{ "spacemit,k1-syscon-apbc",	(uintptr_t)&k1_apbc_bank },
	{ "spacemit,k1-syscon-apmu",	(uintptr_t)&k1_apmu_bank },
	{ NULL,				0 }
};

static struct resource_spec res_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE | RF_SHAREABLE },
	RESOURCE_SPEC_END
};

/*
 * Some APBC clock/reset registers read back as 0 no matter what was written
 * (APBC_TWSI8_CLK_RST at 0x20 is the one we know of -- verified on hardware:
 * write 0x23, read 0x00000000, while the i2c8 block it gates responds
 * normally).  A read-modify-write on such a register silently discards every
 * field it is not itself setting, which is how enabling the TWSI8 gate used to
 * wipe the functional-clock mux back to index 0 and leave the PMIC bus dead.
 * Keep a driver-side shadow for those registers so gate and mux updates
 * compose.  The shadow is per REGISTER, not per clknode, because several
 * clknodes (func gate, bus gate) share one register.
 */
#define	SMCCU_MAX_WO	4

struct smccu_wo {
	uint32_t	off;
	uint32_t	val;
};

struct smccu_softc {
	device_t		dev;
	struct mtx		mtx;
	struct resource		*mem_res;
	struct clkdom		*clkdom;
	const struct smccu_bank	*bank;
	struct syscon		*syscon;
	struct smccu_wo		wo[SMCCU_MAX_WO];
	int			nwo;
};

struct smccu_clknode_sc {
	const struct smccu_clk	*def;
};

#define	READ4(_sc, _off)	bus_read_4((_sc)->mem_res, (_off))
#define	WRITE4(_sc, _off, _val)	bus_write_4((_sc)->mem_res, (_off), (_val))

/*
 * Register accessors that honour the write-only shadow.  Every clock and reset
 * access goes through these; the syscon methods deliberately do not, because
 * their consumers want the real register.
 */
static struct smccu_wo *
smccu_wo_find(struct smccu_softc *sc, uint32_t off)
{
	int i;

	for (i = 0; i < sc->nwo; i++) {
		if (sc->wo[i].off == off)
			return (&sc->wo[i]);
	}
	return (NULL);
}

static uint32_t
smccu_reg_read(struct smccu_softc *sc, uint32_t off)
{
	struct smccu_wo *wo;

	wo = smccu_wo_find(sc, off);
	if (wo != NULL)
		return (wo->val);
	return (READ4(sc, off));
}

static void
smccu_reg_write(struct smccu_softc *sc, uint32_t off, uint32_t val)
{
	struct smccu_wo *wo;

	wo = smccu_wo_find(sc, off);
	if (wo != NULL)
		wo->val = val;
	WRITE4(sc, off, val);
}

#define	DEVICE_LOCK(_clk)					\
	CLKDEV_DEVICE_LOCK(clknode_get_device(_clk))
#define	DEVICE_UNLOCK(_clk)					\
	CLKDEV_DEVICE_UNLOCK(clknode_get_device(_clk))

/*
 * Clock node methods.
 */
static int
smccu_clknode_init(struct clknode *clk, device_t dev)
{

	clknode_init_parent_idx(clk, 0);
	return (0);
}

static int
smccu_clknode_recalc_freq(struct clknode *clk, uint64_t *freq)
{
	struct smccu_softc *sc;
	struct smccu_clknode_sc *csc;
	const struct smccu_clk *def;
	uint64_t rate;
	uint32_t reg, idx, div;

	sc = device_get_softc(clknode_get_device(clk));
	csc = clknode_get_softc(clk);
	def = csc->def;

	rate = def->rate;
	if (def->mux_shift >= 0) {
		DEVICE_LOCK(clk);
		reg = smccu_reg_read(sc, def->reg);
		DEVICE_UNLOCK(clk);

		idx = (reg >> def->mux_shift) & ((1u << def->mux_width) - 1);
		rate = (idx < def->nrates) ? def->prates[idx] : 0;
		if (def->div_shift >= 0 && rate != 0) {
			div = ((reg >> def->div_shift) &
			    ((1u << def->div_width) - 1)) + 1;
			rate /= div;
		}
	}

	*freq = rate;
	return (0);
}

static int
smccu_clknode_set_gate(struct clknode *clk, bool enable)
{
	struct smccu_softc *sc;
	struct smccu_clknode_sc *csc;
	const struct smccu_clk *def;
	uint32_t mask, reg;

	sc = device_get_softc(clknode_get_device(clk));
	csc = clknode_get_softc(clk);
	def = csc->def;

	if (def->gate_shift < 0 && def->gate_mask == 0)
		return (0);

	mask = (def->gate_mask != 0) ? def->gate_mask :
	    (1u << def->gate_shift);

	DEVICE_LOCK(clk);
	reg = smccu_reg_read(sc, def->reg);
	if (enable)
		reg |= mask;
	else
		reg &= ~mask;
	smccu_reg_write(sc, def->reg, reg);
	DEVICE_UNLOCK(clk);

	return (0);
}

/*
 * Reprogram a clock's parent mux.
 *
 * Deliberately opt-in (def->settable): several of the muxes described in this
 * file also carry a divider and feed blocks -- notably SDH, i.e. the eMMC this
 * board boots from -- that are running on whatever the bootloader left them
 * at.  sdhci_fdt_spacemit.c calls clk_set_freq() on its io clock and today
 * gets an error and keeps the inherited rate; quietly making that call succeed
 * would change the eMMC clock as a side effect of an i2c fix.  So only the
 * clocks that have been reasoned about individually are settable, which today
 * means the TWSI (i2c) functional clocks.
 *
 * There is no divider on those, so this only selects a mux index: the nearest
 * available parent rate, exactly as CLK_SET_ROUND_ANY asks for.
 */
static int
smccu_clknode_set_freq(struct clknode *clk, uint64_t fparent, uint64_t *fout,
    int flags, int *stop)
{
	struct smccu_softc *sc;
	struct smccu_clknode_sc *csc;
	const struct smccu_clk *def;
	uint64_t best, diff, bestdiff;
	uint32_t reg, mask;
	int i, bestidx;

	sc = device_get_softc(clknode_get_device(clk));
	csc = clknode_get_softc(clk);
	def = csc->def;

	*stop = 1;

	if (!def->settable || def->mux_shift < 0 || def->prates == NULL ||
	    def->nrates == 0)
		return (ENXIO);

	bestidx = -1;
	best = 0;
	bestdiff = ~(uint64_t)0;
	for (i = 0; i < def->nrates; i++) {
		if (def->prates[i] == 0)
			continue;
		diff = (def->prates[i] > *fout) ? def->prates[i] - *fout :
		    *fout - def->prates[i];
		if (diff < bestdiff) {
			bestdiff = diff;
			bestidx = i;
			best = def->prates[i];
		}
	}
	if (bestidx < 0)
		return (ENXIO);

	if ((flags & CLK_SET_DRYRUN) != 0) {
		*fout = best;
		return (0);
	}

	mask = ((1u << def->mux_width) - 1) << def->mux_shift;

	DEVICE_LOCK(clk);
	reg = smccu_reg_read(sc, def->reg);
	reg &= ~mask;
	reg |= ((uint32_t)bestidx << def->mux_shift) & mask;
	smccu_reg_write(sc, def->reg, reg);
	DEVICE_UNLOCK(clk);

	*fout = best;
	return (0);
}

static clknode_method_t smccu_clknode_methods[] = {
	CLKNODEMETHOD(clknode_init,		smccu_clknode_init),
	CLKNODEMETHOD(clknode_set_gate,		smccu_clknode_set_gate),
	CLKNODEMETHOD(clknode_recalc_freq,	smccu_clknode_recalc_freq),
	CLKNODEMETHOD(clknode_set_freq,		smccu_clknode_set_freq),
	CLKNODEMETHOD_END
};

DEFINE_CLASS_1(smccu_clknode, smccu_clknode_class, smccu_clknode_methods,
    sizeof(struct smccu_clknode_sc), clknode_class);

/*
 * Reset (hwreset) methods.
 */
static const struct smccu_reset *
smccu_reset_lookup(struct smccu_softc *sc, intptr_t id)
{
	int i;

	for (i = 0; i < sc->bank->nresets; i++) {
		if (sc->bank->resets[i].id == id)
			return (&sc->bank->resets[i]);
	}
	return (NULL);
}

static int
smccu_reset_assert(device_t dev, intptr_t id, bool assert)
{
	struct smccu_softc *sc;
	const struct smccu_reset *rst;
	uint32_t mask, reg;

	sc = device_get_softc(dev);
	rst = smccu_reset_lookup(sc, id);
	if (rst == NULL)
		return (ENXIO);

	mask = rst->assert_mask | rst->deassert_mask;

	mtx_lock(&sc->mtx);
	reg = smccu_reg_read(sc, rst->reg) & ~mask;
	reg |= assert ? rst->assert_mask : rst->deassert_mask;
	smccu_reg_write(sc, rst->reg, reg);
	mtx_unlock(&sc->mtx);

	return (0);
}

static int
smccu_reset_is_asserted(device_t dev, intptr_t id, bool *reset)
{
	struct smccu_softc *sc;
	const struct smccu_reset *rst;
	uint32_t reg;

	sc = device_get_softc(dev);
	rst = smccu_reset_lookup(sc, id);
	if (rst == NULL)
		return (ENXIO);

	mtx_lock(&sc->mtx);
	reg = smccu_reg_read(sc, rst->reg);
	mtx_unlock(&sc->mtx);

	if (rst->assert_mask != 0)
		*reset = (reg & rst->assert_mask) != 0;
	else
		*reset = (reg & rst->deassert_mask) == 0;

	return (0);
}

/*
 * Syscon interface: expose the bank registers to consumers such as the
 * EMAC driver, which programs RGMII delays through the APMU bank
 * (devicetree property "spacemit,apmu" = <phandle offset>).
 */
static uint32_t
smccu_syscon_read_4(struct syscon *syscon, bus_size_t offset)
{
	struct smccu_softc *sc;

	sc = device_get_softc(syscon->pdev);
	mtx_assert(&sc->mtx, MA_OWNED);
	return (READ4(sc, offset));
}

static int
smccu_syscon_write_4(struct syscon *syscon, bus_size_t offset, uint32_t val)
{
	struct smccu_softc *sc;

	sc = device_get_softc(syscon->pdev);
	mtx_assert(&sc->mtx, MA_OWNED);
	WRITE4(sc, offset, val);
	return (0);
}

static int
smccu_syscon_modify_4(struct syscon *syscon, bus_size_t offset,
    uint32_t clear_bits, uint32_t set_bits)
{
	struct smccu_softc *sc;
	uint32_t val;

	sc = device_get_softc(syscon->pdev);
	mtx_assert(&sc->mtx, MA_OWNED);
	val = READ4(sc, offset);
	val &= ~clear_bits;
	val |= set_bits;
	WRITE4(sc, offset, val);
	return (0);
}

static syscon_method_t smccu_syscon_methods[] = {
	SYSCONMETHOD(syscon_unlocked_read_4,	smccu_syscon_read_4),
	SYSCONMETHOD(syscon_unlocked_write_4,	smccu_syscon_write_4),
	SYSCONMETHOD(syscon_unlocked_modify_4,	smccu_syscon_modify_4),

	SYSCONMETHOD_END
};
DEFINE_CLASS_1(smccu_syscon, smccu_syscon_class, smccu_syscon_methods, 0,
    syscon_class);

/*
 * Device methods.
 */
static int
smccu_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "SpacemiT K1 clock and reset controller");
	return (BUS_PROBE_DEFAULT);
}

static int
smccu_attach(device_t dev)
{
	struct smccu_softc *sc;
	struct clknode_init_def clkdef;
	struct clknode *clk;
	struct smccu_clknode_sc *csc;
	int err, i;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->bank = (const struct smccu_bank *)
	    ofw_bus_search_compatible(dev, compat_data)->ocd_data;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	err = bus_alloc_resources(dev, res_spec, &sc->mem_res);
	if (err != 0) {
		device_printf(dev, "cannot allocate resources: %d\n", err);
		return (ENXIO);
	}

	/*
	 * Register the write-only clock registers before creating any clknode,
	 * so the very first gate/mux update composes against the shadow rather
	 * than against a read that returns 0.  The shadow starts at 0, which is
	 * also what such a register reads, so nothing is lost.
	 */
	for (i = 0; i < sc->bank->nclks; i++) {
		if (!sc->bank->clks[i].wo_reg)
			continue;
		if (smccu_wo_find(sc, sc->bank->clks[i].reg) != NULL)
			continue;
		if (sc->nwo >= SMCCU_MAX_WO) {
			device_printf(dev,
			    "too many write-only clock registers\n");
			break;
		}
		sc->wo[sc->nwo].off = sc->bank->clks[i].reg;
		sc->wo[sc->nwo].val = 0;
		sc->nwo++;
	}

	sc->clkdom = clkdom_create(dev);
	if (sc->clkdom == NULL) {
		device_printf(dev, "cannot create clkdom\n");
		return (ENXIO);
	}

	for (i = 0; i < sc->bank->nclks; i++) {
		memset(&clkdef, 0, sizeof(clkdef));
		clkdef.id = sc->bank->clks[i].id;
		clkdef.name = sc->bank->clks[i].name;
		clkdef.parent_names = NULL;
		clkdef.parent_cnt = 0;

		clk = clknode_create(sc->clkdom, &smccu_clknode_class,
		    &clkdef);
		if (clk == NULL) {
			device_printf(dev, "cannot create clk %s\n",
			    clkdef.name);
			return (ENXIO);
		}
		csc = clknode_get_softc(clk);
		csc->def = &sc->bank->clks[i];
		clknode_register(sc->clkdom, clk);
	}

	if (clkdom_finit(sc->clkdom) != 0) {
		device_printf(dev, "cannot finalize clkdom\n");
		return (ENXIO);
	}

	if (bootverbose)
		clkdom_dump(sc->clkdom);

	hwreset_register_ofw_provider(dev);

	sc->syscon = syscon_create_ofw_node(dev, &smccu_syscon_class,
	    ofw_bus_get_node(dev));
	if (sc->syscon == NULL)
		device_printf(dev, "cannot register syscon provider\n");

	return (0);
}

static int
smccu_detach(device_t dev)
{

	return (EBUSY);
}

static void
smccu_device_lock(device_t dev)
{
	struct smccu_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
}

static void
smccu_device_unlock(device_t dev)
{
	struct smccu_softc *sc;

	sc = device_get_softc(dev);
	mtx_unlock(&sc->mtx);
}

static device_method_t smccu_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		smccu_probe),
	DEVMETHOD(device_attach,	smccu_attach),
	DEVMETHOD(device_detach,	smccu_detach),

	/* clkdev interface */
	DEVMETHOD(clkdev_device_lock,	smccu_device_lock),
	DEVMETHOD(clkdev_device_unlock,	smccu_device_unlock),

	/* syscon device locking (shares the bank mutex) */
	DEVMETHOD(syscon_device_lock,	smccu_device_lock),
	DEVMETHOD(syscon_device_unlock,	smccu_device_unlock),

	/* Reset interface */
	DEVMETHOD(hwreset_assert,	smccu_reset_assert),
	DEVMETHOD(hwreset_is_asserted,	smccu_reset_is_asserted),

	DEVMETHOD_END
};

DEFINE_CLASS_0(spacemit_ccu, spacemit_ccu_driver, smccu_methods,
    sizeof(struct smccu_softc));
EARLY_DRIVER_MODULE(spacemit_ccu, simplebus, spacemit_ccu_driver, 0, 0,
    BUS_PASS_BUS + BUS_PASS_ORDER_LATE);
MODULE_VERSION(spacemit_ccu, 1);
