/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Daniel Shue <dgshue@gmail.com>
 *
 * SpacemiT K1 (Ky X1) system reset via the vendor watchdog.
 *
 * This board's OpenSBI does not implement a working SRST SYSTEM_RESET, so
 * FreeBSD's default cpu_reset() (SBI COLD_REBOOT) hangs.  The K1 instead
 * resets through its watchdog: arm the WDT in reset mode and route its
 * expiry to a full system reset via the MPMU APRR register.  We hook
 * shutdown_final so that, on reboot, the watchdog fires and restarts the
 * SoC before (or shortly after) the kernel reaches the hung SBI call.
 *
 * Register details: docs/hardware/k1-watchdog-reset.md.  Addresses are
 * hardcoded because the mainline devicetree we boot lacks the vendor
 * "spacemit,soc-wdt" node; an upstream-shaped version would be an
 * ofw-bound spacemit_wdt(4) providing a real watchdog(9) device too.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/eventhandler.h>
#include <sys/kernel.h>
#include <sys/reboot.h>
#include <sys/watchdog.h>

#include <machine/bus.h>
#include <vm/vm.h>
#include <vm/pmap.h>

#define	K1_WDT_PA	0xd4080000UL
#define	K1_WDT_SIZE	0x100
#define	K1_MPMU_PA	0xd4050000UL
#define	K1_MPMU_SIZE	0x2000

/* WDT registers (offset from K1_WDT_PA). */
#define	WDT_WFAR	0xb0	/* write-access key 1 (0xbaba) */
#define	WDT_WSAR	0xb4	/* write-access key 2 (0xeb10) */
#define	WDT_WMER	0xb8	/* mode enable: 0x3 = count+reset */
#define	WDT_WMR		0xbc	/* match/timeout; ticks = sec << 8 */
#define	WDT_WSR		0xc0	/* status */
#define	WDT_WCR		0xc8	/* counter reset (0x1) */

/* MPMU registers (offset from K1_MPMU_PA). */
#define	MPMU_APRR	0x1020
#define	 MPMU_APRR_WDTR	(1u << 4)	/* route WDT expiry to system reset */

static char *k1_wdt_va;
static char *k1_mpmu_va;

static inline void
wdt_wr(uint32_t reg, uint32_t val)
{

	/* Every WDT write must be unlocked by the key pair first. */
	*(volatile uint32_t *)(k1_wdt_va + WDT_WFAR) = 0xbaba;
	*(volatile uint32_t *)(k1_wdt_va + WDT_WSAR) = 0xeb10;
	*(volatile uint32_t *)(k1_wdt_va + reg) = val;
}

static void
k1_reset_final(void *arg __unused, int howto)
{
	uint32_t reg;

	/* Only force a reset on reboot; leave halt/poweroff alone. */
	if ((howto & (RB_HALT | RB_POWEROFF)) != 0)
		return;
	if (k1_wdt_va == NULL || k1_mpmu_va == NULL)
		return;

	/* Short timeout (~1/4 s at ~256 Hz), enable counter in reset mode. */
	wdt_wr(WDT_WMR, 64);
	wdt_wr(WDT_WMER, 0x3);

	/* Route watchdog expiry to a full system reset. */
	reg = *(volatile uint32_t *)(k1_mpmu_va + MPMU_APRR);
	reg |= MPMU_APRR_WDTR;
	*(volatile uint32_t *)(k1_mpmu_va + MPMU_APRR) = reg;

	/* Clear status and (re)start the counter. */
	wdt_wr(WDT_WSR, 0x0);
	wdt_wr(WDT_WCR, 0x1);

	/* Spin until the watchdog resets the SoC. */
	for (;;)
		cpu_spinwait();
}

/*
 * watchdog(9) provider: watchdogd(8) calls this periodically with the
 * requested timeout; each call re-arms the WDT (WCR restarts the counter),
 * so a hung system stops petting it and the WDT resets the SoC.  This also
 * covers hangs the shutdown_final handler cannot (it never runs on a hang).
 */
static void
k1_wd_event(void *arg __unused, u_int cmd, int *error)
{
	uint64_t ns;
	uint32_t secs, ticks;

	if (k1_wdt_va == NULL || k1_mpmu_va == NULL)
		return;

	cmd &= WD_INTERVAL;
	if (cmd == 0) {
		/* Disarm. */
		wdt_wr(WDT_WMER, 0x0);
		*error = 0;
		return;
	}

	/* cmd is log2(nanoseconds); the WDT counts at ~256 Hz. */
	ns = (uint64_t)1 << cmd;
	secs = ns / 1000000000ULL;
	if (secs == 0)
		secs = 1;
	if (secs > 255)
		secs = 255;		/* 16-bit counter at 256 Hz */
	ticks = secs << 8;

	*(volatile uint32_t *)(k1_mpmu_va + MPMU_APRR) |= MPMU_APRR_WDTR;
	wdt_wr(WDT_WMR, ticks);
	wdt_wr(WDT_WMER, 0x3);
	wdt_wr(WDT_WSR, 0x0);
	wdt_wr(WDT_WCR, 0x1);		/* restart counter = pet */
	*error = 0;
}

static void
k1_reset_init(void *arg __unused)
{

	k1_wdt_va = pmap_mapdev(K1_WDT_PA, K1_WDT_SIZE);
	k1_mpmu_va = pmap_mapdev(K1_MPMU_PA, K1_MPMU_SIZE);
	if (k1_wdt_va == NULL || k1_mpmu_va == NULL) {
		printf("spacemit_reset: cannot map WDT/MPMU registers\n");
		return;
	}

	/*
	 * Run late so our handler executes before the SBI-based one in
	 * sbi.c (which would otherwise hang).  SHUTDOWN_PRI_LAST orders us
	 * after most subsystems; the watchdog reset is the final word.
	 */
	EVENTHANDLER_REGISTER(shutdown_final, k1_reset_final, NULL,
	    SHUTDOWN_PRI_LAST);

	/* Register as a watchdog(9) provider for watchdogd(8). */
	EVENTHANDLER_REGISTER(watchdog_list, k1_wd_event, NULL, 0);
}
SYSINIT(k1_reset, SI_SUB_CONFIGURE, SI_ORDER_ANY, k1_reset_init, NULL);
