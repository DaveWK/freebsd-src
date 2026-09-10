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
 * MFD + regulator + power-off driver for the SpacemiT P1 PMIC
 * (compatible: "spacemit,p1"), an I2C device at address 0x41 on the K1's
 * i2c8 bus.  It provides six buck converters and twelve LDOs, a power-off/
 * reset controller, plus an RTC/ADC/GPIO (not handled here).
 *
 * This driver covers the pieces most valuable for a headless appliance:
 *   - the MFD core (iicbus register access at 0x41),
 *   - the buck/LDO regulators via regulator(9)/regnode, and
 *   - a shutdown_final hook that puts the PMIC into its shutdown state
 *     (clean power-off -- the SoC has no SBI/PSCI power-off on this board).
 *
 * Register map + voltage ranges reimplemented from the mainline Linux P1
 * drivers (drivers/regulator/spacemit-p1.c, drivers/power/reset/
 * spacemit-p1-reboot.c; GPL-2.0), NOT copied.  Modeled on FreeBSD's
 * dev/iicbus/pmic/rockchip/rk8xx.c.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/clock.h>
#include <sys/eventhandler.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/reboot.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>
#include <sys/time.h>

#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/regulator/regulator.h>

#include "clock_if.h"
#include "regdev_if.h"

/*
 * RTC registers.  RTC_TIME is 6 consecutive bytes:
 *   [0]=sec [1]=min [2]=hour [3]=mday-1 [4]=mon(0-based) [5]=year-2000
 * (plain integers masked to the field width, NOT BCD).  RTC_CTRL bit 2 must
 * be set for the counter to run; it is cleared during an update.  This is the
 * power-off-persistent clock (battery/supercap backed on the PMIC), unlike
 * the SoC mrvl RTC which only survives a warm reboot.
 */
#define	P1_RTC_TIME	0x0d
#define	P1_RTC_CTRL	0x1d
/*
 * RTC_CTRL (0x1d) bit layout, from the vendor SPM8821 header
 * (include/linux/mfd/spacemit/spm8821.h union rtc_ctl_desc / rtc_ctl.reg=0x1d):
 *   bit0 crystal_en, bit1 out_32k_en, bit2 rtc_en, bit3 rtc_clk_sel,
 *   bit4 tick_type, bit5 alarm_en, bit6 tick_en.
 */
#define	P1_RTC_CRYSTAL_EN	(1u << 0)
#define	P1_RTC_OUT_32K_EN	(1u << 1)	/* PMIC 32KOUT (pin50) -> AP6256 LPO */
#define	P1_RTC_EN	(1u << 2)
#define	P1_RTC_CLK_SEL	(1u << 3)	/* internal 32k clk select */
/*
 * The exact bit set the vendor RTC probe writes (rtc-spt-pmic.c
 * spacemit_rtc_probe): crystal_en | out_32k_en | rtc_en | rtc_clk_sel = 0x0f.
 * out_32k_en drives the 32KOUT pin that feeds the AP6256 WiFi LPO; crystal_en
 * starts the 32.768 kHz crystal that sources it.
 */
#define	P1_RTC_32K_ON	(P1_RTC_CRYSTAL_EN | P1_RTC_OUT_32K_EN | \
			 P1_RTC_EN | P1_RTC_CLK_SEL)
#define	P1_RTC_READ_TRIES	20

/* Power Control Register 2: shutdown/reset requests. */
#define	P1_PWR_CTRL2		0x7e
#define	P1_PWR_CTRL2_SHUTDOWN	(1u << 2)
#define	P1_PWR_CTRL2_RST	(1u << 1)

/*
 * Regulator register layout (from mainline spacemit-p1.c):
 *   enable_reg(n) = base + 3*(n-1), enable bit 0
 *   vsel_reg(n)   = enable_reg(n) + 1
 * Buck base 0x47 (vsel mask 0xff), ALDO base 0x5b, DLDO base 0x67
 * (LDO vsel mask 0x7f).
 */
#define	P1_BUCK_BASE	0x47
#define	P1_ALDO_BASE	0x5b
#define	P1_DLDO_BASE	0x67

#define	P1_BUCK_VSEL_MASK	0xff
#define	P1_LDO_VSEL_MASK	0x7f

enum p1_reg_type {
	P1_TYPE_BUCK,
	P1_TYPE_LDO,
};

struct p1_regdef {
	int		id;
	const char	*name;
	enum p1_reg_type type;
	uint8_t		enable_reg;
	uint8_t		enable_mask;
	uint8_t		vsel_reg;
	uint8_t		vsel_mask;
};

/*
 * Buck: [500000 uV @ sel 0 .. 170, step 5000], [1375000 @ 171 .. 254, step
 *       25000].  LDO: [500000 @ sel 11 .. 127, step 25000] (sel < 11 invalid).
 */
#define	BUCK_R1_MIN	500000
#define	BUCK_R1_STEP	5000
#define	BUCK_R1_SELMAX	170
#define	BUCK_R2_MIN	1375000
#define	BUCK_R2_STEP	25000
#define	BUCK_R2_SELMAX	254

#define	LDO_MIN		500000
#define	LDO_STEP	25000
#define	LDO_SELMIN	11
#define	LDO_SELMAX	127

#define	P1_ENREG(base, n)	((base) + 3 * ((n) - 1))

#define	P1_BUCK(n)	{ (n) - 1, "buck" #n, P1_TYPE_BUCK, \
	P1_ENREG(P1_BUCK_BASE, n), 0x1, P1_ENREG(P1_BUCK_BASE, n) + 1, \
	P1_BUCK_VSEL_MASK }
#define	P1_ALDO(n, id)	{ (id), "aldo" #n, P1_TYPE_LDO, \
	P1_ENREG(P1_ALDO_BASE, n), 0x1, P1_ENREG(P1_ALDO_BASE, n) + 1, \
	P1_LDO_VSEL_MASK }
#define	P1_DLDO(n, id)	{ (id), "dldo" #n, P1_TYPE_LDO, \
	P1_ENREG(P1_DLDO_BASE, n), 0x1, P1_ENREG(P1_DLDO_BASE, n) + 1, \
	P1_LDO_VSEL_MASK }

static const struct p1_regdef p1_regdefs[] = {
	P1_BUCK(1), P1_BUCK(2), P1_BUCK(3), P1_BUCK(4), P1_BUCK(5), P1_BUCK(6),
	P1_ALDO(1, 6), P1_ALDO(2, 7), P1_ALDO(3, 8), P1_ALDO(4, 9),
	P1_DLDO(1, 10), P1_DLDO(2, 11), P1_DLDO(3, 12), P1_DLDO(4, 13),
	P1_DLDO(5, 14), P1_DLDO(6, 15), P1_DLDO(7, 16),
};

struct spacemit_p1_softc {
	device_t	dev;
	eventhandler_tag off_tag;
	eventhandler_tag arm_tag;
	struct timeout_task deadman;	/* fires if shutdown stalls */
	int		deadman_howto;
	uint8_t		rtc_ctl_at_attach; /* RTC_CTRL 0x1d after 32KOUT enable */
	struct intr_config_hook	lpo_hook; /* deferred 32KOUT enable (post-intr) */
};

/*
 * The P1 rail-control register window: BUCK enables/vsel start at 0x47, LDO
 * windows follow, and PWR_CTRL2 (shutdown/reset) is at 0x7e.  Dumping 0x47
 * through 0x7e (inclusive, 56 bytes) captures every rail's enable + voltage
 * selector, which is exactly what we need to PROVE no rail state changed.
 */
#define	P1_RAIL_DUMP_START	0x47
#define	P1_RAIL_DUMP_LEN	((0x7e - 0x47) + 1)	/* = 56 */

/*
 * Single global instance: the userland i2c(8) tool cannot read address 0x41
 * because this driver owns it on iicbus, so the zero-rail-change proof is
 * exposed here (a read-only sysctl + attach/post-sweep logging) using the
 * driver's own bus access.
 */
static struct spacemit_p1_softc *p1_softc;

/*
 * SAFETY GUARD (default ON): never allow this driver to DISABLE a rail or
 * change its voltage.  On the K1, the buck/LDO rails power the CPU, DRAM and
 * SoC; dropping one is an INSTANT, unrecoverable hard brick (the CPU dies
 * before any watchdog/serial recovery can act).  Firmware/U-Boot brings up
 * every rail this board needs before we ever attach, so the correct default
 * is to reflect hardware state read-only and refuse any disable/lower.  The
 * FreeBSD regulator core runs a "disable unused regulators" sweep at
 * SI_SUB_LAST (hw.regulator.disable_unused, default on) that would otherwise
 * turn off any rail not marked regulator-always-on in the DTB -- this guard
 * makes that sweep a physical no-op for the PMIC.  Set the tunable to 0 only
 * with full knowledge of the board's power tree.
 */
static int spacemit_p1_allow_rail_changes = 0;
SYSCTL_NODE(_hw, OID_AUTO, spacemit_p1, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "SpacemiT P1 PMIC");
SYSCTL_INT(_hw_spacemit_p1, OID_AUTO, allow_rail_changes,
    CTLFLAG_RDTUN, &spacemit_p1_allow_rail_changes, 0,
    "Allow the P1 driver to disable rails or lower voltages (DANGEROUS: a "
    "wrong change can hard-brick the board). Default 0 = read-only/enable-only.");

/*
 * Reset the SoC through the PMIC when the system reboots (default ON).
 *
 * The K1 has no other working system reset.  cpu_reset() asks SBI for
 * SBI_SRST_TYPE_COLD_REBOOT, but sbi.c only issues that call when the SRST
 * extension probed non-zero, and on this board it does not.  The OpenSBI the
 * bootloader hands us *does* carry a reset driver for this very PMIC
 * ("spacemit-p1-reset"), but it binds to a DT node with compatible
 * "spacemit,p1" and the U-Boot control DTB for the R2S has i2c8 disabled with
 * no PMIC child, so no reset device is ever registered.  With none registered
 * the SRST probe returns 0, has_srst_extension stays false, and
 * sbi_system_reset() falls through to the legacy SBI_SHUTDOWN call -- a halt,
 * not a reset.  The boot hart stops, the SoC is never reset, and the board
 * sits there still answering ping from the remaining harts until somebody
 * power-cycles it by hand.
 *
 * So do here what OpenSBI would have done if it had been given the node.
 * Set this tunable to 0 to get the old (hanging) behaviour back without
 * rebuilding a kernel.
 */
static int spacemit_p1_reset_on_reboot = 1;
SYSCTL_INT(_hw_spacemit_p1, OID_AUTO, reset_on_reboot,
    CTLFLAG_RDTUN, &spacemit_p1_reset_on_reboot, 0,
    "Reset the SoC via the PMIC on reboot; the K1 has no working SBI SRST. "
    "Default 1; 0 leaves reboot to cpu_reset(), which hangs this board.");

/*
 * Optional post-sync recovery deadline. The earlier K1 watchdog hook used
 * to spin forever before the PMIC callback. That hook is now bounded;
 * retain the tested deadman as a fallback for other stalled shutdowns.
 * This is not proof that all device caches have completed their flush.
 * Set to zero to disable the fallback while diagnosing shutdown.
 */
static int spacemit_p1_reset_deadman = 15;
SYSCTL_INT(_hw_spacemit_p1, OID_AUTO, reset_deadman, CTLFLAG_RWTUN,
    &spacemit_p1_reset_deadman, 0,
    "Seconds after sync before resetting the SoC anyway if shutdown stalls. "
    "Default 15; 0 disables the fallback.");

struct p1_reg_sc {
	struct regnode		*regnode;
	device_t		base_dev;
	const struct p1_regdef	*def;
};

static int
p1_read1(device_t dev, uint8_t reg, uint8_t *val)
{

	return (iicdev_readfrom(dev, reg, val, 1, IIC_INTRWAIT));
}

static int
p1_write1(device_t dev, uint8_t reg, uint8_t val)
{

	return (iicdev_writeto(dev, reg, &val, 1, IIC_INTRWAIT));
}

/*
 * ROUND 29 -- STOP-separated register read.
 *
 * The default iicdev_readfrom() issues a combined transfer with a REPEAT-START
 * (Sr) between the register-pointer write and the read (msgs[0] IIC_M_NOSTOP ->
 * msgs[1] IIC_M_RD).  On the SpacemiT P1, accesses to the RTC sub-block
 * register 0x1d return IIC_ETIMEOUT (the transfer STALLS -- it is not
 * NAK/ENOACK), while ordinary regulator-page registers read fine.  That
 * fingerprint fits an RTC bank that does not accept the repeated-start.  This
 * helper does two SEPARATE, STOP-terminated transfers instead: write the
 * register pointer (full START..STOP), then a fresh START read.
 */
static int
p1_read1_stopsep(device_t dev, uint8_t reg, uint8_t *val)
{
	struct iic_msg wmsg, rmsg;
	uint8_t slaveaddr;
	int error;

	slaveaddr = iicbus_get_addr(dev);

	/* Transfer 1: write the register pointer, terminated by STOP. */
	wmsg.slave = slaveaddr;
	wmsg.flags = IIC_M_WR;		/* normal START..STOP, no NOSTOP */
	wmsg.len   = 1;
	wmsg.buf   = &reg;
	error = iicbus_transfer_excl(dev, &wmsg, 1, IIC_INTRWAIT);
	if (error != 0)
		return (error);

	/* Transfer 2: fresh START read. */
	rmsg.slave = slaveaddr;
	rmsg.flags = IIC_M_RD;
	rmsg.len   = 1;
	rmsg.buf   = val;
	return (iicbus_transfer_excl(dev, &rmsg, 1, IIC_INTRWAIT));
}

static int
p1_read(device_t dev, uint8_t reg, uint8_t *buf, uint8_t len)
{

	return (iicdev_readfrom(dev, reg, buf, len, IIC_INTRWAIT));
}

/*
 * Format the P1 rail-control window (0x47-0x7e) as a hex string into out.
 * Read via the driver's own iicbus access (it owns 0x41).  Returns 0 on
 * success.  Used by the sysctl and by attach/post-sweep logging.
 *
 * Registers are read ONE BYTE AT A TIME with the proven single-byte path
 * (p1_read1) rather than one 56-byte block read.  The first cut used a single
 * 56-byte iicdev_readfrom and produced NOTHING on hardware (both the sysctl and
 * the boot-time log were empty) -- the long repeated-start block read did not
 * complete.  Single-byte reads are exactly what the regulator regnode code uses
 * successfully at 0x41, so read the window a byte at a time.
 */
static int
p1_dump_rail_regs(device_t dev, char *out, size_t outlen)
{
	uint8_t val;
	int error, i, n;

	n = 0;
	for (i = 0; i < P1_RAIL_DUMP_LEN; i++) {
		error = p1_read1(dev, (uint8_t)(P1_RAIL_DUMP_START + i), &val);
		if (error != 0)
			return (error);
		if ((size_t)n + 3 > outlen)
			break;
		n += snprintf(out + n, outlen - n, "%02x", val);
	}
	return (0);
}

/*
 * Read-only sysctl hw.spacemit_p1.rail_regs: dumps the current 0x47-0x7e
 * window as a hex string.  Sample it twice (any interval) and diff -- identical
 * output proves the driver + the regulator core's disable-unused sweep changed
 * no rail's enable or voltage.
 */
static int
p1_sysctl_rail_regs(SYSCTL_HANDLER_ARGS)
{
	char buf[P1_RAIL_DUMP_LEN * 2 + 16];
	struct spacemit_p1_softc *sc;
	int error;

	sc = p1_softc;
	if (sc == NULL) {
		snprintf(buf, sizeof(buf), "no-instance");
		return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
	}
	buf[0] = '\0';
	error = p1_dump_rail_regs(sc->dev, buf, sizeof(buf));
	if (error != 0)
		snprintf(buf, sizeof(buf), "read-error-%d", error);
	return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
}
SYSCTL_PROC(_hw_spacemit_p1, OID_AUTO, rail_regs,
    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, 0,
    p1_sysctl_rail_regs, "A",
    "P1 rail-control registers 0x47-0x7e (hex). Sample twice + diff: identical "
    "== zero rail-state change under allow_rail_changes=0.");

/*
 * hw.spacemit_p1.clk32k_out: live-read RTC_CTRL(0x1d) and report the
 * out_32k_en bit -- the PMIC 32KOUT (pin50) that feeds the AP6256 WiFi LPO.
 * This is the definitive on/off proof that the WiFi sleep clock is running.
 */
static int
p1_sysctl_clk32k_out(SYSCTL_HANDLER_ARGS)
{
	char buf[96];
	struct spacemit_p1_softc *sc;
	uint8_t val;

	sc = p1_softc;
	if (sc == NULL) {
		snprintf(buf, sizeof(buf), "no-instance");
		return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
	}
	if (p1_read1(sc->dev, P1_RTC_CTRL, &val) != 0) {
		/*
		 * The live i2c read can fail from an MPSAFE sysctl context
		 * (bus busy / non-sleepable); fall back to the value cached at
		 * attach, which already reflects the out_32k_en enable we did.
		 */
		val = sc->rtc_ctl_at_attach;
		snprintf(buf, sizeof(buf),
		    "RTC_CTRL(0x1d)=0x%02x(cached) crystal_en=%d out_32k_en=%d "
		    "rtc_en=%d", val, !!(val & P1_RTC_CRYSTAL_EN),
		    !!(val & P1_RTC_OUT_32K_EN), !!(val & P1_RTC_EN));
	} else
		snprintf(buf, sizeof(buf),
		    "RTC_CTRL(0x1d)=0x%02x crystal_en=%d out_32k_en=%d rtc_en=%d",
		    val, !!(val & P1_RTC_CRYSTAL_EN),
		    !!(val & P1_RTC_OUT_32K_EN), !!(val & P1_RTC_EN));
	return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
}
SYSCTL_PROC(_hw_spacemit_p1, OID_AUTO, clk32k_out,
    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, 0,
    p1_sysctl_clk32k_out, "A",
    "PMIC RTC_CTRL(0x1d): out_32k_en is the 32KOUT feeding the AP6256 WiFi LPO");

/*
 * Post-sweep baseline: runs at SI_SUB_LAST + SI_ORDER_ANY -- AFTER the
 * regulator core's disable-unused sweep -- and logs the rail window.  Compared
 * against the attach-time log, identical values prove the sweep (and attach)
 * touched no rail.
 */
static void
p1_log_rail_regs_post_sweep(void *arg __unused)
{
	char buf[P1_RAIL_DUMP_LEN * 2 + 1];

	if (p1_softc == NULL)
		return;
	buf[0] = '\0';
	if (p1_dump_rail_regs(p1_softc->dev, buf, sizeof(buf)) == 0)
		device_printf(p1_softc->dev,
		    "rail regs 0x47-0x7e post-sweep: %s\n", buf);
}
SYSINIT(p1_rail_post_sweep, SI_SUB_LAST, SI_ORDER_ANY,
    p1_log_rail_regs_post_sweep, NULL);

static int
p1_write(device_t dev, uint8_t reg, uint8_t *buf, uint8_t len)
{

	return (iicdev_writeto(dev, reg, buf, len, IIC_INTRWAIT));
}

/* Convert a raw selector to microvolts for this regulator. */
static int
p1_sel_to_uvolt(const struct p1_regdef *def, uint8_t sel)
{

	if (def->type == P1_TYPE_BUCK) {
		if (sel <= BUCK_R1_SELMAX)
			return (BUCK_R1_MIN + sel * BUCK_R1_STEP);
		if (sel <= BUCK_R2_SELMAX)
			return (BUCK_R2_MIN + (sel - (BUCK_R1_SELMAX + 1)) *
			    BUCK_R2_STEP);
		return (BUCK_R2_MIN + (BUCK_R2_SELMAX - (BUCK_R1_SELMAX + 1)) *
		    BUCK_R2_STEP);
	}
	/* LDO */
	if (sel < LDO_SELMIN)
		return (LDO_MIN);
	if (sel > LDO_SELMAX)
		sel = LDO_SELMAX;
	return (LDO_MIN + (sel - LDO_SELMIN) * LDO_STEP);
}

/* Choose the lowest selector whose voltage is >= min_uvolt and <= max_uvolt. */
static int
p1_uvolt_to_sel(const struct p1_regdef *def, int min_uvolt, int max_uvolt,
    uint8_t *selp)
{
	int sel, selmin, selmax, uv;

	if (def->type == P1_TYPE_BUCK) {
		selmin = 0;
		selmax = BUCK_R2_SELMAX;
	} else {
		selmin = LDO_SELMIN;
		selmax = LDO_SELMAX;
	}
	for (sel = selmin; sel <= selmax; sel++) {
		uv = p1_sel_to_uvolt(def, (uint8_t)sel);
		if (uv >= min_uvolt) {
			if (uv > max_uvolt)
				return (EINVAL);
			*selp = (uint8_t)sel;
			return (0);
		}
	}
	return (EINVAL);
}

static int
p1_regnode_init(struct regnode *regnode)
{

	return (0);
}

static int
p1_regnode_enable(struct regnode *regnode, bool enable, int *udelay)
{
	struct p1_reg_sc *sc;
	uint8_t val;

	sc = regnode_get_softc(regnode);
	*udelay = 0;

	/*
	 * SAFETY: never disable a rail unless explicitly allowed.  Report
	 * success so the core's disable-unused sweep does not error, but do
	 * not actually cut power -- see spacemit_p1_allow_rail_changes.
	 */
	if (!enable && !spacemit_p1_allow_rail_changes)
		return (0);

	if (p1_read1(sc->base_dev, sc->def->enable_reg, &val) != 0)
		return (ENXIO);
	if (enable)
		val |= sc->def->enable_mask;
	else
		val &= ~sc->def->enable_mask;
	return (p1_write1(sc->base_dev, sc->def->enable_reg, val));
}

static int
p1_regnode_status(struct regnode *regnode, int *status)
{
	struct p1_reg_sc *sc;
	uint8_t val;

	sc = regnode_get_softc(regnode);
	if (p1_read1(sc->base_dev, sc->def->enable_reg, &val) != 0)
		return (ENXIO);
	*status = (val & sc->def->enable_mask) ? REGULATOR_STATUS_ENABLED : 0;
	return (0);
}

static int
p1_regnode_set_voltage(struct regnode *regnode, int min_uvolt, int max_uvolt,
    int *udelay)
{
	struct p1_reg_sc *sc;
	uint8_t sel, val;
	int error;

	sc = regnode_get_softc(regnode);
	*udelay = 0;

	/*
	 * SAFETY: do not change any rail voltage unless explicitly allowed.
	 * Firmware sets the correct operating voltages; a wrong write to a
	 * CPU/DRAM/SoC rail can hard-brick the board.
	 */
	if (!spacemit_p1_allow_rail_changes)
		return (0);

	error = p1_uvolt_to_sel(sc->def, min_uvolt, max_uvolt, &sel);
	if (error != 0)
		return (error);
	if (p1_read1(sc->base_dev, sc->def->vsel_reg, &val) != 0)
		return (ENXIO);
	val &= ~sc->def->vsel_mask;
	val |= sel & sc->def->vsel_mask;
	return (p1_write1(sc->base_dev, sc->def->vsel_reg, val));
}

static int
p1_regnode_get_voltage(struct regnode *regnode, int *uvolt)
{
	struct p1_reg_sc *sc;
	uint8_t val;

	sc = regnode_get_softc(regnode);
	if (p1_read1(sc->base_dev, sc->def->vsel_reg, &val) != 0)
		return (ENXIO);
	*uvolt = p1_sel_to_uvolt(sc->def, val & sc->def->vsel_mask);
	return (0);
}

static regnode_method_t p1_regnode_methods[] = {
	REGNODEMETHOD(regnode_init,		p1_regnode_init),
	REGNODEMETHOD(regnode_enable,		p1_regnode_enable),
	REGNODEMETHOD(regnode_status,		p1_regnode_status),
	REGNODEMETHOD(regnode_set_voltage,	p1_regnode_set_voltage),
	REGNODEMETHOD(regnode_get_voltage,	p1_regnode_get_voltage),
	REGNODEMETHOD(regnode_check_voltage,	regnode_method_check_voltage),
	REGNODEMETHOD_END
};
DEFINE_CLASS_1(p1_regnode, p1_regnode_class, p1_regnode_methods,
    sizeof(struct p1_reg_sc), regnode_class);

static void
p1_attach_regulators(struct spacemit_p1_softc *sc)
{
	struct regnode_init_def initdef;
	struct regnode *regnode;
	struct p1_reg_sc *reg_sc;
	phandle_t rnode, child;
	unsigned int i;

	rnode = ofw_bus_find_child(ofw_bus_get_node(sc->dev), "regulators");
	if (rnode <= 0)
		return;

	for (i = 0; i < nitems(p1_regdefs); i++) {
		child = ofw_bus_find_child(rnode, p1_regdefs[i].name);
		if (child == 0 || OF_hasprop(child, "regulator-name") != 1)
			continue;

		memset(&initdef, 0, sizeof(initdef));
		if (regulator_parse_ofw_stdparam(sc->dev, child, &initdef) != 0)
			continue;
		initdef.id = p1_regdefs[i].id;
		initdef.ofw_node = child;

		regnode = regnode_create(sc->dev, &p1_regnode_class, &initdef);
		if (regnode == NULL) {
			device_printf(sc->dev, "cannot create regulator %s\n",
			    p1_regdefs[i].name);
			continue;
		}
		reg_sc = regnode_get_softc(regnode);
		reg_sc->regnode = regnode;
		reg_sc->base_dev = sc->dev;
		reg_sc->def = &p1_regdefs[i];
		regnode_register(regnode);
		if (bootverbose)
			device_printf(sc->dev, "regulator %s attached\n",
			    p1_regdefs[i].name);
	}
}

/*
 * clock_if(9): the P1 RTC.  Time fields are plain integers (not BCD).  The
 * hardware latch is buggy, so reads loop until two consecutive snapshots
 * agree (per the mainline driver).
 */
static int
spacemit_p1_gettime(device_t dev, struct timespec *ts)
{
	struct clocktime ct;
	uint8_t time[6], prev0;
	uint8_t val;
	int error, tries;

	if (p1_read1(dev, P1_RTC_CTRL, &val) != 0)
		return (ENXIO);
	if ((val & P1_RTC_EN) == 0)
		return (EINVAL);	/* RTC not running / not set */

	if ((error = p1_read(dev, P1_RTC_TIME, time, sizeof(time))) != 0)
		return (error);
	for (tries = P1_RTC_READ_TRIES; tries > 0; tries--) {
		prev0 = time[0];
		if ((error = p1_read(dev, P1_RTC_TIME, time,
		    sizeof(time))) != 0)
			return (error);
		if (time[0] == prev0)
			break;
	}
	if (tries == 0)
		return (EIO);

	ct.nsec = 0;
	ct.sec = time[0] & 0x3f;
	ct.min = time[1] & 0x3f;
	ct.hour = time[2] & 0x1f;
	ct.day = (time[3] & 0x1f) + 1;
	ct.mon = (time[4] & 0x0f) + 1;		/* clocktime mon is 1-based */
	ct.year = 2000 + (time[5] & 0x3f);
	ct.dow = -1;

	return (clock_ct_to_ts(&ct, ts));
}

static int
spacemit_p1_settime(device_t dev, struct timespec *ts)
{
	struct clocktime ct;
	uint8_t time[6], val;
	int error;

	clock_ts_to_ct(ts, &ct);

	time[0] = ct.sec;
	time[1] = ct.min;
	time[2] = ct.hour;
	time[3] = ct.day - 1;
	time[4] = ct.mon - 1;			/* register mon is 0-based */
	time[5] = (ct.year >= 2000) ? (ct.year - 2000) : 0;

	/* Disable the RTC while updating, then re-enable. */
	if (p1_read1(dev, P1_RTC_CTRL, &val) != 0)
		return (ENXIO);
	if ((error = p1_write1(dev, P1_RTC_CTRL, val & ~P1_RTC_EN)) != 0)
		return (error);
	if ((error = p1_write(dev, P1_RTC_TIME, time, sizeof(time))) != 0)
		return (error);	/* leave disabled on failure, per mainline */
	return (p1_write1(dev, P1_RTC_CTRL, val | P1_RTC_EN));
}

/*
 * Do the actual PMIC hand-off.  Shared by the shutdown_final handler and the
 * deadman, so both take exactly the same path.
 */
static void
p1_pmic_handover(struct spacemit_p1_softc *sc, uint8_t bit)
{
	uint8_t val;

	if (p1_read1(sc->dev, P1_PWR_CTRL2, &val) != 0)
		val = 0;
	if (p1_write1(sc->dev, P1_PWR_CTRL2, val | bit) != 0) {
		device_printf(sc->dev, "PWR_CTRL2 write failed\n");
		return;
	}

	/*
	 * The PMIC acts asynchronously.  Give it time to pull the rails rather
	 * than racing on into shutdown_reset() -> cpu_reset().  Bounded, so a
	 * PMIC that ignores us still falls through to the old behaviour
	 * instead of wedging here.
	 */
	DELAY(1000000);
}

/*
 * Which PWR_CTRL2 bit a given howto asks for, or 0 for "not ours".
 */
static uint8_t
p1_howto_bit(int howto)
{

	if ((howto & RB_POWEROFF) != 0)
		return (P1_PWR_CTRL2_SHUTDOWN);
	if ((howto & (RB_HALT | RB_POWERCYCLE)) == RB_HALT)
		return (0);
	if (spacemit_p1_reset_on_reboot == 0)
		return (0);
	return (P1_PWR_CTRL2_RST);
}

/*
 * Deadman: runs on the shared taskqueue thread, so it may sleep -- which it
 * must, because every register access here is an iicbus transfer issued with
 * IIC_INTRWAIT.
 */
static void
p1_reset_deadman(void *arg, int pending __unused)
{
	struct spacemit_p1_softc *sc = arg;
	uint8_t bit;

	bit = p1_howto_bit(sc->deadman_howto);
	if (bit == 0)
		return;

	device_printf(sc->dev,
	    "shutdown stalled for %ds; handing over to the PMIC\n",
	    spacemit_p1_reset_deadman);
	p1_pmic_handover(sc, bit);
}

/*
 * shutdown_post_sync(9) handler, at SHUTDOWN_PRI_FIRST: arm the deadman
 * before any device teardown gets a chance to stall, then get out of the way
 * so the normal shutdown path can run to completion if it is able to.
 */
static void
p1_shutdown_arm(void *arg, int howto)
{
	struct spacemit_p1_softc *sc = arg;

	if (spacemit_p1_reset_deadman <= 0)
		return;
	if (p1_howto_bit(howto) == 0)
		return;

	sc->deadman_howto = howto;
	taskqueue_enqueue_timeout(taskqueue_thread, &sc->deadman,
	    hz * spacemit_p1_reset_deadman);
}

/*
 * shutdown_final(9) handler: hand the machine over to the PMIC.
 *
 *   RB_POWEROFF	 -> PWR_CTRL2 bit 2, the PMIC's shutdown request.  This
 *			    board has no working SBI/PSCI poweroff either.
 *   RB_POWERCYCLE	 -> PWR_CTRL2 bit 1 (a PMIC reset *is* a power cycle).
 *   plain reboot	 -> PWR_CTRL2 bit 1, the PMIC's reset request.
 *   RB_HALT on its own	 -> nothing.  The operator asked for the machine to
 *			    stop, not to come back; leave it to shutdown_halt().
 *
 * Why this lives in a shutdown_final handler and not in cpu_reset():
 * cpu_reset() runs from shutdown_reset() with smp_ipi_mtx held, i.e. with
 * interrupts disabled, and every register access here is an iicbus transfer
 * issued with IIC_INTRWAIT -- it sleeps waiting on the controller interrupt.
 * It cannot run from that context.  shutdown_final runs earlier, with
 * interrupts still live, which is the same reason the poweroff path has
 * always been here.  The consequence is that a reset from ddb ("reset", which
 * calls cpu_reset() directly) still hangs; only kern_reboot(9) paths are
 * covered.
 *
 * The bit assignments are cross-checked against two independent vendor
 * binaries, neither of which was copied:
 *   - ky_restart_notify() and ky_pm_power_off() in the vendor Linux
 *     6.6.63-ky kernel: regmap_update_bits() on register 0x7e, value 0x02 for
 *     restart and 0x04 for power-off.
 *   - the "spacemit-p1-reset" sbi_system_reset_device in the OpenSBI v1.9
 *     fw_dynamic this board boots: reads register 0x7e over i2c, then ORs
 *     0x02 for SBI_SRST_TYPE_{COLD,WARM}_REBOOT and 0x04 for _SHUTDOWN.
 */
static void
p1_shutdown_final(void *arg, int howto)
{
	struct spacemit_p1_softc *sc = arg;
	uint8_t bit;

	bit = p1_howto_bit(howto);
	if (bit == 0)
		return;

	if (bit == P1_PWR_CTRL2_RST)
		device_printf(sc->dev, "resetting the SoC via PWR_CTRL2\n");

	p1_pmic_handover(sc, bit);
}

static int
p1_regdev_map(device_t dev, phandle_t xref, int ncells, pcell_t *cells,
    intptr_t *num)
{

	/* Single-cell regulator specifier: the cell is the regulator id. */
	if (ncells == 0)
		*num = 0;
	else
		*num = cells[0];
	return (0);
}

/*
 * Enable the PMIC 32.768 kHz crystal + 32KOUT (RTC_CTRL 0x1d) that feeds the
 * AP6256 WiFi LPO.  Deferred to a config_intrhook so it runs AFTER iicbus
 * interrupts are live (see the round-28 note in attach): the same i2c write
 * from attach timed out (IIC_ETIMEOUT) because the bus could not service
 * IIC_INTRWAIT that early.
 *
 * The Orange Pi RV2 has NO discrete 32.768 kHz oscillator for WiFi -- the
 * board's only 32 kHz source is this PMIC's crystal, whose 32KOUT (pin50, net
 * PMIC_32K_OUT) routes to the AP6256 LPO.  Without it the AP6256 PMU cannot
 * sequence the ALP/HT backplane clocks and brcmfmac fails with "clock enable
 * timeout".  Set the full vendor bit pattern (crystal_en|out_32k_en|rtc_en|
 * rtc_clk_sel = 0x0f, exactly rtc-spt-pmic.c spacemit_rtc_probe), write-verify-
 * retry, then let the crystal settle.  Clock-output enable, not a rail ->
 * independent of allow_rail_changes.
 */
static void
p1_enable_lpo_32k(void *arg)
{
	struct spacemit_p1_softc *sc = arg;
	device_t dev = sc->dev;
	uint8_t rtc_ctl, want;
	int werr, rerr, attempt;

	uint8_t ctl_val;
	int ctl_err;
	bool use_stopsep = false;

	/* One-shot: tear the hook down so boot can proceed. */
	config_intrhook_disestablish(&sc->lpo_hook);

	/*
	 * ROUND 29 -- SCOPE the failure with a CONTROL read of a known-good
	 * regulator register (BUCK1 enable, 0x47) right before touching 0x1d.
	 * If the control read succeeds (werr=0) but 0x1d times out, the failure
	 * is specific to the RTC sub-block (page/repeated-start), not the whole
	 * PMIC.  If the control read ALSO times out, the bus is broken for this
	 * device at this point (broader problem).
	 */
	ctl_err = p1_read1(dev, P1_BUCK_BASE, &ctl_val);
	device_printf(dev,
	    "PMIC control read BUCK1_CTRL(0x%02x): err=%d val=0x%02x "
	    "(err=0 => regulator page OK)\n", P1_BUCK_BASE, ctl_err, ctl_val);

	/*
	 * Try the RTC_CTRL read with the default repeat-start path first; if it
	 * times out, retry with the STOP-separated path (some RTC banks reject
	 * repeated-start -> IIC_ETIMEOUT).  Whichever works is used for the rest.
	 */
	rerr = p1_read1(dev, P1_RTC_CTRL, &rtc_ctl);
	device_printf(dev,
	    "RTC_CTRL(0x1d) repeat-start read: err=%d val=0x%02x\n",
	    rerr, rerr == 0 ? rtc_ctl : 0);
	if (rerr != 0) {
		rerr = p1_read1_stopsep(dev, P1_RTC_CTRL, &rtc_ctl);
		device_printf(dev,
		    "RTC_CTRL(0x1d) STOP-separated read: err=%d val=0x%02x\n",
		    rerr, rerr == 0 ? rtc_ctl : 0);
		if (rerr == 0)
			use_stopsep = true;
	}
	if (rerr != 0) {
		rtc_ctl = 0;
		device_printf(dev,
		    "WARNING: RTC_CTRL(0x1d) unreadable by both paths\n");
	} else
		device_printf(dev,
		    "RTC_CTRL(0x1d)=0x%02x via %s "
		    "(crystal_en=%d out_32k_en=%d rtc_en=%d clk_sel=%d)\n",
		    rtc_ctl, use_stopsep ? "STOP-sep" : "repeat-start",
		    !!(rtc_ctl & P1_RTC_CRYSTAL_EN),
		    !!(rtc_ctl & P1_RTC_OUT_32K_EN),
		    !!(rtc_ctl & P1_RTC_EN),
		    !!(rtc_ctl & P1_RTC_CLK_SEL));

	want = rtc_ctl | P1_RTC_32K_ON;
	sc->rtc_ctl_at_attach = rtc_ctl;
	for (attempt = 1; attempt <= 5; attempt++) {
		uint8_t rb = 0;

		werr = p1_write1(dev, P1_RTC_CTRL, want);
		if (use_stopsep)
			rerr = p1_read1_stopsep(dev, P1_RTC_CTRL, &rb);
		else
			rerr = p1_read1(dev, P1_RTC_CTRL, &rb);
		device_printf(dev,
		    "PMIC 32KOUT enable attempt %d: wrote 0x%02x (werr=%d) "
		    "readback=0x%02x (rerr=%d)\n",
		    attempt, want, werr, rb, rerr);
		if (rerr == 0)
			sc->rtc_ctl_at_attach = rb;
		if (rerr == 0 && (rb & P1_RTC_OUT_32K_EN) != 0) {
			device_printf(dev,
			    "PMIC 32KOUT (AP6256 LPO) ENABLED: RTC_CTRL=0x%02x\n",
			    rb);
			break;
		}
		DELAY(20000);	/* 20 ms between attempts */
	}
	if (attempt > 5)
		device_printf(dev,
		    "WARNING: PMIC 32KOUT never latched (RTC_CTRL still 0x%02x) "
		    "-- AP6256 LPO may be absent\n", sc->rtc_ctl_at_attach);
	else
		DELAY(300000);	/* 300 ms 32.768 kHz crystal start-up */
}

static int
spacemit_p1_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (!ofw_bus_is_compatible(dev, "spacemit,p1"))
		return (ENXIO);
	device_set_desc(dev, "SpacemiT P1 PMIC");
	return (BUS_PROBE_DEFAULT);
}

static int
spacemit_p1_attach(device_t dev)
{
	struct spacemit_p1_softc *sc;

	sc = device_get_softc(dev);
	sc->dev = dev;
	p1_softc = sc;

	/*
	 * Log the rail-control window BEFORE registering any regnode -- this is
	 * the earliest-possible baseline.  Compared against the SI_SUB_LAST
	 * post-sweep log (and the rail_regs sysctl), identical values prove
	 * neither attach nor the regulator core's disable-unused sweep changed
	 * any rail's enable or voltage.
	 */
	{
		char buf[P1_RAIL_DUMP_LEN * 2 + 1];

		buf[0] = '\0';
		if (p1_dump_rail_regs(dev, buf, sizeof(buf)) == 0)
			device_printf(dev,
			    "rail regs 0x47-0x7e at attach: %s\n", buf);
	}

	p1_attach_regulators(sc);

	/*
	 * Enable the PMIC 32.768 kHz clock output (RTC_CTRL 0x1d) -- it feeds
	 * the AP6256 WiFi LPO -- from a config_intrhook, NOT here in attach.
	 *
	 * ROUND 28.  On hardware, doing the i2c write in attach returned
	 * IIC_ETIMEOUT (3) on EVERY read and write to 0x1d: this PMIC attaches
	 * before the SpacemiT iicbus controller can service IIC_INTRWAIT
	 * transfers (interrupts not yet live / bus not settled), so every
	 * blocking transfer times out.  The RTC clock registration, which reads
	 * the same register successfully, runs later once the bus is up (and the
	 * old 244 comment about a 56-byte block read "producing NOTHING" is the
	 * same early-bus smell).  config_intrhook_establish() fires after
	 * interrupts are enabled but before the root mount, which is exactly the
	 * window where iicbus INTRWAIT transfers work -- and still well before
	 * the SDIO/brcmfmac stack needs the LPO.  Defer the whole enable there.
	 */
	sc->lpo_hook.ich_func = p1_enable_lpo_32k;
	sc->lpo_hook.ich_arg = sc;
	if (config_intrhook_establish(&sc->lpo_hook) != 0) {
		device_printf(dev,
		    "WARNING: cannot establish 32KOUT intrhook; enabling inline "
		    "(may fail if bus not ready)\n");
		p1_enable_lpo_32k(sc);
	}

	/*
	 * Register the power-off/reset handler.  Ordering inside
	 * shutdown_final matters: shutdown_panic() sits at SHUTDOWN_PRI_LAST +
	 * 100 and offers "press a key to abort" after a panic, and
	 * shutdown_halt() sits at SHUTDOWN_PRI_LAST + 200 and blocks in
	 * cngetc().  Sit between them so we neither cut the abort window short
	 * nor get stranded behind the halt loop.
	 */
	sc->off_tag = EVENTHANDLER_REGISTER(shutdown_final, p1_shutdown_final,
	    sc, SHUTDOWN_PRI_LAST + 150);

	/* Arm the fallback after filesystem sync, before device teardown. */
	TIMEOUT_TASK_INIT(taskqueue_thread, &sc->deadman, 0, p1_reset_deadman,
	    sc);
	sc->arm_tag = EVENTHANDLER_REGISTER(shutdown_post_sync,
	    p1_shutdown_arm, sc, SHUTDOWN_PRI_FIRST);

	/*
	 * Register the power-off-persistent RTC.  1s resolution; higher
	 * priority than the SoC mrvl RTC so this (cold-boot-surviving) clock
	 * wins when both are present.
	 */
	clock_register_flags(dev, 1000000, 0);

	return (0);
}

static int
spacemit_p1_detach(device_t dev)
{
	struct spacemit_p1_softc *sc;

	sc = device_get_softc(dev);
	if (sc->off_tag != NULL)
		EVENTHANDLER_DEREGISTER(shutdown_final, sc->off_tag);
	if (sc->arm_tag != NULL) {
		EVENTHANDLER_DEREGISTER(shutdown_post_sync, sc->arm_tag);
		taskqueue_cancel_timeout(taskqueue_thread, &sc->deadman, NULL);
		taskqueue_drain_timeout(taskqueue_thread, &sc->deadman);
	}
	return (0);
}

static device_method_t spacemit_p1_methods[] = {
	DEVMETHOD(device_probe,		spacemit_p1_probe),
	DEVMETHOD(device_attach,	spacemit_p1_attach),
	DEVMETHOD(device_detach,	spacemit_p1_detach),

	/* Regulator device interface. */
	DEVMETHOD(regdev_map,		p1_regdev_map),

	/* RTC clock_if(9) interface. */
	DEVMETHOD(clock_gettime,	spacemit_p1_gettime),
	DEVMETHOD(clock_settime,	spacemit_p1_settime),

	DEVMETHOD_END
};

static driver_t spacemit_p1_driver = {
	"spacemit_p1",
	spacemit_p1_methods,
	sizeof(struct spacemit_p1_softc)
};

DRIVER_MODULE(spacemit_p1, iicbus, spacemit_p1_driver, NULL, NULL);
MODULE_DEPEND(spacemit_p1, iicbus, 1, 1, 1);
MODULE_DEPEND(spacemit_p1, regulator, 1, 1, 1);
MODULE_VERSION(spacemit_p1, 1);
