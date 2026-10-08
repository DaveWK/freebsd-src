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
 * SDHCI glue driver for the SpacemiT K1 (aka Ky X1) SoC.
 *
 * The controller is a standard SDHCI with vendor-specific PHY/pad control
 * registers in the same register block.  A software RESET_ALL clears the
 * PHY configuration, so it is re-applied after every such reset.  Register
 * semantics documented by the mainline Linux driver
 * (drivers/mmc/host/sdhci-of-k1.c) and its devicetree binding.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/types.h>
#include <sys/taskqueue.h>
#include <sys/module.h>
#include <sys/gpio.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>
#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/phy/phy.h>

#include <dev/gpio/gpiobusvar.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcbrvar.h>
#include <dev/mmc/mmcreg.h>

#include <dev/fdt/fdt_common.h>
#include <dev/mmc/mmc_fdt_helpers.h>

#include <dev/sdhci/sdhci.h>
#include <dev/sdhci/sdhci_fdt.h>
#include <dev/sdhci/sdhci_fdt_gpio.h>

#include "mmcbr_if.h"
#include "sdhci_if.h"
#include "gpio_if.h"

#include "opt_mmccam.h"

/*
 * EXT_PWR_EN: SoC GPIO 116 (bank3 offset20) gates the AP6256 WIFI_VCC33 (VBAT)
 * load switch on the Orange Pi RV2.  Active-high.  Not modelled in the DTS
 * (vendor rf-pwrseq pwr-gpios is empty; U-Boot drives it on Linux), so the pin
 * number is hardcoded for the SDIO slot.
 */
#define	SPACEMIT_WIFI_VBAT_GPIO	116

/* Vendor registers within the SDHCI block. */
#define	SPACEMIT_SDHC_OP_EXT_REG	0x108
#define	 SDHC_OVRRD_CLK_OEN		(1u << 11)
#define	 SDHC_FORCE_CLK_ON		(1u << 12)
#define	SPACEMIT_SDHC_LEGACY_CTRL_REG	0x10c
#define	 SDHC_GEN_PAD_CLK_ON		(1u << 6)
#define	SPACEMIT_SDHC_MMC_CTRL_REG	0x114
#define	 SDHC_MISC_INT_EN		(1u << 1)
#define	 SDHC_MISC_INT			(1u << 2)
#define	 SDHC_ENHANCE_STROBE_EN		(1u << 8)  /* HS400 enhanced strobe */
#define	 SDHC_MMC_HS400			(1u << 9)
#define	 SDHC_MMC_HS200			(1u << 10)
#define	 SDHC_MMC_CARD_MODE		(1u << 12) /* eMMC mode (vs SD) */
#define	SPACEMIT_SDHC_TX_CFG_REG	0x11c
#define	 SDHC_TX_INT_CLK_SEL		(1u << 30)
#define	 SDHC_TX_MUX_SEL		(1u << 31)
/*
 * TX/RX delay-line registers (vendor sdhci-of-x1.c).  The SDIO slot's
 * host->card (TX) data path needs a fixed delay code even at 25 MHz on this
 * board (the vendor DTS gives &sdhci1 ky,tx_delaycode=0x9f, ky,tx_dline_reg=0),
 * or block WRITES fail with DAT_CRC while reads are fine.
 */
#define	SPACEMIT_SDHC_DLINE_CTRL_REG	0x130
#define	 SDHC_DLINE_PU			(1u << 0)
#define	 SDHC_RX_DLINE_CODE_SHIFT	16
#define	 SDHC_RX_DLINE_CODE_MASK	0xffu
#define	 SDHC_TX_DLINE_CODE_SHIFT	24
#define	 SDHC_TX_DLINE_CODE_MASK	0xffu
#define	SPACEMIT_SDHC_DLINE_CFG_REG	0x134
#define	 SDHC_TX_DLINE_REG_SHIFT	16
#define	 SDHC_TX_DLINE_REG_MASK		0xffu
#define	SPACEMIT_SDIO_TX_DELAYCODE	0x9fu	/* vendor &sdhci1 ky,tx_delaycode */
#define	SPACEMIT_SDIO_TX_DLINE_REG	0x00u	/* vendor &sdhci1 ky,tx_dline_reg */
#define	SPACEMIT_SDHC_PHY_CTRL_REG	0x160
#define	 SDHC_PHY_FUNC_EN		(1u << 0)
#define	 SDHC_PHY_PLL_LOCK		(1u << 1)
#define	SPACEMIT_SDHC_PHY_PADCFG_REG	0x178
#define	 SDHC_PHY_DRIVE_SEL_MASK	0x7u
#define	 SDHC_PHY_DRIVE_SEL_DEFAULT	4u
#define	 SDHC_RX_BIAS_CTRL		(1u << 5)
/* PHY DLL block -- HS400/enhanced-strobe only (eMMC). */
#define	SPACEMIT_SDHC_PHY_DLLCFG	0x168
#define	 SDHC_DLL_PREDLY_NUM		0x04
#define	 SDHC_DLL_FULLDLY_RANGE		0x10
#define	 SDHC_DLL_VREG_CTRL		0x40
#define	 SDHC_DLL_ENABLE		0x80000000u
#define	SPACEMIT_SDHC_PHY_DLLCFG1	0x16c
#define	 SDHC_DLL_REG1_CTRL		0x92
#define	SPACEMIT_SDHC_PHY_DLLSTS	0x170
#define	 SDHC_DLL_LOCK_STATE		(1u << 0)

static struct ofw_compat_data compat_data[] = {
	{ "spacemit,k1-sdhci",	1 },
	{ NULL,			0 }
};

/*
 * OPTIONAL conservative-SD-timing knobs (default OFF -> current behavior).
 *
 * The microSD on the Orange Pi RV2 is marginal: vendor U-Boot already logs
 * "mmc block read error" and "clk wait timeout" at firmware level (before any
 * OS driver), and FreeBSD has hit ffs_fsfail panics on syncs.  Our driver is
 * NOT overdriving the card -- it negotiates high-speed ~34 MHz on its own and
 * our TX-clock setup matches mainline for that mode -- so this is primarily a
 * marginal-card mitigation, not a driver bug fix.  These tunables give the
 * card a lower, safer operating point without changing the DTB or the default:
 *
 *   hw.sdhci.spacemit.conservative_clock_hz  (default 0 = unchanged)
 *       If nonzero, cap the base clock reported to the SDHCI layer to this
 *       value so it computes divisors against a lower ceiling.  E.g. 25000000
 *       pins the card near default-speed.
 *
 *   hw.sdhci.spacemit.no_highspeed  (default 0 = unchanged)
 *       If nonzero, mask off SDHCI_CAN_DO_HISPD so the card stays in
 *       default-speed (25 MHz) mode -- the most conservative safe timing.
 *
 * Set at the loader prompt / loader.conf for a test boot; leave unset for the
 * normal (current) configuration.
 */
static u_int spacemit_sd_conservative_clock_hz = 0;
static int spacemit_sd_no_highspeed = 0;
SYSCTL_DECL(_hw_sdhci);
SYSCTL_NODE(_hw_sdhci, OID_AUTO, spacemit, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "SpacemiT K1 SDHCI");
SYSCTL_UINT(_hw_sdhci_spacemit, OID_AUTO, conservative_clock_hz,
    CTLFLAG_RDTUN, &spacemit_sd_conservative_clock_hz, 0,
    "If nonzero, cap the SD base clock (Hz) for a more conservative, more "
    "reliable timing on marginal cards. Default 0 = unchanged.");
SYSCTL_INT(_hw_sdhci_spacemit, OID_AUTO, no_highspeed,
    CTLFLAG_RDTUN, &spacemit_sd_no_highspeed, 0,
    "If nonzero, disable SD high-speed (force default-speed 25 MHz) for "
    "marginal cards. Default 0 = unchanged.");

/* Bound cached eMMC map maintenance; zero restores maxphys sizing. */
static int spacemit_emmc_sdma_kib = 128;
SYSCTL_INT(_hw_sdhci_spacemit, OID_AUTO, emmc_sdma_kib,
    CTLFLAG_RDTUN, &spacemit_emmc_sdma_kib, 0,
    "eMMC SDMA buffer KiB: 0=maxphys, 4/8/16/32/64/128/256/512; default 128");

/*
 * ROUND 34 test lever for the SDIO (WiFi) slot firmware-download write failure.
 * If nonzero, cap the SDIO base clock so post-enumeration the card runs slower
 * than the 25 MHz the MMCCAM SDIO probe selects -- to test whether CMD53 block
 * writes fail due to too-fast data timing.  SDIO slot only; root SD untouched.
 * Set at the loader (hw.sdhci.spacemit.sdio_clock_hz=...) for a test boot.
 */
static u_int spacemit_sdio_clock_hz = 0;
SYSCTL_UINT(_hw_sdhci_spacemit, OID_AUTO, sdio_clock_hz,
    CTLFLAG_RDTUN, &spacemit_sdio_clock_hz, 0,
    "If nonzero, cap the SDIO (WiFi) slot base clock (Hz) to test whether "
    "CMD53 block-write failures are data-timing related. Default 0 = unchanged.");

/*
 * ROUND 37 -- LIVE-sweepable SDIO data-path timing tunables (SDIO slot only),
 * read at every clock program so a reboot with a different loader.conf value
 * re-times the bus without a rebuild.  These let us characterize the CMD53
 * DAT-line failure (the vendor's own tx_delaycode=0x9f did not fix it):
 *   sdio_tx_delaycode  0..255  TX DLINE code (default 0x9f = vendor &sdhci1)
 *   sdio_rx_delaycode  -1/0..255  RX DLINE code (-1 = don't program RX; the
 *                       vendor only RX-tunes >=100 MHz, so default off)
 *   sdio_tx_dline      1/0     select the delay-line TX path (TX_MUX_SEL);
 *                       0 = direct path (test whether the DLINE itself hurts)
 *   sdio_force_1bit    1/0     mask CAN_DO_4BIT so the bus stays 1-bit -- test
 *                       whether 4-bit DAT1-3 signalling is the culprit
 */
static int spacemit_sdio_tx_delaycode = 0x9f;
static int spacemit_sdio_rx_delaycode = -1;
static int spacemit_sdio_tx_dline = 1;
static int spacemit_sdio_force_1bit = 0;
/*
 * ROUND 41 -- per-command SDIO CMD52/53 issue/complete trace, DEFAULT OFF.
 * These lines flood the dmesg ring (thousands of them during the 495 KB
 * firmware download), burying brcmfmac's own messages.  Off by default; the
 * low-volume SDIO DATA-error line + CMD53 counters stay always-on.  Toggle
 * with hw.sdhci.spacemit.sdio_cmd_trace=1 when the per-command detail is needed.
 */
static int spacemit_sdio_cmd_trace = 0;
SYSCTL_INT(_hw_sdhci_spacemit, OID_AUTO, sdio_cmd_trace,
    CTLFLAG_RWTUN, &spacemit_sdio_cmd_trace, 0,
    "SDIO slot: 1 = per-command CMD52/53 issue/complete trace (noisy), "
    "0 = off (default). DATA-error line + CMD53 counters are always on.");
/*
 * ROUND 38 -- force PIO (disable DMA) on the SDIO slot.  No CMD53 (DAT-line
 * data-phase) transfer succeeds in ANY width at ANY TX delay while CMD52
 * (CMD-line, no data buffer) works perfectly -- the signature of an SDHCI
 * DMA/data-buffer cache-coherency bug on riscv64.  If nonzero, set
 * SDHCI_QUIRK_BROKEN_DMA on the SDIO controller so sdhci(4) uses the PIO
 * transfer path; if CMD53 then succeeds, it is a DMA/coherency bug (and the
 * firmware loads -> wlan0 even if slower).  SDIO slot only; the root-SD
 * controller is a separate instance and keeps DMA.
 */
static int spacemit_sdio_force_pio = 0;
SYSCTL_INT(_hw_sdhci_spacemit, OID_AUTO, sdio_force_pio,
    CTLFLAG_RDTUN, &spacemit_sdio_force_pio, 0,
    "SDIO slot: if nonzero, force PIO (SDHCI_QUIRK_BROKEN_DMA) to test whether "
    "the CMD53 data-phase failure is a DMA/cache-coherency bug.");
SYSCTL_INT(_hw_sdhci_spacemit, OID_AUTO, sdio_tx_delaycode,
    CTLFLAG_RWTUN, &spacemit_sdio_tx_delaycode, 0,
    "SDIO slot TX DLINE delaycode 0..255 (default 0x9f). Live-sweepable.");
SYSCTL_INT(_hw_sdhci_spacemit, OID_AUTO, sdio_rx_delaycode,
    CTLFLAG_RWTUN, &spacemit_sdio_rx_delaycode, 0,
    "SDIO slot RX DLINE delaycode 0..255, or -1 to not program RX (default -1).");
SYSCTL_INT(_hw_sdhci_spacemit, OID_AUTO, sdio_tx_dline,
    CTLFLAG_RWTUN, &spacemit_sdio_tx_dline, 0,
    "SDIO slot: 1 = use the delay-line TX path (TX_MUX_SEL), 0 = direct.");
SYSCTL_INT(_hw_sdhci_spacemit, OID_AUTO, sdio_force_1bit,
    CTLFLAG_RDTUN, &spacemit_sdio_force_1bit, 0,
    "SDIO slot: if nonzero, force 1-bit bus (mask CAN_DO_4BIT) to test 4-bit.");

/*
 * ROUND 37 -- CMD53 (SD_IO_RW_EXTENDED, DAT-line block/byte) success/fail
 * counters, exposed so we can answer 'has ANY CMD53 DAT-line transfer ever
 * succeeded, read or write?'  A data transfer that completes with DATA_END and
 * no data-error bit is a success; one that completes with a data-error bit is a
 * failure.  Split read vs write.
 */
static u_long spacemit_sdio_cmd53_rd_ok = 0, spacemit_sdio_cmd53_rd_err = 0;
static u_long spacemit_sdio_cmd53_wr_ok = 0, spacemit_sdio_cmd53_wr_err = 0;
SYSCTL_ULONG(_hw_sdhci_spacemit, OID_AUTO, sdio_cmd53_rd_ok, CTLFLAG_RD,
    &spacemit_sdio_cmd53_rd_ok, 0, "SDIO CMD53 DAT-line reads that succeeded");
SYSCTL_ULONG(_hw_sdhci_spacemit, OID_AUTO, sdio_cmd53_rd_err, CTLFLAG_RD,
    &spacemit_sdio_cmd53_rd_err, 0, "SDIO CMD53 DAT-line reads that data-errored");
SYSCTL_ULONG(_hw_sdhci_spacemit, OID_AUTO, sdio_cmd53_wr_ok, CTLFLAG_RD,
    &spacemit_sdio_cmd53_wr_ok, 0, "SDIO CMD53 DAT-line writes that succeeded");
SYSCTL_ULONG(_hw_sdhci_spacemit, OID_AUTO, sdio_cmd53_wr_err, CTLFLAG_RD,
    &spacemit_sdio_cmd53_wr_err, 0, "SDIO CMD53 DAT-line writes that data-errored");

struct sdhci_fdt_spacemit_softc {
	struct sdhci_fdt_softc	base;	/* must be first */
	uint32_t		io_freq;
	struct sdhci_fdt_gpio	*gpio;	/* cd-gpios card detect */
	struct mmc_helper	mmc_helper; /* vmmc/vqmmc + mmc-pwrseq */
	bool			non_removable; /* DT "non-removable" */
	bool			wp_disabled; /* DT "disable-wp" */
	bool			wp_inverted; /* DT "wp-inverted" */
	bool			sdio_slot; /* DT "no-mmc"+"no-sd": SDIO-only */

	/*
	 * WL_REG_ON power sequencing for the AP6256 SDIO WiFi module, driven
	 * directly from this driver instead of the upstream mmc_pwrseq(4)
	 * device (see the pwrseq comment in _update_ios).
	 *
	 * We drive the reset-gpio via the gpio CONTROLLER's kobj methods
	 * (GPIO_PIN_SET/SETFLAGS on pwrseq_gpiodev) rather than a reserved
	 * gpiobus pin: round 8 found pin 67 (WL_REG_ON) is already reserved by
	 * another gpiobus consumer, so gpio_pin_get_by_* returns EBUSY.  The
	 * controller-level methods need no reservation and sidestep that.
	 */
	device_t		pwrseq_gpiodev;	/* gpio controller device */
	uint32_t		pwrseq_pin;	/* WL_REG_ON pin number (67) */
	bool			pwrseq_activelow; /* reset-gpios ACTIVE_LOW */
	uint32_t		pwrseq_pon_delay_ms; /* post-power-on-delay-ms */
	uint32_t		pwrseq_poff_delay_us; /* power-off-delay-us */
	bool			pwrseq_powered; /* current REG_ON-on state */

	/*
	 * WIFI_VCC33 main (VBAT) rail enable for the AP6256.  On the Orange Pi
	 * RV2 the module's 3.3V VBAT (WIFI_VCC33 -- internal LDOs, VDD_TCXO, the
	 * 37.4 MHz REF_CLK, WLAN core) is gated by a board load switch enabled
	 * by EXT_PWR_EN = SoC GPIO 116 (schematic p13; vendor DTS
	 * x1,pwr_on = <&gpio 116 0>, active-high).  U-Boot asserts it on Linux
	 * boot; FreeBSD must drive it high itself or the module has NO main
	 * power (no VBAT -> no REF_CLK -> dead -> CMD5 timeout, no CRC).  Driven
	 * via the gpio controller kobj like WL_REG_ON.
	 */
	device_t		vbat_gpiodev;	/* gpio controller device (gpio0) */
	uint32_t		vbat_pin;	/* EXT_PWR_EN pin number (116) */
	bool			vbat_on;	/* current WIFI_VCC33 enable state */

	/* Round-13/14 CMD-completion trace (SDIO slot). */
	uint32_t		dbg_last_cmd_intstat;
	uint32_t		dbg_last_dat_intstat; /* last data-error INT */
	bool			dbg_last_is_cmd53; /* last cmd was CMD53 */
	bool			dbg_last_cmd53_wr; /* last CMD53 was a write */
	uint32_t		dbg_last_datend_intstat; /* dedup DATA_END counting */
	uint8_t			dbg_last_cmd_opcode; /* last opcode issued */
};

/*
 * Restore the vendor PHY/pad configuration.  Required after RESET_ALL,
 * which clears it.  This is the SD-card variant of the sequence (the
 * eMMC-only MMC_CARD_MODE setup is not applied).
 */
static void
sdhci_fdt_spacemit_phy_setup(struct sdhci_fdt_softc *sc, int slotnum)
{
	struct resource *mem;
	uint32_t v;

	mem = sc->mem_res[slotnum];

	/*
	 * PHY block (0x160..0x178): per the vendor driver (sdhci-of-x1.c)
	 * and the vendor board DTS, only the eMMC instance (SDH2) has a
	 * functional PHY module; the SD (SDH0) and SDIO (SDH1) instances
	 * carry SDHCI_QUIRK2_BROKEN_PHY_MODULE ("sd/sdio only be
	 * SDHCI_QUIRK2_BROKEN_PHY_MODULE") and the vendor reset path never
	 * touches the PHY registers on them -- confirmed on hardware:
	 * PHY_CTRL/PHY_PADCFG read 0xffffffff on the SDIO slot (register
	 * block absent).  Skip the writes on the SDIO slot to match the
	 * vendor exactly; keep them on the root SD slot, where they have
	 * shipped since bring-up (harmless there, and that slot's behavior
	 * must not change).  The vendor's PHY-DLL init (0x168/0x16c/0x170)
	 * is HS400/enhanced-strobe-only, i.e. eMMC-only -- never needed for
	 * SDIO enumeration.
	 */
	if (!((struct sdhci_fdt_spacemit_softc *)sc)->sdio_slot) {
		v = bus_read_4(mem, SPACEMIT_SDHC_PHY_CTRL_REG);
		v |= SDHC_PHY_FUNC_EN | SDHC_PHY_PLL_LOCK;
		bus_write_4(mem, SPACEMIT_SDHC_PHY_CTRL_REG, v);

		v = bus_read_4(mem, SPACEMIT_SDHC_PHY_PADCFG_REG);
		v &= ~SDHC_PHY_DRIVE_SEL_MASK;
		v |= SDHC_RX_BIAS_CTRL | SDHC_PHY_DRIVE_SEL_DEFAULT;
		bus_write_4(mem, SPACEMIT_SDHC_PHY_PADCFG_REG, v);
	}

	/*
	 * eMMC card mode: the vendor sets MMC_CARD_MODE (MMC_CTRL bit12) on the
	 * eMMC (SDH2) instance so the controller/PHY run in eMMC mode -- the
	 * HS200/HS400(ES) path needs it.  eMMC only (non-removable, not the SDIO
	 * slot).  Re-applied after RESET_ALL with the rest of the PHY setup.
	 */
	if (((struct sdhci_fdt_spacemit_softc *)sc)->non_removable &&
	    !((struct sdhci_fdt_spacemit_softc *)sc)->sdio_slot) {
		v = bus_read_4(mem, SPACEMIT_SDHC_MMC_CTRL_REG);
		v |= SDHC_MMC_CARD_MODE;
		bus_write_4(mem, SPACEMIT_SDHC_MMC_CTRL_REG, v);
	}

	/*
	 * GEN_PAD_CLK_ON (LEGACY_CTRL bit6): kept on the root-SD slot where it
	 * has shipped since bring-up and the card works.  NOT applied to the
	 * SDIO slot: the vendor reset path for a BROKEN_PHY_MODULE host does
	 * ONLY "TX_CFG |= TX_INT_CLK_SEL" (ky_sdhci_reset) and uses
	 * GEN_PAD_CLK_ON solely inside the transient 74-clock burst -- never as
	 * a standing bit.  (Round 18.)
	 */
	if (!((struct sdhci_fdt_spacemit_softc *)sc)->sdio_slot) {
		v = bus_read_4(mem, SPACEMIT_SDHC_LEGACY_CTRL_REG);
		v |= SDHC_GEN_PAD_CLK_ON;
		bus_write_4(mem, SPACEMIT_SDHC_LEGACY_CTRL_REG, v);
	}

	/*
	 * Do NOT force the SD card clock permanently on (OP_EXT
	 * OVRRD_CLK_OEN | FORCE_CLK_ON).  A free-running, non-gated card clock
	 * shifts the TX-clock-vs-TX-data phase the pad drives, producing
	 * write-only CRC errors (reads are card-clocked and unaffected) at
	 * every bus speed -- observed on hardware as mmcsd "Bad CRC" ->
	 * ffs_fsfail at both 25 and 34 MHz.  The vendor BSP only pulses these
	 * transiently during a 1.8V voltage switch, never for SD data transfer,
	 * so leave auto clock-gating in effect -- FOR BOTH SLOTS.
	 *
	 * ROUND 18 -- the SDIO slot no longer forces the clock either.
	 *
	 * Rounds 1-17 held OP_EXT FORCE_CLK_ON|OVRRD_CLK_OEN permanently on the
	 * SDIO slot as a substitute for the vendor's 74-clock burst.  With the
	 * chip finally powered (round 16 gpio67=1) CMD5 still TIMED OUT with no
	 * CRC even on a COLD power-cycle -- i.e. a clocked command frame is not
	 * reaching the card.  Vendor ground truth (sdhci-of-x1.c): the SDIO
	 * slot runs with AUTO clock gating; ky_sdhci_set_clk_gate FORCES the
	 * clock (sets OVRRD_CLK_OEN|FORCE_CLK_ON) ONLY transiently, around a
	 * SD_SWITCH_VOLTAGE at 1.8V, then RECOVERS auto-gating in card_busy.
	 * It is never held.  A permanently forced, non-gated clock on this slot
	 * mis-phases exactly like the root-SD TX case -- the very failure this
	 * comment warns about -- so the CMD5 frame is clocked out wrong and the
	 * card answers nothing (timeout, no CRC).  Drop the permanent force:
	 * let the SDHCI core drive SDCLK the normal way (SDHCI_CLOCK_CARD_EN via
	 * sdhci_set_clock) exactly as it does for the working root-SD slot0.
	 *
	 * (History for reference -- the standard-SDHCI CDTEST card-present bits
	 * were tried rounds 2-3 and dropped round 4: the IP ignores them for its
	 * internal present state, and the vendor enumerates this module with
	 * PRESENT bit16 = 0.  Software-side always-present, get_card_present +
	 * MMCCAM discovery, is the correct Linux-NONREMOVABLE equivalent.)
	 *
	 * ROUND 20 -- do NOT force HOST_CONTROL2 S18 (1.8V signaling enable)
	 * here at init.
	 *
	 * Rounds 9-19 set S18 in phy_setup unconditionally, reasoning that the
	 * vendor set_uhs_signaling sets VDD_180 for SDIO hosts.  But the vendor
	 * sets it inside set_uhs_signaling, which the Linux mmc core calls only
	 * as part of a UHS TIMING change -- i.e. AFTER a successful
	 * CMD11/SD_SWITCH_VOLTAGE handshake; at the FIRST CMD5 the vendor has
	 * S18 = 0.  Per the SD/SDIO spec, HOST_CONTROL2 bit3 (1.8V Signal
	 * Enable) must be asserted ONLY after the CMD11 voltage switch
	 * completes; setting it prematurely makes the controller drive the
	 * CMD/DAT lines at 1.8V signalling levels while the card is still in its
	 * power-up (3.3V-signalling) state, so the CMD5 frame is at the wrong
	 * level and the card recognises nothing -- a command timeout with NO
	 * CRC, which is EXACTLY our symptom (r14-r19: chip powered, SDCLK 401kHz
	 * stable, VDD 1.8V, yet every CMD5 [TIMEOUT], never a CRC).
	 *
	 * Critically, FreeBSD's MMCCAM SDIO probe (cam/mmc/mmc_xpt.c
	 * PROBE_SDIO_INIT) issues CMD5 and NEVER performs a CMD11 voltage
	 * switch at all -- so if we force S18=1 up front, the bus is stuck at
	 * 1.8V signalling with no handshake ever reconciling it.  Leave S18 at
	 * its reset default (0) and let the card be probed at the controller's
	 * default signalling; the module's VDD/VDDIO 1.8V rail (PWR=0x0b, kept)
	 * is a separate concern from the CMD/DAT signalling-level bit.
	 *
	 * (If the AP6256 genuinely needs 1.8V *signalling* from power-up -- a
	 * fixed-1.8V embedded design with no 3.3V-signalling phase -- the
	 * correct place is a start_signal_voltage_switch equivalent, not a raw
	 * phy_setup force; that is the round-21 fallback if removing the force
	 * still times out.  The vendor_regs sysctl reports S18 so we can see the
	 * actual bit on hardware after this change.)
	 */

	/*
	 * Select the internal clock for the TX path.  Required for the
	 * default-speed/high-speed timings (up to SDR50); without it the
	 * transmit path is misclocked and writes fail with CRC errors
	 * although reads work.
	 */
	v = bus_read_4(mem, SPACEMIT_SDHC_TX_CFG_REG);
	v |= SDHC_TX_INT_CLK_SEL;
	bus_write_4(mem, SPACEMIT_SDHC_TX_CFG_REG, v);
}

/*
 * Debug sysctl: dev.sdhci_spacemit.<unit>.vendor_regs
 *
 * Dumps the vendor-specific and card-detect-relevant registers of slot 0
 * at read time (live, from hardware).  These are NOT in the standard
 * sdhci "dumpregs" output, and on riscv userland /dev/mem cannot map this
 * controller, so this driver sysctl is the only way to PROVE at runtime
 * whether the SDIO-slot overrides (OP_EXT FORCE_CLK_ON|OVRRD_CLK_OEN, the
 * CDTEST card-present bits in HOST_CONTROL, and S18 in HOST_CONTROL2)
 * actually stuck after the mmc init sequence -- the round-1 blind spot.
 */
static int
sdhci_fdt_spacemit_sysctl_vendor_regs(SYSCTL_HANDLER_ARGS)
{
	struct sdhci_fdt_spacemit_softc *sc = arg1;
	struct resource *mem;
	char buf[640];

	if (sc->base.num_slots < 1)
		return (SYSCTL_OUT(req, "no slots\n", 10));
	mem = sc->base.mem_res[0];

	snprintf(buf, sizeof(buf),
	    "OP_EXT(0x108)=0x%08x LEGACY(0x10c)=0x%08x MMC_CTRL(0x114)=0x%08x "
	    "TX_CFG(0x11c)=0x%08x "
	    "PHY_CTRL(0x160)=0x%08x PHY_PAD(0x178)=0x%08x | "
	    "HOST_CTL(0x28)=0x%02x PWR(0x29)=0x%02x HOST_CTL2(0x3e)=0x%04x "
	    "PRESENT(0x24)=0x%08x CLOCK(0x2c)=0x%04x | "
	    "sdio_slot=%d io_freq=%u FORCE_CLK_ON=%d OVRRD_CLK_OEN=%d "
	    "CDTEST(det|force)=%d|%d S18=%d | "
	    "pwrseq(gpio=%d on=%d pon=%ums poff=%uus) vbat(gpio=%d on=%d)\n",
	    bus_read_4(mem, SPACEMIT_SDHC_OP_EXT_REG),
	    bus_read_4(mem, SPACEMIT_SDHC_LEGACY_CTRL_REG),
	    bus_read_4(mem, SPACEMIT_SDHC_MMC_CTRL_REG),
	    bus_read_4(mem, SPACEMIT_SDHC_TX_CFG_REG),
	    bus_read_4(mem, SPACEMIT_SDHC_PHY_CTRL_REG),
	    bus_read_4(mem, SPACEMIT_SDHC_PHY_PADCFG_REG),
	    bus_read_1(mem, SDHCI_HOST_CONTROL),
	    bus_read_1(mem, SDHCI_POWER_CONTROL),
	    bus_read_2(mem, SDHCI_HOST_CONTROL2),
	    bus_read_4(mem, SDHCI_PRESENT_STATE),
	    bus_read_2(mem, SDHCI_CLOCK_CONTROL),
	    sc->sdio_slot, sc->io_freq,
	    !!(bus_read_4(mem, SPACEMIT_SDHC_OP_EXT_REG) & SDHC_FORCE_CLK_ON),
	    !!(bus_read_4(mem, SPACEMIT_SDHC_OP_EXT_REG) & SDHC_OVRRD_CLK_OEN),
	    !!(bus_read_1(mem, SDHCI_HOST_CONTROL) & SDHCI_CTRL_CARD_DET),
	    !!(bus_read_1(mem, SDHCI_HOST_CONTROL) & SDHCI_CTRL_FORCE_CARD),
	    !!(bus_read_2(mem, SDHCI_HOST_CONTROL2) & SDHCI_CTRL2_S18_ENABLE),
	    sc->pwrseq_gpiodev != NULL, sc->pwrseq_powered,
	    sc->pwrseq_pon_delay_ms, sc->pwrseq_poff_delay_us,
	    sc->vbat_gpiodev != NULL, sc->vbat_on);

	return (SYSCTL_OUT(req, buf, strlen(buf) + 1));
}

/*
 * Apply the fixed TX delay-line configuration for the SDIO slot, mirroring the
 * vendor tx-tuning sequence (sdhci-of-x1.c ky_sw_tx_set_dlinereg /
 * ky_sw_tx_set_delaycode / ky_sw_tx_tuning_prepare) but with the fixed
 * DTS-provided values, applied at 25 MHz too (not gated behind the >=100 MHz
 * RX-tuning threshold).
 *
 * ROUND 36.  TX_INT_CLK_SEL was proven live at the failing write (int_clk_sel=1)
 * yet the host->card 64-byte block WRITE still DAT_CRCs at 25 MHz -- the TX data
 * reaches the card mis-sampled.  The vendor board DTS carries a per-slot TX
 * delay for exactly this host (&sdhci1 ky,tx_delaycode=0x9f, ky,tx_dline_reg=0);
 * program it so the write data-vs-clock phase is correct.  Sequence:
 *   DLINE_CFG(0x134) TX_DLINE_REG[23:16] = tx_dline_reg (0)
 *   DLINE_CTRL(0x130) TX_DLINE_CODE[31:24] = tx_delaycode (0x9f)
 *   TX_CFG(0x11c) |= TX_MUX_SEL  (select the delay-line TX path)
 *   DLINE_CTRL(0x130) |= DLINE_PU (power up the delay line) + settle
 */
static void
sdhci_fdt_spacemit_apply_tx_delay(struct sdhci_fdt_spacemit_softc *sc,
    int slotnum)
{
	struct resource *mem = sc->base.mem_res[slotnum];
	uint32_t reg;
	uint8_t txcode = (uint8_t)(spacemit_sdio_tx_delaycode & 0xff);

	/*
	 * DLINE_CFG(0x134): TX_DLINE_REG[23:16] = 0 (vendor ky,tx_dline_reg);
	 * RX_DLINE_REG[7:0] = 0 unless an RX code is being programmed.
	 */
	reg = bus_read_4(mem, SPACEMIT_SDHC_DLINE_CFG_REG);
	reg &= ~(SDHC_TX_DLINE_REG_MASK << SDHC_TX_DLINE_REG_SHIFT);
	reg |= (SPACEMIT_SDIO_TX_DLINE_REG & SDHC_TX_DLINE_REG_MASK) <<
	    SDHC_TX_DLINE_REG_SHIFT;
	bus_write_4(mem, SPACEMIT_SDHC_DLINE_CFG_REG, reg);

	/*
	 * DLINE_CTRL(0x130): TX_DLINE_CODE[31:24] = txcode; optionally
	 * RX_DLINE_CODE[23:16] = rxcode (round-37 RX sweep).
	 */
	reg = bus_read_4(mem, SPACEMIT_SDHC_DLINE_CTRL_REG);
	reg &= ~(SDHC_TX_DLINE_CODE_MASK << SDHC_TX_DLINE_CODE_SHIFT);
	reg |= ((uint32_t)txcode << SDHC_TX_DLINE_CODE_SHIFT);
	if (spacemit_sdio_rx_delaycode >= 0) {
		reg &= ~(SDHC_RX_DLINE_CODE_MASK << SDHC_RX_DLINE_CODE_SHIFT);
		reg |= ((uint32_t)(spacemit_sdio_rx_delaycode & 0xff) <<
		    SDHC_RX_DLINE_CODE_SHIFT);
	}
	bus_write_4(mem, SPACEMIT_SDHC_DLINE_CTRL_REG, reg);

	/* Select or clear the delay-line TX path (TX_MUX_SEL) in TX_CFG. */
	reg = bus_read_4(mem, SPACEMIT_SDHC_TX_CFG_REG);
	if (spacemit_sdio_tx_dline != 0)
		reg |= SDHC_TX_MUX_SEL;
	else
		reg &= ~SDHC_TX_MUX_SEL;
	bus_write_4(mem, SPACEMIT_SDHC_TX_CFG_REG, reg);

	/* Power up the delay line and let it settle. */
	reg = bus_read_4(mem, SPACEMIT_SDHC_DLINE_CTRL_REG);
	reg |= SDHC_DLINE_PU;
	bus_write_4(mem, SPACEMIT_SDHC_DLINE_CTRL_REG, reg);
	DELAY(5);
}

/*
 * The controller's capabilities register reports no base clock frequency.
 * Inject the actual rate of the "io" clock so the SDHCI layer computes
 * correct divisors (mirrors the Linux driver's get_max_clock op).
 */
static uint32_t
sdhci_fdt_spacemit_read_4(device_t dev, struct sdhci_slot *slot,
    bus_size_t off)
{
	struct sdhci_fdt_spacemit_softc *sc;
	uint32_t val, base_hz;

	sc = device_get_softc(dev);
	val = bus_read_4(sc->base.mem_res[slot->num], off);

	/*
	 * ROUND 13 diagnostic: trace command COMPLETION on the SDIO slot.
	 *
	 * The core reads SDHCI_INT_STATUS(0x30) in the ISR to learn how a
	 * command finished.  Log it (deduped) when a command-response or
	 * command-error bit is present, so we SEE whether CMD5 completed with a
	 * response (RESPONSE bit) vs a command-timeout (TIMEOUT) vs a CRC/frame
	 * error -- the missing half of round 11's issue-only trace.  This tells
	 * "card partially responds (signaling)" from "card totally dead".
	 */
	if (sc->sdio_slot && spacemit_sdio_cmd_trace && off == SDHCI_INT_STATUS &&
	    (val & SDHCI_INT_CMD_MASK) != 0 &&
	    val != sc->dbg_last_cmd_intstat) {
		sc->dbg_last_cmd_intstat = val;
		device_printf(dev,
		    "SDIO CMD%u complete: INT=0x%08x [%s%s%s%s] RESP0=0x%08x\n",
		    sc->dbg_last_cmd_opcode, val,
		    (val & SDHCI_INT_RESPONSE) ? "RESPONSE " : "",
		    (val & SDHCI_INT_TIMEOUT) ? "TIMEOUT " : "",
		    (val & SDHCI_INT_CRC) ? "CRC " : "",
		    (val & (SDHCI_INT_END_BIT | SDHCI_INT_INDEX)) ? "IDX/END " :
		    "",
		    bus_read_4(sc->base.mem_res[slot->num], SDHCI_RESPONSE));
	}

	/*
	 * ROUND 34 diagnostic: trace DATA-phase errors on the SDIO slot.  The
	 * AP6256 enumerates and small reads work, but CMD53 block WRITES (the
	 * firmware download) fail with CAM error=5.  Decode the SDHCI data error
	 * bits (DATA_TIMEOUT / DATA_CRC / DATA_END_BIT) at completion so the
	 * capture shows WHY the write fails -- a data timeout (wrong clock/too
	 * slow), a data CRC (bad TX phase / 4-bit signalling), or an end-bit
	 * error.  Deduped separately from the command trace.
	 */
	if (sc->sdio_slot && off == SDHCI_INT_STATUS &&
	    (val & (SDHCI_INT_DATA_TIMEOUT | SDHCI_INT_DATA_CRC |
	    SDHCI_INT_DATA_END_BIT | SDHCI_INT_ACMD12ERR)) != 0 &&
	    val != sc->dbg_last_dat_intstat) {
		sc->dbg_last_dat_intstat = val;
		device_printf(dev,
		    "SDIO DATA error: INT=0x%08x [%s%s%s%s] CLOCK=0x%04x "
		    "HOST_CTL=0x%02x TX_CFG=0x%08x[int_clk_sel=%d mux_sel=%d] "
		    "DLINE_CTRL=0x%08x DLINE_CFG=0x%08x\n", val,
		    (val & SDHCI_INT_DATA_TIMEOUT) ? "DAT_TIMEOUT " : "",
		    (val & SDHCI_INT_DATA_CRC) ? "DAT_CRC " : "",
		    (val & SDHCI_INT_DATA_END_BIT) ? "DAT_END_BIT " : "",
		    (val & SDHCI_INT_ACMD12ERR) ? "ACMD12ERR " : "",
		    bus_read_2(sc->base.mem_res[slot->num], SDHCI_CLOCK_CONTROL),
		    bus_read_1(sc->base.mem_res[slot->num], SDHCI_HOST_CONTROL),
		    bus_read_4(sc->base.mem_res[slot->num], SPACEMIT_SDHC_TX_CFG_REG),
		    !!(bus_read_4(sc->base.mem_res[slot->num],
		    SPACEMIT_SDHC_TX_CFG_REG) & SDHC_TX_INT_CLK_SEL),
		    !!(bus_read_4(sc->base.mem_res[slot->num],
		    SPACEMIT_SDHC_TX_CFG_REG) & SDHC_TX_MUX_SEL),
		    bus_read_4(sc->base.mem_res[slot->num],
		    SPACEMIT_SDHC_DLINE_CTRL_REG),
		    bus_read_4(sc->base.mem_res[slot->num],
		    SPACEMIT_SDHC_DLINE_CFG_REG));

		/* ROUND 37: count a CMD53 DAT-line data-error (per direction). */
		if (sc->dbg_last_is_cmd53) {
			if (sc->dbg_last_cmd53_wr)
				spacemit_sdio_cmd53_wr_err++;
			else
				spacemit_sdio_cmd53_rd_err++;
		}
	}

	/*
	 * ROUND 37: count a CMD53 DAT-line SUCCESS -- a data transfer that
	 * reaches DATA_END with no data-error bit set.  Deduped separately.
	 * This answers 'has ANY CMD53 DAT-line transfer succeeded, read or
	 * write?' -- if reads succeed but writes don't it is TX-only; if
	 * neither ever succeeds the whole DAT block path is broken.
	 */
	if (sc->sdio_slot && off == SDHCI_INT_STATUS &&
	    (val & SDHCI_INT_DATA_END) != 0 &&
	    (val & (SDHCI_INT_DATA_TIMEOUT | SDHCI_INT_DATA_CRC |
	    SDHCI_INT_DATA_END_BIT)) == 0 &&
	    val != sc->dbg_last_datend_intstat) {
		sc->dbg_last_datend_intstat = val;
		if (sc->dbg_last_is_cmd53) {
			if (sc->dbg_last_cmd53_wr)
				spacemit_sdio_cmd53_wr_ok++;
			else
				spacemit_sdio_cmd53_rd_ok++;
		}
	}

	if (off == SDHCI_CAPABILITIES && sc->io_freq != 0 &&
	    (val & SDHCI_CLOCK_V3_BASE_MASK) == 0) {
		/*
		 * Normally inject the real io-clock rate as the base clock.
		 * If the conservative tunable is set (and lower), inject that
		 * instead so the SDHCI layer clamps the SD clock to a safer
		 * value on a marginal card.
		 */
		base_hz = sc->io_freq;
		if (spacemit_sd_conservative_clock_hz != 0 &&
		    spacemit_sd_conservative_clock_hz < base_hz)
			base_hz = spacemit_sd_conservative_clock_hz;
		val |= ((base_hz / 1000000) << SDHCI_CLOCK_BASE_SHIFT) &
		    SDHCI_CLOCK_V3_BASE_MASK;
	}

	/*
	 * Optionally hide the high-speed capability so the card stays in
	 * default-speed (25 MHz) mode -- the most conservative safe timing.
	 */
	if (off == SDHCI_CAPABILITIES && spacemit_sd_no_highspeed != 0)
		val &= ~SDHCI_CAN_DO_HISPD;

	/*
	 * SDIO slot (AP6256): make the SDHCI core negotiate 1.8V bus power.
	 *
	 * ROUND 9.  With WL_REG_ON driven and the rail up, the card was still
	 * silent (no CMD5 answer).  Root cause: sdhci_init_slot() only adds
	 * MMC_OCR_LOW_VOLTAGE (1.8V) to host_ocr when the slot is EMBEDDED --
	 * for a v3.0+ controller 1.8V is otherwise excluded ("not for removable
	 * cards").  The SpacemiT caps report SLOTTYPE=removable, so host_ocr
	 * ends up 3.3V-only; mmc_xpt then sets ios.vdd to the highest voltage =
	 * 3.3V and sdhci_set_power() drives SDHCI_POWER_330 on the AP6256, whose
	 * VDDIO is fixed 1.8V (dcdc_3) -- so the card is powered at the wrong
	 * bus voltage and never responds to CMD5.
	 *
	 * The AP6256 IS an embedded, non-removable, 1.8V-only SDIO device.
	 * Report SLOTTYPE=EMBEDDED (so sdhci_init_slot sets SDHCI_SLOT_EMBEDDED
	 * and folds 1.8V into host_ocr) and ensure CAN_VDD_180 is set, so the
	 * probe negotiates 1.8V and sdhci_set_power() drives SDHCI_POWER_180.
	 * Only the SDIO slot; the root-SD slot's caps are untouched.
	 */
	if (off == SDHCI_CAPABILITIES && sc->sdio_slot) {
		val = (val & ~SDHCI_SLOTTYPE_MASK) | SDHCI_SLOTTYPE_EMBEDDED;
		val |= SDHCI_CAN_VDD_180;
	}

	/*
	 * eMMC slot: the HS200/HS400(ES) modes are 1.8V-signaling only, so the
	 * slot must offer 1.8V.  Like the SDIO fix above, report the slot as
	 * EMBEDDED and CAN_VDD_180 so sdhci_init_slot() folds MMC_OCR_LOW_VOLTAGE
	 * into host_ocr (a v3.0+ controller otherwise excludes 1.8V as "not for
	 * removable cards").  eMMC only (non-removable, not the SDIO slot).
	 */
	if (off == SDHCI_CAPABILITIES && sc->non_removable && !sc->sdio_slot) {
		val = (val & ~SDHCI_SLOTTYPE_MASK) | SDHCI_SLOTTYPE_EMBEDDED;
		val |= SDHCI_CAN_VDD_180;
	}

	return (val);
}

/*
 * Intercept byte-wide register writes so the PHY setup can be re-applied
 * immediately after a software RESET_ALL completes.
 *
 * (Rounds 2-3 also forced the SDHCI CDTEST card-present bits into every
 * HOST_CONTROL write on the SDIO slot; hardware testing proved the bits
 * stick (HOST_CTL=0xc0) but this IP ignores them for its internal
 * card-present state, and the vendor driver never uses them -- removed.)
 */
static void
sdhci_fdt_spacemit_write_1(device_t dev, struct sdhci_slot *slot,
    bus_size_t off, uint8_t val)
{
	struct sdhci_fdt_softc *sc;
	int timeout;

	sc = device_get_softc(dev);

	/*
	 * ROUND 37 test lever: force the SDIO slot to 1-bit by masking the
	 * 4-bit-bus-mode bit out of every HOST_CONTROL write.  Isolates whether
	 * the CMD53 DAT-line failure is a 4-bit (DAT1-3) signalling problem --
	 * if CMD53 succeeds in 1-bit but fails in 4-bit, the issue is the extra
	 * data lanes, not the general data path.
	 */
	if (((struct sdhci_fdt_spacemit_softc *)sc)->sdio_slot &&
	    off == SDHCI_HOST_CONTROL && spacemit_sdio_force_1bit != 0)
		val &= ~SDHCI_CTRL_4BITBUS;

	bus_write_1(sc->mem_res[slot->num], off, val);

	if (off == SDHCI_SOFTWARE_RESET && (val & SDHCI_RESET_ALL) != 0) {
		timeout = 1000;
		while ((bus_read_1(sc->mem_res[slot->num],
		    SDHCI_SOFTWARE_RESET) & SDHCI_RESET_ALL) != 0 &&
		    --timeout > 0)
			DELAY(10);
		sdhci_fdt_spacemit_phy_setup(sc, slot->num);
	}
}

/*
 * Round 11 diagnostic: trace every command ISSUED on the SDIO slot.
 *
 * The SDHCI core writes SDHCI_COMMAND_FLAGS (0x0E) last to launch a command
 * (opcode in the high byte), after having written SDHCI_ARGUMENT (0x08).  The
 * boot-time MMCCAM SDIO probe (mmcprobe: CMD0/CMD8/CMD5 ...) runs before any
 * runtime sysctl can be set, and hw.sdhci.debug's command trace is gated on a
 * runtime flag -- so intercept the command launch here for the SDIO slot only
 * and log opcode + argument + a snapshot of PRESENT/INT status.  This makes it
 * directly visible whether CMD5 (IO_SEND_OP_COND, opcode 5) is even issued on
 * scbus1, and (paired with the completion Int-stat via hw.sdhci.debug) whether
 * it times out.  Zero effect on the root-SD slot; low volume (SDIO probe only).
 */
static void
sdhci_fdt_spacemit_write_2(device_t dev, struct sdhci_slot *slot,
    bus_size_t off, uint16_t val)
{
	struct sdhci_fdt_spacemit_softc *sc;

	sc = device_get_softc(dev);

	if (sc->sdio_slot && off == SDHCI_COMMAND_FLAGS) {
		/* Stash the opcode so the completion trace can tag which CMD. */
		sc->dbg_last_cmd_opcode = (val >> 8) & 0x3f;
		sc->dbg_last_cmd_intstat = 0; /* re-arm completion dedup */
		/*
		 * ROUND 40: re-arm the data-completion dedups per command so
		 * every CMD53's data phase is counted once (the r37 counter
		 * under-counted back-to-back reads with identical INT=0x03).
		 */
		sc->dbg_last_dat_intstat = 0;
		sc->dbg_last_datend_intstat = 0;
		/*
		 * ROUND 37: remember if this is a CMD53 (SD_IO_RW_EXTENDED, the
		 * DAT-line block/byte transfer) and its direction, so the data
		 * completion can bump the per-direction success/fail counters.
		 * The write bit (SD_IOE_RW_WR = bit31) is in SDHCI_ARGUMENT.
		 */
		sc->dbg_last_is_cmd53 = (sc->dbg_last_cmd_opcode == 53);
		sc->dbg_last_cmd53_wr = (bus_read_4(sc->base.mem_res[slot->num],
		    SDHCI_ARGUMENT) & 0x80000000u) != 0;

		/*
		 * ROUND 41: the per-command issue trace is DEFAULT OFF (it floods
		 * dmesg during the firmware download and buries brcmfmac's own
		 * messages).  The CMD53-direction bookkeeping above stays always-on
		 * (the counters depend on it); only the printf is gated.
		 *
		 * ROUND 19 decode (when enabled): the actual SDCLK at CMD issue.
		 * v3 divider = bits[15:8] | (bits[7:6] << 8); SDCLK = base/(2*div).
		 */
		if (spacemit_sdio_cmd_trace) {
			uint16_t clk = bus_read_2(sc->base.mem_res[slot->num],
			    SDHCI_CLOCK_CONTROL);
			uint32_t div = ((clk >> SDHCI_DIVIDER_SHIFT) & 0xff) |
			    (((clk >> SDHCI_DIVIDER_HI_SHIFT) & 0x3) << 8);

			device_printf(dev,
			    "SDIO CMD%u issued: arg=0x%08x flags=0x%02x "
			    "PRESENT=0x%08x INT=0x%08x CLOCK=0x%04x PWR=0x%02x "
			    "HOST_CTL2=0x%04x | base=%uHz div=%u SDCLK=%uHz "
			    "clk[int_en=%d stable=%d card_en=%d]\n",
			    sc->dbg_last_cmd_opcode,
			    bus_read_4(sc->base.mem_res[slot->num], SDHCI_ARGUMENT),
			    val & 0xff,
			    bus_read_4(sc->base.mem_res[slot->num],
			    SDHCI_PRESENT_STATE),
			    bus_read_4(sc->base.mem_res[slot->num],
			    SDHCI_INT_STATUS), clk,
			    bus_read_1(sc->base.mem_res[slot->num],
			    SDHCI_POWER_CONTROL),
			    bus_read_2(sc->base.mem_res[slot->num],
			    SDHCI_HOST_CONTROL2), sc->io_freq, div,
			    div != 0 ? sc->io_freq / (2 * div) : sc->io_freq,
			    !!(clk & SDHCI_CLOCK_INT_EN),
			    !!(clk & SDHCI_CLOCK_INT_STABLE),
			    !!(clk & SDHCI_CLOCK_CARD_EN));
		}
	}

	/*
	 * ROUND 35 -- SDIO clock-cap test lever, FIXED.
	 *
	 * Round 34's cap injected a fake low base into CAPABILITIES, but the
	 * hardware divider divides the REAL io clock (~204.8 MHz), so a low fake
	 * base made the core UNDER-divide -> a FASTER actual SDCLK (the inverted
	 * mapping the coordinator saw).  Instead, intercept the core's
	 * SDHCI_CLOCK_CONTROL write and, if the requested SDCLK exceeds the cap,
	 * raise the v3 divider so the ACTUAL clock (io_freq / (2*div)) is <= cap.
	 * SDIO slot only; only when the tunable is set.
	 */
	if (sc->sdio_slot && off == SDHCI_CLOCK_CONTROL &&
	    spacemit_sdio_clock_hz != 0 && sc->io_freq != 0 &&
	    (val & SDHCI_CLOCK_CARD_EN) != 0) {
		uint32_t need, cur;

		/* Smallest even divider giving <= cap: ceil(io/(2*cap)). */
		need = (sc->io_freq + (2 * spacemit_sdio_clock_hz - 1)) /
		    (2 * spacemit_sdio_clock_hz);
		cur = ((val >> SDHCI_DIVIDER_SHIFT) & 0xff) |
		    (((val >> SDHCI_DIVIDER_HI_SHIFT) & 0x3) << 8);
		if (cur < need && need <= 0x3ff) {
			/* Rebuild the divider field with the larger value. */
			val &= ~((0xffu << SDHCI_DIVIDER_SHIFT) |
			    (0x3u << SDHCI_DIVIDER_HI_SHIFT));
			val |= (need & 0xff) << SDHCI_DIVIDER_SHIFT;
			val |= ((need >> 8) & 0x3) << SDHCI_DIVIDER_HI_SHIFT;
		}
	}

	bus_write_2(sc->base.mem_res[slot->num], off, val);

	/*
	 * ROUND 35 -- re-assert TX_INT_CLK_SEL on EVERY clock program.
	 *
	 * The SDIO firmware download fails with DAT_CRC on host->card (TX) block
	 * writes at 25 MHz while reads succeed -- the textbook symptom of the TX
	 * internal-clock-select (which guarantees the TX data-vs-clock hold time)
	 * not being in effect during data transfer.  The vendor driver
	 * (sdhci-of-x1.c ky_sdhci_set_clock) sets TX_INT_CLK_SEL for LEGACY/HS/
	 * SDR12/SDR25/SDR50 timings on EVERY set_clock call.  We only set it in
	 * phy_setup (after RESET_ALL), so a post-enumeration clock reprogram to
	 * 25 MHz can leave the TX path without it.  The SDHCI core reprograms the
	 * card clock by writing SDHCI_CLOCK_CONTROL (0x2c); mirror the vendor by
	 * (re)asserting TX_INT_CLK_SEL right after -- for the SDIO slot only, at
	 * the sub-100 MHz speeds this board runs (SDR104 tuning, which clears it,
	 * is >=100 MHz and not used here).
	 */
	if (sc->sdio_slot && off == SDHCI_CLOCK_CONTROL) {
		uint32_t tx = bus_read_4(sc->base.mem_res[slot->num],
		    SPACEMIT_SDHC_TX_CFG_REG);

		if ((tx & SDHC_TX_INT_CLK_SEL) == 0) {
			tx |= SDHC_TX_INT_CLK_SEL;
			bus_write_4(sc->base.mem_res[slot->num],
			    SPACEMIT_SDHC_TX_CFG_REG, tx);
		}

		/*
		 * ROUND 36: also (re)apply the fixed TX delay-line config on every
		 * clock program, once the card clock is being enabled.  This is
		 * what finally gives clean write data-vs-clock phase at 25 MHz
		 * (TX_INT_CLK_SEL alone was insufficient -- round 35).
		 */
		if ((val & SDHCI_CLOCK_CARD_EN) != 0)
			sdhci_fdt_spacemit_apply_tx_delay(sc, slot->num);
	}
}

/*
 * PIO multi-register data-port accessors.
 *
 * ROUND 39.  Because we override the sdhci read_4/write_1/write_2 accessors,
 * this subclass must also provide read_multi_4 / write_multi_4 -- the generic
 * sdhci PIO data path (sdhci_transfer_pio) calls SDHCI_READ_MULTI_4 /
 * SDHCI_WRITE_MULTI_4 to move the data buffer through the BUFFER register.
 * Forcing PIO (SDHCI_QUIRK_BROKEN_DMA) on r38 panicked because the multi method
 * resolved to the kobj null default (deref at 0).  Implement them exactly like
 * sdhci_fdt (bus_read/write_multi over the data port); our softc's first member
 * is the sdhci_fdt_softc base, so mem_res is at sc->base.mem_res.
 */
static void
sdhci_fdt_spacemit_read_multi_4(device_t dev, struct sdhci_slot *slot,
    bus_size_t off, uint32_t *data, bus_size_t count)
{
	struct sdhci_fdt_spacemit_softc *sc = device_get_softc(dev);
	struct resource *mem = sc->base.mem_res[slot->num];

	/*
	 * ROUND 40: loop bus_read_4 over the FIXED data-port offset.  DO NOT use
	 * bus_read_multi_4(): on riscv64 the memory bus_space tag sets
	 * .bs_rm_4 = NULL (bus_machdep.c) -- the multi op is unimplemented, so
	 * bus_read_multi_4 dereferences a NULL fn ptr and panics (the r39
	 * crash).  A manual loop reading the same register each iteration is the
	 * correct equivalent (the BUFFER data port is a single fixed address).
	 */
	while (count-- > 0)
		*data++ = bus_read_4(mem, off);
}

static void
sdhci_fdt_spacemit_write_multi_4(device_t dev, struct sdhci_slot *slot,
    bus_size_t off, uint32_t *data, bus_size_t count)
{
	struct sdhci_fdt_spacemit_softc *sc = device_get_softc(dev);
	struct resource *mem = sc->base.mem_res[slot->num];

	/* See read_multi_4: riscv64 has no bus_write_multi_4 (bs_wm_4 NULL). */
	while (count-- > 0)
		bus_write_4(mem, off, *data++);
}

/*
 * Card-presence accessor.
 *
 * The controller's internal card-detect (SDHCI_PRESENT_STATE CARD_PRESENT) is
 * not reliably wired on this board: the real CD switch is a GPIO (cd-gpios,
 * gpio2_16 = pin 80, active-low) described in the devicetree, mirroring how
 * mainline Linux flags this IP SDHCI_QUIRK_BROKEN_CARD_DETECTION and reads the
 * GPIO instead.  Worse, on a warm/watchdog reboot the SDH pad block is never
 * truly reset, so the internal CD can latch "no card" -> the slot comes up with
 * no mmcsd child -> mountroot fails with error 19.  Route presence through the
 * cd-gpio when it was set up, falling back to the generic internal-CD read only
 * if the gpio is unavailable.
 */
static bool
sdhci_fdt_spacemit_get_card_present(device_t dev, struct sdhci_slot *slot)
{
	struct sdhci_fdt_spacemit_softc *sc;

	sc = device_get_softc(dev);
	/*
	 * A soldered-down device (DT "non-removable", e.g. the AP6256 SDIO
	 * WiFi module on SDH1) has no card-detect wiring at all -- the
	 * internal CD would read "no card" forever.  Report always-present.
	 */
	if (sc->non_removable)
		return (true);
	if (sc->gpio != NULL)
		return (sdhci_fdt_gpio_get_present(sc->gpio));
	return (sdhci_generic_get_card_present(dev, slot));
}

/*
 * Set up WL_REG_ON power sequencing for the SDIO WiFi module by grabbing the
 * reset-gpios of the DT "mmc-pwrseq" node directly, instead of relying on the
 * upstream mmc_pwrseq(4) device.
 *
 * WHY NOT mmc_pwrseq(4):  the upstream mmc_pwrseq driver is registered
 * EARLY_DRIVER_MODULE(... BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_FIRST), but our
 * GPIO controller (spacemit_gpio) is BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_LATE
 * -- LATER in the same pass.  So mmc_pwrseq attaches BEFORE the gpio provider
 * has registered its OFW xref: its gpio_pin_get_by_ofw_property("reset-gpios")
 * fails ENXIO ("mmc_pwrseq0: Cannot get the reset-gpios", attach returns 6),
 * mmc_pwrseq never calls OF_device_register_xref(), so mmc_fdt_parse()'s
 * OF_device_from_xref() yields helper->mmc_pwrseq == NULL and
 * MMC_PWRSEQ_SET_POWER() is never invoked -- WL_REG_ON is never toggled and the
 * AP6256 is never reset/enabled, so it never answers CMD5.  (Observed on
 * hardware round 5: card silent, GPIO67 merely at its firmware/boot default.)
 *
 * This driver attaches at the DEFAULT pass, long after spacemit_gpio, so the
 * gpio xref always resolves here.  We reproduce mmc-pwrseq-simple semantics:
 * on attach, drive REG_ON to the "off/reset asserted" state to guarantee a
 * clean power-on edge; on POWER_UP deassert (REG_ON high = chip on) then wait
 * post-power-on-delay-ms (vendor: 200 ms) before the bus issues CMD5; on
 * POWER_OFF re-assert (REG_ON low) and wait power-off-delay-us.  The reset-gpio
 * DT polarity is GPIO_ACTIVE_LOW (reset asserted == REG_ON low == off), so
 * gpio_pin_set_active(true) = assert = off, (false) = deassert = on -- matching
 * the upstream mmc_pwrseq convention exactly.
 */
/*
 * Round 8: acquire the reset-gpio WITHOUT reserving a gpiobus pin.
 *
 * Rounds 6-7 confirmed on hardware (bootverbose): the gpio controller xref
 * resolves fine (gpio0 = spacemit_gpio), but BOTH the OFW consumer helper
 * (gpio_pin_get_by_ofw_property) and our direct-by-pinnum fallback
 * (gpio_pin_get_by_bus_pinnum) return EBUSY (errno 16) -- pin 67 (WL_REG_ON,
 * gpio2_3) is already reserved by another gpiobus consumer, so any path that
 * tries to *reserve* the pin fails.  (This is also the real reason upstream
 * mmc_pwrseq can never work here even with correct pass ordering.)
 *
 * Fix: don't reserve the pin at all.  Resolve the gpio CONTROLLER device from
 * the reset-gpios provider phandle and drive the pin through the controller's
 * own kobj methods GPIO_PIN_SETFLAGS()/GPIO_PIN_SET(), which operate at the
 * controller level and require no gpiobus reservation.  We honor ACTIVE_LOW
 * ourselves (DT flags cell bit0).  spacemit_gpio implements both methods
 * (it advertises GPIO_PIN_INPUT|OUTPUT caps).
 */
static void
sdhci_fdt_spacemit_pwrseq_setup(struct sdhci_fdt_spacemit_softc *sc)
{
	device_t dev = sc->base.dev;
	phandle_t node, pwrseq;
	pcell_t *cells;
	uint32_t xref;
	ssize_t len;

	sc->pwrseq_gpiodev = NULL;

	node = ofw_bus_get_node(dev);
	if (!OF_hasprop(node, "mmc-pwrseq"))
		return;
	if (OF_getencprop(node, "mmc-pwrseq", &xref, sizeof(xref)) <= 0)
		return;
	pwrseq = OF_node_from_xref(xref);
	if (pwrseq == 0 || !OF_hasprop(pwrseq, "reset-gpios"))
		return;

	/* reset-gpios = <&gpio bank offset flags> (4 cells, 3-cell provider). */
	len = OF_getencprop_alloc_multi(pwrseq, "reset-gpios", sizeof(pcell_t),
	    (void **)&cells);
	if (len < 4) {
		if (len > 0)
			OF_prop_free(cells);
		device_printf(dev, "WiFi pwrseq: malformed reset-gpios\n");
		return;
	}
	sc->pwrseq_gpiodev = OF_device_from_xref((phandle_t)cells[0]);
	sc->pwrseq_pin = cells[1] * 32 + cells[2];
	/*
	 * DT gpio flags cell bit0 = active-low (dt-bindings/gpio.h
	 * GPIO_ACTIVE_LOW == 1).  We apply the polarity ourselves below since
	 * the controller-level GPIO_PIN_SET takes a raw physical level.
	 */
	sc->pwrseq_activelow = (cells[3] & 1) != 0;
	OF_prop_free(cells);

	if (sc->pwrseq_gpiodev == NULL) {
		device_printf(dev,
		    "WiFi pwrseq: gpio controller xref unresolved\n");
		return;
	}

	sc->pwrseq_pon_delay_ms = 0;
	sc->pwrseq_poff_delay_us = 0;
	(void)OF_getencprop(pwrseq, "post-power-on-delay-ms",
	    &sc->pwrseq_pon_delay_ms, sizeof(sc->pwrseq_pon_delay_ms));
	(void)OF_getencprop(pwrseq, "power-off-delay-us",
	    &sc->pwrseq_poff_delay_us, sizeof(sc->pwrseq_poff_delay_us));

	/*
	 * Configure the pin as output and drive REG_ON to the off
	 * (reset-asserted) state now so the first POWER_UP produces a real
	 * off->on edge and a fresh chip power-on reset, regardless of the level
	 * firmware/U-Boot left the pin at.
	 *
	 * ROUND 16 polarity fix (see _pwrseq_power): for reset-gpios
	 * ACTIVE_LOW, the OFF / reset-asserted physical level is LOW.  Raw
	 * physical off_level = (activelow ? 0 : 1).  (The prior code drove
	 * (activelow ? 1 : 0) = physical HIGH here and physical LOW on
	 * power_up -- exactly inverted -- so the module was left enabled at
	 * attach then *disabled* at power_up, i.e. held in reset for the whole
	 * probe; gpioctl gpio67 read LOW on the running board, confirming it.)
	 */
	GPIO_PIN_SETFLAGS(sc->pwrseq_gpiodev, sc->pwrseq_pin, GPIO_PIN_OUTPUT);
	GPIO_PIN_SET(sc->pwrseq_gpiodev, sc->pwrseq_pin,
	    sc->pwrseq_activelow ? 0 : 1);
	sc->pwrseq_powered = false;
	device_printf(dev,
	    "WiFi pwrseq: driving WL_REG_ON via %s pin %u (active%s, "
	    "pon=%ums poff=%uus)\n",
	    device_get_nameunit(sc->pwrseq_gpiodev), sc->pwrseq_pin,
	    sc->pwrseq_activelow ? "-low" : "-high",
	    sc->pwrseq_pon_delay_ms, sc->pwrseq_poff_delay_us);
}

/*
 * Enable the AP6256 WIFI_VCC33 (VBAT) main rail by driving EXT_PWR_EN
 * (GPIO 116, active-high) HIGH.  Grounded in the RV2 schematic (p13:
 * EXT_PWR_EN on GPIO116 -> load switch -> WIFI_VCC33) + the vendor DTS
 * (x1,pwr_on = <&gpio 116 0>).  Without this the module has NO main power:
 * no VBAT -> no 37.4 MHz REF_CLK / VDD_TCXO / WLAN core -> the chip is dead
 * and answers no SDIO command (CMD5 timeout, no CRC) -- the root cause of
 * rounds 5-24.  The DTS does not model this pin (vendor rf-pwrseq pwr-gpios
 * is empty; U-Boot drives it on Linux), so we drive GPIO116 directly through
 * the gpio controller's kobj methods, exactly as we do WL_REG_ON (gpio67).
 * Both pins live on the same controller (spacemit_gpio0), resolved during
 * pwrseq_setup, so reuse that device handle.
 */
static void
sdhci_fdt_spacemit_vbat_setup(struct sdhci_fdt_spacemit_softc *sc)
{
	device_t dev = sc->base.dev;

	sc->vbat_gpiodev = NULL;
	if (sc->pwrseq_gpiodev == NULL)
		return;			/* no gpio controller resolved */

	sc->vbat_gpiodev = sc->pwrseq_gpiodev;
	sc->vbat_pin = SPACEMIT_WIFI_VBAT_GPIO;

	/*
	 * Drive EXT_PWR_EN HIGH now (at attach) so WIFI_VCC33 comes up as early
	 * as possible; the AP6256 needs its supplies + REF_CLK settled well
	 * before WL_REG_ON and CMD5.  Re-asserted (and settle-delayed) in the
	 * power_on path too.
	 */
	GPIO_PIN_SETFLAGS(sc->vbat_gpiodev, sc->vbat_pin, GPIO_PIN_OUTPUT);
	GPIO_PIN_SET(sc->vbat_gpiodev, sc->vbat_pin, 1);
	sc->vbat_on = true;
	device_printf(dev,
	    "WiFi VBAT: driving EXT_PWR_EN (WIFI_VCC33) HIGH via %s pin %u\n",
	    device_get_nameunit(sc->vbat_gpiodev), sc->vbat_pin);
}

static void
sdhci_fdt_spacemit_pwrseq_power(struct sdhci_fdt_spacemit_softc *sc,
    enum mmc_power_mode power_mode)
{
	uint32_t off_level, on_level;

	if (sc->pwrseq_gpiodev == NULL)
		return;

	/*
	 * Physical levels for REG_ON off/on, honoring ACTIVE_LOW.
	 *
	 * ROUND 16 -- POLARITY FIX.  Rounds 6-15 had these inverted, holding
	 * the AP6256 permanently in reset (gpioctl gpio67 read 0/LOW on the
	 * running board while we thought the chip was "on").  Re-derived from
	 * upstream mmc_pwrseq(4) + the vendor driver, which agree:
	 *
	 * mmc_pwrseq_simple drives the reset-gpio with the ACTIVE-aware
	 * gpio_pin_set_active(): power_ON = set_active(FALSE) = DEASSERT reset;
	 * power_OFF/attach = set_active(TRUE) = ASSERT reset.  The physical
	 * level it puts out is (active_bool XOR active_low):
	 *   - power_on : set_active(false) -> phys = active_low
	 *   - power_off: set_active(true)  -> phys = !active_low
	 * For our reset-gpios = <&gpio 67 GPIO_ACTIVE_LOW> (active_low=1):
	 *   chip ON  = physical HIGH (1),  chip OFF = physical LOW (0).
	 *
	 * This matches the hardware reality: WL_REG_ON on the Broadcom module
	 * is a physical active-HIGH enable (drive HIGH = powered), which the
	 * vendor expresses the other way round as regon-gpios = ACTIVE_HIGH +
	 * gpiod_set_value(1) -> also physical HIGH.  Both stacks converge on
	 * "chip running == WL_REG_ON HIGH".
	 *
	 * We drive the controller's kobj GPIO_PIN_SET with a RAW physical
	 * level (no active-low translation in the callee), so compute the raw
	 * levels explicitly here:
	 */
	off_level = sc->pwrseq_activelow ? 0 : 1;	/* reset asserted = REG_ON low */
	on_level = sc->pwrseq_activelow ? 1 : 0;	/* reset deasserted = REG_ON high */

	if (power_mode == power_off) {
		if (!sc->pwrseq_powered)
			return;
		GPIO_PIN_SET(sc->pwrseq_gpiodev, sc->pwrseq_pin, off_level);
		sc->pwrseq_powered = false;
		if (sc->pwrseq_poff_delay_us != 0)
			DELAY(sc->pwrseq_poff_delay_us);
		return;
	}

	/* power_up / power_on: reset-pulse then deassert (REG_ON high = on). */
	if (sc->pwrseq_powered)
		return;

	/*
	 * ROUND 25 -- VBAT FIRST.  Ensure the AP6256 main rail (WIFI_VCC33 via
	 * EXT_PWR_EN / GPIO116) is on and its supplies + 37.4 MHz REF_CLK have
	 * settled BEFORE WL_REG_ON, per the CYW43455/AP6256 power-up ordering
	 * (VBAT + VDDIO up -> wait for REF_CLK -> assert WL_REG_ON -> wait ->
	 * CMD5).  vbat is driven high at attach; re-assert here and give it a
	 * >=150 ms settle before the reset pulse.  (The chip was never powered
	 * before round 25: gpio116 read LOW on the live board, so WIFI_VCC33 was
	 * off the entire time -- no VBAT, no REF_CLK, mute to every CMD5.)
	 */
	if (sc->vbat_gpiodev != NULL) {
		if (!sc->vbat_on) {
			GPIO_PIN_SET(sc->vbat_gpiodev, sc->vbat_pin, 1);
			sc->vbat_on = true;
		}
		DELAY(150000);	/* >=150 ms for supplies + REF_CLK to settle */
	}

	/*
	 * ROUND 17 -- guarantee a real WL_REG_ON reset PULSE with a hold,
	 * every boot, regardless of the pin's prior physical level.
	 *
	 * The AP6256 is a soldered module with no board power switch: on a
	 * WARM reboot it is NOT power-cycled, so its internal PMU keeps
	 * whatever state a prior boot left.  WL_REG_ON must be driven LOW long
	 * enough (Broadcom PMU spec: >~10 ms) to force a clean internal reset
	 * before the low->high edge that powers the core up; otherwise the
	 * chip can come up half-initialised and answer nothing (round 16 got
	 * REG_ON HIGH -- gpio67=1, chip powered -- but CMD5 still TIMEOUT).
	 *
	 * Rounds 6-16 relied on the mmc_xpt PROBE_RESET(power_off) step to
	 * assert the low hold, but our power_off path early-returns when
	 * pwrseq_powered is still false (its state at attach), so that hold
	 * was SKIPPED and REG_ON only saw a brief, undelayed low at attach
	 * before being driven high.  Do the full pulse here, self-contained,
	 * so it cannot be short-circuited:
	 *   1. drive LOW  (assert reset)              -- unconditional
	 *   2. hold poff_delay_us (floor 20 ms)       -- real PMU reset hold
	 *   3. drive HIGH (deassert reset)            -- power the core up
	 *   4. settle pon_delay_ms (floor 500 ms)     -- before first CMD5
	 * This matches upstream mmc_pwrseq's off(hold)->on(settle) intent while
	 * being independent of the pwrseq_powered bookkeeping.
	 */
	{
		uint32_t hold_us = sc->pwrseq_poff_delay_us;
		uint32_t settle_ms = sc->pwrseq_pon_delay_ms;

		if (hold_us < 20000)
			hold_us = 20000;
		if (settle_ms < 500)
			settle_ms = 500;

		GPIO_PIN_SET(sc->pwrseq_gpiodev, sc->pwrseq_pin, off_level);
		DELAY(hold_us);
		GPIO_PIN_SET(sc->pwrseq_gpiodev, sc->pwrseq_pin, on_level);
		sc->pwrseq_powered = true;
		DELAY(settle_ms * 1000);

		if (bootverbose)
			device_printf(sc->base.dev,
			    "WiFi pwrseq: WL_REG_ON reset pulse LOW %ums -> HIGH, "
			    "settle %ums (pin %u)\n", hold_us / 1000, settle_ms,
			    sc->pwrseq_pin);
	}
}

/*
 * NOTE (round 18): the SDIO-slot 74-init-clocks burst was REMOVED.  The
 * shipping Bianbu driver (spacemit-com/linux-k1x @ k1,
 * drivers/mmc/host/sdhci-of-k1x.c, and the identical orangepi sdhci-of-x1.c)
 * runs platform_send_init_74_clocks ONLY on non-SDIO slots
 * (`if (!(caps2 & MMC_CAP2_NO_SDIO)) return;`) and clocks the SDIO slot purely
 * via the standard SDHCI core with auto clock gating.  Our earlier burst +
 * held FORCE_CLK_ON were a wrong substitute that mis-phased the command clock;
 * removed to match the working config exactly.  The MISC_INT / GEN_PAD_CLK_ON
 * register defs remain for potential SD/eMMC use.
 */

/*
 * Chip-specific IOS wrapper for the legacy mmcbr path.  FreeBSD main removed
 * the old SDHCI platform IOS hook.  This subclass performs its board
 * sequencing first, then delegates the standard register update to the
 * generic core.
 *
 * On this board the SD-card rails are guarded by the PMIC driver's
 * allow_rail_changes=0 policy (enable/disable are physical no-ops that
 * return success), so mmc_fdt_set_power is behaviourally inert for the root
 * SD slot.  The R2S has no onboard SDIO; MMCCAM would need a new generic
 * platform hook before the inherited RV2 WiFi sequence could run there.
 */
/*
 * Perform the SD-spec 1.8V signalling voltage switch as a self-contained,
 * clock-gated sequence, then leave HOST_CONTROL2 S18 asserted -- run ONCE on
 * the SDIO slot at power_on, BEFORE the first CMD5.
 *
 * ROUND 21.  Rounds 14-19 forced S18=1 from init (bus stuck at 1.8V signalling
 * with no handshake); round 20 left S18=0 (bus at 3.3V signalling).  BOTH raw
 * states leave the AP6256 mute -- because 1.8V signalling must be entered
 * through the proper GATED sequence, not a bare register bit.  The AP6256 is a
 * fixed-1.8V embedded SDIO part (VDDIO = dcdc_3, no 3.3V phase), so it needs
 * valid 1.8V signalling at CMD5 -- but achieved via clock-off -> S18 -> 5 ms
 * settle -> clock-on, exactly as the SDHCI core's sdhci_generic_switch_vccq()
 * (vccq_180 path) and the SD Physical Layer spec do.  FreeBSD's MMCCAM SDIO
 * probe never drives a voltage switch (cam/mmc/mmc_xpt.c PROBE_SDIO_INIT issues
 * CMD5 with no CMD11 and no vccq_180), so we must run the switch here.
 *
 * Sequence (mirrors sdhci_generic_switch_vccq + the vendor's transient force):
 *   1. stop SDCLK (clear SDHCI_CLOCK_CARD_EN)          -- clock gated for the switch
 *   2. transiently force the pad clock on              -- vendor ky set_clk_gate(0)
 *      around SD_SWITCH_VOLTAGE (OP_EXT FORCE_CLK_ON|OVRRD_CLK_OEN); needed so
 *      the level shifter settles with a defined clock, recovered below
 *   3. set HOST_CONTROL2 S18 (1.8V Signal Enable)
 *   4. DELAY(5 ms)  -- SD spec: clock closed >=5 ms during the level switch
 *   5. drop the transient force (recover auto clock gating) -- vendor card_busy
 *   6. re-enable SDCLK (SDHCI_CLOCK_CARD_EN) and wait for INT_STABLE
 * After this the bus is at valid 1.8V signalling for the first CMD5.
 */
static void
sdhci_fdt_spacemit_switch_1v8(struct sdhci_fdt_spacemit_softc *sc, int slotnum)
{
	struct resource *mem = sc->base.mem_res[slotnum];
	uint16_t clk, clk_saved, ctl2;
	uint32_t op_ext;
	int timeout;

	/*
	 * ROUND 22 -- preserve the FULL clock config across the switch.
	 *
	 * Round 21 only toggled SDHCI_CLOCK_CARD_EN and left INT_EN + the
	 * divider at whatever they were, which on hardware ended as
	 * CLOCK=0x0004 (CARD_EN only, no internal clock, no divider) -- a DEAD
	 * clock at CMD5.  Worse, the sdhci core's sdhci_set_clock() early-returns
	 * when the requested rate equals its cached slot->clock, so it would NOT
	 * restore our clobbered register.  Snapshot the whole SDHCI_CLOCK_CONTROL
	 * word up front and write it back verbatim at the end, so the running
	 * ~401 kHz init clock (INT_EN|INT_STABLE|CARD_EN|divider = 0xff07) is
	 * exactly restored after the switch.
	 */
	clk_saved = bus_read_2(mem, SDHCI_CLOCK_CONTROL);

	/* 1. Stop the SD card clock (clear only CARD_EN; keep INT_EN + div). */
	clk = clk_saved & ~SDHCI_CLOCK_CARD_EN;
	bus_write_2(mem, SDHCI_CLOCK_CONTROL, clk);

	/* 2. Transiently force the pad clock on across the switch (vendor). */
	op_ext = bus_read_4(mem, SPACEMIT_SDHC_OP_EXT_REG);
	bus_write_4(mem, SPACEMIT_SDHC_OP_EXT_REG,
	    op_ext | SDHC_OVRRD_CLK_OEN | SDHC_FORCE_CLK_ON);

	/* 3. Assert 1.8V Signal Enable. */
	ctl2 = bus_read_2(mem, SDHCI_HOST_CONTROL2);
	ctl2 |= SDHCI_CTRL2_S18_ENABLE;
	bus_write_2(mem, SDHCI_HOST_CONTROL2, ctl2);

	/* 4. SD spec: clock closed >= 5 ms during the level switch. */
	DELAY(5000);

	/* 5. Recover auto clock gating (drop the transient force). */
	op_ext = bus_read_4(mem, SPACEMIT_SDHC_OP_EXT_REG);
	bus_write_4(mem, SPACEMIT_SDHC_OP_EXT_REG,
	    op_ext & ~(SDHC_OVRRD_CLK_OEN | SDHC_FORCE_CLK_ON));

	/*
	 * 6. Restore the exact pre-switch clock word (INT_EN + divider +
	 * INT_STABLE + CARD_EN) so the ~401 kHz init clock runs again, then wait
	 * for the internal clock to report stable.  If for any reason INT_EN was
	 * not set in the snapshot, force a minimal valid enable so we never leave
	 * a dead clock.
	 */
	if ((clk_saved & SDHCI_CLOCK_INT_EN) == 0)
		clk_saved |= SDHCI_CLOCK_INT_EN | SDHCI_CLOCK_CARD_EN;
	bus_write_2(mem, SDHCI_CLOCK_CONTROL, clk_saved);
	for (timeout = 0; timeout < 1000; timeout++) {
		if (bus_read_2(mem, SDHCI_CLOCK_CONTROL) & SDHCI_CLOCK_INT_STABLE)
			break;
		DELAY(10);
	}

	ctl2 = bus_read_2(mem, SDHCI_HOST_CONTROL2);
	if (bootverbose)
		device_printf(sc->base.dev,
		    "SDIO: 1.8V switch done (clock-gated 5ms): S18=%d "
		    "CLOCK 0x%04x->0x%04x\n",
		    !!(ctl2 & SDHCI_CTRL2_S18_ENABLE), clk_saved,
		    bus_read_2(mem, SDHCI_CLOCK_CONTROL));
}

static int
sdhci_fdt_spacemit_update_ios(device_t dev, device_t reqdev)
{
	struct sdhci_fdt_spacemit_softc *sc;
	struct sdhci_slot *slot;
	enum mmc_power_mode pm;

	sc = device_get_softc(dev);
	slot = device_get_ivars(reqdev);
	pm = slot->host.ios.power_mode;

	/*
	 * ROUND 26 -- keep WIFI_VCC33 (EXT_PWR_EN / GPIO116) latched HIGH.
	 *
	 * GPIO116 gates a SHARED SYSTEM rail (EXT_3V3 -> WiFi VBAT + PCIe 3.3V);
	 * it must be asserted ONCE and held, NEVER tied to the mmc power on/off
	 * cycle.  Round 25 drove it high only at attach + power_up, but the
	 * MMCCAM probe ends every failed pass with a power_off, and on the live
	 * board GPIO116 read 0 after boot -- the pad's pull-down (mux 0xb040)
	 * wins whenever the output latch is lost across the probe's power
	 * cycling.  Re-assert it HIGH unconditionally at the top of EVERY ios
	 * update (power_off included) so it can never fall low.  It is a
	 * board-level rail enable, independent of the SDIO card's power state.
	 */
	if (sc->vbat_gpiodev != NULL) {
		GPIO_PIN_SETFLAGS(sc->vbat_gpiodev, sc->vbat_pin, GPIO_PIN_OUTPUT);
		GPIO_PIN_SET(sc->vbat_gpiodev, sc->vbat_pin, 1);
		sc->vbat_on = true;
	}

	mmc_fdt_set_power(&sc->mmc_helper, pm);
	sdhci_fdt_spacemit_pwrseq_power(sc, pm);

	/*
	 * ROUND 21: on the SDIO slot, do the proper clock-gated 1.8V signalling
	 * switch at power_on (after WL_REG_ON + regulators are up, before the
	 * first CMD5).  Both raw S18 states failed (S18=1 from init r14-r19,
	 * S18=0 r20); the AP6256 needs 1.8V signalling entered via the gated
	 * sequence.  Runs once per power_up->power_on transition.
	 */
	if (sc->sdio_slot && pm == power_on)
		sdhci_fdt_spacemit_switch_1v8(sc, slot->num);

	/*
	 * ROUND 18: do NOT run the 74-clock burst on the SDIO slot.
	 *
	 * Ground truth from the SHIPPING Bianbu driver
	 * (github.com/spacemit-com/linux-k1x @ branch k1,
	 * drivers/mmc/host/sdhci-of-k1x.c -- the driver the working
	 * Ubuntu/Bianbu image on this exact board uses; the orangepi
	 * sdhci-of-x1.c is identical here): spacemit_sdhci_gen_init_74_clocks()
	 * opens with `if (!(caps2 & MMC_CAP2_NO_SDIO)) return;`, i.e. it runs
	 * the burst ONLY on non-SDIO (SD/eMMC) slots and DELIBERATELY SKIPS the
	 * SDIO slot.  The SDIO slot is clocked purely by the standard SDHCI core
	 * (SDHCI_CLOCK_CARD_EN via sdhci_set_clock) with auto clock gating; the
	 * only vendor pad-clock touch on this host class is TX_INT_CLK_SEL in
	 * reset (which we do in phy_setup).  Our earlier SDIO 74-clock burst +
	 * held FORCE_CLK_ON were a wrong substitute that, with the chip finally
	 * powered (r16 gpio67=1), still left CMD5 timing out even on a cold
	 * power-cycle -- consistent with the forced/burst clock mis-phasing the
	 * command so no valid frame reaches the card.  The burst call is removed
	 * (the helper is retained but now unused on this slot).
	 */

	return (sdhci_generic_update_ios(dev, reqdev));
}

static int
sdhci_fdt_spacemit_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "SpacemiT K1 SDHCI controller");
	return (BUS_PROBE_SPECIFIC);
}

static int
sdhci_fdt_spacemit_attach(device_t dev)
{
	struct sdhci_fdt_softc *sc;
	clk_t clk_core, clk_io;
	hwreset_t rst;
	uint64_t freq;
	int error;

	sc = device_get_softc(dev);

	/* Enable the bus (core/AXI) and functional (io) clocks. */
	error = clk_get_by_ofw_name(dev, 0, "core", &clk_core);
	if (error != 0) {
		device_printf(dev, "cannot get 'core' clock: %d\n", error);
		return (ENXIO);
	}
	error = clk_enable(clk_core);
	if (error != 0) {
		device_printf(dev, "cannot enable 'core' clock: %d\n", error);
		return (ENXIO);
	}

	error = clk_get_by_ofw_name(dev, 0, "io", &clk_io);
	if (error != 0) {
		device_printf(dev, "cannot get 'io' clock: %d\n", error);
		return (ENXIO);
	}
	error = clk_enable(clk_io);
	if (error != 0) {
		device_printf(dev, "cannot enable 'io' clock: %d\n", error);
		return (ENXIO);
	}

	/*
	 * Release the controller resets.
	 *
	 * The "axi" reset is shared with the sibling SDH instances, so it must
	 * only ever be released (deasserted), never pulsed -- asserting it would
	 * knock out the other controllers.
	 */
	if (hwreset_get_by_ofw_name(dev, 0, "axi", &rst) == 0) {
		error = hwreset_deassert(rst);
		if (error != 0)
			device_printf(dev,
			    "warning: cannot deassert 'axi' reset: %d\n",
			    error);
	}
	/*
	 * The "sdh" reset is per-instance, so pulse it (assert -> deassert) to
	 * force a true hardware reset of this controller's vendor PHY/pad block.
	 * On a warm/watchdog reboot the reset line is already released, so a bare
	 * deassert is a no-op and the block keeps whatever (possibly wedged)
	 * state it latched -- and the SDHCI-core RESET_ALL does NOT reach the
	 * vendor PHY/pad registers, so this pulse is the only true hardware reset
	 * the block ever gets.  Without it, a warm reboot can leave the internal
	 * card-detect stuck at "no card".
	 */
	if (hwreset_get_by_ofw_name(dev, 0, "sdh", &rst) == 0) {
		error = hwreset_assert(rst);
		if (error != 0)
			device_printf(dev,
			    "warning: cannot assert 'sdh' reset: %d\n",
			    error);
		DELAY(10);
		error = hwreset_deassert(rst);
		if (error != 0)
			device_printf(dev,
			    "warning: cannot deassert 'sdh' reset: %d\n",
			    error);
		DELAY(100);
	}

	/*
	 * ROUND 19 -- match the SDIO host's io-clock rate to the working
	 * shipping config.
	 *
	 * The shipping Bianbu driver (sdhci-of-k1x.c) does, at probe:
	 *   clk_set_rate(clk_io, pdata->host_freq)   // DT spacemit,sdh-freq
	 * and the board DTS sets &sdhci1 spacemit,sdh-freq = <375000000>.  Its
	 * get_max_clock then returns clk_get_rate(clk_io) as the SDHCI base, so
	 * the core computes every divider (including the ~400 kHz init divider
	 * for CMD5) against 375 MHz.  Our CCU leaves SDH1 at its reset/firmware
	 * default (~204.8 MHz), so we divide against a different base -- a real
	 * delta vs the working stack.  Request 375 MHz on the SDIO slot the same
	 * way the vendor does; if our CCU cannot serve it (no set_freq / missing
	 * pll2 parent), log the failure and fall back to whatever the clock
	 * currently is, so this is safe either way.  Root-SD slot is untouched
	 * (its clock stays exactly as it has shipped).
	 */
	if (OF_hasprop(ofw_bus_get_node(dev), "no-mmc") &&
	    OF_hasprop(ofw_bus_get_node(dev), "no-sd")) {
		uint32_t sdh_freq = 375000000;

		(void)OF_getencprop(ofw_bus_get_node(dev), "spacemit,sdh-freq",
		    &sdh_freq, sizeof(sdh_freq));
		error = clk_set_freq(clk_io, sdh_freq, CLK_SET_ROUND_ANY);
		if (error != 0)
			device_printf(dev,
			    "SDIO: clk_set_freq(%u) failed: %d (CCU may lack "
			    "set_freq/pll2 parent; using current rate)\n",
			    sdh_freq, error);
		else
			device_printf(dev,
			    "SDIO: io clock set to %u Hz (vendor sdh-freq)\n",
			    sdh_freq);
	}

	/*
	 * Use the functional clock rate as the base clock if the
	 * capabilities register does not provide one ("max-frequency"
	 * from the devicetree still takes precedence).
	 */
	freq = 0;
	error = clk_get_freq(clk_io, &freq);
	if (bootverbose || freq == 0)
		device_printf(dev, "'io' clock rate %ju Hz (error %d)\n",
		    (uintmax_t)freq, error);
	((struct sdhci_fdt_spacemit_softc *)sc)->io_freq = (uint32_t)freq;

	/*
	 * Read card presence from the cd-gpio (see get_card_present).  Because
	 * presence is driven by a gpio the SDHCI core cannot see, force polling
	 * for card-present.  This quirk must be set BEFORE sdhci_fdt_attach()
	 * runs sdhci_init_slot(), which copies quirks into the slot and arms the
	 * presence poll.
	 */
	sc->quirks |= SDHCI_QUIRK_POLL_CARD_PRESENT;

	/*
	 * Parse the generic MMC devicetree properties for the power-handling
	 * pieces: vmmc-supply/vqmmc-supply regulators and the "mmc-pwrseq"
	 * power sequence (WL_REG_ON GPIO for the AP6256 WiFi module on SDH1).
	 *
	 * Deliberately parse into a SCRATCH mmc_host, not the slot's: on the
	 * root-SD instance mmc_fdt_parse() would also fold DT caps (sd-uhs-*,
	 * 1.8V signaling) into host.caps, changing the negotiated timing of
	 * the working -- and marginal -- boot SD card.  We only want the
	 * regulator/pwrseq handles here; the slot's caps continue to come
	 * from the controller's capability registers exactly as before.
	 */
	{
		struct mmc_host scratch_host;

		memset(&scratch_host, 0, sizeof(scratch_host));
		mmc_fdt_parse(dev, ofw_bus_get_node(dev),
		    &((struct sdhci_fdt_spacemit_softc *)sc)->mmc_helper,
		    &scratch_host);
	}
	((struct sdhci_fdt_spacemit_softc *)sc)->non_removable =
	    OF_hasprop(ofw_bus_get_node(dev), "non-removable");
	((struct sdhci_fdt_spacemit_softc *)sc)->wp_disabled =
	    OF_hasprop(ofw_bus_get_node(dev), "disable-wp");
	((struct sdhci_fdt_spacemit_softc *)sc)->wp_inverted =
	    OF_hasprop(ofw_bus_get_node(dev), "wp-inverted");
	/*
	 * SDIO-only slot (neither SD memory nor eMMC allowed by the DT):
	 * needs the forced pad-clock + 1.8V signaling in phy_setup.  Must be
	 * latched before sdhci_fdt_attach() -> sdhci_init_slot() issues the
	 * first RESET_ALL, whose write_1 intercept runs phy_setup.
	 */
	((struct sdhci_fdt_spacemit_softc *)sc)->sdio_slot =
	    OF_hasprop(ofw_bus_get_node(dev), "no-mmc") &&
	    OF_hasprop(ofw_bus_get_node(dev), "no-sd");

	/*
	 * ROUND 38: optionally force PIO on the SDIO controller (BROKEN_DMA
	 * clears SDHCI_HAVE_DMA in sdhci_init_slot).  Must be set before
	 * sdhci_fdt_attach() copies quirks into the slot.  SDIO slot only; the
	 * root-SD controller is a separate device instance and is untouched.
	 */
	if (((struct sdhci_fdt_spacemit_softc *)sc)->sdio_slot &&
	    spacemit_sdio_force_pio != 0) {
		sc->quirks |= SDHCI_QUIRK_BROKEN_DMA;
		device_printf(dev, "SDIO: forcing PIO (BROKEN_DMA) per tunable\n");
	}

	/*
	 * Limit full-map cache maintenance for small eMMC requests without
	 * changing synchronization, mapping attributes, ISA or clock policy.
	 * A smaller boundary increases interrupt frequency for bulk I/O;
	 * 128 KiB balances small-request cache cost and measured bulk throughput.
	 * BROKEN_SDMA_BOUNDARY tells the core to retain a front-end boundary.
	 */
	if (((struct sdhci_fdt_spacemit_softc *)sc)->non_removable &&
	    !((struct sdhci_fdt_spacemit_softc *)sc)->sdio_slot &&
	    spacemit_emmc_sdma_kib != 0) {
		switch (spacemit_emmc_sdma_kib) {
		case 4: sc->sdma_boundary = SDHCI_BLKSZ_SDMA_BNDRY_4K; break;
		case 8: sc->sdma_boundary = SDHCI_BLKSZ_SDMA_BNDRY_8K; break;
		case 16: sc->sdma_boundary = SDHCI_BLKSZ_SDMA_BNDRY_16K; break;
		case 32: sc->sdma_boundary = SDHCI_BLKSZ_SDMA_BNDRY_32K; break;
		case 64: sc->sdma_boundary = SDHCI_BLKSZ_SDMA_BNDRY_64K; break;
		case 128: sc->sdma_boundary = SDHCI_BLKSZ_SDMA_BNDRY_128K; break;
		case 256: sc->sdma_boundary = SDHCI_BLKSZ_SDMA_BNDRY_256K; break;
		case 512: sc->sdma_boundary = SDHCI_BLKSZ_SDMA_BNDRY_512K; break;
		default:
			device_printf(dev, "Ignoring invalid emmc_sdma_kib=%d\n",
			    spacemit_emmc_sdma_kib);
			goto sdma_default;
		}
		sc->quirks |= SDHCI_QUIRK_BROKEN_SDMA_BOUNDARY;
		device_printf(dev, "eMMC SDMA buffer: %d KiB\n",
		    spacemit_emmc_sdma_kib);
	}
sdma_default:

	/*
	 * Use ADMA2 on the eMMC.  Validated on this controller (2026-09-16):
	 * integrity holds for unaligned, partial-page and multi-segment
	 * requests and across a path switch, with no ADMA error in any test,
	 * and sequential reads gain about 6 percent over the SDMA bounce
	 * buffer.  The SDIO slot is deliberately left on SDMA: it is the one
	 * with the troubled history here and nothing has been measured on it.
	 * Removable SD slots (the RV2's microSD boot card) stay on SDMA too:
	 * ADMA2 has only been measured on the non-removable eMMC.
	 */
	if (((struct sdhci_fdt_spacemit_softc *)sc)->non_removable &&
	    !((struct sdhci_fdt_spacemit_softc *)sc)->sdio_slot)
		sc->quirks2 |= SDHCI_QUIRK2_USE_ADMA2;

	error = sdhci_fdt_attach(dev);
	if (error != 0)
		return (error);

	/*
	 * Acquire the WL_REG_ON reset-gpio (WiFi slot only) now that the gpio
	 * controller is up; drive it to the off state.  Replaces the broken
	 * mmc_pwrseq(4) device (see sdhci_fdt_spacemit_pwrseq_setup).
	 */
	sdhci_fdt_spacemit_pwrseq_setup(
	    (struct sdhci_fdt_spacemit_softc *)sc);

	/*
	 * Enable the AP6256 WIFI_VCC33 (VBAT) main rail via EXT_PWR_EN (GPIO116)
	 * now that the gpio controller is resolved.  Must be on -- and settled --
	 * before WL_REG_ON / CMD5 (see sdhci_fdt_spacemit_vbat_setup).  WiFi slot
	 * only (guarded by pwrseq_gpiodev, which is set only when the DT node has
	 * mmc-pwrseq).
	 */
	if (((struct sdhci_fdt_spacemit_softc *)sc)->sdio_slot)
		sdhci_fdt_spacemit_vbat_setup(
		    (struct sdhci_fdt_spacemit_softc *)sc);

	if (sc->num_slots > 0) {
		struct sdhci_fdt_spacemit_softc *spc =
		    (struct sdhci_fdt_spacemit_softc *)sc;

		/*
		 * Override the SDHCI base clock (max_clk) with the real io-clock
		 * rate.  The read handler injects io_freq into the SDHCI-v3
		 * capabilities base-clock field, but that field is only 8 bits
		 * (max 255 MHz): an io_freq above that (e.g. a 375/409.6 MHz eMMC
		 * clock) truncates, and the mmc divisor search against the REAL
		 * base then over-clocks the card ~2x -> Bad CRC/Timeout.  Setting
		 * max_clk to the full 32-bit io_freq here (after sdhci_init_slot)
		 * mirrors Linux's get_max_clock and is correct for any base.
		 *
		 * eMMC: cap the reported base to 200 MHz so the HS400 200 MHz
		 * request satisfies clock >= base -> the v3 divisor picks 1:1
		 * passthrough and the pad runs at the true ~204.8 MHz SDH2 rate
		 * (the vendor operating point); uncapped it would divide to
		 * ~102 MHz.  Non-eMMC keeps the full io_freq.
		 */
		if (spc->io_freq != 0) {
			uint32_t mc = spc->io_freq;

			if (spc->non_removable && !spc->sdio_slot &&
			    mc > 200000000u)
				mc = 200000000u;
			sc->slots[0].max_clk = mc;
		}

		/*
		 * eMMC high-speed modes: advertise HS200/HS400 + enhanced strobe
		 * + 8-bit + 1.8V signaling so the legacy mmc(4) bridge negotiates
		 * HS400ES; the host-side timing/PHY/DLL is applied in
		 * sdhci_fdt_spacemit_set_uhs_timing().  Done post-attach to bypass
		 * sdhci_init_slot()'s cap strip (which drops these when mmcbr_tune
		 * is the default -- the enhanced-strobe path needs no tuning).
		 * eMMC only.
		 */
		if (spc->non_removable && !spc->sdio_slot) {
			sc->slots[0].host.caps |= MMC_CAP_8_BIT_DATA |
			    MMC_CAP_MMC_HS200 | MMC_CAP_MMC_HS200_180 |
			    MMC_CAP_MMC_HS400 | MMC_CAP_MMC_HS400_180 |
			    MMC_CAP_MMC_ENH_STROBE | MMC_CAP_MMC_DDR52 |
			    MMC_CAP_SIGNALING_180;
			sc->slots[0].host.host_ocr |= MMC_OCR_LOW_VOLTAGE;
		}

		/*
		 * SDIO slot (AP6256): force host_ocr to 1.8V-ONLY.
		 *
		 * ROUND 10.  Round 9 made the slot EMBEDDED so sdhci_init_slot()
		 * folded 1.8V (MMC_OCR_LOW_VOLTAGE) INTO host_ocr -- but host_ocr
		 * then held {3.3V | 1.8V}, and the MMCCAM probe picks the bus
		 * voltage with mmc_highest_voltage(host_ocr), which returns the
		 * HIGHEST bit = 3.3V.  So sdhci_set_power() still drove
		 * SDHCI_POWER_330 (PWR reg 0x0f) on the 1.8V-only AP6256 (S18=1
		 * signaling but VDD=3.3V -- mismatched), and the card stayed mute.
		 *
		 * The AP6256 VDDIO is fixed 1.8V (dcdc_3), so the host must offer
		 * ONLY 1.8V for this slot.  Clear every 3.0/3.3V OCR bit, leaving
		 * just MMC_OCR_LOW_VOLTAGE, so mmc_highest_voltage() returns 1.8V
		 * and sdhci_set_power() drives SDHCI_POWER_180 (PWR reg 0x0b).
		 * host_ocr is the single source read by both the MMCCAM
		 * GET_TRAN_SETTINGS path and the legacy bridge, so masking it here
		 * (after sdhci_init_slot built it) fixes the voltage selection at
		 * the source.  SDIO slot only -- the root-SD slot must stay
		 * 3.3V-capable and is untouched.
		 */
		if (((struct sdhci_fdt_spacemit_softc *)sc)->sdio_slot) {
			sc->slots[0].host.host_ocr &= ~(MMC_OCR_290_300 |
			    MMC_OCR_300_310 | MMC_OCR_320_330 | MMC_OCR_330_340);
			sc->slots[0].host.host_ocr |= MMC_OCR_LOW_VOLTAGE;
			if (bootverbose)
				device_printf(dev,
				    "SDIO slot: host_ocr forced 1.8V-only = 0x%x\n",
				    sc->slots[0].host.host_ocr);
		}

		((struct sdhci_fdt_spacemit_softc *)sc)->gpio =
		    sdhci_fdt_gpio_setup(dev, &sc->slots[0]);

		if (bootverbose)
			device_printf(dev,
			    "PRESENT_STATE=0x%08x cd-gpio present=%d\n",
			    bus_read_4(sc->mem_res[0], SDHCI_PRESENT_STATE),
			    sdhci_fdt_spacemit_get_card_present(dev,
			    &sc->slots[0]));

		sdhci_handle_card_present(&sc->slots[0],
		    sdhci_fdt_spacemit_get_card_present(dev, &sc->slots[0]));
	}

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "vendor_regs", CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, sdhci_fdt_spacemit_sysctl_vendor_regs, "A",
	    "Live dump of vendor/CD registers (OP_EXT, HOST_CONTROL CDTEST, "
	    "S18) to verify the SDIO-slot overrides stuck");

	return (0);
}

/*
 * The eMMC is soldered down (DT "non-removable") and has no write-protect
 * line; the SDHCI WRITE_PROTECT present-state bit is meaningless for it and
 * reads as protected, which forces the root fs read-only and panics
 * ("UFS: root fs would be forcibly unmounted").  A microSD slot has no
 * write-protect line either -- the RV2's boot card came up "(read-only)" the
 * same way and panicked in the rc "mount -u" -- and its DT says so with
 * "disable-wp".  Report a non-removable or write-protect-less slot writable
 * and fall back to the present-state bit, honouring "wp-inverted", only for
 * a slot that describes neither.
 */
static int
sdhci_fdt_spacemit_get_ro(device_t brdev, device_t reqdev)
{
	struct sdhci_fdt_spacemit_softc *sc = device_get_softc(brdev);

	if (sc->non_removable || sc->wp_disabled)
		return (0);
	return (sdhci_generic_get_ro(brdev, reqdev) ^ sc->wp_inverted);
}

/*
 * eMMC bus-timing hook.  After the generic HOST_CONTROL2 UHS write, program the
 * SpacemiT vendor MMC_CTRL mode bits and TX-clock source, and for HS400
 * Enhanced Strobe the strobe enable + PHY DLL lock.  HS400ES uses the device
 * data strobe, so there is no per-unit RX tuning -- just this fixed register
 * recipe ending in a DLL lock poll (ported from the vendor U-Boot driver).
 */
static void
sdhci_fdt_spacemit_set_uhs_timing(device_t brdev, struct sdhci_slot *slot)
{
	struct sdhci_fdt_spacemit_softc *sc = device_get_softc(brdev);
	struct resource *mem = sc->base.mem_res[slot->num];
	enum mmc_bus_timing timing = slot->host.ios.timing;
	uint32_t v;
	int i;

	/* Generic path first: HOST_CONTROL2 UHS/HS400 field + set_clock. */
	sdhci_generic_set_uhs_timing(brdev, slot);

	/* TX clock source: internal for <=SDR50, PHY-derived for HS200/HS400. */
	v = bus_read_4(mem, SPACEMIT_SDHC_TX_CFG_REG);
	if (timing == bus_timing_mmc_hs200 ||
	    timing == bus_timing_mmc_hs400 ||
	    timing == bus_timing_mmc_hs400es)
		v &= ~SDHC_TX_INT_CLK_SEL;
	else
		v |= SDHC_TX_INT_CLK_SEL;
	bus_write_4(mem, SPACEMIT_SDHC_TX_CFG_REG, v);

	/* MMC_CTRL mode select. */
	v = bus_read_4(mem, SPACEMIT_SDHC_MMC_CTRL_REG);
	v &= ~(SDHC_MMC_HS200 | SDHC_MMC_HS400 | SDHC_ENHANCE_STROBE_EN);
	if (timing == bus_timing_mmc_hs200)
		v |= SDHC_MMC_HS200;
	else if (timing == bus_timing_mmc_hs400 ||
	    timing == bus_timing_mmc_hs400es)
		v |= SDHC_MMC_HS400;
	bus_write_4(mem, SPACEMIT_SDHC_MMC_CTRL_REG, v);

	if (timing != bus_timing_mmc_hs400es)
		return;

	/* Enhanced strobe + PHY DLL init/lock (strobe path: no data tuning). */
	v = bus_read_4(mem, SPACEMIT_SDHC_MMC_CTRL_REG);
	v |= SDHC_ENHANCE_STROBE_EN;
	bus_write_4(mem, SPACEMIT_SDHC_MMC_CTRL_REG, v);

	v = bus_read_4(mem, SPACEMIT_SDHC_PHY_DLLCFG);
	v |= SDHC_DLL_PREDLY_NUM | SDHC_DLL_FULLDLY_RANGE | SDHC_DLL_VREG_CTRL;
	bus_write_4(mem, SPACEMIT_SDHC_PHY_DLLCFG, v);

	v = bus_read_4(mem, SPACEMIT_SDHC_PHY_DLLCFG1);
	v |= SDHC_DLL_REG1_CTRL;
	bus_write_4(mem, SPACEMIT_SDHC_PHY_DLLCFG1, v);

	v = bus_read_4(mem, SPACEMIT_SDHC_PHY_DLLCFG);
	v |= SDHC_DLL_ENABLE;
	bus_write_4(mem, SPACEMIT_SDHC_PHY_DLLCFG, v);

	for (i = 0; i < 100; i++) {
		if (bus_read_4(mem, SPACEMIT_SDHC_PHY_DLLSTS) &
		    SDHC_DLL_LOCK_STATE)
			break;
		DELAY(10);
	}
	if (i == 100)
		device_printf(brdev, "HS400ES PHY DLL lock timeout\n");
}

static device_method_t sdhci_fdt_spacemit_methods[] = {
	/* device_if */
	DEVMETHOD(device_probe,		sdhci_fdt_spacemit_probe),
	DEVMETHOD(device_attach,	sdhci_fdt_spacemit_attach),

	/*
	 * Bus interface.
	 *
	 * MMCCAM's SDIO discovery (sdiobdiscover() -> sdio_newbus_sim_add())
	 * calls BUS_ADD_CHILD on the SDHCI controller device to hang the
	 * "sdiob" bus node (and thus the brcmfmac function driver) off it.
	 * Neither sdhci_fdt nor our subclass declared bus_add_child, so the
	 * call resolved to the kobj default null_add_child(), which is a
	 * panic ("bus_add_child is not implemented") the moment the AP6256
	 * answers CMD5 and discovery runs.  The MMCCAM-proven SDIO hosts
	 * (bcm2835_sdhci -- narqo's brcmfmac reference platform -- and
	 * sdhci_acpi) all provide exactly this method.
	 */
	DEVMETHOD(bus_add_child,	bus_generic_add_child),

	/* SDHCI registers accessors */
	DEVMETHOD(sdhci_read_4,		sdhci_fdt_spacemit_read_4),
	DEVMETHOD(sdhci_write_1,	sdhci_fdt_spacemit_write_1),
	DEVMETHOD(sdhci_write_2,	sdhci_fdt_spacemit_write_2),
	/*
	 * PIO data-port multi accessors (required once we override the single
	 * accessors; the generic PIO path calls these -- see round 39).
	 */
	DEVMETHOD(sdhci_read_multi_4,	sdhci_fdt_spacemit_read_multi_4),
	DEVMETHOD(sdhci_write_multi_4,	sdhci_fdt_spacemit_write_multi_4),
	DEVMETHOD(sdhci_get_card_present, sdhci_fdt_spacemit_get_card_present),
	DEVMETHOD(mmcbr_get_ro,		sdhci_fdt_spacemit_get_ro),
	DEVMETHOD(mmcbr_update_ios,	sdhci_fdt_spacemit_update_ios),
	DEVMETHOD(sdhci_set_uhs_timing,	sdhci_fdt_spacemit_set_uhs_timing),

	DEVMETHOD_END
};

extern driver_t sdhci_fdt_driver;

DEFINE_CLASS_1(sdhci_spacemit, sdhci_fdt_spacemit_driver,
    sdhci_fdt_spacemit_methods, sizeof(struct sdhci_fdt_spacemit_softc),
    sdhci_fdt_driver);
DRIVER_MODULE(sdhci_spacemit, simplebus, sdhci_fdt_spacemit_driver, NULL,
    NULL);

#ifndef MMCCAM
MMC_DECLARE_BRIDGE(sdhci_spacemit);
#endif
