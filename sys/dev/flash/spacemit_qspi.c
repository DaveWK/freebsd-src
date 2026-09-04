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
 * Driver for the SpacemiT K1 (Ky X1) QSPI controller with an attached
 * SPI-NOR boot flash (compatible: "spacemit,k1-qspi", node spi@d420c000 on
 * the Orange Pi RV2).  On this board the flash holds the boot chain
 * (bootinfo | private | fsbl | env | opensbi | uboot); the primary use case
 * is reading/updating the U-Boot environment at offset 0x60000 (64 KiB).
 *
 * The controller is the NXP/Freescale QuadSPI IP (the FlexSPI predecessor
 * with the LUT-based sequence engine).  Mainline Linux drives it with
 * drivers/spi/spi-fsl-qspi.c, which gained "spacemit,k1-qspi" support in
 * Alex Elder's "spi: enable the SpacemiT K1 SoC QSPI" series.  Register
 * offsets, LUT encoding and the K1 instance parameters (128-byte RX FIFO,
 * 256-byte TX FIFO, TKT253890 fill-to-16-bytes TX erratum, little-endian
 * registers) were derived from that driver (GPL-2.0) but the code here is
 * a from-scratch FreeBSD reimplementation, NOT a copy -- same convention
 * as our other spacemit drivers (see spacemit_spi.c).
 *
 * DESIGN NOTE -- why this is a monolithic flash driver and not a spibus(4)
 * controller with mx25l(4) on top: the QuadSPI IP is a sequence engine.
 * Each chip-select assertion executes one programmed LUT sequence
 * (command / address / dummy / data phases) with hard FIFO limits (128 B
 * in, 256 B out per IP command).  It cannot perform the arbitrary-length,
 * opaque full-duplex shifts that spibus(4) transfers assume -- mx25l(4)
 * issues a single FAST_READ transfer for up to 64 KiB with the address
 * hidden in an opaque command buffer, which a sequence engine can only
 * honor by protocol-sniffing the command bytes and re-issuing chunked
 * sequences.  Rather than build that fragile decoding layer, this driver
 * talks to the NOR itself (it owns the protocol, so it chunks naturally)
 * and exposes the same disk(9) surface as mx25l(4): /dev/flash/qspi0,
 * 512-byte read granularity, writes must be erase-sector (4 KiB) aligned
 * and are handled erase-then-program.
 *
 * Clocks/resets: intentionally not touched.  U-Boot itself boots from this
 * flash, so the QSPI clock tree is guaranteed enabled and the controller
 * out of reset when we attach; the K1 CCU driver in this tree exposes the
 * relevant APMU clocks read-only anyway.  We also do not reprogram the SPI
 * clock rate -- we inherit the (conservative) firmware setting.
 *
 * SAFETY: this device is the BOOT FLASH.  A bad write to fsbl/opensbi/
 * uboot regions leaves the board recoverable only via K1 maskrom USB
 * download mode.  Therefore all writes are refused unless the sysctl
 *   dev.spacemit_qspi.<unit>.allow_write = 1
 * is explicitly set (default 0, resets to 0 are never automatic -- userland
 * tooling is expected to set it, write, verify, and clear it).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bio.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <machine/bus.h>

#include <geom/geom_disk.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

/*
 * QuadSPI register offsets (from mainline drivers/spi/spi-fsl-qspi.c;
 * K1 instance is little-endian).
 */
#define	QSPI_MCR		0x00
#define	 MCR_RESERVED		0x000f0000	/* bits 19:16, keep set */
#define	 MCR_MDIS		(1u << 14)
#define	 MCR_CLR_TXF		(1u << 11)
#define	 MCR_CLR_RXF		(1u << 10)
#define	 MCR_END_CFG		0x0000000c	/* bits 3:2 */
#define	 MCR_SWRSTHD		(1u << 1)
#define	 MCR_SWRSTSD		(1u << 0)
#define	QSPI_IPCR		0x08
#define	 IPCR_SEQID(x)		((uint32_t)(x) << 24)
#define	QSPI_FLSHCR		0x0c
#define	QSPI_BUF0CR		0x10
#define	QSPI_BUF1CR		0x14
#define	QSPI_BUF2CR		0x18
#define	 BUFXCR_INVALID_MSTRID	0xe
#define	QSPI_BUF3CR		0x1c
#define	 BUF3CR_ALLMST		(1u << 31)
#define	 BUF3CR_ADATSZ(x)	((uint32_t)(x) << 8)
#define	QSPI_BFGENCR		0x20
#define	 BFGENCR_SEQID(x)	((uint32_t)(x) << 12)
#define	QSPI_BUF0IND		0x30
#define	QSPI_BUF1IND		0x34
#define	QSPI_BUF2IND		0x38
#define	QSPI_SFAR		0x100
#define	QSPI_SMPR		0x108
#define	 SMPR_DDRSMP		0x00070000	/* bits 18:16 */
#define	 SMPR_FSDLY		(1u << 6)
#define	 SMPR_FSPHS		(1u << 5)
#define	 SMPR_HSENA		(1u << 0)
#define	QSPI_RBCT		0x110
#define	 RBCT_WMRK_MASK		0x1f
#define	 RBCT_RXBRD_USEIPS	(1u << 8)
#define	QSPI_TBDR		0x154
#define	QSPI_SR			0x15c
#define	 SR_IP_ACC		(1u << 1)
#define	 SR_AHB_ACC		(1u << 2)
#define	QSPI_FR			0x160
#define	 FR_TFF			(1u << 0)
#define	QSPI_RSER		0x164
#define	QSPI_SPTRCLR		0x16c
#define	 SPTRCLR_IPPTRC		(1u << 8)
#define	 SPTRCLR_BFPTRC		(1u << 0)
#define	QSPI_SFA1AD		0x180
#define	QSPI_SFA2AD		0x184
#define	QSPI_SFB1AD		0x188
#define	QSPI_SFB2AD		0x18c
#define	QSPI_RBDR(x)		(0x200 + (x) * 4)
#define	QSPI_LUTKEY		0x300
#define	 LUTKEY_VALUE		0x5af05af0
#define	QSPI_LCKCR		0x304
#define	 LCKCR_LOCK		(1u << 0)
#define	 LCKCR_UNLOCK		(1u << 1)
#define	QSPI_LUT_BASE		0x310

/* LUT instruction opcodes. */
#define	LUT_STOP		0
#define	LUT_CMD			1
#define	LUT_ADDR		2
#define	LUT_DUMMY		3
#define	LUT_MODE		4
#define	LUT_READ		7
#define	LUT_WRITE		8

/* All our transfers are plain single-line SPI: pad field = 0 (1 line). */
#define	LUT_ENT(ins, pad, opr)	((((ins) << 10) | ((pad) << 8) | (opr)))

/*
 * We use the last LUT sequence slot and rewrite it per operation, like the
 * Linux driver does (slot 0 holds the firmware's AHB read sequence; we do
 * not disturb it beyond the mandatory module reset at attach).
 */
#define	SEQID			15

/* K1 instance parameters (spacemit_k1_data in spi-fsl-qspi.c). */
#define	K1_RXFIFO		128	/* bytes */
#define	K1_TXFIFO		256	/* bytes */
#define	K1_AHB_BUF		512	/* bytes */
#define	K1_SFA_SIZE		1024	/* per-CS AHB window stride */
#define	K1_MEMMAP_DFLT		0xb8000000u	/* AHB window (2nd reg) */

/* Conservative chunk for IP-command reads (RX FIFO is 128 B; Linux caps
 * FIFO-path reads at rxfifo-4). */
#define	QSPI_READ_CHUNK		64

/* SPI-NOR command set (single-line, 3-byte addressing only). */
#define	NOR_CMD_WRSR		0x01
#define	NOR_CMD_PAGE_PROGRAM	0x02
#define	NOR_CMD_WRDI		0x04
#define	NOR_CMD_RDSR		0x05
#define	 NOR_SR_WIP		(1u << 0)
#define	 NOR_SR_WEL		(1u << 1)
#define	NOR_CMD_WREN		0x06
#define	NOR_CMD_FAST_READ	0x0b
#define	NOR_CMD_SECTOR_ERASE_4K	0x20
#define	NOR_CMD_RDID		0x9f

#define	NOR_PAGE_SIZE		256
#define	NOR_SECTOR_4K		4096

/* Timeouts (microseconds). */
#define	TO_CTRL_BUSY		100000		/* controller SR busy */
#define	TO_IP_DONE		200000		/* one IP sequence */
#define	TO_WIP_PROGRAM		30000		/* page program (typ 0.7 ms) */
#define	TO_WIP_ERASE		3000000		/* 4K erase (typ 45-120 ms) */

/* Disk surface, mirroring mx25l(4). */
#define	QSPI_DISK_SECTORSIZE	512

struct k1qspi_softc {
	device_t		 sc_dev;
	struct resource		*sc_mem;
	uint32_t		 sc_memmap_phy;	/* AHB window phys base */
	struct mtx		 sc_mtx;
	struct disk		*sc_disk;
	struct proc		*sc_p;
	struct bio_queue_head	 sc_bio_queue;
	int			 sc_taskstate;
	uint8_t			 sc_jedec[3];
	off_t			 sc_mediasize;
	u_int			 sc_erasesize;
	int			 sc_known;	/* JEDEC vendor recognized */
	int			 sc_allow_write;	/* sysctl gate */
	uint8_t			 sc_pagebuf[NOR_PAGE_SIZE];
};

#define	TSTATE_STOPPED	0
#define	TSTATE_STOPPING	1
#define	TSTATE_RUNNING	2

#define	K1QSPI_LOCK(sc)		mtx_lock(&(sc)->sc_mtx)
#define	K1QSPI_UNLOCK(sc)	mtx_unlock(&(sc)->sc_mtx)

#define	RD4(sc, off)		bus_read_4((sc)->sc_mem, (off))
#define	WR4(sc, off, val)	bus_write_4((sc)->sc_mem, (off), (val))

/*
 * JEDEC manufacturers we trust for the generic "capacity byte" size rule
 * and universal 0x20 4-KiB sector erase.  Anything else attaches read-only.
 */
static const struct {
	uint8_t		id;
	const char	*name;
} k1qspi_vendors[] = {
	{ 0xc8, "GigaDevice" },
	{ 0xef, "Winbond" },
	{ 0x20, "Micron/XMC" },
	{ 0xc2, "Macronix" },
	{ 0x1c, "EON" },
	{ 0xa1, "Fudan" },
	{ 0x0b, "XTX" },
	{ 0x68, "Boya" },
	{ 0x5e, "Zbit" },
	{ 0x9d, "ISSI" },
};

static struct ofw_compat_data compat_data[] = {
	{ "spacemit,k1-qspi",	1 },
	{ NULL,			0 },
};

/*
 * Wait for a masked register condition; returns 0 or ETIMEDOUT.
 * Polling (no interrupts): sequences are short and this path is rare.
 */
static int
k1qspi_poll(struct k1qspi_softc *sc, bus_size_t reg, uint32_t mask,
    uint32_t want, u_int timo_us)
{
	u_int i;

	for (i = 0; i < timo_us; i++) {
		if ((RD4(sc, reg) & mask) == want)
			return (0);
		DELAY(1);
	}
	return (ETIMEDOUT);
}

/*
 * Execute one flash operation through the IP command engine.
 *
 * opcode:      SPI-NOR command byte (1 line)
 * addrlen:     0 or 3 address bytes (sent as LUT_MODE literals, matching
 *              the Linux driver's approach so the bytes go out verbatim)
 * dummy_bytes: 0 or 1 (8 clock cycles on 1 line)
 * dir/buf/len: optional data phase; IN <= K1_RXFIFO, OUT <= K1_TXFIFO
 */
#define	OP_NONE	0
#define	OP_IN	1
#define	OP_OUT	2

static int
k1qspi_op(struct k1qspi_softc *sc, uint8_t opcode, int addrlen, uint32_t addr,
    int dummy_bytes, int dir, void *buf, uint32_t len)
{
	uint32_t lut[4];
	uint32_t val, mcr;
	u_int i, idx, nwords;
	int err;

	KASSERT(addrlen == 0 || addrlen == 3, ("k1qspi: bad addrlen"));
	KASSERT(dir != OP_IN || len <= K1_RXFIFO, ("k1qspi: IN too long"));
	KASSERT(dir != OP_OUT || len <= K1_TXFIFO, ("k1qspi: OUT too long"));

	/* Controller idle (neither IP nor AHB access in flight). */
	err = k1qspi_poll(sc, QSPI_SR, SR_IP_ACC | SR_AHB_ACC, 0,
	    TO_CTRL_BUSY);
	if (err != 0) {
		device_printf(sc->sc_dev, "controller busy, SR=%#x\n",
		    RD4(sc, QSPI_SR));
		return (err);
	}

	/* CS0: serial flash address = start of the AHB window. */
	WR4(sc, QSPI_SFAR, sc->sc_memmap_phy);

	/* Clear FIFOs and both sequence pointers. */
	WR4(sc, QSPI_MCR, RD4(sc, QSPI_MCR) | MCR_CLR_TXF | MCR_CLR_RXF);
	WR4(sc, QSPI_SPTRCLR, SPTRCLR_BFPTRC | SPTRCLR_IPPTRC);
	WR4(sc, QSPI_BUF0CR, BUFXCR_INVALID_MSTRID);
	WR4(sc, QSPI_BUF1CR, BUFXCR_INVALID_MSTRID);
	WR4(sc, QSPI_BUF2CR, BUFXCR_INVALID_MSTRID);

	/*
	 * Build the LUT sequence.  Each 32-bit LUT word holds two 16-bit
	 * entries.  Layout: CMD [MODE-addr x3] [DUMMY] [READ|WRITE] STOP.
	 * Worst case 6 entries -- fits the 8-entry sequence with room.
	 */
	memset(lut, 0, sizeof(lut));
	idx = 0;
#define	LUT_APPEND(ins, opr) do {					\
	lut[idx / 2] |= (uint32_t)LUT_ENT((ins), 0, (opr)) <<		\
	    ((idx % 2) * 16);						\
	idx++;								\
} while (0)
	LUT_APPEND(LUT_CMD, opcode);
	for (i = 0; i < (u_int)addrlen; i++)
		LUT_APPEND(LUT_MODE, (addr >> (8 * (addrlen - i - 1))) & 0xff);
	if (dummy_bytes != 0)
		LUT_APPEND(LUT_DUMMY, dummy_bytes * 8);
	if (dir != OP_NONE && len != 0)
		LUT_APPEND(dir == OP_IN ? LUT_READ : LUT_WRITE, 0);
	LUT_APPEND(LUT_STOP, 0);
#undef LUT_APPEND

	WR4(sc, QSPI_LUTKEY, LUTKEY_VALUE);
	WR4(sc, QSPI_LCKCR, LCKCR_UNLOCK);
	for (i = 0; i < nitems(lut); i++)
		WR4(sc, QSPI_LUT_BASE + SEQID * 16 + i * 4, lut[i]);
	WR4(sc, QSPI_LUTKEY, LUTKEY_VALUE);
	WR4(sc, QSPI_LCKCR, LCKCR_LOCK);

	/* IP-command RX buffer readout via RBDR registers. */
	WR4(sc, QSPI_RBCT, RBCT_WMRK_MASK | RBCT_RXBRD_USEIPS);

	if (dir == OP_OUT && len != 0) {
		const uint8_t *p = buf;

		for (i = 0; i + 4 <= len; i += 4) {
			memcpy(&val, p + i, 4);	/* LE regs, LE cpu */
			WR4(sc, QSPI_TBDR, val);
		}
		if (i < len) {
			val = 0;
			memcpy(&val, p + i, len - i);
			WR4(sc, QSPI_TBDR, val);
		}
		/*
		 * TKT253890 erratum (present on the K1 instance): the
		 * engine only starts when the TX FIFO holds >= 16 bytes;
		 * pad with dummy words that are not transferred.
		 */
		for (i = roundup2(len, 4); i < 16; i += 4)
			WR4(sc, QSPI_TBDR, 0);
	}

	/* Clear stale flags, then trigger the sequence. */
	WR4(sc, QSPI_FR, 0xffffffff);
	WR4(sc, QSPI_IPCR, len | IPCR_SEQID(SEQID));

	err = k1qspi_poll(sc, QSPI_FR, FR_TFF, FR_TFF, TO_IP_DONE);
	if (err != 0) {
		device_printf(sc->sc_dev,
		    "IP command timeout, op %#x FR=%#x SR=%#x\n",
		    opcode, RD4(sc, QSPI_FR), RD4(sc, QSPI_SR));
	} else if (dir == OP_IN && len != 0) {
		uint8_t *p = buf;

		nwords = (len + 3) / 4;
		for (i = 0; i < nwords; i++) {
			val = RD4(sc, QSPI_RBDR(i));
			memcpy(p + i * 4, &val, MIN(4, len - i * 4));
		}
	}
	WR4(sc, QSPI_FR, 0xffffffff);

	/*
	 * Invalidate the AHB buffer (reset AHB + serial flash domains) so
	 * later AHB users never see stale data after our writes/erases.
	 */
	mcr = RD4(sc, QSPI_MCR);
	WR4(sc, QSPI_MCR, mcr | MCR_SWRSTHD | MCR_SWRSTSD);
	DELAY(1);
	WR4(sc, QSPI_MCR, mcr & ~(MCR_SWRSTHD | MCR_SWRSTSD));

	return (err);
}

static int
k1qspi_wait_ready(struct k1qspi_softc *sc, u_int timo_us)
{
	uint8_t sr;
	u_int waited;
	int err;

	for (waited = 0; waited < timo_us; waited += 100) {
		sr = NOR_SR_WIP;
		err = k1qspi_op(sc, NOR_CMD_RDSR, 0, 0, 0, OP_IN, &sr, 1);
		if (err != 0)
			return (err);
		if ((sr & NOR_SR_WIP) == 0)
			return (0);
		DELAY(100);
	}
	return (ETIMEDOUT);
}

static int
k1qspi_read_range(struct k1qspi_softc *sc, off_t offset, caddr_t data,
    off_t count)
{
	uint32_t chunk;
	int err;

	if (offset < 0 || count < 0 ||
	    offset + count > sc->sc_mediasize)
		return (EIO);

	while (count > 0) {
		chunk = MIN(count, QSPI_READ_CHUNK);
		err = k1qspi_op(sc, NOR_CMD_FAST_READ, 3, (uint32_t)offset,
		    1, OP_IN, data, chunk);
		if (err != 0)
			return (err);
		offset += chunk;
		data += chunk;
		count -= chunk;
	}
	return (0);
}

static int
k1qspi_erase_sector(struct k1qspi_softc *sc, off_t offset)
{
	int err;

	if ((err = k1qspi_op(sc, NOR_CMD_WREN, 0, 0, 0, OP_NONE, NULL, 0)))
		return (err);
	if ((err = k1qspi_op(sc, NOR_CMD_SECTOR_ERASE_4K, 3,
	    (uint32_t)offset, 0, OP_NONE, NULL, 0)))
		return (err);
	return (k1qspi_wait_ready(sc, TO_WIP_ERASE));
}

static int
k1qspi_write_range(struct k1qspi_softc *sc, off_t offset, caddr_t data,
    off_t count)
{
	off_t chunk;
	int err;

	if (!sc->sc_allow_write) {
		device_printf(sc->sc_dev,
		    "write refused: set dev.spacemit_qspi.%d.allow_write=1 "
		    "(BOOT FLASH -- know what you are doing)\n",
		    device_get_unit(sc->sc_dev));
		return (EPERM);
	}
	if (!sc->sc_known)
		return (EOPNOTSUPP);	/* unrecognized JEDEC vendor */

	/*
	 * Writes must be aligned to the erase sector size, since sectors
	 * are fully erased before being (re)programmed -- mx25l semantics.
	 */
	if (count % sc->sc_erasesize != 0 || offset % sc->sc_erasesize != 0)
		return (EIO);
	if (offset < 0 || offset + count > sc->sc_mediasize)
		return (EIO);

	while (count > 0) {
		if ((offset % sc->sc_erasesize) == 0) {
			err = k1qspi_erase_sector(sc, offset);
			if (err != 0)
				return (err);
		}

		chunk = MIN(NOR_PAGE_SIZE, count);
		if ((err = k1qspi_wait_ready(sc, TO_WIP_PROGRAM)))
			return (err);
		if ((err = k1qspi_op(sc, NOR_CMD_WREN, 0, 0, 0, OP_NONE,
		    NULL, 0)))
			return (err);
		/* Copy to an aligned bounce buffer (bio data may move). */
		memcpy(sc->sc_pagebuf, data, chunk);
		err = k1qspi_op(sc, NOR_CMD_PAGE_PROGRAM, 3, (uint32_t)offset,
		    0, OP_OUT, sc->sc_pagebuf, chunk);
		if (err != 0)
			return (err);
		if ((err = k1qspi_wait_ready(sc, TO_WIP_PROGRAM)))
			return (err);

		offset += chunk;
		data += chunk;
		count -= chunk;
	}
	return (0);
}

/*
 * One-time controller setup, mirroring fsl_qspi_default_setup() minus the
 * clock manipulation (see the header comment for why we skip clocks).
 */
static void
k1qspi_hw_setup(struct k1qspi_softc *sc)
{
	uint32_t reg;

	/* Reset both domains, then disable the module for configuration. */
	WR4(sc, QSPI_MCR, MCR_SWRSTSD | MCR_SWRSTHD);
	DELAY(1);
	WR4(sc, QSPI_MCR, MCR_MDIS | MCR_RESERVED);

	reg = RD4(sc, QSPI_SMPR);
	WR4(sc, QSPI_SMPR,
	    reg & ~(SMPR_FSDLY | SMPR_FSPHS | SMPR_HSENA | SMPR_DDRSMP));

	/* AHB buffers 0-2 unused; buffer 3 catches all masters. */
	WR4(sc, QSPI_BUF0IND, 0);
	WR4(sc, QSPI_BUF1IND, 0);
	WR4(sc, QSPI_BUF2IND, 0);
	WR4(sc, QSPI_BFGENCR, BFGENCR_SEQID(SEQID));
	WR4(sc, QSPI_RBCT, RBCT_WMRK_MASK);
	WR4(sc, QSPI_BUF3CR, BUF3CR_ALLMST | BUF3CR_ADATSZ(K1_AHB_BUF / 8));

	/* Four CS regions of K1_SFA_SIZE each above the AHB window base. */
	WR4(sc, QSPI_SFA1AD, sc->sc_memmap_phy + 1 * K1_SFA_SIZE);
	WR4(sc, QSPI_SFA2AD, sc->sc_memmap_phy + 2 * K1_SFA_SIZE);
	WR4(sc, QSPI_SFB1AD, sc->sc_memmap_phy + 3 * K1_SFA_SIZE);
	WR4(sc, QSPI_SFB2AD, sc->sc_memmap_phy + 4 * K1_SFA_SIZE);

	/* Enable the module; clear + keep all interrupts masked (polled). */
	WR4(sc, QSPI_MCR, MCR_RESERVED | MCR_END_CFG);
	WR4(sc, QSPI_FR, 0xffffffff);
	WR4(sc, QSPI_RSER, 0);
}

/* --- disk(9) surface (mx25l-style) ---------------------------------- */

static int
k1qspi_open(struct disk *dp)
{
	return (0);
}

static int
k1qspi_close(struct disk *dp)
{
	return (0);
}

static int
k1qspi_ioctl(struct disk *dp, u_long cmd, void *data, int fflag,
    struct thread *td)
{
	return (EINVAL);
}

static void
k1qspi_strategy(struct bio *bp)
{
	struct k1qspi_softc *sc;

	sc = (struct k1qspi_softc *)bp->bio_disk->d_drv1;
	K1QSPI_LOCK(sc);
	bioq_disksort(&sc->sc_bio_queue, bp);
	wakeup(sc);
	K1QSPI_UNLOCK(sc);
}

static void
k1qspi_task(void *arg)
{
	struct k1qspi_softc *sc = arg;
	struct bio *bp;

	for (;;) {
		K1QSPI_LOCK(sc);
		do {
			if (sc->sc_taskstate == TSTATE_STOPPING) {
				sc->sc_taskstate = TSTATE_STOPPED;
				K1QSPI_UNLOCK(sc);
				wakeup(sc);
				kproc_exit(0);
			}
			bp = bioq_first(&sc->sc_bio_queue);
			if (bp == NULL)
				msleep(sc, &sc->sc_mtx, PRIBIO, "k1qspiq", 0);
		} while (bp == NULL);
		bioq_remove(&sc->sc_bio_queue, bp);
		K1QSPI_UNLOCK(sc);

		switch (bp->bio_cmd) {
		case BIO_READ:
			/* Reads aligned to disk sectorsize (512). */
			if (bp->bio_bcount % QSPI_DISK_SECTORSIZE != 0 ||
			    bp->bio_offset % QSPI_DISK_SECTORSIZE != 0)
				bp->bio_error = EIO;
			else
				bp->bio_error = k1qspi_read_range(sc,
				    bp->bio_offset, bp->bio_data,
				    bp->bio_bcount);
			break;
		case BIO_WRITE:
			bp->bio_error = k1qspi_write_range(sc,
			    bp->bio_offset, bp->bio_data, bp->bio_bcount);
			break;
		default:
			bp->bio_error = EOPNOTSUPP;
			break;
		}
		if (bp->bio_error != 0)
			bp->bio_flags |= BIO_ERROR;
		else
			bp->bio_resid = 0;
		biodone(bp);
	}
}

/* --- newbus ----------------------------------------------------------- */

static int
k1qspi_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "SpacemiT K1 QSPI SPI-NOR (boot flash)");
	return (BUS_PROBE_DEFAULT);
}

static int
k1qspi_attach(device_t dev)
{
	struct k1qspi_softc *sc;
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;
	rman_res_t start, count;
	const char *vendor;
	uint8_t id[3];
	u_int i;
	int err, rid;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	rid = 0;
	sc->sc_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->sc_mem == NULL) {
		device_printf(dev, "cannot map registers\n");
		return (ENXIO);
	}

	/*
	 * Second reg entry ("QuadSPI-memory") is the AHB-mapped flash
	 * window; we only need its physical base for SFAR/SFxAD.
	 */
	if (bus_get_resource(dev, SYS_RES_MEMORY, 1, &start, &count) == 0)
		sc->sc_memmap_phy = (uint32_t)start;
	else
		sc->sc_memmap_phy = K1_MEMMAP_DFLT;

	mtx_init(&sc->sc_mtx, device_get_nameunit(dev), "k1qspi", MTX_DEF);

	k1qspi_hw_setup(sc);

	/* JEDEC ID (0x9F). */
	memset(id, 0, sizeof(id));
	err = k1qspi_op(sc, NOR_CMD_RDID, 0, 0, 0, OP_IN, id, sizeof(id));
	if (err != 0)
		goto fail;
	memcpy(sc->sc_jedec, id, 3);
	if ((id[0] == 0x00 && id[1] == 0x00) ||
	    (id[0] == 0xff && id[1] == 0xff)) {
		device_printf(dev, "no flash detected (JEDEC %02x %02x %02x)\n",
		    id[0], id[1], id[2]);
		err = ENXIO;
		goto fail;
	}

	vendor = NULL;
	for (i = 0; i < nitems(k1qspi_vendors); i++) {
		if (k1qspi_vendors[i].id == id[0]) {
			vendor = k1qspi_vendors[i].name;
			break;
		}
	}
	sc->sc_known = (vendor != NULL);

	/*
	 * Standard capacity encoding: third ID byte = log2(size in bytes).
	 * We only implement 3-byte addressing, so cap at 16 MiB.
	 */
	if (id[2] >= 0x11 && id[2] <= 0x1a)
		sc->sc_mediasize = (off_t)1 << id[2];
	else
		sc->sc_mediasize = 16 * 1024 * 1024;
	if (sc->sc_mediasize > 16 * 1024 * 1024) {
		device_printf(dev,
		    "chip larger than 16 MiB; limiting to 3-byte range\n");
		sc->sc_mediasize = 16 * 1024 * 1024;
	}
	sc->sc_erasesize = NOR_SECTOR_4K;

	device_printf(dev,
	    "JEDEC %02x %02x %02x (%s), %jd KiB, 4 KiB erase%s\n",
	    id[0], id[1], id[2], vendor != NULL ? vendor : "unknown vendor",
	    (intmax_t)(sc->sc_mediasize / 1024),
	    sc->sc_known ? "" : " -- READ-ONLY (vendor not in table)");

	/* Write gate. */
	ctx = device_get_sysctl_ctx(dev);
	tree = device_get_sysctl_tree(dev);
	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "allow_write",
	    CTLFLAG_RW, &sc->sc_allow_write, 0,
	    "allow writes to the boot SPI-NOR (DANGER: 0x0-0x60000 and "
	    "0x70000+ hold SPL/OpenSBI/U-Boot)");

	sc->sc_disk = disk_alloc();
	sc->sc_disk->d_open = k1qspi_open;
	sc->sc_disk->d_close = k1qspi_close;
	sc->sc_disk->d_strategy = k1qspi_strategy;
	sc->sc_disk->d_ioctl = k1qspi_ioctl;
	sc->sc_disk->d_name = "flash/qspi";
	sc->sc_disk->d_drv1 = sc;
	sc->sc_disk->d_maxsize = DFLTPHYS;
	sc->sc_disk->d_sectorsize = QSPI_DISK_SECTORSIZE;
	sc->sc_disk->d_mediasize = sc->sc_mediasize;
	sc->sc_disk->d_stripesize = sc->sc_erasesize;
	sc->sc_disk->d_unit = device_get_unit(dev);
	sc->sc_disk->d_dump = NULL;
	snprintf(sc->sc_disk->d_descr, sizeof(sc->sc_disk->d_descr),
	    "K1 boot SPI-NOR (JEDEC %02x%02x%02x)", id[0], id[1], id[2]);

	disk_create(sc->sc_disk, DISK_VERSION);
	bioq_init(&sc->sc_bio_queue);
	kproc_create(&k1qspi_task, sc, &sc->sc_p, 0, 0, "task: k1qspi flash");
	sc->sc_taskstate = TSTATE_RUNNING;

	return (0);

fail:
	mtx_destroy(&sc->sc_mtx);
	bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->sc_mem);
	return (err);
}

static int
k1qspi_detach(device_t dev)
{
	struct k1qspi_softc *sc;
	int err;

	sc = device_get_softc(dev);
	err = 0;

	K1QSPI_LOCK(sc);
	if (sc->sc_taskstate == TSTATE_RUNNING) {
		sc->sc_taskstate = TSTATE_STOPPING;
		wakeup(sc);
		while (err == 0 && sc->sc_taskstate != TSTATE_STOPPED) {
			err = msleep(sc, &sc->sc_mtx, 0, "k1qspid", hz * 3);
			if (err != 0) {
				sc->sc_taskstate = TSTATE_RUNNING;
				device_printf(dev,
				    "failed to stop queue task\n");
			}
		}
	}
	K1QSPI_UNLOCK(sc);

	if (err == 0 && sc->sc_taskstate == TSTATE_STOPPED) {
		disk_destroy(sc->sc_disk);
		bioq_flush(&sc->sc_bio_queue, NULL, ENXIO);
		mtx_destroy(&sc->sc_mtx);
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->sc_mem);
	}
	return (err);
}

static device_method_t k1qspi_methods[] = {
	DEVMETHOD(device_probe,		k1qspi_probe),
	DEVMETHOD(device_attach,	k1qspi_attach),
	DEVMETHOD(device_detach,	k1qspi_detach),

	DEVMETHOD_END
};

static driver_t k1qspi_driver = {
	"spacemit_qspi",
	k1qspi_methods,
	sizeof(struct k1qspi_softc),
};

DRIVER_MODULE(spacemit_qspi, simplebus, k1qspi_driver, 0, 0);
