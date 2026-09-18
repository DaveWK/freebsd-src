/*-
 * SPDX-License-Identifier: BSD-2-Clause AND ISC
 *
 * Copyright (c) 2026 Daniel Shue <dgshue@gmail.com>
 *
 * Portions derived from OpenBSD's if_smte.c:
 * Copyright (c) 2026 Mark Kettenis <kettenis@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * Driver for the Ethernet MAC of the SpacemiT K1 (aka Ky X1) RISC-V SoC
 * (compatible: spacemit,k1-emac), as found on the Orange Pi RV2 and
 * Banana Pi BPI-F3.
 *
 * The MAC uses simple descriptor rings with 32-bit DMA addresses; all DMA
 * tags are therefore restricted to the low 4 GB (the SoC has RAM up to
 * 10 GB physical, so bounce buffering handles the rest).  The RGMII
 * delay lines live in the APMU system controller, reached through the
 * "spacemit,apmu" (<phandle offset>) property via syscon(4).
 */

#include "opt_inet.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/gpio.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>
#include <sys/time.h>

#include <machine/bus.h>

#include <net/bpf.h>
#include <net/if.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_var.h>
#ifdef INET
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <netinet/tcp_lro.h>
#include <machine/in_cksum.h>
#endif

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/syscon/syscon.h>
#include <dev/gpio/gpiobusvar.h>

#include "if_smtereg.h"

#include "miibus_if.h"
#include "syscon_if.h"

#define	SMTE_NTXDESC	256
#define	SMTE_NTXSEGS	16
#define	SMTE_TX_QUEUE_MIN	(SMTE_NTXDESC - 1)
#define	SMTE_TX_QUEUE_MAX	4096

/*
 * A receive frame must fit one descriptor: RX_DESC1_SIZE1_MASK is 12 bits,
 * so the buffer handed to the engine cannot exceed 4095 bytes.  A page
 * cluster less the alignment pad is the largest that qualifies, which caps
 * the MTU below the MAC's jumbo jabber limit.  True 9K frames would need the
 * receive path to chain descriptors, which it does not do.
 */
#define	SMTE_MAX_MTU	(MJUMPAGESIZE - ETHER_ALIGN - ETHER_HDR_LEN - \
    ETHER_VLAN_ENCAP_LEN - ETHER_CRC_LEN)
CTASSERT(MJUMPAGESIZE - ETHER_ALIGN <= RX_DESC1_SIZE1_MASK);
#define	SMTE_NRXDESC	256

/* Optional diagnostic build; production does not read clocks in the fast path. */
#ifndef SMTE_SERVICE_DIAGNOSTICS
#define SMTE_SERVICE_DIAGNOSTICS 0
#endif
#define SMTE_SERVICE_NOW() (SMTE_SERVICE_DIAGNOSTICS ? sbinuptime() : 0)
#define SMTE_DMA_MISSED_FRAME_COUNTER 0x0024

enum smte_service_stat {
	SMTE_STAT_INTR_CALLS,
	SMTE_STAT_INTR_NS,
	SMTE_STAT_INTR_MAX_NS,
	SMTE_STAT_INTR_LOCK_WAIT_NS,
	SMTE_STAT_INTR_LOCK_WAIT_MAX_NS,
	SMTE_STAT_RX_CALLS,
	SMTE_STAT_RX_DESCRIPTORS,
	SMTE_STAT_RX_BUDGET_EXHAUSTED,
	SMTE_STAT_RX_BATCH_MAX,
	SMTE_STAT_RX_DRAIN_NS,
	SMTE_STAT_RX_DRAIN_MAX_NS,
	SMTE_STAT_RX_HANDOFF_NS,
	SMTE_STAT_RX_HANDOFF_MAX_NS,
	SMTE_STAT_DMA_MISSED_RAW_TOTAL,
	SMTE_STAT_DMA_MISSED_RAW_OR,
	SMTE_STAT_DMA_MISSED_RAW_MAX,
	SMTE_STAT_DMA_MISSED_DRAIN_RAW,
	SMTE_STAT_DMA_MISSED_HANDOFF_RAW,
	SMTE_STAT_DMA_MISSED_OTHER_RAW,
	SMTE_STAT_TX_USED_MAX,
	SMTE_STAT_TX_RECLAIMED,
	SMTE_STAT_TX_RECLAIM_NS,
	SMTE_STAT_TX_RECLAIM_MAX_NS,
	SMTE_STAT_TX_RING_BLOCKED,
	SMTE_STAT_TX_EARLY_RECLAIMS,
	SMTE_STAT_COUNT
};

#define	SMTE_LOCK(sc)		mtx_lock(&(sc)->mtx)
#define	SMTE_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)
#define	SMTE_ASSERT_LOCKED(sc)	mtx_assert(&(sc)->mtx, MA_OWNED)

#define	SMTE_WATCHDOG_TIMEOUT	5

struct smte_bufmap {
	bus_dmamap_t	map;
	struct mbuf	*mbuf;
	bus_addr_t	paddr; /* Device address; map retains CPU physical addresses. */
};

struct smte_softc {
#ifdef INET
	/* Single interrupt consumer; flushed at every unlocked RX handoff. */
	struct lro_ctrl sw_lro_ctrl;
	int sw_lro_ready;
	int sw_lro;
#endif
	device_t	dev;
	struct resource	*mem_res;
	struct resource	*irq_res;
	void		*intrhand;
	struct mtx	mtx;
	struct callout	tick_ch;

	if_t		ifp;
	device_t	miibus;
	struct mii_data	*mii;
	int		phyloc;
	int		link;
	int		tx_watchdog;

	struct syscon	*apmu;
	uint32_t	apmu_offset;
	uint32_t	rx_delay_ps;
	uint32_t	tx_delay_ps;

	/* Descriptor rings. */
	bus_dma_tag_t	desc_tag;
	bus_dmamap_t	txdesc_map;
	struct smte_desc *txdesc;
	bus_addr_t	txdesc_paddr;
	bus_dmamap_t	rxdesc_map;
	struct smte_desc *rxdesc;
	bus_addr_t	rxdesc_paddr;

	/* Buffers. */
	bus_dma_tag_t	txbuf_tag;
	struct smte_bufmap txbuf[SMTE_NTXDESC];
	int		tx_prod;
	int		tx_cons;
	int		tx_used;
	bus_dma_tag_t	rxbuf_tag;
	bus_dmamap_t	rx_sparemap;
	struct smte_bufmap rxbuf[SMTE_NRXDESC];
	int		rx_cons;
	int		rx_bufsize;
	int		rx_dbg;
	int		dma_translate;
	int		tx_pack;
	int		tx_queue_len;
	bus_addr_t	dma_lowaddr;
	int		tx_reclaim_first;
	uint64_t	service[SMTE_STAT_COUNT];
};

static struct ofw_compat_data compat_data[] = {
	{ "spacemit,k1-emac",	1 },
	{ NULL,			0 }
};

#define	RD4(sc, off)		bus_read_4((sc)->mem_res, (off))
#define	WR4(sc, off, val)	bus_write_4((sc)->mem_res, (off), (val))

static void smte_txeof(struct smte_softc *sc);
static void smte_rxeof(struct smte_softc *sc);
static void smte_stop_locked(struct smte_softc *sc);
static void smte_reset_rings(struct smte_softc *sc);
static void smte_dma_reset(struct smte_softc *sc);
static void smte_init_locked(struct smte_softc *sc);
static void smte_start_locked(if_t ifp);
static void smte_tick(void *arg);

/* All service counters and the experimental switch use the driver lock. */
static void
smte_service_time(struct smte_softc *sc, int total, int peak, sbintime_t start)
{
#if SMTE_SERVICE_DIAGNOSTICS
	uint64_t elapsed;

	SMTE_ASSERT_LOCKED(sc);
	elapsed = sbttons(sbinuptime() - start);
	sc->service[total] += elapsed;
	if (elapsed > sc->service[peak])
		sc->service[peak] = elapsed;
#else
	(void)sc;
	(void)total;
	(void)peak;
	(void)start;
#endif
}

static void
smte_service_missed(struct smte_softc *sc, int phase)
{
	uint32_t raw;

	SMTE_ASSERT_LOCKED(sc);
	/* Read clears the hardware counter. Keep raw bits; no overflow guess. */
	raw = RD4(sc, SMTE_DMA_MISSED_FRAME_COUNTER);
	sc->service[SMTE_STAT_DMA_MISSED_RAW_TOTAL] += raw;
	sc->service[SMTE_STAT_DMA_MISSED_RAW_OR] |= raw;
	if (raw > sc->service[SMTE_STAT_DMA_MISSED_RAW_MAX])
		sc->service[SMTE_STAT_DMA_MISSED_RAW_MAX] = raw;
	sc->service[phase] += raw;
}

static int
smte_service_counter(SYSCTL_HANDLER_ARGS)
{
	struct smte_softc *sc;
	uint64_t value;

	sc = arg1;
	if (arg2 < 0 || arg2 >= SMTE_STAT_COUNT)
		return (EINVAL);
	SMTE_LOCK(sc);
	value = sc->service[arg2];
	SMTE_UNLOCK(sc);
	return (sysctl_handle_64(oidp, &value, 0, req));
}

static int
smte_service_order(SYSCTL_HANDLER_ARGS)
{
	struct smte_softc *sc;
	int error, value;

	sc = arg1;
	SMTE_LOCK(sc);
	value = sc->tx_reclaim_first;
	SMTE_UNLOCK(sc);
	error = sysctl_handle_int(oidp, &value, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (value != 0 && value != 1)
		return (EINVAL);
	SMTE_LOCK(sc);
	sc->tx_reclaim_first = value;
	SMTE_UNLOCK(sc);
	return (0);
}

static void
smte_service_sysctls(struct smte_softc *sc)
{
	static const char * const names[SMTE_STAT_COUNT] = {
		"intr_calls",
		"intr_ns",
		"intr_max_ns",
		"intr_lock_wait_ns",
		"intr_lock_wait_max_ns",
		"rx_calls",
		"rx_descriptors",
		"rx_budget_exhausted",
		"rx_batch_max",
		"rx_drain_ns",
		"rx_drain_max_ns",
		"rx_handoff_ns",
		"rx_handoff_max_ns",
		"dma_missed_raw_total",
		"dma_missed_raw_or",
		"dma_missed_raw_max",
		"dma_missed_drain_raw",
		"dma_missed_handoff_raw",
		"dma_missed_other_raw",
		"tx_used_max",
		"tx_reclaimed",
		"tx_reclaim_ns",
		"tx_reclaim_max_ns",
		"tx_ring_blocked",
		"tx_early_reclaims",
	};
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;
	int i;

	ctx = device_get_sysctl_ctx(sc->dev);
	tree = SYSCTL_ADD_NODE(ctx,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev)), OID_AUTO,
	    "service", CTLFLAG_RD | CTLFLAG_MPSAFE, NULL,
	    "SMTE service counters; timing requires a diagnostic build");
	if (tree == NULL)
		return;
	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "timing_enabled", CTLFLAG_RD, NULL, SMTE_SERVICE_DIAGNOSTICS,
	    "Fast-path clock sampling compiled in");
	for (i = 0; i < SMTE_STAT_COUNT; i++)
		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, names[i],
		    CTLTYPE_U64 | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, i,
		    smte_service_counter, "QU", "Cumulative diagnostic (raw DMA or ns where named)");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "tx_reclaim_first", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
	    sc, 0, smte_service_order, "I",
	    "Opt-in TX reclaim before RX delivery (default 0)");
}


/* Device-scoped boot policy. Invalid feature values fail closed. */
static int
smte_boot_flag(struct smte_softc *sc, const char *feature)
{
	char name[64];
	int requested;

	requested = 0;
	snprintf(name, sizeof(name), "hw.smte.%d.%s",
	    device_get_unit(sc->dev), feature);
	TUNABLE_INT_FETCH(name, &requested);
	if (requested != 0 && requested != 1) {
		device_printf(sc->dev, "invalid %s value %d; disabled\n",
		    feature, requested);
		return (0);
	}
	return (requested);
}

/* Packet queue capacity is independent of the number of DMA descriptors.
 * Keep the existing default until the larger bounded queue passes hardware
 * acceptance. Boot-only: changing if_snd limits while enqueue runs would race.
 */
static int
smte_tx_queue_size(int requested)
{

	if (requested < SMTE_TX_QUEUE_MIN || requested > SMTE_TX_QUEUE_MAX)
		return (SMTE_TX_QUEUE_MIN);
	return (requested);
}

static void
smte_tx_queue_setup(struct smte_softc *sc)
{
	char name[64];
	int requested;

	requested = SMTE_TX_QUEUE_MIN;
	snprintf(name, sizeof(name), "hw.smte.%d.tx_queue_len",
	    device_get_unit(sc->dev));
	TUNABLE_INT_FETCH(name, &requested);
	sc->tx_queue_len = smte_tx_queue_size(requested);
	if (sc->tx_queue_len != requested)
		device_printf(sc->dev, "invalid TX queue limit %d; using %d\n",
		    requested, sc->tx_queue_len);
	if_setsendqlen(sc->ifp, sc->tx_queue_len);
}

/* Ring ownership is per descriptor; packing may change between packets. */
static int
smte_tx_pack_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct smte_softc *sc;
	int error, value;

	sc = arg1;
	SMTE_LOCK(sc);
	value = sc->tx_pack;
	SMTE_UNLOCK(sc);
	error = sysctl_handle_int(oidp, &value, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (value != 0 && value != 1)
		return (EINVAL);
	SMTE_LOCK(sc);
	sc->tx_pack = value;
	SMTE_UNLOCK(sc);
	return (0);
}

/*
 * Scoped K1 DMA-window support.  busdma remains responsible for CPU cache
 * maintenance and out-of-reach bouncing; only device addresses are translated.
 * No arbitrary 64-bit descriptor truncation is permitted.
 */
static int
smte_dma_addr(struct smte_softc *sc, bus_addr_t pa, bus_size_t len,
    bus_addr_t *da)
{
	bus_addr_t base, limit;

	if (len == 0)
		return (EINVAL);
	if (!sc->dma_translate) {
		base = 0;
		limit = BUS_SPACE_MAXADDR_32BIT;
	} else if (pa < 0x80000000ULL) {
		base = 0;
		limit = 0x7fffffffULL;
	} else if (pa >= 0x100000000ULL && pa < 0x180000000ULL) {
		base = 0x80000000ULL;
		limit = 0x17fffffffULL;
	} else
		return (EFBIG);
	/* Subtraction checks both full extent and overflow before conversion. */
	if (pa > limit || len - 1 > limit - pa)
		return (EFBIG);
	*da = pa - base;
	return (0);
}

static int
smte_dma_window_valid(const pcell_t *cells, size_t bytes)
{
	static const pcell_t expected[] = {
		0, 0, 0, 0, 0, 0x80000000,
		0, 0x80000000, 1, 0, 0, 0x80000000
	};

	return (bytes == sizeof(expected) &&
	    memcmp(cells, expected, sizeof(expected)) == 0);
}

static void
smte_dma_configure(struct smte_softc *sc)
{
	device_t parent;
	phandle_t node, ancestor;
	pcell_t cells[12], ac;
	char name[64];
	int requested;

	sc->dma_translate = 0;
	sc->dma_lowaddr = BUS_SPACE_MAXADDR_32BIT;
	requested = 0;
	snprintf(name, sizeof(name), "hw.smte.%d.dma_translate",
	    device_get_unit(sc->dev));
	TUNABLE_INT_FETCH(name, &requested);
	if (requested == 0)
		return;
	parent = device_get_parent(sc->dev);
	node = ofw_bus_get_node(parent);
	/* NULL parent tag selects the identity RISC-V bounce backend. */
	if (requested != 1 || bus_get_dma_tag(sc->dev) != NULL ||
	    node <= 0 || !ofw_bus_is_compatible(parent, "simple-bus") ||
	    ofw_bus_get_name(parent) == NULL ||
	    strcmp(ofw_bus_get_name(parent), "network-bus") != 0 ||
	    OF_getproplen(ofw_bus_get_node(sc->dev), "iommus") > 0 ||
	    OF_getproplen(ofw_bus_get_node(sc->dev), "dma-ranges") > 0 ||
	    OF_getproplen(node, "ranges") != 0 ||
	    OF_getencprop(node, "#address-cells", &ac, sizeof(ac)) != sizeof(ac) ||
	    ac != 2 ||
	    OF_getencprop(node, "#size-cells", &ac, sizeof(ac)) != sizeof(ac) ||
	    ac != 2 ||
	    OF_getencprop(OF_parent(node), "#address-cells", &ac, sizeof(ac)) != sizeof(ac) ||
	    ac != 2 || OF_getproplen(node, "dma-ranges") != sizeof(cells) ||
	    OF_getencprop(node, "dma-ranges", cells, sizeof(cells)) != sizeof(cells) ||
	    !smte_dma_window_valid(cells, sizeof(cells)))
		goto unsupported;
	for (ancestor = OF_parent(node); ancestor > 0;
	    ancestor = OF_parent(ancestor)) {
		if (OF_getproplen(ancestor, "dma-ranges") > 0)
			goto unsupported;
	}
	sc->dma_translate = 1;
	sc->dma_lowaddr = 0x17fffffffULL;
	device_printf(sc->dev, "using K1 network DMA address windows\n");
	return;
unsupported:
	device_printf(sc->dev, "DMA translation request rejected; retaining 32-bit bounce mapping\n");
}

/*
 * MII (MDIO) access.
 */
static int
smte_miibus_readreg(device_t dev, int phy, int reg)
{
	struct smte_softc *sc;
	int timo;

	sc = device_get_softc(dev);

	WR4(sc, MAC_MDIO_DATA, 0);
	WR4(sc, MAC_MDIO_CTRL, MAC_MDIO_CTRL_START_MDIO_TRANS |
	    MAC_MDIO_CTRL_MDIO_READ_WRITE |
	    reg << MAC_MDIO_CTRL_REGISTER_ADDRESS_SHIFT |
	    phy << MAC_MDIO_CTRL_PHY_ADDRESS_SHIFT);

	for (timo = 100; timo > 0; timo--) {
		if ((RD4(sc, MAC_MDIO_CTRL) &
		    MAC_MDIO_CTRL_START_MDIO_TRANS) == 0)
			return (RD4(sc, MAC_MDIO_DATA) & 0xffff);
		DELAY(100);
	}

	device_printf(dev, "MDIO read timeout\n");
	return (0);
}

static int
smte_miibus_writereg(device_t dev, int phy, int reg, int val)
{
	struct smte_softc *sc;
	int timo;

	sc = device_get_softc(dev);

	WR4(sc, MAC_MDIO_DATA, val & 0xffff);
	WR4(sc, MAC_MDIO_CTRL, MAC_MDIO_CTRL_START_MDIO_TRANS |
	    reg << MAC_MDIO_CTRL_REGISTER_ADDRESS_SHIFT |
	    phy << MAC_MDIO_CTRL_PHY_ADDRESS_SHIFT);

	for (timo = 100; timo > 0; timo--) {
		if ((RD4(sc, MAC_MDIO_CTRL) &
		    MAC_MDIO_CTRL_START_MDIO_TRANS) == 0)
			return (0);
		DELAY(100);
	}

	device_printf(dev, "MDIO write timeout\n");
	return (0);
}

static void
smte_miibus_statchg(device_t dev)
{
	struct smte_softc *sc;
	struct mii_data *mii;
	uint32_t ctrl;

	sc = device_get_softc(dev);
	mii = sc->mii;

	if ((mii->mii_media_status & (IFM_ACTIVE | IFM_AVALID)) !=
	    (IFM_ACTIVE | IFM_AVALID)) {
		sc->link = 0;
		return;
	}

	ctrl = RD4(sc, MAC_GLOBAL_CTRL);
	ctrl &= ~MAC_GLOBAL_CTRL_SPEED_MASK;

	switch (IFM_SUBTYPE(mii->mii_media_active)) {
	case IFM_1000_T:
	case IFM_1000_SX:
		ctrl |= MAC_GLOBAL_CTRL_SPEED_1000;
		sc->link = 1;
		break;
	case IFM_100_TX:
		ctrl |= MAC_GLOBAL_CTRL_SPEED_100;
		sc->link = 1;
		break;
	case IFM_10_T:
		ctrl |= MAC_GLOBAL_CTRL_SPEED_10;
		sc->link = 1;
		break;
	default:
		sc->link = 0;
		return;
	}

	if ((mii->mii_media_active & IFM_GMASK) == IFM_FDX)
		ctrl |= MAC_GLOBAL_CTRL_DUPLEX_MODE;
	else
		ctrl &= ~MAC_GLOBAL_CTRL_DUPLEX_MODE;

	WR4(sc, MAC_GLOBAL_CTRL, ctrl);
}

/*
 * Media.
 */
static int
smte_media_change(if_t ifp)
{
	struct smte_softc *sc;
	int error;

	sc = if_getsoftc(ifp);
	SMTE_LOCK(sc);
	error = mii_mediachg(sc->mii);
	SMTE_UNLOCK(sc);
	return (error);
}

static void
smte_media_status(if_t ifp, struct ifmediareq *ifmr)
{
	struct smte_softc *sc;

	sc = if_getsoftc(ifp);
	SMTE_LOCK(sc);
	mii_pollstat(sc->mii);
	ifmr->ifm_active = sc->mii->mii_media_active;
	ifmr->ifm_status = sc->mii->mii_media_status;
	SMTE_UNLOCK(sc);
}

/*
 * MAC address filter.
 */
static void
smte_lladdr_write(struct smte_softc *sc)
{
	const uint8_t *ea;

	ea = if_getlladdr(sc->ifp);
	WR4(sc, MAC_ADDR1_HI, ea[1] << 8 | ea[0]);
	WR4(sc, MAC_ADDR1_ME, ea[3] << 8 | ea[2]);
	WR4(sc, MAC_ADDR1_LO, ea[5] << 8 | ea[4]);
}

static u_int
smte_hash_maddr(void *arg, struct sockaddr_dl *sdl, u_int cnt)
{
	uint16_t *hash = arg;
	uint32_t crc;

	crc = ether_crc32_be(LLADDR(sdl), ETHER_ADDR_LEN) >> 26;
	hash[crc >> 4] |= (1 << (crc & 0xf));
	return (1);
}

static void
smte_setup_rxfilter(struct smte_softc *sc)
{
	uint16_t hash[4];
	uint32_t val;

	SMTE_ASSERT_LOCKED(sc);

	smte_lladdr_write(sc);

	val = MAC_ADDR_CTRL_MAC_ADDR1_ENABLE;
	memset(hash, 0, sizeof(hash));

	if ((if_getflags(sc->ifp) & IFF_PROMISC) != 0)
		val |= MAC_ADDR_CTRL_PROMISCUOUS_MODE;
	else if ((if_getflags(sc->ifp) & IFF_ALLMULTI) != 0)
		memset(hash, 0xff, sizeof(hash));
	else
		if_foreach_llmaddr(sc->ifp, smte_hash_maddr, hash);

	WR4(sc, MAC_MULTICAST_HASH_TABLE1, hash[0]);
	WR4(sc, MAC_MULTICAST_HASH_TABLE2, hash[1]);
	WR4(sc, MAC_MULTICAST_HASH_TABLE3, hash[2]);
	WR4(sc, MAC_MULTICAST_HASH_TABLE4, hash[3]);
	WR4(sc, MAC_ADDR_CTRL, val);
}

/*
 * Receive ring.
 */
/* Smallest cluster that holds a full frame for this MTU, plus the pad. */
static int
smte_rx_bufsize(int mtu)
{

	if (mtu + ETHER_HDR_LEN + ETHER_VLAN_ENCAP_LEN + ETHER_CRC_LEN +
	    ETHER_ALIGN <= MCLBYTES)
		return (MCLBYTES);
	return (MJUMPAGESIZE);
}

static int
smte_rx_alloc(struct smte_softc *sc, struct smte_bufmap *buf, int size)
{
	struct mbuf *m;
	bus_dma_segment_t seg;
	int error, nsegs;

	m = m_getjcl(M_NOWAIT, MT_DATA, M_PKTHDR, size);
	if (m == NULL)
		return (ENOBUFS);
	m->m_len = m->m_pkthdr.len = size;
	m_adj(m, ETHER_ALIGN);

	error = bus_dmamap_load_mbuf_sg(sc->rxbuf_tag, buf->map,
	    m, &seg, &nsegs, BUS_DMA_NOWAIT);
	if (error != 0) {
		m_freem(m);
		return (error);
	}

	if (nsegs != 1)
		error = EFBIG;
	else
		error = smte_dma_addr(sc, seg.ds_addr, seg.ds_len, &buf->paddr);
	if (error != 0) {
		bus_dmamap_unload(sc->rxbuf_tag, buf->map);
		m_freem(m);
		return (error);
	}
	buf->mbuf = m;
	return (0);
}

static void
smte_rx_rearm(struct smte_softc *sc, int idx)
{

	bus_dmamap_sync(sc->rxbuf_tag, sc->rxbuf[idx].map,
	    BUS_DMASYNC_PREREAD);
	sc->rxdesc[idx].sd_addr1 = (uint32_t)sc->rxbuf[idx].paddr;
	sc->rxdesc[idx].sd_desc1 =
	    (sc->rx_bufsize - ETHER_ALIGN) & RX_DESC1_SIZE1_MASK;
	if (idx == SMTE_NRXDESC - 1)
		sc->rxdesc[idx].sd_desc1 |= RX_DESC1_END_RING;
	bus_dmamap_sync(sc->desc_tag, sc->rxdesc_map,
	    BUS_DMASYNC_PREWRITE);
	sc->rxdesc[idx].sd_desc0 = RX_DESC0_OWN;
	bus_dmamap_sync(sc->desc_tag, sc->rxdesc_map,
	    BUS_DMASYNC_PREWRITE);
}

/* Keep the old mapping intact until its replacement is ready. */
static int
smte_newbuf(struct smte_softc *sc, int idx)
{
	struct smte_bufmap buf;
	int error;

	buf.map = sc->rx_sparemap;
	error = smte_rx_alloc(sc, &buf, sc->rx_bufsize);
	if (error != 0)
		return (error);
	if (sc->rxbuf[idx].mbuf != NULL)
		bus_dmamap_unload(sc->rxbuf_tag, sc->rxbuf[idx].map);
	sc->rx_sparemap = sc->rxbuf[idx].map;
	sc->rxbuf[idx] = buf;
	smte_rx_rearm(sc, idx);

	return (0);
}

#ifdef INET
/*
 * The MAC has no supported transport checksum offload. Verify the original
 * single-buffer IPv4/TCP packet in software before giving LRO trusted metadata.
 * All other traffic follows the original stack path; no header bytes change.
 */
static int
smte_sw_lro_verified(struct mbuf *m)
{
	const struct ether_header *eh;
	const struct ip *ip;
	const struct tcphdr *th;
	int iplen, tcplen, thlen;
	uint16_t sum;

	if (m->m_next != NULL || m->m_len != m->m_pkthdr.len ||
	    m->m_len < ETHER_HDR_LEN + sizeof(*ip) + sizeof(*th))
		return (0);
	eh = mtod(m, const struct ether_header *);
	if (eh->ether_type != htons(ETHERTYPE_IP) ||
	    (eh->ether_dhost[0] & 1) != 0)
		return (0);
	ip = (const struct ip *)(m->m_data + ETHER_HDR_LEN);
	if (ip->ip_v != 4 || ip->ip_hl != 5 || ip->ip_p != IPPROTO_TCP ||
	    (ip->ip_off & htons(IP_MF | IP_OFFMASK)) != 0)
		return (0);
	iplen = ntohs(ip->ip_len);
	if (iplen < sizeof(*ip) + sizeof(*th) ||
	    iplen > m->m_len - ETHER_HDR_LEN)
		return (0);
	tcplen = iplen - sizeof(*ip);
	th = (const struct tcphdr *)(ip + 1);
	thlen = th->th_off << 2;
	if (thlen < sizeof(*th) || thlen > tcplen || in_cksum_hdr(ip) != 0)
		return (0);
	sum = in_cksum_skip(m, ETHER_HDR_LEN + iplen,
	    ETHER_HDR_LEN + sizeof(*ip));
	sum = in_addword((uint16_t)~sum, in_pseudo(ip->ip_src.s_addr,
	    ip->ip_dst.s_addr, htonl(tcplen + IPPROTO_TCP)));
	if (sum != 0xffff)
		return (0);
	m->m_pkthdr.csum_flags |= CSUM_IP_CHECKED | CSUM_IP_VALID |
	    CSUM_DATA_VALID | CSUM_PSEUDO_HDR;
	m->m_pkthdr.csum_data = 0xffff;
	return (1);
}

static void
smte_sw_lro_deliver(struct smte_softc *sc, struct mbufq *mq, int use_lro)
{
	struct mbuf *m;

	/* Called only by the serialized interrupt consumer, without sc->mtx. */
	while ((m = mbufq_dequeue(mq)) != NULL) {
		if (use_lro) {
			if (smte_sw_lro_verified(m) &&
			    tcp_lro_rx(&sc->sw_lro_ctrl, m, 0) == 0)
				continue;
			/* Keep rejected/unsupported packets in stream order. */
			tcp_lro_flush_all(&sc->sw_lro_ctrl);
		}
		if_input(sc->ifp, m);
	}
	if (use_lro)
		tcp_lro_flush_all(&sc->sw_lro_ctrl);
}

static int
smte_sw_lro_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct smte_softc *sc;
	int error, value;

	sc = arg1;
	SMTE_LOCK(sc);
	value = sc->sw_lro;
	SMTE_UNLOCK(sc);
	error = sysctl_handle_int(oidp, &value, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (value != 0 && value != 1)
		return (EINVAL);
	SMTE_LOCK(sc);
	if (value != 0 && !sc->sw_lro_ready)
		error = ENXIO;
	else
		sc->sw_lro = value;
	SMTE_UNLOCK(sc);
	return (error);
}
#endif

static void
smte_rxeof(struct smte_softc *sc)
{
	struct mbufq mq;
	struct mbuf *m;
	uint32_t desc0;
	int budget, idx, len;
#ifdef INET
	int use_lro;
#endif
	sbintime_t phase_start;

	SMTE_ASSERT_LOCKED(sc);

	sc->service[SMTE_STAT_RX_CALLS]++;
	phase_start = SMTE_SERVICE_NOW();
	mbufq_init(&mq, SMTE_NRXDESC);

	/*
	 * Hardware can refill descriptors while we drain them.  Limit a pass
	 * to the local queue capacity so wrapping the ring cannot overrun mq.
	 * Completions after the interrupt acknowledgement remain pending for
	 * the next pass (or the RX mitigation timer).
	 */
	for (budget = SMTE_NRXDESC; budget > 0; budget--) {
		idx = sc->rx_cons;

		bus_dmamap_sync(sc->desc_tag, sc->rxdesc_map,
		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
		desc0 = sc->rxdesc[idx].sd_desc0;
		if ((desc0 & RX_DESC0_OWN) != 0)
			break;

		sc->service[SMTE_STAT_RX_DESCRIPTORS]++;
		len = (desc0 & RX_DESC0_FRAME_PACKET_LENGTH_MASK) >>
		    RX_DESC0_FRAME_PACKET_LENGTH_SHIFT;

		bus_dmamap_sync(sc->rxbuf_tag, sc->rxbuf[idx].map,
		    BUS_DMASYNC_POSTREAD);
		m = sc->rxbuf[idx].mbuf;

		if (len < ETHER_CRC_LEN || len > sc->rx_bufsize - ETHER_ALIGN ||
		    (desc0 & (RX_DESC0_FRAME_RUNT | RX_DESC0_FRAME_CRC_ERR |
		    RX_DESC0_FRAME_MAX_LEN_ERR | RX_DESC0_FRAME_JABBER_ERR |
		    RX_DESC0_FRAME_LENGTH_ERR)) != 0) {
			if_inc_counter(sc->ifp, IFCOUNTER_IERRORS, 1);
			smte_rx_rearm(sc, idx);
		} else if (smte_newbuf(sc, idx) != 0) {
			/* Drop the packet, but retain a usable DMA buffer. */
			if_inc_counter(sc->ifp, IFCOUNTER_IQDROPS, 1);
			smte_rx_rearm(sc, idx);
		} else {
			len -= ETHER_CRC_LEN;
			m->m_pkthdr.len = m->m_len = len;
			m->m_pkthdr.rcvif = sc->ifp;

			if_inc_counter(sc->ifp, IFCOUNTER_IPACKETS, 1);
			if (mbufq_enqueue(&mq, m) != 0) {
				if_inc_counter(sc->ifp, IFCOUNTER_IQDROPS, 1);
				m_freem(m);
			}
		}

		sc->rx_cons = (idx == SMTE_NRXDESC - 1) ? 0 : idx + 1;
	}

	if (budget == 0)
		sc->service[SMTE_STAT_RX_BUDGET_EXHAUSTED]++;
	if ((uint64_t)mbufq_len(&mq) > sc->service[SMTE_STAT_RX_BATCH_MAX])
		sc->service[SMTE_STAT_RX_BATCH_MAX] = mbufq_len(&mq);
	smte_service_time(sc, SMTE_STAT_RX_DRAIN_NS,
	    SMTE_STAT_RX_DRAIN_MAX_NS, phase_start);
	smte_service_missed(sc, SMTE_STAT_DMA_MISSED_DRAIN_RAW);

	/* Restart the receive engine in case it stopped on a full ring. */
	WR4(sc, DMA_RECEIVE_POLL_DEMAND, 1);

	/* Hand the batch to the stack without holding our lock. */
	if (mbufq_len(&mq) > 0) {
		phase_start = SMTE_SERVICE_NOW();
#ifdef INET
		use_lro = sc->sw_lro && sc->sw_lro_ready;
#endif
		SMTE_UNLOCK(sc);
#ifdef INET
		smte_sw_lro_deliver(sc, &mq, use_lro);
#else
		while ((m = mbufq_dequeue(&mq)) != NULL)
			if_input(sc->ifp, m);
#endif
		SMTE_LOCK(sc);
		smte_service_time(sc, SMTE_STAT_RX_HANDOFF_NS,
		    SMTE_STAT_RX_HANDOFF_MAX_NS, phase_start);
		smte_service_missed(sc, SMTE_STAT_DMA_MISSED_HANDOFF_RAW);
	}
}

/*
 * Transmit.
 */
static int
smte_encap(struct smte_softc *sc, struct mbuf **mp)
{
	bus_dma_segment_t segs[SMTE_NTXSEGS];
	struct smte_desc *txd;
	struct mbuf *m;
	int error, first, i, idx, nsegs, ndesc, stride;

	SMTE_ASSERT_LOCKED(sc);

	m = *mp;
	first = idx = sc->tx_prod;

	error = bus_dmamap_load_mbuf_sg(sc->txbuf_tag, sc->txbuf[first].map,
	    m, segs, &nsegs, BUS_DMA_NOWAIT);
	if (error == EFBIG) {
		m = m_collapse(m, M_NOWAIT, SMTE_NTXSEGS);
		if (m == NULL) {
			m_freem(*mp);
			*mp = NULL;
			return (ENOMEM);
		}
		*mp = m;
		error = bus_dmamap_load_mbuf_sg(sc->txbuf_tag,
		    sc->txbuf[first].map, m, segs, &nsegs, BUS_DMA_NOWAIT);
	}
	if (error != 0) {
		m_freem(*mp);
		*mp = NULL;
		return (error);
	}

	if (nsegs < 1 || nsegs > SMTE_NTXSEGS) {
		bus_dmamap_unload(sc->txbuf_tag, sc->txbuf[first].map);
		m_freem(m);
		*mp = NULL;
		return (EFBIG);
	}
	stride = sc->tx_pack ? 2 : 1;
	ndesc = howmany(nsegs, stride);
	if (sc->tx_used + ndesc + 1 > SMTE_NTXDESC) {
		bus_dmamap_unload(sc->txbuf_tag, sc->txbuf[first].map);
		return (ENOBUFS);
	}

	/* Validate every device address before modifying or publishing the ring. */
	for (i = 0; i < nsegs; i++) {
		if (segs[i].ds_len == 0 || segs[i].ds_len > TX_DESC1_SIZE1_MASK ||
		    smte_dma_addr(sc, segs[i].ds_addr, segs[i].ds_len,
		    &segs[i].ds_addr) != 0) {
			bus_dmamap_unload(sc->txbuf_tag, sc->txbuf[first].map);
			m_freem(m);
			*mp = NULL;
			return (EFBIG);
		}
	}
	bus_dmamap_sync(sc->txbuf_tag, sc->txbuf[first].map,
	    BUS_DMASYNC_PREWRITE);

	for (i = 0; i < nsegs; i += stride) {
		txd = &sc->txdesc[idx];
		txd->sd_desc0 = 0;
		txd->sd_addr1 = (uint32_t)segs[i].ds_addr;
		txd->sd_addr2 = 0;
		txd->sd_desc1 = segs[i].ds_len & TX_DESC1_SIZE1_MASK;
		if (stride == 2 && i + 1 < nsegs) {
			txd->sd_addr2 = (uint32_t)segs[i + 1].ds_addr;
			txd->sd_desc1 |= segs[i + 1].ds_len << 12;
		}
		if (idx == SMTE_NTXDESC - 1)
			txd->sd_desc1 |= TX_DESC1_END_RING;
		if (i == 0)
			txd->sd_desc1 |= TX_DESC1_FIRST_SEGMENT;
		/*
		 * Interrupt on the last segment of every frame: without the
		 * bit the DMA never raises TX_TRANSFER_DONE, and completed
		 * frames are only reclaimed when an RX interrupt or the 1 Hz
		 * tick happens to run smte_txeof().
		 */
		if (i + stride >= nsegs)
			txd->sd_desc1 |= TX_DESC1_LAST_SEGMENT |
			    TX_DESC1_INTERRUPT_ON_COMPLETION;
		if (i != 0)
			txd->sd_desc0 = TX_DESC0_OWN;

		idx = (idx == SMTE_NTXDESC - 1) ? 0 : idx + 1;
	}

	/*
	 * The mbuf and the loaded map ride with the last descriptor;
	 * the first slot's spare map is swapped into the last slot.
	 */
	i = (idx == 0) ? SMTE_NTXDESC - 1 : idx - 1;
	if (i != first) {
		bus_dmamap_t tmp;

		tmp = sc->txbuf[i].map;
		sc->txbuf[i].map = sc->txbuf[first].map;
		sc->txbuf[first].map = tmp;
	}
	sc->txbuf[i].mbuf = m;

	/* Publish everything, then hand the chain to hardware. */
	bus_dmamap_sync(sc->desc_tag, sc->txdesc_map, BUS_DMASYNC_PREWRITE);
	sc->txdesc[first].sd_desc0 = TX_DESC0_OWN;
	bus_dmamap_sync(sc->desc_tag, sc->txdesc_map, BUS_DMASYNC_PREWRITE);

	sc->tx_prod = idx;
	sc->tx_used += ndesc;

	return (0);
}

/*
 * Return both rings to their post-attach state after stopping the DMA
 * engines: smte_init_locked() reprograms DMA_{TRANSMIT,RECEIVE}_BASE_
 * ADDRESS, which parks the hardware back on descriptor 0, so the software
 * producer/consumer indices have to be rewound to match or the two sides
 * address different descriptors from then on.
 */
static void
smte_reset_rings(struct smte_softc *sc)
{
	int i;

	SMTE_ASSERT_LOCKED(sc);

	for (i = 0; i < SMTE_NTXDESC; i++) {
		if (sc->txbuf[i].mbuf != NULL) {
			bus_dmamap_sync(sc->txbuf_tag, sc->txbuf[i].map,
			    BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(sc->txbuf_tag, sc->txbuf[i].map);
			m_freem(sc->txbuf[i].mbuf);
			sc->txbuf[i].mbuf = NULL;
		}
		sc->txdesc[i].sd_desc0 = 0;
		sc->txdesc[i].sd_desc1 = 0;
	}
	sc->tx_prod = sc->tx_cons = sc->tx_used = 0;
	sc->tx_watchdog = 0;

	/* Discard pending receives and return the mapped buffers to DMA. */
	for (i = 0; i < SMTE_NRXDESC; i++) {
		bus_dmamap_sync(sc->rxbuf_tag, sc->rxbuf[i].map,
		    BUS_DMASYNC_POSTREAD);
		smte_rx_rearm(sc, i);
	}
	sc->rx_cons = 0;

	bus_dmamap_sync(sc->desc_tag, sc->txdesc_map, BUS_DMASYNC_PREWRITE);
	bus_dmamap_sync(sc->desc_tag, sc->rxdesc_map, BUS_DMASYNC_PREWRITE);
}

static void
smte_txeof(struct smte_softc *sc)
{
	struct smte_desc *txd;
	int idx, freed;
	sbintime_t start;

	SMTE_ASSERT_LOCKED(sc);
	start = SMTE_SERVICE_NOW();

	bus_dmamap_sync(sc->desc_tag, sc->txdesc_map,
	    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
	if ((uint64_t)sc->tx_used > sc->service[SMTE_STAT_TX_USED_MAX])
		sc->service[SMTE_STAT_TX_USED_MAX] = sc->tx_used;
	freed = 0;
	while (sc->tx_cons != sc->tx_prod) {
		idx = sc->tx_cons;
		txd = &sc->txdesc[idx];
		if ((txd->sd_desc0 & TX_DESC0_OWN) != 0)
			break;

		if (sc->txbuf[idx].mbuf != NULL) {
			bus_dmamap_sync(sc->txbuf_tag, sc->txbuf[idx].map,
			    BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(sc->txbuf_tag, sc->txbuf[idx].map);
			m_freem(sc->txbuf[idx].mbuf);
			sc->txbuf[idx].mbuf = NULL;
			if_inc_counter(sc->ifp, IFCOUNTER_OPACKETS, 1);
		}

		txd->sd_desc0 = 0;
		txd->sd_desc1 = 0;
		freed++;
		sc->tx_cons = (idx == SMTE_NTXDESC - 1) ? 0 : idx + 1;
	}

	if (freed > 0) {
		sc->tx_used -= freed;
		if_setdrvflagbits(sc->ifp, 0, IFF_DRV_OACTIVE);
	}

	if (sc->tx_used == 0)
		sc->tx_watchdog = 0;
	sc->service[SMTE_STAT_TX_RECLAIMED] += freed;
	smte_service_time(sc, SMTE_STAT_TX_RECLAIM_NS,
	    SMTE_STAT_TX_RECLAIM_MAX_NS, start);
}

static void
smte_start_locked(if_t ifp)
{
	struct smte_softc *sc;
	struct mbuf *m;
	int queued;

	sc = if_getsoftc(ifp);
	SMTE_ASSERT_LOCKED(sc);

	if ((if_getdrvflags(ifp) & (IFF_DRV_RUNNING | IFF_DRV_OACTIVE)) !=
	    IFF_DRV_RUNNING || sc->link == 0)
		return;

	queued = 0;
	for (;;) {
		if (sc->tx_used + howmany(SMTE_NTXSEGS, sc->tx_pack ? 2 : 1) +
		    1 > SMTE_NTXDESC) {
			sc->service[SMTE_STAT_TX_RING_BLOCKED]++;
			if_setdrvflagbits(ifp, IFF_DRV_OACTIVE, 0);
			break;
		}

		m = if_dequeue(ifp);
		if (m == NULL)
			break;

		if (smte_encap(sc, &m) != 0) {
			if (m == NULL)
				if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
			else {
				if_sendq_prepend(ifp, m);
				if_setdrvflagbits(ifp, IFF_DRV_OACTIVE, 0);
			}
			break;
		}
		queued++;
		bpf_mtap_if(ifp, m);
	}

	if (queued > 0) {
		sc->tx_watchdog = SMTE_WATCHDOG_TIMEOUT;
		WR4(sc, DMA_TRANSMIT_POLL_DEMAND, 0xff);
	}
}

static void
smte_start(if_t ifp)
{
	struct smte_softc *sc;

	sc = if_getsoftc(ifp);
	SMTE_LOCK(sc);
	smte_start_locked(ifp);
	SMTE_UNLOCK(sc);
}

/*
 * Interrupt handler.
 */
static void
smte_intr(void *arg)
{
	struct smte_softc *sc;
	uint32_t stat;
	sbintime_t start;

	sc = arg;
	start = SMTE_SERVICE_NOW();
	SMTE_LOCK(sc);
	smte_service_time(sc, SMTE_STAT_INTR_LOCK_WAIT_NS,
	    SMTE_STAT_INTR_LOCK_WAIT_MAX_NS, start);

	stat = RD4(sc, DMA_STATUS_IRQ);

	/*
	 * Ack (write-1-to-clear) the events we handle BEFORE draining the
	 * rings.  smte_rxeof() drops the lock to call if_input(), and any
	 * completion that arrives in that window then re-asserts its bit
	 * after this ack and retriggers the interrupt, rather than being
	 * cleared unprocessed (the old post-drain ack lost those, stranding
	 * RX until the mitigation timeout fired).  Mask to handled bits only:
	 * DMA_STATUS_IRQ has sticky state bits that are not write-1-to-clear.
	 */
	stat &= (DMA_STATUS_IRQ_RX_TRANSFER_DONE |
	    DMA_STATUS_IRQ_RX_MISSED_FRAME | DMA_STATUS_IRQ_TX_TRANSFER_DONE |
	    DMA_STATUS_IRQ_TX_DES_UNAVAILABLE | DMA_STATUS_IRQ_TX_DMA_STOPPED);
	if (stat == 0) {
		SMTE_UNLOCK(sc);
		return;
	}
	WR4(sc, DMA_STATUS_IRQ, stat);
	sc->service[SMTE_STAT_INTR_CALLS]++;
	smte_service_missed(sc, SMTE_STAT_DMA_MISSED_OTHER_RAW);

	/* Reclaim only: do not delay RX with an extra TX enqueue pass. */
	if (sc->tx_reclaim_first && sc->tx_used != 0 &&
	    (stat & (DMA_STATUS_IRQ_RX_TRANSFER_DONE |
	    DMA_STATUS_IRQ_RX_MISSED_FRAME)) != 0) {
		sc->service[SMTE_STAT_TX_EARLY_RECLAIMS]++;
		smte_txeof(sc);
	}

	if ((stat & (DMA_STATUS_IRQ_RX_TRANSFER_DONE |
	    DMA_STATUS_IRQ_RX_MISSED_FRAME)) != 0)
		smte_rxeof(sc);

	/*
	 * TX_DES_UNAVAILABLE is raised when the transmit DMA walks onto a
	 * descriptor the driver still owns; the engine then suspends and
	 * stays suspended until the condition is acked above.  Leaving it
	 * latched is what killed transmit outright: a handful of frames go
	 * out, the ring drains, the engine suspends, and every subsequent
	 * DMA_TRANSMIT_POLL_DEMAND is ignored.  Reclaim, then re-arm -- if
	 * there is nothing new to send but descriptors are still owned by
	 * hardware, poll demand alone is what restarts it.
	 */
	if ((stat & (DMA_STATUS_IRQ_TX_TRANSFER_DONE |
	    DMA_STATUS_IRQ_TX_DES_UNAVAILABLE |
	    DMA_STATUS_IRQ_TX_DMA_STOPPED)) != 0) {
		smte_txeof(sc);
		if (!if_sendq_empty(sc->ifp))
			smte_start_locked(sc->ifp);
		else if (sc->tx_used != 0)
			WR4(sc, DMA_TRANSMIT_POLL_DEMAND, 0xff);
	}

	smte_service_missed(sc, SMTE_STAT_DMA_MISSED_OTHER_RAW);
	smte_service_time(sc, SMTE_STAT_INTR_NS, SMTE_STAT_INTR_MAX_NS, start);
	SMTE_UNLOCK(sc);
}

/*
 * Init / stop.
 */
static void
smte_stop_locked(struct smte_softc *sc)
{

	SMTE_ASSERT_LOCKED(sc);

	callout_stop(&sc->tick_ch);
	sc->tx_watchdog = 0;
	if_setdrvflagbits(sc->ifp, 0, IFF_DRV_RUNNING | IFF_DRV_OACTIVE);

	WR4(sc, MAC_INTR_ENABLE, 0);
	WR4(sc, DMA_INTR_ENABLE, 0);
	WR4(sc, MAC_TRANSMIT_CTRL, 0);
	WR4(sc, MAC_RECEIVE_CTRL, 0);
	WR4(sc, DMA_CTRL, 0);

	/*
	 * init restarts DMA at descriptor zero.  Quiesce the engines before
	 * freeing pending TX buffers, then rewind both software rings for
	 * every stop/init path, including MTU changes and interface down/up.
	 */
	smte_dma_reset(sc);
	smte_reset_rings(sc);
}

static void
smte_init_locked(struct smte_softc *sc)
{
	if_t ifp;

	SMTE_ASSERT_LOCKED(sc);
	ifp = sc->ifp;

	if ((if_getdrvflags(ifp) & IFF_DRV_RUNNING) != 0)
		return;

	smte_setup_rxfilter(sc);

	/* Thresholds and frame sizes. */
	WR4(sc, MAC_TRANSMIT_FIFO_ALMOST_FULL, 0x1f8);
	WR4(sc, MAC_TRANSMIT_PACKET_START_THRESHOLD, 1518);
	WR4(sc, MAC_RECEIVE_PACKET_START_THRESHOLD, 12);
	WR4(sc, MAC_MAXIMUM_FRAME_SIZE, if_getmtu(ifp) + ETHER_HDR_LEN +
	    ETHER_VLAN_ENCAP_LEN + ETHER_CRC_LEN);
	WR4(sc, MAC_TRANSMIT_JABBER_SIZE, ETHER_MAX_LEN_JUMBO);
	WR4(sc, MAC_RECEIVE_JABBER_SIZE, ETHER_MAX_LEN_JUMBO);

	/*
	 * Receive interrupt mitigation (proven-good values).  RX throughput
	 * tuning is deferred until remote reboot works and iteration is free;
	 * writing 0 here wedges the controller, and the ack-ordering rework
	 * needs on-hardware validation.
	 */
	WR4(sc, DMA_RECEIVE_IRQ_MITIGATION,
	    (64 << DMA_RECEIVE_IRQ_MITIGATION_FRAME_COUNTER_SHIFT) |
	    ((600 * 312) << DMA_RECEIVE_IRQ_MITIGATION_TIMEOUT_COUNTER_SHIFT) |
	    DMA_RECEIVE_IRQ_MITIGATION_MITIGATION_ENABLE);

	/* Ring base addresses. */
	WR4(sc, DMA_TRANSMIT_BASE_ADDRESS, (uint32_t)sc->txdesc_paddr);
	WR4(sc, DMA_RECEIVE_BASE_ADDRESS, (uint32_t)sc->rxdesc_paddr);

	/*
	 * Enable only events acknowledged by smte_intr().  RX stopped and
	 * descriptor-unavailable events are not handled there; enabling them
	 * leaves a level interrupt asserted indefinitely after RX starvation.
	 * smte_rxeof() rearms RX with poll demand after draining the ring.
	 */
	WR4(sc, DMA_INTR_ENABLE, DMA_INTR_ENABLE_TX_TRANSFER_DONE |
	    DMA_INTR_ENABLE_TX_DES_UNAVAILABLE |
	    DMA_INTR_ENABLE_TX_DMA_STOPPED |
	    DMA_INTR_ENABLE_RX_TRANSFER_DONE |
	    DMA_INTR_ENABLE_RX_MISSED_FRAME);

	WR4(sc, MAC_TRANSMIT_CTRL,
	    (RD4(sc, MAC_TRANSMIT_CTRL) & ~MAC_TRANSMIT_CTRL_IFG_LEN_MASK) |
	    MAC_TRANSMIT_CTRL_TX_ENABLE | MAC_TRANSMIT_CTRL_TX_AUTO_RETRY);
	WR4(sc, MAC_RECEIVE_CTRL, RD4(sc, MAC_RECEIVE_CTRL) |
	    MAC_RECEIVE_CTRL_RX_ENABLE | MAC_RECEIVE_CTRL_STORE_FORWARD);

	/* Clear any status latched from a previous run before restarting. */
	WR4(sc, DMA_STATUS_IRQ, RD4(sc, DMA_STATUS_IRQ));

	WR4(sc, DMA_TRANSMIT_AUTO_POLL_COUNTER, 0);
	WR4(sc, DMA_CTRL, RD4(sc, DMA_CTRL) | DMA_CTRL_START_STOP_TX_DMA |
	    DMA_CTRL_START_STOP_RX_DMA);

	if_setdrvflagbits(ifp, IFF_DRV_RUNNING, IFF_DRV_OACTIVE);

	mii_mediachg(sc->mii);
	callout_reset(&sc->tick_ch, hz, smte_tick, sc);
}

static void
smte_tick(void *arg)
{
	struct smte_softc *sc;
	int link_was;

	sc = arg;
	SMTE_ASSERT_LOCKED(sc);

	link_was = sc->link;
	mii_tick(sc->mii);

	/*
	 * Reclaim anything hardware finished but did not interrupt for, and
	 * re-issue poll demand while descriptors are still outstanding: the
	 * engine suspends whenever it walks onto a descriptor the driver
	 * still owns, and poll demand is what brings it back.  smte_txeof()
	 * clears tx_watchdog once the ring drains, so a ring that is merely
	 * slow no longer trips the watchdog below.
	 */
	if (sc->tx_used != 0) {
		smte_txeof(sc);
		if (sc->tx_used != 0)
			WR4(sc, DMA_TRANSMIT_POLL_DEMAND, 0xff);
	}

	if (sc->tx_watchdog > 0 && --sc->tx_watchdog == 0) {
		device_printf(sc->dev, "watchdog timeout (DMA_STATUS 0x%08x); "
		    "restarting\n", RD4(sc, DMA_STATUS_IRQ));
		if_inc_counter(sc->ifp, IFCOUNTER_OERRORS, 1);

		/*
		 * smte_init_locked() returns early when IFF_DRV_RUNNING is
		 * still set, so the interface has to be stopped first or the
		 * "restart" is a no-op -- and, because smte_tick() returns
		 * here, the tick callout would never be re-armed either,
		 * silently killing the watchdog after its first firing.
		 */
		smte_txeof(sc);
		smte_stop_locked(sc);
		smte_init_locked(sc);
		if (!if_sendq_empty(sc->ifp))
			smte_start_locked(sc->ifp);
		return;
	}

	if (link_was == 0 && sc->link != 0 && !if_sendq_empty(sc->ifp))
		smte_start_locked(sc->ifp);

	callout_reset(&sc->tick_ch, hz, smte_tick, sc);
}

static void
smte_init(void *arg)
{
	struct smte_softc *sc;

	sc = arg;
	SMTE_LOCK(sc);
	smte_init_locked(sc);
	SMTE_UNLOCK(sc);
}

/*
 * Prepare every replacement buffer before stopping the interface.  A failed
 * allocation leaves the MTU, running state and active RX ring untouched.
 */
static int
smte_change_mtu(struct smte_softc *sc, int mtu)
{
	struct smte_bufmap *bufs;
	bus_dmamap_t map;
	int error, i, running, size;

	SMTE_ASSERT_LOCKED(sc);

	if (mtu < ETHERMIN || mtu > SMTE_MAX_MTU)
		return (EINVAL);
	if (mtu == if_getmtu(sc->ifp))
		return (0);

	size = smte_rx_bufsize(mtu);
	bufs = NULL;
	if (size != sc->rx_bufsize) {
		bufs = malloc(sizeof(*bufs) * SMTE_NRXDESC, M_DEVBUF,
		    M_NOWAIT | M_ZERO);
		if (bufs == NULL)
			return (ENOBUFS);
		for (i = 0; i < SMTE_NRXDESC; i++) {
			error = bus_dmamap_create(sc->rxbuf_tag, BUS_DMA_NOWAIT,
			    &map);
			if (error != 0)
				goto fail;
			bufs[i].map = map;
			error = smte_rx_alloc(sc, &bufs[i], size);
			if (error != 0)
				goto fail;
		}
	}

	/* No fallible operations remain after the old configuration is stopped. */
	running = (if_getdrvflags(sc->ifp) & IFF_DRV_RUNNING) != 0;
	if (running)
		smte_stop_locked(sc);
	if (bufs != NULL) {
		for (i = 0; i < SMTE_NRXDESC; i++) {
			bus_dmamap_sync(sc->rxbuf_tag, sc->rxbuf[i].map,
			    BUS_DMASYNC_POSTREAD);
			bus_dmamap_unload(sc->rxbuf_tag, sc->rxbuf[i].map);
			m_freem(sc->rxbuf[i].mbuf);
			bus_dmamap_destroy(sc->rxbuf_tag, sc->rxbuf[i].map);
			sc->rxbuf[i] = bufs[i];
		}
		free(bufs, M_DEVBUF);
		sc->rx_bufsize = size;
		sc->rx_cons = 0;
		for (i = 0; i < SMTE_NRXDESC; i++)
			smte_rx_rearm(sc, i);
	}
	if_setmtu(sc->ifp, mtu);
	if (running)
		smte_init_locked(sc);

	return (0);

fail:
	for (i = 0; i < SMTE_NRXDESC; i++) {
		if (bufs[i].mbuf != NULL) {
			bus_dmamap_unload(sc->rxbuf_tag, bufs[i].map);
			m_freem(bufs[i].mbuf);
		}
		if (bufs[i].map != NULL)
			bus_dmamap_destroy(sc->rxbuf_tag, bufs[i].map);
	}
	free(bufs, M_DEVBUF);
	return (error);
}

static int
smte_ioctl(if_t ifp, u_long cmd, caddr_t data)
{
	struct smte_softc *sc;
	struct ifreq *ifr;
	int error;

	sc = if_getsoftc(ifp);
	ifr = (struct ifreq *)data;
	error = 0;

	switch (cmd) {
	case SIOCSIFFLAGS:
		SMTE_LOCK(sc);
		if ((if_getflags(ifp) & IFF_UP) != 0) {
			if ((if_getdrvflags(ifp) & IFF_DRV_RUNNING) != 0)
				smte_setup_rxfilter(sc);
			else
				smte_init_locked(sc);
		} else if ((if_getdrvflags(ifp) & IFF_DRV_RUNNING) != 0)
			smte_stop_locked(sc);
		SMTE_UNLOCK(sc);
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		if ((if_getdrvflags(ifp) & IFF_DRV_RUNNING) != 0) {
			SMTE_LOCK(sc);
			smte_setup_rxfilter(sc);
			SMTE_UNLOCK(sc);
		}
		break;
	case SIOCSIFMTU:
		SMTE_LOCK(sc);
		error = smte_change_mtu(sc, ifr->ifr_mtu);
		SMTE_UNLOCK(sc);
		break;
	case SIOCSIFMEDIA:
	case SIOCGIFMEDIA:
		error = ifmedia_ioctl(ifp, ifr, &sc->mii->mii_media, cmd);
		break;
	default:
		error = ether_ioctl(ifp, cmd, data);
		break;
	}

	return (error);
}

/*
 * DMA setup.
 */
struct smte_ring_load {
	struct smte_softc *sc;
	bus_addr_t *device_addr;
	bus_size_t size;
	int error;
};

static void
smte_get1paddr(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{
	struct smte_ring_load *load;

	load = arg;
	load->error = error;
	if (error != 0)
		return;
	if (nsegs != 1 || segs[0].ds_len != load->size) {
		load->error = EFBIG;
		return;
	}
	load->error = smte_dma_addr(load->sc, segs[0].ds_addr,
	    segs[0].ds_len, load->device_addr);
}

static int
smte_setup_dma(struct smte_softc *sc)
{
	struct smte_ring_load load;
	int error, i;

	smte_dma_configure(sc);

	/* 8-byte rings; constrain CPU physical addresses before translating. */
	error = bus_dma_tag_create(bus_get_dma_tag(sc->dev), 8, 0,
	    sc->dma_lowaddr, BUS_SPACE_MAXADDR, NULL, NULL,
	    SMTE_NTXDESC * sizeof(struct smte_desc), 1,
	    SMTE_NTXDESC * sizeof(struct smte_desc), 0, NULL, NULL,
	    &sc->desc_tag);
	if (error != 0)
		return (error);

	error = bus_dmamem_alloc(sc->desc_tag, (void **)&sc->txdesc,
	    BUS_DMA_NOWAIT | BUS_DMA_COHERENT | BUS_DMA_ZERO,
	    &sc->txdesc_map);
	if (error != 0)
		return (error);
	load = (struct smte_ring_load) { sc, &sc->txdesc_paddr,
	    SMTE_NTXDESC * sizeof(struct smte_desc), EINPROGRESS };
	error = bus_dmamap_load(sc->desc_tag, sc->txdesc_map, sc->txdesc,
	    load.size, smte_get1paddr, &load, BUS_DMA_NOWAIT);
	if (error != 0)
		return (error);
	if (load.error != 0) {
		bus_dmamap_unload(sc->desc_tag, sc->txdesc_map);
		return (load.error);
	}

	error = bus_dmamem_alloc(sc->desc_tag, (void **)&sc->rxdesc,
	    BUS_DMA_NOWAIT | BUS_DMA_COHERENT | BUS_DMA_ZERO,
	    &sc->rxdesc_map);
	if (error != 0)
		return (error);
	load = (struct smte_ring_load) { sc, &sc->rxdesc_paddr,
	    SMTE_NRXDESC * sizeof(struct smte_desc), EINPROGRESS };
	error = bus_dmamap_load(sc->desc_tag, sc->rxdesc_map, sc->rxdesc,
	    load.size, smte_get1paddr, &load, BUS_DMA_NOWAIT);
	if (error != 0)
		return (error);
	if (load.error != 0) {
		bus_dmamap_unload(sc->desc_tag, sc->rxdesc_map);
		return (load.error);
	}

	/* Physical reachability; device fields remain 32-bit after translation. */
	error = bus_dma_tag_create(bus_get_dma_tag(sc->dev), 1, 0,
	    sc->dma_lowaddr, BUS_SPACE_MAXADDR, NULL, NULL,
	    MCLBYTES * SMTE_NTXSEGS, SMTE_NTXSEGS, MCLBYTES, 0, NULL, NULL,
	    &sc->txbuf_tag);
	if (error != 0)
		return (error);

	error = bus_dma_tag_create(bus_get_dma_tag(sc->dev), 1, 0,
	    sc->dma_lowaddr, BUS_SPACE_MAXADDR, NULL, NULL,
	    MJUMPAGESIZE, 1, MJUMPAGESIZE, 0, NULL, NULL, &sc->rxbuf_tag);
	if (error != 0)
		return (error);

	sc->rx_bufsize = smte_rx_bufsize(ETHERMTU);
	error = bus_dmamap_create(sc->rxbuf_tag, 0, &sc->rx_sparemap);
	if (error != 0)
		return (error);

	for (i = 0; i < SMTE_NTXDESC; i++) {
		error = bus_dmamap_create(sc->txbuf_tag, 0,
		    &sc->txbuf[i].map);
		if (error != 0)
			return (error);
	}

	for (i = 0; i < SMTE_NRXDESC; i++) {
		error = bus_dmamap_create(sc->rxbuf_tag, 0,
		    &sc->rxbuf[i].map);
		if (error != 0)
			return (error);
		error = smte_newbuf(sc, i);
		if (error != 0)
			return (error);
	}

	return (0);
}

/*
 * One-time MAC/DMA initialization: RGMII delay lines, address filtering,
 * DMA engine reset and configuration.
 */
static void
smte_hw_init(struct smte_softc *sc)
{
	uint32_t rx_delay, tx_delay, val;

	/* Stop everything. */
	WR4(sc, MAC_INTR_ENABLE, 0);
	WR4(sc, DMA_INTR_ENABLE, 0);
	WR4(sc, MAC_TRANSMIT_CTRL, 0);
	WR4(sc, MAC_RECEIVE_CTRL, 0);
	WR4(sc, DMA_CTRL, 0);

	/*
	 * Route the AXI master and put the MAC into RGMII mode with the TX
	 * clock derived from the PHY's RX clock (bit 8 clear).  That is the
	 * configuration the vendor DT ("ref-clock-from-phy") and the Linux
	 * driver use for every RGMII mode.  With bit 8 set ("TX clock from the
	 * SoC") the MAC's transmit side and its statistics block have no
	 * running clock on this board: the TX ring completes ten frames into
	 * the FIFO and stalls for good, nothing reaches the wire, and the stat
	 * counter reads never finish.  REF_CLK_SEL (bit 3) only applies to RMII
	 * and is cleared as well.  The clock gate and reset (bits 0/1) are
	 * handled by clk_enable()/hwreset_deassert() in attach; a bootloader
	 * that never programs this word (our U-Boot) leaves the reset default,
	 * RMII, and the PHY then does not even answer on MDIO, so program it
	 * here to stay bootloader-independent.
	 */
	SYSCON_MODIFY_4(sc->apmu, sc->apmu_offset + APMU_EMAC_CLK_RST_CTRL,
	    APMU_EMAC_RGMII_TXC_SRC_SEL | APMU_EMAC_REF_CLK_SEL,
	    APMU_EMAC_AXI_MST_ID | APMU_EMAC_PHY_SEL_RGMII);

	/* Program RGMII delay lines (ps -> 15.6 ps steps). */
	rx_delay = (sc->rx_delay_ps * 10 + 78) / 156;
	tx_delay = (sc->tx_delay_ps * 10 + 78) / 156;
	val = APMU_EMAC_RGMII_DLINE_RX_EN | APMU_EMAC_RGMII_DLINE_TX_EN;
	val |= APMU_EMAC_RGMII_DLINE_RX_STEP_15P6;
	val |= rx_delay << APMU_EMAC_RGMII_DLINE_RX_DELAY_SHIFT;
	val |= APMU_EMAC_RGMII_DLINE_TX_STEP_15P6;
	val |= tx_delay << APMU_EMAC_RGMII_DLINE_TX_DELAY_SHIFT;
	SYSCON_WRITE_4(sc->apmu, sc->apmu_offset + APMU_EMAC_RGMII_DLINE,
	    val);

	smte_dma_reset(sc);
}

/*
 * Software-reset the DMA engine and reprogram DMA_CONFIG.  Used at attach
 * and by the watchdog: once the transmit engine has wedged, stopping and
 * restarting the MAC is not enough, the engine has to be reset before it
 * will fetch descriptors again.
 *
 * The Linux driver programs the same STRICT_BURST | 64BIT | burst-length
 * word; the vendor U-Boot additionally sets WAIT_FOR_DONE (bit 16), which
 * is left clear here as Linux does.
 */
static void
smte_dma_reset(struct smte_softc *sc)
{

	WR4(sc, DMA_CONFIG, DMA_CONFIG_SOFTWARE_RESET);
	DELAY(10000);
	WR4(sc, DMA_CONFIG, 0);
	DELAY(10000);
	WR4(sc, DMA_CONFIG, DMA_CONFIG_STRICT_BURST |
	    DMA_CONFIG_DMA_64BIT_MODE | DMA_CONFIG_BURST_LENGTH_16);
}

/*
 * Some EMAC instances hold their PHY in reset via a GPIO on the mdio-bus
 * child node (reset-gpios).  Pulse it per the reset-delay-us /
 * reset-post-delay-us timings so MDIO can reach the PHY.  (smte0's PHY
 * happens to work without this; smte1's does not.)
 */
static void
smte_phy_reset(struct smte_softc *sc)
{
	gpio_pin_t reset;
	phandle_t node, mdio;
	uint32_t predelay, postdelay;
	int error;

	node = ofw_bus_get_node(sc->dev);
	mdio = ofw_bus_find_child(node, "mdio-bus");
	if (mdio <= 0)
		return;
	if (!OF_hasprop(mdio, "reset-gpios"))
		return;

	error = gpio_pin_get_by_ofw_property(sc->dev, mdio, "reset-gpios",
	    &reset);
	if (error != 0) {
		if (bootverbose)
			device_printf(sc->dev,
			    "PHY reset-gpios lookup failed (%d)\n", error);
		return;
	}

	predelay = 10000;
	postdelay = 100000;
	OF_getencprop(mdio, "reset-delay-us", &predelay, sizeof(predelay));
	OF_getencprop(mdio, "reset-post-delay-us", &postdelay,
	    sizeof(postdelay));

	gpio_pin_setflags(reset, GPIO_PIN_OUTPUT);
	gpio_pin_set_active(reset, true);	/* assert reset */
	DELAY(predelay);
	gpio_pin_set_active(reset, false);	/* release reset */
	DELAY(postdelay);

	gpio_pin_release(reset);
	if (bootverbose)
		device_printf(sc->dev, "pulsed PHY reset gpio\n");
}

/*
 * Probe / attach.
 */
static int
smte_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "SpacemiT K1 Ethernet MAC");
	return (BUS_PROBE_DEFAULT);
}

static int
smte_attach(device_t dev)
{
	struct smte_softc *sc;
	struct ether_addr eaddr;
	phandle_t node, phy_node, apmu_node;
	pcell_t apmu_prop[2];
	clk_t clk;
	hwreset_t rst;
	uint8_t lladdr[ETHER_ADDR_LEN];
	int error, rid;
	bool valid_mac;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	mtx_init(&sc->mtx, device_get_nameunit(dev), MTX_NETWORK_LOCK,
	    MTX_DEF);
	callout_init_mtx(&sc->tick_ch, &sc->mtx, 0);

	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "cannot allocate registers\n");
		return (ENXIO);
	}
	rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "cannot allocate interrupt\n");
		return (ENXIO);
	}

	/* APMU syscon reference: <phandle offset>. */
	if (OF_getencprop(node, "spacemit,apmu", apmu_prop,
	    sizeof(apmu_prop)) != sizeof(apmu_prop)) {
		device_printf(dev, "cannot get 'spacemit,apmu' property\n");
		return (ENXIO);
	}
	apmu_node = OF_node_from_xref(apmu_prop[0]);
	sc->apmu_offset = apmu_prop[1];
	if (syscon_get_by_ofw_node(dev, apmu_node, &sc->apmu) != 0) {
		device_printf(dev, "cannot get APMU syscon\n");
		return (ENXIO);
	}

	sc->rx_delay_ps = 0;
	sc->tx_delay_ps = 0;
	OF_getencprop(node, "rx-internal-delay-ps", &sc->rx_delay_ps,
	    sizeof(sc->rx_delay_ps));
	OF_getencprop(node, "tx-internal-delay-ps", &sc->tx_delay_ps,
	    sizeof(sc->tx_delay_ps));

	/* PHY location from phy-handle. */
	sc->phyloc = MII_PHY_ANY;
	{
		pcell_t phy_xref = 0;

		OF_getencprop(node, "phy-handle", &phy_xref,
		    sizeof(phy_xref));
		phy_node = OF_node_from_xref(phy_xref);
		if (phy_node > 0)
			OF_getencprop(phy_node, "reg", &sc->phyloc,
			    sizeof(sc->phyloc));
	}

	/* Clock and reset. */
	if (clk_get_by_ofw_index(dev, 0, 0, &clk) == 0) {
		error = clk_enable(clk);
		if (error != 0)
			device_printf(dev, "warning: cannot enable clock\n");
	} else
		device_printf(dev, "warning: cannot get clock\n");
	if (hwreset_get_by_ofw_idx(dev, 0, 0, &rst) == 0)
		hwreset_deassert(rst);

	/* MAC address: devicetree, else generated. */
	valid_mac = false;
	if (OF_getprop(node, "local-mac-address", lladdr,
	    ETHER_ADDR_LEN) == ETHER_ADDR_LEN ||
	    OF_getprop(node, "mac-address", lladdr,
	    ETHER_ADDR_LEN) == ETHER_ADDR_LEN) {
		if ((lladdr[0] | lladdr[1] | lladdr[2] | lladdr[3] |
		    lladdr[4] | lladdr[5]) != 0)
			valid_mac = true;
	}

	sc->ifp = if_alloc(IFT_ETHER);
	if_setsoftc(sc->ifp, sc);
	if_initname(sc->ifp, device_get_name(dev), device_get_unit(dev));

	if (!valid_mac) {
		ether_gen_addr(sc->ifp, &eaddr);
		memcpy(lladdr, eaddr.octet, ETHER_ADDR_LEN);
	}

	smte_phy_reset(sc);
	smte_hw_init(sc);

	error = smte_setup_dma(sc);
	if (error != 0) {
		device_printf(dev, "cannot set up DMA: %d\n", error);
		return (error);
	}

	if_setflags(sc->ifp, IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST);
	if_setstartfn(sc->ifp, smte_start);
	if_setioctlfn(sc->ifp, smte_ioctl);
	if_setinitfn(sc->ifp, smte_init);
	smte_tx_queue_setup(sc);
	sc->tx_pack = smte_boot_flag(sc, "tx_pack");
	if_setsendqready(sc->ifp);
	if_setcapabilities(sc->ifp, IFCAP_VLAN_MTU);
	if_setcapenable(sc->ifp, if_getcapabilities(sc->ifp));

	error = mii_attach(dev, &sc->miibus, sc->ifp, smte_media_change,
	    smte_media_status, BMSR_DEFCAPMASK, sc->phyloc, MII_OFFSET_ANY,
	    0);
	if (error != 0) {
		device_printf(dev, "cannot attach PHY: %d\n", error);
		return (error);
	}
	sc->mii = device_get_softc(sc->miibus);

	error = bus_setup_intr(dev, sc->irq_res, INTR_TYPE_NET |
	    INTR_MPSAFE, NULL, smte_intr, sc, &sc->intrhand);
	if (error != 0) {
		device_printf(dev, "cannot set up interrupt: %d\n", error);
		return (error);
	}

	ether_ifattach(sc->ifp, lladdr);
#ifdef INET
	/* Detach returns EBUSY; state lives for this device's whole boot. */
	error = tcp_lro_init_args(&sc->sw_lro_ctrl, sc->ifp, 16, 0);
	if (error == 0) {
		sc->sw_lro_ready = 1;
		sc->sw_lro = smte_boot_flag(sc, "sw_lro");
	} else
		device_printf(dev, "software LRO unavailable: %d\n", error);
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "sw_lro", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
	    sc, 0, smte_sw_lro_sysctl, "I",
	    "Software-verified IPv4 TCP receive aggregation (default 0)");
#endif
	smte_service_sysctls(sc);
	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "tx_queue_len", CTLFLAG_RD, &sc->tx_queue_len, 0,
	    "Bounded TX packet queue capacity (boot-only)");
	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "dma_translate", CTLFLAG_RD, &sc->dma_translate, 0,
	    "K1 DMA window translation active (boot-only)");
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "tx_pack", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
	    sc, 0, smte_tx_pack_sysctl, "I",
	    "Pack two segments per TX descriptor (default 0)");

	return (0);
}

static int
smte_detach(device_t dev)
{

	return (EBUSY);
}

static device_method_t smte_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		smte_probe),
	DEVMETHOD(device_attach,	smte_attach),
	DEVMETHOD(device_detach,	smte_detach),

	/* MII interface */
	DEVMETHOD(miibus_readreg,	smte_miibus_readreg),
	DEVMETHOD(miibus_writereg,	smte_miibus_writereg),
	DEVMETHOD(miibus_statchg,	smte_miibus_statchg),

	DEVMETHOD_END
};

static driver_t smte_driver = {
	"smte",
	smte_methods,
	sizeof(struct smte_softc),
};

DRIVER_MODULE(smte, simplebus, smte_driver, 0, 0);
DRIVER_MODULE(miibus, smte, miibus_driver, 0, 0);
MODULE_DEPEND(smte, ether, 1, 1, 1);
MODULE_DEPEND(smte, miibus, 1, 1, 1);
MODULE_DEPEND(smte, gpiobus, 1, 1, 1);
