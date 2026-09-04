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

#include <machine/bus.h>

#include <net/bpf.h>
#include <net/if.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_var.h>

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
#define	SMTE_NRXDESC	256

#define	SMTE_LOCK(sc)		mtx_lock(&(sc)->mtx)
#define	SMTE_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)
#define	SMTE_ASSERT_LOCKED(sc)	mtx_assert(&(sc)->mtx, MA_OWNED)

#define	SMTE_WATCHDOG_TIMEOUT	5

struct smte_bufmap {
	bus_dmamap_t	map;
	struct mbuf	*mbuf;
};

struct smte_softc {
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
	struct smte_bufmap rxbuf[SMTE_NRXDESC];
	int		rx_cons;
	int		rx_dbg;
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
static void smte_init_locked(struct smte_softc *sc);
static void smte_start_locked(if_t ifp);
static void smte_tick(void *arg);

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
static int
smte_newbuf(struct smte_softc *sc, int idx)
{
	struct mbuf *m;
	bus_dma_segment_t seg;
	int error, nsegs;

	m = m_getcl(M_NOWAIT, MT_DATA, M_PKTHDR);
	if (m == NULL)
		return (ENOBUFS);
	m->m_len = m->m_pkthdr.len = MCLBYTES;
	m_adj(m, ETHER_ALIGN);

	error = bus_dmamap_load_mbuf_sg(sc->rxbuf_tag, sc->rxbuf[idx].map,
	    m, &seg, &nsegs, BUS_DMA_NOWAIT);
	if (error != 0) {
		m_freem(m);
		return (error);
	}

	bus_dmamap_sync(sc->rxbuf_tag, sc->rxbuf[idx].map,
	    BUS_DMASYNC_PREREAD);

	sc->rxbuf[idx].mbuf = m;
	sc->rxdesc[idx].sd_addr1 = (uint32_t)seg.ds_addr;
	sc->rxdesc[idx].sd_desc1 = seg.ds_len & RX_DESC1_SIZE1_MASK;
	if (idx == SMTE_NRXDESC - 1)
		sc->rxdesc[idx].sd_desc1 |= RX_DESC1_END_RING;
	bus_dmamap_sync(sc->desc_tag, sc->rxdesc_map,
	    BUS_DMASYNC_PREWRITE);
	sc->rxdesc[idx].sd_desc0 = RX_DESC0_OWN;
	bus_dmamap_sync(sc->desc_tag, sc->rxdesc_map,
	    BUS_DMASYNC_PREWRITE);

	return (0);
}

static void
smte_rxeof(struct smte_softc *sc)
{
	struct mbufq mq;
	struct mbuf *m;
	uint32_t desc0;
	int idx, len;

	SMTE_ASSERT_LOCKED(sc);

	mbufq_init(&mq, SMTE_NRXDESC);

	for (;;) {
		idx = sc->rx_cons;

		bus_dmamap_sync(sc->desc_tag, sc->rxdesc_map,
		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
		desc0 = sc->rxdesc[idx].sd_desc0;
		if ((desc0 & RX_DESC0_OWN) != 0)
			break;

		len = (desc0 & RX_DESC0_FRAME_PACKET_LENGTH_MASK) >>
		    RX_DESC0_FRAME_PACKET_LENGTH_SHIFT;

		bus_dmamap_sync(sc->rxbuf_tag, sc->rxbuf[idx].map,
		    BUS_DMASYNC_POSTREAD);
		bus_dmamap_unload(sc->rxbuf_tag, sc->rxbuf[idx].map);

		m = sc->rxbuf[idx].mbuf;
		sc->rxbuf[idx].mbuf = NULL;

		if (len < ETHER_CRC_LEN ||
		    (desc0 & (RX_DESC0_FRAME_RUNT | RX_DESC0_FRAME_CRC_ERR |
		    RX_DESC0_FRAME_MAX_LEN_ERR | RX_DESC0_FRAME_JABBER_ERR |
		    RX_DESC0_FRAME_LENGTH_ERR)) != 0) {
			if_inc_counter(sc->ifp, IFCOUNTER_IERRORS, 1);
			m_freem(m);
		} else {
			len -= ETHER_CRC_LEN;
			m->m_pkthdr.len = m->m_len = len;
			m->m_pkthdr.rcvif = sc->ifp;

			if_inc_counter(sc->ifp, IFCOUNTER_IPACKETS, 1);
			if_inc_counter(sc->ifp, IFCOUNTER_IBYTES, len);
			(void)mbufq_enqueue(&mq, m);
		}

		/* Reload this slot with a fresh buffer. */
		if (smte_newbuf(sc, idx) != 0)
			if_inc_counter(sc->ifp, IFCOUNTER_IQDROPS, 1);

		sc->rx_cons = (idx == SMTE_NRXDESC - 1) ? 0 : idx + 1;
	}

	/* Restart the receive engine in case it stopped on a full ring. */
	WR4(sc, DMA_RECEIVE_POLL_DEMAND, 1);

	/* Hand the batch to the stack without holding our lock. */
	if (mbufq_len(&mq) > 0) {
		SMTE_UNLOCK(sc);
		while ((m = mbufq_dequeue(&mq)) != NULL)
			if_input(sc->ifp, m);
		SMTE_LOCK(sc);
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
	int error, first, i, idx, nsegs;

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

	if (sc->tx_used + nsegs + 1 > SMTE_NTXDESC) {
		bus_dmamap_unload(sc->txbuf_tag, sc->txbuf[first].map);
		return (ENOBUFS);
	}

	bus_dmamap_sync(sc->txbuf_tag, sc->txbuf[first].map,
	    BUS_DMASYNC_PREWRITE);

	for (i = 0; i < nsegs; i++) {
		txd = &sc->txdesc[idx];
		txd->sd_addr1 = (uint32_t)segs[i].ds_addr;
		txd->sd_desc1 = segs[i].ds_len & TX_DESC1_SIZE1_MASK;
		if (idx == SMTE_NTXDESC - 1)
			txd->sd_desc1 |= TX_DESC1_END_RING;
		if (i == 0)
			txd->sd_desc1 |= TX_DESC1_FIRST_SEGMENT;
		if (i == nsegs - 1)
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
	sc->tx_used += nsegs;

	return (0);
}

static void
smte_txeof(struct smte_softc *sc)
{
	struct smte_desc *txd;
	int idx, freed;

	SMTE_ASSERT_LOCKED(sc);

	bus_dmamap_sync(sc->desc_tag, sc->txdesc_map,
	    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

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
		if (sc->tx_used + SMTE_NTXSEGS + 1 > SMTE_NTXDESC) {
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
		WR4(sc, DMA_TRANSMIT_POLL_DEMAND, 1);
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

	sc = arg;
	SMTE_LOCK(sc);

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
	    DMA_STATUS_IRQ_RX_MISSED_FRAME | DMA_STATUS_IRQ_TX_TRANSFER_DONE);
	if (stat == 0) {
		SMTE_UNLOCK(sc);
		return;
	}
	WR4(sc, DMA_STATUS_IRQ, stat);

	if ((stat & (DMA_STATUS_IRQ_RX_TRANSFER_DONE |
	    DMA_STATUS_IRQ_RX_MISSED_FRAME)) != 0)
		smte_rxeof(sc);

	if ((stat & DMA_STATUS_IRQ_TX_TRANSFER_DONE) != 0) {
		smte_txeof(sc);
		if (!if_sendq_empty(sc->ifp))
			smte_start_locked(sc->ifp);
	}

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
	WR4(sc, MAC_MAXIMUM_FRAME_SIZE, ETHER_MAX_LEN);
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

	/* Enable completion interrupts. */
	WR4(sc, DMA_INTR_ENABLE, DMA_INTR_ENABLE_TX_TRANSFER_DONE |
	    DMA_INTR_ENABLE_RX_TRANSFER_DONE |
	    DMA_INTR_ENABLE_RX_MISSED_FRAME |
	    DMA_INTR_ENABLE_RX_DMA_STOPPED |
	    DMA_INTR_ENABLE_RX_DES_UNAVAILABLE);

	device_printf(sc->dev,
	    "DBG init: rx ring pa %#lx desc0[0] %#x desc1[0] %#x addr[0] "
	    "%#x\n", (u_long)sc->rxdesc_paddr, sc->rxdesc[0].sd_desc0,
	    sc->rxdesc[0].sd_desc1, sc->rxdesc[0].sd_addr1);

	WR4(sc, MAC_TRANSMIT_CTRL,
	    (RD4(sc, MAC_TRANSMIT_CTRL) & ~MAC_TRANSMIT_CTRL_IFG_LEN_MASK) |
	    MAC_TRANSMIT_CTRL_TX_ENABLE | MAC_TRANSMIT_CTRL_TX_AUTO_RETRY);
	WR4(sc, MAC_RECEIVE_CTRL, RD4(sc, MAC_RECEIVE_CTRL) |
	    MAC_RECEIVE_CTRL_RX_ENABLE | MAC_RECEIVE_CTRL_STORE_FORWARD);

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

	if (sc->tx_watchdog > 0 && --sc->tx_watchdog == 0) {
		device_printf(sc->dev, "watchdog timeout\n");
		if_inc_counter(sc->ifp, IFCOUNTER_OERRORS, 1);
		smte_txeof(sc);
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
static void
smte_get1paddr(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{

	if (error == 0)
		*(bus_addr_t *)arg = segs[0].ds_addr;
}

static int
smte_setup_dma(struct smte_softc *sc)
{
	int error, i;

	/* Descriptor rings: 32-bit DMA, 8-byte alignment. */
	error = bus_dma_tag_create(bus_get_dma_tag(sc->dev), 8, 0,
	    BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL,
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
	error = bus_dmamap_load(sc->desc_tag, sc->txdesc_map, sc->txdesc,
	    SMTE_NTXDESC * sizeof(struct smte_desc), smte_get1paddr,
	    &sc->txdesc_paddr, 0);
	if (error != 0)
		return (error);

	error = bus_dmamem_alloc(sc->desc_tag, (void **)&sc->rxdesc,
	    BUS_DMA_NOWAIT | BUS_DMA_COHERENT | BUS_DMA_ZERO,
	    &sc->rxdesc_map);
	if (error != 0)
		return (error);
	error = bus_dmamap_load(sc->desc_tag, sc->rxdesc_map, sc->rxdesc,
	    SMTE_NRXDESC * sizeof(struct smte_desc), smte_get1paddr,
	    &sc->rxdesc_paddr, 0);
	if (error != 0)
		return (error);

	/* Buffer tags: 32-bit DMA. */
	error = bus_dma_tag_create(bus_get_dma_tag(sc->dev), 1, 0,
	    BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL,
	    MCLBYTES * SMTE_NTXSEGS, SMTE_NTXSEGS, MCLBYTES, 0, NULL, NULL,
	    &sc->txbuf_tag);
	if (error != 0)
		return (error);

	error = bus_dma_tag_create(bus_get_dma_tag(sc->dev), 1, 0,
	    BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL,
	    MCLBYTES, 1, MCLBYTES, 0, NULL, NULL, &sc->rxbuf_tag);
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
	 * Route the AXI master, and put the MAC into RGMII mode with the TX
	 * clock sourced from the SoC.  The clock+reset (bits 0/1) are already
	 * handled by clk_enable()/hwreset_deassert() in attach; but the RGMII
	 * interface-select (bit 2) and TX-clock-source (bit 8) are NOT, and the
	 * reset default is RMII with the wrong TX clock.  A minimal bootloader
	 * (e.g. our modern U-Boot) that doesn't program these leaves the RGMII
	 * PHY mis-clocked, so it never answers on MDIO and mii_attach() fails
	 * with "cannot attach PHY: 6".  Set them here so the driver is
	 * bootloader-independent.  (These boards are always RGMII -- the RGMII
	 * delay lines below are programmed unconditionally too.)
	 */
	SYSCON_MODIFY_4(sc->apmu, sc->apmu_offset + APMU_EMAC_CLK_RST_CTRL,
	    0, APMU_EMAC_AXI_MST_ID | APMU_EMAC_PHY_SEL_RGMII |
	    APMU_EMAC_RGMII_TXC_SRC_SEL);

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

	/* Reset the DMA engine. */
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
	if_setsendqlen(sc->ifp, SMTE_NTXDESC - 1);
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
