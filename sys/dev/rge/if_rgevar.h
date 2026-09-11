/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019, 2020, 2025 Kevin Lo <kevlo@openbsd.org>
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

/*	$OpenBSD: if_rgereg.h,v 1.15 2025/09/19 00:41:14 kevlo Exp $	*/

#ifndef	__IF_RGEVAR_H__
#define	__IF_RGEVAR_H__

#define	RGE_LOCK(sc)		(mtx_lock(&sc->sc_mtx))
#define	RGE_UNLOCK(sc)		(mtx_unlock(&sc->sc_mtx))
#define	RGE_ASSERT_LOCKED(sc)	(mtx_assert(&sc->sc_mtx, MA_OWNED))
#define	RGE_ASSERT_UNLOCKED(sc)	(mtx_assert(&sc->sc_mtx, MA_NOTOWNED))

enum rge_mac_type {
	MAC_UNKNOWN = 1,
	MAC_R25,
	MAC_R25B,
	MAC_R25D_1,
	MAC_R25D_2,
	MAC_R26_1,
	MAC_R26_2,
	MAC_R27
};

struct rge_drv_stats {
	/* How many times if_transmit() was called */
	uint64_t		transmit_call_cnt;
	/* Transmitted frame failed because the interface was stopped */
	uint64_t		transmit_stopped_cnt;
	/* Transmitted frame failed because the TX queue is full */
	uint64_t		transmit_full_cnt;
	/* How many transmit frames were queued for transmit */
	uint64_t		transmit_queued_cnt;

	/* How many times the interrupt routine was called */
	uint64_t		intr_cnt;
	/* How many times SYSTEM_ERR was set, requiring a hardware reset */
	uint64_t		intr_system_err_cnt;
	/* How many times rge_rxeof was called */
	uint64_t		rxeof_cnt;
	/* How many times rge_txeof was called */
	uint64_t		txeof_cnt;

	/* How many times the link state changed */
	uint64_t		link_state_change_cnt;

	/* How many times tx_task was run */
	uint64_t		tx_task_cnt;

	/* Count of received frames delivered to the stack or LRO. */
	uint64_t		recv_input_cnt;
	uint64_t		rx_lro_queued;
	uint64_t		rx_lro_flushed;

	/*
	 * For now - driver doesn't support multi descriptor
	 * RX frames; so count if it happens so it'll be noticed.
	 */
	uint64_t		rx_desc_err_multidesc;

	/*
	 * Number of TX watchdog timeouts.
	 */
	uint64_t		tx_watchdog_timeout_cnt;

	uint64_t		tx_encap_cnt;
	uint64_t		tx_encap_refrag_cnt;
	uint64_t		tx_encap_err_toofrag;
	uint64_t		tx_offload_ip_csum_set;
	uint64_t		tx_offload_tcp_csum_set;
	uint64_t		tx_offload_udp_csum_set;
	uint64_t		tx_offload_vlan_tag_set;
	uint64_t		tx_offload_tso_set;
	uint64_t		tx_offload_tso_err;
	uint64_t		rx_rss_hashed;

	uint64_t		rx_ether_csum_err;
	uint64_t		rx_desc_jumbo_frag;
	uint64_t		rx_offload_vlan_tag;
	uint64_t		rx_offload_csum_ipv4_exists;
	uint64_t		rx_offload_csum_ipv4_valid;

	uint64_t		rx_offload_csum_tcp_exists;
	uint64_t		rx_offload_csum_tcp_valid;

	uint64_t		rx_offload_csum_udp_exists;
	uint64_t		rx_offload_csum_udp_valid;
};

struct rge_txq {
	struct mbuf		*txq_mbuf;
	bus_dmamap_t		txq_dmamap;
	int			txq_descidx;
};

struct rge_rxq {
	struct mbuf		*rxq_mbuf;
	bus_dmamap_t		rxq_dmamap;
};

struct rge_tx {
	struct rge_txq		rge_txq[RGE_TX_LIST_CNT];
	int			rge_txq_prodidx;
	int			rge_txq_considx;

	bus_addr_t		rge_tx_list_paddr;
	bus_dmamap_t		rge_tx_list_map;
	struct rge_tx_desc	*rge_tx_list;
};

struct rge_rx {
	struct rge_rxq		rge_rxq[RGE_RX_LIST_CNT];
	int			rge_rxq_prodidx;
	int			rge_rxq_considx;

	bus_addr_t		rge_rx_list_paddr;
	bus_dmamap_t		rge_rx_list_map;
	struct rge_rx_desc	*rge_rx_list;

	struct mbuf		*rge_head;
	struct mbuf		**rge_tail;
};

struct lro_ctrl;

struct rge_queues {
	struct rge_softc	*q_sc;
	void			*q_ihc;
	int			q_index;
	char			q_name[16];
	struct rge_tx		q_tx;
	struct rge_rx		q_rx;
	/* Receive ring state is protected by q_rx_mtx (sc_mtx -> q_rx_mtx). */
	struct mtx		q_rx_mtx;
	struct lro_ctrl		*q_lro;
	uint64_t		q_rx_frames;
	uint64_t		q_rx_intr;
	/*
	 * Transmit ring state is protected by q_tx_mtx, a leaf lock taken
	 * without sc_mtx on the fast path so that forwarding threads landing
	 * on different TX queues do not serialise (one TX queue was the
	 * router-aggregate ceiling).  init/stop quiesce TX by taking every
	 * q_tx_mtx under sc_mtx.
	 */
	struct mtx		q_tx_mtx;
	char			q_tx_name[16];
	struct mbufq		q_txq;
	struct task		q_tx_task;
	bool			q_tx_task_pending;	/* q_tx_mtx */
	bool			q_tx_active;		/* has a TX ring + hw queue */
	uint64_t		q_tx_frames;
	uint64_t		q_tx_intr;
};

struct rge_mac_stats {
	bus_addr_t		paddr;
	bus_dmamap_t		map;
	/* NIC dma buffer, NIC order */
	struct rge_hw_mac_stats	*stats;

	/* Local copy for retrieval, host order */
	struct rge_hw_mac_stats	lcl_stats;
};

struct rge_softc {
	device_t		sc_dev;
	if_t			sc_ifp;		/* Ethernet common data */
	bool			sc_ether_attached;
	struct mtx		sc_mtx;
	struct resource		*sc_irq[RGE_MSIX_MESSAGES];
	void			*sc_ih[RGE_MSIX_MESSAGES];
	bool			sc_msix;	/* v2 interrupt space, per-queue vectors */
	int			sc_rx_miti;	/* v2 receive mitigation timer byte */
	int			sc_tx_miti;	/* v2 transmit completion timer byte */
	uint32_t		sc_expcap;	/* PCe exp cap */
	struct resource		*sc_bres;	/* bus space MMIO/IOPORT resource */
	struct resource		*sc_msix_res;	/* BAR holding the MSI-X table */
	int			sc_msix_rid;
	bus_space_handle_t	rge_bhandle;	/* bus space handle */
	bus_space_tag_t		rge_btag;	/* bus space tag */
	bus_size_t		rge_bsize;
	bus_dma_tag_t		sc_dmat;
	bus_dma_tag_t		sc_dmat_tx_desc;
	bus_dma_tag_t		sc_dmat_tx_buf;
	bus_dma_tag_t		sc_dmat_rx_desc;
	bus_dma_tag_t		sc_dmat_rx_buf;
	bus_dma_tag_t		sc_dmat_stats_buf;

	struct ifmedia		sc_media;	/* media info */
	enum rge_mac_type	rge_type;

	struct rge_queues	*sc_queues;
	unsigned int		sc_nqueues;	/* receive queues (RSS) */
	unsigned int		sc_ntxq;	/* transmit queues (1 or 2) */

	bool			sc_detaching;
	bool			sc_stopped;
	bool			sc_suspended;

	/* Shared taskqueue for the per-TXQ drain tasks (q_tx_task). */
	struct taskqueue *	sc_tq;
	char			sc_tq_name[32];
	char			sc_tq_thr_name[32];
	int			sc_tx_direct;	/* read-mostly; drain in caller ctx */

	struct callout		sc_timeout;	/* 1 second tick */

	uint64_t		rge_mcodever;
	uint16_t		rge_rcodever;
	uint32_t		rge_flags;
#define RGE_FLAG_MSI		0x00000001
#define RGE_FLAG_PCIE		0x00000002

	uint32_t		rge_intrs;
	int			rge_timerintr;
#define RGE_IMTYPE_NONE		0
#define RGE_IMTYPE_SIM		1
	int			sc_watchdog;

	int			rge_if_flags;

	uint32_t		sc_debug;

	int			sc_rx_process_limit;
	int			sc_rss_hash;
	int			sc_disable_aspm;

	struct rge_drv_stats	sc_drv_stats;

	struct rge_mac_stats	sc_mac_stats;
};

/*
 * Register space access macros.
 */
#define RGE_WRITE_4(sc, reg, val)	\
	bus_space_write_4(sc->rge_btag, sc->rge_bhandle, reg, val)
#define RGE_WRITE_2(sc, reg, val)	\
	bus_space_write_2(sc->rge_btag, sc->rge_bhandle, reg, val)
#define RGE_WRITE_1(sc, reg, val)	\
	bus_space_write_1(sc->rge_btag, sc->rge_bhandle, reg, val)

#define	RGE_WRITE_BARRIER_4(sc, reg)					\
	bus_space_barrier(sc->rge_btag, sc->rge_bhandle, reg, 4,	\
	    BUS_SPACE_BARRIER_WRITE)
#define	RGE_READ_BARRIER_4(sc, reg)					\
	bus_space_barrier(sc->rge_btag, sc->rge_bhandle, reg, 4,	\
	    BUS_SPACE_BARRIER_READ)


#define RGE_READ_4(sc, reg)		\
	bus_space_read_4(sc->rge_btag, sc->rge_bhandle, reg)
#define RGE_READ_2(sc, reg)		\
	bus_space_read_2(sc->rge_btag, sc->rge_bhandle, reg)
#define RGE_READ_1(sc, reg)		\
	bus_space_read_1(sc->rge_btag, sc->rge_bhandle, reg)

#define RGE_SETBIT_4(sc, reg, val)	\
	RGE_WRITE_4(sc, reg, RGE_READ_4(sc, reg) | (val))
#define RGE_SETBIT_2(sc, reg, val)	\
	RGE_WRITE_2(sc, reg, RGE_READ_2(sc, reg) | (val))
#define RGE_SETBIT_1(sc, reg, val)	\
	RGE_WRITE_1(sc, reg, RGE_READ_1(sc, reg) | (val))

#define RGE_CLRBIT_4(sc, reg, val)	\
	RGE_WRITE_4(sc, reg, RGE_READ_4(sc, reg) & ~(val))
#define RGE_CLRBIT_2(sc, reg, val)	\
	RGE_WRITE_2(sc, reg, RGE_READ_2(sc, reg) & ~(val))
#define RGE_CLRBIT_1(sc, reg, val)	\
	RGE_WRITE_1(sc, reg, RGE_READ_1(sc, reg) & ~(val))

#define RGE_EPHY_SETBIT(sc, reg, val)	\
	rge_write_ephy(sc, reg, rge_read_ephy(sc, reg) | (val))

#define RGE_EPHY_CLRBIT(sc, reg, val)	\
	rge_write_ephy(sc, reg, rge_read_ephy(sc, reg) & ~(val))

#define RGE_PHY_SETBIT(sc, reg, val)	\
	rge_write_phy_ocp(sc, reg, rge_read_phy_ocp(sc, reg) | (val))

#define RGE_PHY_CLRBIT(sc, reg, val)	\
	rge_write_phy_ocp(sc, reg, rge_read_phy_ocp(sc, reg) & ~(val))

#define RGE_MAC_SETBIT(sc, reg, val)	\
	rge_write_mac_ocp(sc, reg, rge_read_mac_ocp(sc, reg) | (val))

#define RGE_MAC_CLRBIT(sc, reg, val)	\
	rge_write_mac_ocp(sc, reg, rge_read_mac_ocp(sc, reg) & ~(val))

#endif	/* __IF_RGEVAR_H__ */
