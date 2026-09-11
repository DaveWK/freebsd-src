/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019, 2020, 2023-2025 Kevin Lo <kevlo@openbsd.org>
 * Copyright (c) 2025 Adrian Chadd <adrian@FreeBSD.org>
 *
 * Hardware programming portions from Realtek Semiconductor.
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

/*	$OpenBSD: if_rge.c,v 1.38 2025/09/19 00:41:14 kevlo Exp $	*/

#include "opt_inet.h"
#include "opt_inet6.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/sockio.h>
#include <sys/mbuf.h>
#include <sys/malloc.h>
#include <sys/endian.h>
#include <sys/socket.h>
#include <net/if.h>
#include <net/if_media.h>
#include <sys/queue.h>
#include <sys/taskqueue.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/smp.h>
#include <sys/sysctl.h>

#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <netinet/if_ether.h>
#if defined(INET) || defined(INET6)
#include <netinet/tcp_lro.h>
#endif

#include <net/bpf.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_vlan_var.h>
#include <net/rss_config.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/mii/mii.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>

#include "if_rge_vendor.h"
#include "if_rgereg.h"
#include "if_rgevar.h"
#include "if_rge_hw.h"
#include "if_rge_microcode.h"
#include "if_rge_debug.h"
#include "if_rge_sysctl.h"
#include "if_rge_stats.h"

#define ETHER_IS_VALID(addr) \
	(!ETHER_IS_MULTICAST(addr.octet) && !ETHER_IS_ZERO(addr.octet))

#define	RGE_CSUM_FEATURES		(CSUM_IP | CSUM_TCP | CSUM_UDP)

static SYSCTL_NODE(_hw, OID_AUTO, rge, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "rge driver");
static int rge_ntxq = RGE_MAX_TX_QUEUES;
SYSCTL_INT(_hw_rge, OID_AUTO, ntxq, CTLFLAG_RDTUN, &rge_ntxq, 0,
    "number of transmit queues to use with MSI-X (1 disables the second)");
static int rge_nqueues = RGE_MAX_RX_QUEUES;
SYSCTL_INT(_hw_rge, OID_AUTO, nqueues, CTLFLAG_RDTUN, &rge_nqueues, 0,
    "receive queues per port (1 = single MSI, legacy interrupt path)");
static int rge_msix = 1;
SYSCTL_INT(_hw_rge, OID_AUTO, msix, CTLFLAG_RDTUN, &rge_msix, 0,
    "use MSI-X with a vector per receive queue when nqueues > 1");
static int rge_next_cpu;

static int		rge_attach(device_t);
static int		rge_detach(device_t);

#if 0
int		rge_activate(struct device *, int);
#endif
static void	rge_intr_msi(void *);
static void	rge_intr_rxq(void *);
static void	rge_intr_txq(void *);
static void	rge_intr_link(void *);
static void	rge_rx_deliver(struct rge_queues *, struct mbufq *);
static int	rge_ioctl(struct ifnet *, u_long, caddr_t);
static int	rge_transmit_if(if_t, struct mbuf *);
static void	rge_qflush_if(if_t);
static void	rge_init_if(void *);
static void	rge_init_locked(struct rge_softc *);
static void	rge_stop_locked(struct rge_softc *);
static int	rge_ifmedia_upd(if_t);
static void	rge_ifmedia_sts(if_t, struct ifmediareq *);
static int	rge_allocmem(struct rge_softc *);
static int	rge_alloc_stats_mem(struct rge_softc *);
static int	rge_freemem(struct rge_softc *);
static int	rge_free_stats_mem(struct rge_softc *);
static int	rge_newbuf(struct rge_queues *);
static void	rge_rx_list_init(struct rge_queues *);
static void	rge_tx_list_init(struct rge_queues *);
static void	rge_fill_rx_ring(struct rge_queues *);
static int	rge_rxeof(struct rge_queues *, struct mbufq *);
static int	rge_txeof(struct rge_queues *);
static void	rge_iff_locked(struct rge_softc *);
static void	rge_add_media_types(struct rge_softc *);
static void	rge_tx_task(void *, int);
static void	rge_tx_drain_locked(struct rge_queues *);
static void	rge_schedule_tx_locked(struct rge_queues *);
static void	rge_txq_flush_mbufs(struct rge_softc *sc);
static struct rge_queues *rge_select_txq(struct rge_softc *, struct mbuf *);
static void	rge_tick(void *);
static void	rge_link_state(struct rge_softc *);
static void	rge_setwol(struct rge_softc *);

struct rge_matchid {
	uint16_t vendor;
	uint16_t device;
	const char *name;
};

const struct rge_matchid rge_devices[] = {
	{ PCI_VENDOR_REALTEK, PCI_PRODUCT_REALTEK_E3000, "Killer E3000" },
	{ PCI_VENDOR_REALTEK, PCI_PRODUCT_REALTEK_E5000, "Killer E5000" },
	{ PCI_VENDOR_REALTEK, PCI_PRODUCT_REALTEK_RTL8125, "RTL8125" },
	{ PCI_VENDOR_REALTEK, PCI_PRODUCT_REALTEK_RTL8126, "RTL8126", },
	{ PCI_VENDOR_REALTEK, PCI_PRODUCT_REALTEK_RTL8127, "RTL8127" },
	{ 0, 0, NULL }
};

static int
rge_probe(device_t dev)
{
	uint16_t vendor, device;
	const struct rge_matchid *ri;

	vendor = pci_get_vendor(dev);
	device = pci_get_device(dev);

	for (ri = rge_devices; ri->name != NULL; ri++) {
		if ((vendor == ri->vendor) && (device == ri->device)) {
			device_set_desc(dev, ri->name);
			return (BUS_PROBE_DEFAULT);
		}
	}

	return (ENXIO);
}

static void
rge_attach_if(struct rge_softc *sc, const char *eaddr)
{
	if_initname(sc->sc_ifp, device_get_name(sc->sc_dev),
	    device_get_unit(sc->sc_dev));
	if_setdev(sc->sc_ifp, sc->sc_dev);
	if_setinitfn(sc->sc_ifp, rge_init_if);
	if_setsoftc(sc->sc_ifp, sc);
	if_setflags(sc->sc_ifp, IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST);
	if_setioctlfn(sc->sc_ifp, rge_ioctl);
	if_settransmitfn(sc->sc_ifp, rge_transmit_if);
	if_setqflushfn(sc->sc_ifp, rge_qflush_if);

	/* Set offload as appropriate */
	if_sethwassist(sc->sc_ifp, CSUM_IP | CSUM_TCP | CSUM_UDP);
	if_setcapabilities(sc->sc_ifp, IFCAP_HWCSUM);
	if_setcapenable(sc->sc_ifp, if_getcapabilities(sc->sc_ifp));

	/*
	 * TSO (giant send v4, as Linux r8169/r8125 program it), on by
	 * default: validated on RTL8125 at 2.5 Gb/s line rate with exact
	 * payload hashes and per-segment options/checksums on the wire.
	 * Segment limits keep a full 64 KiB burst within RGE_TX_NSEGS.
	 */
	if_setcapabilitiesbit(sc->sc_ifp, IFCAP_TSO4, 0);
	if_sethwassistbits(sc->sc_ifp, CSUM_IP_TSO, 0);
	if_sethwtsomax(sc->sc_ifp, RGE_TSO_MAXSIZE);
	if_sethwtsomaxsegcount(sc->sc_ifp, RGE_TX_NSEGS - 1);
	if_sethwtsomaxsegsize(sc->sc_ifp, RGE_TSO_MAXSEGSIZE);

	/* Enable WOL if PM is supported. */
	if (pci_has_pm(sc->sc_dev)) {
		if_setcapabilitiesbit(sc->sc_ifp, IFCAP_WOL_MAGIC, 0);
		if_setcapenablebit(sc->sc_ifp, IFCAP_WOL_MAGIC, 0);
	}

	/* Attach interface */
	ether_ifattach(sc->sc_ifp, eaddr);
	sc->sc_ether_attached = true;

	/* post ether_ifattach() bits */

	/* VLAN capabilities */
	if_setcapabilitiesbit(sc->sc_ifp, IFCAP_VLAN_MTU |
	    IFCAP_VLAN_HWTAGGING, 0);
	if_setcapabilitiesbit(sc->sc_ifp, IFCAP_VLAN_HWCSUM, 0);
	if_setcapenable(sc->sc_ifp, if_getcapabilities(sc->sc_ifp));

	if_setifheaderlen(sc->sc_ifp, sizeof(struct ether_vlan_header));

#if defined(INET) || defined(INET6)
	/*
	 * Software LRO, on by default: an MTU-1500 endpoint receives at
	 * 2.5 GbE line rate with it and at ~0.9 Gb/s without.  tcp_lro_rx()
	 * refuses to aggregate while net.inet.ip.forwarding is set, so a
	 * router sees ordinary receive whether or not it is enabled.
	 */
	{
		struct rge_queues *q = sc->sc_queues;
		bool lro_ok = true;
		int i;

		for (i = 0; i < sc->sc_nqueues && lro_ok; i++) {
			q[i].q_lro = malloc(sizeof(*q[i].q_lro), M_DEVBUF,
			    M_NOWAIT | M_ZERO);
			if (q[i].q_lro == NULL ||
			    tcp_lro_init_args(q[i].q_lro, sc->sc_ifp, 128, 0) != 0) {
				free(q[i].q_lro, M_DEVBUF);
				q[i].q_lro = NULL;
				lro_ok = false;
			}
		}
		if (lro_ok) {
			if_setcapabilitiesbit(sc->sc_ifp, IFCAP_LRO, 0);
			if_setcapenablebit(sc->sc_ifp, IFCAP_LRO, 0);
		} else {
			for (i = 0; i < sc->sc_nqueues; i++) {
				if (q[i].q_lro != NULL) {
					tcp_lro_free(q[i].q_lro);
					free(q[i].q_lro, M_DEVBUF);
					q[i].q_lro = NULL;
				}
			}
		}
	}
#endif

	/* TODO: is this needed for iftransmit? */
	if_setsendqlen(sc->sc_ifp, RGE_TX_LIST_CNT - 1);
	if_setsendqready(sc->sc_ifp);
}

static int
rge_attach(device_t dev)
{
	struct ether_addr eaddr;
	struct rge_softc *sc;
	struct rge_queues *q;
	uint32_t hwrev, reg;
	int i, rid;
	int error;
	int msic;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	sc->sc_ifp = if_gethandle(IFT_ETHER);
	mtx_init(&sc->sc_mtx, device_get_nameunit(dev), MTX_NETWORK_LOCK,
	    MTX_DEF);

	callout_init_mtx(&sc->sc_timeout, &sc->sc_mtx, 0);

	/* Enable bus mastering */
	pci_enable_busmaster(dev);

	/*
	 * Map control/status registers.
	 */

	/*
	 * The openbsd driver (and my E3000 NIC) handle registering three
	 * kinds of BARs - a 64 bit MMIO BAR, a 32 bit MMIO BAR, and then
	 * a legacy IO port BAR.
	 *
	 * To simplify bring-up, I'm going to request resources for the first
	 * MMIO BAR (BAR2) which should be a 32 bit BAR.
	 */
	rid = PCIR_BAR(2);
	sc->sc_bres = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->sc_bres == NULL) {
		RGE_PRINT_ERROR(sc,
		    "Unable to allocate bus resource: memory\n");
		goto fail;
	}
	sc->rge_bhandle = rman_get_bushandle(sc->sc_bres);
	sc->rge_btag = rman_get_bustag(sc->sc_bres);
	sc->rge_bsize = rman_get_size(sc->sc_bres);

	/* Determine hardware revision */
	hwrev = RGE_READ_4(sc, RGE_TXCFG) & RGE_TXCFG_HWREV;
	switch (hwrev) {
	case 0x60900000:
		sc->rge_type = MAC_R25;
		device_printf(dev, "chip rev: RTL8125 (0x%08x)\n", hwrev);
		break;
	case 0x64100000:
		sc->rge_type = MAC_R25B;
		device_printf(dev, "chip rev: RTL8125B (0x%08x)\n", hwrev);
		break;
	case 0x64900000:
		sc->rge_type = MAC_R26_1;
		device_printf(dev, "chip rev: RTL8126_1 (0x%08x)\n", hwrev);
		break;
	case 0x64a00000:
		sc->rge_type = MAC_R26_2;
		device_printf(dev, "chip rev: RTL8126_2 (0x%08x)\n", hwrev);
		break;
	case 0x68800000:
		sc->rge_type = MAC_R25D_1;
		device_printf(dev, "chip rev: RTL8125D_1 (0x%08x)\n", hwrev);
		break;
	case 0x68900000:
		sc->rge_type = MAC_R25D_2;
		device_printf(dev, "chip rev: RTL8125D_2 (0x%08x)\n", hwrev);
		break;
	case 0x6c900000:
		sc->rge_type = MAC_R27;
		device_printf(dev, "chip rev: RTL8127 (0x%08x)\n", hwrev);
		break;
	default:
		RGE_PRINT_ERROR(sc, "unknown version 0x%08x\n", hwrev);
		goto fail;
	}

	/*
	 * Receive queues.  Each has its own ring, lock and (with MSI-X on an
	 * RTL8125B+) its own interrupt vector; queue 0 also carries the
	 * transmit ring.  hw.rge.nqueues=1 keeps the proven single-MSI path.
	 */
	sc->sc_nqueues = rge_nqueues;
	if (sc->sc_nqueues < 1)
		sc->sc_nqueues = 1;
	if (sc->sc_nqueues > RGE_MAX_RX_QUEUES)
		sc->sc_nqueues = RGE_MAX_RX_QUEUES;
	if (sc->sc_nqueues > mp_ncpus)
		sc->sc_nqueues = mp_ncpus;
	sc->sc_nqueues = 1 << (fls(sc->sc_nqueues) - 1);	/* power of two */

	/* Check if PCIe */
	if (pci_find_cap(dev, PCIY_EXPRESS, &reg) == 0) {
		sc->rge_flags |= RGE_FLAG_PCIE;
		sc->sc_expcap = reg;
	}

	/*
	 * MSI-X with the v2 interrupt space needs all 22 vectors because the
	 * chip's vector numbers are fixed; fall back to one MSI and one queue
	 * when they are not available or not wanted.
	 */
	sc->sc_msix = false;
	msic = 0;
	if (sc->sc_nqueues > 1 && rge_msix != 0 && sc->rge_type != MAC_R25 &&
	    pci_msix_count(dev) >= RGE_MSIX_MESSAGES) {
		/*
		 * pci_alloc_msix() needs the BAR holding the MSI-X table
		 * mapped by the driver; the RTL8125 keeps it in BAR4, which
		 * firmware may have left unassigned, so the bus assigns it now.
		 */
		sc->sc_msix_rid = pci_msix_table_bar(dev);
		if (sc->sc_msix_rid > 0)
			sc->sc_msix_res = bus_alloc_resource_any(dev,
			    SYS_RES_MEMORY, &sc->sc_msix_rid, RF_ACTIVE);
		if (sc->sc_msix_res != NULL) {
			msic = RGE_MSIX_MESSAGES;
			if (pci_alloc_msix(dev, &msic) == 0) {
				if (msic == RGE_MSIX_MESSAGES)
					sc->sc_msix = true;
				else
					pci_release_msi(dev);
			}
		}
		if (!sc->sc_msix)
			device_printf(dev, "MSI-X unavailable (table BAR %d, %d "
			    "messages); using one MSI and one queue\n",
			    sc->sc_msix_rid, msic);
	}
	if (!sc->sc_msix)
		sc->sc_nqueues = 1;

	/*
	 * Transmit queues share the rge_queues array with the receive
	 * queues.  The 8125 supports up to RGE_MAX_TX_QUEUES; use a second
	 * one (each with its own ring, lock and completion vector) only on
	 * the MSI-X path, so forwarding threads stop serialising on one TX
	 * lock.  hw.rge.ntxq=1 keeps the single-TX-queue behaviour.
	 */
	sc->sc_ntxq = sc->sc_msix ? rge_ntxq : 1;
	if (sc->sc_ntxq < 1)
		sc->sc_ntxq = 1;
	if (sc->sc_ntxq > RGE_MAX_TX_QUEUES)
		sc->sc_ntxq = RGE_MAX_TX_QUEUES;
	if (sc->sc_ntxq > sc->sc_nqueues)
		sc->sc_ntxq = sc->sc_nqueues;

	q = malloc(sizeof(struct rge_queues) * sc->sc_nqueues, M_DEVBUF,
	    M_NOWAIT | M_ZERO);
	if (q == NULL) {
		RGE_PRINT_ERROR(sc, "Unable to malloc rge_queues memory\n");
		goto fail;
	}
	sc->sc_queues = q;
	for (i = 0; i < sc->sc_nqueues; i++) {
		q[i].q_sc = sc;
		q[i].q_index = i;
		snprintf(q[i].q_name, sizeof(q[i].q_name), "%s:rx%d",
		    device_get_nameunit(dev), i);
		mtx_init(&q[i].q_rx_mtx, q[i].q_name, NULL, MTX_DEF);
	}

	if (sc->sc_msix) {
		sc->rge_flags |= RGE_FLAG_MSI;
		/* Receive queue vectors 0..n-1, bound to distinct CPUs. */
		for (i = 0; i < sc->sc_nqueues; i++) {
			rid = i + 1;
			sc->sc_irq[i] = bus_alloc_resource_any(dev, SYS_RES_IRQ,
			    &rid, RF_ACTIVE);
			if (sc->sc_irq[i] == NULL) {
				RGE_PRINT_ERROR(sc, "%s: couldn't allocate "
				    "MSI-X vector %d\n", __func__, i);
				goto fail;
			}
			error = bus_setup_intr(dev, sc->sc_irq[i],
			    INTR_TYPE_NET | INTR_MPSAFE, NULL, rge_intr_rxq,
			    &q[i], &sc->sc_ih[i]);
			if (error != 0) {
				RGE_PRINT_ERROR(sc, "%s: couldn't setup rx "
				    "vector %d (error %d)\n", __func__, i, error);
				goto fail;
			}
			bus_describe_intr(dev, sc->sc_irq[i], sc->sc_ih[i],
			    "rx%d", i);
			bus_bind_intr(dev, sc->sc_irq[i],
			    rge_next_cpu++ % mp_ncpus);
		}
		/* Transmit completion vector(s): one per TX queue. */
		for (i = 0; i < sc->sc_ntxq; i++) {
			int vec = RGE_V2_VEC_TX_Q(i);

			rid = vec + 1;
			sc->sc_irq[vec] = bus_alloc_resource_any(dev,
			    SYS_RES_IRQ, &rid, RF_ACTIVE);
			if (sc->sc_irq[vec] == NULL) {
				RGE_PRINT_ERROR(sc, "%s: couldn't allocate TX "
				    "MSI-X vector %d\n", __func__, vec);
				goto fail;
			}
			error = bus_setup_intr(dev, sc->sc_irq[vec],
			    INTR_TYPE_NET | INTR_MPSAFE, NULL, rge_intr_txq,
			    &q[i], &sc->sc_ih[vec]);
			if (error != 0) {
				RGE_PRINT_ERROR(sc, "%s: couldn't setup TX "
				    "vector %d (error %d)\n", __func__, vec,
				    error);
				goto fail;
			}
			bus_describe_intr(dev, sc->sc_irq[vec],
			    sc->sc_ih[vec], "tx%d", i);
			q[i].q_tx_active = true;
		}
		/* Link change vector. */
		rid = RGE_V2_VEC_LINK + 1;
		sc->sc_irq[RGE_V2_VEC_LINK] = bus_alloc_resource_any(dev,
		    SYS_RES_IRQ, &rid, RF_ACTIVE);
		if (sc->sc_irq[RGE_V2_VEC_LINK] == NULL) {
			RGE_PRINT_ERROR(sc, "%s: couldn't allocate the link "
			    "MSI-X vector\n", __func__);
			goto fail;
		}
		error = bus_setup_intr(dev, sc->sc_irq[RGE_V2_VEC_LINK],
		    INTR_TYPE_NET | INTR_MPSAFE, NULL, rge_intr_link, sc,
		    &sc->sc_ih[RGE_V2_VEC_LINK]);
		if (error != 0) {
			RGE_PRINT_ERROR(sc, "%s: couldn't setup link "
			    "vector (error %d)\n", __func__, error);
			goto fail;
		}
		bus_describe_intr(dev, sc->sc_irq[RGE_V2_VEC_LINK],
		    sc->sc_ih[RGE_V2_VEC_LINK], "link");
		device_printf(dev, "MSI-X: %d receive queues\n", sc->sc_nqueues);
	} else {
		/* Allocate MSI */
		msic = pci_msi_count(dev);
		if (msic == 0) {
			RGE_PRINT_ERROR(sc, "%s: only MSI interrupts supported\n",
			    __func__);
			goto fail;
		}

		msic = RGE_MSI_MESSAGES;
		if (pci_alloc_msi(dev, &msic) != 0) {
			RGE_PRINT_ERROR(sc, "%s: failed to allocate MSI\n",
			    __func__);
			goto fail;
		}

		sc->rge_flags |= RGE_FLAG_MSI;

		/* We need at least one MSI */
		if (msic < RGE_MSI_MESSAGES) {
			RGE_PRINT_ERROR(sc, "%s: didn't allocate enough MSI\n",
			    __func__);
			goto fail;
		}

		/*
		 * Allocate interrupt entries.
		 */
		for (i = 0, rid = 1; i < RGE_MSI_MESSAGES; i++, rid++) {
			sc->sc_irq[i] = bus_alloc_resource_any(dev, SYS_RES_IRQ,
			    &rid, RF_ACTIVE);
			if (sc->sc_irq[i] == NULL) {
				RGE_PRINT_ERROR(sc, "%s: couldn't allocate MSI %d",
				    __func__, rid);
				goto fail;
			}
		}

		/* Hook interrupts */
		for (i = 0; i < RGE_MSI_MESSAGES; i++) {
			error = bus_setup_intr(dev, sc->sc_irq[i],
			    INTR_TYPE_NET | INTR_MPSAFE, NULL, rge_intr_msi,
			    sc, &sc->sc_ih[i]);
			if (error != 0) {
				RGE_PRINT_ERROR(sc,
				    "%s: couldn't setup intr %d (error %d)", __func__,
				    i, error);
				goto fail;
			}
		}
	}

	/* Allocate top level bus DMA tag */
	error = bus_dma_tag_create(bus_get_dma_tag(dev),
	    1, /* alignment */
	    0, /* boundary */
	    BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR,
	    NULL, NULL, /* filter (unused) */
	    BUS_SPACE_MAXSIZE, /* maxsize */
	    BUS_SPACE_UNRESTRICTED, /* nsegments */
	    BUS_SPACE_MAXSIZE, /* maxsegsize */
	    0, /* flags */
	    NULL, NULL, /* lockfunc, lockarg */
	    &sc->sc_dmat);
	if (error) {
		RGE_PRINT_ERROR(sc,
		    "couldn't allocate device DMA tag (error %d)\n", error);
		goto fail;
	}

	/* Allocate TX/RX descriptor and buffer tags */
	error = bus_dma_tag_create(sc->sc_dmat,
	    RGE_ALIGN, /* alignment */
	    0, /* boundary */
	    BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR,
	    NULL, NULL, /* filter (unused) */
	    RGE_TX_LIST_SZ, /* maxsize */
	    1, /* nsegments */
	    RGE_TX_LIST_SZ, /* maxsegsize */
	    0, /* flags */
	    NULL, NULL, /* lockfunc, lockarg */
	    &sc->sc_dmat_tx_desc);
	if (error) {
		RGE_PRINT_ERROR(sc,
		    "couldn't allocate device TX descriptor "
		    "DMA tag (error %d)\n", error);
		    goto fail;
	}

	error = bus_dma_tag_create(sc->sc_dmat,
	    1, /* alignment */
	    0, /* boundary */
	    BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR,
	    NULL, NULL, /* filter (unused) */
	    RGE_TSO_MAXSIZE, /* maxsize */
	    RGE_TX_NSEGS, /* nsegments */
	    RGE_JUMBO_FRAMELEN, /* maxsegsize */
	    0, /* flags */
	    NULL, NULL, /* lockfunc, lockarg */
	    &sc->sc_dmat_tx_buf);
	if (error) {
		RGE_PRINT_ERROR(sc,
		    "couldn't allocate device TX buffer DMA tag (error %d)\n",
		    error);
		goto fail;
	}

	error = bus_dma_tag_create(sc->sc_dmat,
	    RGE_ALIGN, /* alignment */
	    0, /* boundary */
	    BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR,
	    NULL, NULL, /* filter (unused) */
	    RGE_RX_LIST_SZ, /* maxsize */
	    1, /* nsegments */
	    RGE_RX_LIST_SZ, /* maxsegsize */
	    0, /* flags */
	    NULL, NULL, /* lockfunc, lockarg */
	    &sc->sc_dmat_rx_desc);
	if (error) {
		RGE_PRINT_ERROR(sc,
		    "couldn't allocate device RX descriptor "
		    "DMA tag (error %d)\n", error);
		goto fail;
	}

	error = bus_dma_tag_create(sc->sc_dmat,
	    1, /* alignment */
	    0, /* boundary */
	    BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR,
	    NULL, NULL, /* filter (unused) */
	    MCLBYTES, /* maxsize */
	    1, /* nsegments */
	    MCLBYTES, /* maxsegsize */
	    0, /* flags */
	    NULL, NULL, /* lockfunc, lockarg */
	    &sc->sc_dmat_rx_buf);
	if (error) {
		RGE_PRINT_ERROR(sc,
		    "couldn't allocate device RX buffer DMA tag (error %d)\n",
		    error);
		goto fail;
	}

	error = bus_dma_tag_create(sc->sc_dmat,
	    RGE_STATS_ALIGNMENT, /* alignment */
	    0, /* boundary */
	    BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR,
	    NULL, NULL, /* filter (unused) */
	    RGE_STATS_BUF_SIZE, /* maxsize */
	    1, /* nsegments */
	    RGE_STATS_BUF_SIZE, /* maxsegsize */
	    0, /* flags */
	    NULL, NULL, /* lockfunc, lockarg */
	    &sc->sc_dmat_stats_buf);
	if (error) {
		RGE_PRINT_ERROR(sc,
		    "couldn't allocate device RX buffer DMA tag (error %d)\n",
		    error);
		goto fail;
	}


	/* Attach sysctl nodes */
	rge_sysctl_attach(sc);

	rge_config_imtype(sc, RGE_IMTYPE_SIM);

	/* Disable PCIe ASPM and ECPM if requested. */
	if (sc->sc_disable_aspm) {
		int ecap;
		if (pci_find_cap(dev, PCIY_EXPRESS, &ecap) == 0) {
			uint16_t lctl;
			lctl = pci_read_config(dev,
			    ecap + PCIER_LINK_CTL, 2);
			lctl &= ~(PCIEM_LINK_CTL_ASPMC |
			    PCIEM_LINK_CTL_ECPM);
			pci_write_config(dev,
			    ecap + PCIER_LINK_CTL, lctl, 2);
		}
	}

	RGE_LOCK(sc);
	if (rge_chipinit(sc)) {
		RGE_UNLOCK(sc);
		goto fail;
	}

	rge_get_macaddr(sc, eaddr.octet);
	RGE_UNLOCK(sc);

	if (!ETHER_IS_VALID(eaddr)) {
		device_printf(dev,
		    "No MAC address found.  Using ether_gen_addr().\n");
		ether_gen_addr_byname(device_get_nameunit(dev), &eaddr);
	}

	if (rge_allocmem(sc))
		goto fail;
	if (rge_alloc_stats_mem(sc))
		goto fail;

	/* Initialize ifmedia structures. */
	ifmedia_init(&sc->sc_media, IFM_IMASK, rge_ifmedia_upd,
	    rge_ifmedia_sts);
	rge_add_media_types(sc);
	ifmedia_add(&sc->sc_media, IFM_ETHER | IFM_AUTO, 0, NULL);
	ifmedia_set(&sc->sc_media, IFM_ETHER | IFM_AUTO);
	sc->sc_media.ifm_media = sc->sc_media.ifm_cur->ifm_media;

	rge_attach_if(sc, eaddr.octet);

	snprintf(sc->sc_tq_name, sizeof(sc->sc_tq_name),
	    "%s taskq", device_get_nameunit(sc->sc_dev));
	snprintf(sc->sc_tq_thr_name, sizeof(sc->sc_tq_thr_name),
	    "%s taskq thread", device_get_nameunit(sc->sc_dev));

	sc->sc_tq = taskqueue_create(sc->sc_tq_name, M_NOWAIT,
	    taskqueue_thread_enqueue, &sc->sc_tq);
	taskqueue_start_threads(&sc->sc_tq, 1, PI_NET, "%s",
	    sc->sc_tq_thr_name);

	/* Per-TX-queue software queue, drain task and leaf lock. */
	for (i = 0; i < sc->sc_nqueues; i++) {
		mbufq_init(&q[i].q_txq, RGE_TX_LIST_CNT);
		TASK_INIT(&q[i].q_tx_task, 0, rge_tx_task, &q[i]);
		snprintf(q[i].q_tx_name, sizeof(q[i].q_tx_name), "%s:tx%d",
		    device_get_nameunit(dev), i);
		mtx_init(&q[i].q_tx_mtx, q[i].q_tx_name, NULL, MTX_DEF);
	}

	return (0);
fail:
	rge_detach(dev);
	return (ENXIO);
}

/**
 * @brief flush the mbufq queue
 *
 * Again this should likely be per-TXQ.
 *
 * This should be called with the driver lock held.
 */
static void
rge_txq_flush_mbufs(struct rge_softc *sc)
{
	struct mbuf *m;
	int ntx = 0;
	unsigned int i;

	RGE_ASSERT_LOCKED(sc);

	for (i = 0; i < sc->sc_nqueues; i++) {
		struct rge_queues *q = &sc->sc_queues[i];

		mtx_lock(&q->q_tx_mtx);
		while ((m = mbufq_dequeue(&q->q_txq)) != NULL) {
			m_freem(m);
			ntx++;
		}
		mtx_unlock(&q->q_tx_mtx);
	}

	RGE_DPRINTF(sc, RGE_DEBUG_XMIT, "%s: %d frames flushed\n", __func__,
	    ntx);
}

static int
rge_detach(device_t dev)
{
	struct rge_softc *sc = device_get_softc(dev);
	int i, rid;

	/* global flag, detaching */
	RGE_LOCK(sc);
	sc->sc_stopped = true;
	sc->sc_detaching = true;
	RGE_UNLOCK(sc);

	/* stop/drain network interface */
	callout_drain(&sc->sc_timeout);

	/* Make sure no per-queue TX task is running */
	if (sc->sc_tq != NULL && sc->sc_queues != NULL) {
		unsigned int i;

		for (i = 0; i < sc->sc_nqueues; i++) {
			struct task *t = &sc->sc_queues[i].q_tx_task;

			while (taskqueue_cancel(sc->sc_tq, t, NULL) != 0)
				taskqueue_drain(sc->sc_tq, t);
		}
	}

	RGE_LOCK(sc);
	callout_stop(&sc->sc_timeout);

	/* stop NIC / DMA */
	rge_stop_locked(sc);

	/* TODO: wait for completion */

	/* Free pending TX mbufs */
	rge_txq_flush_mbufs(sc);

	RGE_UNLOCK(sc);

	/*
	 * The interrupt handler delivers detached RX mbufs after dropping
	 * RGE_LOCK. Drain it before freeing queues, DMA maps or the ifnet.
	 */
	for (i = 0; i < RGE_MSIX_MESSAGES; i++) {
		if (sc->sc_ih[i] != NULL) {
			bus_teardown_intr(sc->sc_dev, sc->sc_irq[i],
			    sc->sc_ih[i]);
			sc->sc_ih[i] = NULL;
		}
	}

	/* Free taskqueue */
	if (sc->sc_tq != NULL) {
		taskqueue_free(sc->sc_tq);
		sc->sc_tq = NULL;
	}

#if defined(INET) || defined(INET6)
	/* Interrupt delivery has drained, so no LRO operation can still run. */
	if (sc->sc_queues != NULL) {
		for (i = 0; i < sc->sc_nqueues; i++) {
			if (sc->sc_queues[i].q_lro != NULL) {
				tcp_lro_free(sc->sc_queues[i].q_lro);
				free(sc->sc_queues[i].q_lro, M_DEVBUF);
				sc->sc_queues[i].q_lro = NULL;
			}
		}
	}
#endif

	/* Free descriptor memory */
	RGE_DPRINTF(sc, RGE_DEBUG_SETUP, "%s: freemem\n", __func__);
	rge_freemem(sc);
	rge_free_stats_mem(sc);

	if (sc->sc_ifp) {
		RGE_DPRINTF(sc, RGE_DEBUG_SETUP, "%s: ifdetach/if_free\n",
		    __func__);
		if (sc->sc_ether_attached)
			ether_ifdetach(sc->sc_ifp);
		if_free(sc->sc_ifp);
	}

	RGE_DPRINTF(sc, RGE_DEBUG_SETUP, "%s: sc_dmat_tx_desc\n", __func__);
	if (sc->sc_dmat_tx_desc)
		bus_dma_tag_destroy(sc->sc_dmat_tx_desc);
	RGE_DPRINTF(sc, RGE_DEBUG_SETUP, "%s: sc_dmat_tx_buf\n", __func__);
	if (sc->sc_dmat_tx_buf)
		bus_dma_tag_destroy(sc->sc_dmat_tx_buf);
	RGE_DPRINTF(sc, RGE_DEBUG_SETUP, "%s: sc_dmat_rx_desc\n", __func__);
	if (sc->sc_dmat_rx_desc)
		bus_dma_tag_destroy(sc->sc_dmat_rx_desc);
	RGE_DPRINTF(sc, RGE_DEBUG_SETUP, "%s: sc_dmat_rx_buf\n", __func__);
	if (sc->sc_dmat_rx_buf)
		bus_dma_tag_destroy(sc->sc_dmat_rx_buf);
	RGE_DPRINTF(sc, RGE_DEBUG_SETUP, "%s: sc_dmat_stats_buf\n", __func__);
	if (sc->sc_dmat_stats_buf)
		bus_dma_tag_destroy(sc->sc_dmat_stats_buf);
	RGE_DPRINTF(sc, RGE_DEBUG_SETUP, "%s: sc_dmat\n", __func__);
	if (sc->sc_dmat)
		bus_dma_tag_destroy(sc->sc_dmat);

	/* Free interrupt resources */
	for (i = 0, rid = 1; i < RGE_MSIX_MESSAGES; i++, rid++) {
		if (sc->sc_irq[i] != NULL) {
			bus_release_resource(sc->sc_dev, SYS_RES_IRQ,
			    rid, sc->sc_irq[i]);
			sc->sc_irq[i] = NULL;
		}
	}

	/* Free MSI allocation */
	if (sc->rge_flags & RGE_FLAG_MSI)
		pci_release_msi(dev);

	if (sc->sc_bres) {
		RGE_DPRINTF(sc, RGE_DEBUG_SETUP, "%s: release mmio\n",
		    __func__);
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->sc_bres), sc->sc_bres);
		sc->sc_bres = NULL;
	}
	if (sc->sc_msix_res) {
		bus_release_resource(dev, SYS_RES_MEMORY, sc->sc_msix_rid,
		    sc->sc_msix_res);
		sc->sc_msix_res = NULL;
	}

	if (sc->sc_queues) {
		for (i = 0; i < sc->sc_nqueues; i++) {
			if (mtx_initialized(&sc->sc_queues[i].q_rx_mtx))
				mtx_destroy(&sc->sc_queues[i].q_rx_mtx);
			if (mtx_initialized(&sc->sc_queues[i].q_tx_mtx))
				mtx_destroy(&sc->sc_queues[i].q_tx_mtx);
		}
		free(sc->sc_queues, M_DEVBUF);
		sc->sc_queues = NULL;
	}

	mtx_destroy(&sc->sc_mtx);

	return (0);
}

/*
 * Hand received frames to the stack outside the driver locks, through
 * this queue's LRO controller when enabled.
 */
static void
rge_rx_deliver(struct rge_queues *q, struct mbufq *mq)
{
	struct rge_softc *sc = q->q_sc;
	struct epoch_tracker et;
	struct mbuf *m;
#if defined(INET) || defined(INET6)
	uint64_t lro_flushed = 0;
	bool do_lro;

	do_lro = q->q_lro != NULL &&
	    (if_getcapenable(sc->sc_ifp) & IFCAP_LRO) != 0;
	if (do_lro)
		lro_flushed = q->q_lro->lro_flushed;
#endif
	if (mbufq_first(mq) == NULL)
		return;

	NET_EPOCH_ENTER(et);
	while ((m = mbufq_dequeue(mq)) != NULL) {
		sc->sc_drv_stats.recv_input_cnt++;
#if defined(INET) || defined(INET6)
		if (do_lro) {
			/* tcp_lro_rx rejects forwarding and invalid checksums. */
			if (tcp_lro_rx(q->q_lro, m, 0) == 0) {
				sc->sc_drv_stats.rx_lro_queued++;
				continue;
			}
			/* Preserve ordering before delivering an unaggregated frame. */
			tcp_lro_flush_all(q->q_lro);
		}
#endif
		if_input(sc->sc_ifp, m);
	}
#if defined(INET) || defined(INET6)
	if (do_lro) {
		tcp_lro_flush_all(q->q_lro);
		sc->sc_drv_stats.rx_lro_flushed +=
		    q->q_lro->lro_flushed - lro_flushed;
	}
#endif
	NET_EPOCH_EXIT(et);
}

/*
 * MSI-X, v2 interrupt space: one vector per receive queue.  The vector's
 * bit is masked while its ring is drained and re-enabled afterwards; the
 * mask/ack registers are per-bit so no other vector is disturbed.
 */
static void
rge_intr_rxq(void *arg)
{
	struct rge_queues *q = arg;
	struct rge_softc *sc = q->q_sc;
	struct mbufq rx_mq;
	uint32_t bit = RGE_ISR_V2_ROK(q->q_index);
	int pass, more;

	q->q_rx_intr++;
	sc->sc_drv_stats.intr_cnt++;
	if ((if_getdrvflags(sc->sc_ifp) & IFF_DRV_RUNNING) == 0)
		return;

	mbufq_init(&rx_mq, RGE_RX_LIST_CNT);
	RGE_WRITE_4(sc, RGE_IMR_V2_CLR, bit);
	RGE_WRITE_4(sc, RGE_ISR_V2, bit);

	/*
	 * Drain the ring: the vector only re-fires on a new arrival, so
	 * frames left behind a full batch would wait for the next packet.
	 * Batches of rx_process_limit bound the lock hold and the mbuf
	 * queue; the pass bound keeps a flood from monopolising the thread.
	 */
	for (pass = 0; pass < RGE_RX_LIST_CNT / 16; pass++) {
		mtx_lock(&q->q_rx_mtx);
		more = !sc->sc_stopped && !sc->sc_detaching &&
		    rge_rxeof(q, &rx_mq);
		mtx_unlock(&q->q_rx_mtx);
		rge_rx_deliver(q, &rx_mq);
		if (!more)
			break;
	}

	RGE_WRITE_4(sc, RGE_IMR_V2_SET, bit);
}

static void
rge_intr_txq(void *arg)
{
	struct rge_queues *q = arg;
	struct rge_softc *sc = q->q_sc;
	uint32_t bit = RGE_ISR_V2_TOK(q->q_index);

	if ((if_getdrvflags(sc->sc_ifp) & IFF_DRV_RUNNING) == 0)
		return;
	q->q_tx_intr++;
	RGE_WRITE_4(sc, RGE_IMR_V2_CLR, bit);
	RGE_WRITE_4(sc, RGE_ISR_V2, bit);
	mtx_lock(&q->q_tx_mtx);
	if (!sc->sc_stopped && !sc->sc_detaching)
		rge_txeof(q);
	mtx_unlock(&q->q_tx_mtx);
	RGE_WRITE_4(sc, RGE_IMR_V2_SET, bit);
}

static void
rge_intr_link(void *arg)
{
	struct rge_softc *sc = arg;
	uint32_t bit = RGE_ISR_V2_LINKCHG;

	RGE_WRITE_4(sc, RGE_IMR_V2_CLR, bit);
	RGE_WRITE_4(sc, RGE_ISR_V2, bit);
	RGE_LOCK(sc);
	if (!sc->sc_detaching)
		rge_link_state(sc);
	RGE_UNLOCK(sc);
	RGE_WRITE_4(sc, RGE_IMR_V2_SET, bit);
}

static void
rge_intr_msi(void *arg)
{
	struct mbufq rx_mq;
	struct rge_softc *sc = arg;
	struct rge_queues *q = sc->sc_queues;
	uint32_t status;
	int claimed = 0, rv;

	sc->sc_drv_stats.intr_cnt++;

	mbufq_init(&rx_mq, RGE_RX_LIST_CNT);

	if ((if_getdrvflags(sc->sc_ifp) & IFF_DRV_RUNNING) == 0)
		return;

	RGE_LOCK(sc);

	if (sc->sc_suspended || sc->sc_stopped || sc->sc_detaching) {
		RGE_UNLOCK(sc);
		return;
	}

	/* Disable interrupts. */
	RGE_WRITE_4(sc, RGE_IMR, 0);

	if (!(sc->rge_flags & RGE_FLAG_MSI)) {
		if ((RGE_READ_4(sc, RGE_ISR) & sc->rge_intrs) == 0)
			goto done;
	}

	status = RGE_READ_4(sc, RGE_ISR);
	if (status)
		RGE_WRITE_4(sc, RGE_ISR, status);

	if (status & RGE_ISR_PCS_TIMEOUT)
		claimed = 1;

	rv = 0;
	if (status & sc->rge_intrs) {

		mtx_lock(&q->q_rx_mtx);
		rv |= rge_rxeof(q, &rx_mq);
		mtx_unlock(&q->q_rx_mtx);
		mtx_lock(&q->q_tx_mtx);
		rv |= rge_txeof(q);
		mtx_unlock(&q->q_tx_mtx);

		if (status & RGE_ISR_SYSTEM_ERR) {
			sc->sc_drv_stats.intr_system_err_cnt++;
			rge_init_locked(sc);
		}
		claimed = 1;
	}

	if (sc->rge_timerintr) {
		if (!rv) {
			/*
			 * Nothing needs to be processed, fallback
			 * to use TX/RX interrupts.
			 */
			rge_setup_intr(sc, RGE_IMTYPE_NONE);

			/*
			 * Recollect, mainly to avoid the possible
			 * race introduced by changing interrupt
			 * masks.
			 */
			mtx_lock(&q->q_rx_mtx);
			rge_rxeof(q, &rx_mq);
			mtx_unlock(&q->q_rx_mtx);
			mtx_lock(&q->q_tx_mtx);
			rge_txeof(q);
			mtx_unlock(&q->q_tx_mtx);
		} else
			RGE_WRITE_4(sc, RGE_TIMERCNT, 1);
	} else if (rv) {
		/*
		 * Assume that using simulated interrupt moderation
		 * (hardware timer based) could reduce the interrupt
		 * rate.
		 */
		rge_setup_intr(sc, RGE_IMTYPE_SIM);
	}

	RGE_WRITE_4(sc, RGE_IMR, sc->rge_intrs);

done:
	RGE_UNLOCK(sc);

	rge_rx_deliver(q, &rx_mq);

	(void) claimed;
}

static inline void
rge_tx_list_sync(struct rge_softc *sc, struct rge_queues *q,
    unsigned int idx, unsigned int len, int ops)
{
	bus_dmamap_sync(sc->sc_dmat_tx_desc, q->q_tx.rge_tx_list_map, ops);
}

/*
 * TSO: the chip segments a giant IPv4/TCP frame itself, given the TCP
 * header offset (cmdsts, repeated in every descriptor) and the MSS (extsts),
 * as Linux r8169/r8125 program giant send v4.  The stack has already stored
 * the length-free pseudo-header checksum in th_sum; the hardware fills the
 * IP and TCP checksums of each segment.  Returns non-zero (chain freed,
 * *mp NULL) when the headers cannot be made contiguous, or when the offset
 * or MSS does not fit the descriptor fields.
 */
static int
rge_tso_setup(struct mbuf **mp, uint32_t *tsoflags, uint32_t *cflags)
{
	struct ether_header *eh;
	struct ip *ip;
	struct mbuf *m = *mp;
	int ehlen, poff;

	ehlen = ETHER_HDR_LEN;
	if (m->m_len < ehlen + sizeof(struct ip) &&
	    (m = m_pullup(m, ehlen + sizeof(struct ip))) == NULL)
		goto fail;
	eh = mtod(m, struct ether_header *);
	if (eh->ether_type == htons(ETHERTYPE_VLAN)) {
		ehlen += ETHER_VLAN_ENCAP_LEN;
		if (m->m_len < ehlen + sizeof(struct ip) &&
		    (m = m_pullup(m, ehlen + sizeof(struct ip))) == NULL)
			goto fail;
	}
	ip = (struct ip *)(mtod(m, char *) + ehlen);
	poff = ehlen + (ip->ip_hl << 2);
	if (m->m_len < poff + sizeof(struct tcphdr) &&
	    (m = m_pullup(m, poff + sizeof(struct tcphdr))) == NULL)
		goto fail;
	*mp = m;
	if (poff > RGE_TDCMDSTS_GTTCPHO_MAX ||
	    m->m_pkthdr.tso_segsz > RGE_TDEXTSTS_MSS_MAX) {
		m_freem(m);
		goto fail;
	}
	*tsoflags = RGE_TDCMDSTS_GTSENV4 |
	    ((uint32_t)poff << RGE_TDCMDSTS_GTTCPHO_SHIFT);
	*cflags |= (uint32_t)m->m_pkthdr.tso_segsz << RGE_TDEXTSTS_MSS_SHIFT;
	return (0);
fail:
	*mp = NULL;
	return (1);
}

/**
 * @brief Queue the given mbuf at the given TX slot index for transmit.
 *
 * If the frame couldn't be enqueued then 0 is returned.
 * The caller needs to handle that and free/re-queue the mbuf as required.
 *
 * Note that this doesn't actually kick-start the transmit itself;
 * see rge_txstart() for the register to poke to start transmit.
 *
 * This must be called with the driver lock held.
 *
 * @param sc	driver softc
 * @param q	TX queue ring
 * @param mp	mbuf to enqueue, updated if defragmentation replaces it
 * @returns	if the mbuf is enqueued, it's consumed here and the number of
 * 		TX descriptors used is returned; if there's no space then 0 is
 *		returned; if the mbuf couldn't be mapped and the caller should
 *		free it then -1 is returned.
 */
static int
rge_encap(struct rge_softc *sc, struct rge_queues *q, struct mbuf **mp,
    int idx)
{
	struct rge_tx_desc *d = NULL;
	struct rge_txq *txq;
	bus_dmamap_t txmap;
	struct mbuf *m;
	uint32_t cmdsts, cflags = 0, tsoflags = 0;
	int cur, error, i;
	bus_dma_segment_t seg[RGE_TX_NSEGS];
	int nsegs;

	mtx_assert(&q->q_tx_mtx, MA_OWNED);

	m = *mp;
	txq = &q->q_tx.rge_txq[idx];
	txmap = txq->txq_dmamap;

	sc->sc_drv_stats.tx_encap_cnt++;

	if ((m->m_pkthdr.csum_flags & CSUM_IP_TSO) != 0) {
		if (rge_tso_setup(mp, &tsoflags, &cflags) != 0) {
			/* A failed pullup has already freed the chain. */
			sc->sc_drv_stats.tx_offload_tso_err++;
			return (-1);
		}
		m = *mp;
		sc->sc_drv_stats.tx_offload_tso_set++;
	}

	nsegs = RGE_TX_NSEGS;
	error = bus_dmamap_load_mbuf_sg(sc->sc_dmat_tx_buf, txmap, m,
	    seg, &nsegs, BUS_DMA_NOWAIT);

	switch (error) {
	case 0:
		break;
	case EFBIG: /* mbuf chain is too fragmented */
		sc->sc_drv_stats.tx_encap_refrag_cnt++;
		nsegs = RGE_TX_NSEGS;
		m = m_defrag(m, M_NOWAIT);
		if (m != NULL) {
			*mp = m;
			if (bus_dmamap_load_mbuf_sg(sc->sc_dmat_tx_buf, txmap,
			    m, seg, &nsegs, BUS_DMA_NOWAIT) == 0)
				break;
		}
		/* FALLTHROUGH */
	default:
		sc->sc_drv_stats.tx_encap_err_toofrag++;
		return (-1);
	}

	bus_dmamap_sync(sc->sc_dmat_tx_buf, txmap, BUS_DMASYNC_PREWRITE);

	/*
	 * Set RGE_TDEXTSTS_IPCSUM if any checksum offloading is requested.
	 * Otherwise, RGE_TDEXTSTS_TCPCSUM / RGE_TDEXTSTS_UDPCSUM does not
	 * take affect.
	 */
	if (tsoflags == 0 &&
	    (m->m_pkthdr.csum_flags & RGE_CSUM_FEATURES) != 0) {
		cflags |= RGE_TDEXTSTS_IPCSUM;
		sc->sc_drv_stats.tx_offload_ip_csum_set++;
		if (m->m_pkthdr.csum_flags & CSUM_TCP) {
			sc->sc_drv_stats.tx_offload_tcp_csum_set++;
			cflags |= RGE_TDEXTSTS_TCPCSUM;
		}
		if (m->m_pkthdr.csum_flags & CSUM_UDP) {
			sc->sc_drv_stats.tx_offload_udp_csum_set++;
			cflags |= RGE_TDEXTSTS_UDPCSUM;
		}
	}

	/* Set up hardware VLAN tagging */
	if (m->m_flags & M_VLANTAG) {
		sc->sc_drv_stats.tx_offload_vlan_tag_set++;
		cflags |= htons(m->m_pkthdr.ether_vtag) | RGE_TDEXTSTS_VTAG;
	}

	cur = idx;
	for (i = 1; i < nsegs; i++) {
		cur = RGE_NEXT_TX_DESC(cur);

		cmdsts = RGE_TDCMDSTS_OWN | tsoflags;
		cmdsts |= seg[i].ds_len;

		if (cur == RGE_TX_LIST_CNT - 1)
			cmdsts |= RGE_TDCMDSTS_EOR;
		if (i == nsegs - 1)
			cmdsts |= RGE_TDCMDSTS_EOF;

		/*
		 * Note: vendor driver puts wmb() after opts2/extsts,
		 * before opts1/status.
		 *
		 * See the other place I have this comment for more
		 * information.
		 */
		d = &q->q_tx.rge_tx_list[cur];
		d->rge_addr = htole64(seg[i].ds_addr);
		d->rge_extsts = htole32(cflags);
		wmb();
		d->rge_cmdsts = htole32(cmdsts);
	}

	/* Update info of TX queue and descriptors. */
	txq->txq_mbuf = m;
	txq->txq_descidx = cur;

	cmdsts = RGE_TDCMDSTS_SOF | tsoflags;
	cmdsts |= seg[0].ds_len;

	if (idx == RGE_TX_LIST_CNT - 1)
		cmdsts |= RGE_TDCMDSTS_EOR;
	if (nsegs == 1)
		cmdsts |= RGE_TDCMDSTS_EOF;

	/*
	 * Note: vendor driver puts wmb() after opts2/extsts,
	 * before opts1/status.
	 *
	 * It does this:
	 * - set rge_addr
	 * - set extsts
	 * - wmb
	 * - set status - at this point it's owned by the hardware
	 *
	 */
	d = &q->q_tx.rge_tx_list[idx];
	d->rge_addr = htole64(seg[0].ds_addr);
	d->rge_extsts = htole32(cflags);
	wmb();
	d->rge_cmdsts = htole32(cmdsts);
	wmb();

	if (cur >= idx) {
		rge_tx_list_sync(sc, q, idx, nsegs,
		    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	} else {
		rge_tx_list_sync(sc, q, idx, RGE_TX_LIST_CNT - idx,
		    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
		rge_tx_list_sync(sc, q, 0, cur + 1,
		    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	}

	/* Transfer ownership of packet to the chip. */
	cmdsts |= RGE_TDCMDSTS_OWN;
	rge_tx_list_sync(sc, q, idx, 1, BUS_DMASYNC_POSTWRITE);
	d->rge_cmdsts = htole32(cmdsts);
	rge_tx_list_sync(sc, q, idx, 1, BUS_DMASYNC_PREWRITE);
	wmb();

	return (nsegs);
}

static int
rge_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct rge_softc *sc = if_getsoftc(ifp);
	struct ifreq *ifr = (struct ifreq *)data;
	int error = 0;

	switch (cmd) {
	case SIOCSIFMTU:
		/* Note: no hardware reinit is required */
		if (ifr->ifr_mtu < ETHERMIN || ifr->ifr_mtu > RGE_JUMBO_MTU) {
			error = EINVAL;
			break;
		}
		if (if_getmtu(ifp) != ifr->ifr_mtu)
			if_setmtu(ifp, ifr->ifr_mtu);

		/* The 11-bit MSS field cannot describe larger frames. */
		if (if_getmtu(ifp) > RGE_TSO_MTU &&
		    (if_getcapenable(ifp) & IFCAP_TSO4) != 0) {
			if_setcapenablebit(ifp, 0, IFCAP_TSO4);
			if_sethwassistbits(ifp, 0, CSUM_IP_TSO);
		}

		VLAN_CAPABILITIES(ifp);
		break;

	case SIOCSIFFLAGS:
		RGE_LOCK(sc);
		if ((if_getflags(ifp) & IFF_UP) != 0) {
			if ((if_getdrvflags(ifp) & IFF_DRV_RUNNING) != 0) {
				if (((if_getflags(ifp) ^ sc->rge_if_flags)
				    & (IFF_PROMISC | IFF_ALLMULTI)) != 0)
					rge_iff_locked(sc);
			} else
				rge_init_locked(sc);
		} else {
			if ((if_getdrvflags(ifp) & IFF_DRV_RUNNING) != 0)
				rge_stop_locked(sc);
		}
		sc->rge_if_flags = if_getflags(ifp);
		RGE_UNLOCK(sc);
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		RGE_LOCK(sc);
		if ((if_getdrvflags(ifp) & IFF_DRV_RUNNING) != 0)
			rge_iff_locked(sc);
		RGE_UNLOCK(sc);
		break;
	case SIOCGIFMEDIA:
	case SIOCSIFMEDIA:
		error = ifmedia_ioctl(ifp, ifr, &sc->sc_media, cmd);
		break;
	case SIOCSIFCAP:
		{
			int mask;
			bool reinit = false;

			/* Get the mask of changed bits */
			mask = ifr->ifr_reqcap ^ if_getcapenable(ifp);

			/*
			 * Locked so we don't have a narrow window where frames
			 * are being processed with the updated flags but the
			 * hardware configuration hasn't yet changed.
			 */
			RGE_LOCK(sc);

			if ((mask & IFCAP_TXCSUM) != 0 &&
			    (if_getcapabilities(ifp) & IFCAP_TXCSUM) != 0) {
				if_togglecapenable(ifp, IFCAP_TXCSUM);
				if ((if_getcapenable(ifp) & IFCAP_TXCSUM) != 0)
					if_sethwassistbits(ifp, RGE_CSUM_FEATURES, 0);
				else
					if_sethwassistbits(ifp, 0, RGE_CSUM_FEATURES);
				reinit = 1;
			}

			if ((mask & IFCAP_VLAN_HWTAGGING) != 0 &&
			    (if_getcapabilities(ifp) & IFCAP_VLAN_HWTAGGING) != 0) {
				if_togglecapenable(ifp, IFCAP_VLAN_HWTAGGING);
				reinit = 1;
			}

			/* TSO is per-descriptor and needs no hardware reset. */
			if ((mask & IFCAP_TSO4) != 0 &&
			    (if_getcapabilities(ifp) & IFCAP_TSO4) != 0) {
				if_togglecapenable(ifp, IFCAP_TSO4);
				if ((if_getcapenable(ifp) & IFCAP_TSO4) != 0 &&
				    if_getmtu(ifp) <= RGE_TSO_MTU)
					if_sethwassistbits(ifp, CSUM_IP_TSO, 0);
				else {
					if_setcapenablebit(ifp, 0, IFCAP_TSO4);
					if_sethwassistbits(ifp, 0, CSUM_IP_TSO);
				}
			}

			/* LRO is software-only and needs no hardware reset. */
			if ((mask & IFCAP_LRO) != 0 &&
			    (if_getcapabilities(ifp) & IFCAP_LRO) != 0)
				if_togglecapenable(ifp, IFCAP_LRO);

			if ((mask & IFCAP_WOL_MAGIC) != 0 &&
			    (if_getcapabilities(ifp) & IFCAP_WOL_MAGIC) != 0)
				if_togglecapenable(ifp, IFCAP_WOL_MAGIC);

			if ((mask & IFCAP_RXCSUM) != 0 &&
			    (if_getcapabilities(ifp) & IFCAP_RXCSUM) != 0) {
				if_togglecapenable(ifp, IFCAP_RXCSUM);
				reinit = 1;
			}

			if (reinit && if_getdrvflags(ifp) & IFF_DRV_RUNNING) {
				if_setdrvflagbits(ifp, 0, IFF_DRV_RUNNING);
				rge_init_locked(sc);
			}

			RGE_UNLOCK(sc);
			VLAN_CAPABILITIES(ifp);
		}
		break;
	default:
		error = ether_ioctl(ifp, cmd, data);
		break;
	}

	return (error);
}

static void
rge_qflush_if(if_t ifp)
{
	struct rge_softc *sc = if_getsoftc(ifp);

	/* TODO: this should iterate over the TXQs */
	RGE_LOCK(sc);
	rge_txq_flush_mbufs(sc);
	RGE_UNLOCK(sc);
}

/* Schedule once, including while the worker is waiting for q_tx_mtx. */
static void
rge_schedule_tx_locked(struct rge_queues *q)
{
	mtx_assert(&q->q_tx_mtx, MA_OWNED);
	if (!q->q_tx_task_pending) {
		q->q_tx_task_pending = true;
		taskqueue_enqueue(q->q_sc->sc_tq, &q->q_tx_task);
	}
}

/*
 * Choose a transmit queue.  Steer by the flow hash when the stack set one
 * (a forwarded frame keeps the RSS flowid the receive path stamped, so a
 * flow stays on one TX ring and keeps its ordering) and by the current CPU
 * otherwise, so that forwarding threads on different cores drive different
 * TX rings and their per-ring locks instead of serialising on one.
 */
static struct rge_queues *
rge_select_txq(struct rge_softc *sc, struct mbuf *m)
{
	unsigned int i;

	if (sc->sc_ntxq <= 1)
		return (&sc->sc_queues[0]);
	if (M_HASHTYPE_GET(m) != M_HASHTYPE_NONE)
		i = m->m_pkthdr.flowid % sc->sc_ntxq;
	else
		i = curcpu % sc->sc_ntxq;
	return (&sc->sc_queues[i]);
}

/* Queue a frame, optionally submitting it in the caller context. */
static int
rge_transmit_if(if_t ifp, struct mbuf *m)
{
	struct rge_softc *sc = if_getsoftc(ifp);
	struct rge_queues *q;
	int ret;

	q = rge_select_txq(sc, m);

	mtx_lock(&q->q_tx_mtx);
	sc->sc_drv_stats.transmit_call_cnt++;
	if (sc->sc_stopped == true) {
		sc->sc_drv_stats.transmit_stopped_cnt++;
		mtx_unlock(&q->q_tx_mtx);
		m_freem(m);
		return (ENETDOWN);	/* TODO: better error? */
	}

	ret = mbufq_enqueue(&q->q_txq, m);
	if (ret != 0) {
		sc->sc_drv_stats.transmit_full_cnt++;
		if_inc_counter(ifp, IFCOUNTER_OQDROPS, 1);
		mtx_unlock(&q->q_tx_mtx);
		m_freem(m);
		return (ret);
	}
	/* A pending worker owns the drain; otherwise direct mode avoids a wakeup. */
	if (sc->sc_tx_direct && !q->q_tx_task_pending)
		rge_tx_drain_locked(q);
	else
		rge_schedule_tx_locked(q);
	sc->sc_drv_stats.transmit_queued_cnt++;
	mtx_unlock(&q->q_tx_mtx);

	return (0);
}

static void
rge_init_if(void *xsc)
{
	struct rge_softc *sc = xsc;

	RGE_LOCK(sc);
	rge_init_locked(sc);
	RGE_UNLOCK(sc);
}

/*
 * Program the RSS hash engine: the stack's RSS key (so the value matches
 * software hashing), an indirection table spreading the hash over the
 * receive queues, and the queue count.  With one queue this only makes the
 * chip write a Toeplitz hash and its type into each v3 Rx descriptor.
 */
static void
rge_setup_rss(struct rge_softc *sc)
{
	uint8_t key[RSS_KEYSIZE];
	uint32_t ctrl;
	int i;

	if (!sc->sc_rss_hash) {
		RGE_WRITE_4(sc, RGE_RSS_CTRL, 0);
		return;
	}
	rss_getkey(key);
	for (i = 0; i < RGE_RSS_KEY_LEN; i += 4)
		RGE_WRITE_4(sc, RGE_RSS_KEY + i, le32dec(key + i));
	for (i = 0; i < RGE_RSS_INDIR_ENTRIES; i += 4) {
		uint32_t entries = 0;
		int j;

		for (j = 0; j < 4; j++)
			entries |= ((i + j) % sc->sc_nqueues) << (j * 8);
		RGE_WRITE_4(sc, RGE_RSS_INDIR_TBL + i, entries);
	}
	ctrl = RGE_RSS_CTRL_TCP_IPV4 | RGE_RSS_CTRL_IPV4 | RGE_RSS_CTRL_IPV6 |
	    RGE_RSS_CTRL_IPV6_EXT | RGE_RSS_CTRL_TCP_IPV6 |
	    RGE_RSS_CTRL_TCP_IPV6_EXT | RGE_RSS_CTRL_UDP_IPV4 |
	    RGE_RSS_CTRL_UDP_IPV6 | RGE_RSS_CTRL_UDP_IPV6_EXT |
	    (7 << RGE_RSS_CTRL_HASH_MASK_SHIFT) |
	    ((fls(sc->sc_nqueues) - 1) << RGE_RSS_CTRL_CPU_NUM_SHIFT);
	RGE_WRITE_4(sc, RGE_RSS_CTRL, ctrl);
}

static void
rge_init_locked(struct rge_softc *sc)
{
	struct rge_queues *q = sc->sc_queues;
	uint32_t rxconf, val;
	int i, num_miti;

	RGE_ASSERT_LOCKED(sc);

	RGE_DPRINTF(sc, RGE_DEBUG_INIT, "%s: called!\n", __func__);

	/* Don't double-init the hardware */
	if ((if_getdrvflags(sc->sc_ifp) & IFF_DRV_RUNNING) != 0) {
		/*
		 * Note: I'm leaving this disabled by default; however
		 * I'm leaving it in here so I can figure out what's
		 * causing this to be initialised both from the ioctl
		 * API and if_init() API.
		 */
/*		RGE_PRINT_ERROR(sc, "%s: called whilst running?\n", __func__); */
		return;
	}

	/*
	 * Bring the hardware down so we know it's in a good known
	 * state before we bring it up in a good known state.
	 */
	rge_stop_locked(sc);

	/* Set MAC address. */
	rge_set_macaddr(sc, if_getlladdr(sc->sc_ifp));

	/* Initialize RX and TX descriptors lists. */
	for (i = 0; i < sc->sc_nqueues; i++) {
		mtx_lock(&q[i].q_rx_mtx);
		rge_rx_list_init(&q[i]);
		mtx_unlock(&q[i].q_rx_mtx);
	}
	for (i = 0; i < sc->sc_ntxq; i++) {
		mtx_lock(&q[i].q_tx_mtx);
		rge_tx_list_init(&q[i]);
		mtx_unlock(&q[i].q_tx_mtx);
	}

	if (rge_chipinit(sc)) {
		RGE_PRINT_ERROR(sc, "%s: ERROR: chip init fail!\n", __func__);
		return;
	}

	if (rge_phy_config(sc))
		return;

	RGE_SETBIT_1(sc, RGE_EECMD, RGE_EECMD_WRITECFG);

	RGE_CLRBIT_1(sc, 0xf1, 0x80);
	rge_disable_aspm_clkreq(sc);
	RGE_WRITE_2(sc, RGE_EEE_TXIDLE_TIMER,
	    RGE_JUMBO_MTU + ETHER_HDR_LEN + 32);

	/* Load the addresses of the RX and TX lists into the chip. */
	RGE_WRITE_4(sc, RGE_RXDESC_ADDR_LO,
	    RGE_ADDR_LO(q->q_rx.rge_rx_list_paddr));
	RGE_WRITE_4(sc, RGE_RXDESC_ADDR_HI,
	    RGE_ADDR_HI(q->q_rx.rge_rx_list_paddr));
	for (i = 1; i < sc->sc_nqueues; i++) {
		RGE_WRITE_4(sc, RGE_RXDESC_ADDR_Q_LO(i),
		    RGE_ADDR_LO(q[i].q_rx.rge_rx_list_paddr));
		RGE_WRITE_4(sc, RGE_RXDESC_ADDR_Q_HI(i),
		    RGE_ADDR_HI(q[i].q_rx.rge_rx_list_paddr));
	}
	RGE_WRITE_4(sc, RGE_TXDESC_ADDR_LO,
	    RGE_ADDR_LO(q->q_tx.rge_tx_list_paddr));
	RGE_WRITE_4(sc, RGE_TXDESC_ADDR_HI,
	    RGE_ADDR_HI(q->q_tx.rge_tx_list_paddr));
	for (i = 1; i < sc->sc_ntxq; i++) {
		RGE_WRITE_4(sc, RGE_TXDESC_ADDR_Q_LO(i),
		    RGE_ADDR_LO(q[i].q_tx.rge_tx_list_paddr));
		RGE_WRITE_4(sc, RGE_TXDESC_ADDR_Q_HI(i),
		    RGE_ADDR_HI(q[i].q_tx.rge_tx_list_paddr));
	}

	/* Set the initial RX and TX configurations. */
	if (sc->rge_type == MAC_R25)
		rxconf = RGE_RXCFG_CONFIG;
	else if (sc->rge_type == MAC_R25B)
		rxconf = RGE_RXCFG_CONFIG_8125B;
	else if (RGE_TYPE_R25D(sc))
		rxconf = RGE_RXCFG_CONFIG_8125D;
	else
		rxconf = RGE_RXCFG_CONFIG_8126;
	RGE_WRITE_4(sc, RGE_RXCFG, rxconf);
	RGE_WRITE_4(sc, RGE_TXCFG, RGE_TXCFG_CONFIG);

	val = rge_read_csi(sc, 0x70c) & ~0x3f000000;
	rge_write_csi(sc, 0x70c, val | 0x27000000);

	if (RGE_TYPE_R26(sc) || sc->rge_type == MAC_R27) {
		/* Disable L1 timeout. */
		val = rge_read_csi(sc, 0x890) & ~0x00000001;
		rge_write_csi(sc, 0x890, val);
	} else if (!RGE_TYPE_R25D(sc))
		RGE_WRITE_2(sc, 0x0382, 0x221b);

	rge_setup_rss(sc);

	val = RGE_READ_2(sc, RGE_RXQUEUE_CTRL) & ~0x001c;
	RGE_WRITE_2(sc, RGE_RXQUEUE_CTRL, val | (fls(sc->sc_nqueues) - 1) << 2);

	RGE_CLRBIT_1(sc, RGE_CFG1, RGE_CFG1_SPEED_DOWN);

	rge_write_mac_ocp(sc, 0xc140, 0xffff);
	rge_write_mac_ocp(sc, 0xc142, 0xffff);

	RGE_MAC_SETBIT(sc, 0xeb58, 0x0001);

	if (RGE_TYPE_R26(sc) || sc->rge_type == MAC_R27) {
		RGE_CLRBIT_1(sc, 0xd8, 0x02);
		if (sc->rge_type == MAC_R27) {
			RGE_CLRBIT_1(sc, 0x20e4, 0x04);
			RGE_MAC_CLRBIT(sc, 0xe00c, 0x1000);
			RGE_MAC_CLRBIT(sc, 0xc0c2, 0x0040);
		}
	}

	val = rge_read_mac_ocp(sc, 0xe614);
	val &= (sc->rge_type == MAC_R27) ? ~0x0f00 : ~0x0700;
	if (sc->rge_type == MAC_R25 || RGE_TYPE_R25D(sc))
		rge_write_mac_ocp(sc, 0xe614, val | 0x0300);
	else if (sc->rge_type == MAC_R25B)
		rge_write_mac_ocp(sc, 0xe614, val | 0x0200);
	else if (RGE_TYPE_R26(sc))
		rge_write_mac_ocp(sc, 0xe614, val | 0x0300);
	else
		rge_write_mac_ocp(sc, 0xe614, val | 0x0f00);

	/*
	 * Bits 10-11 are the number of *transmit* queues, encoded as
	 * log2(count) (Realtek's rtl8125_set_tx_q_num): 1 queue -> 0,
	 * 2 queues -> 1.
	 */
	val = rge_read_mac_ocp(sc, 0xe63e) & ~0x0c00;
	val |= ((fls(sc->sc_ntxq) - 1) & 0x03) << 10;
	rge_write_mac_ocp(sc, 0xe63e, val);

	val = rge_read_mac_ocp(sc, 0xe63e) & ~0x0030;
	rge_write_mac_ocp(sc, 0xe63e, val | 0x0020);

	RGE_MAC_CLRBIT(sc, 0xc0b4, 0x0001);
	RGE_MAC_SETBIT(sc, 0xc0b4, 0x0001);

	RGE_MAC_SETBIT(sc, 0xc0b4, 0x000c);

	val = rge_read_mac_ocp(sc, 0xeb6a) & ~0x00ff;
	rge_write_mac_ocp(sc, 0xeb6a, val | 0x0033);

	val = rge_read_mac_ocp(sc, 0xeb50) & ~0x03e0;
	rge_write_mac_ocp(sc, 0xeb50, val | 0x0040);

	RGE_MAC_CLRBIT(sc, 0xe056, 0x00f0);

	RGE_WRITE_1(sc, RGE_TDFNR, 0x10);

	RGE_MAC_CLRBIT(sc, 0xe040, 0x1000);

	val = rge_read_mac_ocp(sc, 0xea1c) & ~0x0003;
	rge_write_mac_ocp(sc, 0xea1c, val | 0x0001);

	if (RGE_TYPE_R25D(sc))
		rge_write_mac_ocp(sc, 0xe0c0, 0x4403);
	else
		rge_write_mac_ocp(sc, 0xe0c0, 0x4000);

	RGE_MAC_SETBIT(sc, 0xe052, 0x0060);
	RGE_MAC_CLRBIT(sc, 0xe052, 0x0088);

	val = rge_read_mac_ocp(sc, 0xd430) & ~0x0fff;
	rge_write_mac_ocp(sc, 0xd430, val | 0x045f);

	RGE_SETBIT_1(sc, RGE_DLLPR, RGE_DLLPR_PFM_EN | RGE_DLLPR_TX_10M_PS_EN);

	if (sc->rge_type == MAC_R25)
		RGE_SETBIT_1(sc, RGE_MCUCMD, 0x01);

	if (!RGE_TYPE_R25D(sc)) {
		/* Disable EEE plus. */
		RGE_MAC_CLRBIT(sc, 0xe080, 0x0002);
	}

	if (RGE_TYPE_R26(sc) || sc->rge_type == MAC_R27)
		RGE_MAC_CLRBIT(sc, 0xea1c, 0x0304);
	else
		RGE_MAC_CLRBIT(sc, 0xea1c, 0x0004);

	/* Clear tcam entries. */
	RGE_MAC_SETBIT(sc, 0xeb54, 0x0001);
	DELAY(1);
	RGE_MAC_CLRBIT(sc, 0xeb54, 0x0001);

	RGE_CLRBIT_2(sc, 0x1880, 0x0030);

	if (sc->rge_type == MAC_R27) {
		val = rge_read_mac_ocp(sc, 0xd40c) & ~0xe038;
		rge_write_phy_ocp(sc, 0xd40c, val | 0x8020);
	}

	/* Config interrupt type. */
	if (sc->rge_type == MAC_R27)
		RGE_CLRBIT_1(sc, RGE_INT_CFG0, RGE_INT_CFG0_AVOID_MISS_INTR);
	else if (sc->rge_type != MAC_R25)
		RGE_CLRBIT_1(sc, RGE_INT_CFG0, RGE_INT_CFG0_EN);

	/* Clear timer interrupts. */
	RGE_WRITE_4(sc, RGE_TIMERINT0, 0);
	RGE_WRITE_4(sc, RGE_TIMERINT1, 0);
	RGE_WRITE_4(sc, RGE_TIMERINT2, 0);
	RGE_WRITE_4(sc, RGE_TIMERINT3, 0);

	num_miti =
	    (sc->rge_type == MAC_R25B || RGE_TYPE_R26(sc)) ? 32 : 64;
	/* Clear interrupt moderation timer. */
	for (i = 0; i < num_miti; i++)
		RGE_WRITE_4(sc, RGE_INTMITI(i), 0);

	if (RGE_TYPE_R26(sc)) {
		RGE_CLRBIT_1(sc, RGE_INT_CFG0,
		    RGE_INT_CFG0_TIMEOUT_BYPASS | RGE_INT_CFG0_RDU_BYPASS_8126 |
		    RGE_INT_CFG0_MITIGATION_BYPASS);
		RGE_WRITE_2(sc, RGE_INT_CFG1, 0);
	}

	RGE_MAC_SETBIT(sc, 0xc0ac, 0x1f80);

	rge_write_mac_ocp(sc, 0xe098, 0xc302);

	RGE_MAC_CLRBIT(sc, 0xe032, 0x0003);
	val = rge_read_csi(sc, 0x98) & ~0x0000ff00;
	rge_write_csi(sc, 0x98, val);

	if (RGE_TYPE_R25D(sc)) {
		val = rge_read_mac_ocp(sc, 0xe092) & ~0x00ff;
		rge_write_mac_ocp(sc, 0xe092, val | 0x0008);
	} else
		RGE_MAC_CLRBIT(sc, 0xe092, 0x00ff);

	/* Enable/disable HW VLAN tagging based on enabled capability */
	if ((if_getcapenable(sc->sc_ifp) & IFCAP_VLAN_HWTAGGING) != 0)
		RGE_SETBIT_4(sc, RGE_RXCFG, RGE_RXCFG_VLANSTRIP);
	else
		RGE_CLRBIT_4(sc, RGE_RXCFG, RGE_RXCFG_VLANSTRIP);

	/* Enable/disable RX checksum based on enabled capability */
	if ((if_getcapenable(sc->sc_ifp) & IFCAP_RXCSUM) != 0)
		RGE_SETBIT_2(sc, RGE_CPLUSCMD, RGE_CPLUSCMD_RXCSUM);
	else
		RGE_CLRBIT_2(sc, RGE_CPLUSCMD, RGE_CPLUSCMD_RXCSUM);
	RGE_READ_2(sc, RGE_CPLUSCMD);

	/* Set Maximum frame size. */
	RGE_WRITE_2(sc, RGE_RXMAXSIZE, RGE_JUMBO_FRAMELEN);

	/* Disable RXDV gate. */
	RGE_CLRBIT_1(sc, RGE_PPSW, 0x08);
	DELAY(2000);

	/* Program promiscuous mode and multicast filters. */
	rge_iff_locked(sc);

	if (sc->rge_type == MAC_R27)
		RGE_CLRBIT_1(sc, RGE_RADMFIFO_PROTECT, 0x2001);

	rge_disable_aspm_clkreq(sc);

	RGE_CLRBIT_1(sc, RGE_EECMD, RGE_EECMD_WRITECFG);
	DELAY(10);

	rge_ifmedia_upd(sc->sc_ifp);

	/* Enable transmit and receive. */
	RGE_WRITE_1(sc, RGE_CMD, RGE_CMD_TXENB | RGE_CMD_RXENB);

	/* Enable interrupts. */
	if (sc->sc_msix) {
		uint32_t mask;

		/*
		 * v2 interrupt space: legacy IMR/ISR stay quiet, the per-bit
		 * set/clear registers arm the MSI-X vectors.  Receive
		 * mitigation is the per-queue timer byte (the handler drains
		 * the ring, so it only paces wake-ups).  Transmit completion
		 * is moderated much harder: with the legacy timer it was
		 * reclaimed at ~8 kHz, whereas an unmoderated completion
		 * vector fires per few frames and fights the transmit path
		 * for the driver lock.
		 */
		RGE_WRITE_4(sc, RGE_IMR, 0);
		RGE_WRITE_4(sc, RGE_IMR_V2_CLR, 0xffffffff);
		RGE_WRITE_4(sc, RGE_ISR_V2, 0xffffffff);
		for (i = 0; i < sc->sc_nqueues; i++)
			RGE_WRITE_1(sc, RGE_INTMITI_V2_RX(i), sc->sc_rx_miti);
		for (i = 0; i < sc->sc_ntxq; i++)
			RGE_WRITE_1(sc, RGE_INTMITI_V2_TX(i), sc->sc_tx_miti);
		RGE_SETBIT_1(sc, RGE_INT_CFG0, RGE_INT_CFG0_EN);
		mask = RGE_ISR_V2_LINKCHG;
		for (i = 0; i < sc->sc_ntxq; i++)
			mask |= RGE_ISR_V2_TOK(i);
		for (i = 0; i < sc->sc_nqueues; i++)
			mask |= RGE_ISR_V2_ROK(i);
		RGE_WRITE_4(sc, RGE_IMR_V2_SET, mask);
	} else
		rge_setup_intr(sc, RGE_IMTYPE_SIM);

	if_setdrvflagbits(sc->sc_ifp, IFF_DRV_RUNNING, 0);
	if_setdrvflagbits(sc->sc_ifp, 0, IFF_DRV_OACTIVE);

	callout_reset(&sc->sc_timeout, hz, rge_tick, sc);

	RGE_DPRINTF(sc, RGE_DEBUG_INIT, "%s: init completed!\n", __func__);

	/* Unblock transmit when we release the lock */
	sc->sc_stopped = false;
}

/*
 * @brief Stop the adapter and free any mbufs allocated to the RX and TX lists.
 *
 * Must be called with the driver lock held.
 */
void
rge_stop_locked(struct rge_softc *sc)
{
	struct rge_queues *q = sc->sc_queues;
	int i, nq;

	RGE_ASSERT_LOCKED(sc);

	RGE_DPRINTF(sc, RGE_DEBUG_INIT, "%s: called!\n", __func__);

	callout_stop(&sc->sc_timeout);

	/* Stop pending TX submissions */
	sc->sc_stopped = true;

	if_setdrvflagbits(sc->sc_ifp, 0, IFF_DRV_RUNNING);
	sc->rge_timerintr = 0;
	sc->sc_watchdog = 0;

	if (sc->sc_msix) {
		RGE_WRITE_4(sc, RGE_IMR_V2_CLR, 0xffffffff);
		RGE_WRITE_4(sc, RGE_ISR_V2, 0xffffffff);
	}

	RGE_CLRBIT_4(sc, RGE_RXCFG, RGE_RXCFG_ALLPHYS | RGE_RXCFG_INDIV |
	    RGE_RXCFG_MULTI | RGE_RXCFG_BROAD | RGE_RXCFG_RUNT |
	    RGE_RXCFG_ERRPKT);

	rge_hw_reset(sc);

	RGE_MAC_CLRBIT(sc, 0xc0ac, 0x1f80);

	if_setdrvflagbits(sc->sc_ifp, 0, IFF_DRV_OACTIVE);

	for (nq = 0; nq < sc->sc_nqueues; nq++) {
		mtx_lock(&q[nq].q_rx_mtx);
		if (q[nq].q_rx.rge_head != NULL) {
			m_freem(q[nq].q_rx.rge_head);
			q[nq].q_rx.rge_head = NULL;
			q[nq].q_rx.rge_tail = &q[nq].q_rx.rge_head;
		}
		/* Free the RX list buffers. */
		for (i = 0; i < RGE_RX_LIST_CNT; i++) {
			if (q[nq].q_rx.rge_rxq[i].rxq_mbuf != NULL) {
				bus_dmamap_unload(sc->sc_dmat_rx_buf,
				    q[nq].q_rx.rge_rxq[i].rxq_dmamap);
				m_freem(q[nq].q_rx.rge_rxq[i].rxq_mbuf);
				q[nq].q_rx.rge_rxq[i].rxq_mbuf = NULL;
			}
		}
		mtx_unlock(&q[nq].q_rx_mtx);
	}

	/* Free the TX list buffers for every TX queue. */
	for (nq = 0; nq < sc->sc_ntxq; nq++) {
		mtx_lock(&q[nq].q_tx_mtx);
		for (i = 0; i < RGE_TX_LIST_CNT; i++) {
			if (q[nq].q_tx.rge_txq[i].txq_mbuf != NULL) {
				bus_dmamap_unload(sc->sc_dmat_tx_buf,
				    q[nq].q_tx.rge_txq[i].txq_dmamap);
				m_freem(q[nq].q_tx.rge_txq[i].txq_mbuf);
				q[nq].q_tx.rge_txq[i].txq_mbuf = NULL;
			}
		}
		mtx_unlock(&q[nq].q_tx_mtx);
	}

	/* Free pending TX frames */
	/* TODO: should be per TX queue */
	rge_txq_flush_mbufs(sc);
}

/*
 * Set media options.
 */
static int
rge_ifmedia_upd(if_t ifp)
{
	struct rge_softc *sc = if_getsoftc(ifp);
	struct ifmedia *ifm = &sc->sc_media;
	int anar, gig, val;

	if (IFM_TYPE(ifm->ifm_media) != IFM_ETHER)
		return (EINVAL);

	/* Disable Gigabit Lite. */
	RGE_PHY_CLRBIT(sc, 0xa428, 0x0200);
	RGE_PHY_CLRBIT(sc, 0xa5ea, 0x0001);
	if (RGE_TYPE_R26(sc) || sc->rge_type == MAC_R27)
		RGE_PHY_CLRBIT(sc, 0xa5ea, 0x0007);

	val = rge_read_phy_ocp(sc, 0xa5d4);
	switch (sc->rge_type) {
	case MAC_R27:
		val &= ~RGE_ADV_10000TFDX;
		/* fallthrough */
	case MAC_R26_1:
	case MAC_R26_2:
		val &= ~RGE_ADV_5000TFDX;
		/* fallthrough */
	default:
		val &= ~RGE_ADV_2500TFDX;
		break;
	}

	anar = ANAR_TX_FD | ANAR_TX | ANAR_10_FD | ANAR_10;
	gig = GTCR_ADV_1000TFDX | GTCR_ADV_1000THDX;

	switch (IFM_SUBTYPE(ifm->ifm_media)) {
	case IFM_AUTO:
		val |= RGE_ADV_2500TFDX;
		if (RGE_TYPE_R26(sc))
			val |= RGE_ADV_5000TFDX;
		else if (sc->rge_type == MAC_R27)
			val |= RGE_ADV_5000TFDX | RGE_ADV_10000TFDX;
		break;
	case IFM_10G_T:
		val |= RGE_ADV_10000TFDX;
		if_setbaudrate(ifp, IF_Gbps(10));
		break;
	case IFM_5000_T:
		val |= RGE_ADV_5000TFDX;
		if_setbaudrate(ifp, IF_Gbps(5));
		break;
	case IFM_2500_T:
		val |= RGE_ADV_2500TFDX;
		if_setbaudrate(ifp, IF_Mbps(2500));
		break;
	case IFM_1000_T:
		if_setbaudrate(ifp, IF_Gbps(1));
		break;
	case IFM_100_TX:
		gig = rge_read_phy(sc, 0, MII_100T2CR) &
		    ~(GTCR_ADV_1000TFDX | GTCR_ADV_1000THDX);
		anar = ((ifm->ifm_media & IFM_GMASK) == IFM_FDX) ?
		    ANAR_TX | ANAR_TX_FD | ANAR_10_FD | ANAR_10 :
		    ANAR_TX | ANAR_10_FD | ANAR_10;
		if_setbaudrate(ifp, IF_Mbps(100));
		break;
	case IFM_10_T:
		gig = rge_read_phy(sc, 0, MII_100T2CR) &
		    ~(GTCR_ADV_1000TFDX | GTCR_ADV_1000THDX);
		anar = ((ifm->ifm_media & IFM_GMASK) == IFM_FDX) ?
		    ANAR_10_FD | ANAR_10 : ANAR_10;
		if_setbaudrate(ifp, IF_Mbps(10));
		break;
	default:
		RGE_PRINT_ERROR(sc, "unsupported media type\n");
		return (EINVAL);
	}

	rge_write_phy(sc, 0, MII_ANAR, anar | ANAR_PAUSE_ASYM | ANAR_FC);
	rge_write_phy(sc, 0, MII_100T2CR, gig);
	rge_write_phy_ocp(sc, 0xa5d4, val);
	rge_write_phy(sc, 0, MII_BMCR, BMCR_RESET | BMCR_AUTOEN |
	    BMCR_STARTNEG);

	return (0);
}

/*
 * Report current media status.
 */
static void
rge_ifmedia_sts(if_t ifp, struct ifmediareq *ifmr)
{
	struct rge_softc *sc = if_getsoftc(ifp);
	uint16_t status = 0;

	ifmr->ifm_status = IFM_AVALID;
	ifmr->ifm_active = IFM_ETHER;

	if (rge_get_link_status(sc)) {
		ifmr->ifm_status |= IFM_ACTIVE;

		status = RGE_READ_2(sc, RGE_PHYSTAT);
		if ((status & RGE_PHYSTAT_FDX) ||
		    (status & (RGE_PHYSTAT_1000MBPS | RGE_PHYSTAT_2500MBPS |
		    RGE_PHYSTAT_5000MBPS | RGE_PHYSTAT_10000MBPS)))
			ifmr->ifm_active |= IFM_FDX;
		else
			ifmr->ifm_active |= IFM_HDX;

		if (status & RGE_PHYSTAT_10MBPS)
			ifmr->ifm_active |= IFM_10_T;
		else if (status & RGE_PHYSTAT_100MBPS)
			ifmr->ifm_active |= IFM_100_TX;
		else if (status & RGE_PHYSTAT_1000MBPS)
			ifmr->ifm_active |= IFM_1000_T;
		else if (status & RGE_PHYSTAT_2500MBPS)
			ifmr->ifm_active |= IFM_2500_T;
		else if (status & RGE_PHYSTAT_5000MBPS)
			ifmr->ifm_active |= IFM_5000_T;
		else if (status & RGE_PHYSTAT_10000MBPS)
			ifmr->ifm_active |= IFM_10G_T;
	}
}

/**
 * @brief callback to load/populate a single physical address
 */
static void
rge_dma_load_cb(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{
	bus_addr_t *paddr = (bus_addr_t *) arg;

	*paddr = 0;

	if (error) {
		printf("%s: error! (%d)\n", __func__, error);
		*paddr = 0;
		return;
	}

	if (nsegs != 1) {
		printf("%s: too many segs (got %d)\n", __func__, nsegs);
		*paddr = 0;
		return;
	}

	*paddr = segs[0].ds_addr;
}

/**
 * @brief Allocate memory for RX/TX rings.
 *
 * Called with the driver lock NOT held.
 */
static int
rge_allocmem(struct rge_softc *sc)
{
	struct rge_queues *q = sc->sc_queues;
	int error;
	int i, nq;

	RGE_ASSERT_UNLOCKED(sc);

	/* Allocate DMA'able memory for each TX ring. */
	for (nq = 0; nq < sc->sc_ntxq; nq++) {
	q = &sc->sc_queues[nq];
	error = bus_dmamem_alloc(sc->sc_dmat_tx_desc,
	    (void **) &q->q_tx.rge_tx_list,
	    BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT,
	    &q->q_tx.rge_tx_list_map);
	if (error) {
		RGE_PRINT_ERROR(sc, "%s: error (alloc tx_list.map) (%d)\n",
		    __func__, error);
		goto error;
	}

	RGE_DPRINTF(sc, RGE_DEBUG_SETUP, "%s: tx_list=%p\n", __func__,
	    q->q_tx.rge_tx_list);
	RGE_DPRINTF(sc, RGE_DEBUG_SETUP, "%s: tx_list_map=%p\n", __func__,
	    q->q_tx.rge_tx_list_map);

	/* Load the map for the TX ring. */
	error = bus_dmamap_load(sc->sc_dmat_tx_desc,
	    q->q_tx.rge_tx_list_map,
	    q->q_tx.rge_tx_list,
	    RGE_TX_LIST_SZ,
	    rge_dma_load_cb,
	    (void *) &q->q_tx.rge_tx_list_paddr,
	    BUS_DMA_NOWAIT);

	if ((error != 0) || (q->q_tx.rge_tx_list_paddr == 0)) {
		RGE_PRINT_ERROR(sc, "%s: error (load tx_list.map) (%d)\n",
		    __func__, error);
		goto error;
	}

	/* Create DMA maps for TX buffers. */
	for (i = 0; i < RGE_TX_LIST_CNT; i++) {
		error = bus_dmamap_create(sc->sc_dmat_tx_buf,
		    0, &q->q_tx.rge_txq[i].txq_dmamap);
		if (error) {
			RGE_PRINT_ERROR(sc,
			    "can't create DMA map for TX (%d)\n", error);
			goto error;
		}
	}
	}	/* per-TX-queue loop */

	for (nq = 0; nq < sc->sc_nqueues; nq++) {
		q = &sc->sc_queues[nq];
		/* Allocate DMA'able memory for the RX ring. */
		error = bus_dmamem_alloc(sc->sc_dmat_rx_desc,
		    (void **) &q->q_rx.rge_rx_list,
		    BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT,
		    &q->q_rx.rge_rx_list_map);
		if (error) {
			RGE_PRINT_ERROR(sc, "%s: error (alloc rx_list.map) (%d)\n",
			    __func__, error);
			goto error;
		}

		RGE_DPRINTF(sc, RGE_DEBUG_INIT, "%s: rx_list=%p\n", __func__,
		    q->q_rx.rge_rx_list);
		RGE_DPRINTF(sc, RGE_DEBUG_INIT, "%s: rx_list_map=%p\n", __func__,
		    q->q_rx.rge_rx_list_map);

		/* Load the map for the RX ring. */
		error = bus_dmamap_load(sc->sc_dmat_rx_desc,
		    q->q_rx.rge_rx_list_map,
		    q->q_rx.rge_rx_list,
		    RGE_RX_LIST_SZ,
		    rge_dma_load_cb,
		    (void *) &q->q_rx.rge_rx_list_paddr,
		    BUS_DMA_NOWAIT);

		if ((error != 0) || (q->q_rx.rge_rx_list_paddr == 0)) {
			RGE_PRINT_ERROR(sc, "%s: error (load rx_list.map) (%d)\n",
			    __func__, error);
			goto error;
		}

		/* Create DMA maps for RX buffers. */
		for (i = 0; i < RGE_RX_LIST_CNT; i++) {
			error = bus_dmamap_create(sc->sc_dmat_rx_buf,
			    0, &q->q_rx.rge_rxq[i].rxq_dmamap);
			if (error) {
				RGE_PRINT_ERROR(sc,
				    "can't create DMA map for RX (%d)\n", error);
				goto error;
			}
		}
	}

	return (0);
error:

	rge_freemem(sc);

	return (error);
}

/**
 * @brief Allocate memory for MAC stats.
 *
 * Called with the driver lock NOT held.
 */
static int
rge_alloc_stats_mem(struct rge_softc *sc)
{
	struct rge_mac_stats *ss = &sc->sc_mac_stats;
	int error;

	RGE_ASSERT_UNLOCKED(sc);

	/* Allocate DMA'able memory for the stats buffer. */
	error = bus_dmamem_alloc(sc->sc_dmat_stats_buf,
	    (void **) &ss->stats, BUS_DMA_WAITOK | BUS_DMA_ZERO,
	    &ss->map);
	if (error) {
		RGE_PRINT_ERROR(sc, "%s: error (alloc stats) (%d)\n",
		    __func__, error);
		goto error;
	}

	RGE_DPRINTF(sc, RGE_DEBUG_SETUP, "%s: stats=%p\n", __func__, ss->stats);
	RGE_DPRINTF(sc, RGE_DEBUG_SETUP, "%s: map=%p\n", __func__, ss->map);

	/* Load the map for the TX ring. */
	error = bus_dmamap_load(sc->sc_dmat_stats_buf,
	    ss->map,
	    ss->stats,
	    RGE_STATS_BUF_SIZE,
	    rge_dma_load_cb,
	    (void *) &ss->paddr,
	    BUS_DMA_NOWAIT);

	if ((error != 0) || (ss->paddr == 0)) {
		RGE_PRINT_ERROR(sc, "%s: error (load stats.map) (%d)\n",
		    __func__, error);
		if (error == 0)
			error = ENXIO;
		goto error;
	}

	return (0);

error:
	rge_free_stats_mem(sc);

	return (error);
}


/**
 * @brief Free the TX/RX DMA buffers and mbufs.
 *
 * Called with the driver lock NOT held.
 */
static int
rge_freemem(struct rge_softc *sc)
{
	struct rge_queues *q = sc->sc_queues;
	int i, nq;

	RGE_ASSERT_UNLOCKED(sc);

	/* TX buf + desc, per TX queue */
	for (nq = 0; nq < sc->sc_ntxq; nq++) {
	q = &sc->sc_queues[nq];
	for (i = 0; i < RGE_TX_LIST_CNT; i++) {
		struct rge_txq *tx = &q->q_tx.rge_txq[i];

		/* unmap/free mbuf if it's still alloc'ed and mapped */
		if (tx->txq_mbuf != NULL) {
			static bool do_warning = false;

			if (do_warning == false) {
				RGE_PRINT_ERROR(sc,
				    "%s: TX mbuf should've been freed!\n",
				    __func__);
				do_warning = true;
			}
			if (tx->txq_dmamap != NULL) {
				bus_dmamap_sync(sc->sc_dmat_tx_buf,
				    tx->txq_dmamap, BUS_DMASYNC_POSTREAD);
				bus_dmamap_unload(sc->sc_dmat_tx_buf,
				    tx->txq_dmamap);
			}
			m_freem(tx->txq_mbuf);
			tx->txq_mbuf = NULL;
		}

		/* Destroy the dmamap if it's allocated */
		if (tx->txq_dmamap != NULL) {
			bus_dmamap_destroy(sc->sc_dmat_tx_buf, tx->txq_dmamap);
			tx->txq_dmamap = NULL;
		}
	}

	/* TX desc */
	if (q->q_tx.rge_tx_list != NULL) {
		bus_dmamap_unload(sc->sc_dmat_tx_desc, q->q_tx.rge_tx_list_map);
		bus_dmamem_free(sc->sc_dmat_tx_desc, q->q_tx.rge_tx_list,
		    q->q_tx.rge_tx_list_map);
	}
	memset(&q->q_tx, 0, sizeof(q->q_tx));
	}	/* per-TX-queue loop */

	for (nq = 0; nq < sc->sc_nqueues; nq++) {
		q = &sc->sc_queues[nq];
		/* RX buf */
		for (i = 0; i < RGE_RX_LIST_CNT; i++) {
			struct rge_rxq *rx = &q->q_rx.rge_rxq[i];

			/* unmap/free mbuf if it's still alloc'ed and mapped */
			if (rx->rxq_mbuf != NULL) {
				if (rx->rxq_dmamap != NULL) {
					bus_dmamap_sync(sc->sc_dmat_rx_buf,
					    rx->rxq_dmamap, BUS_DMASYNC_POSTREAD);
					bus_dmamap_unload(sc->sc_dmat_rx_buf,
					    rx->rxq_dmamap);
				}
				m_freem(rx->rxq_mbuf);
				rx->rxq_mbuf = NULL;
			}

			/* Destroy the dmamap if it's allocated */
			if (rx->rxq_dmamap != NULL) {
				bus_dmamap_destroy(sc->sc_dmat_rx_buf, rx->rxq_dmamap);
				rx->rxq_dmamap = NULL;
			}
		}

		/* RX desc */
		if (q->q_rx.rge_rx_list != NULL) {
			bus_dmamap_unload(sc->sc_dmat_rx_desc, q->q_rx.rge_rx_list_map);
			bus_dmamem_free(sc->sc_dmat_rx_desc, q->q_rx.rge_rx_list,
			    q->q_rx.rge_rx_list_map);
		}
		memset(&q->q_rx, 0, sizeof(q->q_rx));
	}

	return (0);
}

/**
 * @brief Free the stats memory.
 *
 * Called with the driver lock NOT held.
 */
static int
rge_free_stats_mem(struct rge_softc *sc)
{
	struct rge_mac_stats *ss = &sc->sc_mac_stats;

	RGE_ASSERT_UNLOCKED(sc);

	if (ss->stats != NULL) {
		bus_dmamap_unload(sc->sc_dmat_stats_buf, ss->map);
		bus_dmamem_free(sc->sc_dmat_stats_buf, ss->stats, ss->map);
	}
	memset(ss, 0, sizeof(*ss));
	return (0);
}

static uint32_t
rx_ring_space(struct rge_queues *q)
{
	uint32_t prod, cons;
	uint32_t ret;

	RGE_ASSERT_LOCKED(q->q_sc);

	prod = q->q_rx.rge_rxq_prodidx;
	cons = q->q_rx.rge_rxq_considx;

	ret = (cons + RGE_RX_LIST_CNT - prod - 1) % RGE_RX_LIST_CNT + 1;

	if (ret > RGE_RX_LIST_CNT)
		return RGE_RX_LIST_CNT;

	return (ret);
}

/*
 * Initialize the RX descriptor and attach an mbuf cluster at the given offset.
 *
 * Note: this relies on the rxr ring buffer abstraction to not
 * over-fill the RX ring.  For FreeBSD we'll need to use the
 * prod/cons RX indexes to know how much RX ring space to
 * populate.
 *
 * This routine will increment the producer index if successful.
 *
 * This must be called with the driver lock held.
 */
static int
rge_newbuf(struct rge_queues *q)
{
	struct rge_softc *sc = q->q_sc;
	struct mbuf *m;
	struct rge_rx_desc *r;
	struct rge_rxq *rxq;
	bus_dmamap_t rxmap;
	bus_dma_segment_t seg[1];
	uint32_t cmdsts;
	int nsegs;
	uint32_t idx;

	mtx_assert(&q->q_rx_mtx, MA_OWNED);

	/*
	 * Verify we have enough space in the ring; error out
	 * if we do not.
	 */
	if (rx_ring_space(q) == 0)
		return (ENOBUFS);

	idx = q->q_rx.rge_rxq_prodidx;
	rxq = &q->q_rx.rge_rxq[idx];
	rxmap = rxq->rxq_dmamap;

	/*
	 * If we already have an mbuf here then something messed up;
	 * exit out as the hardware may be DMAing to it.
	 */
	if (rxq->rxq_mbuf != NULL) {
		RGE_PRINT_ERROR(sc,
		    "%s: RX ring slot %d already has an mbuf?\n", __func__,
		    idx);
		return (ENOBUFS);
	}

	/* Allocate single buffer backed mbuf of MCLBYTES */
	m = m_getcl(M_NOWAIT, MT_DATA, M_PKTHDR);
	if (m == NULL)
		return (ENOBUFS);

	m->m_len = m->m_pkthdr.len = MCLBYTES;

	nsegs = 1;
	if (bus_dmamap_load_mbuf_sg(sc->sc_dmat_rx_buf, rxmap, m, seg, &nsegs,
	    BUS_DMA_NOWAIT)) {
		m_freem(m);
		return (ENOBUFS);
	}

	/*
	 * Make sure any changes made to the buffer have been flushed to host
	 * memory.
	 */
	bus_dmamap_sync(sc->sc_dmat_rx_buf, rxmap,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	/*
	 * Map the segment into RX descriptors.  Note that this
	 * only currently supports a single segment per mbuf;
	 * the call to load_mbuf_sg above specified a single segment.
	 */
	r = &q->q_rx.rge_rx_list[idx];

	rxq->rxq_mbuf = m;

	cmdsts = seg[0].ds_len; /* XXX how big is this field in the descriptor? */
	if (idx == RGE_RX_LIST_CNT - 1)
		cmdsts |= RGE_RDCMDSTS_EOR;

	/*
	 * Configure the DMA pointer and config, but don't hand
	 * it yet to the hardware.
	 */
	r->hi_qword1.rx_qword4.rge_cmdsts = htole32(cmdsts);
	r->hi_qword1.rx_qword4.rge_extsts = htole32(0);
	r->hi_qword0.rge_addr = htole64(seg[0].ds_addr);
	wmb();

	/*
	 * Mark the specific descriptor slot as "this descriptor is now
	 * owned by the hardware", which when the hardware next sees
	 * this, it'll continue RX DMA.
	 */
	cmdsts |= RGE_RDCMDSTS_OWN;
	r->hi_qword1.rx_qword4.rge_cmdsts = htole32(cmdsts);
	wmb();

	/*
	 * At this point the hope is the whole ring is now updated and
	 * consistent; if the hardware was waiting for a descriptor to be
	 * ready to write into then it should be ready here.
	 */

	RGE_DPRINTF(sc, RGE_DEBUG_RECV_DESC,
	    "%s: [%d]: m=%p, m_data=%p, m_len=%ju, phys=0x%jx len %ju, "
	    "desc=0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n",
	    __func__,
	    idx,
	    m,
	    m->m_data,
	    (uintmax_t) m->m_len,
	    (uintmax_t) seg[0].ds_addr,
	    (uintmax_t) seg[0].ds_len,
	    ((uint32_t *) r)[0],
	    ((uint32_t *) r)[1],
	    ((uint32_t *) r)[2],
	    ((uint32_t *) r)[3],
	    ((uint32_t *) r)[4],
	    ((uint32_t *) r)[5],
	    ((uint32_t *) r)[6],
	    ((uint32_t *) r)[7]);

	q->q_rx.rge_rxq_prodidx = RGE_NEXT_RX_DESC(idx);

	return (0);
}

static void
rge_rx_list_init(struct rge_queues *q)
{
	memset(q->q_rx.rge_rx_list, 0, RGE_RX_LIST_SZ);

	mtx_assert(&q->q_rx_mtx, MA_OWNED);

	q->q_rx.rge_rxq_prodidx = q->q_rx.rge_rxq_considx = 0;
	q->q_rx.rge_head = NULL;
	q->q_rx.rge_tail = &q->q_rx.rge_head;

	RGE_DPRINTF(q->q_sc, RGE_DEBUG_SETUP, "%s: rx_list=%p\n", __func__,
	    q->q_rx.rge_rx_list);

	rge_fill_rx_ring(q);
}

/**
 * @brief Fill / refill the RX ring as needed.
 *
 * Refill the RX ring with one less than the total descriptors needed.
 * This makes the check in rge_rxeof() easier - it can just check
 * descriptors from cons -> prod and bail once it hits prod.
 * If the whole ring is filled then cons == prod, and that shortcut
 * fails.
 *
 * This must be called with the driver lock held.
 */
static void
rge_fill_rx_ring(struct rge_queues *q)
{
	struct rge_softc *sc = q->q_sc;
	uint32_t count, i, prod, cons;

	mtx_assert(&q->q_rx_mtx, MA_OWNED);

	prod = q->q_rx.rge_rxq_prodidx;
	cons = q->q_rx.rge_rxq_considx;
	count = rx_ring_space(q);

	/* Fill to count-1; bail if we don't have the space */
	if (count <= 1)
		return;
	count--;

	RGE_DPRINTF(sc, RGE_DEBUG_RECV_DESC, "%s: prod=%u, cons=%u, space=%u\n",
	  __func__, prod, cons, count);

	/* Make sure device->host changes are visible */
	bus_dmamap_sync(sc->sc_dmat_rx_desc, q->q_rx.rge_rx_list_map,
	    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

	for (i = 0; i < count; i++) {
		if (rge_newbuf(q))
			break;
	}

	/* Make changes visible to the device */
	bus_dmamap_sync(sc->sc_dmat_rx_desc, q->q_rx.rge_rx_list_map,
	    BUS_DMASYNC_PREWRITE);
}

static void
rge_tx_list_init(struct rge_queues *q)
{
	struct rge_softc *sc = q->q_sc;
	struct rge_tx_desc *d;
	int i;

	RGE_ASSERT_LOCKED(q->q_sc);

	memset(q->q_tx.rge_tx_list, 0, RGE_TX_LIST_SZ);

	for (i = 0; i < RGE_TX_LIST_CNT; i++)
		q->q_tx.rge_txq[i].txq_mbuf = NULL;

	d = &q->q_tx.rge_tx_list[RGE_TX_LIST_CNT - 1];
	d->rge_cmdsts = htole32(RGE_TDCMDSTS_EOR);

	bus_dmamap_sync(sc->sc_dmat_tx_desc, q->q_tx.rge_tx_list_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	wmb();

	q->q_tx.rge_txq_prodidx = q->q_tx.rge_txq_considx = 0;

	RGE_DPRINTF(sc, RGE_DEBUG_SETUP, "%s: rx_list=%p\n", __func__,
	    q->q_tx.rge_tx_list);
}

int
rge_rxeof(struct rge_queues *q, struct mbufq *mq)
{
	struct rge_softc *sc = q->q_sc;
	struct mbuf *m;
	struct rge_rx_desc *cur_rx;
	struct rge_rxq *rxq;
	uint32_t rxstat, extsts;
	int i, mlen, rx = 0;
	int cons, prod;
	int maxpkt;
	bool check_hwcsum;

	maxpkt = sc->sc_rx_process_limit;
	check_hwcsum = ((if_getcapenable(sc->sc_ifp) & IFCAP_RXCSUM) != 0);

	mtx_assert(&q->q_rx_mtx, MA_OWNED);

	sc->sc_drv_stats.rxeof_cnt++;

	RGE_DPRINTF(sc, RGE_DEBUG_INTR, "%s; called\n", __func__);

	/* Note: if_re is POSTREAD/WRITE, rge is only POSTWRITE */
	bus_dmamap_sync(sc->sc_dmat_rx_desc, q->q_rx.rge_rx_list_map,
	    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

	prod = q->q_rx.rge_rxq_prodidx;

	/*
	 * Loop around until we've run out of active descriptors to check
	 * or maxpkt has been reached.
	 */
	for (i = cons = q->q_rx.rge_rxq_considx;
	    maxpkt > 0 && i != prod;
	    i = RGE_NEXT_RX_DESC(i)) {
		/* break out of loop if we're not running */
		if ((if_getdrvflags(sc->sc_ifp) & IFF_DRV_RUNNING) == 0)
			break;

		/* get the current rx descriptor to check descriptor status */
		cur_rx = &q->q_rx.rge_rx_list[i];
		rxstat = le32toh(cur_rx->hi_qword1.rx_qword4.rge_cmdsts);
		if ((rxstat & RGE_RDCMDSTS_OWN) != 0) {
			break;
		}

		/* Ensure everything else has been DMAed */
		rmb();

		/* Get the current rx buffer, sync */
		rxq = &q->q_rx.rge_rxq[i];

		/* Ensure any device updates are now visible in host memory */
		bus_dmamap_sync(sc->sc_dmat_rx_buf, rxq->rxq_dmamap,
		    BUS_DMASYNC_POSTREAD);

		/* Unload the DMA map, we are done with it here */
		bus_dmamap_unload(sc->sc_dmat_rx_buf, rxq->rxq_dmamap);
		m = rxq->rxq_mbuf;
		rxq->rxq_mbuf = NULL;

		rx = 1;

		RGE_DPRINTF(sc, RGE_DEBUG_RECV_DESC,
		    "%s: RX: [%d]: m=%p, m_data=%p, m_len=%ju, "
		    "desc=0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n",
		    __func__,
		    i,
		    m,
		    m->m_data,
		    (uintmax_t) m->m_len,
		    ((uint32_t *) cur_rx)[0],
		    ((uint32_t *) cur_rx)[1],
		    ((uint32_t *) cur_rx)[2],
		    ((uint32_t *) cur_rx)[3],
		    ((uint32_t *) cur_rx)[4],
		    ((uint32_t *) cur_rx)[5],
		    ((uint32_t *) cur_rx)[6],
		    ((uint32_t *) cur_rx)[7]);

		if ((rxstat & RGE_RDCMDSTS_SOF) != 0) {
			if (q->q_rx.rge_head != NULL) {
				sc->sc_drv_stats.rx_desc_err_multidesc++;
				if_inc_counter(sc->sc_ifp, IFCOUNTER_IERRORS,
				    1);
				m_freem(q->q_rx.rge_head);
				q->q_rx.rge_tail = &q->q_rx.rge_head;
			}

			m->m_pkthdr.len = 0;
		} else if (q->q_rx.rge_head == NULL) {
			m_freem(m);
			continue;
		} else
			m->m_flags &= ~M_PKTHDR;

		*q->q_rx.rge_tail = m;
		q->q_rx.rge_tail = &m->m_next;

		mlen = rxstat & RGE_RDCMDSTS_FRAGLEN;
		m->m_len = mlen;

		m = q->q_rx.rge_head;
		m->m_pkthdr.len += mlen;

		/* Ethernet CRC error */
		if (rxstat & RGE_RDCMDSTS_RXERRSUM) {
			sc->sc_drv_stats.rx_ether_csum_err++;
			if_inc_counter(sc->sc_ifp, IFCOUNTER_IERRORS, 1);
			m_freem(m);
			q->q_rx.rge_head = NULL;
			q->q_rx.rge_tail = &q->q_rx.rge_head;
			continue;
		}

		/*
		 * This mbuf is part of a multi-descriptor frame,
		 * so count it towards that.
		 *
		 * Yes, this means we won't be counting the
		 * final descriptor/mbuf as part of a multi-descriptor
		 * frame; if someone wishes to do that then it
		 * shouldn't be too hard to add.
		 */
		if ((rxstat & RGE_RDCMDSTS_EOF) == 0) {
			sc->sc_drv_stats.rx_desc_jumbo_frag++;
			continue;
		}

		q->q_rx.rge_head = NULL;
		q->q_rx.rge_tail = &q->q_rx.rge_head;

		m_adj(m, -ETHER_CRC_LEN);
		m->m_pkthdr.rcvif = sc->sc_ifp;
		if_inc_counter(sc->sc_ifp, IFCOUNTER_IPACKETS, 1);
		q->q_rx_frames++;

		extsts = le32toh(cur_rx->hi_qword1.rx_qword4.rge_extsts);

		/* Check IP header checksum. */
		if (check_hwcsum) {
			/* Does it exist for IPv4? */
			if (extsts & RGE_RDEXTSTS_IPV4) {
				sc->sc_drv_stats.rx_offload_csum_ipv4_exists++;
				m->m_pkthdr.csum_flags |=
				    CSUM_IP_CHECKED;
			}
			/* XXX IPv6 checksum check? */

			if (((extsts & RGE_RDEXTSTS_IPCSUMERR) == 0)
			    && ((extsts & RGE_RDEXTSTS_IPV4) != 0)) {
				sc->sc_drv_stats.rx_offload_csum_ipv4_valid++;
				m->m_pkthdr.csum_flags |=
				    CSUM_IP_VALID;
			}

			/* Check TCP/UDP checksum. */
			if ((extsts & (RGE_RDEXTSTS_IPV4 | RGE_RDEXTSTS_IPV6)) &&
			    (extsts & RGE_RDEXTSTS_TCPPKT)) {
				sc->sc_drv_stats.rx_offload_csum_tcp_exists++;
				if ((extsts & RGE_RDEXTSTS_TCPCSUMERR) == 0) {
					sc->sc_drv_stats.rx_offload_csum_tcp_valid++;
					/* TCP checksum OK */
					m->m_pkthdr.csum_flags |=
					    CSUM_DATA_VALID|CSUM_PSEUDO_HDR;
					m->m_pkthdr.csum_data = 0xffff;
				}
			}

			if ((extsts & (RGE_RDEXTSTS_IPV4 | RGE_RDEXTSTS_IPV6)) &&
			    (extsts & RGE_RDEXTSTS_UDPPKT)) {
				sc->sc_drv_stats.rx_offload_csum_udp_exists++;
				if ((extsts & RGE_RDEXTSTS_UDPCSUMERR) == 0) {
					sc->sc_drv_stats.rx_offload_csum_udp_valid++;
					/* UDP checksum OK */
					m->m_pkthdr.csum_flags |=
					    CSUM_DATA_VALID|CSUM_PSEUDO_HDR;
					m->m_pkthdr.csum_data = 0xffff;
				}
			}
		}

		if (extsts & RGE_RDEXTSTS_VTAG) {
			sc->sc_drv_stats.rx_offload_vlan_tag++;
			m->m_pkthdr.ether_vtag =
			    ntohs(extsts & RGE_RDEXTSTS_VLAN_MASK);
			m->m_flags |= M_VLANTAG;
		}

		/*
		 * Hardware RSS hash and type (v3 descriptor).  Only trusted
		 * from a frame that fits one descriptor; the hash belongs to
		 * the descriptor carrying the headers.
		 */
		if ((rxstat & (RGE_RDCMDSTS_SOF | RGE_RDCMDSTS_EOF)) ==
		    (RGE_RDCMDSTS_SOF | RGE_RDCMDSTS_EOF)) {
			uint16_t hdr;

			hdr = le16toh(cur_rx->lo_qword1.rx_qword1.hdr_info);
			if (hdr & (RGE_RXHDR_RSS_IPV4 | RGE_RXHDR_RSS_IPV6)) {
				m->m_pkthdr.flowid =
				    le32toh(cur_rx->lo_qword1.rx_qword1.rss);
				if (hdr & RGE_RXHDR_RSS_TCP)
					M_HASHTYPE_SET(m, (hdr & RGE_RXHDR_RSS_IPV4) ?
					    M_HASHTYPE_RSS_TCP_IPV4 :
					    M_HASHTYPE_RSS_TCP_IPV6);
				else if (hdr & RGE_RXHDR_RSS_UDP)
					M_HASHTYPE_SET(m, (hdr & RGE_RXHDR_RSS_IPV4) ?
					    M_HASHTYPE_RSS_UDP_IPV4 :
					    M_HASHTYPE_RSS_UDP_IPV6);
				else
					M_HASHTYPE_SET(m, (hdr & RGE_RXHDR_RSS_IPV4) ?
					    M_HASHTYPE_RSS_IPV4 :
					    M_HASHTYPE_RSS_IPV6);
				sc->sc_drv_stats.rx_rss_hashed++;
			}
		}

		mbufq_enqueue(mq, m);

		maxpkt--;
	}

	if (!rx)
		return (0);

	/*
	 * Make sure any device updates to the descriptor ring are
	 * visible to the host before we continue.
	 */
	bus_dmamap_sync(sc->sc_dmat_rx_desc, q->q_rx.rge_rx_list_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	wmb();

	/* Update the consumer index, refill the RX ring */
	q->q_rx.rge_rxq_considx = i;
	rge_fill_rx_ring(q);

	return (1);
}

int
rge_txeof(struct rge_queues *q)
{
	struct rge_softc *sc = q->q_sc;
	struct ifnet *ifp = sc->sc_ifp;
	struct rge_txq *txq;
	uint32_t txstat;
	int cons, prod, cur, idx;
	int free = 0, ntx = 0;
	int pktlen;
	bool is_mcast;

	mtx_assert(&q->q_tx_mtx, MA_OWNED);
	sc->sc_drv_stats.txeof_cnt++;

	prod = q->q_tx.rge_txq_prodidx;
	cons = q->q_tx.rge_txq_considx;

	idx = cons;
	while (idx != prod) {
		txq = &q->q_tx.rge_txq[idx];
		cur = txq->txq_descidx;

		rge_tx_list_sync(sc, q, cur, 1, BUS_DMASYNC_POSTREAD);
		txstat = q->q_tx.rge_tx_list[cur].rge_cmdsts;
		rge_tx_list_sync(sc, q, cur, 1, BUS_DMASYNC_PREREAD);
		if ((txstat & htole32(RGE_TDCMDSTS_OWN)) != 0) {
			free = 2;
			break;
		}

		bus_dmamap_sync(sc->sc_dmat_tx_buf, txq->txq_dmamap,
		    BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->sc_dmat_tx_buf, txq->txq_dmamap);
		pktlen = txq->txq_mbuf->m_pkthdr.len;
		is_mcast = ((txq->txq_mbuf->m_flags & M_MCAST) != 0);
		m_freem(txq->txq_mbuf);
		txq->txq_mbuf = NULL;
		ntx++;

		if ((txstat &
		    htole32(RGE_TDCMDSTS_EXCESSCOLL | RGE_TDCMDSTS_COLL)) != 0)
			if_inc_counter(ifp, IFCOUNTER_COLLISIONS, 1);
		if ((txstat & htole32(RGE_TDCMDSTS_TXERR)) != 0)
			if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
		else {
			if_inc_counter(ifp, IFCOUNTER_OPACKETS, 1);
			if_inc_counter(ifp, IFCOUNTER_OBYTES, pktlen);
			if (is_mcast)
				if_inc_counter(ifp, IFCOUNTER_OMCASTS, 1);

		}

		idx = RGE_NEXT_TX_DESC(cur);
		free = 1;
	}

	/* Without completion progress, preserve the watchdog deadline. */
	if (ntx == 0)
		return (0);

	if (idx >= cons) {
		rge_tx_list_sync(sc, q, cons, idx - cons,
		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
	} else {
		rge_tx_list_sync(sc, q, cons, RGE_TX_LIST_CNT - cons,
		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
		rge_tx_list_sync(sc, q, 0, idx,
		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
	}

	q->q_tx.rge_txq_considx = idx;

	RGE_DPRINTF(sc, RGE_DEBUG_XMIT,
	    "%s: handled %d frames; prod=%d, cons=%d\n", __func__,
	    ntx, q->q_tx.rge_txq_prodidx, q->q_tx.rge_txq_considx);

	/*
	 * We processed the ring and hit a descriptor that was still
	 * owned by the hardware, so there's still pending work.
	 *
	 * If we got to the end of the ring and there's no further
	 * frames owned by the hardware then we can quieten the
	 * watchdog.
	 */
	if (free == 2)
		sc->sc_watchdog = 5;
	else
		sc->sc_watchdog = 0;

	/* q_tx_mtx protects the queue; only wake a worker with work. */
	if (!mbufq_empty(&q->q_txq))
		rge_schedule_tx_locked(q);

	return (1);
}

static u_int
rge_hash_maddr(void *arg, struct sockaddr_dl *sdl, u_int cnt)
{
	uint32_t crc, *hashes = arg;

	/* XXX TODO: validate this does addrlo? */
	crc = ether_crc32_be(LLADDR(sdl), ETHER_ADDR_LEN) >> 26;
	crc &= 0x3f;

	if (crc < 32)
		hashes[0] |= (1 << crc);
	else
		hashes[1] |= (1 << (crc - 32));

	return (1);
}

/**
 * @brief Configure the RX filter and multicast filter.
 *
 * This must be called with the driver lock held.
 */
static void
rge_iff_locked(struct rge_softc *sc)
{
	uint32_t hashes[2];
	uint32_t rxfilt;

	RGE_ASSERT_LOCKED(sc);

	rxfilt = RGE_READ_4(sc, RGE_RXCFG);
	rxfilt &= ~(RGE_RXCFG_ALLPHYS | RGE_RXCFG_MULTI);

	/*
	 * Always accept frames destined to our station address.
	 * Always accept broadcast frames.
	 */
	rxfilt |= RGE_RXCFG_INDIV | RGE_RXCFG_BROAD;

	if ((if_getflags(sc->sc_ifp) & (IFF_PROMISC | IFF_ALLMULTI)) != 0) {
		rxfilt |= RGE_RXCFG_MULTI;
		if ((if_getflags(sc->sc_ifp) & IFF_PROMISC) != 0)
			rxfilt |= RGE_RXCFG_ALLPHYS;
		hashes[0] = hashes[1] = 0xffffffff;
	} else {
		rxfilt |= RGE_RXCFG_MULTI;
		/* Program new filter. */
		memset(hashes, 0, sizeof(hashes));
		if_foreach_llmaddr(sc->sc_ifp, rge_hash_maddr, &hashes);
	}

	RGE_WRITE_4(sc, RGE_RXCFG, rxfilt);
	RGE_WRITE_4(sc, RGE_MAR0, bswap32(hashes[1]));
	RGE_WRITE_4(sc, RGE_MAR4, bswap32(hashes[0]));
}

static void
rge_add_media_types(struct rge_softc *sc)
{
	ifmedia_add(&sc->sc_media, IFM_ETHER | IFM_10_T, 0, NULL);
	ifmedia_add(&sc->sc_media, IFM_ETHER | IFM_10_T | IFM_FDX, 0, NULL);
	ifmedia_add(&sc->sc_media, IFM_ETHER | IFM_100_TX, 0, NULL);
	ifmedia_add(&sc->sc_media, IFM_ETHER | IFM_100_TX | IFM_FDX, 0, NULL);
	ifmedia_add(&sc->sc_media, IFM_ETHER | IFM_1000_T, 0, NULL);
	ifmedia_add(&sc->sc_media, IFM_ETHER | IFM_1000_T | IFM_FDX, 0, NULL);
	ifmedia_add(&sc->sc_media, IFM_ETHER | IFM_2500_T, 0, NULL);
	ifmedia_add(&sc->sc_media, IFM_ETHER | IFM_2500_T | IFM_FDX, 0, NULL);

	if (RGE_TYPE_R26(sc)) {
		ifmedia_add(&sc->sc_media, IFM_ETHER | IFM_5000_T, 0, NULL);
		ifmedia_add(&sc->sc_media, IFM_ETHER | IFM_5000_T | IFM_FDX,
		    0, NULL);
	} else if (sc->rge_type == MAC_R27) {
		ifmedia_add(&sc->sc_media, IFM_ETHER | IFM_10G_T, 0, NULL);
		ifmedia_add(&sc->sc_media, IFM_ETHER | IFM_10G_T | IFM_FDX,
		    0, NULL);
	}
}

/**
 * @brief Packet dequeue and submit with the driver lock held.
 */
static void
rge_tx_drain_locked(struct rge_queues *q)
{
	struct rge_softc *sc = q->q_sc;
	struct mbuf *m;
	int ntx = 0;
	int idx, free, used;

	RGE_DPRINTF(sc, RGE_DEBUG_XMIT, "%s: running\n", __func__);

	mtx_assert(&q->q_tx_mtx, MA_OWNED);

	if (sc->sc_stopped == true) {
		sc->sc_watchdog = 0;
		return;
	}

	/* Calculate free space. */
	idx = q->q_tx.rge_txq_prodidx;
	free = q->q_tx.rge_txq_considx;
	if (free <= idx)
		free += RGE_TX_LIST_CNT;
	free -= idx;

	for (;;) {
		if (free < RGE_TX_NSEGS + 2) {
			break;
		}

		/* Dequeue */
		m = mbufq_dequeue(&q->q_txq);
		if (m == NULL)
			break;

		/* Attempt to encap, m might change due to defrag */
		used = rge_encap(sc, q, &m, idx);
		if (used < 0) {
			if_inc_counter(sc->sc_ifp, IFCOUNTER_OQDROPS, 1);
			m_freem(m);
			continue;
		} else if (used == 0) {
			mbufq_prepend(&q->q_txq, m);
			break;
		}

		/*
		 * Note: mbuf is now owned by the tx ring, but we hold the
		 * lock so it's safe to pass it up here to be copied without
		 * worrying the TX task will run and dequeue/free it before
		 * we get a shot at it.
		 */
		ETHER_BPF_MTAP(sc->sc_ifp, m);

		/* Update free/idx pointers */
		free -= used;
		idx += used;
		if (idx >= RGE_TX_LIST_CNT)
			idx -= RGE_TX_LIST_CNT;

		ntx++;
	}

	/* Ok, did we queue anything? If so, poke this queue's poll bit. */
	if (ntx > 0) {
		q->q_tx.rge_txq_prodidx = idx;
		q->q_tx_frames += ntx;
		sc->sc_watchdog = 5;
		RGE_WRITE_2(sc, RGE_TXSTART, RGE_TXSTART_Q(q->q_index));
	}

	RGE_DPRINTF(sc, RGE_DEBUG_XMIT,
	    "%s: handled %d frames; prod=%d, cons=%d\n", __func__,
	    ntx, q->q_tx.rge_txq_prodidx, q->q_tx.rge_txq_considx);

}

static void
rge_tx_task(void *arg, int npending)
{
	struct rge_queues *q = arg;

	mtx_lock(&q->q_tx_mtx);
	q->q_tx_task_pending = false;
	q->q_sc->sc_drv_stats.tx_task_cnt++;
	rge_tx_drain_locked(q);
	mtx_unlock(&q->q_tx_mtx);
}

/**
 * @brief Called by the sc_timeout callout.
 *
 * This is called by the callout code with the driver lock held.
 */
void
rge_tick(void *arg)
{
	struct rge_softc *sc = arg;

	RGE_ASSERT_LOCKED(sc);

	rge_link_state(sc);

	/*
	 * Since we don't have any other place yet to trigger/test this,
	 * let's do it here every second and just bite the driver
	 * blocking for a little bit whilst it happens.
	 */
	if ((if_getdrvflags(sc->sc_ifp) & IFF_DRV_RUNNING) != 0)
		rge_hw_mac_stats_fetch(sc, &sc->sc_mac_stats.lcl_stats);

	/*
	 * Handle the TX watchdog.
	 */
	if (sc->sc_watchdog > 0) {
		sc->sc_watchdog--;
		if (sc->sc_watchdog == 0) {
			RGE_PRINT_ERROR(sc, "TX timeout (watchdog)\n");
			rge_init_locked(sc);
			sc->sc_drv_stats.tx_watchdog_timeout_cnt++;
		}
	}

	callout_reset(&sc->sc_timeout, hz, rge_tick, sc);
}

/**
 * @brief process a link state change.
 *
 * Must be called with the driver lock held.
 */
void
rge_link_state(struct rge_softc *sc)
{
	int link = LINK_STATE_DOWN;

	RGE_ASSERT_LOCKED(sc);

	if (rge_get_link_status(sc))
		link = LINK_STATE_UP;

	if (if_getlinkstate(sc->sc_ifp) != link) {
		sc->sc_drv_stats.link_state_change_cnt++;
		if_link_state_change(sc->sc_ifp, link);
	}
}

static void
rge_setwol(struct rge_softc *sc)
{
	if_t ifp = sc->sc_ifp;
	int enable;

	mtx_assert(&sc->sc_mtx, MA_OWNED);

	if (!pci_has_pm(sc->sc_dev))
		return;

	enable = (if_getcapenable(ifp) & IFCAP_WOL_MAGIC) != 0;

	rge_wol_config(sc, enable);
}

/**
 * @brief Suspend
 */
static int
rge_suspend(device_t dev)
{
	struct rge_softc *sc = device_get_softc(dev);

	RGE_LOCK(sc);
	rge_stop_locked(sc);
	rge_setwol(sc);
	sc->sc_suspended = true;
	RGE_UNLOCK(sc);

	return (0);
}

/**
 * @brief Resume
 */
static int
rge_resume(device_t dev)
{
	struct rge_softc *sc = device_get_softc(dev);

	RGE_LOCK(sc);

	/* reinit if required */
	if (if_getflags(sc->sc_ifp) & IFF_UP)
		rge_init_locked(sc);

	sc->sc_suspended = false;

	RGE_UNLOCK(sc);

	return (0);
}

/**
 * @brief Shutdown the driver during shutdown
 */
static int
rge_shutdown(device_t dev)
{
	struct rge_softc *sc = device_get_softc(dev);

	RGE_LOCK(sc);
	rge_stop_locked(sc);
	rge_setwol(sc);
	RGE_UNLOCK(sc);

	return (0);
}

static device_method_t rge_methods[] = {
	DEVMETHOD(device_probe,			rge_probe),
	DEVMETHOD(device_attach,		rge_attach),
	DEVMETHOD(device_detach,		rge_detach),

	DEVMETHOD(device_suspend,		rge_suspend),
	DEVMETHOD(device_resume,		rge_resume),
	DEVMETHOD(device_shutdown,		rge_shutdown),

	DEVMETHOD_END
};

static driver_t rge_driver = {
	"rge",
	rge_methods,
	sizeof(struct rge_softc)
};

MODULE_DEPEND(rge, pci, 1, 1, 1);
MODULE_DEPEND(rge, ether, 1, 1, 1);

DRIVER_MODULE_ORDERED(rge, pci, rge_driver, NULL, NULL, SI_ORDER_ANY);
MODULE_PNP_INFO("U16:vendor;U16:device;D:#", pci, rge, rge_devices,
    nitems(rge_devices) - 1);
