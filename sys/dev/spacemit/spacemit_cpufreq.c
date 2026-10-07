/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Dave Klotz <DaveWK225@protonmail.com>
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
 * CPU frequency driver for the SpacemiT K1 (Ky X1).
 *
 * The K1 has two clusters of four X60 cores.  Each cluster's core clock is
 * an APMU mux with a frequency-change (FC) handshake (APMU_CPU_C0/C1_CLK_
 * CTRL, mainline Linux drivers/clk/spacemit/ccu-k1.c cpu_c{0,1}_core_clk):
 *
 *	mux 0 pll1_d4  614.4 MHz	mux 4 pll1_d2 1228.8 MHz
 *	mux 1 pll1_d3  819.2 MHz	mux 5 pll3_d3
 *	mux 2 pll1_d6  409.6 MHz	mux 6 pll2_d3
 *	mux 3 pll1_d5 491.52 MHz	mux 7 pll3_d2 (bit 13 clear) or pll3
 *
 * Both clusters share one CPU rail (P1 buck1), so like SpacemiT's own
 * kernel this driver runs one policy for all eight CPUs and switches both
 * clusters together.
 *
 * This driver changes frequency only; it never writes a PLL, a PLL output
 * gate or the CPU rail.  A frequency is offered only when its parent PLL is
 * locked and its output gates are open (the PLL1 parents are gated twice, in
 * APBS PLL1_SWCR2 and in MPMU_ACGR), and never above the highest frequency
 * the hardware has been observed running since boot: the firmware or the
 * board's boot service establishes the rail voltage for that operating
 * point (1.05 V for 1.6 GHz) before switching to it, and every lower point
 * needs no more voltage than that.  Lower points therefore remain valid at
 * the established rail.  The 1.0 GHz pll2_d3 point is offered only when the
 * boot firmware left PLL2 running.  hw.spacemit_cpufreq.max_mhz caps the
 * offered levels further, and hw.spacemit_cpufreq.verbose=1 traces every
 * switch on the console.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/eventhandler.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/sched.h>
#include <sys/smp.h>
#include <sys/sysctl.h>

#include <machine/bus.h>
#include <machine/cpufunc.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/syscon/syscon.h>

#include "cpufreq_if.h"
#include "syscon_if.h"

/* APMU (syscon, owned by spacemit_ccu) */
#define	APMU_CPU_C0_CLK_CTRL	0x38c
#define	APMU_CPU_C1_CLK_CTRL	0x390
#define	 CPU_CLK_MUX_MASK	0x7
#define	 CPU_CLK_FC		(1u << 12)
#define	 CPU_CLK_HI_PLL3_D1	(1u << 13)

/* APBS PLL block (read only here) */
#define	APBS_PLL1_SWCR2		0x104
#define	APBS_PLL2_SWCR1		0x118
#define	APBS_PLL2_SWCR2		0x11c
#define	APBS_PLL3_SWCR1		0x124
#define	APBS_PLL3_SWCR2		0x128
#define	APBS_MAP_SIZE		0x130

/* MPMU (read only here) */
#define	MPMU_POSR		0x10
#define	 POSR_PLL1_LOCK		(1u << 27)
#define	 POSR_PLL2_LOCK		(1u << 28)
#define	 POSR_PLL3_LOCK		(1u << 29)
#define	MPMU_ACGR		0x1024	/* PLL1 output gates (second level) */
#define	MPMU_MAP_SIZE		0x1028

/* PLL programming this driver knows the rate of (mainline rate tables). */
#define	PLL2_SWCR1_3000MHZ	0x0050dd66
#define	PLL3_SWCR1_3200MHZ	0x0050dd67

#define	FC_TIMEOUT_US		10000
#define	FC_POLL_US		5
#define	CPU_CLK_LATENCY_US	200	/* OPP clock-latency-ns 200000 */

enum k1_parent {
	K1_PLL1,
	K1_PLL2,
	K1_PLL3,
};

struct k1_cpufreq_point {
	int		mhz;		/* cpufreq(4) level, MHz */
	uint64_t	hz;		/* exact rate */
	int		mux;
	enum k1_parent	parent;
	uint32_t	gate;		/* APBS SWCR2 output gate bit */
	int		acgr;		/* MPMU_ACGR gate bit, -1 none */
};

/* Sorted from fastest to slowest. */
static const struct k1_cpufreq_point k1_points[] = {
	{ 1600, 1600000000ULL, 7, K1_PLL3, 1u << 1, -1 },	/* pll3_d2 */
	{ 1228, 1228800000ULL, 4, K1_PLL1, 1u << 1, 16 },	/* pll1_d2_1228p8 */
	{ 1000, 1000000000ULL, 6, K1_PLL2, 1u << 2, -1 },	/* pll2_d3 */
	{  819,  819200000ULL, 1, K1_PLL1, 1u << 2, 14 },	/* pll1_d3_819p2 */
	{  614,  614400000ULL, 0, K1_PLL1, 1u << 3, 15 },	/* pll1_d4_614p4 */
	{  491,  491520000ULL, 3, K1_PLL1, 1u << 4, 21 },	/* pll1_d5_491p52 */
	{  409,  409600000ULL, 2, K1_PLL1, 1u << 5,  0 },	/* pll1_d6_409p6 */
};
#define	K1_NPOINTS	nitems(k1_points)

static int spacemit_cpufreq_max_mhz = 0;
SYSCTL_NODE(_hw, OID_AUTO, spacemit_cpufreq, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "SpacemiT K1 CPU frequency");
SYSCTL_INT(_hw_spacemit_cpufreq, OID_AUTO, max_mhz, CTLFLAG_RWTUN,
    &spacemit_cpufreq_max_mhz, 0,
    "Highest CPU frequency offered, MHz (0 = highest validated)");

static int spacemit_cpufreq_verbose = 0;
SYSCTL_INT(_hw_spacemit_cpufreq, OID_AUTO, verbose, CTLFLAG_RWTUN,
    &spacemit_cpufreq_verbose, 0, "Trace each cluster switch on the console");

struct k1_cpufreq_softc {
	device_t		dev;
	phandle_t		apmu_node;
	struct syscon		*apmu;
	bus_space_tag_t		bst;
	bus_space_handle_t	apbs;
	bus_space_handle_t	mpmu;
	int			ceiling_mhz;	/* highest observed */
	int			boot_mhz;
	int			fail;		/* FC timeout seen */
	u_int			seen_mux;	/* parents observed running */
	uint64_t		nswitch;
};

static const struct k1_cpufreq_point *
k1_point_by_mhz(int mhz)
{
	u_int i;

	for (i = 0; i < K1_NPOINTS; i++)
		if (k1_points[i].mhz == mhz)
			return (&k1_points[i]);
	return (NULL);
}

static uint32_t
k1_apbs_read(struct k1_cpufreq_softc *sc, bus_size_t off)
{

	return (bus_space_read_4(sc->bst, sc->apbs, off));
}

static uint32_t
k1_posr(struct k1_cpufreq_softc *sc)
{

	return (bus_space_read_4(sc->bst, sc->mpmu, MPMU_POSR));
}

static int
k1_apmu_get(struct k1_cpufreq_softc *sc)
{

	if (sc->apmu != NULL)
		return (0);
	if (syscon_get_by_ofw_node(sc->dev, sc->apmu_node, &sc->apmu) != 0)
		return (ENXIO);
	return (0);
}

/* Is the parent of 'p' running at the rate the table assumes? */
static bool
k1_point_ready(struct k1_cpufreq_softc *sc, const struct k1_cpufreq_point *p)
{
	uint32_t posr;

	posr = k1_posr(sc);
	/*
	 * The PLL1 parents are gated twice: in APBS PLL1_SWCR2 and again in
	 * MPMU_ACGR (mainline ccu-k1.c pll1_dN_* gates).  A cluster switched
	 * to a closed gate stops: the 1228.8 MHz gate is closed under the
	 * oreboot/EDK2 chain, and switching to it hung an R2S.
	 */
	if (p->acgr >= 0 && (bus_space_read_4(sc->bst, sc->mpmu, MPMU_ACGR) &
	    (1u << p->acgr)) == 0)
		return (false);
	switch (p->parent) {
	case K1_PLL1:
		return ((posr & POSR_PLL1_LOCK) != 0 &&
		    (k1_apbs_read(sc, APBS_PLL1_SWCR2) & p->gate) != 0);
	case K1_PLL2:
		return ((posr & POSR_PLL2_LOCK) != 0 &&
		    k1_apbs_read(sc, APBS_PLL2_SWCR1) == PLL2_SWCR1_3000MHZ &&
		    (k1_apbs_read(sc, APBS_PLL2_SWCR2) & p->gate) != 0);
	case K1_PLL3:
		return ((posr & POSR_PLL3_LOCK) != 0 &&
		    k1_apbs_read(sc, APBS_PLL3_SWCR1) == PLL3_SWCR1_3200MHZ &&
		    (k1_apbs_read(sc, APBS_PLL3_SWCR2) & p->gate) != 0);
	}
	return (false);
}

/* Decode a cluster control word into MHz, or -1 when not representable. */
static int
k1_decode(struct k1_cpufreq_softc *sc, uint32_t ctrl)
{
	u_int i, mux;

	mux = ctrl & CPU_CLK_MUX_MASK;
	if (mux == 7 && (ctrl & CPU_CLK_HI_PLL3_D1) != 0)
		return (-1);		/* pll3 undivided: never selected */
	if (mux == 5)
		return (-1);		/* pll3_d3: not an operating point */
	for (i = 0; i < K1_NPOINTS; i++) {
		if (k1_points[i].mux != (int)mux)
			continue;
		if (!k1_point_ready(sc, &k1_points[i]))
			return (-1);
		return (k1_points[i].mhz);
	}
	return (-1);
}

static void
k1_read_ctrl(struct k1_cpufreq_softc *sc, uint32_t *c0, uint32_t *c1)
{

	SYSCON_DEVICE_LOCK(sc->apmu->pdev);
	*c0 = SYSCON_UNLOCKED_READ_4(sc->apmu, APMU_CPU_C0_CLK_CTRL);
	*c1 = SYSCON_UNLOCKED_READ_4(sc->apmu, APMU_CPU_C1_CLK_CTRL);
	SYSCON_DEVICE_UNLOCK(sc->apmu->pdev);
}

/*
 * Read the hardware and raise the validated ceiling when both clusters run
 * a faster known point than seen before.
 */
static int
k1_observe(struct k1_cpufreq_softc *sc, int *mhz0, int *mhz1)
{
	uint32_t c0, c1;
	int f0, f1, low;

	if (k1_apmu_get(sc) != 0)
		return (ENXIO);
	k1_read_ctrl(sc, &c0, &c1);
	f0 = k1_decode(sc, c0);
	f1 = k1_decode(sc, c1);
	if (f0 > 0)
		sc->seen_mux |= 1u << (c0 & CPU_CLK_MUX_MASK);
	if (f1 > 0)
		sc->seen_mux |= 1u << (c1 & CPU_CLK_MUX_MASK);
	if (f0 > 0 && f1 > 0) {
		low = MIN(f0, f1);
		if (low > sc->ceiling_mhz) {
			if (sc->ceiling_mhz != 0)
				device_printf(sc->dev,
				    "validated ceiling %d -> %d MHz\n",
				    sc->ceiling_mhz, low);
			sc->ceiling_mhz = low;
		}
	}
	if (mhz0 != NULL)
		*mhz0 = f0;
	if (mhz1 != NULL)
		*mhz1 = f1;
	return (0);
}

static bool
k1_point_offered(struct k1_cpufreq_softc *sc, const struct k1_cpufreq_point *p)
{

	if (sc->fail)
		return (false);
	if (p->mhz > sc->ceiling_mhz)
		return (false);
	if (spacemit_cpufreq_max_mhz > 0 && p->mhz > spacemit_cpufreq_max_mhz)
		return (false);
	return (k1_point_ready(sc, p));
}

/* Switch one cluster; called with the APMU syscon locked. */
static int
k1_switch_cluster(struct k1_cpufreq_softc *sc, bus_size_t reg, int mux)
{
	uint32_t val;
	int us;

	val = SYSCON_UNLOCKED_READ_4(sc->apmu, reg);
	if ((val & CPU_CLK_FC) != 0)
		return (EBUSY);
	if ((val & CPU_CLK_MUX_MASK) == (uint32_t)mux &&
	    (val & CPU_CLK_HI_PLL3_D1) == 0)
		return (0);
	if (spacemit_cpufreq_verbose)
		printf("%s: 0x%x: 0x%03x -> mux %d\n",
		    device_get_nameunit(sc->dev), (u_int)reg, val, mux);
	/* Keep the ACE/TCM dividers; select pll3_d2 for the high mux. */
	val &= ~(CPU_CLK_MUX_MASK | CPU_CLK_HI_PLL3_D1);
	val |= mux;
	SYSCON_UNLOCKED_WRITE_4(sc->apmu, reg, val);
	SYSCON_UNLOCKED_WRITE_4(sc->apmu, reg, val | CPU_CLK_FC);
	for (us = 0; us < FC_TIMEOUT_US; us += FC_POLL_US) {
		if ((SYSCON_UNLOCKED_READ_4(sc->apmu, reg) & CPU_CLK_FC) == 0)
			break;
		DELAY(FC_POLL_US);
	}
	val = SYSCON_UNLOCKED_READ_4(sc->apmu, reg);
	if (spacemit_cpufreq_verbose)
		printf("%s: 0x%x: FC %s after %d us, 0x%03x\n",
		    device_get_nameunit(sc->dev), (u_int)reg,
		    (val & CPU_CLK_FC) ? "pending" : "done", us, val);
	if ((val & CPU_CLK_FC) != 0)
		return (ETIMEDOUT);
	if ((val & CPU_CLK_MUX_MASK) != (uint32_t)mux)
		return (EIO);
	return (0);
}

static int
k1_cpufreq_set(device_t dev, const struct cf_setting *cf)
{
	struct k1_cpufreq_softc *sc;
	const struct k1_cpufreq_point *p;
	int error, f0, f1;

	sc = device_get_softc(dev);
	if (cf == NULL || cf->freq < 0)
		return (EINVAL);
	if ((p = k1_point_by_mhz(cf->freq)) == NULL)
		return (EINVAL);
	if ((error = k1_observe(sc, &f0, &f1)) != 0)
		return (error);
	if (!k1_point_offered(sc, p))
		return (EINVAL);
	if (f0 == p->mhz && f1 == p->mhz)
		return (0);

	SYSCON_DEVICE_LOCK(sc->apmu->pdev);
	error = k1_switch_cluster(sc, APMU_CPU_C0_CLK_CTRL, p->mux);
	if (error == 0)
		error = k1_switch_cluster(sc, APMU_CPU_C1_CLK_CTRL, p->mux);
	SYSCON_DEVICE_UNLOCK(sc->apmu->pdev);

	if (error != 0) {
		/*
		 * Both clusters stay on a parent this driver validated, so
		 * the rail is sufficient whatever the result; but a failed
		 * handshake means the clock is not in a state we understand.
		 * Stop offering changes.
		 */
		sc->fail = 1;
		device_printf(dev, "switch to %d MHz failed: %d; "
		    "frequency changes disabled\n", p->mhz, error);
		return (error);
	}
	sc->nswitch++;
	return (0);
}

static int
k1_cpufreq_get(device_t dev, struct cf_setting *cf)
{
	struct k1_cpufreq_softc *sc;
	int f0, f1;

	sc = device_get_softc(dev);
	if (cf == NULL)
		return (EINVAL);
	if (k1_observe(sc, &f0, &f1) != 0 || f0 < 0)
		return (ENXIO);
	memset(cf, CPUFREQ_VAL_UNKNOWN, sizeof(*cf));
	/* Report the slower cluster: that bounds both. */
	cf->freq = (f1 > 0) ? MIN(f0, f1) : f0;
	cf->lat = CPU_CLK_LATENCY_US;
	cf->dev = dev;
	return (0);
}

static int
k1_cpufreq_settings(device_t dev, struct cf_setting *sets, int *count)
{
	struct k1_cpufreq_softc *sc;
	u_int i;
	int n;

	sc = device_get_softc(dev);
	if (sets == NULL || count == NULL)
		return (EINVAL);
	(void)k1_observe(sc, NULL, NULL);

	n = 0;
	for (i = 0; i < K1_NPOINTS; i++) {
		if (!k1_point_offered(sc, &k1_points[i]))
			continue;
		if (n >= *count)
			return (E2BIG);
		memset(&sets[n], CPUFREQ_VAL_UNKNOWN, sizeof(sets[n]));
		sets[n].freq = k1_points[i].mhz;
		sets[n].lat = CPU_CLK_LATENCY_US;
		sets[n].dev = dev;
		n++;
	}
	if (n == 0)
		return (ENXIO);
	*count = n;
	return (0);
}

static int
k1_cpufreq_type(device_t dev, int *type)
{

	if (type == NULL)
		return (EINVAL);
	/* Firmware or a boot service may switch the clock behind us. */
	*type = CPUFREQ_TYPE_ABSOLUTE | CPUFREQ_FLAG_UNCACHED;
	return (0);
}

/*
 * Measure a CPU's core clock: cycle CSR ticks per 24 MHz time CSR tick over
 * ~2 ms, bound to that CPU.
 */
static int
k1_measure_mhz(int cpu)
{
	uint64_t c0, c1, t0, t1, tfreq;

	tfreq = 24000000;
	thread_lock(curthread);
	sched_bind(curthread, cpu);
	thread_unlock(curthread);
	critical_enter();
	t0 = rdtime();
	c0 = rdcycle();
	do {
		t1 = rdtime();
	} while (t1 - t0 < tfreq / 500);
	c1 = rdcycle();
	critical_exit();
	thread_lock(curthread);
	sched_unbind(curthread);
	thread_unlock(curthread);
	return ((int)((c1 - c0) * tfreq / (t1 - t0) / 1000000));
}

static int
k1_sysctl_measured(SYSCTL_HANDLER_ARGS)
{
	char buf[64];
	int c1cpu, m0, m1;

	if (!smp_started) {
		strlcpy(buf, "unavailable before SMP start", sizeof(buf));
	} else {
		/* Cluster 1 is CPUs 4-7 when all eight are present. */
		c1cpu = (mp_ncpus > 4) ? 4 : 0;
		m0 = k1_measure_mhz(0);
		m1 = k1_measure_mhz(c1cpu);
		snprintf(buf, sizeof(buf), "cluster0 %d cluster1 %d", m0, m1);
	}
	return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
}

static int
k1_sysctl_clusters(SYSCTL_HANDLER_ARGS)
{
	struct k1_cpufreq_softc *sc = arg1;
	char buf[96];
	uint32_t c0, c1;

	if (k1_apmu_get(sc) != 0)
		strlcpy(buf, "no APMU", sizeof(buf));
	else {
		k1_read_ctrl(sc, &c0, &c1);
		snprintf(buf, sizeof(buf),
		    "cluster0 0x%03x %d MHz cluster1 0x%03x %d MHz",
		    c0, k1_decode(sc, c0), c1, k1_decode(sc, c1));
	}
	return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
}

static bool
k1_cpufreq_platform(void)
{

	return (ofw_bus_node_is_compatible(OF_finddevice("/"), "spacemit,k1"));
}

static void
k1_cpufreq_identify(driver_t *driver, device_t parent)
{

	/* One policy for all CPUs: attach below cpu0 only. */
	if (device_get_unit(parent) != 0 || !k1_cpufreq_platform())
		return;
	if (device_find_child(parent, "spacemit_cpufreq", DEVICE_UNIT_ANY) !=
	    NULL)
		return;
	if (BUS_ADD_CHILD(parent, 0, "spacemit_cpufreq", DEVICE_UNIT_ANY) ==
	    NULL)
		device_printf(parent, "add spacemit_cpufreq child failed\n");
}

static int
k1_cpufreq_probe(device_t dev)
{

	if (device_get_unit(device_get_parent(dev)) != 0 ||
	    !k1_cpufreq_platform())
		return (ENXIO);
	device_set_desc(dev, "SpacemiT K1 CPU frequency");
	return (BUS_PROBE_DEFAULT);
}

static int
k1_map_node(struct k1_cpufreq_softc *sc, const char *compat, bus_size_t size,
    bus_space_handle_t *bshp)
{
	phandle_t node;
	bus_size_t sz;

	node = ofw_bus_find_compatible(OF_finddevice("/"), compat);
	if (node == 0 || OF_decode_addr(node, 0, &sc->bst, bshp, &sz) != 0)
		return (ENXIO);
	if (sz < size)
		return (ENXIO);
	return (0);
}

static int
k1_cpufreq_attach(device_t dev)
{
	struct k1_cpufreq_softc *sc;
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;
	int f0, f1;

	sc = device_get_softc(dev);
	sc->dev = dev;

	sc->apmu_node = ofw_bus_find_compatible(OF_finddevice("/"),
	    "spacemit,k1-syscon-apmu");
	if (sc->apmu_node == 0) {
		device_printf(dev, "no APMU node\n");
		return (ENXIO);
	}
	if (k1_map_node(sc, "spacemit,k1-pll", APBS_MAP_SIZE, &sc->apbs) != 0 ||
	    k1_map_node(sc, "spacemit,k1-syscon-mpmu", MPMU_MAP_SIZE,
	    &sc->mpmu) != 0) {
		device_printf(dev, "cannot map PLL/MPMU registers\n");
		return (ENXIO);
	}
	/* The APMU syscon may register after us; it is looked up lazily. */
	if (k1_observe(sc, &f0, &f1) == 0) {
		sc->boot_mhz = (f0 > 0 && f1 > 0) ? MIN(f0, f1) : -1;
		device_printf(dev, "cluster clocks %d/%d MHz at attach\n",
		    f0, f1);
	}

	ctx = device_get_sysctl_ctx(dev);
	tree = device_get_sysctl_tree(dev);
	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "ceiling_mhz",
	    CTLFLAG_RD, &sc->ceiling_mhz, 0,
	    "Highest frequency observed running, hence offered");
	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "attach_mhz",
	    CTLFLAG_RD, &sc->boot_mhz, 0, "Cluster frequency at attach");
	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "failed",
	    CTLFLAG_RD, &sc->fail, 0, "A frequency change handshake failed");
	SYSCTL_ADD_UINT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "seen_mux",
	    CTLFLAG_RD, &sc->seen_mux, 0,
	    "Bitmask of mux parents observed running the CPUs (information)");
	SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "switches",
	    CTLFLAG_RD, &sc->nswitch, 0, "Completed frequency changes");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "clusters",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    k1_sysctl_clusters, "A", "Cluster clock control words");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "measured_mhz",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    k1_sysctl_measured, "A",
	    "Core clocks measured from the cycle counter");

	cpufreq_register(dev);
	return (0);
}

static int
k1_cpufreq_detach(device_t dev)
{

	return (EBUSY);
}

static device_method_t k1_cpufreq_methods[] = {
	DEVMETHOD(device_identify,	k1_cpufreq_identify),
	DEVMETHOD(device_probe,		k1_cpufreq_probe),
	DEVMETHOD(device_attach,	k1_cpufreq_attach),
	DEVMETHOD(device_detach,	k1_cpufreq_detach),

	DEVMETHOD(cpufreq_drv_get,	k1_cpufreq_get),
	DEVMETHOD(cpufreq_drv_set,	k1_cpufreq_set),
	DEVMETHOD(cpufreq_drv_settings,	k1_cpufreq_settings),
	DEVMETHOD(cpufreq_drv_type,	k1_cpufreq_type),

	DEVMETHOD_END
};

static driver_t k1_cpufreq_driver = {
	"spacemit_cpufreq",
	k1_cpufreq_methods,
	sizeof(struct k1_cpufreq_softc),
};

DRIVER_MODULE(spacemit_cpufreq, cpu, k1_cpufreq_driver, 0, 0);
