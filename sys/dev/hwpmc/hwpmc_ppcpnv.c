/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2013 Justin Hibbits
 * Copyright (c) 2020 Leandro Lupori
 * All rights reserved.
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/pmc.h>
#include <sys/pmckern.h>
#include <sys/syslog.h>
#include <sys/systm.h>

#include <machine/pmc_mdep.h>
#include <machine/spr.h>
#include <machine/cpu.h>

#include "hwpmc_powerpc.h"

#define	PPCPNV_MAX_PMCS		6
#define	PPCPNV_MAX_PMC_VALUE	0x7fffffffUL
#define	PPCPNV_PMC_HAS_OVERFLOWED(n) (ppcpnv_pmcn_read(n) & (0x1 << 31))

#define	PMC_PPCPNV_FLAG_PMCS	0x3f
#define	PMC_PPCPNV_FLAG_PMC1	0x01
#define	PMC_PPCPNV_FLAG_PMC2	0x02
#define	PMC_PPCPNV_FLAG_PMC3	0x04
#define	PMC_PPCPNV_FLAG_PMC4	0x08
#define	PMC_PPCPNV_FLAG_PMC5	0x10
#define	PMC_PPCPNV_FLAG_PMC6	0x20

struct pmc_ppcpnv_event {
	enum pmc_event pe_event;
	uint32_t pe_flags;
	uint32_t pe_code;
};

static struct pmc_ppcpnv_event ppcpnv_event_codes[] = {
	{PMC_EV_PPCPNV_INSTR_COMPLETED,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC5,
	    .pe_code = 0x00
	},
	/*
	 * PMC1 can also count cycles, but as PMC6 can only count cycles
	 * it's better to always use it and leave PMC1 free to count
	 * other events.
	 */
	{PMC_EV_PPCPNV_CYCLES,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC6,
	    .pe_code = 0xf0
	},
	{PMC_EV_PPCPNV_CYCLES_WITH_INSTRS_COMPLETED,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC1,
	    .pe_code = 0xf2
	},
	{PMC_EV_PPCPNV_FPU_INSTR_COMPLETED,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC1,
	    .pe_code = 0xf4
	},
	{PMC_EV_PPCPNV_ERAT_INSTR_MISS,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC1,
	    .pe_code = 0xf6
	},
	{PMC_EV_PPCPNV_CYCLES_IDLE,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC1,
	    .pe_code = 0xf8
	},
	{PMC_EV_PPCPNV_CYCLES_WITH_ANY_THREAD_RUNNING,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC1,
	    .pe_code = 0xfa
	},
	{PMC_EV_PPCPNV_STORE_COMPLETED,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC2,
	    .pe_code = 0xf0
	},
	{PMC_EV_PPCPNV_INSTR_DISPATCHED,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC2 | PMC_PPCPNV_FLAG_PMC3,
	    .pe_code = 0xf2
	},
	{PMC_EV_PPCPNV_CYCLES_RUNNING,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC2,
	    .pe_code = 0xf4
	},
	{PMC_EV_PPCPNV_ERAT_DATA_MISS,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC2,
	    .pe_code = 0xf6
	},
	{PMC_EV_PPCPNV_EXTERNAL_INTERRUPT,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC2,
	    .pe_code = 0xf8
	},
	{PMC_EV_PPCPNV_BRANCH_TAKEN,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC2,
	    .pe_code = 0xfa
	},
	{PMC_EV_PPCPNV_L1_INSTR_MISS,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC2,
	    .pe_code = 0xfc
	},
	{PMC_EV_PPCPNV_L2_LOAD_MISS,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC2,
	    .pe_code = 0xfe
	},
	{PMC_EV_PPCPNV_STORE_NO_REAL_ADDR,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC3,
	    .pe_code = 0xf0
	},
	{PMC_EV_PPCPNV_INSTR_COMPLETED_WITH_ALL_THREADS_RUNNING,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC3,
	    .pe_code = 0xf4
	},
	{PMC_EV_PPCPNV_L1_LOAD_MISS,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC3,
	    .pe_code = 0xf6
	},
	{PMC_EV_PPCPNV_TIMEBASE_EVENT,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC3,
	    .pe_code = 0xf8
	},
	{PMC_EV_PPCPNV_L3_INSTR_MISS,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC3,
	    .pe_code = 0xfa
	},
	{PMC_EV_PPCPNV_TLB_DATA_MISS,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC3,
	    .pe_code = 0xfc
	},
	{PMC_EV_PPCPNV_L3_LOAD_MISS,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC3,
	    .pe_code = 0xfe
	},
	{PMC_EV_PPCPNV_LOAD_NO_REAL_ADDR,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC4,
	    .pe_code = 0xf0
	},
	{PMC_EV_PPCPNV_CYCLES_WITH_INSTRS_DISPATCHED,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC4,
	    .pe_code = 0xf2
	},
	{PMC_EV_PPCPNV_CYCLES_RUNNING_PURR_INC,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC4,
	    .pe_code = 0xf4
	},
	{PMC_EV_PPCPNV_BRANCH_MISPREDICTED,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC4,
	    .pe_code = 0xf6
	},
	{PMC_EV_PPCPNV_PREFETCHED_INSTRS_DISCARDED,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC4,
	    .pe_code = 0xf8
	},
	{PMC_EV_PPCPNV_INSTR_COMPLETED_RUNNING,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC4,
	    .pe_code = 0xfa
	},
	{PMC_EV_PPCPNV_TLB_INSTR_MISS,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC4,
	    .pe_code = 0xfc
	},
	{PMC_EV_PPCPNV_CACHE_LOAD_MISS,
	    .pe_flags = PMC_PPCPNV_FLAG_PMC4,
	    .pe_code = 0xfe
	}
};
static size_t ppcpnv_event_codes_size = nitems(ppcpnv_event_codes);

static pmc_value_t
ppcpnv_pmcn_read(unsigned int pmc)
{
	pmc_value_t val;

	switch (pmc) {
	case 0:
		val = mfspr(SPR_PNVPMC1);
		break;
	case 1:
		val = mfspr(SPR_PNVPMC2);
		break;
	case 2:
		val = mfspr(SPR_PNVPMC3);
		break;
	case 3:
		val = mfspr(SPR_PNVPMC4);
		break;
	case 4:
		val = mfspr(SPR_PNVPMC5);
		break;
	case 5:
		val = mfspr(SPR_PNVPMC6);
		break;
	default:
		panic("Invalid PMC number: %d\n", pmc);
	}

	return (val);
}

static void
ppcpnv_pmcn_write(unsigned int pmc, uint32_t val)
{
	switch (pmc) {
	case 0:
		mtspr(SPR_PNVPMC1, val);
		break;
	case 1:
		mtspr(SPR_PNVPMC2, val);
		break;
	case 2:
		mtspr(SPR_PNVPMC3, val);
		break;
	case 3:
		mtspr(SPR_PNVPMC4, val);
		break;
	case 4:
		mtspr(SPR_PNVPMC5, val);
		break;
	case 5:
		mtspr(SPR_PNVPMC6, val);
		break;
	default:
		panic("Invalid PMC number: %d\n", pmc);
	}
}

static int
ppcpnv_config_pmc(int cpu, int ri, struct pmc *pm)
{
	struct pmc_hw *phw;

	PMCDBG3(MDP,CFG,1, "cpu=%d ri=%d pm=%p", cpu, ri, pm);

	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("%s:%d: illegal CPU value: %d", __func__, __LINE__, cpu));
	KASSERT(ri >= 0 && ri < PPCPNV_MAX_PMCS,
	    ("%s:%d: illegal row-index: %d", __func__, __LINE__, ri));

	phw = &powerpc_pcpu[cpu]->pc_ppcpmcs[ri];

	KASSERT(pm == NULL || phw->phw_pmc == NULL,
	    ("%s:%d: pm=%p phw->pm=%p pmc not unconfigured",
	    __func__, __LINE__, pm, phw->phw_pmc));

	phw->phw_pmc = pm;
	return (0);
}

static void
ppcpnv_set_pmc(int cpu, int ri, int config)
{
	register_t mmcr;

	/* Select event */
	switch (ri) {
	case 0:
	case 1:
	case 2:
	case 3:
		mmcr = mfspr(SPR_PNVMMCR1);
		mmcr &= ~SPR_PNVMMCR1_PMCNSEL_MASK(ri);
		mmcr |= SPR_PNVMMCR1_PMCNSEL(ri, config & ~POWERPC_PMC_ENABLE);
		mtspr(SPR_PNVMMCR1, mmcr);
		break;
	}

	/*
	 * By default, freeze counter in all states.
	 * If counter is being started, unfreeze it in selected states.
	 */
	mmcr = mfspr(SPR_PNVMMCR2) | SPR_PNVMMCR2_FCNHSP(ri);
	if (config != PMCPNVN_NONE) {
		if (config & POWERPC_PMC_USER_ENABLE)
			mmcr &= ~(SPR_PNVMMCR2_FCNP0(ri) |
			    SPR_PNVMMCR2_FCNP1(ri));
		if (config & POWERPC_PMC_KERNEL_ENABLE)
			mmcr &= ~(SPR_PNVMMCR2_FCNH(ri) |
			    SPR_PNVMMCR2_FCNS(ri));
	}
	mtspr(SPR_PNVMMCR2, mmcr);
}

static int
ppcpnv_start_pmc(int cpu, int ri)
{
	struct pmc *pm;

	PMCDBG2(MDP,STA,1, "cpu=%d ri=%d", cpu, ri);
	pm = powerpc_pcpu[cpu]->pc_ppcpmcs[ri].phw_pmc;
	ppcpnv_set_pmc(cpu, ri, pm->pm_md.pm_powerpc.pm_powerpc_evsel);
	return (0);
}

static int
ppcpnv_stop_pmc(int cpu, int ri)
{
	PMCDBG2(MDP,STO,1, "cpu=%d ri=%d", cpu, ri);
	ppcpnv_set_pmc(cpu, ri, PMCPNVN_NONE);
	return (0);
}

static int
ppcpnv_read_pmc(int cpu, int ri, pmc_value_t *v)
{
	struct pmc *pm;
	pmc_value_t p, r;

	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("%s:%d: illegal CPU value: %d", __func__, __LINE__, cpu));
	KASSERT(ri >= 0 && ri < PPCPNV_MAX_PMCS,
	    ("%s:%d: illegal row-index: %d", __func__, __LINE__, ri));

	pm = powerpc_pcpu[cpu]->pc_ppcpmcs[ri].phw_pmc;
	KASSERT(pm, ("%s:%d: cpu=%d ri=%d pmc not configured",
	    __func__, __LINE__, cpu, ri));

	p = ppcpnv_pmcn_read(ri);
	r = pm->pm_sc.pm_reloadcount;

	/*
	 * Check if p and r are in the expected range, to avoid returning
	 * wrong values when PMCs were not configured correctly.
	 * (e.g., r is too big or PMC was reseted)
	 *
	 * Also, after an interrupt occurs because of a PMC overflow, the PMC
	 * value is not always MAX_PMC_VALUE + 1, but may be a little above it.
	 * As hwpmc code panics when reading a value greater that r in
	 * sampling mode, use r as an upper limit.
	 */
	if (PMC_IS_SAMPLING_MODE(PMC_TO_MODE(pm)) &&
	    r <= PPCPNV_MAX_PMC_VALUE && p + r > PPCPNV_MAX_PMC_VALUE)
		*v = MIN(p + r - PPCPNV_MAX_PMC_VALUE - 1, r);
	else
		*v = p;

	PMCDBG3(MDP,REA,1, "cpu=%d ri=%d -> %jx", cpu, ri, (uintmax_t)*v);
	return (0);
}

static int
ppcpnv_write_pmc(int cpu, int ri, pmc_value_t v)
{
	struct pmc *pm;

	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("%s:%d: illegal CPU value: %d", __func__, __LINE__, cpu));
	KASSERT(ri >= 0 && ri < PPCPNV_MAX_PMCS,
	    ("%s:%d: illegal row-index: %d", __func__, __LINE__, ri));

	pm = powerpc_pcpu[cpu]->pc_ppcpmcs[ri].phw_pmc;

	/*
	 * Allow -1 (~0UL) in counting mode, as pmcstat uses it as the
	 * initial PMC value.
	 */
	if (v == ~0UL && PMC_IS_COUNTING_MODE(PMC_TO_MODE(pm)))
		v = 0;
	else if (v > PPCPNV_MAX_PMC_VALUE) {
		PMCDBG3(MDP,WRI,2, "cpu=%d ri=%d: PMC value is too big: %jx",
		    cpu, ri, (uintmax_t)v);
		return (EINVAL);
	}

	PMCDBG3(MDP,WRI,1, "cpu=%d ri=%d -> %jx", cpu, ri, (uintmax_t)v);
	if (PMC_IS_SAMPLING_MODE(PMC_TO_MODE(pm)))
		v = POWERPC_RELOAD_COUNT_TO_PERFCTR_VALUE(v);

	ppcpnv_pmcn_write(ri, v);
	return (0);
}

static int
ppcpnv_intr(struct trapframe *tf)
{
	struct pmc *pm;
	struct powerpc_cpu *pc;
	int cpu, error, i, retval;
	register_t mmcr0;

	cpu = curcpu;
	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("%s:%d: illegal CPU value: %d", __func__, __LINE__, cpu));

	PMCDBG3(MDP,INT,1, "cpu=%d tf=%p um=%d", cpu, (void *) tf,
	    TRAPF_USERMODE(tf));

	retval = 0;
	pc = powerpc_pcpu[cpu];

	/*
	 * Look for a running, sampling PMC which has overflowed
	 * and which has a valid 'struct pmc' association.
	 */
	for (i = 0; i < PPCPNV_MAX_PMCS; i++) {
		if (!PPCPNV_PMC_HAS_OVERFLOWED(i))
			continue;
		retval = 1;

		/*
		 * Always clear the PMC, to make it stop interrupting.
		 * If pm is available and in sampling mode, use reload
		 * count, to make PMC read after stop correct.
		 * Otherwise, just reset the PMC.
		 */
		if ((pm = pc->pc_ppcpmcs[i].phw_pmc) != NULL &&
		    PMC_IS_SAMPLING_MODE(PMC_TO_MODE(pm))) {
			if (pm->pm_state != PMC_STATE_RUNNING) {
				ppcpnv_write_pmc(cpu, i,
				    pm->pm_sc.pm_reloadcount);
				continue;
			}
		} else {
			ppcpnv_pmcn_write(i, 0);
			continue;
		}

		error = pmc_process_interrupt(PMC_HR, pm, tf);
		if (error != 0) {
			PMCDBG3(MDP,INT,2,
			    "cpu=%d ri=%d: error %d processing interrupt",
			    cpu, i, error);
			ppcpnv_stop_pmc(cpu, i);
		}

		/* Reload sampling count */
		ppcpnv_write_pmc(cpu, i, pm->pm_sc.pm_reloadcount);
	}

	if (retval)
		counter_u64_add(pmc_stats.pm_intr_processed, 1);
	else
		counter_u64_add(pmc_stats.pm_intr_ignored, 1);

	/*
	 * Unfreeze counters, clear PMAO and re-enable PERF exceptions if we
	 * were able to find the interrupt source and handle it. Otherwise,
	 * it's better to disable PERF interrupts, to avoid the risk of
	 * processing the same interrupt forever.
	 */
	mmcr0 = mfspr(SPR_PNVMMCR0);
	mmcr0 &= ~(SPR_MMCR0_FC | SPR_PNVMMCR0_PMAO | SPR_PNVMMCR0_PMAE);
	if (retval)
		mmcr0 |= SPR_PNVMMCR0_PMAE;
	else
		log(LOG_WARNING,
		    "pmc_intr: couldn't find interrupting PMC on cpu %d - "
		    "disabling PERF interrupts\n", cpu);
	mtspr(SPR_PNVMMCR0, mmcr0);

	return (retval);
}

static int
ppcpnv_pcpu_init(struct pmc_mdep *md, int cpu)
{
	struct pmc_cpu *pmc;
	struct powerpc_cpu *pc;
	struct pmc_hw  *phw;
	int first_ri, i;
	register_t mmcr0;

	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("%s:%d: illegal CPU value: %d", __func__, __LINE__, cpu));
	PMCDBG1(MDP,INI,1, "pcpu_init cpu=%d", cpu);

	powerpc_pcpu[cpu] = pc = malloc(sizeof(struct powerpc_cpu), M_PMC,
	    M_WAITOK | M_ZERO);
	pc->pc_ppcpmcs = malloc(sizeof(struct pmc_hw) * PPCPNV_MAX_PMCS,
	    M_PMC, M_WAITOK | M_ZERO);
	pc->pc_class = PMC_CLASS_PPCPNV;

	pmc = pmc_pcpu[cpu];
	first_ri = md->pmd_classdep[PMC_MDEP_CLASS_INDEX_POWERPC].pcd_ri;
	KASSERT(pmc != NULL, ("%s:%d: NULL per-cpu pointer",
	    __func__, __LINE__));

	for (i = 0, phw = pc->pc_ppcpmcs; i < PPCPNV_MAX_PMCS; i++, phw++) {
		phw->phw_state = PMC_PHW_FLAG_IS_ENABLED |
		    PMC_PHW_CPU_TO_STATE(cpu) | PMC_PHW_INDEX_TO_STATE(i);
		phw->phw_pmc = NULL;
		pmc->pc_hwpmcs[i + first_ri] = phw;
	}

	/* Freeze all counters before modifying PMC registers */
	mmcr0 = mfspr(SPR_PNVMMCR0) | SPR_MMCR0_FC;
	mtspr(SPR_PNVMMCR0, mmcr0);

	/*
	 * Now setup MMCR0:
	 *  - PMAO=0: clear alerts
	 *  - FCPC=0, FCP=0: don't freeze counters in problem state
	 *  - FCECE: Freeze Counters on Enabled Condition or Event
	 *  - PMC1CE/PMCNCE: PMC1/N Condition Enable
	 */
	mmcr0 &= ~(SPR_PNVMMCR0_PMAO | SPR_PNVMMCR0_FCPC | SPR_MMCR0_FCP);
	mmcr0 |= SPR_MMCR0_FCECE | SPR_MMCR0_PMC1CE | SPR_MMCR0_PMCNCE;
	mtspr(SPR_PNVMMCR0, mmcr0);

	/* Clear all PMCs to prevent enabled condition interrupts */
	for (i = 0; i < PPCPNV_MAX_PMCS; i++)
		ppcpnv_pmcn_write(i, 0);

	/* Disable events in PMCs 1-4 */
	mtspr(SPR_PNVMMCR1, mfspr(SPR_PNVMMCR1) & ~SPR_PNVMMCR1_PMCSEL_ALL);

	/* Freeze each counter, in all states */
	mtspr(SPR_PNVMMCR2, mfspr(SPR_PNVMMCR2) |
	    SPR_PNVMMCR2_FCNHSP(0) | SPR_PNVMMCR2_FCNHSP(1) |
	    SPR_PNVMMCR2_FCNHSP(2) | SPR_PNVMMCR2_FCNHSP(3) |
	    SPR_PNVMMCR2_FCNHSP(4) | SPR_PNVMMCR2_FCNHSP(5));

	/* Enable interrupts, unset global freeze */
	mmcr0 &= ~SPR_MMCR0_FC;
	mmcr0 |= SPR_PNVMMCR0_PMAE;
	mtspr(SPR_PNVMMCR0, mmcr0);
	return (0);
}

static int
ppcpnv_pcpu_fini(struct pmc_mdep *md, int cpu)
{
	register_t mmcr0;

	PMCDBG1(MDP,INI,1, "pcpu_fini cpu=%d", cpu);

	/* Freeze counters, disable interrupts */
	mmcr0 = mfspr(SPR_PNVMMCR0);
	mmcr0 &= ~SPR_PNVMMCR0_PMAE;
	mmcr0 |= SPR_MMCR0_FC;
	mtspr(SPR_PNVMMCR0, mmcr0);

	free(powerpc_pcpu[cpu]->pc_ppcpmcs, M_PMC);
	free(powerpc_pcpu[cpu], M_PMC);

	return (0);
}

static int
ppcpnv_allocate_pmc(int cpu, int ri, struct pmc *pm,
  const struct pmc_op_pmcallocate *a)
{
	enum pmc_event pe;
	uint32_t caps, config = 0, counter = 0;
	int i;

	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("%s:%d: illegal CPU value: %d", __func__, __LINE__, cpu));
	KASSERT(ri >= 0 && ri < PPCPNV_MAX_PMCS,
	    ("%s:%d: illegal row-index: %d", __func__, __LINE__, ri));

	caps = a->pm_caps;
	pe = a->pm_ev;

	if (pe < PMC_EV_PPCPNV_FIRST || pe > PMC_EV_PPCPNV_LAST)
		return (EINVAL);

	for (i = 0; i < ppcpnv_event_codes_size; i++) {
		if (ppcpnv_event_codes[i].pe_event == pe) {
			config = ppcpnv_event_codes[i].pe_code;
			counter = ppcpnv_event_codes[i].pe_flags;
			break;
		}
	}
	if (i == ppcpnv_event_codes_size)
		return (EINVAL);

	if ((counter & (1 << ri)) == 0)
		return (EINVAL);

	if (caps & PMC_CAP_SYSTEM)
		config |= POWERPC_PMC_KERNEL_ENABLE;
	if (caps & PMC_CAP_USER)
		config |= POWERPC_PMC_USER_ENABLE;
	if ((caps & (PMC_CAP_USER | PMC_CAP_SYSTEM)) == 0)
		config |= POWERPC_PMC_ENABLE;

	pm->pm_md.pm_powerpc.pm_powerpc_evsel = config;

	PMCDBG3(MDP,ALL,1, "cpu=%d ri=%d -> config=0x%x", cpu, ri, config);
	return (0);
}

static int
ppcpnv_release_pmc(int cpu, int ri, struct pmc *pmc)
{
	struct pmc_hw *phw;

	PMCDBG2(MDP,REL,1, "cpu=%d ri=%d", cpu, ri);
	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("%s:%d: illegal CPU value: %d", __func__, __LINE__, cpu));
	KASSERT(ri >= 0 && ri < PPCPNV_MAX_PMCS,
	    ("%s:%d: illegal row-index: %d", __func__, __LINE__, ri));

	phw = &powerpc_pcpu[cpu]->pc_ppcpmcs[ri];
	KASSERT(phw->phw_pmc == NULL,
	    ("%s:%d: PHW pmc %p non-NULL", __func__, __LINE__, phw->phw_pmc));
	return (0);
}

int
pmc_ppcpnv_initialize(struct pmc_mdep *pmc_mdep)
{
	struct pmc_classdep *pcd;

	pmc_mdep->pmd_cputype = PMC_CPU_PPC_POWERNV;

	pcd = &pmc_mdep->pmd_classdep[PMC_MDEP_CLASS_INDEX_POWERPC];
	pcd->pcd_caps  = POWERPC_PMC_CAPS;
	pcd->pcd_class = PMC_CLASS_PPCPNV;
	pcd->pcd_num   = PPCPNV_MAX_PMCS;
	pcd->pcd_ri    = pmc_mdep->pmd_npmc;
	pcd->pcd_width = 32;

	pcd->pcd_pcpu_init      = ppcpnv_pcpu_init;
	pcd->pcd_pcpu_fini      = ppcpnv_pcpu_fini;
	pcd->pcd_allocate_pmc   = ppcpnv_allocate_pmc;
	pcd->pcd_release_pmc    = ppcpnv_release_pmc;
	pcd->pcd_start_pmc      = ppcpnv_start_pmc;
	pcd->pcd_stop_pmc       = ppcpnv_stop_pmc;
	pcd->pcd_get_config     = powerpc_get_config;
	pcd->pcd_config_pmc     = ppcpnv_config_pmc;
	pcd->pcd_describe       = powerpc_describe;
	pcd->pcd_read_pmc       = ppcpnv_read_pmc;
	pcd->pcd_write_pmc      = ppcpnv_write_pmc;

	pmc_mdep->pmd_npmc     += PPCPNV_MAX_PMCS;
	pmc_mdep->pmd_intr      = ppcpnv_intr;

	return (0);
}
