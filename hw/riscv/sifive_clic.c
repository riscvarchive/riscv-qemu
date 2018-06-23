/*
 * SiFive CLIC (Core Local Interrupt Controller)
 *
 * Copyright (c) 2016-2017 Sagar Karandikar, sagark@eecs.berkeley.edu
 * Copyright (c) 2017-2018 SiFive, Inc.
 *
 * This provides real-time clock, timer, interprocessor interrupts
 * and pre-emptable local interrupts.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "target/riscv/cpu.h"
#include "sysemu/sysemu.h"
#include "hw/riscv/sifive_clic.h"
#include "hw/riscv/sifive_clint.h"
#include "qemu/timer.h"
#include "trace.h"

static const bool debug = false;

static void intcfg_decode(SiFiveCLICState *clic, int hartid, int intcfg,
                          int *mode, int *level, int *priority)
{
    int nmbits = clic->nmbits[hartid];
    int nlbits = clic->nlbits[hartid];
    int npbits = clic->npbits[hartid];
    int nmshift = SIFIVE_CLIC_MAX_INT_BITS - nmbits;
    int nlshift = SIFIVE_CLIC_MAX_INT_BITS - nmbits - nlbits;
    int npshift = SIFIVE_CLIC_MAX_INT_BITS - nmbits - nlbits - npbits;
    int decoded_mode =     (intcfg >> nmshift) & ((1 << nmbits) - 1);
    int decoded_level =    (intcfg >> nlshift) & ((1 << nlbits) - 1);
    int decoded_priority = (intcfg >> npshift) & ((1 << npbits) - 1);

    switch (nmbits) {
    case 0: /* if nmbits == 0; then mode is PRV_M */
        *mode = PRV_M;
        break;
    case 1: /* if nmbits == 1; then mode = intcfg[8] ? PRV_M : PRV_U */
        *mode = decoded_mode ? PRV_M : PRV_U;
        break;
    case 2: /* if nmbits == 2, then mode = intcfg[8:7]*/
        *mode = decoded_mode;
        break;
    }

    /* unused level bits are set to 1 */
    *level = decoded_level << (SIFIVE_CLIC_MAX_LEVEL_BITS - nlbits) |
        ((1 << (SIFIVE_CLIC_MAX_LEVEL_BITS - nlbits)) - 1);

    /* unused priority bits are set to 1 */
    *priority = decoded_priority << (SIFIVE_CLIC_MAX_PRIORITY_BITS - nlbits) |
        ((1 << (SIFIVE_CLIC_MAX_PRIORITY_BITS - npbits)) - 1);
}

static void sifive_clic_next_interrupt(SiFiveCLICState *clic, int hartid)
{
    /*
     * Scan active list for highest priority pending interrupts
     * comparing against this harts mintstatus register and interrupt
     * the core if we have a higher priority interrupt to deliver
     */
    RISCVCPU *cpu = RISCV_CPU(qemu_get_cpu(hartid));
    CPURISCVState *env = &cpu->env;

    int il[4] = {
        get_field(env->mintstatus, MINTSTATUS_UIL), /* PRV_U */
        get_field(env->mintstatus, MINTSTATUS_SIL), /* PRV_S */
        0,                                          /* reserverd */
        get_field(env->mintstatus, MINTSTATUS_MIL)  /* PRV_M */
    };

    /* get sorted list of enabled interrupts for this hart */
    size_t hart_offset = hartid * clic->num_sources;
    CLICActiveInterrupt *active = &clic->active_list[hart_offset];
    size_t active_count = clic->active_count[hartid];
    int mode = 0, level = 0, priority = 0;

    /* loop through the enabled interrupts sorted by mode+priority+level */
    while (active_count) {
        intcfg_decode(clic, hartid, active->intcfg, &mode, &level, &priority);
        if (mode < env->priv || (mode == env->priv && level <= il[mode])) {
            /* no pending interrupts with high enough mode+priority+level
             * break and clear pending interrupt for this hart*/
            break;
        }
        /* check pending interrupt with high enough mode+priority+level */
        if (clic->clicintip[hartid * clic->num_sources + active->irq]) {
            /* post pending interrupt for this hart */
            riscv_cpu_clic_interrupt(cpu, active->irq | mode<<10 | level<<12);
            return;
        }
        /* check next enabled interrupt */
        active_count--;
        active++;
    }

    /* clear pending interrupt for this hart */
    riscv_cpu_clic_interrupt(cpu, -1);
}

static void sifive_clic_update_intip(SiFiveCLICState *clic, int mode,
                                     int hartid, int irq, int new_intip);

static void sifive_clic_clint_irq(RISCVCPU *cpu, int irq, int level)
{
    /*
     * CLIC vs CLINT timer/software interrupt dispatch
     *
     * - check mtvec/stvec for CLINT mode vs CLIC mode
     * - CLINT mode uses mip/mie
     *   - riscv_cpu_update_mip(cpu, irq, BOOL_TO_MASK(level));
     * - CLIC mode uses clicintie/clicintip
     *   - sifive_clic_update_intip(clic, mode, hartid, irq, level);
     *   - sifive_clic_next_interrupt(clic, hartid);
     */

    CPURISCVState *env = &cpu->env;
    SiFiveCLICState *clic = env->clic;
    uint32_t mmode_clic_mask = BOOL_TO_MASK((env->mtvec & 0b111110) == 0b10);
    uint32_t smode_clic_mask = BOOL_TO_MASK((env->stvec & 0b111110) == 0b10);
    uint32_t mmode_intrs = (MIP_MTIP | MIP_MSIP | MIP_MEIP);
    uint32_t smode_intrs = (MIP_STIP | MIP_SSIP | MIP_SEIP);
    uint32_t mmode_clic_intrs = mmode_intrs & mmode_clic_mask;
    uint32_t smode_clic_intrs = smode_intrs & smode_clic_mask;

    if ((irq & mmode_clic_intrs) || (irq & smode_clic_intrs)) {
        sifive_clic_update_intip(clic, PRV_M, env->mhartid, ctz32(irq), level);
    } else {
        riscv_cpu_update_mip(cpu, irq, BOOL_TO_MASK(level));
    }
}

typedef QEMUTimer *(*timer_fn)(CPURISCVState *env);
typedef uint64_t *(*timecmp_fn)(CPURISCVState *env);

static QEMUTimer *mtimer(CPURISCVState *env) { return env->mtimer; }
static QEMUTimer *stimer(CPURISCVState *env) { return env->stimer; }

static uint64_t *mtimecmp(CPURISCVState *env) { return &env->mtimecmp; }
static uint64_t *stimecmp(CPURISCVState *env) { return &env->stimecmp; }

static void sifive_clic_mtimecmp_cb(void *cpu)
{
    sifive_clic_clint_irq(cpu, MIP_MTIP, 1);
}

static void sifive_clic_stimecmp_cb(void *cpu)
{
    sifive_clic_clint_irq(cpu, MIP_STIP, 1);
}

static uint64_t cpu_riscv_read_rtc(void)
{
    return muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
        SIFIVE_CLINT_TIMEBASE_FREQ, NANOSECONDS_PER_SECOND);
}

static void sifive_clic_write_timecmp(RISCVCPU *cpu, uint64_t value,
                                      timecmp_fn timecmp, timer_fn timer,
                                      uint32_t timerirq)
{
    CPURISCVState *env = &cpu->env;

    uint64_t rtc = cpu_riscv_read_rtc();
    uint64_t cmp = *timecmp(env) = value;
    uint64_t diff = cmp - rtc;
    uint64_t next_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
        muldiv64(diff, NANOSECONDS_PER_SECOND, SIFIVE_CLINT_TIMEBASE_FREQ);

    if (cmp <= rtc) {
        /* if we're setting a timecmp value in the "past",
           immediately raise the timer interrupt */
        sifive_clic_clint_irq(cpu, timerirq, 1);
    } else {
        /* otherwise, set up the future timer interrupt */
        sifive_clic_clint_irq(cpu, timerirq, 0);
        timer_mod(timer(env), next_ns);
    }
}

static uint64_t sifive_clic_clint_read(SiFiveCLICState *clic, hwaddr addr,
                                       unsigned size, timecmp_fn timecmp)
{
    /* reads must be 4 byte aligned words */
    if ((addr & 0x3) != 0 || size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
            "clic: invalid read size %u: 0x%" HWADDR_PRIx "\n", size, addr);
        return 0;
    }

    if (addr >= clic->sip_base &&
        addr < clic->sip_base + (clic->num_harts << 2)) {
        size_t hartid = (addr - clic->sip_base) >> 2;
        CPUState *cpu = qemu_get_cpu(hartid);
        CPURISCVState *env = cpu ? cpu->env_ptr : NULL;
        if (!env) {
            qemu_log_mask(LOG_GUEST_ERROR,
                "clic: invalid sip hartid: %zu\n", hartid);
        } else if ((addr & 0x3) == 0) {
            return (env->mip & MIP_MSIP) > 0;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                "clic: invalid sip read: 0x%" HWADDR_PRIx "\n", addr);
            return 0;
        }
    } else if (addr >= clic->timecmp_base &&
        addr < clic->timecmp_base + (clic->num_harts << 3)) {
        size_t hartid = (addr - clic->timecmp_base) >> 3;
        CPUState *cpu = qemu_get_cpu(hartid);
        CPURISCVState *env = cpu ? cpu->env_ptr : NULL;
        if (!env) {
            qemu_log_mask(LOG_GUEST_ERROR,
                "clic: invalid timecmp hartid: %zu\n", hartid);
        } else if ((addr & 0x7) == 0) {
            /* timecmp_lo */
            return *timecmp(env) & 0xFFFFFFFF;
        } else if ((addr & 0x7) == 4) {
            /* timecmp_hi */
            return (*timecmp(env) >> 32) & 0xFFFFFFFF;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                "clic: invalid read: 0x%" HWADDR_PRIx "\n", addr);
            return 0;
        }
    } else if (addr == clic->time_base) {
        /* time_lo */
        return cpu_riscv_read_rtc() & 0xFFFFFFFF;
    } else if (addr == clic->time_base + 4) {
        /* time_hi */
        return (cpu_riscv_read_rtc() >> 32) & 0xFFFFFFFF;
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
            "clic: invalid read: 0x%" HWADDR_PRIx "\n", addr);
    }

    return 0;
}

static void sifive_clic_clint_write(SiFiveCLICState *clic, hwaddr addr,
                                    uint64_t value, unsigned size,
                                    timecmp_fn timecmp, timer_fn timer,
                                    uint32_t timerirq, uint32_t softirq)
{
    /* writes must be 4 byte aligned words */
    if ((addr & 0x3) != 0 || size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
            "clic: invalid write size %u: 0x%" HWADDR_PRIx "\n", size, addr);
        return;
    }

    if (addr >= clic->sip_base &&
        addr < clic->sip_base + (clic->num_harts << 2)) {
        size_t hartid = (addr - clic->sip_base) >> 2;
        CPUState *cpu = qemu_get_cpu(hartid);
        CPURISCVState *env = cpu ? cpu->env_ptr : NULL;
        if (!env) {
            qemu_log_mask(LOG_GUEST_ERROR,
                "clic: invalid sip hartid: %zu\n", hartid);
        } else if ((addr & 0x3) == 0) {
            sifive_clic_clint_irq(RISCV_CPU(cpu), softirq, !!value);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                "clic: invalid sip write: 0x%" HWADDR_PRIx "\n", addr);
        }
        return;
    } else if (addr >= clic->timecmp_base &&
        addr < clic->timecmp_base + (clic->num_harts << 3)) {
        size_t hartid = (addr - clic->timecmp_base) >> 3;
        CPUState *cpu = qemu_get_cpu(hartid);
        CPURISCVState *env = cpu ? cpu->env_ptr : NULL;
        if (!env) {
            qemu_log_mask(LOG_GUEST_ERROR,
                "clic: invalid timecmp hartid: %zu\n", hartid);
        } else if ((addr & 0x7) == 0) {
            /* timecmp_lo */
            uint64_t timecmp_hi = *timecmp(env) >> 32;
            sifive_clic_write_timecmp(RISCV_CPU(cpu),
                timecmp_hi << 32 | (value & 0xFFFFFFFF),
                timecmp, timer, timerirq);
            return;
        } else if ((addr & 0x7) == 4) {
            /* timecmp_hi */
            uint64_t timecmp_lo = *timecmp(env);
            sifive_clic_write_timecmp(RISCV_CPU(cpu),
                value << 32 | (timecmp_lo & 0xFFFFFFFF),
                timecmp, timer, timerirq);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                "clic: invalid timecmp write: 0x%" HWADDR_PRIx "\n", addr);
        }
        return;
    } else if (addr == clic->time_base) {
        /* time_lo */
        qemu_log_mask(LOG_UNIMP, "clic: time_lo write not implemented\n");
    } else if (addr == clic->time_base + 4) {
        /* time_hi */
        qemu_log_mask(LOG_UNIMP, "clic: time_hi write not implemented");
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
            "clic: invalid write: 0x%" HWADDR_PRIx "\n", addr);
    }
}

static uint64_t sifive_clic_hart_read(SiFiveCLICState *clic, hwaddr addr,
                                      unsigned size, int mode, int hartid)
{
    int req = extract32(addr, 10, 2);
    int irq = extract32(addr, 0, 10);
    size_t irq_offset = hartid * clic->num_sources + irq;

    if (hartid >= clic->num_harts) {
        qemu_log_mask(LOG_GUEST_ERROR,
            "clic: invalid hartid %u: 0x%" HWADDR_PRIx "\n", hartid, addr);
        return 0;
    }

    if (irq >= clic->num_sources) {
        qemu_log_mask(LOG_GUEST_ERROR,
            "clic: invalid irq %u: 0x%" HWADDR_PRIx "\n", irq, addr);
        return 0;
    }

    switch (req) {
    case 0: /* clicintip[i] */
        /* TODO - need to check mode has access to pending bit */
        return clic->clicintip[irq_offset];
    case 1: /* clicintie[i] */
        /* TODO - need to check mode has access to enable bit */
        return clic->clicintie[irq_offset];
    case 2: /* clicintcfg[i] */
        /* TODO - need to check mode has access to config bit */
        return clic->clicintcfg[irq_offset];
    case 3: /* cliccfg */
        if (irq == 0 && mode == PRV_M) {
            return clic->nvbits[hartid] |
                   (clic->nlbits[hartid] << 1) |
                   (clic->nmbits[hartid] << 4);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                "clic: invalid cliccfg read: 0x%" HWADDR_PRIx "\n", addr);
            return 0;
        }
        break;
    }

    return 0;
}

static inline int sifive_clic_encode_priority(const CLICActiveInterrupt *i)
{
    return ((0xff - i->intcfg)         << 12) |/* highest mode+level+priority */
           (((0x3ff - i->irq) & 0x3ff) << 0);  /* highest irq number */
}

static int sifive_clic_active_compare(const void *a, const void *b)
{
    return sifive_clic_encode_priority(a) - sifive_clic_encode_priority(b);
}

static void sifive_clic_print_active_irqs(SiFiveCLICState *clic)
{
    int hartid, i;
    for (hartid = 0; hartid < clic->num_harts; hartid++) {
        size_t hart_offset = hartid * clic->num_sources;
        CLICActiveInterrupt *active_list = &clic->active_list[hart_offset];
        size_t *active_count = &clic->active_count[hartid];
        for (i = 0; i < *active_count; i++) {
            CLICActiveInterrupt *active = active_list + i;
            printf("hartid=%d intcfg=0x%02hhx, irq=%d\n",
                hartid, active->intcfg, active->irq);
        }
    }
}

static bool sifive_clic_validate_intip(SiFiveCLICState *clic, int mode,
                                       int hartid, int irq, int new_intcfg)
{
    /*
     * TODO - current implementation allows software control of pending bits
     * TODO - need to check whether the mode has access to this pending bit
     */

    return true;
}

static void sifive_clic_update_intip(SiFiveCLICState *clic, int mode,
                                     int hartid, int irq, int new_intip)
{
    size_t irq_offset = hartid * clic->num_sources + irq;
    clic->clicintip[irq_offset] = !!new_intip;
    trace_sifive_clic_intip(mode, hartid, irq, new_intip);
    sifive_clic_next_interrupt(clic, hartid);
}

static bool sifive_clic_validate_intie(SiFiveCLICState *clic, int mode,
                                       int hartid, int irq, int new_intcfg)
{
    /*
     * TODO - need to check whether the mode has access to this enable bit
     */

    return true;
}

static void sifive_clic_update_intie(SiFiveCLICState *clic, int mode,
                                     int hartid, int irq, int new_intie)
{
    size_t hart_offset = hartid * clic->num_sources;
    size_t irq_offset = hartid * clic->num_sources + irq;
    CLICActiveInterrupt *active_list = &clic->active_list[hart_offset];
    size_t *active_count = &clic->active_count[hartid];

    uint8_t old_intie = clic->clicintie[irq_offset];
    clic->clicintie[irq_offset] = !!new_intie;

    /* add to or remove from list of active interrupts */
    if (new_intie && !old_intie) {
        active_list[*active_count].intcfg = clic->clicintcfg[irq_offset];
        active_list[*active_count].irq = irq;
        (*active_count)++;
    } else if (!new_intie && old_intie) {
        CLICActiveInterrupt key = {
            clic->clicintcfg[irq_offset], irq
        };
        CLICActiveInterrupt *result = bsearch(&key,
                                              active_list, *active_count,
                                              sizeof(CLICActiveInterrupt),
                                              sifive_clic_active_compare);
        size_t elem = (result - active_list) / sizeof(CLICActiveInterrupt);
        size_t sz = (--(*active_count) - elem) * sizeof(CLICActiveInterrupt);
        assert(result);
        memmove(&result[0], &result[1], sz);
    }

    /* sort list of active interrupts */
    qsort(active_list, *active_count,
          sizeof(CLICActiveInterrupt),
          sifive_clic_active_compare);

    trace_sifive_clic_intie(mode, hartid, irq, new_intie);
    sifive_clic_next_interrupt(clic, hartid);
}

static bool sifive_clic_validate_intcfg(SiFiveCLICState *clic, int mode,
                                        int hartid, int irq, int intcfg)
{
    int nmbits = clic->nmbits[hartid], nmshift = 8 - nmbits;
    int decoded_mode = (intcfg >> nmshift) & ((1 << nmbits) - 1);

    switch (nmbits) {
    case 0: /* if nmbits == 0; then PRV_M <= mode */
        return PRV_M <= mode;
    case 1: /* if nmbits == 1; then intcfg[8] ? PRV_M : PRV_U <= mode */
        return (decoded_mode ? PRV_M : PRV_U) <= mode;
    case 2: /* if nmbits == 2; then intcfg[8:7] <= mode */
        return decoded_mode <= mode;
    }
    return false;
}

static void sifive_clic_update_intcfg(SiFiveCLICState *clic, int mode,
                                      int hartid, int irq, int new_intcfg)
{
    size_t hart_offset = hartid * clic->num_sources;
    size_t irq_offset = hartid * clic->num_sources + irq;
    CLICActiveInterrupt *active_list = &clic->active_list[hart_offset];
    size_t *active_count = &clic->active_count[hartid];

    new_intcfg &= (((1 << clic->int_bits) - 1) << (8 - clic->int_bits));

    uint8_t old_intcfg = clic->clicintcfg[irq_offset];
    clic->clicintcfg[irq_offset] = new_intcfg;

    /* sort list of active interrupts based on new intcfg */
    if (clic->clicintie[irq_offset] && old_intcfg != new_intcfg) {
        CLICActiveInterrupt key = { old_intcfg, irq };
        CLICActiveInterrupt *result = bsearch(&key,
                                              active_list, *active_count,
                                              sizeof(CLICActiveInterrupt),
                                              sifive_clic_active_compare);
        assert(result);
        result->intcfg = new_intcfg;
        qsort(active_list, *active_count,
              sizeof(CLICActiveInterrupt),
              sifive_clic_active_compare);
    }

    trace_sifive_clic_intcfg(mode, hartid, irq, new_intcfg);
    sifive_clic_next_interrupt(clic, hartid);
}

static void sifive_clic_hart_write(SiFiveCLICState *clic, hwaddr addr,
                                   uint64_t value, unsigned size,
                                   int mode, int hartid)
{
    int req = extract32(addr, 10, 2);
    int irq = extract32(addr, 0, 10);

    if (hartid >= clic->num_harts) {
        qemu_log_mask(LOG_GUEST_ERROR,
            "clic: invalid hartid %u: 0x%" HWADDR_PRIx "\n", hartid, addr);
        return;
    }

    if (irq >= clic->num_sources) {
        qemu_log_mask(LOG_GUEST_ERROR,
            "clic: invalid irq %u: 0x%" HWADDR_PRIx "\n", irq, addr);
        return;
    }

    switch (req) {
    case 0: /* clicintip[i] */
        if (sifive_clic_validate_intip(clic, mode, hartid, irq, value)) {
            sifive_clic_update_intip(clic, mode, hartid, irq, value);
        }
        break;
    case 1: /* clicintie[i] */
        if (sifive_clic_validate_intie(clic, mode, hartid, irq, value)) {
            sifive_clic_update_intie(clic, mode, hartid, irq, value);
        }
        if (debug) {
            sifive_clic_print_active_irqs(clic);
        }
        break;
    case 2: /* clicintcfg[i] */
        if (sifive_clic_validate_intcfg(clic, mode, hartid, irq, value)) {
            sifive_clic_update_intcfg(clic, mode, hartid, irq, value);
        }
        if (debug) {
            sifive_clic_print_active_irqs(clic);
        }
        break;
    case 3: /* cliccfg */
        if (irq == 0 && mode == PRV_M) {
            clic->nvbits[hartid] = MIN(extract32(value, 0, 1),
                                       clic->vec_bits);
            clic->nlbits[hartid] = MIN(extract32(value, 1, 3),
                                       clic->level_bits);
            clic->nmbits[hartid] = MIN(extract32(value, 4, 2),
                                       clic->mode_bits);
            trace_sifive_clic_cfg(hartid,
                                  clic->nmbits[hartid],
                                  clic->nlbits[hartid],
                                  clic->nvbits[hartid]);
            clic->npbits[hartid] = clic->int_bits -
                                   clic->nmbits[hartid] -
                                   clic->nlbits[hartid];
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                "clic: invalid cliccfg write: 0x%" HWADDR_PRIx "\n", addr);
            return;
        }
        break;
    }
}

static uint64_t sifive_clic_read(void *opaque, hwaddr addr, unsigned size)
{
    SiFiveCLICState *clic = opaque;
    hwaddr clic_size = clic->num_harts * SIFIVE_CLIC_HART_SIZE;
    int hartid = 0;

    if (addr >= clic->clint_mmode_base &&
        addr < clic->clint_mmode_base + SIFIVE_CLIC_CLINT_SIZE) {
        /* M-mode CLINT aperture */
        addr -= clic->clint_mmode_base;
        return sifive_clic_clint_read(clic, addr, size, mtimecmp);
    } else if (addr >= clic->clint_smode_base &&
               addr < clic->clint_smode_base + SIFIVE_CLIC_CLINT_SIZE) {
        /* S-mode CLINT aperture */
        addr -= clic->clint_smode_base;
        return sifive_clic_clint_read(clic, addr, size, stimecmp);
    } else if (addr >= clic->clic_mmode_base &&
               addr < clic->clic_mmode_base + clic_size) {
        /* M-mode CLIC per hart apertures */
        addr -= clic->clic_mmode_base;
        hartid = addr / SIFIVE_CLIC_HART_SIZE;
        addr -= hartid * SIFIVE_CLIC_HART_SIZE;
        return sifive_clic_hart_read(clic, addr, size, PRV_M, hartid);
    } else if (addr >= clic->clic_smode_base &&
               addr < clic->clic_smode_base + clic_size) {
        /* S-mode CLIC per hart apertures */
        addr -= clic->clic_smode_base;
        hartid = addr / SIFIVE_CLIC_HART_SIZE;
        addr -= hartid * SIFIVE_CLIC_HART_SIZE;
        return sifive_clic_hart_read(clic, addr, size, PRV_S, hartid);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
            "clic: invalid read: 0x%" HWADDR_PRIx "\n", addr);
    }
    return 0;
}

static void sifive_clic_write(void *opaque, hwaddr addr, uint64_t value,
        unsigned size)
{
    SiFiveCLICState *clic = opaque;
    hwaddr clic_size = clic->num_harts * SIFIVE_CLIC_HART_SIZE;
    int hartid = 0;

    if (addr >= clic->clint_mmode_base &&
        addr < clic->clint_mmode_base + SIFIVE_CLIC_CLINT_SIZE) {
        /* M-mode CLINT aperture */
        addr -= clic->clint_mmode_base;
        sifive_clic_clint_write(clic, addr, value, size,
                                mtimecmp, mtimer, MIP_MTIP, MIP_MSIP);
    } else if (addr >= clic->clint_smode_base &&
               addr < clic->clint_smode_base + SIFIVE_CLIC_CLINT_SIZE) {
        /* S-mode CLINT aperture */
        addr -= clic->clint_smode_base;
        sifive_clic_clint_write(clic, addr, value, size,
                                stimecmp, stimer, MIP_STIP, MIP_SSIP);
    } else if (addr >= clic->clic_mmode_base &&
               addr < clic->clic_mmode_base + clic_size) {
        /* M-mode CLIC per hart apertures */
        addr -= clic->clic_mmode_base;
        hartid = addr / SIFIVE_CLIC_HART_SIZE;
        addr -= hartid * SIFIVE_CLIC_HART_SIZE;
        sifive_clic_hart_write(clic, addr, value, size, PRV_M, hartid);
    } else if (addr >= clic->clic_smode_base &&
               addr < clic->clic_smode_base + clic_size) {
        /* S-mode CLIC per hart apertures */
        addr -= clic->clic_smode_base;
        hartid = addr / SIFIVE_CLIC_HART_SIZE;
        addr -= hartid * SIFIVE_CLIC_HART_SIZE;
        sifive_clic_hart_write(clic, addr, value, size, PRV_S, hartid);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
            "clic: invalid write: 0x%" HWADDR_PRIx "\n", addr);
    }
}

static const MemoryRegionOps sifive_clic_ops = {
    .read = sifive_clic_read,
    .write = sifive_clic_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static Property sifive_clic_properties[] = {
    DEFINE_PROP_UINT32("num-harts", SiFiveCLICState, num_harts, 0),
    DEFINE_PROP_UINT32("sip-base", SiFiveCLICState, sip_base, 0),
    DEFINE_PROP_UINT32("timecmp-base", SiFiveCLICState, timecmp_base, 0),
    DEFINE_PROP_UINT32("time-base", SiFiveCLICState, time_base, 0),
    DEFINE_PROP_UINT32("aperture-size", SiFiveCLICState, aperture_size, 0),
    DEFINE_PROP_UINT32("num-sources", SiFiveCLICState, num_sources, 0),
    DEFINE_PROP_UINT8("int-bits", SiFiveCLICState, int_bits, 0),
    DEFINE_PROP_UINT8("mode-bits", SiFiveCLICState, mode_bits, 0),
    DEFINE_PROP_UINT8("level-bits", SiFiveCLICState, level_bits, 0),
    DEFINE_PROP_UINT8("vec-bits", SiFiveCLICState, vec_bits, 0),
    DEFINE_PROP_UINT32("clint-mmode-base", SiFiveCLICState, clint_mmode_base, 0),
    DEFINE_PROP_UINT32("clint-smode-base", SiFiveCLICState, clint_smode_base, 0),
    DEFINE_PROP_UINT32("clic-mmode-base", SiFiveCLICState, clic_mmode_base, 0),
    DEFINE_PROP_UINT32("clic-smode-base", SiFiveCLICState, clic_smode_base, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static int sifive_clic_encode_irq_id(int irq, int hartid)
{
    return ((irq & 0x3ff) << 10) | ((hartid & 0x3ff) << 0);
}

static void sifive_clic_decode_irq_id(int *irq, int *hartid, int id)
{
    *irq = (id >> 10) & 0x3ff;   /* 1024 interrupt sources */
    *hartid = (id >> 0) & 0x3ff; /* 1024 harts */
}

static void sifive_clic_irq(void *opaque, int id, int level)
{
    SiFiveCLICState *clic = opaque;
    int irq, hartid;
    sifive_clic_decode_irq_id(&hartid, &irq, id);
    trace_sifive_clic_irq(hartid, irq, level);
    sifive_clic_update_intip(clic, PRV_M, hartid, irq, level);
}

qemu_irq sifive_clic_get_irq(DeviceState *dev, int hartid, int irq)
{
    SiFiveCLICState *clic = SIFIVE_CLIC(dev);
    size_t irq_offset = hartid * clic->num_sources + irq;
    return clic->irqs[irq_offset];
}

static void sifive_clic_realize(DeviceState *dev, Error **errp)
{
    SiFiveCLICState *clic = SIFIVE_CLIC(dev);
    size_t harts_x_sources = clic->num_harts * clic->num_sources;
    int i, irq, hartid;

    memory_region_init_io(&clic->mmio, OBJECT(dev), &sifive_clic_ops, clic,
                          TYPE_SIFIVE_CLIC, clic->aperture_size);

    clic->clicintip = g_new0(uint8_t, harts_x_sources);
    clic->clicintie = g_new0(uint8_t, harts_x_sources);
    clic->clicintcfg = g_new0(uint8_t, harts_x_sources);
    clic->active_list = g_new0(CLICActiveInterrupt, harts_x_sources);
    clic->active_count = g_new0(size_t, clic->num_harts);
    clic->nmbits = g_new0(uint8_t, clic->num_harts);
    clic->nlbits = g_new0(uint8_t, clic->num_harts);
    clic->nvbits = g_new0(uint8_t, clic->num_harts);
    clic->npbits = g_new0(uint8_t, clic->num_harts);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &clic->mmio);
    clic->irqs = g_new0(qemu_irq, harts_x_sources);
    for (hartid = 0; hartid < clic->num_harts; hartid++) {
        for (irq = 0; irq < clic->num_sources; irq++) {
            int id = sifive_clic_encode_irq_id(hartid, irq);
            size_t irq_offset = hartid * clic->num_sources + irq;
            clic->irqs[irq_offset] = qemu_allocate_irq(sifive_clic_irq,
                                                       clic, id);
        }
    }

    /* The CLIC controls SSIP and STIP */
    for (i = 0; i < smp_cpus; i++) {
        RISCVCPU *cpu = RISCV_CPU(qemu_get_cpu(i));
        cpu->env.clic = clic;
        if (riscv_cpu_claim_interrupts(cpu, MIP_STIP | MIP_SSIP) < 0) {
            qemu_log_mask(LOG_GUEST_ERROR,
                "sifive_plic_realize: STIP and SSIP already claimed\n");
            exit(1);
        }
    }
}

static void sifive_clic_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = sifive_clic_realize;
    dc->props = sifive_clic_properties;
}

static const TypeInfo sifive_clic_info = {
    .name          = TYPE_SIFIVE_CLIC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SiFiveCLICState),
    .class_init    = sifive_clic_class_init,
};

static void sifive_clic_register_types(void)
{
    type_register_static(&sifive_clic_info);
}

type_init(sifive_clic_register_types)


/*
 * Create CLIC device.
 */
DeviceState *sifive_clic_create(hwaddr addr, hwaddr size,
    uint32_t num_harts,
    uint32_t sip_base,
    uint32_t timecmp_base,
    uint32_t time_base,
    uint32_t num_sources,
    uint8_t int_bits,
    uint8_t mode_bits,
    uint8_t level_bits,
    uint8_t vec_bits,
    uint32_t clint_mmode_base,
    uint32_t clint_smode_base,
    uint32_t clic_mmode_base,
    uint32_t clic_smode_base)
{
    int i;

    assert(num_sources >= SIFIVE_CLIC_MIN_SOURCES);
    assert(num_sources <= SIFIVE_CLIC_MAX_SOURCES);
    assert(int_bits >= SIFIVE_CLIC_MIN_INT_BITS);
    assert(int_bits <= SIFIVE_CLIC_MAX_INT_BITS);
    assert(mode_bits <= SIFIVE_CLIC_MAX_MODE_BITS);
    assert(level_bits <= SIFIVE_CLIC_MAX_LEVEL_BITS);
    assert(vec_bits <= SIFIVE_CLIC_MAX_VEC_BITS);

    DeviceState *dev = qdev_create(NULL, TYPE_SIFIVE_CLIC);

    for (i = 0; i < num_harts; i++) {
        RISCVCPU *cpu = RISCV_CPU(qemu_get_cpu(i));
        CPURISCVState *env = &cpu->env;
        env->features |= (1ULL << RISCV_FEATURE_CLIC);
        env->mtimer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                   &sifive_clic_mtimecmp_cb, cpu);
        env->stimer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                   &sifive_clic_stimecmp_cb, cpu);
        env->mtimecmp = 0;
        env->stimecmp = 0;
    }

    qdev_prop_set_uint32(dev, "num-harts", num_harts);
    qdev_prop_set_uint32(dev, "sip-base", sip_base);
    qdev_prop_set_uint32(dev, "timecmp-base", timecmp_base);
    qdev_prop_set_uint32(dev, "time-base", time_base);
    qdev_prop_set_uint32(dev, "aperture-size", size);
    qdev_prop_set_uint32(dev, "num-sources", num_sources);
    qdev_prop_set_uint8(dev, "int-bits", int_bits);
    qdev_prop_set_uint8(dev, "mode-bits", mode_bits);
    qdev_prop_set_uint8(dev, "level-bits", level_bits);
    qdev_prop_set_uint8(dev, "vec-bits", vec_bits);
    qdev_prop_set_uint32(dev, "clint-mmode-base", clint_mmode_base);
    qdev_prop_set_uint32(dev, "clint-smode-base", clint_smode_base);
    qdev_prop_set_uint32(dev, "clic-mmode-base", clic_mmode_base);
    qdev_prop_set_uint32(dev, "clic-smode-base", clic_smode_base);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, addr);
    return dev;
}
