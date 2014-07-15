/*
 *  MIPS emulation helpers for qemu.
 *
 *  Copyright (c) 2004-2005 Jocelyn Mayer
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
#include <stdlib.h>
#include "cpu.h"
#include "qemu/host-utils.h"

#include "helper.h"

#if !defined(CONFIG_USER_ONLY)
#include "exec/softmmu_exec.h"
#endif /* !defined(CONFIG_USER_ONLY) */

#ifndef CONFIG_USER_ONLY
static inline void cpu_riscv_tlb_flush (CPUMIPSState *env, int flush_global);
#endif

/*****************************************************************************/
/* Exceptions processing helpers */

static inline void QEMU_NORETURN do_raise_exception_err(CPUMIPSState *env,
                                                        uint32_t exception,
                                                        int error_code,
                                                        uintptr_t pc)
{
    CPUState *cs = CPU(riscv_env_get_cpu(env));

    qemu_log("%s: %d %d\n", __func__, exception, error_code);

    cs->exception_index = exception;

    if (pc) {
        /* now we have a real cpu fault */
        cpu_restore_state(cs, pc);
    }

    cpu_loop_exit(cs);
}

static inline void QEMU_NORETURN do_raise_exception(CPUMIPSState *env,
                                                    uint32_t exception,
                                                    uintptr_t pc)
{
    do_raise_exception_err(env, exception, 0, pc);
}

void helper_raise_exception_err(CPUMIPSState *env, uint32_t exception,
                                int error_code)
{
    do_raise_exception_err(env, exception, error_code, 0);
}

void helper_raise_exception(CPUMIPSState *env, uint32_t exception)
{
    do_raise_exception(env, exception, 0);
}

/* MODIFIED FOR RISCV*/
target_ulong helper_mulhsu(CPUMIPSState *env, target_ulong arg1,
                          target_ulong arg2)
{
    int64_t a = arg1;
    uint64_t b = arg2;
    return (int64_t)((__int128_t)a*b >> 64);
}

target_ulong helper_csrrw(CPUMIPSState *env, target_ulong src, target_ulong csr) {
    if (csr != CSR_COUNT && csr != CSR_COMPARE) {
        uint64_t csr_backup = env->helper_csr[csr]; 
        env->helper_csr[csr] = src;
        return csr_backup;
    } else if (csr == CSR_COUNT) {
        uint64_t csr_backup = cpu_riscv_get_count(env);
        cpu_riscv_store_count(env, (uint32_t)src);
        return csr_backup;
    } else if (csr == CSR_COMPARE) {
        uint64_t csr_backup = env->helper_csr[CSR_COMPARE];
        cpu_riscv_store_compare(env, (uint32_t)src);
        return csr_backup;
    } else if (csr == CSR_CYCLE) {
        return cpu_riscv_get_cycle(env);
    }
    return 0;
}

target_ulong helper_csrrs(CPUMIPSState *env, target_ulong src, target_ulong csr) {
    if (csr != CSR_COUNT && csr != CSR_COMPARE && csr != CSR_CYCLE) {
        uint64_t csr_backup = env->helper_csr[csr]; 
        env->helper_csr[csr] = csr_backup | src;
        return csr_backup;
    } else if (csr == CSR_COUNT) {
        uint64_t csr_backup = cpu_riscv_get_count(env);
        cpu_riscv_store_count(env, (uint32_t)(csr_backup | src));
        return csr_backup;
    } else if (csr == CSR_COMPARE) {
        uint64_t csr_backup = env->helper_csr[CSR_COMPARE];
        cpu_riscv_store_compare(env, (uint32_t)(csr_backup | src));
        return csr_backup;
    } else if (csr == CSR_CYCLE) {
        return cpu_riscv_get_cycle(env);
    }
    return 0;
}

target_ulong helper_csrrc(CPUMIPSState *env, target_ulong src, target_ulong csr) {
    if (csr != CSR_COUNT && csr != CSR_COMPARE && csr != CSR_CYCLE) {
        uint64_t csr_backup = env->helper_csr[csr]; 
        env->helper_csr[csr] = csr_backup & (~src);
        return csr_backup;
    } else if (csr == CSR_COUNT) {
        uint64_t csr_backup = cpu_riscv_get_count(env);
        cpu_riscv_store_count(env, (uint32_t)(csr_backup & (~src)));
        return csr_backup;
    } else if (csr == CSR_COMPARE) {
        uint64_t csr_backup = env->helper_csr[CSR_COMPARE];
        cpu_riscv_store_compare(env, (uint32_t)(csr_backup & (~src)));
        return csr_backup;
    } else if (csr == CSR_CYCLE) {
        return cpu_riscv_get_cycle(env);
    }
    return 0;
}

target_ulong helper_sret(CPUMIPSState *env) {
    // first handle S/PS stack
    if (env->helper_csr[CSR_STATUS] & SR_PS) {
        env->helper_csr[CSR_STATUS] |= SR_S;
    } else {
        env->helper_csr[CSR_STATUS] &= ~((uint64_t)SR_S);
    }

    // handle EI/PEI stack
    if (env->helper_csr[CSR_STATUS] & SR_PEI) {
        env->helper_csr[CSR_STATUS] |= SR_EI;
    } else {
        env->helper_csr[CSR_STATUS] &= ~((uint64_t)SR_EI);
    }

    // finally, return EPC value to set cpu_PC
    return env->helper_csr[CSR_EPC];
}

target_ulong helper_scall(CPUMIPSState *env, target_ulong bad_pc) {
    env->helper_csr[CSR_CAUSE] = RISCV_EXCP_SCALL;

    if (env->helper_csr[CSR_STATUS] & SR_S) {
        env->helper_csr[CSR_STATUS] |= SR_PS;
    } else {
        env->helper_csr[CSR_STATUS] &= ~((uint64_t)SR_PS);
    }
    env->helper_csr[CSR_STATUS] |= SR_S;

    if (env->helper_csr[CSR_STATUS] & SR_EI) {
        env->helper_csr[CSR_STATUS] |= SR_PEI;
    } else {
        env->helper_csr[CSR_STATUS] &= ~((uint64_t)SR_PEI);
    }
    env->helper_csr[CSR_STATUS] &= ~((uint64_t)SR_EI);

    env->helper_csr[CSR_EPC] = bad_pc;

    return env->helper_csr[CSR_EVEC];
}
/*
target_ulong helper_read_cycle(CPUMIPSState *env) 
{
    uint32_t val = (int32_t)cpu_riscv_get_cycle(env);
    return val;
}
*/
target_ulong helper_read_count(CPUMIPSState *env)
{
    uint32_t val = (int32_t)cpu_riscv_get_count(env);
//    printf("got count val: %d\n", val);
    return val;
}

void helper_store_compare(CPUMIPSState *env, target_ulong arg1)
{
    cpu_riscv_store_compare(env, arg1);
}

void helper_store_count(CPUMIPSState *env, target_ulong arg1)
{
    cpu_riscv_store_count(env, arg1);
}



#ifndef CONFIG_USER_ONLY
/* TLB management */
static void cpu_riscv_tlb_flush (CPUMIPSState *env, int flush_global)
{
    MIPSCPU *cpu = riscv_env_get_cpu(env);

    /* Flush qemu's TLB and discard all shadowed entries.  */
    tlb_flush(CPU(cpu), flush_global);
}


void helper_tlb_flush(CPUMIPSState *env)
{
    cpu_riscv_tlb_flush(env, 1);
}


#endif /* !CONFIG_USER_ONLY */

void helper_wait(CPUMIPSState *env)
{
    CPUState *cs = CPU(riscv_env_get_cpu(env));

    cs->halted = 1;
    cpu_reset_interrupt(cs, CPU_INTERRUPT_WAKE);
    printf("NOT IMPLEMENTED FOR RISCV\n");
    exit(1);
}

#if !defined(CONFIG_USER_ONLY)

static void /*QEMU_NORETURN*/ do_unaligned_access(CPUMIPSState *env,
                                              target_ulong addr, int is_write,
                                              int is_user, uintptr_t retaddr);

#define MMUSUFFIX _mmu
#define ALIGNED_ONLY

#define SHIFT 0
#include "exec/softmmu_template.h"

#define SHIFT 1
#include "exec/softmmu_template.h"

#define SHIFT 2
#include "exec/softmmu_template.h"

#define SHIFT 3
#include "exec/softmmu_template.h"

static void do_unaligned_access(CPUMIPSState *env, target_ulong addr,
                                int is_write, int is_user, uintptr_t retaddr)
{
    printf("REACHED DO UNALIGNED ACCESS\n");
    printf("%016lX\n", (uint64_t)addr);
    exit(1);
}

/* called by qemu's softmmu to fill the qemu tlb */
void tlb_fill(CPUState *cs, target_ulong addr, int is_write, int mmu_idx,
              uintptr_t retaddr)
{
    int ret;

    ret = riscv_cpu_handle_mmu_fault(cs, addr, is_write, mmu_idx);
    if (ret) {
        MIPSCPU *cpu = MIPS_CPU(cs);
        CPUMIPSState *env = &cpu->env;

        do_raise_exception_err(env, cs->exception_index,
                               0, retaddr);
    }
}

void riscv_cpu_unassigned_access(CPUState *cs, hwaddr addr,
                                bool is_write, bool is_exec, int unused,
                                unsigned size)
{
    printf("unassigned address was called?\n");
    printf("with addr: %016lX\n", addr);

    printf("not implemented for riscv\n");
    exit(1);
}
#endif /* !CONFIG_USER_ONLY */
