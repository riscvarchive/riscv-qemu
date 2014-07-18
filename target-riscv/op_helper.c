/*
 *  RISCV emulation helpers for qemu.
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

// custom floating point includes. use this instead of qemu's included 
// fpu/softmmu since we know it already works exactly as desired for riscv 
#include "fpu-custom-riscv/softfloat.h"
#include "fpu-custom-riscv/platform.h"
#include "fpu-custom-riscv/internals.h"

#ifndef CONFIG_USER_ONLY
static inline void cpu_riscv_tlb_flush (CPURISCVState *env, int flush_global);
#endif

#define RISCV_RM ({ if(rm == 7) rm = env->helper_csr[CSR_FRM]; \
                    /* TODO: throw trap for rm > 4 */ \
                    rm; })
                    
#define set_fp_exceptions ({ env->helper_csr[CSR_FFLAGS] |= softfloat_exceptionFlags;\
                             softfloat_exceptionFlags = 0; })


/*****************************************************************************/
/* Exceptions processing helpers */

static inline void QEMU_NORETURN do_raise_exception_err(CPURISCVState *env,
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

static inline void QEMU_NORETURN do_raise_exception(CPURISCVState *env,
                                                    uint32_t exception,
                                                    uintptr_t pc)
{
    do_raise_exception_err(env, exception, 0, pc);
}

void helper_raise_exception_err(CPURISCVState *env, uint32_t exception,
                                int error_code)
{
    do_raise_exception_err(env, exception, error_code, 0);
}

void helper_raise_exception(CPURISCVState *env, uint32_t exception)
{
    do_raise_exception(env, exception, 0);
}

uint64_t helper_fmadd_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t frs3, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = f32_mulAdd(frs1, frs2, frs3);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fmadd_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t frs3, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = f64_mulAdd(frs1, frs2, frs3);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fmsub_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t frs3, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = f32_mulAdd(frs1, frs2, frs3 ^ (uint32_t)INT32_MIN);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fmsub_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t frs3, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = f64_mulAdd(frs1, frs2, frs3 ^ (uint64_t)INT64_MIN);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fnmsub_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t frs3, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = f32_mulAdd(frs1 ^ (uint32_t)INT32_MIN, frs2, frs3);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fnmsub_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t frs3, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = f64_mulAdd(frs1 ^ (uint64_t)INT64_MIN, frs2, frs3);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fnmadd_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t frs3, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = f32_mulAdd(frs1 ^ (uint32_t)INT32_MIN, frs2, frs3 ^ (uint32_t)INT32_MIN);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fnmadd_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t frs3, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = f64_mulAdd(frs1 ^ (uint64_t)INT64_MIN, frs2, frs3 ^ (uint64_t)INT64_MIN);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fadd_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = f32_mulAdd(frs1, 0x3f800000, frs2);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fsub_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = f32_mulAdd(frs1, 0x3f800000, frs2 ^ (uint32_t)INT32_MIN);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fmul_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = f32_mulAdd(frs1, frs2, (frs1 ^ frs2) & (uint32_t)INT32_MIN);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fdiv_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = f32_div(frs1, frs2);
    set_fp_exceptions;
    return frs1;
}


uint64_t helper_fsgnj_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = (frs1 &~ (uint32_t)INT32_MIN) | (frs2 & (uint32_t)INT32_MIN);
    return frs1;
}

uint64_t helper_fsgnjn_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = (frs1 &~ (uint32_t)INT32_MIN) | ((~frs2) & (uint32_t)INT32_MIN);
    return frs1;
}

uint64_t helper_fsgnjx_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = frs1 ^ (frs2 & (uint32_t)INT32_MIN);
    return frs1;
}

uint64_t helper_fmin_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = isNaNF32UI(frs2) || f32_lt_quiet(frs1, frs2) ? frs1 : frs2;
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fmax_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = isNaNF32UI(frs2) || f32_lt_quiet(frs2, frs1) ? frs1 : frs2;
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fsqrt_s(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = f32_sqrt(frs1);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fle_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = f32_le(frs1, frs2);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_flt_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = f32_lt(frs1, frs2);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_feq_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = f32_eq(frs1, frs2);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_w_s(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = (int64_t)((int32_t)f32_to_i32(frs1, RISCV_RM, true));
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_wu_s(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = (int64_t)((int32_t)f32_to_ui32(frs1, RISCV_RM, true));
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_l_s(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = f32_to_i64(frs1, RISCV_RM, true);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_lu_s(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = f32_to_ui64(frs1, RISCV_RM, true);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_s_w(CPURISCVState *env, uint64_t rs1, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    rs1 = i32_to_f32((int32_t)rs1);
    set_fp_exceptions;
    return rs1;
}

uint64_t helper_fcvt_s_wu(CPURISCVState *env, uint64_t rs1, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    rs1 = ui32_to_f32((uint32_t)rs1);
    set_fp_exceptions;
    return rs1;
}

uint64_t helper_fcvt_s_l(CPURISCVState *env, uint64_t rs1, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    rs1 = i64_to_f32(rs1);
    set_fp_exceptions;
    return rs1;
}

uint64_t helper_fcvt_s_lu(CPURISCVState *env, uint64_t rs1, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    rs1 = ui64_to_f32(rs1);
    set_fp_exceptions;
    return rs1;
}

uint64_t helper_fclass_s(CPURISCVState *env, uint64_t frs1)
{
    frs1 = f32_classify(frs1);
    return frs1;
}










uint64_t helper_fadd_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = f64_mulAdd(frs1, 0x3ff0000000000000ULL, frs2);
    set_fp_exceptions;
    return frs1;
}














/* MODIFIED FOR RISCV*/
target_ulong helper_mulhsu(CPURISCVState *env, target_ulong arg1,
                          target_ulong arg2)
{
    int64_t a = arg1;
    uint64_t b = arg2;
    return (int64_t)((__int128_t)a*b >> 64);
}

target_ulong helper_csrrw(CPURISCVState *env, target_ulong src, target_ulong csr) {
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

target_ulong helper_csrrs(CPURISCVState *env, target_ulong src, target_ulong csr) {
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

target_ulong helper_csrrc(CPURISCVState *env, target_ulong src, target_ulong csr) {
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

target_ulong helper_sret(CPURISCVState *env) {
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

target_ulong helper_scall(CPURISCVState *env, target_ulong bad_pc) {
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
target_ulong helper_read_cycle(CPURISCVState *env) 
{
    uint32_t val = (int32_t)cpu_riscv_get_cycle(env);
    return val;
}
*/
target_ulong helper_read_count(CPURISCVState *env)
{
    uint32_t val = (int32_t)cpu_riscv_get_count(env);
//    printf("got count val: %d\n", val);
    return val;
}

void helper_store_compare(CPURISCVState *env, target_ulong arg1)
{
    cpu_riscv_store_compare(env, arg1);
}

void helper_store_count(CPURISCVState *env, target_ulong arg1)
{
    cpu_riscv_store_count(env, arg1);
}



#ifndef CONFIG_USER_ONLY
/* TLB management */
static void cpu_riscv_tlb_flush (CPURISCVState *env, int flush_global)
{
    RISCVCPU *cpu = riscv_env_get_cpu(env);

    /* Flush qemu's TLB and discard all shadowed entries.  */
    tlb_flush(CPU(cpu), flush_global);
}


void helper_tlb_flush(CPURISCVState *env)
{
    cpu_riscv_tlb_flush(env, 1);
}


#endif /* !CONFIG_USER_ONLY */

void helper_wait(CPURISCVState *env)
{
    CPUState *cs = CPU(riscv_env_get_cpu(env));

    cs->halted = 1;
    cpu_reset_interrupt(cs, CPU_INTERRUPT_WAKE);
    printf("NOT IMPLEMENTED FOR RISCV\n");
    exit(1);
}

#if !defined(CONFIG_USER_ONLY)

static void /*QEMU_NORETURN*/ do_unaligned_access(CPURISCVState *env,
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

static void do_unaligned_access(CPURISCVState *env, target_ulong addr,
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
        RISCVCPU *cpu = RISCV_CPU(cs);
        CPURISCVState *env = &cpu->env;

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
