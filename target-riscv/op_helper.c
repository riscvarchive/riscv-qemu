/*
 *  RISC-V emulation helpers for qemu.
 *
 *  Author: Sagar Karandikar, skarandikar@berkeley.edu
 *  Based on the MIPS target
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
void csr_write_helper(CPURISCVState *env, target_ulong val_to_write, target_ulong csrno);
target_ulong csr_read_helper(CPURISCVState *env, target_ulong csrno);

#endif

#define RISCV_RM ({ if(rm == 7) rm = env->helper_csr[CSR_FRM]; \
                    /* TODO: throw trap for rm > 4 */ \
                    rm; })
                    
#define set_fp_exceptions ({ env->helper_csr[CSR_FFLAGS] |= softfloat_exceptionFlags;\
                             softfloat_exceptionFlags = 0; })

/* Exceptions processing helpers */
static inline void QEMU_NORETURN do_raise_exception_err(CPURISCVState *env,
                                                        uint32_t exception,
                                                        uintptr_t pc)
{
    CPUState *cs = CPU(riscv_env_get_cpu(env));
    qemu_log("%s: %d\n", __func__, exception);
    cs->exception_index = exception;
    if (pc) {
        /* now we have a real cpu fault */
        cpu_restore_state(cs, pc);
    }
    cpu_loop_exit(cs);
}

void helper_raise_exception(CPURISCVState *env, uint32_t exception)
{
    do_raise_exception_err(env, exception, 0);
}

/* floating point */
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

uint64_t helper_fsub_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = f64_mulAdd(frs1, 0x3ff0000000000000ULL, frs2 ^ (uint64_t)INT64_MIN);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fmul_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = f64_mulAdd(frs1, frs2, (frs1 ^ frs2) & (uint64_t)INT64_MIN);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fdiv_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = f64_div(frs1, frs2);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fsgnj_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = (frs1 &~ INT64_MIN) | (frs2 & INT64_MIN);
    return frs1;
}

uint64_t helper_fsgnjn_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = (frs1 &~ INT64_MIN) | ((~frs2) & INT64_MIN);
    return frs1;
}

uint64_t helper_fsgnjx_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = frs1 ^ (frs2 & INT64_MIN);
    return frs1;
}

uint64_t helper_fmin_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = isNaNF64UI(frs2) || f64_lt_quiet(frs1, frs2) ? frs1 : frs2;
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fmax_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = isNaNF64UI(frs2) || f64_lt_quiet(frs2, frs1) ? frs1 : frs2;
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_s_d(CPURISCVState *env, uint64_t rs1, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    rs1 = f64_to_f32(rs1);
    set_fp_exceptions;
    return rs1;
}

uint64_t helper_fcvt_d_s(CPURISCVState *env, uint64_t rs1, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    rs1 = f32_to_f64(rs1);
    set_fp_exceptions;
    return rs1;
}

uint64_t helper_fsqrt_d(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = f64_sqrt(frs1);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fle_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = f64_le(frs1, frs2);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_flt_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = f64_lt(frs1, frs2);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_feq_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = f64_eq(frs1, frs2);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_w_d(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = (int64_t)((int32_t)f64_to_i32(frs1, RISCV_RM, true));
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_wu_d(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = (int64_t)((int32_t)f64_to_ui32(frs1, RISCV_RM, true));
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_l_d(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = f64_to_i64(frs1, RISCV_RM, true);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_lu_d(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = f64_to_ui64(frs1, RISCV_RM, true);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_d_w(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = i32_to_f64((int32_t)frs1);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_d_wu(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = ui32_to_f64((uint32_t)frs1);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_d_l(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = i64_to_f64(frs1);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_d_lu(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    softfloat_roundingMode = RISCV_RM;
    frs1 = ui64_to_f64(frs1);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fclass_d(CPURISCVState *env, uint64_t frs1)
{
    frs1 = f64_classify(frs1);
    return frs1;
}

target_ulong helper_mulhsu(CPURISCVState *env, target_ulong arg1,
                          target_ulong arg2)
{
    int64_t a = arg1;
    uint64_t b = arg2;
    return (int64_t)((__int128_t)a*b >> 64);
}

inline void csr_write_helper(CPURISCVState *env, target_ulong val_to_write, target_ulong csrno)
{

    switch (csrno) {
        case CSR_STATUS:
            env->helper_csr[CSR_STATUS] = (val_to_write & ~(SR_IP)) | (env->helper_csr[CSR_STATUS] & SR_IP);
            break;
        case CSR_COUNT:
            cpu_riscv_store_count(env, (uint32_t)val_to_write);
            break;
        case CSR_COMPARE:
            cpu_riscv_store_compare(env, (uint32_t)val_to_write);
            break;
        case CSR_CYCLE:
            // DO NOT WRITE TO CSR_CYCLE
            break;
        case CSR_FCSR:
            env->helper_csr[CSR_FFLAGS] = val_to_write & 0x1F;
            env->helper_csr[CSR_FRM] = (val_to_write >> 5) & 0x7;
            break;
        default:
            env->helper_csr[csrno] = val_to_write;
            break;
    }
}

inline target_ulong csr_read_helper(CPURISCVState *env, target_ulong csrno)
{
    switch (csrno) {
        case CSR_COUNT:
            return cpu_riscv_get_count(env);
            break;
        case CSR_CYCLE:
            return cpu_riscv_get_cycle(env);
            break;
        case CSR_FCSR:
            return env->helper_csr[CSR_FFLAGS] | (env->helper_csr[CSR_FRM] << 5);
            break;
        default:
            return env->helper_csr[csrno];
            break;
    }
}

target_ulong helper_csrrw(CPURISCVState *env, target_ulong src, target_ulong csr)
{
    uint64_t csr_backup = csr_read_helper(env, csr);
    csr_write_helper(env, src, csr);
    return csr_backup;
}

target_ulong helper_csrrs(CPURISCVState *env, target_ulong src, target_ulong csr)
{
    uint64_t csr_backup = csr_read_helper(env, csr);
    csr_write_helper(env, src | csr_backup, csr);
    return csr_backup;
}

target_ulong helper_csrrc(CPURISCVState *env, target_ulong src, target_ulong csr) {
    uint64_t csr_backup = csr_read_helper(env, csr);
    csr_write_helper(env, (~src) & csr_backup, csr);
    return csr_backup;
}

target_ulong helper_sret(CPURISCVState *env)
{
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

target_ulong helper_scall(CPURISCVState *env, target_ulong bad_pc)
{
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

#ifndef CONFIG_USER_ONLY
/* TLB management */
inline static void cpu_riscv_tlb_flush (CPURISCVState *env, int flush_global)
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

// todo implement for RISC-V?
/*void helper_wait(CPURISCVState *env)
{
    CPUState *cs = CPU(riscv_env_get_cpu(env));

    cs->halted = 1;
    cpu_reset_interrupt(cs, CPU_INTERRUPT_WAKE);
    printf("NOT IMPLEMENTED FOR RISCV\n");
    exit(1);
}*/

#if !defined(CONFIG_USER_ONLY)

static void /*QEMU_NORETURN*/ do_unaligned_access(CPURISCVState *env,
                                              target_ulong addr, int rw,
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
                                int rw, int is_user, uintptr_t retaddr)
{
    CPUState *cs = CPU(riscv_env_get_cpu(env));
    if (rw & 0x2) {
        cs->exception_index = RISCV_EXCP_INST_ADDR_MIS;
    } else if (rw == 0x1) {
        cs->exception_index = RISCV_EXCP_STORE_ADDR_MIS;
        env->helper_csr[CSR_BADVADDR] = addr;
    } else {
        cs->exception_index = RISCV_EXCP_LOAD_ADDR_MIS;
        env->helper_csr[CSR_BADVADDR] = addr;
    }
    do_raise_exception_err(env, cs->exception_index, 0);
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
        do_raise_exception_err(env, cs->exception_index, retaddr);
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
