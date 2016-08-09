/*
 * RISC-V Emulation Helpers for QEMU.
 *
 * Author: Sagar Karandikar, sagark@eecs.berkeley.edu
 *
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
#include "exec/helper-proto.h"

int validate_priv(target_ulong priv);

int validate_priv(target_ulong priv) {
    return priv == PRV_U || priv == PRV_S || priv == PRV_M;
}

// TODO duplicated in helper.c
void set_privilege(CPURISCVState *env, target_ulong newpriv);

void set_privilege(CPURISCVState *env, target_ulong newpriv) {
    if (!validate_priv(newpriv)) {
        printf("INVALID PRIV SET\n");
        exit(1);
    }
    helper_tlb_flush(env);
    env->priv = newpriv;
}

static int validate_vm(target_ulong vm) {
    return vm == VM_SV39 || vm == VM_SV48 || vm == VM_MBARE;
}

/* Exceptions processing helpers */
static inline void QEMU_NORETURN do_raise_exception_err(CPURISCVState *env,
                                          uint32_t exception, uintptr_t pc)
{
    CPUState *cs = CPU(riscv_env_get_cpu(env));
    qemu_log("%s: %d\n", __func__, exception);
    cs->exception_index = exception;
    cpu_loop_exit_restore(cs, pc);
}

void helper_raise_exception(CPURISCVState *env, uint32_t exception)
{
    do_raise_exception_err(env, exception, 0);
}

void helper_raise_exception_debug(CPURISCVState *env)
{
    do_raise_exception_err(env, EXCP_DEBUG, 0);
}


void helper_raise_exception_err(CPURISCVState *env, uint32_t exception, target_ulong pc)
{
    do_raise_exception_err(env, exception, pc);
}

void helper_raise_exception_mbadaddr(CPURISCVState *env, uint32_t exception,
        target_ulong bad_pc) {
    env->badaddr = bad_pc;
    do_raise_exception_err(env, exception, 0);
}

/* floating point */

/* convert RISC-V rounding mode to IEEE library numbers */
unsigned int ieee_rm[] = {
    float_round_nearest_even,
    float_round_to_zero,
    float_round_down,
    float_round_up,
    float_round_ties_away
};

// obtain rm value to use in computation
// as the last step, convert rm codes to what the softfloat library expects
#define RM ({ if (rm == 7) rm = env->csr[NEW_CSR_FRM]; \
              if (rm > 4) { /* TODO throw trap*/ }; \
              ieee_rm[rm]; })

unsigned int softfloat_flags_to_riscv(unsigned int flag);

/* convert softfloat library flag numbers to RISC-V 
 * TODO better way...
 */
unsigned int softfloat_flags_to_riscv(unsigned int flag) {
    switch (flag) {
        case float_flag_inexact:
            return 1;
        case float_flag_underflow:
            return 2;
        case float_flag_overflow:
            return 4;
        case float_flag_divbyzero:
            return 8;
        case float_flag_invalid:
            return 16;
        default:
            return 0;
    }
}

#define set_fp_exceptions ({ env->csr[NEW_CSR_FFLAGS] |= softfloat_flags_to_riscv(get_float_exception_flags(&env->fp_status)); \
                             set_float_exception_flags(0, &env->fp_status); })

uint64_t helper_fmadd_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t frs3, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = float32_muladd(frs1, frs2, frs3, 0, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fmadd_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t frs3, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = float64_muladd(frs1, frs2, frs3, 0, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fmsub_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t frs3, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = float32_muladd(frs1, frs2, frs3 ^ (uint32_t)INT32_MIN, 0, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fmsub_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t frs3, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = float64_muladd(frs1, frs2, frs3 ^ (uint64_t)INT64_MIN, 0, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fnmsub_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t frs3, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = float32_muladd(frs1 ^ (uint32_t)INT32_MIN, frs2, frs3, 0, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fnmsub_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t frs3, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = float64_muladd(frs1 ^ (uint64_t)INT64_MIN, frs2, frs3, 0, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fnmadd_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t frs3, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = float32_muladd(frs1 ^ (uint32_t)INT32_MIN, frs2, frs3 ^ (uint32_t)INT32_MIN, 0, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fnmadd_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t frs3, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = float64_muladd(frs1 ^ (uint64_t)INT64_MIN, frs2, frs3 ^ (uint64_t)INT64_MIN, 0, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fadd_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = float32_add(frs1, frs2, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fsub_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = float32_sub(frs1, frs2, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fmul_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = float32_mul(frs1, frs2, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fdiv_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = float32_div(frs1, frs2, &env->fp_status);
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
    frs1 = float32_is_any_nan(frs2) || float32_lt_quiet(frs1, frs2, &env->fp_status) ? frs1 : frs2;
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fmax_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = float32_is_any_nan(frs2) || float32_le_quiet(frs2, frs1, &env->fp_status) ? frs1 : frs2;
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fsqrt_s(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = float32_sqrt(frs1, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fle_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = float32_le(frs1, frs2, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_flt_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = float32_lt(frs1, frs2, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_feq_s(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = float32_eq(frs1, frs2, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_w_s(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = (int64_t)((int32_t)float32_to_int32(frs1, &env->fp_status));
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_wu_s(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = (int64_t)((int32_t)float32_to_uint32(frs1, &env->fp_status));
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_l_s(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = float32_to_int64(frs1, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_lu_s(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = float32_to_uint64(frs1, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_s_w(CPURISCVState *env, uint64_t rs1, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    rs1 = int32_to_float32((int32_t)rs1, &env->fp_status);
    set_fp_exceptions;
    return rs1;
}

uint64_t helper_fcvt_s_wu(CPURISCVState *env, uint64_t rs1, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    rs1 = uint32_to_float32((uint32_t)rs1, &env->fp_status);
    set_fp_exceptions;
    return rs1;
}

uint64_t helper_fcvt_s_l(CPURISCVState *env, uint64_t rs1, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    rs1 = int64_to_float32(rs1, &env->fp_status);
    set_fp_exceptions;
    return rs1;
}

uint64_t helper_fcvt_s_lu(CPURISCVState *env, uint64_t rs1, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    rs1 = uint64_to_float32(rs1, &env->fp_status);
    set_fp_exceptions;
    return rs1;
}

/* adapted from spike */
#define isNaNF32UI( ui ) (0xFF000000<(uint32_t)((uint_fast32_t)(ui)<<1))
#define signF32UI( a ) ((bool)((uint32_t)(a)>>31))
#define expF32UI( a ) ((int_fast16_t)((a)>>23)&0xFF)
#define fracF32UI( a ) ((a)&0x007FFFFF)

union ui32_f32 { uint32_t ui; uint32_t f; };

uint_fast16_t float32_classify( uint32_t a );

uint_fast16_t float32_classify( uint32_t a )
{
    union ui32_f32 uA;
    uint_fast32_t uiA;

    uA.f = a;
    uiA = uA.ui;

    uint_fast16_t infOrNaN = expF32UI( uiA ) == 0xFF;
    uint_fast16_t subnormalOrZero = expF32UI( uiA ) == 0;
    bool sign = signF32UI( uiA );

    return
        (  sign && infOrNaN && fracF32UI( uiA ) == 0 )          << 0 |
        (  sign && !infOrNaN && !subnormalOrZero )              << 1 |
        (  sign && subnormalOrZero && fracF32UI( uiA ) )        << 2 |
        (  sign && subnormalOrZero && fracF32UI( uiA ) == 0 )   << 3 |
        ( !sign && infOrNaN && fracF32UI( uiA ) == 0 )          << 7 |
        ( !sign && !infOrNaN && !subnormalOrZero )              << 6 |
        ( !sign && subnormalOrZero && fracF32UI( uiA ) )        << 5 |
        ( !sign && subnormalOrZero && fracF32UI( uiA ) == 0 )   << 4 |
        ( isNaNF32UI( uiA ) &&  float32_is_signaling_nan( uiA )) << 8 |
        ( isNaNF32UI( uiA ) && !float32_is_signaling_nan( uiA )) << 9;
}

uint64_t helper_fclass_s(CPURISCVState *env, uint64_t frs1)
{
    frs1 = float32_classify(frs1);
    return frs1;
}

uint64_t helper_fadd_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = float64_add(frs1, frs2, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fsub_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = float64_sub(frs1, frs2, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fmul_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = float64_mul(frs1, frs2, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fdiv_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = float64_div(frs1, frs2, &env->fp_status);
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
    frs1 = float64_is_any_nan(frs2) || float64_lt_quiet(frs1, frs2, &env->fp_status) ? frs1 : frs2;
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fmax_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = float64_is_any_nan(frs2) || float64_le_quiet(frs2, frs1, &env->fp_status) ? frs1 : frs2;
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_s_d(CPURISCVState *env, uint64_t rs1, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    rs1 = float64_to_float32(rs1, &env->fp_status);
    set_fp_exceptions;
    return rs1;
}

uint64_t helper_fcvt_d_s(CPURISCVState *env, uint64_t rs1, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    rs1 = float32_to_float64(rs1, &env->fp_status);
    set_fp_exceptions;
    return rs1;
}

uint64_t helper_fsqrt_d(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = float64_sqrt(frs1, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fle_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = float64_le(frs1, frs2, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_flt_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = float64_lt(frs1, frs2, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_feq_d(CPURISCVState *env, uint64_t frs1, uint64_t frs2)
{
    frs1 = float64_eq(frs1, frs2, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_w_d(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = (int64_t)((int32_t)float64_to_int32(frs1, &env->fp_status));
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_wu_d(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = (int64_t)((int32_t)float64_to_uint32(frs1, &env->fp_status));
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_l_d(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = float64_to_int64(frs1, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_lu_d(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = float64_to_uint64(frs1, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_d_w(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = int32_to_float64((int32_t)frs1, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_d_wu(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = uint32_to_float64((uint32_t)frs1, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_d_l(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = int64_to_float64(frs1, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

uint64_t helper_fcvt_d_lu(CPURISCVState *env, uint64_t frs1, uint64_t rm)
{
    set_float_rounding_mode(RM, &env->fp_status);
    frs1 = uint64_to_float64(frs1, &env->fp_status);
    set_fp_exceptions;
    return frs1;
}

/* adapted from spike */
#define isNaNF64UI( ui ) (UINT64_C(0xFFE0000000000000)<(uint64_t)((uint_fast64_t)(ui)<<1))
#define signF64UI( a ) ((bool)((uint64_t)(a)>>63))
#define expF64UI( a ) ((int_fast16_t)((a)>>52)&0x7FF)
#define fracF64UI( a ) ((a)&UINT64_C(0x000FFFFFFFFFFFFF))

union ui64_f64 { uint64_t ui; uint64_t f; };

uint_fast16_t float64_classify( uint64_t a );

uint_fast16_t float64_classify( uint64_t a )
{
    union ui64_f64 uA;
    uint_fast64_t uiA;

    uA.f = a;
    uiA = uA.ui;

    uint_fast16_t infOrNaN = expF64UI( uiA ) == 0x7FF;
    uint_fast16_t subnormalOrZero = expF64UI( uiA ) == 0;
    bool sign = signF64UI( uiA );

    return
        (  sign && infOrNaN && fracF64UI( uiA ) == 0 )          << 0 |
        (  sign && !infOrNaN && !subnormalOrZero )              << 1 |
        (  sign && subnormalOrZero && fracF64UI( uiA ) )        << 2 |
        (  sign && subnormalOrZero && fracF64UI( uiA ) == 0 )   << 3 |
        ( !sign && infOrNaN && fracF64UI( uiA ) == 0 )          << 7 |
        ( !sign && !infOrNaN && !subnormalOrZero )              << 6 |
        ( !sign && subnormalOrZero && fracF64UI( uiA ) )        << 5 |
        ( !sign && subnormalOrZero && fracF64UI( uiA ) == 0 )   << 4 |
        ( isNaNF64UI( uiA ) &&  float64_is_signaling_nan( uiA )) << 8 |
        ( isNaNF64UI( uiA ) && !float64_is_signaling_nan( uiA )) << 9;
}

uint64_t helper_fclass_d(CPURISCVState *env, uint64_t frs1)
{
    frs1 = float64_classify(frs1);
    return frs1;
}

target_ulong helper_mulhsu(CPURISCVState *env, target_ulong arg1,
                          target_ulong arg2)
{
    int64_t a = arg1;
    uint64_t b = arg2;
    return (int64_t)((__int128_t)a*b >> 64);
}

/*
 * Handle writes to CSRs and any resulting special behavior
 *
 * Note: mtohost and mfromhost are not handled here
 */
inline void csr_write_helper(CPURISCVState *env, target_ulong val_to_write,
        target_ulong csrno)
{
    #ifdef RISCV_DEBUG_PRINT
    fprintf(stderr, "Write CSR reg: 0x" TARGET_FMT_lx "\n", csrno);
    fprintf(stderr, "Write CSR val: 0x" TARGET_FMT_lx "\n", val_to_write);
    #endif

    uint64_t delegable_ints = MIP_SSIP | MIP_STIP | MIP_SEIP | (1 << IRQ_COP);
    uint64_t all_ints = delegable_ints | MIP_MSIP | MIP_MTIP;

    switch (csrno)
    {
    case NEW_CSR_FFLAGS:
        env->csr[NEW_CSR_MSTATUS] |= MSTATUS_FS | MSTATUS64_SD;
        env->csr[NEW_CSR_FFLAGS] = val_to_write & (FSR_AEXC >> FSR_AEXC_SHIFT);
        break;
    case NEW_CSR_FRM:
        env->csr[NEW_CSR_MSTATUS] |= MSTATUS_FS | MSTATUS64_SD;
        env->csr[NEW_CSR_FRM] = val_to_write & (FSR_RD >> FSR_RD_SHIFT);
        break;
    case NEW_CSR_FCSR:
        env->csr[NEW_CSR_MSTATUS] |= MSTATUS_FS | MSTATUS64_SD;
        env->csr[NEW_CSR_FFLAGS] = (val_to_write & FSR_AEXC) >> FSR_AEXC_SHIFT;
        env->csr[NEW_CSR_FRM] = (val_to_write & FSR_RD) >> FSR_RD_SHIFT;
        break;
    case NEW_CSR_MSTATUS: {
        target_ulong mstatus = env->csr[NEW_CSR_MSTATUS];
        if ((val_to_write ^ mstatus) &
            (MSTATUS_VM | MSTATUS_MPP | MSTATUS_MPRV | MSTATUS_PUM | MSTATUS_MXR)) {
            helper_tlb_flush(env);
        }

        // no extension support
        target_ulong mask = MSTATUS_SIE | MSTATUS_SPIE | MSTATUS_MIE | MSTATUS_MPIE
            | MSTATUS_SPP | MSTATUS_FS | MSTATUS_MPRV | MSTATUS_PUM
            | MSTATUS_MXR;

        if (validate_vm(get_field(val_to_write, MSTATUS_VM))) {
            mask |= MSTATUS_VM;
        }
        if (validate_priv(get_field(val_to_write, MSTATUS_MPP))) {
            mask |= MSTATUS_MPP;
        }

        mstatus = (mstatus & ~mask) | (val_to_write & mask);

        int dirty = (mstatus & MSTATUS_FS) == MSTATUS_FS;
        dirty |= (mstatus & MSTATUS_XS) == MSTATUS_XS;
        mstatus = set_field(mstatus, MSTATUS64_SD, dirty);
        env->csr[NEW_CSR_MSTATUS] = mstatus;
        break;
    }
    case NEW_CSR_MIP: {
        target_ulong mask = MIP_SSIP | MIP_STIP;
        env->csr[NEW_CSR_MIP] = (env->csr[NEW_CSR_MIP] & ~mask) |
            (val_to_write & mask);
        CPUState *cs = CPU(riscv_env_get_cpu(env));
        if (env->csr[NEW_CSR_MIP] & MIP_SSIP) {
            stw_phys(cs->as, 0xFFFFFFFFF0000020, 0x1);
        } else {
            stw_phys(cs->as, 0xFFFFFFFFF0000020, 0x0);
        }
        if (env->csr[NEW_CSR_MIP] & MIP_STIP) {
            stw_phys(cs->as, 0xFFFFFFFFF0000040, 0x1);
        } else {
            stw_phys(cs->as, 0xFFFFFFFFF0000040, 0x0);
        }
        if (env->csr[NEW_CSR_MIP] & MIP_MSIP) {
            stw_phys(cs->as, 0xFFFFFFFFF0000060, 0x1);
        } else {
            stw_phys(cs->as, 0xFFFFFFFFF0000060, 0x0);
        }
        break;
    }
    case NEW_CSR_MIE: {
        env->csr[NEW_CSR_MIE] = (env->csr[NEW_CSR_MIE] & ~all_ints) |
            (val_to_write & all_ints);
        break;
    }
    case NEW_CSR_MIDELEG:
        env->csr[NEW_CSR_MIDELEG] = (env->csr[NEW_CSR_MIDELEG] & ~delegable_ints) | (val_to_write & delegable_ints);
        break;
    case NEW_CSR_MEDELEG: {
        target_ulong mask = 0;
        mask |= 1ULL << (NEW_RISCV_EXCP_INST_ADDR_MIS);
        mask |= 1ULL << (NEW_RISCV_EXCP_INST_ACCESS_FAULT);
        mask |= 1ULL << (NEW_RISCV_EXCP_ILLEGAL_INST);
        mask |= 1ULL << (NEW_RISCV_EXCP_BREAKPOINT);
        mask |= 1ULL << (NEW_RISCV_EXCP_LOAD_ADDR_MIS);
        mask |= 1ULL << (NEW_RISCV_EXCP_LOAD_ACCESS_FAULT);
        mask |= 1ULL << (NEW_RISCV_EXCP_STORE_AMO_ADDR_MIS);
        mask |= 1ULL << (NEW_RISCV_EXCP_STORE_AMO_ACCESS_FAULT);
        mask |= 1ULL << (NEW_RISCV_EXCP_U_ECALL);
        mask |= 1ULL << (NEW_RISCV_EXCP_S_ECALL);
        mask |= 1ULL << (NEW_RISCV_EXCP_H_ECALL);
        mask |= 1ULL << (NEW_RISCV_EXCP_M_ECALL);
        env->csr[NEW_CSR_MEDELEG] = (env->csr[NEW_CSR_MEDELEG] & ~mask) | (val_to_write & mask);
        break;
    }
    case NEW_CSR_MUCOUNTEREN:
        env->csr[NEW_CSR_MUCOUNTEREN] = val_to_write & 7;
        break;
    case NEW_CSR_MSCOUNTEREN:
        env->csr[NEW_CSR_MSCOUNTEREN] = val_to_write & 7;
        break;
    case NEW_CSR_SSTATUS: {
        target_ulong ms = env->csr[NEW_CSR_MSTATUS];
        target_ulong mask = SSTATUS_SIE | SSTATUS_SPIE | SSTATUS_SPP | SSTATUS_FS
            | SSTATUS_XS | SSTATUS_PUM; // TODO should XS be here?
        ms = (ms & ~mask) | (val_to_write & mask);
        csr_write_helper(env, ms, NEW_CSR_MSTATUS);
        break;
    }
    case NEW_CSR_SIP: {
        target_ulong next_mip = (env->csr[NEW_CSR_MIP] & ~env->csr[NEW_CSR_MIDELEG]) | (val_to_write & env->csr[NEW_CSR_MIDELEG]);
        csr_write_helper(env, next_mip, NEW_CSR_MIP);
        // note: stw_phys should be done by the call to set MIP if necessary,
        // so we don't do it here
        break;
    }
    case NEW_CSR_SIE: {
        target_ulong next_mie = (env->csr[NEW_CSR_MIE] & ~env->csr[NEW_CSR_MIDELEG]) | (val_to_write & env->csr[NEW_CSR_MIDELEG]);
        csr_write_helper(env, next_mie, NEW_CSR_MIE);
        break;
    }
    case NEW_CSR_SPTBR: {
        env->csr[NEW_CSR_SPTBR] = val_to_write & (((target_ulong)1 << (50 - PGSHIFT)) - 1);
        break;
    }
    case NEW_CSR_SEPC:
        env->csr[NEW_CSR_SEPC] = val_to_write;
        break;
    case NEW_CSR_STVEC:
        env->csr[NEW_CSR_STVEC] = val_to_write >> 2 << 2;
        break;
    case NEW_CSR_SSCRATCH:
        env->csr[NEW_CSR_SSCRATCH] = val_to_write;
        break;
    case NEW_CSR_SCAUSE:
        env->csr[NEW_CSR_SCAUSE] = val_to_write;
        break;
    case NEW_CSR_SBADADDR:
        env->csr[NEW_CSR_SBADADDR] = val_to_write;
        break;
    case NEW_CSR_MEPC:
        env->csr[NEW_CSR_MEPC] = val_to_write;
        break;
    case NEW_CSR_MTVEC:
        env->csr[NEW_CSR_MTVEC] = val_to_write << 2 >> 2;
        break;
    case NEW_CSR_MSCRATCH:
        env->csr[NEW_CSR_MSCRATCH] = val_to_write;
        break;
    case NEW_CSR_MCAUSE:
        env->csr[NEW_CSR_MCAUSE] = val_to_write;
        break;
    case NEW_CSR_MBADADDR:
        env->csr[NEW_CSR_MBADADDR] = val_to_write;
        break;
    case NEW_CSR_DCSR:
        printf("DEBUG NOT SUPPORTED\n");
        exit(1);
        break;
    case NEW_CSR_DPC:
        printf("DEBUG NOT SUPPORTED\n");
        exit(1);
        break;
    case NEW_CSR_DSCRATCH:
        printf("DEBUG NOT SUPPORTED\n");
        exit(1);
        break;
    }
}

/*
 * Handle reads to CSRs and any resulting special behavior
 *
 * Note: mtohost and mfromhost are not handled here
 */
inline target_ulong csr_read_helper(CPURISCVState *env, target_ulong csrno)
{
    int csrno2 = (int)csrno;
    #ifdef RISCV_DEBUG_PRINT
    fprintf(stderr, "READ CSR 0x%x\n", csrno2);
    #endif

    switch (csrno2)
    {
    case NEW_CSR_FFLAGS:
        return env->csr[NEW_CSR_FFLAGS];
    case NEW_CSR_FRM:
        return env->csr[NEW_CSR_FRM];
    case NEW_CSR_FCSR:
        return (env->csr[NEW_CSR_FFLAGS] << FSR_AEXC_SHIFT) |
            (env->csr[NEW_CSR_FRM] << FSR_RD_SHIFT);
    case NEW_CSR_TIME:
    case NEW_CSR_INSTRET:
    case NEW_CSR_CYCLE:
        if ((env->csr[NEW_CSR_MUCOUNTEREN] >> (csrno2 & (63))) & 1) {
            return csr_read_helper(env, csrno2 + (NEW_CSR_MCYCLE - NEW_CSR_CYCLE));
        }
        break;
    case NEW_CSR_STIME:
    case NEW_CSR_SINSTRET:
    case NEW_CSR_SCYCLE:
        if ((env->csr[NEW_CSR_MSCOUNTEREN] >> (csrno2 & (63))) & 1) {
            return csr_read_helper(env, csrno2 + (NEW_CSR_MCYCLE - NEW_CSR_SCYCLE));
        }
        break;
    case NEW_CSR_MUCOUNTEREN:
        return env->csr[NEW_CSR_MUCOUNTEREN];
    case NEW_CSR_MSCOUNTEREN:
        return env->csr[NEW_CSR_MSCOUNTEREN];
    case NEW_CSR_MUCYCLE_DELTA: 
        return 0; // as spike does
    case NEW_CSR_MUTIME_DELTA:  
        return 0; // as spike does
    case NEW_CSR_MUINSTRET_DELTA:  
        return 0; // as spike does
    case NEW_CSR_MSCYCLE_DELTA:  
        return 0; // as spike does
    case NEW_CSR_MSTIME_DELTA:  
        return 0; // as spike does
    case NEW_CSR_MSINSTRET_DELTA:  
        return 0; // as spike does
    case NEW_CSR_MUCYCLE_DELTAH:  
        printf("CSR 0x%x unsupported on RV64\n", csrno2);
        exit(1);
    case NEW_CSR_MUTIME_DELTAH:  
        printf("CSR 0x%x unsupported on RV64\n", csrno2);
        exit(1);
    case NEW_CSR_MUINSTRET_DELTAH:  
        printf("CSR 0x%x unsupported on RV64\n", csrno2);
        exit(1);
    case NEW_CSR_MSCYCLE_DELTAH:  
        printf("CSR 0x%x unsupported on RV64\n", csrno2);
        exit(1);
    case NEW_CSR_MSTIME_DELTAH:  
        printf("CSR 0x%x unsupported on RV64\n", csrno2);
        exit(1);
    case NEW_CSR_MSINSTRET_DELTAH:  
        printf("CSR 0x%x unsupported on RV64\n", csrno2);
        exit(1);
    // notice the lack of CSR_MTIME - this is handled by throwing an exception
    // and letting the handler read from the RTC
    case NEW_CSR_MCYCLE:  
        return cpu_riscv_read_instret(env);
    case NEW_CSR_MINSTRET:  
        return cpu_riscv_read_instret(env);
    case NEW_CSR_MCYCLEH:  
        printf("CSR 0x%x unsupported on RV64\n", csrno2);
        exit(1);
    case NEW_CSR_MINSTRETH:  
        printf("CSR 0x%x unsupported on RV64\n", csrno2);
        exit(1);
    case NEW_CSR_SSTATUS: {
        target_ulong mask = SSTATUS_SIE | SSTATUS_SPIE | SSTATUS_SPP | SSTATUS_FS
            | SSTATUS_XS | SSTATUS_PUM;
        target_ulong sstatus = env->csr[NEW_CSR_MSTATUS] & mask;
        if ((sstatus & SSTATUS_FS) == SSTATUS_FS ||
                (sstatus & SSTATUS_XS) == SSTATUS_XS) {
            sstatus |= SSTATUS64_SD;
        }
        return sstatus;
    }
    case NEW_CSR_SIP:
        return env->csr[NEW_CSR_MIP] & env->csr[NEW_CSR_MIDELEG];
    case NEW_CSR_SIE:
        return env->csr[NEW_CSR_MIE] & env->csr[NEW_CSR_MIDELEG];
    case NEW_CSR_SEPC:
        return env->csr[NEW_CSR_SEPC];
    case NEW_CSR_SBADADDR:
        return env->csr[NEW_CSR_SBADADDR];
    case NEW_CSR_STVEC:
        return env->csr[NEW_CSR_STVEC];
    case NEW_CSR_SCAUSE:
        return env->csr[NEW_CSR_SCAUSE];
    case NEW_CSR_SPTBR:
        return env->csr[NEW_CSR_SPTBR];
    case NEW_CSR_SSCRATCH:
        return env->csr[NEW_CSR_SSCRATCH];
    case NEW_CSR_MSTATUS:
        return env->csr[NEW_CSR_MSTATUS];
    case NEW_CSR_MIP:
        return env->csr[NEW_CSR_MIP];
    case NEW_CSR_MIE:
        return env->csr[NEW_CSR_MIE];
    case NEW_CSR_MEPC:
        return env->csr[NEW_CSR_MEPC];
    case NEW_CSR_MSCRATCH:
        return env->csr[NEW_CSR_MSCRATCH];
    case NEW_CSR_MCAUSE:
        return env->csr[NEW_CSR_MCAUSE];
    case NEW_CSR_MBADADDR:
        return env->csr[NEW_CSR_MBADADDR];
    case NEW_CSR_MISA:
        return env->csr[NEW_CSR_MISA];
    case NEW_CSR_MARCHID:
        return 0; // as spike does
    case NEW_CSR_MIMPID:
        return 0; // as spike does
    case NEW_CSR_MVENDORID:
        return 0; // as spike does
    case NEW_CSR_MHARTID:
        return 0;
    case NEW_CSR_MTVEC:
        return env->csr[NEW_CSR_MTVEC];
    case NEW_CSR_MEDELEG:
        return env->csr[NEW_CSR_MEDELEG];
    case NEW_CSR_MIDELEG:
        return env->csr[NEW_CSR_MIDELEG];
    case NEW_CSR_TDRSELECT:
        return 0; // as spike does
    case NEW_CSR_DCSR:
        printf("DEBUG NOT IMPLEMENTED\n");
        exit(1);
    case NEW_CSR_DPC:
        printf("DEBUG NOT IMPLEMENTED\n");
        exit(1);
    case NEW_CSR_DSCRATCH:
        printf("DEBUG NOT IMPLEMENTED\n");
        exit(1);
    }
    // used by e.g. MTIME read
    helper_raise_exception(env, NEW_RISCV_EXCP_ILLEGAL_INST);
    return 0;
}


void validate_csr(CPURISCVState *env, uint64_t which, uint64_t write,
        uint64_t new_pc) {
    unsigned csr_priv = get_field((which), 0x300);
    unsigned csr_read_only = get_field((which), 0xC00) == 3;
    if (((write) && csr_read_only) || (env->priv < csr_priv)) {
        do_raise_exception_err(env, NEW_RISCV_EXCP_ILLEGAL_INST, new_pc);
    }
    return;
}

target_ulong helper_csrrw(CPURISCVState *env, target_ulong src,
        target_ulong csr, target_ulong new_pc)
{
    validate_csr(env, csr, 1, new_pc);
    uint64_t csr_backup = csr_read_helper(env, csr);
    csr_write_helper(env, src, csr);
    return csr_backup;
}

target_ulong helper_csrrs(CPURISCVState *env, target_ulong src,
        target_ulong csr, target_ulong new_pc, target_ulong rs1_pass)
{
    validate_csr(env, csr, rs1_pass != 0, new_pc);
    uint64_t csr_backup = csr_read_helper(env, csr);
    if (rs1_pass != 0) {
        csr_write_helper(env, src | csr_backup, csr);
    }
    return csr_backup;
}

target_ulong helper_csrrc(CPURISCVState *env, target_ulong src,
        target_ulong csr, target_ulong new_pc, target_ulong rs1_pass) {
    validate_csr(env, csr, rs1_pass != 0, new_pc);
    uint64_t csr_backup = csr_read_helper(env, csr);
    if (rs1_pass != 0) {
        csr_write_helper(env, (~src) & csr_backup, csr);
    }
    return csr_backup;
}

/*
 * This is a debug print helper for printing trace.
 * Currently calls spike-dasm, so very slow.
 * Probably not useful unless you're debugging riscv-qemu
 */
void helper_debug_print(CPURISCVState *env, target_ulong cpu_pc_deb,
        target_ulong instruction)
{
/*    int buflen = 100;
    char runbuf[buflen];
    char path[buflen];

    snprintf(runbuf, buflen, "echo 'DASM(%08lx)\n' | spike-dasm", instruction);

    FILE *fp;
    fp = popen(runbuf, "r");
    if (fp == NULL) {
        printf("popen fail\n");
        exit(1);
    }
    if (fgets(path, sizeof(path)-1, fp) != NULL) {
        fprintf(stderr, ": core   0: 0x" TARGET_FMT_lx " (0x%08lx) %s",
                cpu_pc_deb, instruction, path);
    } else {*/
        fprintf(stderr, ": core   0: 0x" TARGET_FMT_lx " (0x%08lx) %s",
                cpu_pc_deb, instruction, "DASM BAD RESULT\n");
/*    }
    pclose(fp);*/
}

void helper_debug_print_reg1(CPURISCVState *env, target_ulong reg1) {
    printf("reg1: 0x%016lx\n", reg1);
}

void helper_debug_print_reg2(CPURISCVState *env, target_ulong reg1, target_ulong reg2) {
    printf("reg1: 0x%016lx\n", reg1);
    printf("reg2: 0x%016lx\n", reg2);
}

target_ulong helper_sret(CPURISCVState *env, target_ulong cpu_pc_deb)
{
    if (!(env->priv >= PRV_S)) {
        // TODO: real illegal instruction trap
        printf("ILLEGAL INST");
        exit(1);
    }

    target_ulong retpc = env->csr[NEW_CSR_SEPC];
    if (retpc & 0x3) {
        printf("MISALIGNED INST FETCH\n");
        exit(1);
    }

    target_ulong mstatus = env->csr[NEW_CSR_MSTATUS];
    target_ulong prev_priv = get_field(mstatus, MSTATUS_SPP);
    mstatus = set_field(mstatus, MSTATUS_UIE << prev_priv, get_field(mstatus, MSTATUS_SPIE));
    mstatus = set_field(mstatus, MSTATUS_SPIE, 0);
    mstatus = set_field(mstatus, MSTATUS_SPP, PRV_U); 
    set_privilege(env, prev_priv);
    csr_write_helper(env, mstatus, NEW_CSR_MSTATUS);

    return retpc;
}

target_ulong helper_mret(CPURISCVState *env, target_ulong cpu_pc_deb)
{
    if (!(env->priv >= PRV_M)) {
        // TODO: real illegal instruction trap
        printf("ILLEGAL INST");
        exit(1);
    }

    target_ulong retpc = env->csr[NEW_CSR_MEPC];
    if (retpc & 0x3) {
        printf("MISALIGNED INST FETCH\n");
        exit(1);
    }

    target_ulong mstatus = env->csr[NEW_CSR_MSTATUS];
    target_ulong prev_priv = get_field(mstatus, MSTATUS_MPP);
    mstatus = set_field(mstatus, MSTATUS_UIE << prev_priv, get_field(mstatus, MSTATUS_MPIE));
    mstatus = set_field(mstatus, MSTATUS_MPIE, 0);
    mstatus = set_field(mstatus, MSTATUS_MPP, PRV_U); 
    set_privilege(env, prev_priv);
    csr_write_helper(env, mstatus, NEW_CSR_MSTATUS);

    return retpc;
}

#ifndef CONFIG_USER_ONLY

void helper_fence_i(CPURISCVState *env) {
    RISCVCPU *cpu = riscv_env_get_cpu(env);
    CPUState *cs = CPU(cpu);
    // Flush QEMU's TLB
    tlb_flush(cs, 1);
    // ARM port seems to not know if this is okay inside a TB...
    // But we need to do it
    tb_flush(cs);
}

void helper_tlb_flush(CPURISCVState *env)
{
    RISCVCPU *cpu = riscv_env_get_cpu(env);
    tlb_flush(CPU(cpu), 1);
}

#endif /* !CONFIG_USER_ONLY */

// TODO: implement for RISC-V WFI
/*void helper_wait(CPURISCVState *env)
{
    CPUState *cs = CPU(riscv_env_get_cpu(env));

    cs->halted = 1;
    cpu_reset_interrupt(cs, CPU_INTERRUPT_WAKE);
    printf("NOT IMPLEMENTED FOR RISCV\n");
    exit(1);
}*/

#if !defined(CONFIG_USER_ONLY)

void riscv_cpu_do_unaligned_access(CPUState *cs, target_ulong addr,
                                int rw, int is_user, uintptr_t retaddr)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;
    printf("addr: %016lx\n", addr);
    if (rw & 0x2) {
        fprintf(stderr, "unaligned inst fetch not handled here\n");
        exit(1);
    } else if (rw == 0x1) {
        printf("Store\n");
        cs->exception_index = NEW_RISCV_EXCP_STORE_AMO_ADDR_MIS;
        env->badaddr = addr;
    } else {
        printf("Load\n");
        cs->exception_index = NEW_RISCV_EXCP_LOAD_ADDR_MIS;
        env->badaddr = addr;
    }
    do_raise_exception_err(env, cs->exception_index, retaddr);
}

/* called by qemu's softmmu to fill the qemu tlb */
void tlb_fill(CPUState *cs, target_ulong addr, int is_write, int mmu_idx,
              uintptr_t retaddr)
{
    int ret;
    ret = riscv_cpu_handle_mmu_fault(cs, addr, is_write, mmu_idx);
    if (ret == TRANSLATE_FAIL) {
        RISCVCPU *cpu = RISCV_CPU(cs);
        CPURISCVState *env = &cpu->env;
        do_raise_exception_err(env, cs->exception_index, retaddr);
    }
}

void riscv_cpu_unassigned_access(CPUState *cs, hwaddr addr, bool is_write,
        bool is_exec, int unused, unsigned size)
{
    printf("unassigned address not implemented for riscv\n");
    printf("are you trying to fetch instructions from an MMIO page?\n");
    printf("unassigned Address: %016lX\n", addr);
    exit(1);
}

#endif /* !CONFIG_USER_ONLY */
