/*
 * RISC-V user-mode atomic memory ops
 *
 * Copyright (c) 2016 Alex Suykov <alex.suykov@gmail.com>
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

#include "qemu/osdep.h"
#include "cpu.h"

#ifdef CONFIG_USER_ONLY

#include "qemu.h"

/* The code in this file runs outside of cpu_loop and may not raise
   any exceptions. Instead, it should return one of RISCV_AMO_* consts.

   See comments around cpu_list_mutex in linux-user/main.c
   on why exclusive memory ops are done this way.

   Some other arches put AMO handling right into main.c,
   but for RISC-V there's just too many ops to handle and too much code,
   so it's all here instead. */

#define BITFIELD(src, end, start) \
            (((src) >> start) & ((1 << (end - start + 1)) - 1))

#define ENV CPURISCVState* env

/* Load-Reserve: rd = [rs1], creating reservation for [rs1]. */

static int rv_do_lr(ENV, int rd, int rs1, int rs2, int width)
{
    int64_t val64;
    int32_t val32;
    target_long val;
    int fault;

    target_long addr = env->gpr[rs1];

    if(rs2)
        return RISCV_AMO_BADINSN;

    switch(width) {
        case /* 010 */ 2: fault = get_user_s32(val32, addr); val = val32; break;
        case /* 011 */ 3: fault = get_user_s64(val64, addr); val = val64; break;
        default: return RISCV_AMO_BADINSN;
    }

    if(fault)
        return RISCV_AMO_BADADDR;

    if(rd) env->gpr[rd] = val;

    env->amoaddr = addr;
    env->amotest = val;

    return RISCV_AMO_OK;
}

/* Store-Conditional: [rs1] = rs2, rd = 0 if reservation is intact,
   otherwise rd = 1. */

static int rv_do_sc(ENV, int rd, int rs1, int rs2, int width)
{
    int64_t val64;
    int32_t val32;
    target_long val;
    int fault;

    target_long addr = env->gpr[rs1];
    target_long resaddr = env->amoaddr;
    target_long restest = env->amotest;

    if(addr != resaddr)
        goto fail; /* no reservation for this address */

    /* Load and test */
    switch(width) {
        case /* 010 */ 2: fault = get_user_s32(val32, addr); val = val32; break;
        case /* 011 */ 3: fault = get_user_s64(val64, addr); val = val64; break;
        default: return RISCV_AMO_BADINSN;
    }

    if(fault)
        return RISCV_AMO_BADADDR;
    if(val != restest)
        goto fail;

    /* Store */
    val = env->gpr[rs2];
    switch(width) {
        case /* 010 */ 2: val32 = val; fault = put_user_s32(val32, addr); break;
        case /* 011 */ 3: val64 = val; fault = put_user_s64(val64, addr); break;
        default: return RISCV_AMO_BADINSN; /* should not happen */
    }

    if(fault)
        return RISCV_AMO_BADADDR;

    if(rd) env->gpr[rd] = 0;
    return RISCV_AMO_OK;

fail:
    if(rd) env->gpr[rd] = 1;
    return RISCV_AMO_OK;
}

/* Tricky signed-unsigned minmaxes */
#define DEFMINMAX(type, name, ret) \
    static inline type name(type a, type b) { return ret; }
DEFMINMAX(target_long, rv_min, a < b ? a : b);
DEFMINMAX(target_long, rv_max, a > b ? a : b);
DEFMINMAX(target_ulong, rv_minu, a < b ? a : b);
DEFMINMAX(target_ulong, rv_maxu, a > b ? a : b);

/* Atomic memory ops: [rs1] = rd = [rs1] op rs2;
   amoswap, amoadd, amoxor, amoor, amomin, amomax, amominu, amomaxu. */

static int rv_do_amo(ENV, int func, int rd, int rs1, int rs2, int width)
{
    int64_t val64;
    int32_t val32;
    int fault = 0;
    target_long addr = env->gpr[rs1];
    target_long arg = env->gpr[rs2];

    target_long val;    /* read/written */

    /* Load, but do not report BADADDR yet */
    switch(width) {
        case /* 010 */ 2: fault = get_user_s32(val32, addr); val = val32; break;
        case /* 011 */ 3: fault = get_user_s64(val64, addr); val = val64; break;
        default: return RISCV_AMO_BADINSN;
    }

    target_long vrd = val;

    switch(func) {
        case /* 00001 */ 0x01: val = arg; break;
        case /* 00000 */ 0x00: val += arg; break;
        case /* 00100 */ 0x04: val ^= arg; break;
        case /* 01100 */ 0x0C: val &= arg; break;
        case /* 01000 */ 0x08: val |= arg; break;
        case /* 10000 */ 0x10: val = rv_min(val, arg); break;
        case /* 10100 */ 0x14: val = rv_max(val, arg); break;
        case /* 11000 */ 0x18: val = rv_minu(val, arg); break;
        case /* 11100 */ 0x1C: val = rv_maxu(val, arg); break;
        default: return RISCV_AMO_BADINSN;
    }

    /* No BADINSN during decoding, ok to report BADADDR */
    if(fault)
        return RISCV_AMO_BADADDR;
    /* No BADINSN on decoding and no BADADDR on read, ok to write rd */
    if(rd) env->gpr[rd] = vrd;

    switch(width) {
        case /* 010 */ 2: val32 = val; fault = put_user_s32(val32, addr); break;
        case /* 011 */ 3: val64 = val; fault = put_user_s64(val64, addr); break;
        default: return RISCV_AMO_BADINSN; /* should not happen */
    }

    if(fault)
        return RISCV_AMO_BADADDR;

    return RISCV_AMO_OK;
}

int riscv_cpu_do_usermode_amo(CPUState* cs)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;

    uint64_t insn = env->amoinsn;
    /* Major opcode must always be 0101111 AMO here */
    if(BITFIELD(insn, 6, 0) != /* 0101111 */ 0x2F)
        return RISCV_AMO_BADINSN;
    env->amoinsn = 0; /* clear amo request, just in case */

    int func = BITFIELD(insn, 31, 27);
    int rd = BITFIELD(insn, 11, 7);
    int width = BITFIELD(insn, 14, 12);
    int rs1 = BITFIELD(insn, 19, 15);
    int rs2 = BITFIELD(insn, 24, 20);

    int ret;

    switch(func) {
        case /* 00010 */ 2: ret = rv_do_lr(env, rd, rs1, rs2, width); break;
        case /* 00011 */ 3: ret = rv_do_sc(env, rd, rs1, rs2, width); break;
        default: ret = rv_do_amo(env, func, rd, rs1, rs2, width); break;
    }

    if(ret == RISCV_AMO_BADADDR)
        env->badaddr = rs1;

    return ret;
}

#endif
