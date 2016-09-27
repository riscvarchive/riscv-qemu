/*
 * RISC-V emulation for qemu: main translation routines.
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

#include "qemu/osdep.h"
#include "cpu.h"
#include "disas/disas.h"
#include "tcg-op.h"
#include "exec/cpu_ldst.h"

#include "exec/helper-proto.h"
#include "exec/helper-gen.h"

#include "instmap.h"

#define RISCV_DEBUG_DISAS 0

/* global register indices */
static TCGv_ptr cpu_env;
static TCGv cpu_gpr[32], cpu_PC;
static TCGv_i64 cpu_fpr[32]; /* assume F and D extensions */
static TCGv load_res;

#include "exec/gen-icount.h"

typedef struct DisasContext {
    struct TranslationBlock *tb;
    target_ulong pc;
    uint32_t opcode;
    int singlestep_enabled;
    int mem_idx;
    int bstate;
} DisasContext;

static inline void kill_unknown(DisasContext *ctx, int excp);

enum {
    BS_NONE     = 0, /* When seen outside of translation while loop, indicates
                     need to exit tb due to end of page. */
    BS_STOP     = 1, /* Need to exit tb for syscall, sret, etc. */
    BS_BRANCH   = 2, /* Need to exit tb for branch, jal, etc. */
};


static const char * const regnames[] = {
  "zero", "ra  ", "sp  ", "gp  ", "tp  ", "t0  ",  "t1  ",  "t2  ",
  "s0  ", "s1  ", "a0  ", "a1  ", "a2  ", "a3  ",  "a4  ",  "a5  ",
  "a6  ", "a7  ", "s2  ", "s3  ", "s4  ", "s5  ",  "s6  ",  "s7  ",
  "s8  ", "s9  ", "s10 ", "s11 ", "t3  ", "t4  ",  "t5  ",  "t6  "
};

static const char * const fpr_regnames[] = {
  "ft0", "ft1", "ft2",  "ft3",  "ft4", "ft5", "ft6",  "ft7",
  "fs0", "fs1", "fa0",  "fa1",  "fa2", "fa3", "fa4",  "fa5",
  "fa6", "fa7", "fs2",  "fs3",  "fs4", "fs5", "fs6",  "fs7",
  "fs8", "fs9", "fs10", "fs11", "ft8", "ft9", "ft10", "ft11"
};

/* convert riscv funct3 to qemu memop for load/store */
static int tcg_memop_lookup[] = { MO_SB, MO_TESW, MO_TESL, MO_TEQ, MO_UB,
    MO_TEUW, MO_TEUL };

static inline void generate_exception(DisasContext *ctx, int excp)
{
    tcg_gen_movi_tl(cpu_PC, ctx->pc);
    TCGv_i32 helper_tmp = tcg_const_i32(excp);
    gen_helper_raise_exception(cpu_env, helper_tmp);
    tcg_temp_free_i32(helper_tmp);
}

static inline void generate_exception_mbadaddr(DisasContext *ctx, int excp)
{
    tcg_gen_movi_tl(cpu_PC, ctx->pc);
    TCGv_i32 helper_tmp = tcg_const_i32(excp);
    gen_helper_raise_exception_mbadaddr(cpu_env, helper_tmp, cpu_PC);
    tcg_temp_free_i32(helper_tmp);
}

/* unknown instruction */
static inline void kill_unknown(DisasContext *ctx, int excp)
{
    generate_exception(ctx, excp);
    ctx->bstate = BS_STOP;
}

static inline bool use_goto_tb(DisasContext *ctx, target_ulong dest)
{
    if (unlikely(ctx->singlestep_enabled)) {
        return false;
    }

#ifndef CONFIG_USER_ONLY
    return (ctx->tb->pc & TARGET_PAGE_MASK) == (dest & TARGET_PAGE_MASK);
#else
    return true;
#endif
}

static inline void gen_goto_tb(DisasContext *ctx, int n, target_ulong dest)
{
    if (use_goto_tb(ctx, dest)) {
        /* chaining is only allowed when the jump is to the same page */
        tcg_gen_goto_tb(n);
        tcg_gen_movi_tl(cpu_PC, dest);
        tcg_gen_exit_tb((uintptr_t)ctx->tb + n);
    } else {
        tcg_gen_movi_tl(cpu_PC, dest);
        if (ctx->singlestep_enabled) {
            gen_helper_raise_exception_debug(cpu_env);
        }
        tcg_gen_exit_tb(0);
    }
}

/* Wrapper for getting reg values - need to check of reg is zero since
 * cpu_gpr[0] is not actually allocated
 */
static inline void gen_get_gpr(TCGv t, int reg_num)
{
    if (reg_num == 0) {
        tcg_gen_movi_tl(t, 0);
    } else {
        tcg_gen_mov_tl(t, cpu_gpr[reg_num]);
    }
}

/* Wrapper for setting reg values - need to check of reg is zero since
 * cpu_gpr[0] is not actually allocated. this is more for safety purposes,
 * since we usually avoid calling the OP_TYPE_gen function if we see a write to
 * $zero
 */
static inline void gen_set_gpr(int reg_num_dst, TCGv t)
{
    if (reg_num_dst != 0) {
        tcg_gen_mov_tl(cpu_gpr[reg_num_dst], t);
    }
}

static void gen_mulhsu(TCGv ret, TCGv arg1, TCGv arg2)
{
#if defined(TARGET_RISCV64)
    gen_helper_mulhsu(ret, cpu_env, arg1, arg2);
#else
    TCGv_i64 t0 = tcg_temp_new_i64();
    TCGv_i64 t1 = tcg_temp_new_i64();

    tcg_gen_ext_i32_i64(t0, arg1);
    tcg_gen_extu_i32_i64(t1, arg2);
    tcg_gen_mul_i64(t0, t0, t1);

    tcg_gen_shri_i64(t0, t0, 32);
    tcg_gen_extrl_i64_i32(ret, t0);

    tcg_temp_free_i64(t0);
    tcg_temp_free_i64(t1);
#endif
}

static inline void gen_arith(DisasContext *ctx, uint32_t opc, int rd, int rs1,
        int rs2)
{
    TCGv source1, source2, cond1, cond2, zeroreg, resultopt1;
    cond1 = tcg_temp_new();
    cond2 = tcg_temp_new();
    source1 = tcg_temp_new();
    source2 = tcg_temp_new();
    zeroreg = tcg_temp_new();
    resultopt1 = tcg_temp_new();
    gen_get_gpr(source1, rs1);
    gen_get_gpr(source2, rs2);
    tcg_gen_movi_tl(zeroreg, 0); /* hardcoded zero for compare in DIV, etc */

    switch (opc) {
#if defined(TARGET_RISCV64)
    case OPC_RISC_ADDW:
#endif
    case OPC_RISC_ADD:
        tcg_gen_add_tl(source1, source1, source2);
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_SUBW:
#endif
    case OPC_RISC_SUB:
        tcg_gen_sub_tl(source1, source1, source2);
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_SLLW:
        tcg_gen_andi_tl(source2, source2, 0x1F);
        /* fall through to SLL */
#endif
    case OPC_RISC_SLL:
        tcg_gen_andi_tl(source2, source2, TARGET_LONG_BITS - 1);
        tcg_gen_shl_tl(source1, source1, source2);
        break;
    case OPC_RISC_SLT:
        tcg_gen_setcond_tl(TCG_COND_LT, source1, source1, source2);
        break;
    case OPC_RISC_SLTU:
        tcg_gen_setcond_tl(TCG_COND_LTU, source1, source1, source2);
        break;
    case OPC_RISC_XOR:
        tcg_gen_xor_tl(source1, source1, source2);
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_SRLW:
        /* clear upper 32 */
        tcg_gen_andi_tl(source1, source1, 0x00000000FFFFFFFFLL);
        tcg_gen_andi_tl(source2, source2, 0x1F);
        /* fall through to SRL */
#endif
    case OPC_RISC_SRL:
        tcg_gen_andi_tl(source2, source2, TARGET_LONG_BITS - 1);
        tcg_gen_shr_tl(source1, source1, source2);
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_SRAW:
        /* first, trick to get it to act like working on 32 bits (get rid of
        upper 32, sign extend to fill space) */
        tcg_gen_shli_tl(source1, source1, 32);
        tcg_gen_sari_tl(source1, source1, 32);
        tcg_gen_andi_tl(source2, source2, 0x1F);
        /* fall through to SRA */
#endif
    case OPC_RISC_SRA:
        tcg_gen_andi_tl(source2, source2, TARGET_LONG_BITS - 1);
        tcg_gen_sar_tl(source1, source1, source2);
        break;
    case OPC_RISC_OR:
        tcg_gen_or_tl(source1, source1, source2);
        break;
    case OPC_RISC_AND:
        tcg_gen_and_tl(source1, source1, source2);
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_MULW:
#endif
    case OPC_RISC_MUL:
        tcg_gen_muls2_tl(source1, source2, source1, source2);
        break;
    case OPC_RISC_MULH:
        tcg_gen_muls2_tl(source2, source1, source1, source2);
        break;
    case OPC_RISC_MULHSU:
        gen_mulhsu(source1, source1, source2);
        break;
    case OPC_RISC_MULHU:
        tcg_gen_mulu2_tl(source2, source1, source1, source2);
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_DIVW:
        tcg_gen_ext32s_tl(source1, source1);
        tcg_gen_ext32s_tl(source2, source2);
        /* fall through to DIV */
#endif
    case OPC_RISC_DIV:
        /* Handle by altering args to tcg_gen_div to produce req'd results:
         * For overflow: want source1 in source1 and 1 in source2
         * For div by zero: want -1 in source1 and 1 in source2 -> -1 result */
        tcg_gen_movi_tl(resultopt1, (target_ulong)0xFFFFFFFFFFFFFFFF);
        tcg_gen_setcondi_tl(TCG_COND_EQ, cond2, source2, (target_ulong)(~0L));
        tcg_gen_setcondi_tl(TCG_COND_EQ, cond1, source1,
                            1L << (TARGET_LONG_BITS - 1));
        tcg_gen_and_tl(cond1, cond1, cond2); /* cond1 = overflow */
        tcg_gen_setcondi_tl(TCG_COND_EQ, cond2, source2, 0); /* cond2 = div 0 */
        /* if div by zero, set source1 to -1, otherwise don't change */
        tcg_gen_movcond_tl(TCG_COND_EQ, source1, cond2, zeroreg, source1,
                resultopt1);
        /* if overflow or div by zero, set source2 to 1, else don't change */
        tcg_gen_or_tl(cond1, cond1, cond2);
        tcg_gen_movi_tl(resultopt1, (target_ulong)1);
        tcg_gen_movcond_tl(TCG_COND_EQ, source2, cond1, zeroreg, source2,
                resultopt1);
        tcg_gen_div_tl(source1, source1, source2);
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_DIVUW:
        tcg_gen_ext32u_tl(source1, source1);
        tcg_gen_ext32u_tl(source2, source2);
        /* fall through to DIVU */
#endif
    case OPC_RISC_DIVU:
        tcg_gen_setcondi_tl(TCG_COND_EQ, cond1, source2, 0);
        tcg_gen_movi_tl(resultopt1, (target_ulong)(~0L));
        tcg_gen_movcond_tl(TCG_COND_EQ, source1, cond1, zeroreg, source1,
                resultopt1);
        tcg_gen_movi_tl(resultopt1, (target_ulong)(1L));
        tcg_gen_movcond_tl(TCG_COND_EQ, source2, cond1, zeroreg, source2,
                resultopt1);
        tcg_gen_divu_tl(source1, source1, source2);
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_REMW:
        tcg_gen_ext32s_tl(source1, source1);
        tcg_gen_ext32s_tl(source2, source2);
        /* fall through to REM */
#endif
    case OPC_RISC_REM:
        tcg_gen_movi_tl(resultopt1, 1L);
        tcg_gen_setcondi_tl(TCG_COND_EQ, cond2, source2, (target_ulong)(~0L));
        tcg_gen_setcondi_tl(TCG_COND_EQ, cond1, source1,
                            1L << (TARGET_LONG_BITS - 1));
        tcg_gen_and_tl(cond2, cond1, cond2); /* cond1 = overflow */
        tcg_gen_setcondi_tl(TCG_COND_EQ, cond1, source2, 0); /* cond2 = div 0 */
        /* if overflow or div by zero, set source2 to 1, else don't change */
        tcg_gen_or_tl(cond2, cond1, cond2);
        tcg_gen_movcond_tl(TCG_COND_EQ, source2, cond2, zeroreg, source2,
                resultopt1);
        tcg_gen_rem_tl(resultopt1, source1, source2);
        /* if div by zero, just return the original dividend */
        tcg_gen_movcond_tl(TCG_COND_EQ, source1, cond1, zeroreg, resultopt1,
                source1);
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_REMUW:
        tcg_gen_ext32u_tl(source1, source1);
        tcg_gen_ext32u_tl(source2, source2);
        /* fall through to REMU */
#endif
    case OPC_RISC_REMU:
        tcg_gen_movi_tl(resultopt1, 1L);
        tcg_gen_setcondi_tl(TCG_COND_EQ, cond1, source2, 0);
        tcg_gen_movcond_tl(TCG_COND_EQ, source2, cond1, zeroreg, source2,
                resultopt1);
        tcg_gen_remu_tl(resultopt1, source1, source2);
        /* if div by zero, just return the original dividend */
        tcg_gen_movcond_tl(TCG_COND_EQ, source1, cond1, zeroreg, resultopt1,
                source1);
        break;
    default:
        kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        break;
    }

    if (opc & 0x8) { /* sign extend for W instructions */
        tcg_gen_ext32s_tl(source1, source1);
    }

    gen_set_gpr(rd, source1);
    tcg_temp_free(source1);
    tcg_temp_free(source2);
    tcg_temp_free(cond1);
    tcg_temp_free(cond2);
    tcg_temp_free(zeroreg);
    tcg_temp_free(resultopt1);
}

static inline void gen_arith_imm(DisasContext *ctx, uint32_t opc, int rd,
        int rs1, int16_t imm)
{
    TCGv source1;
    source1 = tcg_temp_new();
    gen_get_gpr(source1, rs1);
    /* lower 12 bits of imm are valid */
    target_long uimm = (target_long)imm; /* sign ext 16->64 bits */
    target_long extra_shamt = 0;

    switch (opc) {
    case OPC_RISC_ADDI:
#if defined(TARGET_RISCV64)
    case OPC_RISC_ADDIW:
#endif
        tcg_gen_addi_tl(source1, source1, uimm);
        break;
    case OPC_RISC_SLTI:
        tcg_gen_setcondi_tl(TCG_COND_LT, source1, source1, uimm);
        break;
    case OPC_RISC_SLTIU:
        tcg_gen_setcondi_tl(TCG_COND_LTU, source1, source1, uimm);
        break;
    case OPC_RISC_XORI:
        tcg_gen_xori_tl(source1, source1, uimm);
        break;
    case OPC_RISC_ORI:
        tcg_gen_ori_tl(source1, source1, uimm);
        break;
    case OPC_RISC_ANDI:
        tcg_gen_andi_tl(source1, source1, uimm);
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_SLLIW:
         if ((uimm >= 32)) {
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
         }
        /* fall through to SLLI */
#endif
    case OPC_RISC_SLLI:
        if (uimm < TARGET_LONG_BITS) {
            tcg_gen_shli_tl(source1, source1, uimm);
        } else {
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_SHIFT_RIGHT_IW:
        if ((uimm & 0x3ff) >= 32) {
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        }
        tcg_gen_shli_tl(source1, source1, 32);
        extra_shamt = 32;
        /* fall through to SHIFT_RIGHT_I */
#endif
    case OPC_RISC_SHIFT_RIGHT_I:
        /* differentiate on IMM */
        if ((uimm & 0x3ff) < TARGET_LONG_BITS) {
            if (uimm & 0x400) {
                /* SRAI[W] */
                tcg_gen_sari_tl(source1, source1, (uimm ^ 0x400) + extra_shamt);
            } else {
                /* SRLI[W] */
                tcg_gen_shri_tl(source1, source1, uimm + extra_shamt);
            }
        } else {
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    default:
        kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        break;
    }

    if (opc & 0x8) { /* sign-extend for W instructions */
        tcg_gen_ext32s_tl(source1, source1);
    }

    gen_set_gpr(rd, source1);
    tcg_temp_free(source1);
}

static inline void gen_jalr(DisasContext *ctx, uint32_t opc, int rd, int rs1,
        int16_t imm)
{
    /* no chaining with JALR */
    TCGLabel *misaligned = gen_new_label();
    TCGLabel *done = gen_new_label();
    target_long uimm = (target_long)imm; /* sign ext 16->64 bits */
    TCGv t0, t1, t2, t3;
    t0 = tcg_temp_local_new();
    t1 = tcg_temp_local_new();
    t2 = tcg_temp_local_new(); /* old_pc */
    t3 = tcg_temp_local_new();

    switch (opc) {
    case OPC_RISC_JALR:
        gen_get_gpr(t0, rs1);
        tcg_gen_addi_tl(t0, t0, uimm);
        tcg_gen_andi_tl(t0, t0, (target_ulong)0xFFFFFFFFFFFFFFFEll);
        tcg_gen_andi_tl(t3, t0, 0x2);
        tcg_gen_movi_tl(t2, ctx->pc);
        tcg_gen_brcondi_tl(TCG_COND_NE, t3, 0x0, misaligned);
        tcg_gen_mov_tl(cpu_PC, t0);
        tcg_gen_addi_tl(t1, t2, 4);
        gen_set_gpr(rd, t1);
        tcg_gen_br(done);
        gen_set_label(misaligned);
        generate_exception_mbadaddr(ctx, RISCV_EXCP_INST_ADDR_MIS);
        gen_set_label(done);
        tcg_gen_exit_tb(0);
        ctx->bstate = BS_BRANCH;
        break;
    default:
        kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
    tcg_temp_free(t0);
    tcg_temp_free(t1);
    tcg_temp_free(t2);
    tcg_temp_free(t3);
}

static inline void gen_branch(DisasContext *ctx, uint32_t opc, int rs1, int rs2,
        int16_t bimm)
{
    TCGLabel *l = gen_new_label();
    TCGv source1, source2;
    source1 = tcg_temp_new();
    source2 = tcg_temp_new();
    gen_get_gpr(source1, rs1);
    gen_get_gpr(source2, rs2);
    target_ulong ubimm = (target_long)bimm; /* sign ext 16->64 bits */

    switch (opc) {
    case OPC_RISC_BEQ:
        tcg_gen_brcond_tl(TCG_COND_EQ, source1, source2, l);
        break;
    case OPC_RISC_BNE:
        tcg_gen_brcond_tl(TCG_COND_NE, source1, source2, l);
        break;
    case OPC_RISC_BLT:
        tcg_gen_brcond_tl(TCG_COND_LT, source1, source2, l);
        break;
    case OPC_RISC_BGE:
        tcg_gen_brcond_tl(TCG_COND_GE, source1, source2, l);
        break;
    case OPC_RISC_BLTU:
        tcg_gen_brcond_tl(TCG_COND_LTU, source1, source2, l);
        break;
    case OPC_RISC_BGEU:
        tcg_gen_brcond_tl(TCG_COND_GEU, source1, source2, l);
        break;
    default:
        kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        break;
    }

#ifdef DISABLE_CHAINING_BRANCH
    tcg_gen_movi_tl(cpu_PC, ctx->pc + 4);
    tcg_gen_exit_tb(0);
#else
    gen_goto_tb(ctx, 1, ctx->pc + 4); /* must use this for safety */
#endif
    gen_set_label(l); /* branch taken */
    if ((ctx->pc + ubimm) & 0x3) {
        /* misaligned */
        generate_exception_mbadaddr(ctx, RISCV_EXCP_INST_ADDR_MIS);
        tcg_gen_exit_tb(0);
    } else {
#ifdef DISABLE_CHAINING_BRANCH
        tcg_gen_movi_tl(cpu_PC, ctx->pc + ubimm);
        tcg_gen_exit_tb(0);
#else
        gen_goto_tb(ctx, 0, ctx->pc + ubimm); /* must use this for safety */
#endif
    }
    tcg_temp_free(source1);
    tcg_temp_free(source2);
    ctx->bstate = BS_BRANCH;
}

static inline void gen_load(DisasContext *ctx, uint32_t opc, int rd, int rs1,
        int16_t imm)
{
    target_long uimm = (target_long)imm; /* sign ext 16->64 bits */
    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();
    gen_get_gpr(t0, rs1);
    tcg_gen_addi_tl(t0, t0, uimm); /* */
    int memop = (opc >> 12) & 0x7;

#if defined(TARGET_RISCV64)
    if (memop == 0x7) {
#else
    if (memop == 0x7 || memop == 0x3 || memop == 0x6) {
#endif
        kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
    } else {
        tcg_gen_qemu_ld_tl(t1, t0, ctx->mem_idx, tcg_memop_lookup[memop]);
    }

    gen_set_gpr(rd, t1);
    tcg_temp_free(t0);
    tcg_temp_free(t1);
}

static inline void gen_store(DisasContext *ctx, uint32_t opc, int rs1, int rs2,
        int16_t imm)
{
    target_long uimm = (target_long)imm; /* sign ext 16->64 bits */

    TCGv t0 = tcg_temp_new();
    TCGv dat = tcg_temp_new();
    gen_get_gpr(t0, rs1);
    tcg_gen_addi_tl(t0, t0, uimm);
    gen_get_gpr(dat, rs2);
    int memop = (opc >> 12) & 0x7;

#if defined(TARGET_RISCV64)
    if (memop > 0x3) {
#else
    if (memop > 0x2) {
#endif
        kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
    } else {
        tcg_gen_qemu_st_tl(dat, t0, ctx->mem_idx, tcg_memop_lookup[memop]);
    }

    tcg_temp_free(t0);
    tcg_temp_free(dat);
}

static inline void gen_fp_load(DisasContext *ctx, uint32_t opc, int rd,
        int rs1, int16_t imm)
{
    target_long uimm = (target_long)imm; /* sign ext 16->64 bits */
    TCGv t0 = tcg_temp_new();
    gen_get_gpr(t0, rs1);
    tcg_gen_addi_tl(t0, t0, uimm);

    switch (opc) {
    case OPC_RISC_FLW:
        tcg_gen_qemu_ld_i64(cpu_fpr[rd], t0, ctx->mem_idx, MO_TEUL);
        break;
    case OPC_RISC_FLD:
        tcg_gen_qemu_ld_i64(cpu_fpr[rd], t0, ctx->mem_idx, MO_TEQ);
        break;
    default:
        kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
    tcg_temp_free(t0);
}

static inline void gen_fp_store(DisasContext *ctx, uint32_t opc, int rs1,
        int rs2, int16_t imm)
{
    target_long uimm = (target_long)imm; /* sign ext 16->64 bits */

    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();
    gen_get_gpr(t0, rs1);
    tcg_gen_addi_tl(t0, t0, uimm);

    switch (opc) {
    case OPC_RISC_FSW:
        tcg_gen_qemu_st_i64(cpu_fpr[rs2], t0, ctx->mem_idx, MO_TEUL);
        break;
    case OPC_RISC_FSD:
        tcg_gen_qemu_st_i64(cpu_fpr[rs2], t0, ctx->mem_idx, MO_TEQ);
        break;
    default:
        kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        break;
    }

    tcg_temp_free(t0);
    tcg_temp_free(t1);
}

static inline void gen_atomic(DisasContext *ctx, uint32_t opc,
                      int rd, int rs1, int rs2)
{
    /* TODO: handle aq, rl bits? - for now just get rid of them: */
    opc = MASK_OP_ATOMIC_NO_AQ_RL(opc);
    TCGv source1, source2, dat;
    TCGLabel *j = gen_new_label();
    TCGLabel *done = gen_new_label();
    source1 = tcg_temp_local_new();
    source2 = tcg_temp_local_new();
    dat = tcg_temp_local_new();
    gen_get_gpr(source1, rs1);
    gen_get_gpr(source2, rs2);

    switch (opc) {
        /* all currently implemented as non-atomics */
    case OPC_RISC_LR_W:
        /* put addr in load_res */
        tcg_gen_mov_tl(load_res, source1);
        tcg_gen_qemu_ld_tl(dat, source1, ctx->mem_idx, MO_TESL | MO_ALIGN);
        break;
    case OPC_RISC_SC_W:
        tcg_gen_brcond_tl(TCG_COND_NE, load_res, source1, j);
        tcg_gen_qemu_st_tl(source2, source1, ctx->mem_idx, MO_TEUL | MO_ALIGN);
        tcg_gen_movi_tl(dat, 0); /*success */
        tcg_gen_br(done);
        gen_set_label(j);
        tcg_gen_movi_tl(dat, 1); /*fail */
        gen_set_label(done);
        break;
    case OPC_RISC_AMOSWAP_W:
        tcg_gen_qemu_ld_tl(dat, source1, ctx->mem_idx, MO_TESL | MO_ALIGN);
        tcg_gen_qemu_st_tl(source2, source1, ctx->mem_idx, MO_TEUL | MO_ALIGN);
        break;
    case OPC_RISC_AMOADD_W:
        tcg_gen_qemu_ld_tl(dat, source1, ctx->mem_idx, MO_TESL | MO_ALIGN);
        tcg_gen_add_tl(source2, dat, source2);
        tcg_gen_qemu_st_tl(source2, source1, ctx->mem_idx, MO_TEUL | MO_ALIGN);
        break;
    case OPC_RISC_AMOXOR_W:
        tcg_gen_qemu_ld_tl(dat, source1, ctx->mem_idx, MO_TESL | MO_ALIGN);
        tcg_gen_xor_tl(source2, dat, source2);
        tcg_gen_qemu_st_tl(source2, source1, ctx->mem_idx, MO_TEUL | MO_ALIGN);
        break;
    case OPC_RISC_AMOAND_W:
        tcg_gen_qemu_ld_tl(dat, source1, ctx->mem_idx, MO_TESL | MO_ALIGN);
        tcg_gen_and_tl(source2, dat, source2);
        tcg_gen_qemu_st_tl(source2, source1, ctx->mem_idx, MO_TEUL | MO_ALIGN);
        break;
    case OPC_RISC_AMOOR_W:
        tcg_gen_qemu_ld_tl(dat, source1, ctx->mem_idx, MO_TESL | MO_ALIGN);
        tcg_gen_or_tl(source2, dat, source2);
        tcg_gen_qemu_st_tl(source2, source1, ctx->mem_idx, MO_TEUL | MO_ALIGN);
        break;
    case OPC_RISC_AMOMIN_W:
        tcg_gen_ext32s_tl(source2, source2); /* since comparing */
        tcg_gen_qemu_ld_tl(dat, source1, ctx->mem_idx, MO_TESL | MO_ALIGN);
        tcg_gen_movcond_tl(TCG_COND_LT, source2, dat, source2, dat, source2);
        tcg_gen_qemu_st_tl(source2, source1, ctx->mem_idx, MO_TEUL | MO_ALIGN);
        break;
    case OPC_RISC_AMOMAX_W:
        tcg_gen_ext32s_tl(source2, source2); /* since comparing */
        tcg_gen_qemu_ld_tl(dat, source1, ctx->mem_idx, MO_TESL | MO_ALIGN);
        tcg_gen_movcond_tl(TCG_COND_GT, source2, dat, source2, dat, source2);
        tcg_gen_qemu_st_tl(source2, source1, ctx->mem_idx, MO_TEUL | MO_ALIGN);
        break;
    case OPC_RISC_AMOMINU_W:
        tcg_gen_ext32u_tl(source2, source2); /* since comparing */
        tcg_gen_qemu_ld_tl(dat, source1, ctx->mem_idx, MO_TEUL | MO_ALIGN);
        tcg_gen_movcond_tl(TCG_COND_LTU, source2, dat, source2, dat, source2);
        tcg_gen_qemu_st_tl(source2, source1, ctx->mem_idx, MO_TEUL | MO_ALIGN);
        tcg_gen_ext32s_tl(dat, dat); /* since load was TEUL */
        break;
    case OPC_RISC_AMOMAXU_W:
        tcg_gen_ext32u_tl(source2, source2); /* since comparing */
        tcg_gen_qemu_ld_tl(dat, source1, ctx->mem_idx, MO_TEUL | MO_ALIGN);
        tcg_gen_movcond_tl(TCG_COND_GTU, source2, dat, source2, dat, source2);
        tcg_gen_qemu_st_tl(source2, source1, ctx->mem_idx, MO_TEUL | MO_ALIGN);
        tcg_gen_ext32s_tl(dat, dat); /* since load was TEUL */
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_LR_D:
        /* put addr in load_res */
        tcg_gen_mov_tl(load_res, source1);
        tcg_gen_qemu_ld_tl(dat, source1, ctx->mem_idx, MO_TEQ | MO_ALIGN);
        break;
    case OPC_RISC_SC_D:
        tcg_gen_brcond_tl(TCG_COND_NE, load_res, source1, j);
        tcg_gen_qemu_st_tl(source2, source1, ctx->mem_idx, MO_TEQ | MO_ALIGN);
        tcg_gen_movi_tl(dat, 0); /* success */
        tcg_gen_br(done);
        gen_set_label(j);
        tcg_gen_movi_tl(dat, 1); /* fail */
        gen_set_label(done);
        break;
    case OPC_RISC_AMOSWAP_D:
        tcg_gen_qemu_ld_tl(dat, source1, ctx->mem_idx, MO_TEQ | MO_ALIGN);
        tcg_gen_qemu_st_tl(source2, source1, ctx->mem_idx, MO_TEQ | MO_ALIGN);
        break;
    case OPC_RISC_AMOADD_D:
        tcg_gen_qemu_ld_tl(dat, source1, ctx->mem_idx, MO_TEQ | MO_ALIGN);
        tcg_gen_add_tl(source2, dat, source2);
        tcg_gen_qemu_st_tl(source2, source1, ctx->mem_idx, MO_TEQ | MO_ALIGN);
        break;
    case OPC_RISC_AMOXOR_D:
        tcg_gen_qemu_ld_tl(dat, source1, ctx->mem_idx, MO_TEQ | MO_ALIGN);
        tcg_gen_xor_tl(source2, dat, source2);
        tcg_gen_qemu_st_tl(source2, source1, ctx->mem_idx, MO_TEQ | MO_ALIGN);
        break;
    case OPC_RISC_AMOAND_D:
        tcg_gen_qemu_ld_tl(dat, source1, ctx->mem_idx, MO_TEQ | MO_ALIGN);
        tcg_gen_and_tl(source2, dat, source2);
        tcg_gen_qemu_st_tl(source2, source1, ctx->mem_idx, MO_TEQ | MO_ALIGN);
        break;
    case OPC_RISC_AMOOR_D:
        tcg_gen_qemu_ld_tl(dat, source1, ctx->mem_idx, MO_TEQ | MO_ALIGN);
        tcg_gen_or_tl(source2, dat, source2);
        tcg_gen_qemu_st_tl(source2, source1, ctx->mem_idx, MO_TEQ | MO_ALIGN);
        break;
    case OPC_RISC_AMOMIN_D:
        tcg_gen_qemu_ld_tl(dat, source1, ctx->mem_idx, MO_TEQ | MO_ALIGN);
        tcg_gen_movcond_tl(TCG_COND_LT, source2, dat, source2, dat, source2);
        tcg_gen_qemu_st_tl(source2, source1, ctx->mem_idx, MO_TEQ | MO_ALIGN);
        break;
    case OPC_RISC_AMOMAX_D:
        tcg_gen_qemu_ld_tl(dat, source1, ctx->mem_idx, MO_TEQ | MO_ALIGN);
        tcg_gen_movcond_tl(TCG_COND_GT, source2, dat, source2, dat, source2);
        tcg_gen_qemu_st_tl(source2, source1, ctx->mem_idx, MO_TEQ | MO_ALIGN);
        break;
    case OPC_RISC_AMOMINU_D:
        tcg_gen_qemu_ld_tl(dat, source1, ctx->mem_idx, MO_TEQ | MO_ALIGN);
        tcg_gen_movcond_tl(TCG_COND_LTU, source2, dat, source2, dat, source2);
        tcg_gen_qemu_st_tl(source2, source1, ctx->mem_idx, MO_TEQ | MO_ALIGN);
        break;
    case OPC_RISC_AMOMAXU_D:
        tcg_gen_qemu_ld_tl(dat, source1, ctx->mem_idx, MO_TEQ | MO_ALIGN);
        tcg_gen_movcond_tl(TCG_COND_GTU, source2, dat, source2, dat, source2);
        tcg_gen_qemu_st_tl(source2, source1, ctx->mem_idx, MO_TEQ | MO_ALIGN);
        break;
#endif
    default:
        kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        break;
    }

    gen_set_gpr(rd, dat);
    tcg_temp_free(source1);
    tcg_temp_free(source2);
    tcg_temp_free(dat);
}

static inline void gen_fp_fmadd(DisasContext *ctx, uint32_t opc, int rd,
        int rs1, int rs2, int rs3, int rm)
{
    TCGv_i64 rm_reg = tcg_temp_new_i64();
    tcg_gen_movi_i64(rm_reg, rm);

    switch (opc) {
    case OPC_RISC_FMADD_S:
        gen_helper_fmadd_s(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2],
                           cpu_fpr[rs3], rm_reg);
        break;
    case OPC_RISC_FMADD_D:
        gen_helper_fmadd_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2],
                           cpu_fpr[rs3], rm_reg);
        break;
    default:
        kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
    tcg_temp_free_i64(rm_reg);

}

static inline void gen_fp_fmsub(DisasContext *ctx, uint32_t opc, int rd,
        int rs1, int rs2, int rs3, int rm)
{
    TCGv_i64 rm_reg = tcg_temp_new_i64();
    tcg_gen_movi_i64(rm_reg, rm);

    switch (opc) {
    case OPC_RISC_FMSUB_S:
        gen_helper_fmsub_s(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2],
                           cpu_fpr[rs3], rm_reg);
        break;
    case OPC_RISC_FMSUB_D:
        gen_helper_fmsub_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2],
                           cpu_fpr[rs3], rm_reg);
        break;
    default:
        kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
    tcg_temp_free_i64(rm_reg);
}

static inline void gen_fp_fnmsub(DisasContext *ctx, uint32_t opc, int rd,
        int rs1, int rs2, int rs3, int rm)
{
    TCGv_i64 rm_reg = tcg_temp_new_i64();
    tcg_gen_movi_i64(rm_reg, rm);

    switch (opc) {
    case OPC_RISC_FNMSUB_S:
        gen_helper_fnmsub_s(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2],
                            cpu_fpr[rs3], rm_reg);
        break;
    case OPC_RISC_FNMSUB_D:
        gen_helper_fnmsub_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2],
                            cpu_fpr[rs3], rm_reg);
        break;
    default:
        kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
    tcg_temp_free_i64(rm_reg);
}

static inline void gen_fp_fnmadd(DisasContext *ctx, uint32_t opc, int rd,
        int rs1, int rs2, int rs3, int rm)
{
    TCGv_i64 rm_reg = tcg_temp_new_i64();
    tcg_gen_movi_i64(rm_reg, rm);

    switch (opc) {
    case OPC_RISC_FNMADD_S:
        gen_helper_fnmadd_s(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2],
                            cpu_fpr[rs3], rm_reg);
        break;
    case OPC_RISC_FNMADD_D:
        gen_helper_fnmadd_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2],
                            cpu_fpr[rs3], rm_reg);
        break;
    default:
        kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
    tcg_temp_free_i64(rm_reg);
}

static inline void gen_fp_arith(DisasContext *ctx, uint32_t opc, int rd,
        int rs1, int rs2, int rm)
{
    TCGv_i64 rm_reg = tcg_temp_new_i64();
    TCGv write_int_rd = tcg_temp_new();
    tcg_gen_movi_i64(rm_reg, rm);

    switch (opc) {
    case OPC_RISC_FADD_S:
        gen_helper_fadd_s(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2],
                          rm_reg);
        break;
    case OPC_RISC_FSUB_S:
        gen_helper_fsub_s(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2],
                          rm_reg);
        break;
    case OPC_RISC_FMUL_S:
        gen_helper_fmul_s(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2],
                          rm_reg);
        break;
    case OPC_RISC_FDIV_S:
        gen_helper_fdiv_s(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2],
                          rm_reg);
        break;
    case OPC_RISC_FSGNJ_S:
        /* also handles: OPC_RISC_FSGNJN_S, OPC_RISC_FSGNJX_S */
        if (rm == 0x0) {
            gen_helper_fsgnj_s(cpu_fpr[rd], cpu_env, cpu_fpr[rs1],
                               cpu_fpr[rs2]);
        } else if (rm == 0x1) {
            gen_helper_fsgnjn_s(cpu_fpr[rd], cpu_env, cpu_fpr[rs1],
                                cpu_fpr[rs2]);
        } else if (rm == 0x2) {
            gen_helper_fsgnjx_s(cpu_fpr[rd], cpu_env, cpu_fpr[rs1],
                                cpu_fpr[rs2]);
        } else {
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case OPC_RISC_FMIN_S:
        /* also handles: OPC_RISC_FMAX_S */
        if (rm == 0x0) {
            gen_helper_fmin_s(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2]);
        } else if (rm == 0x1) {
            gen_helper_fmax_s(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2]);
        } else {
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case OPC_RISC_FSQRT_S:
        gen_helper_fsqrt_s(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], rm_reg);
        break;
    case OPC_RISC_FEQ_S:
        /* also handles: OPC_RISC_FLT_S, OPC_RISC_FLE_S */
        if (rm == 0x0) {
            gen_helper_fle_s(write_int_rd, cpu_env, cpu_fpr[rs1], cpu_fpr[rs2]);
        } else if (rm == 0x1) {
            gen_helper_flt_s(write_int_rd, cpu_env, cpu_fpr[rs1], cpu_fpr[rs2]);
        } else if (rm == 0x2) {
            gen_helper_feq_s(write_int_rd, cpu_env, cpu_fpr[rs1], cpu_fpr[rs2]);
        } else {
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        }
        gen_set_gpr(rd, write_int_rd);
        break;
    case OPC_RISC_FCVT_W_S:
        /* also OPC_RISC_FCVT_WU_S, OPC_RISC_FCVT_L_S, OPC_RISC_FCVT_LU_S */
        if (rs2 == 0x0) { /* FCVT_W_S */
            gen_helper_fcvt_w_s(write_int_rd, cpu_env, cpu_fpr[rs1], rm_reg);
        } else if (rs2 == 0x1) { /* FCVT_WU_S */
            gen_helper_fcvt_wu_s(write_int_rd, cpu_env, cpu_fpr[rs1], rm_reg);
        } else if (rs2 == 0x2) { /* FCVT_L_S */
#if defined(TARGET_RISCV64)
            gen_helper_fcvt_l_s(write_int_rd, cpu_env, cpu_fpr[rs1], rm_reg);
#else
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
#endif
        } else if (rs2 == 0x3) { /* FCVT_LU_S */
#if defined(TARGET_RISCV64)
            gen_helper_fcvt_lu_s(write_int_rd, cpu_env, cpu_fpr[rs1], rm_reg);
#else
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
#endif
        } else {
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        }
        gen_set_gpr(rd, write_int_rd);
        break;
    case OPC_RISC_FCVT_S_W:
        /* also OPC_RISC_FCVT_S_WU, OPC_RISC_FCVT_S_L, OPC_RISC_FCVT_S_LU */
        gen_get_gpr(write_int_rd, rs1);
        if (rs2 == 0) { /* FCVT_S_W */
            gen_helper_fcvt_s_w(cpu_fpr[rd], cpu_env, write_int_rd, rm_reg);
        } else if (rs2 == 0x1) { /* FCVT_S_WU */
            gen_helper_fcvt_s_wu(cpu_fpr[rd], cpu_env, write_int_rd, rm_reg);
        } else if (rs2 == 0x2) { /* FCVT_S_L */
#if defined(TARGET_RISCV64)
            gen_helper_fcvt_s_l(cpu_fpr[rd], cpu_env, write_int_rd, rm_reg);
#else
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
#endif
        } else if (rs2 == 0x3) { /* FCVT_S_LU */
#if defined(TARGET_RISCV64)
            gen_helper_fcvt_s_lu(cpu_fpr[rd], cpu_env, write_int_rd, rm_reg);
#else
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
#endif
        } else {
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case OPC_RISC_FMV_X_S:
        /* also OPC_RISC_FCLASS_S */
        if (rm == 0x0) { /* FMV */
#if defined(TARGET_RISCV64)
            tcg_gen_ext32s_tl(write_int_rd, cpu_fpr[rs1]);
#else
            tcg_gen_extrl_i64_i32(write_int_rd, cpu_fpr[rs1]);
#endif
        } else if (rm == 0x1) {
            gen_helper_fclass_s(write_int_rd, cpu_env, cpu_fpr[rs1]);
        } else {
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        }
        gen_set_gpr(rd, write_int_rd);
        break;
    case OPC_RISC_FMV_S_X:
        gen_get_gpr(write_int_rd, rs1);
#if defined(TARGET_RISCV64)
        tcg_gen_mov_tl(cpu_fpr[rd], write_int_rd);
#else
        tcg_gen_extu_i32_i64(cpu_fpr[rd], write_int_rd);
#endif
        break;
    /* double */
    case OPC_RISC_FADD_D:
        gen_helper_fadd_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2],
                          rm_reg);
        break;
    case OPC_RISC_FSUB_D:
        gen_helper_fsub_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2],
                          rm_reg);
        break;
    case OPC_RISC_FMUL_D:
        gen_helper_fmul_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2],
                          rm_reg);
        break;
    case OPC_RISC_FDIV_D:
        gen_helper_fdiv_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2],
                          rm_reg);
        break;
    case OPC_RISC_FSGNJ_D:
        /* also OPC_RISC_FSGNJN_D, OPC_RISC_FSGNJX_D */
        if (rm == 0x0) {
            gen_helper_fsgnj_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1],
                               cpu_fpr[rs2]);
        } else if (rm == 0x1) {
            gen_helper_fsgnjn_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1],
                                cpu_fpr[rs2]);
        } else if (rm == 0x2) {
            gen_helper_fsgnjx_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1],
                                cpu_fpr[rs2]);
        } else {
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case OPC_RISC_FMIN_D:
        /* also OPC_RISC_FMAX_D */
        if (rm == 0x0) {
            gen_helper_fmin_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2]);
        } else if (rm == 0x1) {
            gen_helper_fmax_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2]);
        } else {
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case OPC_RISC_FCVT_S_D:
        if (rs2 == 0x1) {
            gen_helper_fcvt_s_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], rm_reg);
        } else {
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case OPC_RISC_FCVT_D_S:
        if (rs2 == 0x0) {
            gen_helper_fcvt_d_s(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], rm_reg);
        } else {
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case OPC_RISC_FSQRT_D:
        gen_helper_fsqrt_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], rm_reg);
        break;
    case OPC_RISC_FEQ_D:
        /* also OPC_RISC_FLT_D, OPC_RISC_FLE_D */
        if (rm == 0x0) {
            gen_helper_fle_d(write_int_rd, cpu_env, cpu_fpr[rs1], cpu_fpr[rs2]);
        } else if (rm == 0x1) {
            gen_helper_flt_d(write_int_rd, cpu_env, cpu_fpr[rs1], cpu_fpr[rs2]);
        } else if (rm == 0x2) {
            gen_helper_feq_d(write_int_rd, cpu_env, cpu_fpr[rs1], cpu_fpr[rs2]);
        } else {
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        }
        gen_set_gpr(rd, write_int_rd);
        break;
    case OPC_RISC_FCVT_W_D:
        /* also OPC_RISC_FCVT_WU_D, OPC_RISC_FCVT_L_D, OPC_RISC_FCVT_LU_D */
        if (rs2 == 0x0) {
            gen_helper_fcvt_w_d(write_int_rd, cpu_env, cpu_fpr[rs1], rm_reg);
        } else if (rs2 == 0x1) {
            gen_helper_fcvt_wu_d(write_int_rd, cpu_env, cpu_fpr[rs1], rm_reg);
        } else if (rs2 == 0x2) {
#if defined(TARGET_RISCV64)
            gen_helper_fcvt_l_d(write_int_rd, cpu_env, cpu_fpr[rs1], rm_reg);
#else
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
#endif
        } else if (rs2 == 0x3) {
#if defined(TARGET_RISCV64)
            gen_helper_fcvt_lu_d(write_int_rd, cpu_env, cpu_fpr[rs1], rm_reg);
#else
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
#endif
        } else {
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        }
        gen_set_gpr(rd, write_int_rd);
        break;
    case OPC_RISC_FCVT_D_W:
        /* also OPC_RISC_FCVT_D_WU, OPC_RISC_FCVT_D_L, OPC_RISC_FCVT_D_LU */
        gen_get_gpr(write_int_rd, rs1);
        if (rs2 == 0x0) {
            gen_helper_fcvt_d_w(cpu_fpr[rd], cpu_env, write_int_rd, rm_reg);
        } else if (rs2 == 0x1) {
            gen_helper_fcvt_d_wu(cpu_fpr[rd], cpu_env, write_int_rd, rm_reg);
        } else if (rs2 == 0x2) {
#if defined(TARGET_RISCV64)
            gen_helper_fcvt_d_l(cpu_fpr[rd], cpu_env, write_int_rd, rm_reg);
#else
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
#endif
        } else if (rs2 == 0x3) {
#if defined(TARGET_RISCV64)
            gen_helper_fcvt_d_lu(cpu_fpr[rd], cpu_env, write_int_rd, rm_reg);
#else
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
#endif
        } else {
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_FMV_X_D:
        /* also OPC_RISC_FCLASS_D */
        if (rm == 0x0) { /* FMV */
            tcg_gen_mov_tl(write_int_rd, cpu_fpr[rs1]);
        } else if (rm == 0x1) {
            gen_helper_fclass_d(write_int_rd, cpu_env, cpu_fpr[rs1]);
        } else {
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        }
        gen_set_gpr(rd, write_int_rd);
        break;
    case OPC_RISC_FMV_D_X:
        gen_get_gpr(write_int_rd, rs1);
        tcg_gen_mov_tl(cpu_fpr[rd], write_int_rd);
        break;
#endif
    default:
        kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
    tcg_temp_free_i64(rm_reg);
    tcg_temp_free(write_int_rd);
}

static inline void gen_system(DisasContext *ctx, uint32_t opc,
                      int rd, int rs1, int csr)
{
    TCGv source1, csr_store, dest, rs1_pass, imm_rs1;
    source1 = tcg_temp_new();
    csr_store = tcg_temp_new();
    dest = tcg_temp_new();
    rs1_pass = tcg_temp_new();
    imm_rs1 = tcg_temp_new();
    gen_get_gpr(source1, rs1);
    tcg_gen_movi_tl(rs1_pass, rs1);
    tcg_gen_movi_tl(csr_store, csr); /* copy into temp reg to feed to helper */

    switch (opc) {
    case OPC_RISC_ECALL:
        switch (csr) {
        case 0x0: /* ECALL */
            /* always generates U-level ECALL, fixed in do_interrupt handler */
            generate_exception(ctx, RISCV_EXCP_U_ECALL);
            tcg_gen_exit_tb(0); /* no chaining */
            ctx->bstate = BS_BRANCH;
            break;
        case 0x1: /* EBREAK */
            generate_exception(ctx, RISCV_EXCP_BREAKPOINT);
            tcg_gen_exit_tb(0); /* no chaining */
            ctx->bstate = BS_BRANCH;
            break;
        case 0x002: /* URET */
            printf("URET unimplemented\n");
            exit(1);
            break;
        case 0x102: /* SRET */
            tcg_gen_movi_tl(cpu_PC, ctx->pc);
            gen_helper_sret(cpu_PC, cpu_env, cpu_PC);
            tcg_gen_exit_tb(0); /* no chaining */
            ctx->bstate = BS_BRANCH;
            break;
        case 0x202: /* HRET */
            printf("HRET unimplemented\n");
            exit(1);
            break;
        case 0x302: /* MRET */
            tcg_gen_movi_tl(cpu_PC, ctx->pc);
            gen_helper_mret(cpu_PC, cpu_env, cpu_PC);
            tcg_gen_exit_tb(0); /* no chaining */
            ctx->bstate = BS_BRANCH;
            break;
        case 0x7b2: /* DRET */
            printf("DRET unimplemented\n");
            exit(1);
            break;
        case 0x105: /* WFI */
            /* nop for now, as in spike */
            break;
        case 0x104: /* SFENCE.VM */
            gen_helper_tlb_flush(cpu_env);
            break;
        default:
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
            break;
        }
        break;
    default:
        tcg_gen_movi_tl(cpu_PC, ctx->pc);
        tcg_gen_movi_tl(imm_rs1, rs1);
        switch (opc) {
        case OPC_RISC_CSRRW:
            gen_helper_csrrw(dest, cpu_env, source1, csr_store, cpu_PC);
            break;
        case OPC_RISC_CSRRS:
            gen_helper_csrrs(dest, cpu_env, source1, csr_store, cpu_PC,
                    rs1_pass);
            break;
        case OPC_RISC_CSRRC:
            gen_helper_csrrc(dest, cpu_env, source1, csr_store, cpu_PC,
                    rs1_pass);
            break;
        case OPC_RISC_CSRRWI:
            gen_helper_csrrw(dest, cpu_env, imm_rs1, csr_store, cpu_PC);
            break;
        case OPC_RISC_CSRRSI:
            gen_helper_csrrs(dest, cpu_env, imm_rs1, csr_store, cpu_PC,
                             rs1_pass);
            break;
        case OPC_RISC_CSRRCI:
            gen_helper_csrrc(dest, cpu_env, imm_rs1, csr_store, cpu_PC,
                             rs1_pass);
            break;
        default:
            kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
            break;
        }
        gen_set_gpr(rd, dest);
        /* end tb since we may be changing priv modes, to get mmu_index right */
        tcg_gen_movi_tl(cpu_PC, ctx->pc + 4);
        tcg_gen_exit_tb(0); /* no chaining */
        ctx->bstate = BS_BRANCH;
        break;
    }
    tcg_temp_free(source1);
    tcg_temp_free(csr_store);
    tcg_temp_free(dest);
    tcg_temp_free(rs1_pass);
    tcg_temp_free(imm_rs1);
}

static void decode_opc(CPURISCVState *env, DisasContext *ctx)
{
    int rs1;
    int rs2;
    int rd;
    uint32_t op;
    int16_t imm;
    target_long ubimm;

    /* We do not do misaligned address check here: the address should never be
     * misaligned at this point. Instructions that set PC must do the check,
     * since epc must be the address of the instruction that caused us to
     * perform the misaligned instruction fetch */

    op = MASK_OP_MAJOR(ctx->opcode);
    rs1 = GET_RS1(ctx->opcode);
    rs2 = GET_RS2(ctx->opcode);
    rd = GET_RD(ctx->opcode);
    imm = GET_IMM(ctx->opcode);

    switch (op) {
    case OPC_RISC_LUI:
        if (rd == 0) {
            break; /* NOP */
        }
        tcg_gen_movi_tl(cpu_gpr[rd], (ctx->opcode & 0xFFFFF000));
        tcg_gen_ext32s_tl(cpu_gpr[rd], cpu_gpr[rd]);
        break;
    case OPC_RISC_AUIPC:
        if (rd == 0) {
            break; /* NOP */
        }
        tcg_gen_movi_tl(cpu_gpr[rd], (ctx->opcode & 0xFFFFF000));
        tcg_gen_ext32s_tl(cpu_gpr[rd], cpu_gpr[rd]);
        tcg_gen_addi_tl(cpu_gpr[rd], cpu_gpr[rd], ctx->pc);
        break;
    case OPC_RISC_JAL: {
            TCGv nextpc = tcg_temp_local_new();
            TCGv testpc = tcg_temp_local_new();
            TCGLabel *misaligned = gen_new_label();
            TCGLabel *done = gen_new_label();
            ubimm = (target_long) (GET_JAL_IMM(ctx->opcode));
            tcg_gen_movi_tl(nextpc, ctx->pc + ubimm);
            /* check misaligned: */
            tcg_gen_andi_tl(testpc, nextpc, 0x3);
            tcg_gen_brcondi_tl(TCG_COND_NE, testpc, 0x0, misaligned);
            if (rd != 0) {
                tcg_gen_movi_tl(cpu_gpr[rd], ctx->pc + 4);
            }

#ifdef DISABLE_CHAINING_JAL
            tcg_gen_mov_tl(cpu_PC, nextpc);
            tcg_gen_exit_tb(0);
#else
            gen_goto_tb(ctx, 0, ctx->pc + ubimm); /* must use this for safety */
#endif
            tcg_gen_br(done);
            gen_set_label(misaligned);
            /* throw exception for misaligned case */
            generate_exception_mbadaddr(ctx, RISCV_EXCP_INST_ADDR_MIS);
            gen_set_label(done);
            ctx->bstate = BS_BRANCH;
            tcg_temp_free(nextpc);
            tcg_temp_free(testpc);
        }
        break;
    case OPC_RISC_JALR:
        gen_jalr(ctx, MASK_OP_JALR(ctx->opcode), rd, rs1, imm);
        break;
    case OPC_RISC_BRANCH:
        gen_branch(ctx, MASK_OP_BRANCH(ctx->opcode), rs1, rs2,
                   GET_B_IMM(ctx->opcode));
        break;
    case OPC_RISC_LOAD:
        gen_load(ctx, MASK_OP_LOAD(ctx->opcode), rd, rs1, imm);
        break;
    case OPC_RISC_STORE:
        gen_store(ctx, MASK_OP_STORE(ctx->opcode), rs1, rs2,
                  GET_STORE_IMM(ctx->opcode));
        break;
    case OPC_RISC_ARITH_IMM:
#if defined(TARGET_RISCV64)
    case OPC_RISC_ARITH_IMM_W:
#endif
        if (rd == 0) {
            break; /* NOP */
        }
        gen_arith_imm(ctx, MASK_OP_ARITH_IMM(ctx->opcode), rd, rs1, imm);
        break;
    case OPC_RISC_ARITH:
#if defined(TARGET_RISCV64)
    case OPC_RISC_ARITH_W:
#endif
        if (rd == 0) {
            break; /* NOP */
        }
        gen_arith(ctx, MASK_OP_ARITH(ctx->opcode), rd, rs1, rs2);
        break;
    case OPC_RISC_FP_LOAD:
        gen_fp_load(ctx, MASK_OP_FP_LOAD(ctx->opcode), rd, rs1, imm);
        break;
    case OPC_RISC_FP_STORE:
        gen_fp_store(ctx, MASK_OP_FP_STORE(ctx->opcode), rs1, rs2,
                     GET_STORE_IMM(ctx->opcode));
        break;
    case OPC_RISC_ATOMIC:
        gen_atomic(ctx, MASK_OP_ATOMIC(ctx->opcode), rd, rs1, rs2);
        break;
    case OPC_RISC_FMADD:
        gen_fp_fmadd(ctx, MASK_OP_FP_FMADD(ctx->opcode), rd, rs1, rs2,
                     GET_RS3(ctx->opcode), GET_RM(ctx->opcode));
        break;
    case OPC_RISC_FMSUB:
        gen_fp_fmsub(ctx, MASK_OP_FP_FMSUB(ctx->opcode), rd, rs1, rs2,
                     GET_RS3(ctx->opcode), GET_RM(ctx->opcode));
        break;
    case OPC_RISC_FNMSUB:
        gen_fp_fnmsub(ctx, MASK_OP_FP_FNMSUB(ctx->opcode), rd, rs1, rs2,
                      GET_RS3(ctx->opcode), GET_RM(ctx->opcode));
        break;
    case OPC_RISC_FNMADD:
        gen_fp_fnmadd(ctx, MASK_OP_FP_FNMADD(ctx->opcode), rd, rs1, rs2,
                      GET_RS3(ctx->opcode), GET_RM(ctx->opcode));
        break;
    case OPC_RISC_FP_ARITH:
        gen_fp_arith(ctx, MASK_OP_FP_ARITH(ctx->opcode), rd, rs1, rs2,
                     GET_RM(ctx->opcode));
        break;
    case OPC_RISC_FENCE:
        /* standard fence is nop, fence_i flushes TB (like an icache): */
        if (ctx->opcode & 0x1000) { /* FENCE_I */
            gen_helper_fence_i(cpu_env);
            tcg_gen_movi_tl(cpu_PC, ctx->pc + 4);
            tcg_gen_exit_tb(0); /* no chaining */
            ctx->bstate = BS_BRANCH;
        }
        break;
    case OPC_RISC_SYSTEM:
        gen_system(ctx, MASK_OP_SYSTEM(ctx->opcode), rd, rs1,
                   (ctx->opcode & 0xFFF00000) >> 20);
        break;
    default:
        kill_unknown(ctx, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
}

void gen_intermediate_code(CPURISCVState *env, TranslationBlock *tb)
{
    RISCVCPU *cpu = riscv_env_get_cpu(env);
    CPUState *cs = CPU(cpu);
    DisasContext ctx;
    target_ulong pc_start;
    target_ulong next_page_start;
    int num_insns;
    int max_insns;
    pc_start = tb->pc;
    next_page_start = (pc_start & TARGET_PAGE_MASK) + TARGET_PAGE_SIZE;
    ctx.pc = pc_start;

    /* once we have GDB, the rest of the translate.c implementation should be
       ready for singlestep */
    ctx.singlestep_enabled = cs->singlestep_enabled;

    ctx.tb = tb;
    ctx.bstate = BS_NONE;

    ctx.mem_idx = cpu_mmu_index(env, false);
    num_insns = 0;
    max_insns = tb->cflags & CF_COUNT_MASK;
    if (max_insns == 0) {
        max_insns = CF_COUNT_MASK;
    }
    if (max_insns > TCG_MAX_INSNS) {
        max_insns = TCG_MAX_INSNS;
    }
    gen_tb_start(tb);

    while (ctx.bstate == BS_NONE) {
        tcg_gen_insn_start(ctx.pc);
        num_insns++;

        if (unlikely(cpu_breakpoint_test(cs, ctx.pc, BP_ANY))) {
            tcg_gen_movi_tl(cpu_PC, ctx.pc);
            ctx.bstate = BS_BRANCH;
            gen_helper_raise_exception_debug(cpu_env);
            /* The address covered by the breakpoint must be included in
               [tb->pc, tb->pc + tb->size) in order to for it to be
               properly cleared -- thus we increment the PC here so that
               the logic setting tb->size below does the right thing.  */
            ctx.pc += 4;
            goto done_generating;
        }

        if (num_insns == max_insns && (tb->cflags & CF_LAST_IO)) {
            gen_io_start();
        }

        ctx.opcode = cpu_ldl_code(env, ctx.pc);
        decode_opc(env, &ctx);
        ctx.pc += 4;

        if (cs->singlestep_enabled) {
            break;
        }
        if (ctx.pc >= next_page_start) {
            break;
        }
        if (tcg_op_buf_full()) {
            break;
        }
        if (num_insns >= max_insns) {
            break;
        }
        if (singlestep) {
            break;
        }

    }
    if (tb->cflags & CF_LAST_IO) {
        gen_io_end();
    }
    if (cs->singlestep_enabled && ctx.bstate != BS_BRANCH) {
        if (ctx.bstate == BS_NONE) {
            tcg_gen_movi_tl(cpu_PC, ctx.pc);
        }
        gen_helper_raise_exception_debug(cpu_env);
    } else {
        switch (ctx.bstate) {
        case BS_STOP:
            gen_goto_tb(&ctx, 0, ctx.pc);
            break;
        case BS_NONE: /* handle end of page - DO NOT CHAIN. See gen_goto_tb. */
            tcg_gen_movi_tl(cpu_PC, ctx.pc);
            tcg_gen_exit_tb(0);
            break;
        case BS_BRANCH: /* ops using BS_BRANCH generate own exit seq */
        default:
            break;
        }
    }
done_generating:
    gen_tb_end(tb, num_insns);
    tb->size = ctx.pc - pc_start;
    tb->icount = num_insns;
}

void riscv_cpu_dump_state(CPUState *cs, FILE *f, fprintf_function cpu_fprintf,
                         int flags)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;
    int i;

    cpu_fprintf(f, "pc=0x" TARGET_FMT_lx "\n", env->PC);
    for (i = 0; i < 32; i++) {
        cpu_fprintf(f, " %s " TARGET_FMT_lx, regnames[i], env->gpr[i]);
        if ((i & 3) == 3) {
            cpu_fprintf(f, "\n");
        }
    }

    cpu_fprintf(f, " %s " TARGET_FMT_lx "\n", "MSTATUS ",
                env->csr[CSR_MSTATUS]);
    cpu_fprintf(f, " %s " TARGET_FMT_lx "\n", "MIP     ", env->csr[CSR_MIP]);
    cpu_fprintf(f, " %s " TARGET_FMT_lx "\n", "MIE     ", env->csr[CSR_MIE]);

    for (i = 0; i < 32; i++) {
        if ((i & 3) == 0) {
            cpu_fprintf(f, "FPR%02d:", i);
        }
        cpu_fprintf(f, " %s %016lx", fpr_regnames[i], env->fpr[i]);
        if ((i & 3) == 3) {
            cpu_fprintf(f, "\n");
        }
    }
}

void riscv_tcg_init(void)
{
    int i;
    static int inited;

    /* Initialize various static tables. */
    if (inited) {
        return;
    }

    cpu_env = tcg_global_reg_new_ptr(TCG_AREG0, "env");

    /* WARNING: cpu_gpr[0] is not allocated ON PURPOSE. Do not use it. */
    /* Use the gen_set_gpr and gen_get_gpr helper functions when accessing */
    /* registers, unless you specifically block reads/writes to reg 0 */
    TCGV_UNUSED(cpu_gpr[0]);
    for (i = 1; i < 32; i++) {
        cpu_gpr[i] = tcg_global_mem_new(cpu_env,
                             offsetof(CPURISCVState, gpr[i]), regnames[i]);
    }

    for (i = 0; i < 32; i++) {
        cpu_fpr[i] = tcg_global_mem_new_i64(cpu_env,
                             offsetof(CPURISCVState, fpr[i]), fpr_regnames[i]);
    }

    cpu_PC = tcg_global_mem_new(cpu_env, offsetof(CPURISCVState, PC), "PC");
    load_res = tcg_global_mem_new(cpu_env, offsetof(CPURISCVState, load_res),
                             "load_res");
    inited = 1;
}

#define MCPUID_RV64I   (2L << (TARGET_LONG_BITS - 2))
#define MCPUID_RV32I   (1L << (TARGET_LONG_BITS - 2))
#define MCPUID_SUPER   (1L << ('S' - 'A'))
#define MCPUID_USER    (1L << ('U' - 'A'))
#define MCPUID_I       (1L << ('I' - 'A'))
#define MCPUID_M       (1L << ('M' - 'A'))
#define MCPUID_A       (1L << ('A' - 'A'))
#define MCPUID_F       (1L << ('F' - 'A'))
#define MCPUID_D       (1L << ('D' - 'A'))

struct riscv_def_t {
    const char *name;
    uint64_t init_misa_reg;
};

/* RISC-V CPU definitions */
static const riscv_def_t riscv_defs[] = {
    {
        .name = "riscv",
#if defined(TARGET_RISCV64)
        /* RV64G */
        .init_misa_reg = MCPUID_RV64I | MCPUID_SUPER | MCPUID_USER | MCPUID_I
            | MCPUID_M | MCPUID_A | MCPUID_F | MCPUID_D,
#else
        /* RV32G */
        .init_misa_reg = MCPUID_RV32I | MCPUID_SUPER | MCPUID_USER | MCPUID_I
            | MCPUID_M | MCPUID_A | MCPUID_F | MCPUID_D,
#endif
    },
};

static const riscv_def_t *cpu_riscv_find_by_name(const char *name)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(riscv_defs); i++) {
        if (strcasecmp(name, riscv_defs[i].name) == 0) {
            return &riscv_defs[i];
        }
    }
    return NULL;
}

void riscv_cpu_list(FILE *f, fprintf_function cpu_fprintf)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(riscv_defs); i++) {
        (*cpu_fprintf)(f, "RISCV '%s'\n", riscv_defs[i].name);
    }
}

RISCVCPU *cpu_riscv_init(const char *cpu_model)
{
    RISCVCPU *cpu;
    CPURISCVState *env;
    const riscv_def_t *def;

    def = cpu_riscv_find_by_name(cpu_model);
    if (!def) {
        return NULL;
    }
    cpu = RISCV_CPU(object_new(TYPE_RISCV_CPU));
    env = &cpu->env;
    env->cpu_model = def;

    memset(env->csr, 0, 4096 * sizeof(target_ulong));
    env->priv = PRV_M;

    /* set mcpuid from def */
    env->csr[CSR_MISA] = def->init_misa_reg;
    object_property_set_bool(OBJECT(cpu), true, "realized", NULL);

    /* fpu flags: */
    set_default_nan_mode(1, &env->fp_status);

    return cpu;
}

void restore_state_to_opc(CPURISCVState *env, TranslationBlock *tb,
                          target_ulong *data)
{
    env->PC = data[0];
}
