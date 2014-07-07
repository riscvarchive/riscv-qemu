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
static inline void cpu_mips_tlb_flush (CPUMIPSState *env, int flush_global);
#endif

/*****************************************************************************/
/* Exceptions processing helpers */

static inline void QEMU_NORETURN do_raise_exception_err(CPUMIPSState *env,
                                                        uint32_t exception,
                                                        int error_code,
                                                        uintptr_t pc)
{
    CPUState *cs = CPU(mips_env_get_cpu(env));

    if (exception < EXCP_SC) {
        qemu_log("%s: %d %d\n", __func__, exception, error_code);
    }
    cs->exception_index = exception;
    env->error_code = error_code;

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

#if defined(CONFIG_USER_ONLY)
#define HELPER_LD(name, insn, type)                                     \
static inline type do_##name(CPUMIPSState *env, target_ulong addr,      \
                             int mem_idx)                               \
{                                                                       \
    return (type) insn##_raw(addr);                                     \
}
#else
#define HELPER_LD(name, insn, type)                                     \
static inline type do_##name(CPUMIPSState *env, target_ulong addr,      \
                             int mem_idx)                               \
{                                                                       \
    switch (mem_idx)                                                    \
    {                                                                   \
    case 0: return (type) cpu_##insn##_kernel(env, addr); break;        \
    case 1: return (type) cpu_##insn##_super(env, addr); break;         \
    default:                                                            \
    case 2: return (type) cpu_##insn##_user(env, addr); break;          \
    }                                                                   \
}
#endif
HELPER_LD(lbu, ldub, uint8_t)
HELPER_LD(lw, ldl, int32_t)
#ifdef TARGET_MIPS64
HELPER_LD(ld, ldq, int64_t)
#endif
#undef HELPER_LD

#if defined(CONFIG_USER_ONLY)
#define HELPER_ST(name, insn, type)                                     \
static inline void do_##name(CPUMIPSState *env, target_ulong addr,      \
                             type val, int mem_idx)                     \
{                                                                       \
    insn##_raw(addr, val);                                              \
}
#else
#define HELPER_ST(name, insn, type)                                     \
static inline void do_##name(CPUMIPSState *env, target_ulong addr,      \
                             type val, int mem_idx)                     \
{                                                                       \
    switch (mem_idx)                                                    \
    {                                                                   \
    case 0: cpu_##insn##_kernel(env, addr, val); break;                 \
    case 1: cpu_##insn##_super(env, addr, val); break;                  \
    default:                                                            \
    case 2: cpu_##insn##_user(env, addr, val); break;                   \
    }                                                                   \
}
#endif
HELPER_ST(sb, stb, uint8_t)
HELPER_ST(sw, stl, uint32_t)
#ifdef TARGET_MIPS64
HELPER_ST(sd, stq, uint64_t)
#endif
#undef HELPER_ST

/* 64 bits arithmetic for 32 bits hosts */
static inline uint64_t get_HILO(CPUMIPSState *env)
{
    return ((uint64_t)(env->active_tc.HI[0]) << 32) | (uint32_t)env->active_tc.LO[0];
}

static inline target_ulong set_HIT0_LO(CPUMIPSState *env, uint64_t HILO)
{
    target_ulong tmp;
    env->active_tc.LO[0] = (int32_t)(HILO & 0xFFFFFFFF);
    tmp = env->active_tc.HI[0] = (int32_t)(HILO >> 32);
    return tmp;
}

static inline target_ulong set_HI_LOT0(CPUMIPSState *env, uint64_t HILO)
{
    target_ulong tmp = env->active_tc.LO[0] = (int32_t)(HILO & 0xFFFFFFFF);
    env->active_tc.HI[0] = (int32_t)(HILO >> 32);
    return tmp;
}

/* Multiplication variants of the vr54xx. */

/* MODIFIED FOR RISCV*/
target_ulong helper_mulsu(CPUMIPSState *env, target_ulong arg1,
                          target_ulong arg2)
{
    int64_t a = arg1;
    uint64_t b = arg2;
    return (int64_t)((__int128_t)a*b >> 64);
}


/* RISCV exception raise */
/*void helper_riscv_exception(CPUMIPSState *env, int excp_no) {
    CPUState *cs = CPU(mips_env_get_cpu(env));
    cs->exception_index = excp_no;

    // required?
//    cpu_loop_exit(cs);

}*/

#ifndef CONFIG_USER_ONLY

static inline hwaddr do_translate_address(CPUMIPSState *env,
                                                      target_ulong address,
                                                      int rw)
{
    hwaddr lladdr;

    lladdr = cpu_mips_translate_address(env, address, rw);

    if (lladdr == -1LL) {
        cpu_loop_exit(CPU(mips_env_get_cpu(env)));
    } else {
        return lladdr;
    }
}

#define HELPER_LD_ATOMIC(name, insn)                                          \
target_ulong helper_##name(CPUMIPSState *env, target_ulong arg, int mem_idx)  \
{                                                                             \
    env->lladdr = do_translate_address(env, arg, 0);                          \
    env->llval = do_##insn(env, arg, mem_idx);                                \
    return env->llval;                                                        \
}
HELPER_LD_ATOMIC(ll, lw)
#ifdef TARGET_MIPS64
HELPER_LD_ATOMIC(lld, ld)
#endif
#undef HELPER_LD_ATOMIC

#define HELPER_ST_ATOMIC(name, ld_insn, st_insn, almask)                      \
target_ulong helper_##name(CPUMIPSState *env, target_ulong arg1,              \
                           target_ulong arg2, int mem_idx)                    \
{                                                                             \
    target_long tmp;                                                          \
                                                                              \
    if (arg2 & almask) {                                                      \
        env->CP0_BadVAddr = arg2;                                             \
        printf("GOT HERE GOT HERE WAT WAT\n");                                \
        helper_raise_exception(env, EXCP_AdES);                               \
    }                                                                         \
    if (do_translate_address(env, arg2, 1) == env->lladdr) {                  \
        tmp = do_##ld_insn(env, arg2, mem_idx);                               \
        if (tmp == env->llval) {                                              \
            do_##st_insn(env, arg2, arg1, mem_idx);                           \
            return 1;                                                         \
        }                                                                     \
    }                                                                         \
    return 0;                                                                 \
}
HELPER_ST_ATOMIC(sc, lw, sw, 0x3)
#ifdef TARGET_MIPS64
HELPER_ST_ATOMIC(scd, ld, sd, 0x7)
#endif
#undef HELPER_ST_ATOMIC
#endif

#ifdef TARGET_WORDS_BIGENDIAN
#define GET_LMASK(v) ((v) & 3)
#define GET_OFFSET(addr, offset) (addr + (offset))
#else
#define GET_LMASK(v) (((v) & 3) ^ 3)
#define GET_OFFSET(addr, offset) (addr - (offset))
#endif

static const int multiple_regs[] = { 16, 17, 18, 19, 20, 21, 22, 23, 30 };

#ifndef CONFIG_USER_ONLY
/* TLB management */
static void cpu_mips_tlb_flush (CPUMIPSState *env, int flush_global)
{
    MIPSCPU *cpu = mips_env_get_cpu(env);

    /* Flush qemu's TLB and discard all shadowed entries.  */
    tlb_flush(CPU(cpu), flush_global);
    env->tlb->tlb_in_use = env->tlb->nb_tlb;
}

static void r4k_mips_tlb_flush_extra (CPUMIPSState *env, int first)
{
    /* Discard entries from env->tlb[first] onwards.  */
    while (env->tlb->tlb_in_use > first) {
        r4k_invalidate_tlb(env, --env->tlb->tlb_in_use, 0);
    }
}

static void r4k_fill_tlb(CPUMIPSState *env, int idx)
{
    r4k_tlb_t *tlb;

    /* XXX: detect conflicting TLBs and raise a MCHECK exception when needed */
    tlb = &env->tlb->mmu.r4k.tlb[idx];
    tlb->VPN = env->CP0_EntryHi & (TARGET_PAGE_MASK << 1);
#if defined(TARGET_MIPS64)
    tlb->VPN &= env->SEGMask;
#endif
    tlb->ASID = env->CP0_EntryHi & 0xFF;
    tlb->PageMask = env->CP0_PageMask;
    tlb->G = env->CP0_EntryLo0 & env->CP0_EntryLo1 & 1;
    tlb->V0 = (env->CP0_EntryLo0 & 2) != 0;
    tlb->D0 = (env->CP0_EntryLo0 & 4) != 0;
    tlb->C0 = (env->CP0_EntryLo0 >> 3) & 0x7;
    tlb->PFN[0] = (env->CP0_EntryLo0 >> 6) << 12;
    tlb->V1 = (env->CP0_EntryLo1 & 2) != 0;
    tlb->D1 = (env->CP0_EntryLo1 & 4) != 0;
    tlb->C1 = (env->CP0_EntryLo1 >> 3) & 0x7;
    tlb->PFN[1] = (env->CP0_EntryLo1 >> 6) << 12;
}

void r4k_helper_tlbwi(CPUMIPSState *env)
{
    r4k_tlb_t *tlb;
    int idx;
    target_ulong VPN;
    uint8_t ASID;
    bool G, V0, D0, V1, D1;

    idx = (env->CP0_Index & ~0x80000000) % env->tlb->nb_tlb;
    tlb = &env->tlb->mmu.r4k.tlb[idx];
    VPN = env->CP0_EntryHi & (TARGET_PAGE_MASK << 1);
#if defined(TARGET_MIPS64)
    VPN &= env->SEGMask;
#endif
    ASID = env->CP0_EntryHi & 0xff;
    G = env->CP0_EntryLo0 & env->CP0_EntryLo1 & 1;
    V0 = (env->CP0_EntryLo0 & 2) != 0;
    D0 = (env->CP0_EntryLo0 & 4) != 0;
    V1 = (env->CP0_EntryLo1 & 2) != 0;
    D1 = (env->CP0_EntryLo1 & 4) != 0;

    /* Discard cached TLB entries, unless tlbwi is just upgrading access
       permissions on the current entry. */
    if (tlb->VPN != VPN || tlb->ASID != ASID || tlb->G != G ||
        (tlb->V0 && !V0) || (tlb->D0 && !D0) ||
        (tlb->V1 && !V1) || (tlb->D1 && !D1)) {
        r4k_mips_tlb_flush_extra(env, env->tlb->nb_tlb);
    }

    r4k_invalidate_tlb(env, idx, 0);
    r4k_fill_tlb(env, idx);
}

void r4k_helper_tlbwr(CPUMIPSState *env)
{
    int r = cpu_mips_get_random(env);

    r4k_invalidate_tlb(env, r, 1);
    r4k_fill_tlb(env, r);
}

void r4k_helper_tlbp(CPUMIPSState *env)
{
    r4k_tlb_t *tlb;
    target_ulong mask;
    target_ulong tag;
    target_ulong VPN;
    uint8_t ASID;
    int i;

    ASID = env->CP0_EntryHi & 0xFF;
    for (i = 0; i < env->tlb->nb_tlb; i++) {
        tlb = &env->tlb->mmu.r4k.tlb[i];
        /* 1k pages are not supported. */
        mask = tlb->PageMask | ~(TARGET_PAGE_MASK << 1);
        tag = env->CP0_EntryHi & ~mask;
        VPN = tlb->VPN & ~mask;
#if defined(TARGET_MIPS64)
        tag &= env->SEGMask;
#endif
        /* Check ASID, virtual page number & size */
        if ((tlb->G == 1 || tlb->ASID == ASID) && VPN == tag) {
            /* TLB match */
            env->CP0_Index = i;
            break;
        }
    }
    if (i == env->tlb->nb_tlb) {
        /* No match.  Discard any shadow entries, if any of them match.  */
        for (i = env->tlb->nb_tlb; i < env->tlb->tlb_in_use; i++) {
            tlb = &env->tlb->mmu.r4k.tlb[i];
            /* 1k pages are not supported. */
            mask = tlb->PageMask | ~(TARGET_PAGE_MASK << 1);
            tag = env->CP0_EntryHi & ~mask;
            VPN = tlb->VPN & ~mask;
#if defined(TARGET_MIPS64)
            tag &= env->SEGMask;
#endif
            /* Check ASID, virtual page number & size */
            if ((tlb->G == 1 || tlb->ASID == ASID) && VPN == tag) {
                r4k_mips_tlb_flush_extra (env, i);
                break;
            }
        }

        env->CP0_Index |= 0x80000000;
    }
}

void r4k_helper_tlbr(CPUMIPSState *env)
{
    r4k_tlb_t *tlb;
    uint8_t ASID;
    int idx;

    ASID = env->CP0_EntryHi & 0xFF;
    idx = (env->CP0_Index & ~0x80000000) % env->tlb->nb_tlb;
    tlb = &env->tlb->mmu.r4k.tlb[idx];

    /* If this will change the current ASID, flush qemu's TLB.  */
    if (ASID != tlb->ASID)
        cpu_mips_tlb_flush (env, 1);

    r4k_mips_tlb_flush_extra(env, env->tlb->nb_tlb);

    env->CP0_EntryHi = tlb->VPN | tlb->ASID;
    env->CP0_PageMask = tlb->PageMask;
    env->CP0_EntryLo0 = tlb->G | (tlb->V0 << 1) | (tlb->D0 << 2) |
                        (tlb->C0 << 3) | (tlb->PFN[0] >> 6);
    env->CP0_EntryLo1 = tlb->G | (tlb->V1 << 1) | (tlb->D1 << 2) |
                        (tlb->C1 << 3) | (tlb->PFN[1] >> 6);
}

void helper_tlbwi(CPUMIPSState *env)
{
    env->tlb->helper_tlbwi(env);
}

void helper_tlbwr(CPUMIPSState *env)
{
    env->tlb->helper_tlbwr(env);
}

void helper_tlbp(CPUMIPSState *env)
{
    env->tlb->helper_tlbp(env);
}

void helper_tlbr(CPUMIPSState *env)
{
    env->tlb->helper_tlbr(env);
}

/* Specials */
target_ulong helper_di(CPUMIPSState *env)
{
    target_ulong t0 = env->CP0_Status;

    env->CP0_Status = t0 & ~(1 << CP0St_IE);
    return t0;
}

target_ulong helper_ei(CPUMIPSState *env)
{
    target_ulong t0 = env->CP0_Status;

    env->CP0_Status = t0 | (1 << CP0St_IE);
    return t0;
}

static void debug_pre_eret(CPUMIPSState *env)
{
    if (qemu_loglevel_mask(CPU_LOG_EXEC)) {
        qemu_log("ERET: PC " TARGET_FMT_lx " EPC " TARGET_FMT_lx,
                env->active_tc.PC, env->CP0_EPC);
        if (env->CP0_Status & (1 << CP0St_ERL))
            qemu_log(" ErrorEPC " TARGET_FMT_lx, env->CP0_ErrorEPC);
        if (env->hflags & MIPS_HFLAG_DM)
            qemu_log(" DEPC " TARGET_FMT_lx, env->CP0_DEPC);
        qemu_log("\n");
    }
}

static void debug_post_eret(CPUMIPSState *env)
{
    MIPSCPU *cpu = mips_env_get_cpu(env);

    if (qemu_loglevel_mask(CPU_LOG_EXEC)) {
        qemu_log("  =>  PC " TARGET_FMT_lx " EPC " TARGET_FMT_lx,
                env->active_tc.PC, env->CP0_EPC);
        if (env->CP0_Status & (1 << CP0St_ERL))
            qemu_log(" ErrorEPC " TARGET_FMT_lx, env->CP0_ErrorEPC);
        if (env->hflags & MIPS_HFLAG_DM)
            qemu_log(" DEPC " TARGET_FMT_lx, env->CP0_DEPC);
        switch (env->hflags & MIPS_HFLAG_KSU) {
        case MIPS_HFLAG_UM: qemu_log(", UM\n"); break;
        case MIPS_HFLAG_SM: qemu_log(", SM\n"); break;
        case MIPS_HFLAG_KM: qemu_log("\n"); break;
        default:
            cpu_abort(CPU(cpu), "Invalid MMU mode!\n");
            break;
        }
    }
}

static void set_pc(CPUMIPSState *env, target_ulong error_pc)
{
    env->active_tc.PC = error_pc & ~(target_ulong)1;
    if (error_pc & 1) {
        env->hflags |= MIPS_HFLAG_M16;
    } else {
        env->hflags &= ~(MIPS_HFLAG_M16);
    }
}

void helper_eret(CPUMIPSState *env)
{
    debug_pre_eret(env);
    if (env->CP0_Status & (1 << CP0St_ERL)) {
        set_pc(env, env->CP0_ErrorEPC);
        env->CP0_Status &= ~(1 << CP0St_ERL);
    } else {
        set_pc(env, env->CP0_EPC);
        env->CP0_Status &= ~(1 << CP0St_EXL);
    }
    compute_hflags(env);
    debug_post_eret(env);
    env->lladdr = 1;
}

void helper_deret(CPUMIPSState *env)
{
    debug_pre_eret(env);
    set_pc(env, env->CP0_DEPC);

    env->hflags &= MIPS_HFLAG_DM;
    compute_hflags(env);
    debug_post_eret(env);
    env->lladdr = 1;
}
#endif /* !CONFIG_USER_ONLY */

target_ulong helper_rdhwr_cpunum(CPUMIPSState *env)
{
    if ((env->hflags & MIPS_HFLAG_CP0) ||
        (env->CP0_HWREna & (1 << 0)))
        return env->CP0_EBase & 0x3ff;
    else
        helper_raise_exception(env, EXCP_RI);

    return 0;
}

target_ulong helper_rdhwr_synci_step(CPUMIPSState *env)
{
    if ((env->hflags & MIPS_HFLAG_CP0) ||
        (env->CP0_HWREna & (1 << 1)))
        return env->SYNCI_Step;
    else
        helper_raise_exception(env, EXCP_RI);

    return 0;
}

target_ulong helper_rdhwr_cc(CPUMIPSState *env)
{
    if ((env->hflags & MIPS_HFLAG_CP0) ||
        (env->CP0_HWREna & (1 << 2)))
        return env->CP0_Count;
    else
        helper_raise_exception(env, EXCP_RI);

    return 0;
}

target_ulong helper_rdhwr_ccres(CPUMIPSState *env)
{
    if ((env->hflags & MIPS_HFLAG_CP0) ||
        (env->CP0_HWREna & (1 << 3)))
        return env->CCRes;
    else
        helper_raise_exception(env, EXCP_RI);

    return 0;
}

void helper_pmon(CPUMIPSState *env, int function)
{
    function /= 2;
    switch (function) {
    case 2: /* TODO: char inbyte(int waitflag); */
        if (env->active_tc.gpr[4] == 0)
            env->active_tc.gpr[2] = -1;
        /* Fall through */
    case 11: /* TODO: char inbyte (void); */
        env->active_tc.gpr[2] = -1;
        break;
    case 3:
    case 12:
        printf("%c", (char)(env->active_tc.gpr[4] & 0xFF));
        break;
    case 17:
        break;
    case 158:
        {
            unsigned char *fmt = (void *)(uintptr_t)env->active_tc.gpr[4];
            printf("%s", fmt);
        }
        break;
    }
}

void helper_wait(CPUMIPSState *env)
{
    CPUState *cs = CPU(mips_env_get_cpu(env));

    cs->halted = 1;
    cpu_reset_interrupt(cs, CPU_INTERRUPT_WAKE);
    helper_raise_exception(env, EXCP_HLT);
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
    env->CP0_BadVAddr = addr;
    printf("REACHED DO UNALIGNED ACCESS\n");
    printf("%016lX\n", (uint64_t)addr);
    exit(0);
//    helper_riscv_exception(env, (is_write == 1) ? RISCV_EXCP_STORE_ADDR_MIS : RISCV_EXCP_LOAD_ADDR_MIS);
}

void tlb_fill(CPUState *cs, target_ulong addr, int is_write, int mmu_idx,
              uintptr_t retaddr)
{
    int ret;

    ret = mips_cpu_handle_mmu_fault(cs, addr, is_write, mmu_idx);
    if (ret) {
        MIPSCPU *cpu = MIPS_CPU(cs);
        CPUMIPSState *env = &cpu->env;

        do_raise_exception_err(env, cs->exception_index,
                               env->error_code, retaddr);
    }
}

void mips_cpu_unassigned_access(CPUState *cs, hwaddr addr,
                                bool is_write, bool is_exec, int unused,
                                unsigned size)
{
    MIPSCPU *cpu = MIPS_CPU(cs);
    CPUMIPSState *env = &cpu->env;

    if (is_exec) {
        helper_raise_exception(env, EXCP_IBE);
    } else {
        helper_raise_exception(env, EXCP_DBE);
    }
}
#endif /* !CONFIG_USER_ONLY */
