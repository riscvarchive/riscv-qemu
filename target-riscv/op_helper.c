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

#include "qemu/osdep.h"
#include <stdlib.h>
#include "cpu.h"
#include "qemu/host-utils.h"
#include "exec/helper-proto.h"

int validate_priv(target_ulong priv)
{
    return priv == PRV_U || priv == PRV_S || priv == PRV_M;
}

void set_privilege(CPURISCVState *env, target_ulong newpriv)
{
    if (!validate_priv(newpriv)) {
        printf("INVALID PRIV SET\n");
        exit(1);
    }
    helper_tlb_flush(env);
    env->priv = newpriv;
}

static int validate_vm(target_ulong vm)
{
    return vm == VM_SV32 || vm == VM_SV39 || vm == VM_SV48 || vm == VM_MBARE;
}

/* Exceptions processing helpers */
static inline void QEMU_NORETURN do_raise_exception_err(CPURISCVState *env,
                                          uint32_t exception, uintptr_t pc)
{
    CPUState *cs = CPU(riscv_env_get_cpu(env));
    qemu_log_mask(CPU_LOG_INT, "%s: %d\n", __func__, exception);
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

void helper_raise_exception_mbadaddr(CPURISCVState *env, uint32_t exception,
        target_ulong bad_pc) {
    env->badaddr = bad_pc;
    do_raise_exception_err(env, exception, 0);
}

#if defined(TARGET_RISCV64)
target_ulong helper_mulhsu(CPURISCVState *env, target_ulong arg1,
                          target_ulong arg2)
{
    int64_t a = arg1;
    uint64_t b = arg2;
    return (int64_t)((__int128_t)a * b >> 64);
}
#endif

/*
 * Handle writes to CSRs and any resulting special behavior
 *
 * Adapted from Spike's processor_t::set_csr
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

    switch (csrno) {
    case CSR_FFLAGS:
        env->csr[CSR_MSTATUS] |= MSTATUS_FS | MSTATUS64_SD;
        env->csr[CSR_FFLAGS] = val_to_write & (FSR_AEXC >> FSR_AEXC_SHIFT);
        break;
    case CSR_FRM:
        env->csr[CSR_MSTATUS] |= MSTATUS_FS | MSTATUS64_SD;
        env->csr[CSR_FRM] = val_to_write & (FSR_RD >> FSR_RD_SHIFT);
        break;
    case CSR_FCSR:
        env->csr[CSR_MSTATUS] |= MSTATUS_FS | MSTATUS64_SD;
        env->csr[CSR_FFLAGS] = (val_to_write & FSR_AEXC) >> FSR_AEXC_SHIFT;
        env->csr[CSR_FRM] = (val_to_write & FSR_RD) >> FSR_RD_SHIFT;
        break;
    case CSR_MSTATUS: {
        target_ulong mstatus = env->csr[CSR_MSTATUS];
        if ((val_to_write ^ mstatus) &
            (MSTATUS_VM | MSTATUS_MPP | MSTATUS_MPRV | MSTATUS_PUM |
             MSTATUS_MXR)) {
            helper_tlb_flush(env);
        }

        /* no extension support */
        target_ulong mask = MSTATUS_SIE | MSTATUS_SPIE | MSTATUS_MIE
            | MSTATUS_MPIE | MSTATUS_SPP | MSTATUS_FS | MSTATUS_MPRV
            | MSTATUS_PUM | MSTATUS_MXR;

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
        env->csr[CSR_MSTATUS] = mstatus;
        break;
    }
    case CSR_MIP: {
        target_ulong mask = MIP_SSIP | MIP_STIP;
        env->csr[CSR_MIP] = (env->csr[CSR_MIP] & ~mask) |
            (val_to_write & mask);
        if (env->csr[CSR_MIP] & MIP_SSIP) {
            qemu_irq_raise(SSIP_IRQ);
        } else {
            qemu_irq_lower(SSIP_IRQ);
        }
        if (env->csr[CSR_MIP] & MIP_STIP) {
            qemu_irq_raise(STIP_IRQ);
        } else {
            qemu_irq_lower(STIP_IRQ);
        }
        if (env->csr[CSR_MIP] & MIP_MSIP) {
            qemu_irq_raise(MSIP_IRQ);
        } else {
            qemu_irq_lower(MSIP_IRQ);
        }
        break;
    }
    case CSR_MIE: {
        env->csr[CSR_MIE] = (env->csr[CSR_MIE] & ~all_ints) |
            (val_to_write & all_ints);
        break;
    }
    case CSR_MIDELEG:
        env->csr[CSR_MIDELEG] = (env->csr[CSR_MIDELEG] & ~delegable_ints)
                                | (val_to_write & delegable_ints);
        break;
    case CSR_MEDELEG: {
        target_ulong mask = 0;
        mask |= 1ULL << (RISCV_EXCP_INST_ADDR_MIS);
        mask |= 1ULL << (RISCV_EXCP_INST_ACCESS_FAULT);
        mask |= 1ULL << (RISCV_EXCP_ILLEGAL_INST);
        mask |= 1ULL << (RISCV_EXCP_BREAKPOINT);
        mask |= 1ULL << (RISCV_EXCP_LOAD_ADDR_MIS);
        mask |= 1ULL << (RISCV_EXCP_LOAD_ACCESS_FAULT);
        mask |= 1ULL << (RISCV_EXCP_STORE_AMO_ADDR_MIS);
        mask |= 1ULL << (RISCV_EXCP_STORE_AMO_ACCESS_FAULT);
        mask |= 1ULL << (RISCV_EXCP_U_ECALL);
        mask |= 1ULL << (RISCV_EXCP_S_ECALL);
        mask |= 1ULL << (RISCV_EXCP_H_ECALL);
        mask |= 1ULL << (RISCV_EXCP_M_ECALL);
        env->csr[CSR_MEDELEG] = (env->csr[CSR_MEDELEG] & ~mask)
                                | (val_to_write & mask);
        break;
    }
    case CSR_MUCOUNTEREN:
        env->csr[CSR_MUCOUNTEREN] = val_to_write & 7;
        break;
    case CSR_MSCOUNTEREN:
        env->csr[CSR_MSCOUNTEREN] = val_to_write & 7;
        break;
    case CSR_SSTATUS: {
        target_ulong ms = env->csr[CSR_MSTATUS];
        target_ulong mask = SSTATUS_SIE | SSTATUS_SPIE | SSTATUS_SPP
                            | SSTATUS_FS | SSTATUS_XS | SSTATUS_PUM;
        ms = (ms & ~mask) | (val_to_write & mask);
        csr_write_helper(env, ms, CSR_MSTATUS);
        break;
    }
    case CSR_SIP: {
        target_ulong next_mip = (env->csr[CSR_MIP] & ~env->csr[CSR_MIDELEG])
                                | (val_to_write & env->csr[CSR_MIDELEG]);
        csr_write_helper(env, next_mip, CSR_MIP);
        /* note: stw_phys should be done by the call to set MIP if necessary, */
        /* so we don't do it here */
        break;
    }
    case CSR_SIE: {
        target_ulong next_mie = (env->csr[CSR_MIE] & ~env->csr[CSR_MIDELEG])
                                | (val_to_write & env->csr[CSR_MIDELEG]);
        csr_write_helper(env, next_mie, CSR_MIE);
        break;
    }
    case CSR_SPTBR: {
        env->csr[CSR_SPTBR] = val_to_write & (((target_ulong)1 <<
                              (TARGET_PHYS_ADDR_SPACE_BITS - PGSHIFT)) - 1);
        break;
    }
    case CSR_SEPC:
        env->csr[CSR_SEPC] = val_to_write;
        break;
    case CSR_STVEC:
        env->csr[CSR_STVEC] = val_to_write >> 2 << 2;
        break;
    case CSR_SSCRATCH:
        env->csr[CSR_SSCRATCH] = val_to_write;
        break;
    case CSR_SCAUSE:
        env->csr[CSR_SCAUSE] = val_to_write;
        break;
    case CSR_SBADADDR:
        env->csr[CSR_SBADADDR] = val_to_write;
        break;
    case CSR_MEPC:
        env->csr[CSR_MEPC] = val_to_write;
        break;
    case CSR_MTVEC:
        env->csr[CSR_MTVEC] = val_to_write >> 2 << 2;
        break;
    case CSR_MSCRATCH:
        env->csr[CSR_MSCRATCH] = val_to_write;
        break;
    case CSR_MCAUSE:
        env->csr[CSR_MCAUSE] = val_to_write;
        break;
    case CSR_MBADADDR:
        env->csr[CSR_MBADADDR] = val_to_write;
        break;
    case CSR_DCSR:
        printf("DEBUG NOT SUPPORTED\n");
        exit(1);
        break;
    case CSR_DPC:
        printf("DEBUG NOT SUPPORTED\n");
        exit(1);
        break;
    case CSR_DSCRATCH:
        printf("DEBUG NOT SUPPORTED\n");
        exit(1);
        break;
    }
}

/*
 * Handle reads to CSRs and any resulting special behavior
 *
 * Adapted from Spike's processor_t::get_csr
 */
inline target_ulong csr_read_helper(CPURISCVState *env, target_ulong csrno)
{
    int csrno2 = (int)csrno;
    #ifdef RISCV_DEBUG_PRINT
    fprintf(stderr, "READ CSR 0x%x\n", csrno2);
    #endif

    switch (csrno2) {
    case CSR_FFLAGS:
        return env->csr[CSR_FFLAGS];
    case CSR_FRM:
        return env->csr[CSR_FRM];
    case CSR_FCSR:
        return env->csr[CSR_FFLAGS] << FSR_AEXC_SHIFT |
               env->csr[CSR_FRM] << FSR_RD_SHIFT;
    case CSR_TIME:
    case CSR_INSTRET:
    case CSR_CYCLE:
        if ((env->csr[CSR_MUCOUNTEREN] >> (csrno2 & (63))) & 1) {
            return csr_read_helper(env, csrno2 + (CSR_MCYCLE - CSR_CYCLE));
        }
        break;
    case CSR_STIME:
    case CSR_SINSTRET:
    case CSR_SCYCLE:
        if ((env->csr[CSR_MSCOUNTEREN] >> (csrno2 & (63))) & 1) {
            return csr_read_helper(env, csrno2 + (CSR_MCYCLE - CSR_SCYCLE));
        }
        break;
    case CSR_MUCOUNTEREN:
        return env->csr[CSR_MUCOUNTEREN];
    case CSR_MSCOUNTEREN:
        return env->csr[CSR_MSCOUNTEREN];
    case CSR_MUCYCLE_DELTA:
        return 0; /* as spike does */
    case CSR_MUTIME_DELTA:
        return 0; /* as spike does */
    case CSR_MUINSTRET_DELTA:
        return 0; /* as spike does */
    case CSR_MSCYCLE_DELTA:
        return 0; /* as spike does */
    case CSR_MSTIME_DELTA:
        return 0; /* as spike does */
    case CSR_MSINSTRET_DELTA:
        return 0; /* as spike does */
    case CSR_MUCYCLE_DELTAH:
        printf("CSR 0x%x unsupported on RV64\n", csrno2);
        exit(1);
    case CSR_MUTIME_DELTAH:
        printf("CSR 0x%x unsupported on RV64\n", csrno2);
        exit(1);
    case CSR_MUINSTRET_DELTAH:
        printf("CSR 0x%x unsupported on RV64\n", csrno2);
        exit(1);
    case CSR_MSCYCLE_DELTAH:
        printf("CSR 0x%x unsupported on RV64\n", csrno2);
        exit(1);
    case CSR_MSTIME_DELTAH:
        printf("CSR 0x%x unsupported on RV64\n", csrno2);
        exit(1);
    case CSR_MSINSTRET_DELTAH:
        printf("CSR 0x%x unsupported on RV64\n", csrno2);
        exit(1);
    /* notice the lack of CSR_MTIME - this is handled by throwing an exception
       and letting the handler read from the RTC */
    case CSR_MCYCLE:
        return cpu_riscv_read_instret(env);
        break;
    case CSR_MINSTRET:
        return cpu_riscv_read_instret(env);
        break;
    case CSR_MCYCLEH:
        printf("CSR 0x%x unsupported on RV64\n", csrno2);
        exit(1);
    case CSR_MINSTRETH:
        printf("CSR 0x%x unsupported on RV64\n", csrno2);
        exit(1);
    case CSR_SSTATUS: {
        target_ulong mask = SSTATUS_SIE | SSTATUS_SPIE | SSTATUS_SPP
                            | SSTATUS_FS | SSTATUS_XS | SSTATUS_PUM;
        target_ulong sstatus = env->csr[CSR_MSTATUS] & mask;
        if ((sstatus & SSTATUS_FS) == SSTATUS_FS ||
                (sstatus & SSTATUS_XS) == SSTATUS_XS) {
            sstatus |= SSTATUS64_SD;
        }
        return sstatus;
    }
    case CSR_SIP:
        return env->csr[CSR_MIP] & env->csr[CSR_MIDELEG];
    case CSR_SIE:
        return env->csr[CSR_MIE] & env->csr[CSR_MIDELEG];
    case CSR_SEPC:
        return env->csr[CSR_SEPC];
    case CSR_SBADADDR:
        return env->csr[CSR_SBADADDR];
    case CSR_STVEC:
        return env->csr[CSR_STVEC];
    case CSR_SCAUSE:
        return env->csr[CSR_SCAUSE];
    case CSR_SPTBR:
        return env->csr[CSR_SPTBR];
    case CSR_SSCRATCH:
        return env->csr[CSR_SSCRATCH];
    case CSR_MSTATUS:
        return env->csr[CSR_MSTATUS];
    case CSR_MIP:
        return env->csr[CSR_MIP];
    case CSR_MIE:
        return env->csr[CSR_MIE];
    case CSR_MEPC:
        return env->csr[CSR_MEPC];
    case CSR_MSCRATCH:
        return env->csr[CSR_MSCRATCH];
    case CSR_MCAUSE:
        return env->csr[CSR_MCAUSE];
    case CSR_MBADADDR:
        return env->csr[CSR_MBADADDR];
    case CSR_MISA:
        return env->csr[CSR_MISA];
    case CSR_MARCHID:
        return 0; /* as spike does */
    case CSR_MIMPID:
        return 0; /* as spike does */
    case CSR_MVENDORID:
        return 0; /* as spike does */
    case CSR_MHARTID:
        return 0;
    case CSR_MTVEC:
        return env->csr[CSR_MTVEC];
    case CSR_MEDELEG:
        return env->csr[CSR_MEDELEG];
    case CSR_MIDELEG:
        return env->csr[CSR_MIDELEG];
    case CSR_TDRSELECT:
        return 0; /* as spike does */
    case CSR_DCSR:
        printf("DEBUG NOT IMPLEMENTED\n");
        exit(1);
    case CSR_DPC:
        printf("DEBUG NOT IMPLEMENTED\n");
        exit(1);
    case CSR_DSCRATCH:
        printf("DEBUG NOT IMPLEMENTED\n");
        exit(1);
    }
    /* used by e.g. MTIME read */
    helper_raise_exception(env, RISCV_EXCP_ILLEGAL_INST);
    return 0;
}

/*
 * Check that CSR access is allowed.
 *
 * Adapted from Spike's decode.h:validate_csr
 */
void validate_csr(CPURISCVState *env, uint64_t which, uint64_t write,
        uint64_t new_pc) {
    unsigned csr_priv = get_field((which), 0x300);
    unsigned csr_read_only = get_field((which), 0xC00) == 3;
    if (((write) && csr_read_only) || (env->priv < csr_priv)) {
        do_raise_exception_err(env, RISCV_EXCP_ILLEGAL_INST, new_pc);
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

target_ulong helper_sret(CPURISCVState *env, target_ulong cpu_pc_deb)
{
    if (!(env->priv >= PRV_S)) {
        helper_raise_exception(env, RISCV_EXCP_ILLEGAL_INST);
    }

    target_ulong retpc = env->csr[CSR_SEPC];
    if (retpc & 0x3) {
        helper_raise_exception(env, RISCV_EXCP_INST_ADDR_MIS);
    }

    target_ulong mstatus = env->csr[CSR_MSTATUS];
    target_ulong prev_priv = get_field(mstatus, MSTATUS_SPP);
    mstatus = set_field(mstatus, MSTATUS_UIE << prev_priv,
                        get_field(mstatus, MSTATUS_SPIE));
    mstatus = set_field(mstatus, MSTATUS_SPIE, 0);
    mstatus = set_field(mstatus, MSTATUS_SPP, PRV_U);
    set_privilege(env, prev_priv);
    csr_write_helper(env, mstatus, CSR_MSTATUS);

    return retpc;
}

target_ulong helper_mret(CPURISCVState *env, target_ulong cpu_pc_deb)
{
    if (!(env->priv >= PRV_M)) {
        helper_raise_exception(env, RISCV_EXCP_ILLEGAL_INST);
    }

    target_ulong retpc = env->csr[CSR_MEPC];
    if (retpc & 0x3) {
        helper_raise_exception(env, RISCV_EXCP_INST_ADDR_MIS);
    }

    target_ulong mstatus = env->csr[CSR_MSTATUS];
    target_ulong prev_priv = get_field(mstatus, MSTATUS_MPP);
    mstatus = set_field(mstatus, MSTATUS_UIE << prev_priv,
                        get_field(mstatus, MSTATUS_MPIE));
    mstatus = set_field(mstatus, MSTATUS_MPIE, 0);
    mstatus = set_field(mstatus, MSTATUS_MPP, PRV_U);
    set_privilege(env, prev_priv);
    csr_write_helper(env, mstatus, CSR_MSTATUS);

    return retpc;
}

#ifndef CONFIG_USER_ONLY

void helper_fence_i(CPURISCVState *env)
{
    RISCVCPU *cpu = riscv_env_get_cpu(env);
    CPUState *cs = CPU(cpu);
    /* Flush QEMU's TLB */
    tlb_flush(cs, 1);
    /* ARM port seems to not know if this is okay inside a TB
       But we need to do it */
    tb_flush(cs);
}

void helper_tlb_flush(CPURISCVState *env)
{
    RISCVCPU *cpu = riscv_env_get_cpu(env);
    tlb_flush(CPU(cpu), 1);
}

void riscv_cpu_do_unaligned_access(CPUState *cs, vaddr addr,
                                   MMUAccessType access_type, int mmu_idx,
                                   uintptr_t retaddr)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;
    if (access_type == MMU_INST_FETCH) {
        fprintf(stderr, "unaligned inst fetch not handled here. should not "
                "trigger\n");
        exit(1);
    } else if (access_type == MMU_DATA_STORE) {
        cs->exception_index = RISCV_EXCP_STORE_AMO_ADDR_MIS;
        env->badaddr = addr;
    } else if (access_type == MMU_DATA_LOAD) {
        cs->exception_index = RISCV_EXCP_LOAD_ADDR_MIS;
        env->badaddr = addr;
    } else {
        fprintf(stderr, "Invalid MMUAccessType\n");
        exit(1);
    }
    do_raise_exception_err(env, cs->exception_index, retaddr);
}

/* called by qemu's softmmu to fill the qemu tlb */
void tlb_fill(CPUState *cs, target_ulong addr, MMUAccessType access_type,
        int mmu_idx, uintptr_t retaddr)
{
    int ret;
    ret = riscv_cpu_handle_mmu_fault(cs, addr, access_type, mmu_idx);
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
