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
#include "qemu/timer.h"
#include "exec/helper-proto.h"

#ifndef CONFIG_USER_ONLY
static int validate_vm(target_ulong vm)
{
    return vm == VM_SV32 || vm == VM_SV39 || vm == VM_SV48 || vm == VM_MBARE;
}
#endif

/* Exceptions processing helpers */
inline void QEMU_NORETURN do_raise_exception_err(CPURISCVState *env,
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

#ifndef CONFIG_USER_ONLY
    uint64_t delegable_ints = MIP_SSIP | MIP_STIP | MIP_SEIP | (1 << IRQ_COP);
    uint64_t all_ints = delegable_ints | MIP_MSIP | MIP_MTIP;
#endif

    switch (csrno) {
    case CSR_FFLAGS:
#ifndef CONFIG_USER_ONLY
        env->mstatus |= MSTATUS_FS | MSTATUS64_SD;
#endif
        env->fflags = val_to_write & (FSR_AEXC >> FSR_AEXC_SHIFT);
        break;
    case CSR_FRM:
#ifndef CONFIG_USER_ONLY
        env->mstatus |= MSTATUS_FS | MSTATUS64_SD;
#endif
        env->frm = val_to_write & (FSR_RD >> FSR_RD_SHIFT);
        break;
    case CSR_FCSR:
#ifndef CONFIG_USER_ONLY
        env->mstatus |= MSTATUS_FS | MSTATUS64_SD;
#endif
        env->fflags = (val_to_write & FSR_AEXC) >> FSR_AEXC_SHIFT;
        env->frm = (val_to_write & FSR_RD) >> FSR_RD_SHIFT;
        break;
#ifndef CONFIG_USER_ONLY
    case CSR_MSTATUS: {
        target_ulong mstatus = env->mstatus;
        if ((val_to_write ^ mstatus) &
            (MSTATUS_VM | MSTATUS_MPP | MSTATUS_MPRV | MSTATUS_PUM |
             MSTATUS_MXR)) {
            helper_tlb_flush(env);
        }

        /* no extension support */
        target_ulong mask = MSTATUS_SIE | MSTATUS_SPIE | MSTATUS_MIE
            | MSTATUS_MPIE | MSTATUS_SPP | MSTATUS_FS | MSTATUS_MPRV
            | MSTATUS_PUM | MSTATUS_MPP | MSTATUS_MXR;

        if (validate_vm(get_field(val_to_write, MSTATUS_VM))) {
            mask |= MSTATUS_VM;
        }

        mstatus = (mstatus & ~mask) | (val_to_write & mask);

        int dirty = (mstatus & MSTATUS_FS) == MSTATUS_FS;
        dirty |= (mstatus & MSTATUS_XS) == MSTATUS_XS;
        mstatus = set_field(mstatus, MSTATUS64_SD, dirty);
        env->mstatus = mstatus;
        break;
    }
    case CSR_MIP: {
        target_ulong mask = MIP_SSIP | MIP_STIP;
        env->mip = (env->mip & ~mask) |
            (val_to_write & mask);
        if (env->mip & MIP_SSIP) {
            qemu_irq_raise(SSIP_IRQ);
        } else {
            qemu_irq_lower(SSIP_IRQ);
        }
        if (env->mip & MIP_STIP) {
            qemu_irq_raise(STIP_IRQ);
        } else {
            qemu_irq_lower(STIP_IRQ);
        }
        if (env->mip & MIP_MSIP) {
            qemu_irq_raise(MSIP_IRQ);
        } else {
            qemu_irq_lower(MSIP_IRQ);
        }
        break;
    }
    case CSR_MIE: {
        env->mie = (env->mie & ~all_ints) |
            (val_to_write & all_ints);
        break;
    }
    case CSR_MIDELEG:
        env->mideleg = (env->mideleg & ~delegable_ints)
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
        env->medeleg = (env->medeleg & ~mask)
                                | (val_to_write & mask);
        break;
    }
    case CSR_MINSTRET:
    case CSR_MCYCLE:
#if defined(TARGET_RISCV32)
        printf("minstret write todo\n");
        exit(1);
#else
        printf("minstret write todo\n");
        exit(1);
#endif
        break;
    case CSR_MINSTRETH:
    case CSR_MCYCLEH:
        printf("minstret write todo\n");
        exit(1);
        break;
    case CSR_MUCOUNTEREN:
        env->mucounteren = val_to_write;
        break;
    case CSR_MSCOUNTEREN:
        env->mscounteren = val_to_write;
        break;
    case CSR_SSTATUS: {
        target_ulong ms = env->mstatus;
        target_ulong mask = SSTATUS_SIE | SSTATUS_SPIE | SSTATUS_SPP
                            | SSTATUS_FS | SSTATUS_XS | SSTATUS_PUM;
        ms = (ms & ~mask) | (val_to_write & mask);
        csr_write_helper(env, ms, CSR_MSTATUS);
        break;
    }
    case CSR_SIP: {
        target_ulong next_mip = (env->mip & ~env->mideleg)
                                | (val_to_write & env->mideleg);
        csr_write_helper(env, next_mip, CSR_MIP);
        /* note: stw_phys should be done by the call to set MIP if necessary, */
        /* so we don't do it here */
        break;
    }
    case CSR_SIE: {
        target_ulong next_mie = (env->mie & ~env->mideleg)
                                | (val_to_write & env->mideleg);
        csr_write_helper(env, next_mie, CSR_MIE);
        break;
    }
    case CSR_SPTBR: {
        env->sptbr = val_to_write & (((target_ulong)1 <<
                              (TARGET_PHYS_ADDR_SPACE_BITS - PGSHIFT)) - 1);
        break;
    }
    case CSR_SEPC:
        env->sepc = val_to_write;
        break;
    case CSR_STVEC:
        env->stvec = val_to_write >> 2 << 2;
        break;
    case CSR_SSCRATCH:
        env->sscratch = val_to_write;
        break;
    case CSR_SCAUSE:
        env->scause = val_to_write;
        break;
    case CSR_SBADADDR:
        env->sbadaddr = val_to_write;
        break;
    case CSR_MEPC:
        env->mepc = val_to_write;
        break;
    case CSR_MTVEC:
        env->mtvec = val_to_write >> 2 << 2;
        break;
    case CSR_MSCRATCH:
        env->mscratch = val_to_write;
        break;
    case CSR_MCAUSE:
        env->mcause = val_to_write;
        break;
    case CSR_MBADADDR:
        env->mbadaddr = val_to_write;
        break;
    case CSR_MISA: {
        if (!(val_to_write & (1L << ('F' - 'A')))) {
            val_to_write &= ~(1L << ('D' - 'A'));
        }

        // allow MAFDC bits in MISA to be modified
        target_ulong mask = 0;
        mask |= 1L << ('M' - 'A');
        mask |= 1L << ('A' - 'A');
        mask |= 1L << ('F' - 'A');
        mask |= 1L << ('D' - 'A');
        mask |= 1L << ('C' - 'A');
        mask &= env->max_isa;

        env->misa = (val_to_write & mask) | (env->misa & ~mask);
        break;
    }
    case CSR_TSELECT:
        // TSELECT is hardwired in this implementation
        break;
    case CSR_TDATA1:
        printf("CSR_TDATA1 write not implemented.\n");
        exit(1);
        break;
    case CSR_TDATA2:
        printf("CSR_TDATA2 write not implemented.\n");
        exit(1);
        break;
    case CSR_DCSR:
        printf("CSR_DCSR write not implemented.\n");
        exit(1);
        break;
#endif
    default:
        helper_raise_exception(env, RISCV_EXCP_ILLEGAL_INST);
    }
}

/*
 * Handle reads to CSRs and any resulting special behavior
 *
 * Adapted from Spike's processor_t::get_csr
 */
inline target_ulong csr_read_helper(CPURISCVState *env, target_ulong csrno)
{
    #ifdef RISCV_DEBUG_PRINT
    fprintf(stderr, "READ CSR 0x%x\n", csrno);
    #endif
#ifndef CONFIG_USER_ONLY
    target_ulong ctr_en = env->priv == PRV_U ? env->mucounteren :
                   env->priv == PRV_S ? env->mscounteren : -1U;
#else
    target_ulong ctr_en = env->mucounteren;
#endif
    target_ulong ctr_ok = (ctr_en >> (csrno & 31)) & 1;

    if (ctr_ok) {
        if (csrno >= CSR_HPMCOUNTER3 && csrno <= CSR_HPMCOUNTER31) {
            return 0;
        }
#if defined(TARGET_RISCV32)
        if (csrno >= CSR_HPMCOUNTER3H && csrno <= CSR_HPMCOUNTER31H) {
            return 0;
        }
#endif
    }
    if (csrno >= CSR_MHPMCOUNTER3 && csrno <= CSR_MHPMCOUNTER31) {
        return 0;
    }
#if defined(TARGET_RISCV32)
    if (csrno >= CSR_MHPMCOUNTER3 && csrno <= CSR_MHPMCOUNTER31) {
        return 0;
    }
#endif
    if (csrno >= CSR_MHPMEVENT3 && csrno <= CSR_MHPMEVENT31) {
        return 0;
    }

    switch (csrno) {
    case CSR_FFLAGS:
        return env->fflags;
    case CSR_FRM:
        return env->frm;
    case CSR_FCSR:
        return env->fflags << FSR_AEXC_SHIFT |
               env->frm << FSR_RD_SHIFT;
#ifdef CONFIG_USER_ONLY
    case CSR_TIME:
    case CSR_CYCLE:
    case CSR_INSTRET:
        return (target_ulong)cpu_get_host_ticks();
    case CSR_TIMEH:
    case CSR_CYCLEH:
    case CSR_INSTRETH:
#if defined(TARGET_RISCV32)
        return (target_ulong)(cpu_get_host_ticks() >> 32);
#endif
        break;
#endif
#ifndef CONFIG_USER_ONLY
        /* TODO fix TIME, INSTRET, CYCLE in user mode */
        /* 32-bit TIMEH, CYCLEH, INSTRETH, other H stuff */
    case CSR_INSTRET:
    case CSR_CYCLE:
        if (ctr_ok) {
            return cpu_riscv_read_instret(env);
        }
        break;
    case CSR_MINSTRET:
    case CSR_MCYCLE:
        return cpu_riscv_read_instret(env);
    case CSR_MINSTRETH:
    case CSR_MCYCLEH:
#if defined(TARGET_RISCV32)
        return cpu_riscv_read_instret(env) >> 32;
#endif
        break;
    case CSR_MUCOUNTEREN:
        return env->mucounteren;
    case CSR_MSCOUNTEREN:
        return env->mscounteren;
    case CSR_SSTATUS: {
        target_ulong mask = SSTATUS_SIE | SSTATUS_SPIE | SSTATUS_SPP
                            | SSTATUS_FS | SSTATUS_XS | SSTATUS_PUM;
        target_ulong sstatus = env->mstatus & mask;
        if ((sstatus & SSTATUS_FS) == SSTATUS_FS ||
                (sstatus & SSTATUS_XS) == SSTATUS_XS) {
            sstatus |= SSTATUS64_SD;
        }
        return sstatus;
    }
    case CSR_SIP:
        return env->mip & env->mideleg;
    case CSR_SIE:
        return env->mie & env->mideleg;
    case CSR_SEPC:
        return env->sepc;
    case CSR_SBADADDR:
        return env->sbadaddr;
    case CSR_STVEC:
        return env->stvec;
    case CSR_SCAUSE:
        return env->scause;
    case CSR_SPTBR:
        return env->sptbr;
    case CSR_SSCRATCH:
        return env->sscratch;
    case CSR_MSTATUS:
        return env->mstatus;
    case CSR_MIP:
        return env->mip;
    case CSR_MIE:
        return env->mie;
    case CSR_MEPC:
        return env->mepc;
    case CSR_MSCRATCH:
        return env->mscratch;
    case CSR_MCAUSE:
        return env->mcause;
    case CSR_MBADADDR:
        return env->mbadaddr;
    case CSR_MISA:
        return env->misa;
    case CSR_MARCHID:
        return 0; /* as spike does */
    case CSR_MIMPID:
        return 0; /* as spike does */
    case CSR_MVENDORID:
        return 0; /* as spike does */
    case CSR_MHARTID:
        return 0;
    case CSR_MTVEC:
        return env->mtvec;
    case CSR_MEDELEG:
        return env->medeleg;
    case CSR_MIDELEG:
        return env->mideleg;
    case CSR_TSELECT:
        // indicate only usable in debug mode (which we don't have)
        // i.e. software can't use it
        // see: https://dev.sifive.com/documentation/risc-v-external-debug-support-0-11/
        return (1L << (TARGET_LONG_BITS - 5));
    case CSR_TDATA1:
        printf("CSR_TDATA1 read not implemented.\n");
        exit(1);
        break;
    case CSR_TDATA2:
        printf("CSR_TDATA2 read not implemented.\n");
        exit(1);
        break;
    case CSR_TDATA3:
        printf("CSR_TDATA3 read not implemented.\n");
        exit(1);
        break;
    case CSR_DCSR:
        printf("CSR_DCSR read not implemented.\n");
        exit(1);
        break;
#endif
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
void validate_csr(CPURISCVState *env, uint64_t which, uint64_t write)
{
#ifndef CONFIG_USER_ONLY
    unsigned csr_priv = get_field((which), 0x300);
    unsigned csr_read_only = get_field((which), 0xC00) == 3;
    if (((write) && csr_read_only) || (env->priv < csr_priv)) {
        do_raise_exception_err(env, RISCV_EXCP_ILLEGAL_INST, env->pc);
    }
#endif
}

target_ulong helper_csrrw(CPURISCVState *env, target_ulong src,
        target_ulong csr)
{
    validate_csr(env, csr, 1);
    uint64_t csr_backup = csr_read_helper(env, csr);
    csr_write_helper(env, src, csr);
    return csr_backup;
}

target_ulong helper_csrrs(CPURISCVState *env, target_ulong src,
        target_ulong csr, target_ulong rs1_pass)
{
    validate_csr(env, csr, rs1_pass != 0);
    uint64_t csr_backup = csr_read_helper(env, csr);
    if (rs1_pass != 0) {
        csr_write_helper(env, src | csr_backup, csr);
    }
    return csr_backup;
}

target_ulong helper_csrrc(CPURISCVState *env, target_ulong src,
        target_ulong csr, target_ulong rs1_pass)
{
    validate_csr(env, csr, rs1_pass != 0);
    uint64_t csr_backup = csr_read_helper(env, csr);
    if (rs1_pass != 0) {
        csr_write_helper(env, (~src) & csr_backup, csr);
    }
    return csr_backup;
}

#ifndef CONFIG_USER_ONLY

void set_privilege(CPURISCVState *env, target_ulong newpriv)
{
    if (!(newpriv <= PRV_M)) {
        printf("INVALID PRIV SET\n");
        exit(1);
    }
    if (newpriv == PRV_H) {
        newpriv = PRV_U;
    }
    helper_tlb_flush(env);
    env->priv = newpriv;
}

target_ulong helper_sret(CPURISCVState *env, target_ulong cpu_pc_deb)
{
    if (!(env->priv >= PRV_S)) {
        helper_raise_exception(env, RISCV_EXCP_ILLEGAL_INST);
    }

    target_ulong retpc = env->sepc;
    if (!riscv_feature(env, RISCV_FEATURE_RVC) && (retpc & 0x3)) {
        helper_raise_exception(env, RISCV_EXCP_INST_ADDR_MIS);
    }

    target_ulong mstatus = env->mstatus;
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

    target_ulong retpc = env->mepc;
    if (!riscv_feature(env, RISCV_FEATURE_RVC) && (retpc & 0x3)) {
        helper_raise_exception(env, RISCV_EXCP_INST_ADDR_MIS);
    }

    target_ulong mstatus = env->mstatus;
    target_ulong prev_priv = get_field(mstatus, MSTATUS_MPP);
    mstatus = set_field(mstatus, MSTATUS_UIE << prev_priv,
                        get_field(mstatus, MSTATUS_MPIE));
    mstatus = set_field(mstatus, MSTATUS_MPIE, 0);
    mstatus = set_field(mstatus, MSTATUS_MPP, PRV_U);
    set_privilege(env, prev_priv);
    csr_write_helper(env, mstatus, CSR_MSTATUS);

    return retpc;
}


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

#endif /* !CONFIG_USER_ONLY */
