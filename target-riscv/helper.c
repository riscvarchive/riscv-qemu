/*
 *  RISC-V emulation helpers for qemu.
 *
 *  Author: Sagar Karandikar, sagark@eecs.berkeley.edu
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
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <signal.h>
#include "cpu.h"
#include "hw/riscv/riscv_pmp.h"

/*#define RISCV_DEBUG_INTERRUPT */

bool riscv_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
#if !defined(CONFIG_USER_ONLY)
    if (interrupt_request & CPU_INTERRUPT_HARD) {
        RISCVCPU *cpu = RISCV_CPU(cs);
        CPURISCVState *env = &cpu->env;
        int interruptno = cpu_riscv_hw_interrupts_pending(env);
        if (interruptno + 1) {
            cs->exception_index = 0x70000000U | interruptno;
            riscv_cpu_do_interrupt(cs);
            return true;
        }
    }
#endif
    return false;
}

#if !defined(CONFIG_USER_ONLY)

/* get_physical_address - get the physical address for this virtual address
 *
 * Do a page table walk to obtain the physical address corresponding to a
 * virtual address. Returns 0 if the translation was successful
 *
 * Adapted from Spike's mmu_t::translate and mmu_t::walk
 *
 */
static int get_physical_address(CPURISCVState *env, hwaddr *physical,
                                int *prot, target_ulong address,
                                int access_type, int mmu_idx)
{
    /* NOTE: the env->pc value visible here will not be
     * correct, but the value visible to the exception handler
     * (riscv_cpu_do_interrupt) is correct */

    *prot = 0;
    CPUState *cs = CPU(riscv_env_get_cpu(env));

    target_ulong mode = env->priv;
    if (access_type != MMU_INST_FETCH) {
        if (get_field(env->mstatus, MSTATUS_MPRV)) {
            mode = get_field(env->mstatus, MSTATUS_MPP);
        }
    }

    if (get_field(env->satp, SATP64_MODE) == VM_MBARE) {
        mode = PRV_M;
    }

    /* check to make sure that mmu_idx and mode that we get matches */
    if (unlikely(mode != mmu_idx)) {
        fprintf(stderr, "MODE: mmu_idx mismatch\n");
        exit(1);
    }

    if (mode == PRV_M) {
        target_ulong msb_mask = (((target_ulong)2) << (TARGET_LONG_BITS - 1)) - 1;
                                        /*0x7FFFFFFFFFFFFFFF; */
        *physical = address & msb_mask;
        *prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
        return TRANSLATE_SUCCESS;
    }

    target_ulong addr = address;
    int supervisor = mode == PRV_S;
    int nsum = get_field(env->mstatus, MSTATUS_SUM);
    int mxr = get_field(env->mstatus, MSTATUS_MXR);

    mxr = 1; // TODO: EMDALO: HACKITY HACK TAKE THIS OUT
    // PROBABLY GO TO SSTATUS CSR Write handling routine as per 1.10
    // spec and ensure that changes to SSTATUS MXR bit are reflected
    // in env->mstatus.

    int levels, ptidxbits, ptesize;
    int vm = get_field(env->satp, SATP64_MODE);

    // TODO: EMDALO HACKY - REMOVE
    int sum = 0;

    if(nsum == 0) {
       sum = 1;
    } else {
       sum = 0;
    }
    // END HACKITY HACK

    switch (vm) {
    case VM_SV39:
      levels = 3;
      ptidxbits = 9;
      ptesize = 8;
      break;
    case VM_SV48:
      levels = 4;
      ptidxbits = 9;
      ptesize = 8;
      break;
    default:
      printf("unsupported SATP_MODE value\n");
      exit(1);
    }

    int va_bits = PGSHIFT + levels * ptidxbits;
    target_ulong mask = (1L << (TARGET_LONG_BITS - (va_bits - 1))) - 1;
    target_ulong masked_msbs = (addr >> (va_bits - 1)) & mask;
    if (masked_msbs != 0 && masked_msbs != mask) {
        return TRANSLATE_FAIL;
    }

    target_ulong base = (env->satp & 0x0FFFFFFFFFFFFFFF) << PGSHIFT;
    int ptshift = (levels - 1) * ptidxbits;
    int i;
    for (i = 0; i < levels; i++, ptshift -= ptidxbits) {
        target_ulong idx = (addr >> (PGSHIFT + ptshift)) &
                           ((1 << ptidxbits) - 1);

        /* check that physical address of PTE is legal */
        target_ulong pte_addr = base + idx * ptesize;

        /* PTE must reside in memory */
        if (!(pte_addr >= DRAM_BASE && pte_addr < (DRAM_BASE + env->memsize))) {
            printf("PTE was not in DRAM region\n");
            exit(1);
            break;
        }

        target_ulong pte = ldq_phys(cs->as, pte_addr);
        target_ulong ppn = pte >> PTE_PPN_SHIFT;

        if (PTE_TABLE(pte)) { /* next level of page table */
            base = ppn << PGSHIFT;
        } else if ((pte & PTE_U) ? supervisor && sum : !supervisor) {
            break;
        } else if (!(pte & PTE_V) || (!(pte & PTE_R) && (pte & PTE_W))) {
            break;
        } else if (access_type == MMU_INST_FETCH ? !(pte & PTE_X) :
                  access_type == MMU_DATA_LOAD ?  !(pte & PTE_R) &&
                  !(mxr && (pte & PTE_X)) : !((pte & PTE_R) && (pte & PTE_W))) {
            break;
        } else {
            /* set accessed and possibly dirty bits.
               we only put it in the TLB if it has the right stuff */
            stq_phys(cs->as, pte_addr, ldq_phys(cs->as, pte_addr) | PTE_A |
                    ((access_type == MMU_DATA_STORE) * PTE_D));

            /* for superpage mappings, make a fake leaf PTE for the TLB's
               benefit. */
            target_ulong vpn = addr >> PGSHIFT;
            *physical = (ppn | (vpn & ((1L << ptshift) - 1))) << PGSHIFT;

            /* we do not give all prots indicated by the PTE
             * this is because future accesses need to do things like set the
             * dirty bit on the PTE
             *
             * at this point, we assume that protection checks have occurred */
            if (supervisor) {
                if ((pte & PTE_X) && access_type == MMU_INST_FETCH) {
                    *prot |= PAGE_EXEC;
                } else if ((pte & PTE_W) && access_type == MMU_DATA_STORE) {
                    *prot |= PAGE_WRITE;
                } else if ((pte & PTE_R) && access_type == MMU_DATA_LOAD) {
                    *prot |= PAGE_READ;
                } else {
                    printf("err in translation prots");
                    exit(1);
                }
            } else {
                if ((pte & PTE_X) && access_type == MMU_INST_FETCH) {
                    *prot |= PAGE_EXEC;
                } else if ((pte & PTE_W) && access_type == MMU_DATA_STORE) {
                    *prot |= PAGE_WRITE;
                } else if ((pte & PTE_R) && access_type == MMU_DATA_LOAD) {
                    *prot |= PAGE_READ;
                } else {
                    printf("err in translation prots");
                    exit(1);
                }
            }
            return TRANSLATE_SUCCESS;
        }
    }
    return TRANSLATE_FAIL;
}

static int handle_virt(CPURISCVState * env) {

    if (get_field(env->satp, SATP64_MODE) == VM_SV39) {
     return 1;
    }

  return 0;
}

static void raise_mmu_exception(CPURISCVState *env, target_ulong address,
                                MMUAccessType access_type)
{
    CPUState *cs = CPU(riscv_env_get_cpu(env));
    int exception = 0;
    if (access_type == MMU_INST_FETCH) { /* inst access */
        if(handle_virt(env)) {
           exception = RISCV_EXCP_INST_PAGE_FAULT;
        } else {
           exception = RISCV_EXCP_INST_ACCESS_FAULT;
        }
        env->badaddr = address;
    } else if (access_type == MMU_DATA_STORE) { /* store access */
        if(handle_virt(env)) {
           exception = RISCV_EXCP_STORE_PAGE_FAULT;
        } else {
          exception = RISCV_EXCP_STORE_AMO_ACCESS_FAULT;
        }
        env->badaddr = address;
    } else if (access_type == MMU_DATA_LOAD) { /* load access */
        if(handle_virt(env)) {
          exception = RISCV_EXCP_LOAD_PAGE_FAULT;
        } else {
          exception = RISCV_EXCP_LOAD_ACCESS_FAULT;
        }
        env->badaddr = address;
    } else {
        fprintf(stderr, "FAIL: invalid access_type\n");
        exit(1);
    }
    cs->exception_index = exception;
}

hwaddr riscv_cpu_get_phys_page_debug(CPUState *cs, vaddr addr)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    hwaddr phys_addr;
    int prot;
    int mem_idx = cpu_mmu_index(&cpu->env, false);

    if (get_physical_address(&cpu->env, &phys_addr, &prot, addr, 0, mem_idx)) {
        return -1;
    }
    return phys_addr;
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
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;

    ret = riscv_cpu_handle_mmu_fault(cs, addr, access_type, mmu_idx);
    if (ret == TRANSLATE_FAIL) {
        do_raise_exception_err(env, cs->exception_index, retaddr);
    }
}

void riscv_cpu_unassigned_access(CPUState *cs, hwaddr addr, bool is_write,
        bool is_exec, int unused, unsigned size)
{
    printf("unassigned address not implemented for riscv\n");
    printf("are you trying to fetch instructions from an MMIO page?\n");
    printf("unassigned Address: %016" PRIx64 "\n", addr);
    exit(1);
}

#endif

int riscv_cpu_handle_mmu_fault(CPUState *cs, vaddr address,
        int access_type, int mmu_idx)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;
#if !defined(CONFIG_USER_ONLY)
    hwaddr physical = 0;
    int prot;
#endif
    int ret = TRANSLATE_FAIL;

    qemu_log_mask(CPU_LOG_MMU,
            "%s pc " TARGET_FMT_lx " ad %" VADDR_PRIx " access_type %d mmu_idx \
             %d\n", __func__, env->pc, address, access_type, mmu_idx);

#if !defined(CONFIG_USER_ONLY)
    ret = get_physical_address(env, &physical, &prot, address, access_type,
                               mmu_idx);
    qemu_log_mask(CPU_LOG_MMU,
            "%s address=%" VADDR_PRIx " ret %d physical " TARGET_FMT_plx
             " prot %d\n",
             __func__, address, ret, physical, prot);

    if(false == pmp_hart_has_privs(env, physical, TARGET_PAGE_SIZE, 1<<access_type)) {
        ret = TRANSLATE_FAIL;
    }


    if (ret == TRANSLATE_SUCCESS) {
        tlb_set_page(cs, address & TARGET_PAGE_MASK,
                     physical & TARGET_PAGE_MASK,
                     prot, mmu_idx, TARGET_PAGE_SIZE);
    } else if (ret == TRANSLATE_FAIL) {
        raise_mmu_exception(env, address, access_type);
    }
#else
    cs->exception_index = QEMU_USER_EXCP_FAULT;
#endif
    return ret;
}

#ifdef RISCV_DEBUG_INTERRUPT
static const char * const riscv_excp_names[16] = {
    "misaligned fetch",
    "fault fetch",
    "illegal instruction",
    "Breakpoint",
    "misaligned load",
    "fault load",
    "misaligned store",
    "fault store",
    "user_ecall",
    "supervisor_ecall",
    "hypervisor_ecall",
    "machine_ecall",
    "ins page fault",
    "load page fault",
    "reserved",
    "store page fault"
};

static const char * const riscv_interrupt_names[64] = {
    "Reserved 0",
    "S Soft Interrupt",
    "H Soft Interrupt",
    "M Soft Interrupt",
    "Reserved 4",
    "S Timer Interrupt",
    "H Timer Interrupt",
    "M Timer Interrupt",
    "Reserved 8",
    "S Ext Interrupt",
    "H Ext Interrupt",
    "M Ext Interrupt",
    "Reserved 12",
    "Reserved 13",
    "Reserved 14",
    "Reserved 15",
    "Local 0",
    "Local 1",
    "Local 2",
    "Local 3",
    "Local 4",
    "Local 5",
    "Local 6",
    "Local 7",
    "Local 8",
    "Local 9",
    "Local 10",
    "Local 11",
    "Local 12",
    "Local 13",
    "Local 14",
    "Local 15",
    "Local 16",
    "Local 17",
    "Local 18",
    "Local 19",
    "Local 20",
    "Local 21",
    "Local 22",
    "Local 23",
    "Local 24",
    "Local 25",
    "Local 26",
    "Local 27",
    "Local 28",
    "Local 29",
    "Local 30",
    "Local 31",
    "Local 32",
    "Local 33",
    "Local 34",
    "Local 35",
    "Local 36",
    "Local 37",
    "Local 38",
    "Local 39",
    "Local 40",
    "Local 41",
    "Local 42",
    "Local 43",
    "Local 44",
    "Local 45",
    "Local 46",
    "Local 47",
};
#endif     /* RISCV_DEBUG_INTERRUPT */

/*
 * Handle Traps
 *
 * Adapted from Spike's processor_t::take_trap.
 *
 */
void riscv_cpu_do_interrupt(CPUState *cs)
{
#if !defined(CONFIG_USER_ONLY)

    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;

    #ifdef RISCV_DEBUG_INTERRUPT
    if (cs->exception_index & 0x70000000) {
        fprintf(stderr, "core: %d : interrupt trap_%s, epc 0x" TARGET_FMT_lx "\n"
               , env->hart_index, riscv_interrupt_names[cs->exception_index & 0x0fffffff],
               env->pc);
    } else {
        fprintf(stderr, "core: %d: exception trap_%s, epc 0x" TARGET_FMT_lx "\n"
                , env->hart_index, riscv_excp_names[cs->exception_index], env->pc);
    }
    #endif

    if (cs->exception_index == RISCV_EXCP_BREAKPOINT) {
        fprintf(stderr, "debug mode not implemented\n");
    }

    /* skip dcsr cause check */

    target_ulong fixed_cause = 0;
    if (cs->exception_index & (0x70000000)) {
	// EMDALO: TODO: THIS SUCKS
        /* hacky for now. the MSB (bit 63) indicates interrupt but cs->exception
           index is only 32 bits wide */
        fixed_cause = cs->exception_index & 0x0FFFFFFF;
        fixed_cause |= ((target_ulong)1) << (TARGET_LONG_BITS - 1);
    } else {
        /* fixup User ECALL -> correct priv ECALL */
        if (cs->exception_index == RISCV_EXCP_U_ECALL) {
            switch (env->priv) {
            case PRV_U:
                fixed_cause = RISCV_EXCP_U_ECALL;
                break;
            case PRV_S:
                fixed_cause = RISCV_EXCP_S_ECALL;
                break;
            case PRV_H:
                fixed_cause = RISCV_EXCP_H_ECALL;
                break;
            case PRV_M:
                fixed_cause = RISCV_EXCP_M_ECALL;
                break;
            }
        } else {
            fixed_cause = cs->exception_index;
        }
    }

    target_ulong backup_epc = env->pc;

    target_ulong bit = fixed_cause;
    target_ulong deleg = env->medeleg;

    int hasbadaddr =
        (fixed_cause == RISCV_EXCP_INST_ADDR_MIS) ||
        (fixed_cause == RISCV_EXCP_INST_ACCESS_FAULT) ||
        (fixed_cause == RISCV_EXCP_LOAD_ADDR_MIS) ||
        (fixed_cause == RISCV_EXCP_STORE_AMO_ADDR_MIS) ||
        (fixed_cause == RISCV_EXCP_LOAD_ACCESS_FAULT) ||
        (fixed_cause == RISCV_EXCP_STORE_AMO_ACCESS_FAULT) ||
        (fixed_cause == RISCV_EXCP_INST_PAGE_FAULT) ||
        (fixed_cause == RISCV_EXCP_LOAD_PAGE_FAULT) || 
        (fixed_cause == RISCV_EXCP_STORE_PAGE_FAULT);

    if (bit & ((target_ulong)1 << (TARGET_LONG_BITS - 1))) {
        deleg = env->mideleg, bit &= ~((target_ulong)1 << (TARGET_LONG_BITS - 1));
    }

    if (env->priv <= PRV_S && bit < 64 && ((deleg >> bit) & 1)) {
        /* handle the trap in S-mode */
        /* No need to check STVEC for misaligned - lower 2 bits cannot be set */
        env->pc = env->stvec;
        env->scause = fixed_cause;
        env->sepc = backup_epc;

        if (hasbadaddr) {
            #ifdef RISCV_DEBUG_INTERRUPT
            fprintf(stderr, "core %d: badaddr 0x" TARGET_FMT_lx "\n",
                    env->hart_index, env->badaddr);
            #endif
            env->sbadaddr = env->badaddr;
        }

        target_ulong s = env->mstatus;
        s = set_field(s, MSTATUS_SPIE, get_field(s, MSTATUS_UIE << env->priv));
        s = set_field(s, MSTATUS_SPP, env->priv);
        s = set_field(s, MSTATUS_SIE, 0);
        csr_write_helper(env, s, CSR_MSTATUS);
        set_privilege(env, PRV_S);
    } else {
        /* No need to check MTVEC for misaligned - lower 2 bits cannot be set */
        env->pc = env->mtvec;
        env->mepc = backup_epc;
        env->mcause = fixed_cause;

        if (hasbadaddr) {
            #ifdef RISCV_DEBUG_INTERRUPT
            fprintf(stderr, "core %d: badaddr 0x" TARGET_FMT_lx "\n",
                    env->hart_index, env->badaddr);
            #endif
            env->mbadaddr = env->badaddr;
        }

        target_ulong s = env->mstatus;
        s = set_field(s, MSTATUS_MPIE, get_field(s, MSTATUS_UIE << env->priv));
        s = set_field(s, MSTATUS_MPP, env->priv);
        s = set_field(s, MSTATUS_MIE, 0);
        csr_write_helper(env, s, CSR_MSTATUS);
        set_privilege(env, PRV_M);
    }
    /* TODO yield load reservation  */
#endif
    cs->exception_index = EXCP_NONE; /* mark handled to qemu */
}
