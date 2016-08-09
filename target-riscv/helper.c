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
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <signal.h>
#include "cpu.h"

#define QEMU_IN_FETCH 0x2
#define QEMU_IN_WRITE 0x1
#define QEMU_IN_READ  0x0

//#define RISCV_DEBUG_INTERRUPT

#if !defined(CONFIG_USER_ONLY)

// TODO duplicated in op_helper.c
int validate_priv2(target_ulong priv);

int validate_priv2(target_ulong priv) {
    return priv == PRV_U || priv == PRV_S || priv == PRV_M;
}

// TODO duplicated in op_helper.c
void set_privilege2(CPURISCVState *env, target_ulong newpriv);

void set_privilege2(CPURISCVState *env, target_ulong newpriv) {
    if (!validate_priv2(newpriv)) {
        printf("INVALID PRIV SET\n");
        exit(1);
    }
    // copied body of helper_tlb_flush for now
    RISCVCPU *cpu = riscv_env_get_cpu(env);
    tlb_flush(CPU(cpu), 1);
    env->priv = newpriv;
}


bool riscv_cpu_exec_interrupt(CPUState *cs, int interrupt_request) {
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
    return false;
}

/* get_physical_address - get the physical address for this virtual address
 *
 * Do a page table walk to obtain the physical address corresponding to a
 * virtual address.
 *
 * Returns 0 if the translation was successful
*/
static int get_physical_address (CPURISCVState *env, hwaddr *physical,
                                int *prot, target_ulong address,
                                int rw, int mmu_idx)
{
    /* NOTE: the env->active_tc.PC value visible here will not be
     * correct, but the value visible to the exception handler
     * (riscv_cpu_do_interrupt) is correct */

    // rw is either QEMU_IN_FETCH, QEMU_IN_WRITE, or QEMU_IN_READ
    *prot = 0;
    CPUState *cs = CPU(riscv_env_get_cpu(env));

    target_ulong mode = env->priv;
    if (rw != QEMU_IN_FETCH) {
         if(get_field(env->csr[NEW_CSR_MSTATUS], MSTATUS_MPRV)) {
             mode = get_field(env->csr[NEW_CSR_MSTATUS], MSTATUS_MPP);
         }
    }
    if (get_field(env->csr[NEW_CSR_MSTATUS], MSTATUS_VM) == VM_MBARE) {
        mode = PRV_M;
    }

    // check to make sure that mmu_idx and mode that we get matches
    if (unlikely(mode != mmu_idx)) {
        fprintf(stderr, "MODE, mmu_idx mismatch\n");
        exit(1);
    }

    if (mode == PRV_M) {
        target_ulong msb_mask = (2L << 63) - 1;
        *physical = address & msb_mask;
        *prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
        return TRANSLATE_SUCCESS;
    }

    target_ulong addr = address;
    int supervisor = mode == PRV_S;
    int pum = get_field(env->csr[NEW_CSR_MSTATUS], MSTATUS_PUM);
    int mxr = get_field(env->csr[NEW_CSR_MSTATUS], MSTATUS_MXR);

    int levels, ptidxbits, ptesize;
    switch (get_field(env->csr[NEW_CSR_MSTATUS], MSTATUS_VM))
    {
      case VM_SV32:
          printf("SV32 untested. Find and remove to continue.\n");
          exit(1);
          levels = 2;
          ptidxbits = 10;
          ptesize = 4;
          break;
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
          printf("unsupported MSTATUS_VM value\n");
          exit(1);
    }

    int va_bits = PGSHIFT + levels * ptidxbits;
    target_ulong mask = (1L << (64 - (va_bits-1))) - 1;
    target_ulong masked_msbs = (addr >> (va_bits-1)) & mask;
    if (masked_msbs != 0 && masked_msbs != mask) {
        return TRANSLATE_FAIL;
    }

    target_ulong base = env->csr[NEW_CSR_SPTBR] << PGSHIFT;
    int ptshift = (levels - 1) * ptidxbits;
    int i;
    for (i = 0; i < levels; i++, ptshift -= ptidxbits) {
        target_ulong idx = (addr >> (PGSHIFT + ptshift)) & ((1 << ptidxbits) - 1);

        // check that physical address of PTE is legal
        target_ulong pte_addr = base + idx * ptesize;

        // PTE must reside in memory
        if (!(pte_addr >= DRAM_BASE && pte_addr < (DRAM_BASE + env->memsize))) {
            printf("PTE was not in DRAM region\n");
            exit(1);
            break;
        }

        target_ulong pte = ldq_phys(cs->as, pte_addr);
        target_ulong ppn = pte >> PTE_PPN_SHIFT;

        if (PTE_TABLE(pte)) { // next level of page table
            base = ppn << PGSHIFT;
        } else if ((pte & PTE_U) ? supervisor && pum : !supervisor) {
            break;
        } else if (!(pte & PTE_V) || (!(pte & PTE_R) && (pte & PTE_W))) {
            break;
        } else if (rw == QEMU_IN_FETCH ? !(pte & PTE_X) :
                   rw == QEMU_IN_READ ?  !(pte & PTE_R) && !(mxr && (pte & PTE_X)) :
                                   !((pte & PTE_R) && (pte & PTE_W))) {
            break;
        } else {
            // set accessed and possibly dirty bits.
            // we only put it in the TLB if it has the right stuff
            stq_phys(cs->as, pte_addr, ldq_phys(cs->as, pte_addr) | PTE_A |
                    ((rw == QEMU_IN_WRITE) * PTE_D));

            // for superpage mappings, make a fake leaf PTE for the TLB's benefit.
            target_ulong vpn = addr >> PGSHIFT;
            *physical = (ppn | (vpn & ((1L << ptshift) - 1))) << PGSHIFT;

            // we do not give all prots indicated by the PTE
            // this is because future accesses need to do things like set the
            // dirty bit on the PTE
            //
            // at this point, we assume that protection checks have occurred
            if (supervisor) {
                if ((pte & PTE_X) && rw == QEMU_IN_FETCH) {
                    *prot |= PAGE_EXEC;
                } else if ((pte & PTE_W) && rw == QEMU_IN_WRITE) {
                    *prot |= PAGE_WRITE;
                } else if ((pte & PTE_R) && rw == QEMU_IN_READ) {
                    *prot |= PAGE_READ;
                } else {
                    printf("err in translation prots");
                    exit(1);
                }
            } else {
                if ((pte & PTE_X) && rw == QEMU_IN_FETCH) {
                    *prot |= PAGE_EXEC;
                } else if ((pte & PTE_W) && rw == QEMU_IN_WRITE) {
                    *prot |= PAGE_WRITE;
                } else if ((pte & PTE_R) && rw == QEMU_IN_READ) {
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
#endif

static void raise_mmu_exception(CPURISCVState *env, target_ulong address,
                                int rw)
{
    CPUState *cs = CPU(riscv_env_get_cpu(env));
    int exception = 0;
    if (rw == QEMU_IN_FETCH) { // inst access
        exception = NEW_RISCV_EXCP_INST_ACCESS_FAULT;
        env->badaddr = address;
    } else if (rw == QEMU_IN_WRITE) { // store access
        exception = NEW_RISCV_EXCP_STORE_AMO_ACCESS_FAULT;
        env->badaddr = address;
    } else if (rw == QEMU_IN_READ) { // load access
        exception = NEW_RISCV_EXCP_LOAD_ACCESS_FAULT;
        env->badaddr = address;
    } else {
        fprintf(stderr, "FAIL: invalid rw\n");
        exit(1);
    }
    cs->exception_index = exception;
}

#if !defined(CONFIG_USER_ONLY)
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
#endif

/* This is called when there is no QEMU "TLB" match
 *
 * Assuming system mode, only called in target-riscv/op_helper:tlb_fill
 */
int riscv_cpu_handle_mmu_fault(CPUState *cs, vaddr address, int rw, int mmu_idx)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;
    hwaddr physical;
    physical = 0; // stop gcc complaining
    int prot;
    int ret = 0;

    qemu_log("%s pc " TARGET_FMT_lx " ad %" VADDR_PRIx " rw %d mmu_idx %d\n",
              __func__, env->active_tc.PC, address, rw, mmu_idx);

    ret = get_physical_address(env, &physical, &prot, address, rw, mmu_idx);
    qemu_log("%s address=%" VADDR_PRIx " ret %d physical " TARGET_FMT_plx
             " prot %d\n",
             __func__, address, ret, physical, prot);
    if (ret == TRANSLATE_SUCCESS) {
        tlb_set_page(cs, address & TARGET_PAGE_MASK, physical & TARGET_PAGE_MASK,
                prot, mmu_idx, TARGET_PAGE_SIZE);
    } else if (ret == TRANSLATE_FAIL) {
        raise_mmu_exception(env, address, rw);
    }
    return ret;
}

static const char * const riscv_excp_names[12] = {
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
};

static const char * const riscv_interrupt_names[14] = {
    "",
    "S Soft interrupt",
    "H Soft interrupt",
    "M Soft interrupt",
    "",
    "S Timer interrupt",
    "H Timer interrupt",
    "M Timer interrupt",
    "", 
    "S Ext interrupt",
    "H Ext interrupt",
    "M Ext interrupt",
    "COP interrupt",
    "Host interrupt"
};

/* handle traps. similar to take_trap in spike */
void riscv_cpu_do_interrupt(CPUState *cs)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;

    #ifdef RISCV_DEBUG_INTERRUPT
    if (cs->exception_index & 0x70000000) {
        fprintf(stderr, "core   0: exception trap_%s, epc 0x%016lx\n",
                riscv_interrupt_names[cs->exception_index & 0x0fffffff], 
                env->active_tc.PC);
    } else {
        fprintf(stderr, "core   0: exception trap_%s, epc 0x%016lx\n",
                riscv_excp_names[cs->exception_index], env->active_tc.PC);
    }
    #endif

    if (cs->exception_index == NEW_RISCV_EXCP_BREAKPOINT) {
        fprintf(stderr, "debug mode not implemented\n");
    }

    // skip dcsr cause check

    target_ulong fixed_cause = 0;
    if (cs->exception_index & (0x70000000)) {
        // hacky for now. the MSB (bit 63) indicates interrupt but cs->exception
        // index is only 32 bits wide
        fixed_cause = cs->exception_index & 0x0FFFFFFF;
        fixed_cause |= (1L << 63);
    } else {
        // fixup User ECALL -> correct priv ECALL
        if (cs->exception_index == NEW_RISCV_EXCP_U_ECALL) {
            switch(env->priv) {
                case PRV_U:
                    fixed_cause = NEW_RISCV_EXCP_U_ECALL;
                    break;
                case PRV_S:
                    fixed_cause = NEW_RISCV_EXCP_S_ECALL;
                    break;
                case PRV_H:
                    fixed_cause = NEW_RISCV_EXCP_H_ECALL;
                    break;
                case PRV_M:
                    fixed_cause = NEW_RISCV_EXCP_M_ECALL;
                    break;
            }
        } else {
            fixed_cause = cs->exception_index;
        }
    }

    target_ulong backup_epc = env->active_tc.PC;

    target_ulong bit = fixed_cause;
    target_ulong deleg = env->csr[NEW_CSR_MEDELEG];

    int hasbadaddr = 
        (fixed_cause == NEW_RISCV_EXCP_INST_ADDR_MIS) ||
        (fixed_cause == NEW_RISCV_EXCP_INST_ACCESS_FAULT) ||
        (fixed_cause == NEW_RISCV_EXCP_LOAD_ADDR_MIS) || 
        (fixed_cause == NEW_RISCV_EXCP_STORE_AMO_ADDR_MIS) ||
        (fixed_cause == NEW_RISCV_EXCP_LOAD_ACCESS_FAULT) ||
        (fixed_cause == NEW_RISCV_EXCP_STORE_AMO_ACCESS_FAULT);

    if (bit & ((target_ulong)1 << (64-1))) {
        deleg = env->csr[NEW_CSR_MIDELEG], bit &= ~((target_ulong)1 << (64-1));
    }

    if (env->priv <= PRV_S && bit < 64 && ((deleg >> bit) & 1)) {
        // handle the trap in S-mode
        env->active_tc.PC = env->csr[NEW_CSR_STVEC];
        env->csr[NEW_CSR_SCAUSE] = fixed_cause;  
        env->csr[NEW_CSR_SEPC] = backup_epc;

        if (hasbadaddr) {
            #ifdef RISCV_DEBUG_INTERRUPT
            fprintf(stderr, "core   0: badaddr 0x%016lx\n", env->badaddr);
            #endif
            env->csr[NEW_CSR_SBADADDR] = env->badaddr;
        }
  
        target_ulong s = env->csr[NEW_CSR_MSTATUS];
        s = set_field(s, MSTATUS_SPIE, get_field(s, MSTATUS_UIE << env->priv));
        s = set_field(s, MSTATUS_SPP, env->priv);
        s = set_field(s, MSTATUS_SIE, 0);
        csr_write_helper(env, s, NEW_CSR_MSTATUS);
        set_privilege2(env, PRV_S);
    } else {
        env->active_tc.PC = env->csr[NEW_CSR_MTVEC];
        env->csr[NEW_CSR_MEPC] = backup_epc;
        env->csr[NEW_CSR_MCAUSE] = fixed_cause;  
        
        if (hasbadaddr) {
            #ifdef RISCV_DEBUG_INTERRUPT
            fprintf(stderr, "core   0: badaddr 0x%016lx\n", env->badaddr);
            #endif
            env->csr[NEW_CSR_MBADADDR] = env->badaddr;
        }

        target_ulong s = env->csr[NEW_CSR_MSTATUS];
        s = set_field(s, MSTATUS_MPIE, get_field(s, MSTATUS_UIE << env->priv));
        s = set_field(s, MSTATUS_MPP, env->priv);
        s = set_field(s, MSTATUS_MIE, 0);
        csr_write_helper(env, s, NEW_CSR_MSTATUS);
        set_privilege2(env, PRV_M);
    }
    // TODO yield load reservation 

    cs->exception_index = EXCP_NONE; // mark handled to qemu
}
