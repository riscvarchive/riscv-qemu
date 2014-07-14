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
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <signal.h>

#include "cpu.h"

enum {
    TLBRET_DIRTY = -4,
    TLBRET_INVALID = -3,
    TLBRET_NOMATCH = -2,
    TLBRET_BADADDR = -1,
    TLBRET_MATCH = 0
};

#if !defined(CONFIG_USER_ONLY)

/* helper for grabbing PTEs */
static uint64_t load_double_phys_le_f(CPUState *cs, int64_t physaddr)
{
    uint64_t loadval = (uint64_t)ldl_phys(cs->as, physaddr) | (((uint64_t)ldl_phys(cs->as, physaddr+4)) << 32);
    return loadval;
}

static int get_physical_address (CPUMIPSState *env, hwaddr *physical,
                                int *prot, target_ulong address,
                                int rw, int access_type)
{
    // TODO: implement permissions checking, page faults


    // flush TLB
//    MIPSCPU *cpu = mips_env_get_cpu(env);
//    tlb_flush(CPU(cpu), 1); 


    // first, check if VM is on:
    int vm_on = env->helper_csr[CSR_STATUS] & SR_VM; // check if vm on
    int ret = TLBRET_MATCH; // need to change this later probably
    if(!vm_on) { // TODO add unlikely
        *physical = address;
        *prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
    } else {
        // handle translation
        if ((address >= 0x3f8) && (address < 0x400)) {
            *physical = address;
            *prot = PAGE_READ | PAGE_WRITE /*| PAGE_EXEC*/;
            return ret;
        }

        CPUState *cs = CPU(mips_env_get_cpu(env));
        uint64_t pte = 0; 
        uint64_t base = env->helper_csr[CSR_PTBR];
        uint64_t ptd;
        int ptshift = 20;
        int64_t i = 0;
//        printf("input vaddress: %016lX\n", address);
//        printf("currentPC %016lX\n", env->active_tc.PC);

        for (i = 0; i < 3; i++, ptshift -= 10) {
            uint64_t idx = (address >> (13+ptshift)) & ((1 << 10)-1);
            uint64_t pte_addr = base + idx*8;

            ptd = load_double_phys_le_f(cs, pte_addr);
           
            if (!(ptd & 0x1)) { /*
//                printf("INVALID MAPPING (should really page fault)\n");
                printf("          input vaddr: %016lX\n", address);
//                printf("reached walk iter: %d\n", (int)i);
                printf("          access type: %x\n", access_type);
                printf("          access rw val: %x\n", rw);
                printf("          currentPC %016lX\n", env->active_tc.PC);
                printf("          ---\n");
*/
                /* NOTE: the env->active_tc.PC value visible here will not be
                 * correct, but the value visible to the exception handler 
                 * (mips_cpu_do_interrupt) is correct */

                return TLBRET_NOMATCH;
//                exit(0);
            } else if (ptd & 0x2) { 
                base = (ptd >> 13) << 13;
            } else {
                uint64_t vpn = address >> 13;
                ptd |= (vpn & ((1 <<(ptshift))-1)) << 13;
       
                // TODO: fault if physical addr is out of range
                pte = ptd;
                break;
            }
        }

        // TODO: needs optimization
        // check pte access bits
        if (env->helper_csr[CSR_STATUS] & SR_S) {
            // check supervisor
            if ((rw & 0x2) & !(pte & PTE_SX)) {
                return TLBRET_NOMATCH;
            } else if ((rw == 0x1) & !(pte & PTE_SW)) {
                return TLBRET_NOMATCH;
            } else if (!(pte & PTE_SR)) {
                return TLBRET_NOMATCH;
            }
            if (pte & PTE_SX) {
                *prot |= PAGE_EXEC;
            }
            if (pte & PTE_SW) {
                *prot |= PAGE_WRITE;
            }
            if (pte & PTE_SR) {
                *prot |= PAGE_READ;
            }
        } else {
            // check user
            if ((rw & 0x2) & !(pte & PTE_UX)) {
                return TLBRET_NOMATCH;
            } else if ((rw == 0x1) & !(pte & PTE_UW)) {
                return TLBRET_NOMATCH;
            } else if (!(pte & PTE_UR)) {
                return TLBRET_NOMATCH;
            }
            if (pte & PTE_UX) {
                *prot |= PAGE_EXEC;
            }
            if (pte & PTE_UW) {
                *prot |= PAGE_WRITE;
            }
            if (pte & PTE_UR) {
                *prot |= PAGE_READ;
            }
        }
        *physical = ((pte >> 13) << 13) | (address & 0x1FFF);
//        *prot = PAGE_EXEC | PAGE_READ | PAGE_WRITE;

//        asm("int3"); // trigger breakpoint in GDB
    }
    return ret;
}
#endif

static void raise_mmu_exception(CPUMIPSState *env, target_ulong address,
                                int rw, int tlb_error)
{
    CPUState *cs = CPU(mips_env_get_cpu(env));
    int exception = 0;

    switch (tlb_error) {
    case TLBRET_NOMATCH:
        /* No TLB match for a mapped address */
        if (rw & 0x2) { // inst access
            exception = RISCV_EXCP_INST_ACCESS_FAULT;
        } else if (rw == 0x1) { // store access
            exception = RISCV_EXCP_STORE_ACCESS_FAULT;
            env->helper_csr[CSR_BADVADDR] = address;
        } else { // load access
            exception = RISCV_EXCP_LOAD_ACCESS_FAULT;
            env->helper_csr[CSR_BADVADDR] = address;
        }
        break;
    default:
        // currently unhandled for RISCV
        printf("encountered unknown mmu fault in helper.c:raise_mmu_exception\n");
        exit(0);
        break;
    }
    cs->exception_index = exception;
}

#if !defined(CONFIG_USER_ONLY)
hwaddr mips_cpu_get_phys_page_debug(CPUState *cs, vaddr addr)
{
    MIPSCPU *cpu = MIPS_CPU(cs);
    hwaddr phys_addr;
    int prot;

    if (get_physical_address(&cpu->env, &phys_addr, &prot, addr, 0,
                             ACCESS_INT) != 0) {
        return -1;
    }
    return phys_addr;
}
#endif

// NOTE: this gets called a lot
int mips_cpu_handle_mmu_fault(CPUState *cs, vaddr address, int rw,
                              int mmu_idx)
{
    MIPSCPU *cpu = MIPS_CPU(cs);
    CPUMIPSState *env = &cpu->env;
#if !defined(CONFIG_USER_ONLY)
    hwaddr physical;
    int prot;
    int access_type;
#endif
    int ret = 0;

#if 0
    log_cpu_state(cs, 0);
#endif
    qemu_log("%s pc " TARGET_FMT_lx " ad %" VADDR_PRIx " rw %d mmu_idx %d\n",
              __func__, env->active_tc.PC, address, rw, mmu_idx);

//    printf("rw before: %x\n", rw);
//    rw &= 1; // WHYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY
//    printf("rw after: %x\n", rw);

    /* data access */
#if !defined(CONFIG_USER_ONLY)
    /* XXX: put correct access by using cpu_restore_state()
       correctly */
    access_type = ACCESS_INT; // TODO: huh? this was here from mips
    ret = get_physical_address(env, &physical, &prot,
                               address, rw, access_type);
    qemu_log("%s address=%" VADDR_PRIx " ret %d physical " TARGET_FMT_plx
             " prot %d\n",
             __func__, address, ret, physical, prot);
    if (ret == TLBRET_MATCH) {
        tlb_set_page(cs, address & TARGET_PAGE_MASK,
                     physical & TARGET_PAGE_MASK, prot | PAGE_EXEC,
                     mmu_idx, TARGET_PAGE_SIZE);
        ret = 0;
    } else if (ret < 0)
#endif
    {
        raise_mmu_exception(env, address, rw, ret);
        ret = 1;
    }

    return ret;
}

static const char * const riscv_excp_names[13] = {
    "instruction_address_misaligned",
    "instruction_access_fault",
    "illegal_instruction",
    "privileged_instruction",
    "fp_disabled",
    "UNUSED",
    "syscall",
    "breakpoint",
    "load_address_misaligned",
    "store_address_misaligned",
    "load_access_fault",
    "store_access_fault",
    "accelerator_disabled",
};


target_ulong exception_resume_pc (CPUMIPSState *env)
{
    target_ulong bad_pc;
    bad_pc = env->active_tc.PC;
    return bad_pc;
}

inline int set_badvaddr(int excp);

inline int set_badvaddr(int excp) {
    return ((excp == RISCV_EXCP_LOAD_ACCESS_FAULT) || (excp == RISCV_EXCP_STORE_ACCESS_FAULT) || (excp == RISCV_EXCP_LOAD_ADDR_MIS) || (excp == RISCV_EXCP_STORE_ADDR_MIS));
}

void mips_cpu_do_interrupt(CPUState *cs)
{
    MIPSCPU *cpu = MIPS_CPU(cs);
    CPUMIPSState *env = &cpu->env;

//    printf("%d\n", cs->exception_index);

    bool deb_inter = true;
    if (deb_inter) {
        if (!(cs->exception_index & (0x1 << 31))) {
            if ((cs->exception_index == RISCV_EXCP_ILLEGAL_INST) || (cs->exception_index == RISCV_EXCP_FP_DISABLED)) {
                printf("core   0: exception trap_%s, epc 0x%016lx\n", riscv_excp_names[cs->exception_index], env->active_tc.PC);
            }
        } else {
            // printf("SERIAL INTERRUPT");
//            printf("serial interrupt, epc 0x%016lx\n", env->active_tc.PC);
        }
    }

    // Store Cause in CSR_CAUSE. this comes from cs->exception_index
    if (cs->exception_index & (0x1 << 31)) {
        // hacky for now. the MSB (bit 63) indicates interrupt but cs->exception 
        // index is only 32 bits wide
        env->helper_csr[CSR_CAUSE] = cs->exception_index & 0x1F;
        env->helper_csr[CSR_CAUSE] |= (1L << 63);
    } else {
        env->helper_csr[CSR_CAUSE] = cs->exception_index;
    }

    // Manage the PS/S Stack: CSR_STATUS[SR_PS] = CSR_STATUS[SR_S], 
    // CSR_STATUS[SR_S] = 1 // enable supervisor
    if (env->helper_csr[CSR_STATUS] & SR_S) {
        env->helper_csr[CSR_STATUS] |= SR_PS;
    } else {
        env->helper_csr[CSR_STATUS] &= ~((uint64_t)SR_PS);
    }
    env->helper_csr[CSR_STATUS] |= SR_S; // turn on supervisor;

    // Manage the EI/PEI Stack: CSR_STATUS[SR_PEI] = CSR_STATUS[SR_EI]
    // CSR_STATUS[SR_EI] = 0 // disable interrupts
    if (env->helper_csr[CSR_STATUS] & SR_EI) {
        env->helper_csr[CSR_STATUS] |= SR_PEI;
    } else {
        env->helper_csr[CSR_STATUS] &= ~((uint64_t)SR_PEI);
    }
    env->helper_csr[CSR_STATUS] &= ~((uint64_t)SR_EI); // turn off interrupts

    // NOTE: CSR_BADVADDR should be set from the handler that raises the exception

    // Store original PC to epc reg
    // This is correct because the env->active_tc.PC value visible here is 
    // actually the correct value, unlike other places where env->active_tc.PC
    // may be used.
    env->helper_csr[CSR_EPC] = env->active_tc.PC;

    // FINALLY, set PC to value in evec register and return
    env->active_tc.PC = env->helper_csr[CSR_EVEC];

    cs->exception_index = EXCP_NONE; // mark handled to qemu
}
