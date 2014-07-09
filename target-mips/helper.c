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

int r4k_map_address (CPUMIPSState *env, hwaddr *physical, int *prot,
                     target_ulong address, int rw, int access_type)
{
    printf("this was called");
    return TLBRET_NOMATCH;
}

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
/*    MIPSCPU *cpu = mips_env_get_cpu(env);
    tlb_flush(CPU(cpu), 1); */


    // first, check if VM is on:
    int vm_on = env->active_tc.csr[CSR_STATUS] & SR_VM; // check if vm on
    int ret = TLBRET_MATCH; // need to change this later probably
    if(!vm_on) { // TODO add unlikely
        *physical = address;
        *prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
    } else {
        // handle translation
/*        if ((address >= 0x3f8) && (address <= 0x400)) {
            *physical = address;
            *prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
            return ret;
        } */
        if (address < 0x2000) { // IO hole
            *physical = address;
            *prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
            return ret;
        }


/*        if ((address == 0xb)) {
            *physical = address;
            *prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
            return ret;
        }
*/




        CPUState *cs = CPU(mips_env_get_cpu(env));
        uint64_t pte = 0; 
        uint64_t base = env->active_tc.csr[CSR_PTBR];
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
/*                if (rw == 0x2) {
                    printf("core   0: exception trap_instruction_access_fault, epc 0x%016lX\n", env->active_tc.PC);
                } else if (rw == 0x1) {
                    printf("core   0: exception trap_store_access_fault, epc 0x%016lX (pc will not match)\n", env->active_tc.PC);
                } else if (rw == 0x0) {
                    printf("load access fault %016lX (pc will not match)\n", env->active_tc.PC);
                }
*/

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
        *physical = ((pte >> 13) << 13) | (address & 0x1FFF);
        *prot = PAGE_EXEC | PAGE_READ | PAGE_WRITE;

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
            env->CP0_BadVAddr = address;
        } else { // load access
            exception = RISCV_EXCP_LOAD_ACCESS_FAULT;
            env->CP0_BadVAddr = address;
        }
        break;
    default:
        // currently unhandled for RISCV
        printf("encountered unknown mmu fault in helper.c:raise_mmu_exception\n");
        exit(0);
        break;
    }
//    printf("excp: %x\n", exception);
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

static const char * const excp_names[EXCP_LAST + 1] = {
    [EXCP_RESET] = "reset",
    [EXCP_SRESET] = "soft reset",
    [EXCP_DSS] = "debug single step",
    [EXCP_DINT] = "debug interrupt",
    [EXCP_NMI] = "non-maskable interrupt",
    [EXCP_MCHECK] = "machine check",
    [EXCP_EXT_INTERRUPT] = "interrupt",
    [EXCP_DFWATCH] = "deferred watchpoint",
    [EXCP_DIB] = "debug instruction breakpoint",
    [EXCP_IWATCH] = "instruction fetch watchpoint",
    [EXCP_AdEL] = "address error load",
    [EXCP_AdES] = "address error store",
    [EXCP_TLBF] = "TLB refill",
    [EXCP_IBE] = "instruction bus error",
    [EXCP_DBp] = "debug breakpoint",
    [EXCP_SYSCALL] = "syscall",
    [EXCP_BREAK] = "break",
    [EXCP_CpU] = "coprocessor unusable",
    [EXCP_RI] = "reserved instruction",
    [EXCP_OVERFLOW] = "arithmetic overflow",
    [EXCP_TRAP] = "trap",
    [EXCP_FPE] = "floating point",
    [EXCP_DDBS] = "debug data break store",
    [EXCP_DWATCH] = "data watchpoint",
    [EXCP_LTLBL] = "TLB modify",
    [EXCP_TLBL] = "TLB load",
    [EXCP_TLBS] = "TLB store",
    [EXCP_DBE] = "data bus error",
    [EXCP_DDBL] = "debug data break load",
    [EXCP_THREAD] = "thread",
    [EXCP_MDMX] = "MDMX",
    [EXCP_C2E] = "precise coprocessor 2",
    [EXCP_CACHE] = "cache error",
};

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

    bool deb_inter = true;
    if (deb_inter) {
        printf("core   0: exception trap_%s, epc 0x%016lx\n", riscv_excp_names[cs->exception_index], env->active_tc.PC);
    }

    // Store Cause in CSR_CAUSE. this comes from cs->exception_index
    env->active_tc.csr[CSR_CAUSE] = cs->exception_index;

    // Manage the PS/S Stack: CSR_STATUS[SR_PS] = CSR_STATUS[SR_S], 
    // CSR_STATUS[SR_S] = 1 // enable supervisor
    if (env->active_tc.csr[CSR_STATUS] & SR_S) {
        env->active_tc.csr[CSR_STATUS] |= SR_PS;
    } else {
        env->active_tc.csr[CSR_STATUS] &= ~((uint64_t)SR_PS);
    }
    env->active_tc.csr[CSR_STATUS] |= SR_S; // turn on supervisor;

    // Manage the EI/PEI Stack: CSR_STATUS[SR_PEI] = CSR_STATUS[SR_EI]
    // CSR_STATUS[SR_EI] = 0 // disable interrupts
    if (env->active_tc.csr[CSR_STATUS] & SR_EI) {
        env->active_tc.csr[CSR_STATUS] |= SR_PEI;
    } else {
        env->active_tc.csr[CSR_STATUS] &= ~((uint64_t)SR_PEI);
    }
    env->active_tc.csr[CSR_STATUS] &= ~((uint64_t)SR_EI); // turn off interrupts

    // If trap is misaligned address or access fault,
    // set badvaddr to faulting address. this will be in env->CP0_BadVAddr
    if (set_badvaddr(cs->exception_index)) {
        env->active_tc.csr[CSR_BADVADDR] = env->CP0_BadVAddr;
    }

    // Store original PC to epc reg
    // This is correct because the env->active_tc.PC value visible here is 
    // actually the correct value, unlike other places where env->active_tc.PC
    // may be used.
    env->active_tc.csr[CSR_EPC] = env->active_tc.PC;

    // FINALLY, set PC to value in evec register and return
    env->active_tc.PC = env->active_tc.csr[CSR_EVEC];

    cs->exception_index = EXCP_NONE; // mark handled to qemu
}

#if !defined(CONFIG_USER_ONLY)
void r4k_invalidate_tlb (CPUMIPSState *env, int idx, int use_extra)
{
    MIPSCPU *cpu = mips_env_get_cpu(env);
    CPUState *cs;
    r4k_tlb_t *tlb;
    target_ulong addr;
    target_ulong end;
    uint8_t ASID = env->CP0_EntryHi & 0xFF;
    target_ulong mask;

    tlb = &env->tlb->mmu.r4k.tlb[idx];
    /* The qemu TLB is flushed when the ASID changes, so no need to
       flush these entries again.  */
    if (tlb->G == 0 && tlb->ASID != ASID) {
        return;
    }

    if (use_extra && env->tlb->tlb_in_use < MIPS_TLB_MAX) {
        /* For tlbwr, we can shadow the discarded entry into
           a new (fake) TLB entry, as long as the guest can not
           tell that it's there.  */
        env->tlb->mmu.r4k.tlb[env->tlb->tlb_in_use] = *tlb;
        env->tlb->tlb_in_use++;
        return;
    }

    /* 1k pages are not supported. */
    mask = tlb->PageMask | ~(TARGET_PAGE_MASK << 1);
    if (tlb->V0) {
        cs = CPU(cpu);
        addr = tlb->VPN & ~mask;
#if defined(TARGET_MIPS64)
        if (addr >= (0xFFFFFFFF80000000ULL & env->SEGMask)) {
            addr |= 0x3FFFFF0000000000ULL;
        }
#endif
        end = addr | (mask >> 1);
        while (addr < end) {
            tlb_flush_page(cs, addr);
            addr += TARGET_PAGE_SIZE;
        }
    }
    if (tlb->V1) {
        cs = CPU(cpu);
        addr = (tlb->VPN & ~mask) | ((mask >> 1) + 1);
#if defined(TARGET_MIPS64)
        if (addr >= (0xFFFFFFFF80000000ULL & env->SEGMask)) {
            addr |= 0x3FFFFF0000000000ULL;
        }
#endif
        end = addr | mask;
        while (addr - 1 < end) {
            tlb_flush_page(cs, addr);
            addr += TARGET_PAGE_SIZE;
        }
    }
}
#endif
