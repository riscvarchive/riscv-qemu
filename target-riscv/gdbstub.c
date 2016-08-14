/*
 * RISC-V GDB Server Stub
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

#include "config.h"
#include "qemu-common.h"
#include "exec/gdbstub.h"
#include "cpu.h"

// TODO: fix after priv 1.9 bump

// map to CSR NOs for GDB
/*int indexed_csrs[] = {
    CSR_FFLAGS,
    CSR_FRM,
    CSR_FCSR,
    CSR_CYCLE,
    CSR_TIME,
    CSR_INSTRET,
    CSR_STATS,
    CSR_UARCH0,
    CSR_UARCH1,
    CSR_UARCH2,
    CSR_UARCH3,
    CSR_UARCH4,
    CSR_UARCH5,
    CSR_UARCH6,
    CSR_UARCH7,
    CSR_UARCH8,
    CSR_UARCH9,
    CSR_UARCH10,
    CSR_UARCH11,
    CSR_UARCH12,
    CSR_UARCH13,
    CSR_UARCH14,
    CSR_UARCH15,
    CSR_SSTATUS,
    CSR_STVEC,
    CSR_SIE,
    CSR_SSCRATCH,
    CSR_SEPC,
    CSR_SIP,
    CSR_SPTBR,
    CSR_SASID,
    CSR_CYCLEW,
    CSR_TIMEW,
    CSR_INSTRETW,
    CSR_STIME,
    CSR_SCAUSE,
    CSR_SBADADDR,
    CSR_STIMEW,
    CSR_MSTATUS,
    CSR_MTVEC,
    CSR_MTDELEG,
    CSR_MIE,
    CSR_MTIMECMP,
    CSR_MSCRATCH,
    CSR_MEPC,
    CSR_MCAUSE,
    CSR_MBADADDR,
    CSR_MIP,
    CSR_MTIME,
    CSR_MCPUID,
    CSR_MIMPID,
    CSR_MHARTID,
    CSR_MTOHOST,
    CSR_MFROMHOST,
    CSR_MRESET,
    CSR_MIPI,
    CSR_MIOBASE,
    CSR_CYCLEH,
    CSR_TIMEH,
    CSR_INSTRETH,
    CSR_CYCLEHW,
    CSR_TIMEHW,
    CSR_INSTRETHW,
    CSR_STIMEH,
    CSR_STIMEHW,
    CSR_MTIMECMPH,
    CSR_MTIMEH
};*/

int riscv_cpu_gdb_read_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    /*
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;
    int target_csrno;

    if (n < 32) {
        return gdb_get_regl(mem_buf, env->active_tc.gpr[n]);
    } else if (n == 32) {
        return gdb_get_regl(mem_buf, env->active_tc.PC);
    } else if (n < 65) {
        return gdb_get_regl(mem_buf, env->active_tc.fpr[n-33]);
    } else if (n < 132) {
        n -= 65;
        target_csrno = indexed_csrs[n];
        switch (target_csrno) 
        {
        // 32-bit wide 
        case CSR_FFLAGS:
        case CSR_FRM:
        case CSR_FCSR:
            return gdb_get_reg32(mem_buf, csr_read_helper(env, target_csrno));

        // unused on RV64 or not implemented
        case CSR_MTIMEH:
        case CSR_STIMEH:
        case CSR_STIMEHW:
        case CSR_TIMEH:
        case CSR_TIMEHW:
        case CSR_CYCLEH:
        case CSR_INSTRETH:
        case CSR_CYCLEHW:
        case CSR_INSTRETHW:
        case CSR_STATS:
        case CSR_MRESET:
        case CSR_MTIMECMPH:
            return gdb_get_regl(mem_buf, 0L);

        // special MFROMHOST, MTOHOST
        case CSR_MFROMHOST:
        case CSR_MTOHOST:
            return gdb_get_regl(mem_buf, env->csr[target_csrno]);

        // all others
        default:
            return gdb_get_regl(mem_buf, csr_read_helper(env, target_csrno));
        }
    }
    */
    return 0;
}

int riscv_cpu_gdb_write_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    /*
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;
    target_ulong tmp;
    int target_csrno;

    tmp = ldtul_p(mem_buf);

    if (n < 32) {
        env->active_tc.gpr[n] = tmp;
        return sizeof(target_ulong);
    } else if (n == 32) {
        env->active_tc.PC = tmp;
        return sizeof(target_ulong);
    } else if (n < 65) {
        env->active_tc.fpr[n-33] = tmp;
        return sizeof(target_ulong);
    } else if (n < 132) {
        n -= 65;
        target_csrno = indexed_csrs[n];
        env->csr[target_csrno] = tmp;
        if (n < 3) {
            return sizeof(uint32_t);
        } else {
            return sizeof(target_ulong);
        }
    }
    */
    return 0;
}
