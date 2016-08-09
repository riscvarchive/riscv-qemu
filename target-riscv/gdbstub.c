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
    NEW_CSR_FFLAGS,
    NEW_CSR_FRM,
    NEW_CSR_FCSR,
    NEW_CSR_CYCLE,
    NEW_CSR_TIME,
    NEW_CSR_INSTRET,
    NEW_CSR_STATS,
    NEW_CSR_UARCH0,
    NEW_CSR_UARCH1,
    NEW_CSR_UARCH2,
    NEW_CSR_UARCH3,
    NEW_CSR_UARCH4,
    NEW_CSR_UARCH5,
    NEW_CSR_UARCH6,
    NEW_CSR_UARCH7,
    NEW_CSR_UARCH8,
    NEW_CSR_UARCH9,
    NEW_CSR_UARCH10,
    NEW_CSR_UARCH11,
    NEW_CSR_UARCH12,
    NEW_CSR_UARCH13,
    NEW_CSR_UARCH14,
    NEW_CSR_UARCH15,
    NEW_CSR_SSTATUS,
    NEW_CSR_STVEC,
    NEW_CSR_SIE,
    NEW_CSR_SSCRATCH,
    NEW_CSR_SEPC,
    NEW_CSR_SIP,
    NEW_CSR_SPTBR,
    NEW_CSR_SASID,
    NEW_CSR_CYCLEW,
    NEW_CSR_TIMEW,
    NEW_CSR_INSTRETW,
    NEW_CSR_STIME,
    NEW_CSR_SCAUSE,
    NEW_CSR_SBADADDR,
    NEW_CSR_STIMEW,
    NEW_CSR_MSTATUS,
    NEW_CSR_MTVEC,
    NEW_CSR_MTDELEG,
    NEW_CSR_MIE,
    NEW_CSR_MTIMECMP,
    NEW_CSR_MSCRATCH,
    NEW_CSR_MEPC,
    NEW_CSR_MCAUSE,
    NEW_CSR_MBADADDR,
    NEW_CSR_MIP,
    NEW_CSR_MTIME,
    NEW_CSR_MCPUID,
    NEW_CSR_MIMPID,
    NEW_CSR_MHARTID,
    NEW_CSR_MTOHOST,
    NEW_CSR_MFROMHOST,
    NEW_CSR_MRESET,
    NEW_CSR_MIPI,
    NEW_CSR_MIOBASE,
    NEW_CSR_CYCLEH,
    NEW_CSR_TIMEH,
    NEW_CSR_INSTRETH,
    NEW_CSR_CYCLEHW,
    NEW_CSR_TIMEHW,
    NEW_CSR_INSTRETHW,
    NEW_CSR_STIMEH,
    NEW_CSR_STIMEHW,
    NEW_CSR_MTIMECMPH,
    NEW_CSR_MTIMEH
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
        case NEW_CSR_FFLAGS:
        case NEW_CSR_FRM:
        case NEW_CSR_FCSR:
            return gdb_get_reg32(mem_buf, csr_read_helper(env, target_csrno));

        // unused on RV64 or not implemented
        case NEW_CSR_MTIMEH:
        case NEW_CSR_STIMEH:
        case NEW_CSR_STIMEHW:
        case NEW_CSR_TIMEH:
        case NEW_CSR_TIMEHW:
        case NEW_CSR_CYCLEH:
        case NEW_CSR_INSTRETH:
        case NEW_CSR_CYCLEHW:
        case NEW_CSR_INSTRETHW:
        case NEW_CSR_STATS:
        case NEW_CSR_MRESET:
        case NEW_CSR_MTIMECMPH:
            return gdb_get_regl(mem_buf, 0L);

        // special MFROMHOST, MTOHOST
        case NEW_CSR_MFROMHOST:
        case NEW_CSR_MTOHOST:
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
