/*
 *  RISC-V gdb server stub
 *
 *  Author: Sagar Karandikar, skarandikar@berkeley.edu
 *  Based on the MIPS target
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

int riscv_cpu_gdb_read_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;

    if (n < 32) {
        return gdb_get_regl(mem_buf, env->active_tc.gpr[n]);
    } else if (n < 64) {
        return gdb_get_regl(mem_buf, env->helper_csr[n-32]);
    } else if (n == 64) {
        return gdb_get_regl(mem_buf, env->active_tc.PC);
    }
    return 0;
}

int riscv_cpu_gdb_write_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;
    target_ulong tmp;

    tmp = ldtul_p(mem_buf);

    if (n < 32) {
        env->active_tc.gpr[n] = tmp;
        return sizeof(target_ulong);
    } else if (n < 64) {
        env->helper_csr[n-32] = tmp;
        return sizeof(target_ulong);
    } else if (n == 64) {
        env->active_tc.PC = tmp;
        return sizeof(target_ulong);
    }
    return sizeof(target_ulong);
}
