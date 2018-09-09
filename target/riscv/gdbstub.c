/*
 * RISC-V GDB Server Stub
 *
 * Copyright (c) 2016-2017 Sagar Karandikar, sagark@eecs.berkeley.edu
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "exec/gdbstub.h"
#include "cpu.h"

int riscv_cpu_gdb_read_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;

    if (n < 32) {
        return gdb_get_regl(mem_buf, env->gpr[n]);
    } else if (n == 32) {
        return gdb_get_regl(mem_buf, env->pc);
    }
    return 0;
}

int riscv_cpu_gdb_write_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;

    if (n == 0) {
        /* discard writes to x0 */
        return sizeof(target_ulong);
    } else if (n < 32) {
        env->gpr[n] = ldtul_p(mem_buf);
        return sizeof(target_ulong);
    } else if (n == 32) {
        env->pc = ldtul_p(mem_buf);
        return sizeof(target_ulong);
    }
    return 0;
}

static int riscv_gdb_get_fpu(CPURISCVState *env, uint8_t *mem_buf, int n)
{
    if (n < 32) {
        return gdb_get_reg64(mem_buf, env->fpr[n]);
    }
    return 0;
}

static int riscv_gdb_set_fpu(CPURISCVState *env, uint8_t *mem_buf, int n)
{
    if (n < 32) {
        env->fpr[n] = ldq_p(mem_buf); /* always 64-bit */
        return sizeof(uint64_t);
    }
    return 0;
}

static int riscv_gdb_get_csr(CPURISCVState *env, uint8_t *mem_buf, int n)
{
    if (n < 4096) {
        target_ulong val = 0;
        if (riscv_csrrw(env, n, &val, 0, 0) == 0) {
            return gdb_get_regl(mem_buf, val);
        }
    }
    return 0;
}

static int riscv_gdb_set_csr(CPURISCVState *env, uint8_t *mem_buf, int n)
{
    if (n < 4096) {
        target_ulong val = ldtul_p(mem_buf);
        if (riscv_csrrw(env, n, NULL, val, -1) == 0) {
            return sizeof(target_ulong);
        }
    }
    return 0;
}

void riscv_cpu_register_gdb_regs_for_features(CPUState *cs)
{
    /* ??? Assume all targets have FPU regs for now.  */
    gdb_register_coprocessor(cs, riscv_gdb_get_fpu, riscv_gdb_set_fpu,
                             32, "riscv-fpu.xml", 0);

    gdb_register_coprocessor(cs, riscv_gdb_get_csr, riscv_gdb_set_csr,
                             4096, "riscv-csr.xml", 0);
}
