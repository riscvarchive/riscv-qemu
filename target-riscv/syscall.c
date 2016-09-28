/*
 * RISC-V Architecture-specific Syscalls for linux-user mode
 *
 * Copyright (c) 2016 Alex Suykov <alex.suykov@gmail.com>
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
#include "cpu.h"

#ifdef CONFIG_USER_ONLY

#include "qemu.h"

static target_long riscv_syscall_cmpxchg(target_long addr,
        target_long vold, target_long vnew)
{
    uint32_t prev;

    if(get_user_u32(prev, addr))
        return -TARGET_EFAULT;
    if(prev != vold)
        return prev;
    if(put_user_u32(vnew, addr))
        return -TARGET_EFAULT;

    return prev;
}

static target_long riscv_syscall_cmpxchg64(target_long addr,
        target_long vold, target_long vnew)
{
    uint64_t prev;

    if(get_user_u64(prev, addr))
        return -TARGET_EFAULT;
    if(prev != vold)
        return prev;
    if(put_user_u64(vnew, addr))
        return -TARGET_EFAULT;

    return prev;
}

static void riscv_log_cmpxchg(const char* cmdname,
        target_long arg1, target_long arg2, target_long arg3)
{
    if(!do_strace)
        return;

    gemu_log("%d ", getpid());
    gemu_log("sysriscv(%s, 0x" TARGET_ABI_FMT_lx ", "
            TARGET_ABI_FMT_ld ", " TARGET_ABI_FMT_ld ")\n",
            cmdname, arg1, arg2, arg3);
}

static void riscv_log_unknown(target_long cmd,
        target_long arg1, target_long arg2, target_long arg3)
{
    if(!do_strace)
        return;

    gemu_log("%d ", getpid());
    gemu_log("sysriscv(" TARGET_ABI_FMT_ld ", 0x" TARGET_ABI_FMT_lx ", "
            TARGET_ABI_FMT_ld ", " TARGET_ABI_FMT_ld ")\n",
            cmd, arg1, arg2, arg3);
}

/* Syscall 244 sysriscv(cmd, addr, vold, vnew) atomic compare-and-exchange,

       if(*addr == vold) *addr = vnew

   This gets called instead of do_syscall, in an exclusive section
   with all cpus stopped. */

#define RISCV_ATOMIC_CMPXCHG    1
#define RISCV_ATOMIC_CMPXCHG64  2

target_long riscv_arch_specific_syscall(CPURISCVState *env, int num,
        target_long cmd, target_long arg1, target_long arg2, target_long arg3)
{
    CPUState *cpu = ENV_GET_CPU(env);

    trace_guest_user_syscall(cpu, num, cmd, arg1, arg2, arg3, 0, 0, 0, 0);

    switch (cmd) {
        case RISCV_ATOMIC_CMPXCHG:
            riscv_log_cmpxchg("CMPXCHG", arg1, arg2, arg3);
            return riscv_syscall_cmpxchg(arg1, arg2, arg3);
        case RISCV_ATOMIC_CMPXCHG64:
            riscv_log_cmpxchg("CMPXCHG64", arg1, arg2, arg3);
            return riscv_syscall_cmpxchg64(arg1, arg2, arg3);
        default:
            riscv_log_unknown(cmd, arg1, arg2, arg3);
            return -TARGET_ENOSYS;
    }
}

#endif
