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

target_long riscv_flush_icache_syscall(CPURISCVState *env, int num,
        target_long cmd, target_long arg1, target_long arg2, target_long arg3)
{
    CPUState *cs = ENV_GET_CPU(env);

    trace_guest_user_syscall(cs, num, cmd, arg1, arg2, arg3, 0, 0, 0, 0);

    tlb_flush(cs);
    tb_flush(cs);

    return 0;
}

#endif
