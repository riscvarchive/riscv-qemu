/*
 * QEMU RISCV firmware and kernel loader interface
 *
 * Copyright (c) 2017-2018 SiFive, Inc.
 *
 * Holds the state of a heterogenous array of RISC-V harts
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

#ifndef HW_RISCV_BOOT_H
#define HW_RISCV_BOOT_H

hwaddr riscv_load_firmware(const char *filename);
hwaddr riscv_load_kernel(const char *filename, void *fdt);
void riscv_load_initrd(const char *filename, uint64_t mem_size,
                       hwaddr firmware_entry, void *fdt);
hwaddr riscv_load_firmware_kernel_initrd(MachineState *machine, void *fdt);

#endif
