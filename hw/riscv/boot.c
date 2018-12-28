/*
 * QEMU RISCV firmware and kernel loader
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

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "hw/loader.h"
#include "hw/boards.h"
#include "sysemu/device_tree.h"
#include "elf.h"
#include "hw/riscv/boot.h"

#define RISCV_BOOT_DEBUG 0

#define boot_debug(fs, ...) \
    if (RISCV_BOOT_DEBUG) { \
        fprintf(stderr, "boot: %s: "fs, __func__, ##__VA_ARGS__); \
    }

/*
 * Limit the maximum RAM size to 2 GiB to stay within the 32-bit address space
 * and ensure compatibility with both RV32I and RV64I. This assumes that RAM
 * starts at an offset of 2 GiB.
 */
#define RISCV_BIN_LIMIT (2UL << 30)

static uint64_t kernel_offset;

static uint64_t kernel_translate(void *opaque, uint64_t addr)
{
    /* mask kernel virtual address and offset by load address */
    if (kernel_offset) {
        return (addr & 0x7fffffff) + kernel_offset;
    } else {
        return addr;
    }
}

hwaddr riscv_load_firmware(const char *filename, hwaddr ram_start)
{
    uint64_t firmware_entry, firmware_start, firmware_end;
    int64_t size;

    if (load_elf(filename, NULL, NULL,
                 &firmware_entry, &firmware_start, &firmware_end,
                 0, EM_RISCV, 1, 0) < 0) {
        firmware_entry = ram_start;
        firmware_start = ram_start;

        size = load_image_targphys(filename, firmware_start, RISCV_BIN_LIMIT);
        if (size < 0) {
            error_report("riscv_boot: could not load firmware '%s'", filename);
            exit(1);
        }

        firmware_end = firmware_start + size;
    }

    /* align kernel load address to the megapage after the firmware */
#if defined(TARGET_RISCV32)
    kernel_offset = (firmware_end + 0x3fffff) & ~0x3fffff;
#else
    kernel_offset = (firmware_end + 0x1fffff) & ~0x1fffff;
#endif

    boot_debug("entry=0x" TARGET_FMT_plx " start=0x" TARGET_FMT_plx " "
               "end=0x" TARGET_FMT_plx " kernel_offset=0x" TARGET_FMT_plx "\n",
               firmware_entry, firmware_start, firmware_end, kernel_offset);

    return firmware_entry;
}

hwaddr riscv_load_kernel(const char *filename, void *fdt, hwaddr ram_start)
{
    uint64_t kernel_entry, kernel_start, kernel_end;
    int64_t size;

    if (load_elf(filename, kernel_translate, NULL,
                 &kernel_entry, &kernel_start, &kernel_end,
                 0, EM_RISCV, 1, 0) < 0) {
        if (kernel_offset) {
            kernel_entry = kernel_offset;
            kernel_start = kernel_offset;
        } else {
            kernel_entry = ram_start;
            kernel_start = ram_start;
        }

        size = load_image_targphys(filename, kernel_start, RISCV_BIN_LIMIT);
        if (size < 0) {
            error_report("riscv_boot: could not load kernel '%s'", filename);
            exit(1);
        }

        kernel_end = kernel_start + size;
    }

    boot_debug("entry=0x" TARGET_FMT_plx " start=0x" TARGET_FMT_plx " "
               "end=0x" TARGET_FMT_plx "\n", kernel_entry, kernel_start,
               kernel_end);

    /*
     * pass kernel load address via device-tree to firmware
     *
     * BBL reads the kernel address from device-tree
     */
    if (fdt) {
        qemu_fdt_setprop_cells(fdt, "/chosen", "riscv,kernel-end",
                               kernel_end >> 32, kernel_end);
        qemu_fdt_setprop_cells(fdt, "/chosen", "riscv,kernel-start",
                               kernel_start >> 32, kernel_start);
    }

    return kernel_entry;
}

void riscv_load_initrd(const char *filename, uint64_t mem_size,
                       hwaddr firmware_entry, void *fdt)
{
    uint64_t start, size;

    /* We want to put the initrd far enough into RAM that when the
     * kernel is uncompressed it will not clobber the initrd. However
     * on boards without much RAM we must ensure that we still leave
     * enough room for a decent sized initrd, and on boards with large
     * amounts of RAM we must avoid the initrd being so far up in RAM
     * that it is outside lowmem and inaccessible to the kernel.
     * So for boards with less  than 256MB of RAM we put the initrd
     * halfway into RAM, and for boards with 256MB of RAM or more we put
     * the initrd at 128MB.
     */
    start = firmware_entry + MIN(mem_size / 2, 128 * 1024 * 1024);

    size = load_ramdisk(filename, start, mem_size - start);
    if (size == -1) {
        size = load_image_targphys(filename, start, mem_size - start);
        if (size == -1) {
            error_report("riscv_boot: could not load ramdisk '%s'", filename);
            exit(1);
        }
    }

    boot_debug("start=0x" TARGET_FMT_plx " end=0x" TARGET_FMT_plx "\n",
               start, start + size);

    /*
     * pass initrd load address via device-tree to kernel
     *
     * linux-kernel reads the initrd address from device-tree
     */
    if (fdt) {
        qemu_fdt_setprop_cells(fdt, "/chosen", "linux,initrd-end",
                               (start + size) >> 32, start + size);
        qemu_fdt_setprop_cells(fdt, "/chosen", "linux,initrd-start",
                               start >> 32, start);
    }
}

hwaddr riscv_load_firmware_kernel_initrd(MachineState *machine, void *fdt,
                                         hwaddr ram_start)
{
    hwaddr firmware_entry = 0;

    /* load firmware e.g. -bios bbl */
    if (machine->firmware) {
        firmware_entry = riscv_load_firmware(machine->firmware, ram_start);
    }

    /* load combined bbl+kernel or separate kernel */
    if (machine->kernel_filename) {
        if (machine->firmware) {
            /* load separate bios and kernel e.g. -bios bbl -kernel vmlinux */
            riscv_load_kernel(machine->kernel_filename, fdt, ram_start);
        } else {
            /* load traditional combined bbl+kernel e.g. -kernel bbl_vmlimux */
            firmware_entry = riscv_load_kernel(machine->kernel_filename, NULL,
                                               ram_start);
        }
        if (machine->initrd_filename) {
            /* load separate initrd */
            riscv_load_initrd(machine->initrd_filename, machine->ram_size,
                              firmware_entry, fdt);
        }
    }

    return firmware_entry;
}
