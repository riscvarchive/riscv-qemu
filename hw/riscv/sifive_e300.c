/*
 * QEMU RISC-V Board Compatible with SiFive E300 SDK
 *
 * Copyright (c) 2017 SiFive, Inc.
 *
 * Provides a board compatible with the bsp in the SiFive E300 SDK:
 *
 * 0) UART
 * 1) CLINT (Core Level Interruptor)
 * 2) PLIC (Platform Level Interrupt Controller)
 * 3) PRCI (Power, Reset, Clock, Interrupt)
 * 4) Registers emulated as RAM: AON, GPIO, QSPI, PWM
 * 5) Flash memory emulated as RAM
 *
 * The Mask ROM reset vector jumps to the flash payload at 0x2040_0000.
 * The OTP ROM and Flash boot code will be emulated in a future version.
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
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "hw/char/serial.h"
#include "target/riscv/cpu.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/riscv/sifive_plic.h"
#include "hw/riscv/sifive_clint.h"
#include "hw/riscv/sifive_prci.h"
#include "hw/riscv/sifive_uart.h"
#include "hw/riscv/sifive_e300.h"
#include "chardev/char.h"
#include "sysemu/arch_init.h"
#include "exec/address-spaces.h"
#include "elf.h"

static const struct MemmapEntry {
    hwaddr base;
    hwaddr size;
} sifive_e300_memmap[] = {
    [SIFIVE_E300_DEBUG] =    {        0x0,      0x100 },
    [SIFIVE_E300_MROM] =     {     0x1000,     0x2000 },
    [SIFIVE_E300_OTP] =      {    0x20000,     0x2000 },
    [SIFIVE_E300_CLINT] =    {  0x2000000,    0x10000 },
    [SIFIVE_E300_PLIC] =     {  0xc000000,  0x4000000 },
    [SIFIVE_E300_AON] =      { 0x10000000,     0x8000 },
    [SIFIVE_E300_PRCI] =     { 0x10008000,     0x8000 },
    [SIFIVE_E300_OTP_CTRL] = { 0x10010000,     0x1000 },
    [SIFIVE_E300_GPIO0] =    { 0x10012000,     0x1000 },
    [SIFIVE_E300_UART0] =    { 0x10013000,     0x1000 },
    [SIFIVE_E300_QSPI0] =    { 0x10014000,     0x1000 },
    [SIFIVE_E300_PWM0] =     { 0x10015000,     0x1000 },
    [SIFIVE_E300_UART1] =    { 0x10023000,     0x1000 },
    [SIFIVE_E300_QSPI1] =    { 0x10024000,     0x1000 },
    [SIFIVE_E300_PWM1] =     { 0x10025000,     0x1000 },
    [SIFIVE_E300_QSPI2] =    { 0x10034000,     0x1000 },
    [SIFIVE_E300_PWM2] =     { 0x10035000,     0x1000 },
    [SIFIVE_E300_XIP] =      { 0x20000000, 0x20000000 },
    [SIFIVE_E300_DTIM] =     { 0x80000000,     0x4000 }
};

static void copy_le32_to_phys(hwaddr pa, uint32_t *rom, size_t len)
{
    int i;
    for (i = 0; i < (len >> 2); i++) {
        stl_phys(&address_space_memory, pa + (i << 2), rom[i]);
    }
}

static uint64_t identity_translate(void *opaque, uint64_t addr)
{
    return addr;
}

static uint64_t load_kernel(const char *kernel_filename)
{
    uint64_t kernel_entry, kernel_high;

    if (load_elf(kernel_filename, identity_translate, NULL,
                 &kernel_entry, NULL, &kernel_high,
                 0, ELF_MACHINE, 1, 0) < 0) {
        error_report("qemu: could not load kernel '%s'", kernel_filename);
        exit(1);
    }
    return kernel_entry;
}

static void sifive_mmio_emulate(MemoryRegion *parent, const char *name,
                             uintptr_t offset, uintptr_t length)
{
    MemoryRegion *mock_mmio = g_new(MemoryRegion, 1);
    memory_region_init_ram(mock_mmio, NULL, name, length, &error_fatal);
    memory_region_add_subregion(parent, offset, mock_mmio);
}

static void riscv_sifive_e300_init(MachineState *machine)
{
    const struct MemmapEntry *memmap = sifive_e300_memmap;

    SiFiveE300State *s = g_new0(SiFiveE300State, 1);
    MemoryRegion *sys_mem = get_system_memory();
    MemoryRegion *main_mem = g_new(MemoryRegion, 1);
    MemoryRegion *mask_rom = g_new(MemoryRegion, 1);
    MemoryRegion *xip_mem = g_new(MemoryRegion, 1);

    /* Initialize SOC */
    object_initialize(&s->soc, sizeof(s->soc), TYPE_RISCV_HART_ARRAY);
    object_property_add_child(OBJECT(machine), "soc", OBJECT(&s->soc),
                              &error_abort);
    object_property_set_str(OBJECT(&s->soc), TYPE_RISCV_CPU_IMAC_PRIV_1_10,
                            "cpu-type", &error_abort);
    object_property_set_int(OBJECT(&s->soc), smp_cpus, "num-harts",
                            &error_abort);
    object_property_set_bool(OBJECT(&s->soc), true, "realized",
                            &error_abort);

    /* Data Tightly Integrated Memory */
    memory_region_init_ram(main_mem, NULL, "riscv.sifive.e300.ram",
        memmap[SIFIVE_E300_DTIM].size, &error_fatal);
    memory_region_add_subregion(sys_mem,
        memmap[SIFIVE_E300_DTIM].base, main_mem);

    /* Mask ROM */
    memory_region_init_ram(mask_rom, NULL, "riscv.sifive.e300.mrom",
        memmap[SIFIVE_E300_MROM].size, &error_fatal);
    memory_region_add_subregion(sys_mem,
        memmap[SIFIVE_E300_MROM].base, mask_rom);

    /* MMIO */
    s->plic = sifive_plic_create(memmap[SIFIVE_E300_PLIC].base,
        (char *)SIFIVE_E300_PLIC_HART_CONFIG,
        SIFIVE_E300_PLIC_NUM_SOURCES,
        SIFIVE_E300_PLIC_NUM_PRIORITIES,
        SIFIVE_E300_PLIC_PRIORITY_BASE,
        SIFIVE_E300_PLIC_PENDING_BASE,
        SIFIVE_E300_PLIC_ENABLE_BASE,
        SIFIVE_E300_PLIC_ENABLE_STRIDE,
        SIFIVE_E300_PLIC_CONTEXT_BASE,
        SIFIVE_E300_PLIC_CONTEXT_STRIDE,
        memmap[SIFIVE_E300_PLIC].size);
    sifive_clint_create(memmap[SIFIVE_E300_CLINT].base,
        memmap[SIFIVE_E300_CLINT].size, smp_cpus,
        SIFIVE_SIP_BASE, SIFIVE_TIMECMP_BASE, SIFIVE_TIME_BASE);
    sifive_mmio_emulate(sys_mem, "riscv.sifive.e300.aon",
        memmap[SIFIVE_E300_AON].base, memmap[SIFIVE_E300_AON].size);
    sifive_prci_create(memmap[SIFIVE_E300_PRCI].base);
    sifive_mmio_emulate(sys_mem, "riscv.sifive.e300.gpio0",
        memmap[SIFIVE_E300_GPIO0].base, memmap[SIFIVE_E300_GPIO0].size);
    sifive_uart_create(sys_mem, memmap[SIFIVE_E300_UART0].base,
        serial_hds[0], SIFIVE_PLIC(s->plic)->irqs[SIFIVE_E300_UART0_IRQ]);
    sifive_mmio_emulate(sys_mem, "riscv.sifive.e300.qspi0",
        memmap[SIFIVE_E300_QSPI0].base, memmap[SIFIVE_E300_QSPI0].size);
    sifive_mmio_emulate(sys_mem, "riscv.sifive.e300.pwm0",
        memmap[SIFIVE_E300_PWM0].base, memmap[SIFIVE_E300_PWM0].size);
    /* sifive_uart_create(sys_mem, memmap[SIFIVE_E300_UART1].base,
        serial_hds[1], SIFIVE_PLIC(s->plic)->irqs[SIFIVE_E300_UART1_IRQ]); */
    sifive_mmio_emulate(sys_mem, "riscv.sifive.e300.qspi1",
        memmap[SIFIVE_E300_QSPI1].base, memmap[SIFIVE_E300_QSPI1].size);
    sifive_mmio_emulate(sys_mem, "riscv.sifive.e300.pwm1",
        memmap[SIFIVE_E300_PWM1].base, memmap[SIFIVE_E300_PWM1].size);
    sifive_mmio_emulate(sys_mem, "riscv.sifive.e300.qspi2",
        memmap[SIFIVE_E300_QSPI2].base, memmap[SIFIVE_E300_QSPI2].size);
    sifive_mmio_emulate(sys_mem, "riscv.sifive.e300.pwm2",
        memmap[SIFIVE_E300_PWM2].base, memmap[SIFIVE_E300_PWM2].size);

    /* Flash memory */
    memory_region_init_ram(xip_mem, NULL, "riscv.sifive.e300.xip",
        memmap[SIFIVE_E300_XIP].size, &error_fatal);
    memory_region_set_readonly(xip_mem, true);
    memory_region_add_subregion(sys_mem,
        memmap[SIFIVE_E300_XIP].base, xip_mem);

    /* Mask ROM reset vector */
    uint32_t reset_vec[2] = {
        0x204002b7,        /* 0x1000: lui     t0,0x20400 */
        0x00028067,        /* 0x1004: jr      t0 */
    };

    /* copy in the reset vector */
    copy_le32_to_phys(memmap[SIFIVE_E300_MROM].base,
        reset_vec, sizeof(reset_vec));
    memory_region_set_readonly(mask_rom, true);

    if (machine->kernel_filename) {
        load_kernel(machine->kernel_filename);
    }
}

static int riscv_sifive_e300_sysbus_device_init(SysBusDevice *sysbusdev)
{
    return 0;
}

static void riscv_sifive_e300_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);
    k->init = riscv_sifive_e300_sysbus_device_init;
}

static const TypeInfo riscv_sifive_e300_device = {
    .name          = TYPE_SIFIVE_E300,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SiFiveE300State),
    .class_init    = riscv_sifive_e300_class_init,
};

static void riscv_sifive_e300_machine_init(MachineClass *mc)
{
    mc->desc = "RISC-V Board compatible with SiFive E300 SDK";
    mc->init = riscv_sifive_e300_init;
    mc->max_cpus = 1;
}

DEFINE_MACHINE("sifive_e300", riscv_sifive_e300_machine_init)

static void riscv_sifive_e300_register_types(void)
{
    type_register_static(&riscv_sifive_e300_device);
}

type_init(riscv_sifive_e300_register_types);
