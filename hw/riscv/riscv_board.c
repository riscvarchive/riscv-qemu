/*
 * QEMU Board board support
 *
 * Copyright (c) 2006 Aurelien Jarno
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "hw/hw.h"
#include "hw/i386/pc.h"
#include "hw/char/serial.h"
#include "hw/riscv/htif/htif.h"
#include "hw/block/fdc.h"
#include "net/net.h"
#include "hw/boards.h"
#include "hw/i2c/smbus.h"
#include "block/block.h"
#include "hw/block/flash.h"
#include "hw/riscv/riscv.h"
#include "hw/riscv/cpudevs.h"
#include "hw/pci/pci.h"
#include "sysemu/char.h"
#include "sysemu/sysemu.h"
#include "sysemu/arch_init.h"
#include "qemu/log.h"
#include "hw/riscv/bios.h"
#include "hw/ide.h"
#include "hw/loader.h"
#include "elf.h"
#include "hw/timer/mc146818rtc.h"
#include "hw/timer/i8254.h"
#include "sysemu/blockdev.h"
#include "exec/address-spaces.h"
#include "hw/sysbus.h"             /* SysBusDevice */
#include "qemu/host-utils.h"
#include "sysemu/qtest.h"
#include "qemu/error-report.h"
#include "hw/empty_slot.h"

//#define DEBUG_BOARD_INIT

#define ENVP_ADDR		0x80002000l
#define ENVP_NB_ENTRIES	 	16
#define ENVP_ENTRY_SIZE	 	256

/* Hardware addresses */

#define MAX_IDE_BUS 2

typedef struct {
    MemoryRegion iomem;
    MemoryRegion iomem_lo; /* 0 - 0x900 */
    MemoryRegion iomem_hi; /* 0xa00 - 0x100000 */
    uint32_t leds;
    uint32_t brk;
    uint32_t gpout;
    uint32_t i2cin;
    uint32_t i2coe;
    uint32_t i2cout;
    uint32_t i2csel;
    CharDriverState *display;
    char display_text[9];
    SerialState *uart;
} BoardFPGAState;

#define TYPE_RISCV_BOARD "riscv-board"
#define RISCV_BOARD(obj) OBJECT_CHECK(BoardState, (obj), TYPE_RISCV_BOARD)

typedef struct {
    SysBusDevice parent_obj;

    qemu_irq *i8259;
} BoardState;

static ISADevice *pit;

static struct _loaderparams {
    int ram_size;
    const char *kernel_filename;
    const char *kernel_cmdline;
    const char *initrd_filename;
} loaderparams;

/* Network support */
static void network_init(PCIBus *pci_bus)
{
    int i;

    for(i = 0; i < nb_nics; i++) {
        NICInfo *nd = &nd_table[i];
        const char *default_devaddr = NULL;

        if (i == 0 && (!nd->model || strcmp(nd->model, "pcnet") == 0))
            /* The board board has a PCNet card using PCI SLOT 11 */
            default_devaddr = "0b";

        pci_nic_init_nofail(nd, pci_bus, "pcnet", default_devaddr);
    }
}

static void GCC_FMT_ATTR(3, 4) prom_set(uint32_t* prom_buf, int index,
                                        const char *string, ...)
{
    va_list ap;
    int32_t table_addr;

    if (index >= ENVP_NB_ENTRIES)
        return;

    if (string == NULL) {
        prom_buf[index] = 0;
        return;
    }

    table_addr = sizeof(int32_t) * ENVP_NB_ENTRIES + index * ENVP_ENTRY_SIZE;
    prom_buf[index] = tswap32(ENVP_ADDR + table_addr);

    va_start(ap, string);
    vsnprintf((char *)prom_buf + table_addr, ENVP_ENTRY_SIZE, string, ap);
    va_end(ap);
}

/* Kernel */
static int64_t load_kernel (void)
{
    int64_t kernel_entry, kernel_high;
    long initrd_size;
    ram_addr_t initrd_offset;
    int big_endian;
    uint32_t *prom_buf;
    long prom_size;
    int prom_index = 0;

    big_endian = 0;

    if (load_elf(loaderparams.kernel_filename, cpu_riscv_kseg0_to_phys, NULL,
                 (uint64_t *)&kernel_entry, NULL, (uint64_t *)&kernel_high,
                 big_endian, ELF_MACHINE, 1) < 0) {
        fprintf(stderr, "qemu: could not load kernel '%s'\n",
                loaderparams.kernel_filename);
        exit(1);
    }

    /* load initrd */
    initrd_size = 0;
    initrd_offset = 0;
    if (loaderparams.initrd_filename) {
        initrd_size = get_image_size (loaderparams.initrd_filename);
        if (initrd_size > 0) {
            initrd_offset = (kernel_high + ~INITRD_PAGE_MASK) & INITRD_PAGE_MASK;
            if (initrd_offset + initrd_size > ram_size) {
                fprintf(stderr,
                        "qemu: memory too small for initial ram disk '%s'\n",
                        loaderparams.initrd_filename);
                exit(1);
            }
            initrd_size = load_image_targphys(loaderparams.initrd_filename,
                                              initrd_offset,
                                              ram_size - initrd_offset);
        }
        if (initrd_size == (target_ulong) -1) {
            fprintf(stderr, "qemu: could not load initial ram disk '%s'\n",
                    loaderparams.initrd_filename);
            exit(1);
        }
    }

    /* Setup prom parameters. */
    prom_size = ENVP_NB_ENTRIES * (sizeof(int32_t) + ENVP_ENTRY_SIZE);
    prom_buf = g_malloc(prom_size);

    prom_set(prom_buf, prom_index++, "%s", loaderparams.kernel_filename);
    if (initrd_size > 0) {
        prom_set(prom_buf, prom_index++, "rd_start=0x%" PRIx64 " rd_size=%li %s",
                 cpu_riscv_phys_to_kseg0(NULL, initrd_offset), initrd_size,
                 loaderparams.kernel_cmdline);
    } else {
        prom_set(prom_buf, prom_index++, "%s", loaderparams.kernel_cmdline);
    }

    prom_set(prom_buf, prom_index++, "memsize");
    prom_set(prom_buf, prom_index++, "%i",
             MIN(loaderparams.ram_size, 256 << 20));
    prom_set(prom_buf, prom_index++, "modetty0");
    prom_set(prom_buf, prom_index++, "38400n8r");
    prom_set(prom_buf, prom_index++, NULL);

    rom_add_blob_fixed("prom", prom_buf, prom_size,
                       cpu_riscv_kseg0_to_phys(NULL, ENVP_ADDR));

    return kernel_entry;
}

static void main_cpu_reset(void *opaque)
{
    RISCVCPU *cpu = opaque;
    cpu_reset(CPU(cpu));
}

static void cpu_request_exit(void *opaque, int irq, int level)
{
    CPUState *cpu = current_cpu;

    if (cpu && level) {
        cpu_exit(cpu);
    }
}

static
void riscv_board_init(QEMUMachineInitArgs *args)
{
    ram_addr_t ram_size = args->ram_size;
    const char *cpu_model = args->cpu_model;
    const char *kernel_filename = args->kernel_filename;
    const char *kernel_cmdline = args->kernel_cmdline;
    const char *initrd_filename = args->initrd_filename;
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *main_mem = g_new(MemoryRegion, 1);
    const size_t smbus_eeprom_size = 8 * 256;
    uint8_t *smbus_eeprom_buf = g_malloc0(smbus_eeprom_size);
    PCIBus *pci_bus;
    ISABus *isa_bus;
    RISCVCPU *cpu;
    CPURISCVState *env;
    qemu_irq *isa_irq;
    qemu_irq *cpu_exit_irq;
    int piix4_devfn;
    I2CBus *smbus;
    int i;
    DriveInfo *hd[MAX_IDE_BUS * MAX_IDE_DEVS];
    DriveInfo *fd[MAX_FD];

    DeviceState *dev = qdev_create(NULL, TYPE_RISCV_BOARD);
    BoardState *s = RISCV_BOARD(dev);

    /* The whole address space decoded by the GT-64120A doesn't generate
       exception when accessing invalid memory. Create an empty slot to
       emulate this feature. */
//    empty_slot_init(0, 0x20000000);

    object_property_set_bool(OBJECT(dev), true, "realized", NULL);

    /* Make sure the first 3 serial ports are associated with a device. */
    for(i = 0; i < 3; i++) {
        if (!serial_hds[i]) {
            char label[32];
            snprintf(label, sizeof(label), "serial%d", i);
            serial_hds[i] = qemu_chr_new(label, "null", NULL);
        }
    }

    /* init CPUs */
    if (cpu_model == NULL) {
        cpu_model = "riscv-generic";
    }

    for (i = 0; i < smp_cpus; i++) {
        cpu = cpu_riscv_init(cpu_model);
        if (cpu == NULL) {
            fprintf(stderr, "Unable to find CPU definition\n");
            exit(1);
        }
        env = &cpu->env;

        /* Init internal devices */
        cpu_riscv_irq_init_cpu(env);
        cpu_riscv_clock_init(env);
        qemu_register_reset(main_cpu_reset, cpu);
    }
    cpu = RISCV_CPU(first_cpu);
    env = &cpu->env;

    /* allocate RAM */
    if (ram_size > (2048u << 20)) {
        fprintf(stderr,
                "qemu: Too much memory for this machine: %d MB, maximum 2048 MB\n",
                ((unsigned int)ram_size / (1 << 20)));
        exit(1);
    }

    /* register system main memory (actual RAM) */
    memory_region_init_ram(main_mem, NULL, "riscv_board.ram", ram_size);
    vmstate_register_ram_global(main_mem);
    memory_region_add_subregion(system_memory, 0x0, main_mem);

    if (kernel_filename) {
        /* Write a small bootloader to the flash location. */
        loaderparams.ram_size = ram_size;
        loaderparams.kernel_filename = kernel_filename;
        loaderparams.kernel_cmdline = kernel_cmdline;
        loaderparams.initrd_filename = initrd_filename;
        load_kernel();
    }

    // write memory amount in MiB to 0x0
    stl_p(memory_region_get_ram_ptr(main_mem), loaderparams.ram_size >> 20);

    // add serial device 0x3f8-0x3ff
    serial_mm_init(system_memory, 0x3f8, 0, env->irq[4], 1843200/16, serial_hds[0], 
            DEVICE_NATIVE_ENDIAN);

    // add htif device 0x400 - 0x410
    htif_mm_init(system_memory, 0x400, env->irq[0]);

    /* Init internal devices */
    cpu_riscv_irq_init_cpu(env);
    cpu_riscv_clock_init(env);

    /*
     * We have a circular dependency problem: pci_bus depends on isa_irq,
     * isa_irq is provided by i8259, i8259 depends on ISA, ISA depends
     * on piix4, and piix4 depends on pci_bus.  To stop the cycle we have
     * qemu_irq_proxy() adds an extra bit of indirection, allowing us
     * to resolve the isa_irq -> i8259 dependency after i8259 is initialized.
     */
    isa_irq = qemu_irq_proxy(&s->i8259, 16);

    /* Northbridge */
    pci_bus = gt64120_register(isa_irq);

    /* Southbridge */
    ide_drive_get(hd, MAX_IDE_BUS);

    piix4_devfn = piix4_init(pci_bus, &isa_bus, 80);

    /* Interrupt controller */
    /* The 8259 is attached to the RISCV CPU INT0 pin, ie interrupt 2 */
    s->i8259 = i8259_init(isa_bus, env->irq[2]);

    isa_bus_irqs(isa_bus, s->i8259);
    pci_piix4_ide_init(pci_bus, hd, piix4_devfn + 1);
    pci_create_simple(pci_bus, piix4_devfn + 2, "piix4-usb-uhci");
    smbus = piix4_pm_init(pci_bus, piix4_devfn + 3, 0x1100,
                          isa_get_irq(NULL, 9), NULL, 0, NULL);
    smbus_eeprom_init(smbus, 8, smbus_eeprom_buf, smbus_eeprom_size);
    g_free(smbus_eeprom_buf);
    pit = pit_init(isa_bus, 0x40, 0, NULL);
    cpu_exit_irq = qemu_allocate_irqs(cpu_request_exit, NULL, 1);
    DMA_init(0, cpu_exit_irq);

    /* Super I/O */
    isa_create_simple(isa_bus, "i8042");

    rtc_init(isa_bus, 2000, NULL);
//    serial_isa_init(isa_bus, 0, serial_hds[0]);
//    serial_isa_init(isa_bus, 1, serial_hds[1]);
    if (parallel_hds[0])
        parallel_init(isa_bus, 0, parallel_hds[0]);
    for(i = 0; i < MAX_FD; i++) {
        fd[i] = drive_get(IF_FLOPPY, 0, i);
    }
    fdctrl_init_isa(isa_bus, fd);

    /* Network card */
    network_init(pci_bus);

    /* Optional PCI video card */
    pci_vga_init(pci_bus);
}

static int riscv_board_sysbus_device_init(SysBusDevice *sysbusdev)
{
    return 0;
}

static void riscv_board_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = riscv_board_sysbus_device_init;
}

static const TypeInfo riscv_board_device = {
    .name          = TYPE_RISCV_BOARD,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BoardState),
    .class_init    = riscv_board_class_init,
};

static QEMUMachine riscv_board_machine = {
    .name = "board",
    .desc = "RISCV Board",
    .init = riscv_board_init,
    .max_cpus = 16,
    .is_default = 1,
};

static void riscv_board_register_types(void)
{
    type_register_static(&riscv_board_device);
}

static void riscv_board_machine_init(void)
{
    qemu_register_machine(&riscv_board_machine);
}

type_init(riscv_board_register_types)
machine_init(riscv_board_machine_init);
