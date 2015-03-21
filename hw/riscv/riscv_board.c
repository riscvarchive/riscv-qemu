/* 
 *  QEMU RISC-V sample-board support
 *
 *  Author: Sagar Karandikar, skarandikar@berkeley.edu
 *  Originally based on hw/mips/mips_malta.c
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
#include "block/block_int.h" // move later
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

#define TYPE_RISCV_BOARD "riscv-board"
#define RISCV_BOARD(obj) OBJECT_CHECK(BoardState, (obj), TYPE_RISCV_BOARD)

typedef struct {
    SysBusDevice parent_obj;
} BoardState;

static struct _loaderparams {
    int ram_size;
    const char *kernel_filename;
    const char *kernel_cmdline;
    const char *initrd_filename;
} loaderparams;

uint64_t identity_translate(void *opaque, uint64_t addr)
{
    return addr;
}

static int64_t load_kernel (void)
{
    int64_t kernel_entry, kernel_high;
    int big_endian;
    big_endian = 0;

    if (load_elf(loaderparams.kernel_filename, identity_translate, NULL,
                 (uint64_t *)&kernel_entry, NULL, (uint64_t *)&kernel_high,
                 big_endian, ELF_MACHINE, 1) < 0) {
        fprintf(stderr, "qemu: could not load kernel '%s'\n",
                loaderparams.kernel_filename);
        exit(1);
    }
    return kernel_entry;
}

static void main_cpu_reset(void *opaque)
{
    RISCVCPU *cpu = opaque;
    cpu_reset(CPU(cpu));
}

static void riscv_board_init(QEMUMachineInitArgs *args)
{
    ram_addr_t ram_size = args->ram_size;
    const char *cpu_model = args->cpu_model;
    const char *kernel_filename = args->kernel_filename;
    const char *kernel_cmdline = args->kernel_cmdline;
    const char *initrd_filename = args->initrd_filename;
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *main_mem = g_new(MemoryRegion, 1);
    RISCVCPU *cpu;
    CPURISCVState *env;
    int i;
#ifdef CONFIG_RISCV_HTIF
    DriveInfo *htifbd_drive;
    char *htifbd_fname; // htif block device filename
#endif

    DeviceState *dev = qdev_create(NULL, TYPE_RISCV_BOARD);

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

    /* register system main memory (actual RAM) */
    memory_region_init_ram(main_mem, NULL, "riscv_board.ram", ram_size);
    vmstate_register_ram_global(main_mem);
    memory_region_add_subregion(system_memory, 0x0, main_mem);

    if (bios_name) {
        int bios_size;
        MemoryRegion *bios;
        bios_size = get_image_size(bios_name);
	if (bios_size > 0) {
            bios = g_malloc(sizeof(*bios));
            memory_region_init_ram(bios, NULL, "riscv.fw", bios_size);
            vmstate_register_ram_global(bios);
            memory_region_set_readonly(bios, true);
            if (rom_add_file_fixed(bios_name, 0, -1) != 0) {
                fprintf(stderr, "qemu: could not load RISC-V firmware '%s'\n", bios_name);
                exit(1);
            }
	}
    }

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

#ifdef CONFIG_RISCV_HTIF
    // setup HTIF Block Device if one is specified as -hda FILENAME
    htifbd_drive = drive_get_by_index(IF_IDE, 0);
    if (NULL == htifbd_drive) {
        htifbd_fname = NULL;
    } else {
        htifbd_fname = (*(htifbd_drive->bdrv)).filename;
    }

    // add htif device 0x400 - 0x410
    htif_mm_init(system_memory, 0x400, env->irq[0], main_mem, htifbd_fname);
#else
    /* Create MMIO transports, to which virtio backends created by the
     * user are automatically connected as needed.  If no backend is
     * present, the transport simply remains harmlessly idle.
     * Each memory-mapped region is 0x200 bytes in size.
     */
    sysbus_create_simple("virtio-mmio", 0x400, env->irq[1]);
    sysbus_create_simple("virtio-mmio", 0x600, env->irq[2]);
    sysbus_create_simple("virtio-mmio", 0x800, env->irq[3]);
#endif

    /* Init internal devices */
    cpu_riscv_irq_init_cpu(env);
    cpu_riscv_clock_init(env);
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
    .max_cpus = 1,
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
