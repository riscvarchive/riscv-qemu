/*
 * QEMU RISC-V "Unicorn" U54mc compatible board
 * Author: Ivan Griffin, ivan.griffin@emdalo.com
 *         Daire McNamara, daire.mcnamara@emdalo.com
 *
 * based on QEMU RISC-V SiFive U500 SDK Compatible Board
 * Author: Sagar Karandikar, sagark@eecs.berkeley.edu
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

#include "qemu/osdep.h"
#include "hw/riscv/riscv_clint.h"
#include "hw/riscv/riscv_l2cache.h"
#include "hw/riscv/riscv_plic.h"
#include "hw/riscv/riscv_pmp.h"
#include "hw/riscv/riscv_rtc.h"
#include "hw/riscv/riscv_tim.h"
#include "hw/riscv/riscv_uart.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/riscv/soc.h"
#include "hw/boards.h"
#include "hw/riscv/cpudevs.h"
#include "sysemu/arch_init.h"
#include "hw/loader.h"
#include "elf.h"
#include "exec/address-spaces.h"
#include "hw/sysbus.h"             /* SysBusDevice */

#define TYPE_RISCV_UNICORN_BOARD "unicorn_board"
#define RISCV_UNICORN_BOARD(obj) OBJECT_CHECK(BoardState, (obj), \
    TYPE_RISCV_UNICORN_BOARD)

typedef struct {
    SysBusDevice parent_obj;
} BoardState;

static struct _loaderparams {
    int ram_size;
    const char *kernel_filename;
    const char *kernel_cmdline;
    const char *initrd_filename;
} loaderparams;

static uint64_t identity_translate(void *opaque, uint64_t addr)
{
    return addr;
}

static int64_t load_kernel(void)
{
    int64_t kernel_entry, kernel_high;
    int big_endian;
    big_endian = 0;

    if (load_elf(loaderparams.kernel_filename, identity_translate, NULL,
                 (uint64_t *)&kernel_entry, NULL, (uint64_t *)&kernel_high,
                 big_endian, ELF_MACHINE, 1, 0) < 0) {
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


static void riscv_unicorn_board_init(MachineState *args)
{
    fprintf(stderr, "setting up UNICORN board for %lx bytes of RAM\n",
        args->ram_size);

    ram_addr_t ram_size = args->ram_size;
    const char *kernel_filename = args->kernel_filename;
    const char *kernel_cmdline = args->kernel_cmdline;
    const char *initrd_filename = args->initrd_filename;
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *main_mem = g_new(MemoryRegion, 1);
    MemoryRegion *boot_rom = g_new(MemoryRegion, 1);
    /* MemoryRegion *uncached_ram = g_new(MemoryRegion, 1); */
    RISCVCPU *cpu;
    CPURISCVState *env;
    int i;
    DeviceState *dev = qdev_create(NULL, TYPE_RISCV_UNICORN_BOARD);
    object_property_set_bool(OBJECT(dev), true, "realized", NULL);

    /* init CPUs */
    if (args->cpu_model != NULL) {
        fprintf(stderr, "WARNING:: CPUs are automatically setup as either E51"
           " or U54 for UNICORN.\n");
    }

    /* register RAM */
    memory_region_init_ram(main_mem, NULL, "riscv_unicorn_board.ram",
                           ram_size, &error_fatal);
    /* for phys mem size check in page table walk */
    vmstate_register_ram_global(main_mem);
    memory_region_add_subregion(system_memory, DRAM_BASE, main_mem);

    /*
     * TODO: EMDALO: Check if UNICORN has an uncached alias of DRAM_BASE
     *       or not?
    memory_region_init_alias(uncached_ram, NULL, "riscv_unicorn_board.uncached",
                             main_mem, offset, size);
     */

    /*
     * Create the zero device
     */
    l2zero_dev_init(L2ZERO_DEV_BASE_ADDR, L2ZERO_DEV_REG_SZ);

    hart_shared_init(smp_cpus);

    for (i = 0; i < smp_cpus; i++) {
        cpu = cpu_riscv_init(i ? "U54" : "E51");
        if (cpu == NULL) {
            fprintf(stderr, "Unable to find CPU definition\n");
            exit(1);
        } else {
            fprintf(stderr, "initializing CPU %d as type >>%s<<\n", i,
                i ? "U54" : "E51");
        }
        env = &cpu->env;
        env->hart_index = i;
        env->num_harts = smp_cpus;
        env->memsize = ram_size;
        hart_add_env_and_state(i, env, &(cpu->parent_obj));

        /* Init ITIMs and DTIMs
           Note hart 0 DTIM/ITIM differs from other harts */
        if (i == 0) {
            dtim_init(cpu, HART0_DTIM, DEFAULT_DTIM_SZ, DTIM_WAY_SZ);
            itim_init(cpu, HART0_ITIM, HART0_ITIM_SZ, DTIM_WAY_SZ);
        } else {
            itim_init(cpu, HART0_ITIM + (TIM_BLOCK_SZ * i), DEFAULT_ITIM_SZ,
                ITIM_WAY_SZ);
        }
        /* Init internal devices */
        cpu_riscv_irq_init_cpu(env);
        cpu_riscv_clock_init(env);
        qemu_register_reset(main_cpu_reset, cpu);
    }
    cpu = RISCV_CPU(first_cpu);
    env = &cpu->env;

    /* boot rom */
    memory_region_init_ram(boot_rom, NULL, "riscv_unicorn_board.bootrom",
                           0x10000, &error_fatal);
    vmstate_register_ram_global(boot_rom);
    memory_region_set_readonly(boot_rom, true);
    memory_region_add_subregion(system_memory, 0x0, boot_rom);

    clint_init(cpu, smp_cpus);
    plic_init(cpu);
    l2cache_init(cpu, 4, 16, 512, 64);
    pmp_init();
    riscv_serial_init(UART0_BASE_ADDR, plic_raise_irq, plic_lower_irq,
        UART0_PLIC_SRC, 115200, 4, serial_hds[0], get_system_memory());

    if (kernel_filename) {
        loaderparams.ram_size = ram_size;
        loaderparams.kernel_filename = kernel_filename;
        loaderparams.kernel_cmdline = kernel_cmdline;
        loaderparams.initrd_filename = initrd_filename;
        load_kernel();
    }

    uint32_t reset_vec[8] = {
        0x297 + 0x80000000 - 0x1000, /* reset vector */
        0x00028067,                  /* jump to DRAM_BASE */
        0x00000000,                  /* reserved */
        0x0,                         /* config string pointer */
        0, 0, 0, 0                   /* trap vector */
    };
    reset_vec[3] = 0x1000 + sizeof(reset_vec); /* config string pointer */

    /* part one of config string - before memory size specified */
    const char *config_string1 = "platform {\n"
        "  vendor ucb;\n"
        "  arch spike;\n"
        "};\n"
        "plic {\n"
        "  interface \"plic\";\n"
        "  ndevs 2;\n"
        "  priority { mem { 0x60000000 0x60000fff; }; };\n"
        "  pending  { mem { 0x60001000 0x6000107f; }; };\n"
        "  0 {\n"
        "    0 {\n"
        "      m {\n"
        "        ie  { mem { 0x60002000 0x6000207f; }; };\n"
        "        ctl { mem { 0x60200000 0x60200007; }; };\n"
        "      };\n"
        "      s {\n"
        "        ie  { mem { 0x60002080 0x600020ff; }; };\n"
        "        ctl { mem { 0x60201000 0x60201007; }; };\n"
        "      };\n"
        "    }\n"
        "  };\n"
        "};\n"
        "rtc {\n"
        "  addr 0x" "40000000" ";\n"
        "};\n"
        "uart {\n"
        "  addr 0x40002000;\n"
        "};\n"
        "ram {\n"
        "  0 {\n"
        "    addr 0x" "80000000" ";\n"
        "    size 0x";


    /* part two of config string - after memory size specified */
    const char *config_string2 =  ";\n"
        "  };\n"
        "};\n"
        "core {\n"
        "  0" " {\n"
          "    " "0 {\n"
          "      isa " "rv64imafd" ";\n"
          "      timecmp 0x" "40000008" ";\n"
          "      ipi 0x" "40001000" ";\n" /* this must match dummy ipi region
                                           * above */
          "    };\n"
          "  };\n"
          "};\n";

    /* build config string with supplied memory size */
    uint64_t rsz = ram_size;
    char *ramsize_as_hex_str = malloc(17);
    sprintf(ramsize_as_hex_str, "%016" PRIx64, rsz);
    char *config_string = malloc(strlen(config_string1)
        + strlen(ramsize_as_hex_str) + strlen(config_string2) + 1);
    config_string[0] = 0;
    strcat(config_string, config_string1);
    strcat(config_string, ramsize_as_hex_str);
    strcat(config_string, config_string2);

    /* copy in the reset vec and configstring */
    int q;
    for (q = 0; q < sizeof(reset_vec) / sizeof(reset_vec[0]); q++) {
        stl_p(memory_region_get_ram_ptr(boot_rom) + 0x1000 + q * 4,
              reset_vec[q]);
    }

    int confstrlen = strlen(config_string);
    for (q = 0; q < confstrlen; q++) {
        stb_p(memory_region_get_ram_ptr(boot_rom) + reset_vec[3] + q,
              config_string[q]);
    }

    timer_mm_init(system_memory, CLINT_BASE_ADDR + CLINT_MTIMECMP_OFFSET,
       env);
}

static int riscv_unicorn_board_sysbus_device_init(SysBusDevice *sysbusdev)
{
    return 0;
}

static void riscv_unicorn_board_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);
    k->init = riscv_unicorn_board_sysbus_device_init;
}

static const TypeInfo riscv_unicorn_board_device = {
    .name          = TYPE_RISCV_UNICORN_BOARD,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BoardState),
    .class_init    = riscv_unicorn_board_class_init,
};

static void riscv_unicorn_board_machine_init(MachineClass *mc)
{
    fprintf(stderr, "setting up UNICORN machine\n");

    mc->desc = "RISC-V Board compatible with Unicorn (incomplete)";
    mc->init = riscv_unicorn_board_init;
    mc->max_cpus = 5;
}

DEFINE_MACHINE("unicorn", riscv_unicorn_board_machine_init)

static void riscv_unicorn_board_register_types(void)
{
    type_register_static(&riscv_unicorn_board_device);
}

type_init(riscv_unicorn_board_register_types);
