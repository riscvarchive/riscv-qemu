/*
 * QEMU RISC-V CLINT device emulator
 * Author: Daire McNamara, daire.mcnamara@emdalo.com
 *
 * This provides a RISC-V CLINT device:
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
#include "hw/riscv/riscv_hart.h"
#include "hw/riscv/soc.h"
#include "exec/address-spaces.h"

/* #define DEBUG_CLINT */

#ifdef DEBUG_CLINT
#define CLINT_DEBUG(fmt, ...) \
    do { \
        fprintf(stderr, "clint: " fmt, ## __VA_ARGS__);\
    } while (0)
#else
#define CLINT_DEBUG(fmt, ...) \
    do {} while (0)
#endif

#define CLINT_WARN(fmt, ...) \
    do { \
        fprintf(stderr, "clint: WARNING " fmt, ## __VA_ARGS__);\
    } while (0)

#define CLINT_ERR(fmt, ...) \
    do { \
        fprintf(STDERR, "CLINT: ERROR " fmt, ## __VA_ARGS__); exit(-1);\
    } while (0)

static uint64_t clint_read(void *opaque, hwaddr addr, unsigned int size)
{
    return 0;
}

static void clint_write(void *opaque, hwaddr addr, uint64_t val64,
    unsigned int size)
{
    /* set correct value for MSIP */
    CPURISCVState *env = 0;
    uint64_t hart_index = addr >> 2;

    env = hart_get_env(hart_index);

    if (env == 0) {
        CLINT_WARN("hart%ld doesn't exist\n", hart_index);
        return;
    }

    if (val64 == 0) {
        CLINT_DEBUG("clearing hart%d interrupt\n", env->hart_index);
        env->mip &= ~MIP_MSIP;
        return;
    }

    CLINT_DEBUG("interrupting hart%d\n", env->hart_index);

    /* Set MSIP bit in MIP register
      * and MIE in MSTATUS register */
    env->mip |= MIP_MSIP;

    CPUState *cs = hart_get_cpu_state(hart_index);
    cs->interrupt_request |= CPU_INTERRUPT_HARD;
}


static const MemoryRegionOps clint_ops = {
    .read = clint_read,
    .write = clint_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8
    }
};

void clint_init(RISCVCPU *cpu, uint32_t num_harts)
{
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *clint_io = g_new(MemoryRegion, 1);
    CLINT_DEBUG("registering CLINT as MMIO region\n");
    memory_region_init_io(clint_io, NULL, &clint_ops, cpu, "riscv_soc.clint",
        sizeof(uint32_t) * num_harts);
    memory_region_add_subregion(system_memory, CLINT_BASE_ADDR, clint_io);
}

