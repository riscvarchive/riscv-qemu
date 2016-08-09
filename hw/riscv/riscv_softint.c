/*
 * QEMU RISC-V Soft Interrupt Emulation
 *
 * Author: Sagar Karandikar, sagark@eecs.berkeley.edu
 *
 * This module provides shim devices that allow support for interrupts
 * triggered by the RISC-V processor itself (writes to the MIP/SIP CSRs):
 *
 * The following instantiations are enabled by default in riscv_board:
 *
 * softint0 - SSIP
 * softint1 - STIP
 * softint2 - MSIP
 *
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

#include "hw/riscv/riscv_softint.h"
#include "exec/address-spaces.h"
#include "qemu/error-report.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <inttypes.h>


static void softint_pre_save(void *opaque)
{
    return;
}

static int softint_post_load(void *opaque, int version_id)
{
    return 0;
}

const VMStateDescription vmstate_softint = {
    .name = "softint",
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = softint_pre_save,
    .post_load = softint_post_load,
    .fields      = (VMStateField []) {
        VMSTATE_END_OF_LIST()
    },
};

// CPU wants to read an Softint register. Should not happen.
static uint64_t softint_mm_read(void *opaque, hwaddr addr, unsigned size)
{
    fprintf(stderr, "Unimplemented read softint\n");
    exit(1);
}

// CPU wrote to an Softint register
static void softint_mm_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
    SoftintState *softintstate = opaque;

    if (addr == 0x0) {
        if (value != 0) {
            qemu_irq_raise(softintstate->irq);
        } else {
            qemu_irq_lower(softintstate->irq);
        }
    } else {
        fprintf(stderr, "Invalid softint register address %016lx\n", (uint64_t)addr);
        exit(1);
    }
}

static const MemoryRegionOps softint_mm_ops[3] = {
    [DEVICE_LITTLE_ENDIAN] = {
        .read = softint_mm_read,
        .write = softint_mm_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
    },
};

SoftintState *softint_mm_init(MemoryRegion *address_space, hwaddr base, qemu_irq irq,
                        MemoryRegion *main_mem, CPURISCVState *env, const char * name)
{
    // TODO: cleanup the constant buffer sizes
    SoftintState *softintstate;

    softintstate = g_malloc0(sizeof(SoftintState));
    softintstate->irq = irq;
    softintstate->address_space = address_space;
    softintstate->env = env;

    char * badbuf = g_malloc0(sizeof(char)*100);
    sprintf(badbuf, "%s%s", "softint", name);
    softintstate->name = badbuf;

    vmstate_register(NULL, base, &vmstate_softint, softintstate);

    memory_region_init_io(&softintstate->io, NULL,
            &softint_mm_ops[DEVICE_LITTLE_ENDIAN],
            softintstate, badbuf, 4 /* 1 32-bit register */);
    memory_region_add_subregion(address_space, base, &softintstate->io);

    return softintstate;
}
