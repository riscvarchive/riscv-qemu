/*
 * QEMU RISCV Host Target Interface (HTIF) Emulation
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
#ifndef HW_RISCV_HTIF_H
#define HW_RISCV_HTIF_H 1

#include "hw/hw.h"
#include "sysemu/sysemu.h"
#include "exec/memory.h"

typedef struct HTIFState HTIFState;

struct HTIFState {
    uint64_t tohost; // mapped to address base passed into htif_mm_init
    uint64_t fromhost; // mapped to address base + 0x8 passed into htif_mm_init
    hwaddr tohost_addr;
    hwaddr fromhost_addr;
    qemu_irq irq; // host interrupt line
    MemoryRegion io;
    MemoryRegion* address_space;
    MemoryRegion* main_mem;
    void* main_mem_ram_ptr;

    int block_dev_present;
    // TODO: eventually move the following to a separate HTIF block device driver
    const char *block_fname;
    int block_fd;
    char *real_name;

};

typedef struct request_t request_t;

struct request_t
{ 
    uint64_t addr;
    uint64_t offset;
    uint64_t size;
    uint64_t tag;
};

extern const VMStateDescription vmstate_htif;
extern const MemoryRegionOps htif_io_ops;

/* legacy pre qom */
HTIFState *htif_mm_init(MemoryRegion *address_space, hwaddr base, 
                    qemu_irq irq, MemoryRegion *main_mem, char *htifbd_fname);

#endif
