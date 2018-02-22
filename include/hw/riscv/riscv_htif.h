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
#define HW_RISCV_HTIF_H

#include "hw/hw.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include "sysemu/sysemu.h"
#include "exec/memory.h"
#include "target/riscv/cpu.h"

#define TYPE_HTIF_UART "riscv.htif.uart"

typedef struct HTIFState {
    int allow_tohost;
    int fromhost_inprogress;

    hwaddr tohost_offset;
    hwaddr fromhost_offset;
    uint64_t tohost_size;
    uint64_t fromhost_size;
    MemoryRegion mmio;
    MemoryRegion *address_space;
    MemoryRegion *main_mem;
    void *main_mem_ram_ptr;

    CPURISCVState *env;
    CharBackend chr;
    uint64_t pending_read;
} HTIFState;

extern const VMStateDescription vmstate_htif;
extern const MemoryRegionOps htif_io_ops;

/* HTIF symbol callback */
void htif_symbol_callback(const char *st_name, int st_info, uint64_t st_value,
    uint64_t st_size);

/* legacy pre qom */
HTIFState *htif_mm_init(MemoryRegion *address_space, MemoryRegion *main_mem,
    CPURISCVState *env, Chardev *chr);

#endif
