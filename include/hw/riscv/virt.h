/*
 * SiFive VirtIO Board
 *
 * Copyright (c) 2017 SiFive, Inc.
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

#ifndef HW_VIRT_H
#define HW_VIRT_H

#define TYPE_RISCV_VIRT_BOARD "riscv.virt"
#define VIRT(obj) \
    OBJECT_CHECK(RISCVVirtState, (obj), TYPE_RISCV_VIRT_BOARD)

enum { ROM_BASE = 0x1000 };

typedef struct {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    RISCVHartArrayState soc;
    DeviceState *plic;
    void *fdt;
    int fdt_size;
} RISCVVirtState;

enum {
    VIRT_DEBUG,
    VIRT_MROM,
    VIRT_TEST,
    VIRT_CLINT,
    VIRT_PLIC,
    VIRT_UART0,
    VIRT_VIRTIO,
    VIRT_DRAM
};


enum {
    UART0_IRQ = 10,
    VIRTIO_IRQ = 1, /* 1 to 8 */
    VIRTIO_COUNT = 8,
    VIRTIO_NDEV = 10
};

#define VIRT_PLIC_HART_CONFIG "MS"
#define VIRT_PLIC_NUM_SOURCES 127
#define VIRT_PLIC_NUM_PRIORITIES 7
#define VIRT_PLIC_PRIORITY_BASE 0x0
#define VIRT_PLIC_PENDING_BASE 0x1000
#define VIRT_PLIC_ENABLE_BASE 0x2000
#define VIRT_PLIC_ENABLE_STRIDE 0x80
#define VIRT_PLIC_CONTEXT_BASE 0x200000
#define VIRT_PLIC_CONTEXT_STRIDE 0x1000

#endif
