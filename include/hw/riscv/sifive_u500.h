/*
 * SiFive U500 series machine interface
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

#ifndef HW_SIFIVE_U500_H
#define HW_SIFIVE_U500_H

#define TYPE_SIFIVE_U500 "riscv.sifive_u500"

#define SIFIVE_U500(obj) \
    OBJECT_CHECK(SiFiveU500State, (obj), TYPE_SIFIVE_U500)

typedef struct SiFiveU500State {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    RISCVHartArrayState soc;
    DeviceState *plic;
    void *fdt;
    int fdt_size;
} SiFiveU500State;

enum {
    SIFIVE_U500_DEBUG,
    SIFIVE_U500_MROM,
    SIFIVE_U500_CLINT,
    SIFIVE_U500_PLIC,
    SIFIVE_U500_UART0,
    SIFIVE_U500_UART1,
    SIFIVE_U500_DRAM
};

enum {
    SIFIVE_U500_UART0_IRQ = 3,
    SIFIVE_U500_UART1_IRQ = 4
};

#define SIFIVE_U500_PLIC_HART_CONFIG "MS"
#define SIFIVE_U500_PLIC_NUM_SOURCES 127
#define SIFIVE_U500_PLIC_NUM_PRIORITIES 7
#define SIFIVE_U500_PLIC_PRIORITY_BASE 0x0
#define SIFIVE_U500_PLIC_PENDING_BASE 0x1000
#define SIFIVE_U500_PLIC_ENABLE_BASE 0x2000
#define SIFIVE_U500_PLIC_ENABLE_STRIDE 0x80
#define SIFIVE_U500_PLIC_CONTEXT_BASE 0x200000
#define SIFIVE_U500_PLIC_CONTEXT_STRIDE 0x1000

#endif
