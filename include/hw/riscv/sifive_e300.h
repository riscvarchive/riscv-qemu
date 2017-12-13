/*
 * SiFive E300 series machine interface
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

#ifndef HW_SIFIVE_E300_H
#define HW_SIFIVE_E300_H

#define TYPE_SIFIVE_E300 "riscv.sifive.e300"

#define SIFIVE_E300(obj) \
    OBJECT_CHECK(SiFiveE300State, (obj), TYPE_SIFIVE_E300)

typedef struct SiFiveE300State {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    RISCVHartArrayState soc;
    DeviceState *plic;
} SiFiveE300State;

enum {
    SIFIVE_E300_DEBUG,
    SIFIVE_E300_MROM,
    SIFIVE_E300_OTP,
    SIFIVE_E300_CLINT,
    SIFIVE_E300_PLIC,
    SIFIVE_E300_AON,
    SIFIVE_E300_PRCI,
    SIFIVE_E300_OTP_CTRL,
    SIFIVE_E300_GPIO0,
    SIFIVE_E300_UART0,
    SIFIVE_E300_QSPI0,
    SIFIVE_E300_PWM0,
    SIFIVE_E300_UART1,
    SIFIVE_E300_QSPI1,
    SIFIVE_E300_PWM1,
    SIFIVE_E300_QSPI2,
    SIFIVE_E300_PWM2,
    SIFIVE_E300_XIP,
    SIFIVE_E300_DTIM
};

enum {
    SIFIVE_E300_UART0_IRQ = 3,
    SIFIVE_E300_UART1_IRQ = 4
};

#define SIFIVE_E300_PLIC_HART_CONFIG "M"
#define SIFIVE_E300_PLIC_NUM_SOURCES 127
#define SIFIVE_E300_PLIC_NUM_PRIORITIES 7
#define SIFIVE_E300_PLIC_PRIORITY_BASE 0x0
#define SIFIVE_E300_PLIC_PENDING_BASE 0x1000
#define SIFIVE_E300_PLIC_ENABLE_BASE 0x2000
#define SIFIVE_E300_PLIC_ENABLE_STRIDE 0x80
#define SIFIVE_E300_PLIC_CONTEXT_BASE 0x200000
#define SIFIVE_E300_PLIC_CONTEXT_STRIDE 0x1000

#endif
