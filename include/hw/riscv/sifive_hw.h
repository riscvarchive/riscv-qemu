/*
 * SiFive Hardware definitions
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

#ifndef HW_SIFIVE_HW_H
#define HW_SIFIVE_HW_H

typedef struct SiFiveMemmapEntry {
    hwaddr base;
    hwaddr size;    
} SiFiveMemmapEntry;

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
	SIFIVE_SIP_BASE     = 0x0,
	SIFIVE_TIMECMP_BASE = 0x4000,
	SIFIVE_TIME_BASE    = 0xBFF8
};

#define TYPE_SIFIVE_E300 "riscv.sifive.e300"
#define TYPE_SIFIVE_U500 "riscv.sifive_u500"
#define TYPE_SIFIVE_UART "riscv.sifive.uart"
#define TYPE_SIFIVE_PRCI "riscv.sifive.prci"

#define SIFIVE_E300(obj) \
    OBJECT_CHECK(SiFiveE300State, (obj), TYPE_SIFIVE_E300)
#define SIFIVE_U500(obj) \
    OBJECT_CHECK(SiFiveU500State, (obj), TYPE_SIFIVE_U500)
#define SIFIVE_UART(obj) \
    OBJECT_CHECK(SiFiveUARTState, (obj), TYPE_SIFIVE_UART)
#define SIFIVE_PRCI(obj) \
    OBJECT_CHECK(SiFivePRCIState, (obj), TYPE_SIFIVE_PRCI)

typedef struct SiFiveE300State {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    RISCVHartArrayState soc;
} SiFiveE300State;

typedef struct SiFiveU500State {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    RISCVHartArrayState soc;
    void *fdt;
    int fdt_size;
} SiFiveU500State;

typedef struct SiFiveUARTState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion mmio;
    CharDriverState *chr;
    uint8_t rx_fifo[8];
    unsigned int rx_fifo_len;
    uint32_t txctrl;
    uint32_t rxctrl;
    uint32_t div;
} SiFiveUARTState;

typedef struct SiFivePRCIState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion mmio;
} SiFivePRCIState;

DeviceState *sifive_prci_create(hwaddr addr);
DeviceState *sifive_uart_create(hwaddr addr, CharDriverState *chr);

#endif
