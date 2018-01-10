/*
 * SiFive UART interface
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

#ifndef HW_SIFIVE_UART_H
#define HW_SIFIVE_UART_H

enum {
    SIFIVE_UART_TXFIFO        = 0,
    SIFIVE_UART_RXFIFO        = 4,
    SIFIVE_UART_TXCTRL        = 8,
    SIFIVE_UART_TXMARK        = 10,
    SIFIVE_UART_RXCTRL        = 12,
    SIFIVE_UART_RXMARK        = 14,
    SIFIVE_UART_IE            = 16,
    SIFIVE_UART_IP            = 20,
    SIFIVE_UART_DIV           = 24,
    SIFIVE_UART_MAX           = 32
};

enum {
    SIFIVE_UART_IE_TXWM       = 1, /* Transmit watermark interrupt enable */
    SIFIVE_UART_IE_RXWM       = 2  /* Receive watermark interrupt enable */
};

enum {
    SIFIVE_UART_IP_TXWM       = 1, /* Transmit watermark interrupt pending */
    SIFIVE_UART_IP_RXWM       = 2  /* Receive watermark interrupt pending */
};

#define TYPE_SIFIVE_UART "riscv.sifive.uart"

#define SIFIVE_UART(obj) \
    OBJECT_CHECK(SiFiveUARTState, (obj), TYPE_SIFIVE_UART)

typedef struct SiFiveUARTState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    qemu_irq irq;
    MemoryRegion mmio;
    CharBackend chr;
    uint8_t rx_fifo[8];
    unsigned int rx_fifo_len;
    uint32_t ie;
    uint32_t ip;
    uint32_t txctrl;
    uint32_t rxctrl;
    uint32_t div;
} SiFiveUARTState;

SiFiveUARTState *sifive_uart_create(MemoryRegion *address_space, hwaddr base,
    Chardev *chr, qemu_irq irq);

#endif
