/*
 * QEMU 16550A UART emulation, modified for RISC-V "Unicorn" board
 *
 * Copyright (c) 2017 Daire McNamara
 * Copyright (c) 2003-2004 Fabrice Bellard
 * Copyright (c) 2008 Citrix Systems, Inc.
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

#ifndef HW_SERIAL_H
#define HW_SERIAL_H

#include "hw/hw.h"
#include "sysemu/sysemu.h"
#include "exec/memory.h"
#include "qemu/fifo8.h"

#define UART_FIFO_LENGTH    16      /* 16550A Fifo Length */

typedef struct _RiscvSerialState {
    uint16_t divider;
    uint8_t rbr; /* receive register */
    uint8_t thr; /* transmit holding register */
    uint8_t tsr; /* transmit shift register */
    uint8_t ier;
    uint8_t iir; /* read only */
    uint8_t lcr;
    uint8_t mcr;
    uint8_t lsr; /* read only */
    uint8_t msr; /* read only */
    uint8_t scr;
    uint8_t fcr;
    uint8_t mm0; /* multi-mode control 0 */
    uint8_t mm1; /* multi-mode control 1 */
    uint8_t mm2; /* multi-mode control 2 */
    uint8_t dfr;
    uint8_t gfr; /* glitch filter */
    uint8_t ttg; /* transmitter time guard */
    uint8_t rto; /* receiver time out */
    uint8_t fcr_vmstate; /* we can't write directly this value
                            it has side effects */
    /* NOTE: this hidden state is necessary for tx irq generation as
       it can be reset while reading iir */
    int thr_ipending;
    qemu_plic_irq plic_raise_irq;
    qemu_plic_irq plic_lower_irq;
    uint32_t plic_src;
    CharDriverState *chr;
    int last_break_enable;
    int it_shift;
    int baudbase;
    uint32_t tsr_retry;
    guint watch_tag;
    uint32_t wakeup;

    /* Time when the last byte was successfully sent out of the tsr */
    uint64_t last_xmit_ts;
    Fifo8 recv_fifo;
    Fifo8 xmit_fifo;
    /* Interrupt trigger level for recv_fifo */
    uint8_t recv_fifo_itl;

    QEMUTimer *fifo_timeout_timer;
    int timeout_ipending;           /* timeout interrupt pending state */

    uint64_t char_transmit_time;    /* time to transmit a char in ticks */
    int poll_msl;

    QEMUTimer *modem_status_poll;
    MemoryRegion io;
    int reg_width;
} RiscvSerialState;

extern const VMStateDescription vmstate_serial;
extern const MemoryRegionOps serial_io_ops;

void riscv_serial_realize_core(RiscvSerialState *s, Error **errp);
void riscv_serial_exit_core(RiscvSerialState *s);
void riscv_serial_set_frequency(RiscvSerialState *s, uint32_t frequency);

/* legacy pre qom */
RiscvSerialState *riscv_serial_init(int base, qemu_plic_irq plic_raise_irq,
    qemu_plic_irq plic_lower_irq, int plic_src, int baudbase, int regwidth,
    CharDriverState *chr, MemoryRegion *system_io);

RiscvSerialState *riscv_serial_mm_init(MemoryRegion *address_space,
    hwaddr base, int it_shift, qemu_plic_irq plic_raise_irq,
    qemu_plic_irq plic_lower_irq, int plic_src, int baudbase,
    CharDriverState *chr, enum device_endian end);

/* serial-isa.c */
#define TYPE_ISA_SERIAL "isa-serial"
void riscv_serial_hds_isa_init(ISABus *bus, int n);

#endif
