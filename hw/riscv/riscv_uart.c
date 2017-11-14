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

#include "qemu/osdep.h"
#include "hw/riscv/riscv_plic.h"
#include "hw/riscv/riscv_uart.h"
#include "sysemu/char.h"
#include "qapi/error.h"
#include "qemu/timer.h"
#include "exec/address-spaces.h"
#include "qemu/error-report.h"

/* #define DEBUG_SERIAL */

#define UART_LCR_DLAB	0x80	/* Divisor latch access bit */

#define UART_IER_MSI	0x08	/* Enable Modem status interrupt */
#define UART_IER_RLSI	0x04	/* Enable receiver line status interrupt */
#define UART_IER_THRI	0x02	/* Enable Transmitter holding register int. */
#define UART_IER_RDI	0x01	/* Enable receiver data interrupt */

#define UART_IIR_NO_INT	0x01	/* No interrupts pending */
#define UART_IIR_ID	0x06	/* Mask for the interrupt ID */

#define UART_IIR_MSI	0x00	/* Modem status interrupt */
#define UART_IIR_THRI	0x02	/* Transmitter holding register empty */
#define UART_IIR_RDI	0x04	/* Receiver data interrupt */
#define UART_IIR_RLSI	0x06	/* Receiver line status interrupt */
#define UART_IIR_CTI    0x0C    /* Character Timeout Indication */

#define UART_IIR_FENF   0x80    /* Fifo enabled, but not functionning */
#define UART_IIR_FE     0xC0    /* Fifo enabled */

/*
 * These are the definitions for the Modem Control Register
 */
#define UART_MCR_LOOP	0x10	/* Enable loopback test mode */
#define UART_MCR_OUT2	0x08	/* Out2 complement */
#define UART_MCR_OUT1	0x04	/* Out1 complement */
#define UART_MCR_RTS	0x02	/* RTS complement */
#define UART_MCR_DTR	0x01	/* DTR complement */

/*
 * These are the definitions for the Modem Status Register
 */
#define UART_MSR_DCD	0x80	/* Data Carrier Detect */
#define UART_MSR_RI	0x40	/* Ring Indicator */
#define UART_MSR_DSR	0x20	/* Data Set Ready */
#define UART_MSR_CTS	0x10	/* Clear to Send */
#define UART_MSR_DDCD	0x08	/* Delta DCD */
#define UART_MSR_TERI	0x04	/* Trailing edge ring indicator */
#define UART_MSR_DDSR	0x02	/* Delta DSR */
#define UART_MSR_DCTS	0x01	/* Delta CTS */
#define UART_MSR_ANY_DELTA 0x0F	/* Any of the delta bits! */

#define UART_LSR_TEMT	0x40	/* Transmitter empty */
#define UART_LSR_THRE	0x20	/* Transmit-hold-register empty */
#define UART_LSR_BI	0x10	/* Break interrupt indicator */
#define UART_LSR_FE	0x08	/* Frame error indicator */
#define UART_LSR_PE	0x04	/* Parity error indicator */
#define UART_LSR_OE	0x02	/* Overrun error indicator */
#define UART_LSR_DR	0x01	/* Receiver data ready */
#define UART_LSR_INT_ANY 0x1E	/* Any of the lsr-interrupt-triggering status bits */

/* Interrupt trigger levels. The byte-counts are for 16550A - in newer UARTs the byte-count for each ITL is higher. */

#define UART_FCR_ITL_1      0x00 /* 1 byte ITL */
#define UART_FCR_ITL_2      0x40 /* 4 bytes ITL */
#define UART_FCR_ITL_3      0x80 /* 8 bytes ITL */
#define UART_FCR_ITL_4      0xC0 /* 14 bytes ITL */

#define UART_FCR_DMS        0x08    /* DMA Mode Select */
#define UART_FCR_XFR        0x04    /* XMIT Fifo Reset */
#define UART_FCR_RFR        0x02    /* RCVR Fifo Reset */
#define UART_FCR_FE         0x01    /* FIFO Enable */

#define MAX_XMIT_RETRY      4

#ifdef DEBUG_SERIAL
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "serial: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) \
do {} while (0)
#endif

static void serial_receive1(void *opaque, const uint8_t *buf, int size);
static void serial_xmit(RiscvSerialState *s);

static inline void recv_fifo_put(RiscvSerialState *s, uint8_t chr)
{
    /* Receive overruns do not overwrite FIFO contents. */
    if (!fifo8_is_full(&s->recv_fifo)) {
        fifo8_push(&s->recv_fifo, chr);
    } else {
        s->lsr |= UART_LSR_OE;
    }
}

static void serial_update_irq(RiscvSerialState *s)
{
    uint8_t tmp_iir = UART_IIR_NO_INT;

    if ((s->ier & UART_IER_RLSI) && (s->lsr & UART_LSR_INT_ANY)) {
        tmp_iir = UART_IIR_RLSI;
    } else if ((s->ier & UART_IER_RDI) && s->timeout_ipending) {
        /* Note that(s->ier & UART_IER_RDI) can mask this interrupt,
         * this is not in the specification but is observed on existing
         * hardware.  */
        tmp_iir = UART_IIR_CTI;
    } else if ((s->ier & UART_IER_RDI) && (s->lsr & UART_LSR_DR) &&
               (!(s->fcr & UART_FCR_FE) ||
                s->recv_fifo.num >= s->recv_fifo_itl)) {
        tmp_iir = UART_IIR_RDI;
    } else if ((s->ier & UART_IER_THRI) && s->thr_ipending) {
        tmp_iir = UART_IIR_THRI;
    } else if ((s->ier & UART_IER_MSI) && (s->msr & UART_MSR_ANY_DELTA)) {
        tmp_iir = UART_IIR_MSI;
    }

    s->iir = tmp_iir | (s->iir & 0xF0);

    if (tmp_iir != UART_IIR_NO_INT) {
        s->plic_raise_irq(s->plic_src);
    } else {
        s->plic_lower_irq(s->plic_src);
    }
}

static void serial_update_parameters(RiscvSerialState *s)
{
    int speed, parity, data_bits, stop_bits, frame_size;
    QEMUSerialSetParams ssp;

    if (s->divider == 0)
        return;

    /* Start bit. */
    frame_size = 1;
    if (s->lcr & 0x08) {
        /* Parity bit. */
        frame_size++;
        if (s->lcr & 0x10)
            parity = 'E';
        else
            parity = 'O';
    } else {
            parity = 'N';
    }
    if (s->lcr & 0x04)
        stop_bits = 2;
    else
        stop_bits = 1;

    data_bits = (s->lcr & 0x03) + 5;
    frame_size += data_bits + stop_bits;
    speed = s->baudbase / s->divider;
    ssp.speed = speed;
    ssp.parity = parity;
    ssp.data_bits = data_bits;
    ssp.stop_bits = stop_bits;
    s->char_transmit_time =  (NANOSECONDS_PER_SECOND / speed) * frame_size;
    qemu_chr_fe_ioctl(s->chr, CHR_IOCTL_SERIAL_SET_PARAMS, &ssp);

    DPRINTF("speed=%d parity=%c data=%d stop=%d\n",
           speed, parity, data_bits, stop_bits);
}

static void serial_update_msl(RiscvSerialState *s)
{
    uint8_t omsr;
    int flags;

    timer_del(s->modem_status_poll);

    if (qemu_chr_fe_ioctl(s->chr,CHR_IOCTL_SERIAL_GET_TIOCM, &flags) == -ENOTSUP) {
        s->poll_msl = -1;
        return;
    }

    omsr = s->msr;

    s->msr = (flags & CHR_TIOCM_CTS) ? s->msr | UART_MSR_CTS : s->msr & ~UART_MSR_CTS;
    s->msr = (flags & CHR_TIOCM_DSR) ? s->msr | UART_MSR_DSR : s->msr & ~UART_MSR_DSR;
    s->msr = (flags & CHR_TIOCM_CAR) ? s->msr | UART_MSR_DCD : s->msr & ~UART_MSR_DCD;
    s->msr = (flags & CHR_TIOCM_RI) ? s->msr | UART_MSR_RI : s->msr & ~UART_MSR_RI;

    if (s->msr != omsr) {
         /* Set delta bits */
         s->msr = s->msr | ((s->msr >> 4) ^ (omsr >> 4));
         /* UART_MSR_TERI only if change was from 1 -> 0 */
         if ((s->msr & UART_MSR_TERI) && !(omsr & UART_MSR_RI))
             s->msr &= ~UART_MSR_TERI;
         serial_update_irq(s);
    }

    /* The real 16550A apparently has a 250ns response latency to line status changes.
       We'll be lazy and poll only every 10ms, and only poll it at all if MSI interrupts are turned on */

    if (s->poll_msl) {
        timer_mod(s->modem_status_poll, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  NANOSECONDS_PER_SECOND / 100);
    }
}

static gboolean serial_watch_cb(GIOChannel *chan, GIOCondition cond,
                                void *opaque)
{
    RiscvSerialState *s = opaque;
    s->watch_tag = 0;
    serial_xmit(s);
    return FALSE;
}

static void serial_xmit(RiscvSerialState *s)
{
    do {
        assert(!(s->lsr & UART_LSR_TEMT));
        if (s->tsr_retry == 0) {
            assert(!(s->lsr & UART_LSR_THRE));

            if (s->fcr & UART_FCR_FE) {
                assert(!fifo8_is_empty(&s->xmit_fifo));
                s->tsr = fifo8_pop(&s->xmit_fifo);
                if (!s->xmit_fifo.num) {
                    s->lsr |= UART_LSR_THRE;
                }
            } else {
                s->tsr = s->thr;
                s->lsr |= UART_LSR_THRE;
            }
            if ((s->lsr & UART_LSR_THRE) && !s->thr_ipending) {
                s->thr_ipending = 1;
                serial_update_irq(s);
            }
        }

        if (s->mcr & UART_MCR_LOOP) {
            /* in loopback mode, say that we just received a char */
            serial_receive1(s, &s->tsr, 1);
        } else if (qemu_chr_fe_write(s->chr, &s->tsr, 1) != 1 &&
                   s->tsr_retry < MAX_XMIT_RETRY) {
            assert(s->watch_tag == 0);
            s->watch_tag = qemu_chr_fe_add_watch(s->chr, G_IO_OUT|G_IO_HUP,
                                                 serial_watch_cb, s);
            if (s->watch_tag > 0) {
                s->tsr_retry++;
                return;
            }
        }
        s->tsr_retry = 0;

        /* Transmit another byte if it is already available. It is only
           possible when FIFO is enabled and not empty. */
    } while (!(s->lsr & UART_LSR_THRE));

    s->last_xmit_ts = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->lsr |= UART_LSR_TEMT;
}

/* Setter for FCR.
   is_load flag means, that value is set while loading VM state
   and interrupt should not be invoked */
static void serial_write_fcr(RiscvSerialState *s, uint8_t val)
{
    /* Set fcr - val only has the bits that are supposed to "stick" */
    s->fcr = val;

    if (val & UART_FCR_FE) {
        s->iir |= UART_IIR_FE;
        /* Set recv_fifo trigger Level */
        switch (val & 0xC0) {
        case UART_FCR_ITL_1:
            s->recv_fifo_itl = 1;
            break;
        case UART_FCR_ITL_2:
            s->recv_fifo_itl = 4;
            break;
        case UART_FCR_ITL_3:
            s->recv_fifo_itl = 8;
            break;
        case UART_FCR_ITL_4:
            s->recv_fifo_itl = 14;
            break;
        }
    } else {
        s->iir &= ~UART_IIR_FE;
    }
}

static void serial_ioport_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned size)
{
    RiscvSerialState *s = opaque;

    //addr &= 7;
    DPRINTF("write addr=0x%" HWADDR_PRIx " val=0x%" PRIx64 " size=%d\n", addr, val, size);
    if(addr == 0*s->reg_width) {
        if (s->lcr & UART_LCR_DLAB) {
            s->divider = (s->divider & 0xff00) | val;
            serial_update_parameters(s);
        } else {
            s->thr = (uint8_t) val;
            if(s->fcr & UART_FCR_FE) {
                /* xmit overruns overwrite data, so make space if needed */
                if (fifo8_is_full(&s->xmit_fifo)) {
                    fifo8_pop(&s->xmit_fifo);
                }
                fifo8_push(&s->xmit_fifo, s->thr);
            }
            s->thr_ipending = 0;
            s->lsr &= ~UART_LSR_THRE;
            s->lsr &= ~UART_LSR_TEMT;
            serial_update_irq(s);
            if (s->tsr_retry == 0) {
                serial_xmit(s);
            }
        }
    } else if(addr ==  1*s->reg_width) {
        if (s->lcr & UART_LCR_DLAB) {
            s->divider = (s->divider & 0x00ff) | (val << 8);
            serial_update_parameters(s);
        } else {
            uint8_t changed = (s->ier ^ val) & 0x0f;
            s->ier = val & 0x0f;
            /* If the backend device is a real serial port, turn polling of the modem
             * status lines on physical port on or off depending on UART_IER_MSI state.
             */
            if ((changed & UART_IER_MSI) && s->poll_msl >= 0) {
                if (s->ier & UART_IER_MSI) {
                     s->poll_msl = 1;
                     serial_update_msl(s);
                } else {
                     timer_del(s->modem_status_poll);
                     s->poll_msl = 0;
                }
            }

            /* Turning on the THRE interrupt on IER can trigger the interrupt
             * if LSR.THRE=1, even if it had been masked before by reading IIR.
             * This is not in the datasheet, but Windows relies on it.  It is
             * unclear if THRE has to be resampled every time THRI becomes
             * 1, or only on the rising edge.  Bochs does the latter, and Windows
             * always toggles IER to all zeroes and back to all ones, so do the
             * same.
             *
             * If IER.THRI is zero, thr_ipending is not used.  Set it to zero
             * so that the thr_ipending subsection is not migrated.
             */
            if (changed & UART_IER_THRI) {
                if ((s->ier & UART_IER_THRI) && (s->lsr & UART_LSR_THRE)) {
                    s->thr_ipending = 1;
                } else {
                    s->thr_ipending = 0;
                }
            }

            if (changed) {
                serial_update_irq(s);
            }
        }
    } else if (addr == 2*s->reg_width) {
        /* Did the enable/disable flag change? If so, make sure FIFOs get flushed */
        if ((val ^ s->fcr) & UART_FCR_FE) {
            val |= UART_FCR_XFR | UART_FCR_RFR;
        }

        /* FIFO clear */

        if (val & UART_FCR_RFR) {
            s->lsr &= ~(UART_LSR_DR | UART_LSR_BI);
            timer_del(s->fifo_timeout_timer);
            s->timeout_ipending = 0;
            fifo8_reset(&s->recv_fifo);
        }

        if (val & UART_FCR_XFR) {
            s->lsr |= UART_LSR_THRE;
            s->thr_ipending = 1;
            fifo8_reset(&s->xmit_fifo);
        }

        serial_write_fcr(s, val & 0xC9);
        serial_update_irq(s);
    } else if(addr == 3*s->reg_width) {
        {
            int break_enable;
            s->lcr = val;
            serial_update_parameters(s);
            break_enable = (val >> 6) & 1;
            if (break_enable != s->last_break_enable) {
                s->last_break_enable = break_enable;
                qemu_chr_fe_ioctl(s->chr, CHR_IOCTL_SERIAL_SET_BREAK,
                               &break_enable);
            }
        }
    } else if(addr==  4*s->reg_width) {
        {
            int flags;
            int old_mcr = s->mcr;
            s->mcr = val & 0x1f;
            if (val & UART_MCR_LOOP) {

            } else {
                if (s->poll_msl >= 0 && old_mcr != s->mcr) {

                    qemu_chr_fe_ioctl(s->chr,CHR_IOCTL_SERIAL_GET_TIOCM, &flags);

                    flags &= ~(CHR_TIOCM_RTS | CHR_TIOCM_DTR);

                    if (val & UART_MCR_RTS)
                        flags |= CHR_TIOCM_RTS;
                    if (val & UART_MCR_DTR)
                        flags |= CHR_TIOCM_DTR;

                    qemu_chr_fe_ioctl(s->chr,CHR_IOCTL_SERIAL_SET_TIOCM, &flags);
                    /* Update the modem status after a one-character-send wait-time, since there may be a response
                       from the device/computer at the other end of the serial line */
                    timer_mod(s->modem_status_poll, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->char_transmit_time);
                }
            }
        }
    } else if(addr == 5*s->reg_width) {
        ;
    } else if(addr == 6*s->reg_width) {
        ;
    } else if(addr == 7*s->reg_width) {
        s->scr = val;
    } else if(addr == 0x0c*s->reg_width) {
        s->mm0 = val;
    } else if(addr == 0x0d*s->reg_width) {
        s->mm1 = val;
    } else if(addr == 0x0e*s->reg_width) {
        s->mm2 = val;
    } else if(addr == 0x0f*s->reg_width) {
        s->dfr = val;
    } else if(addr == 0x11*s->reg_width) {
        s->gfr = val;
    } else if(addr == 0x12*s->reg_width) {
        s->ttg = val;
    } else if(addr == 0x13*s->reg_width) {
        s->rto = val;
    } else {
      fprintf(stderr, "Unknown UART register write: addr: %lx, val: %lx\n", addr, val);
    }
}

static uint64_t serial_ioport_read(void *opaque, hwaddr addr, unsigned size)
{
    RiscvSerialState *s = opaque;
    uint32_t ret = 0;

    //addr &= 7;
    if(addr == 0*s->reg_width) {
        if (s->lcr & UART_LCR_DLAB) {
            ret = s->divider & 0xff;
        } else {
            if(s->fcr & UART_FCR_FE) {
                ret = fifo8_is_empty(&s->recv_fifo) ?
                            0 : fifo8_pop(&s->recv_fifo);
                if (s->recv_fifo.num == 0) {
                    s->lsr &= ~(UART_LSR_DR | UART_LSR_BI);
                } else {
                    timer_mod(s->fifo_timeout_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->char_transmit_time * 4);
                }
                s->timeout_ipending = 0;
            } else {
                ret = s->rbr;
                s->lsr &= ~(UART_LSR_DR | UART_LSR_BI);
            }
            serial_update_irq(s);
            if (!(s->mcr & UART_MCR_LOOP)) {
                /* in loopback mode, don't receive any data */
                qemu_chr_accept_input(s->chr);
            }
        }
    } else if(addr == 1*s->reg_width) {
        if (s->lcr & UART_LCR_DLAB) {
            ret = (s->divider >> 8) & 0xff;
        } else {
            ret = s->ier;
        }
    } else if(addr == 2*s->reg_width) {
        ret = s->iir;
        if ((ret & UART_IIR_ID) == UART_IIR_THRI) {
            s->thr_ipending = 0;
            serial_update_irq(s);
        }
    } else if(addr == 3*s->reg_width) {
        ret = s->lcr;
    } else if(addr == 4*s->reg_width) {
        ret = s->mcr;
    } else if(addr == 5*s->reg_width) {
        ret = s->lsr;
        /* Clear break and overrun interrupts */
        if (s->lsr & (UART_LSR_BI|UART_LSR_OE)) {
            s->lsr &= ~(UART_LSR_BI|UART_LSR_OE);
            serial_update_irq(s);
        }
    } else if(addr == 6*s->reg_width) {
        if (s->mcr & UART_MCR_LOOP) {
            /* in loopback, the modem output pins are connected to the
               inputs */
            ret = (s->mcr & 0x0c) << 4;
            ret |= (s->mcr & 0x02) << 3;
            ret |= (s->mcr & 0x01) << 5;
        } else {
            if (s->poll_msl >= 0)
                serial_update_msl(s);
            ret = s->msr;
            /* Clear delta bits & msr int after read, if they were set */
            if (s->msr & UART_MSR_ANY_DELTA) {
                s->msr &= 0xF0;
                serial_update_irq(s);
            }
        }
    } else if(addr == 7*s->reg_width) {
        ret = s->scr;
    } else if(addr == 0x0c*s->reg_width) {
        ret = s->mm0;
    } else if(addr == 0x0d*s->reg_width) {
        ret = s->mm1;
    } else if(addr == 0x0e*s->reg_width) {
        ret = s->mm2;
    } else if(addr == 0x0f*s->reg_width) {
        ret = s->dfr;
    } else if(addr == 0x11*s->reg_width) {
        ret = s->gfr;
    } else if(addr == 0x12*s->reg_width) {
        ret = s->ttg;
    } else if(addr == 0x13*s->reg_width) {
        ret = s->rto;
    } else {
       fprintf(stderr, "Unknown UART register read: addr: %lx\n", addr);
    }
    DPRINTF("read addr=0x%" HWADDR_PRIx " val=0x%02x\n", addr, ret);
    return ret;
}

static int serial_can_receive(RiscvSerialState *s)
{
    if(s->fcr & UART_FCR_FE) {
        if (s->recv_fifo.num < UART_FIFO_LENGTH) {
            /*
             * Advertise (fifo.itl - fifo.count) bytes when count < ITL, and 1
             * if above. If UART_FIFO_LENGTH - fifo.count is advertised the
             * effect will be to almost always fill the fifo completely before
             * the guest has a chance to respond, effectively overriding the ITL
             * that the guest has set.
             */
            return (s->recv_fifo.num <= s->recv_fifo_itl) ?
                        s->recv_fifo_itl - s->recv_fifo.num : 1;
        } else {
            return 0;
        }
    } else {
        return !(s->lsr & UART_LSR_DR);
    }
}

static void serial_receive_break(RiscvSerialState *s)
{
    s->rbr = 0;
    /* When the LSR_DR is set a null byte is pushed into the fifo */
    recv_fifo_put(s, '\0');
    s->lsr |= UART_LSR_BI | UART_LSR_DR;
    serial_update_irq(s);
}

/* There's data in recv_fifo and s->rbr has not been read for 4 char transmit times */
static void fifo_timeout_int (void *opaque) {
    RiscvSerialState *s = opaque;
    if (s->recv_fifo.num) {
        s->timeout_ipending = 1;
        serial_update_irq(s);
    }
}

static int serial_can_receive1(void *opaque)
{
    RiscvSerialState *s = opaque;
    return serial_can_receive(s);
}

static void serial_receive1(void *opaque, const uint8_t *buf, int size)
{
    RiscvSerialState *s = opaque;

    if (s->wakeup) {
        qemu_system_wakeup_request(QEMU_WAKEUP_REASON_OTHER);
    }
    if(s->fcr & UART_FCR_FE) {
        int i;
        for (i = 0; i < size; i++) {
            recv_fifo_put(s, buf[i]);
        }
        s->lsr |= UART_LSR_DR;
        /* call the timeout receive callback in 4 char transmit time */
        timer_mod(s->fifo_timeout_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->char_transmit_time * 4);
    } else {
        if (s->lsr & UART_LSR_DR)
            s->lsr |= UART_LSR_OE;
        s->rbr = buf[0];
        s->lsr |= UART_LSR_DR;
    }
    serial_update_irq(s);
}

static void serial_event(void *opaque, int event)
{
    RiscvSerialState *s = opaque;
    DPRINTF("event %x\n", event);
    if (event == CHR_EVENT_BREAK)
        serial_receive_break(s);
}

static void serial_pre_save(void *opaque)
{
    RiscvSerialState *s = opaque;
    DPRINTF("pre-save\n");
    s->fcr_vmstate = s->fcr;
}

static int serial_pre_load(void *opaque)
{
    RiscvSerialState *s = opaque;
    DPRINTF("pre-load\n");
    s->thr_ipending = -1;
    s->poll_msl = -1;
    return 0;
}

static int serial_post_load(void *opaque, int version_id)
{
    RiscvSerialState *s = opaque;

    if (version_id < 3) {
        s->fcr_vmstate = 0;
    }
    if (s->thr_ipending == -1) {
        s->thr_ipending = ((s->iir & UART_IIR_ID) == UART_IIR_THRI);
    }

    if (s->tsr_retry > 0) {
        /* tsr_retry > 0 implies LSR.TEMT = 0 (transmitter not empty).  */
        if (s->lsr & UART_LSR_TEMT) {
            error_report("inconsistent state in serial device "
                         "(tsr empty, tsr_retry=%d", s->tsr_retry);
            return -1;
        }

        if (s->tsr_retry > MAX_XMIT_RETRY) {
            s->tsr_retry = MAX_XMIT_RETRY;
        }

        assert(s->watch_tag == 0);
        s->watch_tag = qemu_chr_fe_add_watch(s->chr, G_IO_OUT|G_IO_HUP,
                                             serial_watch_cb, s);
    } else {
        /* tsr_retry == 0 implies LSR.TEMT = 1 (transmitter empty).  */
        if (!(s->lsr & UART_LSR_TEMT)) {
            error_report("inconsistent state in serial device "
                         "(tsr not empty, tsr_retry=0");
            return -1;
        }
    }

    s->last_break_enable = (s->lcr >> 6) & 1;
    /* Initialize fcr via setter to perform essential side-effects */
    serial_write_fcr(s, s->fcr_vmstate);
    serial_update_parameters(s);
    return 0;
}

static bool serial_thr_ipending_needed(void *opaque)
{
    RiscvSerialState *s = opaque;

    if (s->ier & UART_IER_THRI) {
        bool expected_value = ((s->iir & UART_IIR_ID) == UART_IIR_THRI);
        return s->thr_ipending != expected_value;
    } else {
        /* LSR.THRE will be sampled again when the interrupt is
         * enabled.  thr_ipending is not used in this case, do
         * not migrate it.
         */
        return false;
    }
}

static const VMStateDescription riscv_vmstate_serial_thr_ipending = {
    .name = "serial/thr_ipending",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = serial_thr_ipending_needed,
    .fields = (VMStateField[]) {
        VMSTATE_INT32(thr_ipending, RiscvSerialState),
        VMSTATE_END_OF_LIST()
    }
};

static bool serial_tsr_needed(void *opaque)
{
    RiscvSerialState *s = (RiscvSerialState *)opaque;
    return s->tsr_retry != 0;
}

static const VMStateDescription riscv_vmstate_serial_tsr = {
    .name = "serial/tsr",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = serial_tsr_needed,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(tsr_retry, RiscvSerialState),
        VMSTATE_UINT8(thr, RiscvSerialState),
        VMSTATE_UINT8(tsr, RiscvSerialState),
        VMSTATE_END_OF_LIST()
    }
};

static bool serial_recv_fifo_needed(void *opaque)
{
    RiscvSerialState *s = (RiscvSerialState *)opaque;
    return !fifo8_is_empty(&s->recv_fifo);

}

static const VMStateDescription riscv_vmstate_serial_recv_fifo = {
    .name = "serial/recv_fifo",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = serial_recv_fifo_needed,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT(recv_fifo, RiscvSerialState, 1, vmstate_fifo8, Fifo8),
        VMSTATE_END_OF_LIST()
    }
};

static bool serial_xmit_fifo_needed(void *opaque)
{
    RiscvSerialState *s = (RiscvSerialState *)opaque;
    return !fifo8_is_empty(&s->xmit_fifo);
}

static const VMStateDescription riscv_vmstate_serial_xmit_fifo = {
    .name = "serial/xmit_fifo",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = serial_xmit_fifo_needed,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT(xmit_fifo, RiscvSerialState, 1, vmstate_fifo8, Fifo8),
        VMSTATE_END_OF_LIST()
    }
};

static bool serial_fifo_timeout_timer_needed(void *opaque)
{
    RiscvSerialState *s = (RiscvSerialState *)opaque;
    return timer_pending(s->fifo_timeout_timer);
}

static const VMStateDescription riscv_vmstate_serial_fifo_timeout_timer = {
    .name = "serial/fifo_timeout_timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = serial_fifo_timeout_timer_needed,
    .fields = (VMStateField[]) {
        VMSTATE_TIMER_PTR(fifo_timeout_timer, RiscvSerialState),
        VMSTATE_END_OF_LIST()
    }
};

static bool serial_timeout_ipending_needed(void *opaque)
{
    RiscvSerialState *s = (RiscvSerialState *)opaque;
    return s->timeout_ipending != 0;
}

static const VMStateDescription riscv_vmstate_serial_timeout_ipending = {
    .name = "serial/timeout_ipending",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = serial_timeout_ipending_needed,
    .fields = (VMStateField[]) {
        VMSTATE_INT32(timeout_ipending, RiscvSerialState),
        VMSTATE_END_OF_LIST()
    }
};

static bool serial_poll_needed(void *opaque)
{
    RiscvSerialState *s = (RiscvSerialState *)opaque;
    return s->poll_msl >= 0;
}

static const VMStateDescription riscv_vmstate_serial_poll = {
    .name = "serial/poll",
    .version_id = 1,
    .needed = serial_poll_needed,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_INT32(poll_msl, RiscvSerialState),
        VMSTATE_TIMER_PTR(modem_status_poll, RiscvSerialState),
        VMSTATE_END_OF_LIST()
    }
};

const VMStateDescription riscv_vmstate_serial = {
    .name = "riscv-serial",
    .version_id = 3,
    .minimum_version_id = 2,
    .pre_save = serial_pre_save,
    .pre_load = serial_pre_load,
    .post_load = serial_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT16_V(divider, RiscvSerialState, 2),
        VMSTATE_UINT8(rbr, RiscvSerialState),
        VMSTATE_UINT8(ier, RiscvSerialState),
        VMSTATE_UINT8(iir, RiscvSerialState),
        VMSTATE_UINT8(lcr, RiscvSerialState),
        VMSTATE_UINT8(mcr, RiscvSerialState),
        VMSTATE_UINT8(lsr, RiscvSerialState),
        VMSTATE_UINT8(msr, RiscvSerialState),
        VMSTATE_UINT8(scr, RiscvSerialState),
        VMSTATE_UINT8_V(fcr_vmstate, RiscvSerialState, 3),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription*[]) {
        &riscv_vmstate_serial_thr_ipending,
        &riscv_vmstate_serial_tsr,
        &riscv_vmstate_serial_recv_fifo,
        &riscv_vmstate_serial_xmit_fifo,
        &riscv_vmstate_serial_fifo_timeout_timer,
        &riscv_vmstate_serial_timeout_ipending,
        &riscv_vmstate_serial_poll,
        NULL
    }
};

static void serial_reset(void *opaque)
{
    RiscvSerialState *s = opaque;

    if (s->watch_tag > 0) {
        g_source_remove(s->watch_tag);
        s->watch_tag = 0;
    }

    s->rbr = 0;
    s->ier = 0;
    s->iir = UART_IIR_NO_INT;
    s->lcr = 0;
    s->lsr = UART_LSR_TEMT | UART_LSR_THRE;
    s->msr = UART_MSR_DCD | UART_MSR_DSR | UART_MSR_CTS;
    /* Default to 9600 baud, 1 start bit, 8 data bits, 1 stop bit, no parity. */
    s->divider = 0x0C;
    s->mcr = UART_MCR_OUT2;
    s->scr = 0;
    s->tsr_retry = 0;
    s->char_transmit_time = (NANOSECONDS_PER_SECOND / 9600) * 10;
    s->poll_msl = 0;

    s->timeout_ipending = 0;
    timer_del(s->fifo_timeout_timer);
    timer_del(s->modem_status_poll);

    fifo8_reset(&s->recv_fifo);
    fifo8_reset(&s->xmit_fifo);

    s->last_xmit_ts = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->thr_ipending = 0;
    s->last_break_enable = 0;
    s->plic_lower_irq(s->plic_src);

    serial_update_msl(s);
    s->msr &= ~UART_MSR_ANY_DELTA;
}

void riscv_serial_realize_core(RiscvSerialState *s, Error **errp)
{
    if (!s->chr) {
        error_setg(errp, "Can't create serial device, empty char device");
        return;
    }

    s->modem_status_poll = timer_new_ns(QEMU_CLOCK_VIRTUAL, (QEMUTimerCB *) serial_update_msl, s);

    s->fifo_timeout_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, (QEMUTimerCB *) fifo_timeout_int, s);
    qemu_register_reset(serial_reset, s);

    qemu_chr_add_handlers(s->chr, serial_can_receive1, serial_receive1,
                          serial_event, s);
    fifo8_create(&s->recv_fifo, UART_FIFO_LENGTH);
    fifo8_create(&s->xmit_fifo, UART_FIFO_LENGTH);
    serial_reset(s);
}

void riscv_serial_exit_core(RiscvSerialState *s)
{
    qemu_chr_add_handlers(s->chr, NULL, NULL, NULL, NULL);
    qemu_unregister_reset(serial_reset, s);
}

/* Change the main reference oscillator frequency. */
void riscv_serial_set_frequency(RiscvSerialState *s, uint32_t frequency)
{
    s->baudbase = frequency;
    serial_update_parameters(s);
}

const MemoryRegionOps riscv_serial_io_ops = {
    .read = serial_ioport_read,
    .write = serial_ioport_write,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

RiscvSerialState *riscv_serial_init(int base, qemu_plic_irq plic_raise_irq,
                         qemu_plic_irq plic_lower_irq,
                         int plic_src, int baudbase, int reg_width,
                         CharDriverState *chr, MemoryRegion *system_io)
{
    RiscvSerialState *s;

    s = g_malloc0(sizeof(RiscvSerialState));

    s->plic_raise_irq = plic_raise_irq;
    s->plic_lower_irq = plic_lower_irq;
    s->plic_src = plic_src;
    s->baudbase = baudbase;
    s->chr = chr;
    s->reg_width = reg_width;
    riscv_serial_realize_core(s, &error_fatal);

    vmstate_register(NULL, base, &riscv_vmstate_serial, s);

    memory_region_init_io(&s->io, NULL, &riscv_serial_io_ops, s, "riscv-serial", 0x50);
    memory_region_add_subregion(system_io, base, &s->io);

    return s;
}

/* Memory mapped interface */
static uint64_t serial_mm_read(void *opaque, hwaddr addr,
                               unsigned size)
{
    RiscvSerialState *s = opaque;
    return serial_ioport_read(s, addr >> s->it_shift, 1);
}

static void serial_mm_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
    RiscvSerialState *s = opaque;
    value &= ~0u >> (32 - (size * 8));
    serial_ioport_write(s, addr >> s->it_shift, value, 1);
}

static const MemoryRegionOps serial_mm_ops[3] = {
    [DEVICE_NATIVE_ENDIAN] = {
        .read = serial_mm_read,
        .write = serial_mm_write,
        .endianness = DEVICE_NATIVE_ENDIAN,
    },
    [DEVICE_LITTLE_ENDIAN] = {
        .read = serial_mm_read,
        .write = serial_mm_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
    },
    [DEVICE_BIG_ENDIAN] = {
        .read = serial_mm_read,
        .write = serial_mm_write,
        .endianness = DEVICE_BIG_ENDIAN,
    },
};

RiscvSerialState *riscv_serial_mm_init(MemoryRegion *address_space,
                            hwaddr base, int it_shift,
                            qemu_plic_irq plic_raise_irq, 
                            qemu_plic_irq plic_lower_irq, int plic_src, 
                            int baudbase,
                            CharDriverState *chr, enum device_endian end)
{
    RiscvSerialState *s;

    s = g_malloc0(sizeof(RiscvSerialState));

    s->it_shift = it_shift;
    s->plic_raise_irq = plic_raise_irq;
    s->plic_lower_irq = plic_lower_irq;
    s->plic_src = plic_src;
    s->baudbase = baudbase;
    s->chr = chr;

    riscv_serial_realize_core(s, &error_fatal);
    vmstate_register(NULL, base, &riscv_vmstate_serial, s);

    memory_region_init_io(&s->io, NULL, &serial_mm_ops[end], s,
                          "riscv-serial-mm", 0x50 << it_shift);
    memory_region_add_subregion(address_space, base, &s->io);
    return s;
}
