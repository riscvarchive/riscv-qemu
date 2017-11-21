/*
 * QEMU model of the UART on the SiFive U500 SoC (identity unknown).
 *
 * Copyright (c) 2016 Stefan O'Rear
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
#include "hw/sysbus.h"
#include "sysemu/char.h"
#include "hw/riscv/cpudevs.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/riscv/sifive_hw.h"

/* See https://github.com/sifive/sifive-blocks/tree/072d0c1b58/src/main/scala/devices/uart */

/* Not yet implemented: TXFIFO / async writing, interrupt generation, divisor */

#define DUART(x)

#define R_TXFIFO        0
#define R_RXFIFO        4
#define R_TXCTRL        8
#define R_TXMARK        10
#define R_RXCTRL        12
#define R_RXMARK        14
#define R_IE            16
#define R_IP            20
#define R_DIV           24

#define R_MAX           32

static void sifive_uart_reset(DeviceState *dev)
{
}

static uint64_t
uart_read(void *opaque, hwaddr addr, unsigned int size)
{
    SiFiveUARTState *s = opaque;
    unsigned char r;
    switch (addr)
    {
        case R_RXFIFO:
            if (s->rx_fifo_len) {
                r = s->rx_fifo[0];
                memmove(s->rx_fifo, s->rx_fifo + 1, s->rx_fifo_len - 1);
                s->rx_fifo_len--;
                qemu_chr_accept_input(s->chr);
                return r;
            }
            return 0x80000000;

        case R_TXFIFO:
            return 0; /* Should check tx fifo */
        case R_TXCTRL:
            return s->txctrl;
        case R_RXCTRL:
            return s->rxctrl;
        case R_DIV:
            return s->div;
    }

    hw_error("%s: bad read: addr=0x%x\n", __func__, (int)addr);
    return 0;
}

static void
uart_write(void *opaque, hwaddr addr,
           uint64_t val64, unsigned int size)
{
    SiFiveUARTState *s = opaque;
    uint32_t value = val64;
    unsigned char ch = value;

    switch (addr)
    {
        case R_TXFIFO:
            if (s->chr)
                /* XXX this blocks entire thread. Rewrite to use
                 * qemu_chr_fe_write and background I/O callbacks */
                qemu_chr_fe_write_all(s->chr, &ch, 1);
            return;
        case R_TXCTRL:
            s->txctrl = val64;
            return;
        case R_RXCTRL:
            s->rxctrl = val64;
            return;
        case R_DIV:
            s->div = val64;
            return;
    }
    hw_error("%s: bad write: addr=0x%x v=0x%x\n", __func__, (int)addr, (int)value);
}

static const MemoryRegionOps uart_ops = {
    .read = uart_read,
    .write = uart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static Property sifive_uart_properties[] = {
    DEFINE_PROP_CHR("chardev", SiFiveUARTState, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void uart_rx(void *opaque, const uint8_t *buf, int size)
{
    SiFiveUARTState *s = opaque;

    /* Got a byte.  */
    if (s->rx_fifo_len >= sizeof(s->rx_fifo)) {
        printf("WARNING: UART dropped char.\n");
        return;
    }
    s->rx_fifo[s->rx_fifo_len++] = *buf;
}

static int uart_can_rx(void *opaque)
{
    SiFiveUARTState *s = opaque;

    return s->rx_fifo_len < sizeof(s->rx_fifo);
}

static void uart_event(void *opaque, int event)
{
}

static void sifive_uart_realize(DeviceState *dev, Error **errp)
{
    SiFiveUARTState *s = SIFIVE_UART(dev);

    if (s->chr)
        qemu_chr_add_handlers(s->chr, uart_can_rx, uart_rx, uart_event, s);
}

static void sifive_uart_init(Object *obj)
{
    SiFiveUARTState *s = SIFIVE_UART(obj);

    memory_region_init_io(&s->mmio, obj, &uart_ops, s,
                          TYPE_SIFIVE_UART, R_MAX);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static void sifive_uart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = sifive_uart_reset;
    dc->realize = sifive_uart_realize;
    dc->props = sifive_uart_properties;
}

static const TypeInfo sifive_uart_info = {
    .name          = TYPE_SIFIVE_UART,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SiFiveUARTState),
    .instance_init = sifive_uart_init,
    .class_init    = sifive_uart_class_init,
};

static void sifive_uart_register_types(void)
{
    type_register_static(&sifive_uart_info);
}

type_init(sifive_uart_register_types)


/*
 * Create UART device.
 */
DeviceState *sifive_uart_create(hwaddr addr, CharDriverState *chr)
{
    DeviceState *dev = qdev_create(NULL, TYPE_SIFIVE_UART);
    qdev_prop_set_chr(dev, "chardev", chr);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, addr);
    return dev;
}
