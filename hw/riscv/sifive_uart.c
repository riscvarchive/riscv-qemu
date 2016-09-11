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

/* I have no documentation on the part/IP block, this is reverse engineered
 * from the driver.  Structually derived from hw/char/xilinx_uartlite.c.
 */

#define DUART(x)

#define R_DATA          0
#define R_TXCNT         1
#define R_RXCNT         2
#define R_DIV           3
#define R_MAX           4

#define TYPE_SIFIVE_UART "riscv.sifive-uart"
#define SIFIVE_UART(obj) \
    OBJECT_CHECK(SiFiveUART, (obj), TYPE_SIFIVE_UART)

typedef struct SiFiveUART {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    CharDriverState *chr;

    uint8_t rx_fifo[8];
    unsigned int rx_fifo_len;
} SiFiveUART;

static void sifive_uart_reset(DeviceState *dev)
{
}

static uint64_t
uart_read(void *opaque, hwaddr addr, unsigned int size)
{
    SiFiveUART *s = opaque;
    unsigned char r;
    switch (addr >> 2)
    {
        case R_DATA:
            if (s->rx_fifo_len) {
                r = s->rx_fifo[0];
                memmove(s->rx_fifo, s->rx_fifo + 1, s->rx_fifo_len - 1);
                s->rx_fifo_len--;
                qemu_chr_accept_input(s->chr);
                return r;
            }
            return 0;

        case R_RXCNT:
            return s->rx_fifo_len;

        default:
            hw_error("%s: bad read: addr=%x\n", __func__, (int)addr);
            return 0;
    }
    return r;
}

static void
uart_write(void *opaque, hwaddr addr,
           uint64_t val64, unsigned int size)
{
    SiFiveUART *s = opaque;
    uint32_t value = val64;
    unsigned char ch = value;

    switch (addr >> 2)
    {
        case R_DATA:
            if (s->chr)
                /* XXX this blocks entire thread. Rewrite to use
                 * qemu_chr_fe_write and background I/O callbacks */
                qemu_chr_fe_write_all(s->chr, &ch, 1);
            break;

        default:
            hw_error("%s: bad write: addr=%x v=%x\n", __func__, (int)addr, (int)value);
            break;
    }
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
    DEFINE_PROP_CHR("chardev", SiFiveUART, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void uart_rx(void *opaque, const uint8_t *buf, int size)
{
    SiFiveUART *s = opaque;

    /* Got a byte.  */
    if (s->rx_fifo_len >= sizeof(s->rx_fifo)) {
        printf("WARNING: UART dropped char.\n");
        return;
    }
    s->rx_fifo[s->rx_fifo_len++] = *buf;
}

static int uart_can_rx(void *opaque)
{
    SiFiveUART *s = opaque;

    return s->rx_fifo_len < sizeof(s->rx_fifo);
}

static void uart_event(void *opaque, int event)
{
}

static void sifive_uart_realize(DeviceState *dev, Error **errp)
{
    SiFiveUART *s = SIFIVE_UART(dev);

    if (s->chr)
        qemu_chr_add_handlers(s->chr, uart_can_rx, uart_rx, uart_event, s);
}

static void sifive_uart_init(Object *obj)
{
    SiFiveUART *s = SIFIVE_UART(obj);

    memory_region_init_io(&s->mmio, obj, &uart_ops, s,
                          "riscv.sifive_uart", R_MAX * 4);
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
    .instance_size = sizeof(SiFiveUART),
    .instance_init = sifive_uart_init,
    .class_init    = sifive_uart_class_init,
};

static void sifive_uart_register_types(void)
{
    type_register_static(&sifive_uart_info);
}

type_init(sifive_uart_register_types)
