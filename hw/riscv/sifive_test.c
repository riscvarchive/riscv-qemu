/*
 * QEMU SiFive Test Finisher
 *
 * Copyright (c) 2018 SiFive, Inc.
 *
 * Test finisher memory mapped device used to exit simulation
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
#include "target/riscv/cpu.h"
#include "hw/riscv/sifive_test.h"

static uint64_t sifive_test_read(void *opaque, hwaddr addr, unsigned int size)
{
    return 0;
}

static void sifive_test_write(void *opaque, hwaddr addr,
           uint64_t val64, unsigned int size)
{
    if (addr == 0) {
        int status = val64 & 0xffff;
        int code = (val64 >> 16) & 0xffff;
        switch (status) {
        case FINISHER_FAIL:
            exit(code);
        case FINISHER_PASS:
            exit(0);
        default:
            break;
        }
    }
    hw_error("%s: write: addr=0x%x val=0x%016" PRIx64 "\n",
        __func__, (int)addr, val64);
}

static const MemoryRegionOps sifive_test_ops = {
    .read = sifive_test_read,
    .write = sifive_test_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static void sifive_test_init(Object *obj)
{
    SiFiveTestState *s = SIFIVE_TEST(obj);

    memory_region_init_io(&s->mmio, obj, &sifive_test_ops, s,
                          TYPE_SIFIVE_TEST, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static const TypeInfo sifive_test_info = {
    .name          = TYPE_SIFIVE_TEST,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SiFiveTestState),
    .instance_init = sifive_test_init,
};

static void sifive_test_register_types(void)
{
    type_register_static(&sifive_test_info);
}

type_init(sifive_test_register_types)


/*
 * Create Test device.
 */
DeviceState *sifive_test_create(hwaddr addr)
{
    DeviceState *dev = qdev_create(NULL, TYPE_SIFIVE_TEST);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, addr);
    return dev;
}
