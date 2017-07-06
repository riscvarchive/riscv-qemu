/*
 * QEMU RISC-V - Platform-Level Interrupt Controller (PLIC) Support
 *
 * Author: Vadim Kaushan, admin@disasm.info
 *
 * This interrupt controller currently doesn't support interrupt priorities.
 * Interrupt priorities are hardwired to 1 and priority thresholds are
 * hardwired to zero.
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

/*
  Default configuration: 32 sources and 2 targets.

  Memory map:

  plic
    0000..0fff - priority registers (1024 * 4-byte) - hardwired to value 1
    1000..107f - pending bitmap (1024 * 1-bit)

  plic.target.ie
    0000..007f - interrupt enables for target 0
    ...
    3f80..3fff - interrupt enables for target 127

  plic.target.ctl
    0000..0007 - control registers for target 0
    ...
    1ff8..1fff - control registers for target 127
*/

#define MAX_SOURCES 1024
#define MAX_TARGETS 128

#define PENDING_OFFSET ((MAX_SOURCES) * 4)
#define SOURCES_BITMASK_SIZE ((MAX_SOURCES) / 8)

#define TYPE_PLIC "riscv_plic"
#define PLIC(obj) OBJECT_CHECK(PLICState, (obj), TYPE_PLIC)

struct PLICState;

typedef struct PLICTarget {
    uint32_t enables[MAX_SOURCES/32];
    uint32_t interrupt_id; // starting from one
    qemu_irq irq;
} PLICTarget;

typedef struct PLICState {
    SysBusDevice parent_obj;

    int ninputs;
    //uint32_t priority[MAX_SOURCES];
    uint32_t level[MAX_SOURCES/32];
    uint32_t pending[MAX_SOURCES/32];
    uint32_t ready[MAX_SOURCES/32];

    MemoryRegion iomem; // memory region for priority and pending arrays
    MemoryRegion iomem_ie;  // memory region for interrupt enables
    MemoryRegion iomem_ctl; // memory region for control registers

    int ntargets;
    PLICTarget* targets;
} PLICState;


static void plic_update(PLICState *s)
{
    int i, j;
    for (i = 0; i < s->ntargets; i++)
    {
        PLICTarget *t = &s->targets[i];

        t->interrupt_id = 0;
        for (j = 0; j < s->ninputs; j += 32)
        {
            uint32_t p = t->enables[j] & s->pending[j];
            if (p)
            {
                t->interrupt_id = 1 + (j * 32) + ctz32(p);
                break;
            }
        }

        qemu_set_irq(t->irq, (t->interrupt_id != 0));
    }
}

static void plic_set_irq(void *opaque, int irq, int level)
{
    PLICState *s = (PLICState *)opaque;

    if (irq >= s->ninputs) return;

    int word_offset = irq / 32;
    int word_index = irq % 32;
    uint32_t mask = 1u << word_index;

    if (level)
    {
        s->level[word_offset] |= mask;
    }
    else
    {
        s->level[word_offset] &= ~mask;
    }
    s->pending[word_offset] |= s->level[word_offset] & s->ready[word_offset];

    plic_update(s);
}

static uint64_t plic_common_read(void *opaque, hwaddr offset, unsigned size)
{
    PLICState *s = (PLICState *)opaque;

    if (offset < PENDING_OFFSET) {
        // Priority array
        return 1;
    } else {
        offset -= PENDING_OFFSET;
        return s->pending[offset / 4];
    }
}

static void plic_common_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
    // Ignore writes for now
}

static uint64_t plic_ie_read(void *opaque, hwaddr offset, unsigned size)
{
    PLICState *s = (PLICState *)opaque;

    uint32_t target_index = (offset / SOURCES_BITMASK_SIZE);
    if (target_index >= s->ntargets) return 0;
    PLICTarget *t = &s->targets[target_index];

    offset %= SOURCES_BITMASK_SIZE;
    return t->enables[offset / 4];
}

static void plic_ie_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
    PLICState *s = (PLICState *)opaque;

    uint32_t target_index = (offset / 8);
    if (target_index >= s->ntargets) return;
    PLICTarget *t = &s->targets[target_index];

    offset %= SOURCES_BITMASK_SIZE;
    t->enables[offset / 4] = val;
}

static uint64_t plic_ctl_read(void *opaque, hwaddr offset, unsigned size)
{
    PLICState *s = (PLICState *)opaque;

    uint32_t target_index = (offset / 8);
    if (target_index >= s->ntargets) return 0;
    PLICTarget *t = &s->targets[target_index];

    offset %= 8;
    if (offset == 0)
    {
        // Priority threshold is hardwired to zero
        return 0;
    }
    if (offset == 4)
    {
        // Claim interrupt
        uint32_t id = t->interrupt_id;
        if (id)
        {
            uint32_t index = (id-1)/32;
            uint32_t mask = 1u << ((id-1)%32);
            s->ready[index] &= ~mask;
            s->pending[index] &= ~mask;
            plic_update(s);
        }
        return id;
    }
    return 0;
}

static void plic_ctl_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
    PLICState *s = (PLICState *)opaque;

    offset %= 8;
    if (offset == 0)
    {
        // Ignore write to the priority threshold
        return;
    }
    if (offset == 4)
    {
        // Complete interrupt
        uint32_t id = val;
        if (id)
        {
            if ((id-1) >= s->ninputs) return;
            uint32_t index = (id-1)/32;
            uint32_t mask = 1u << ((id-1)%32);
            s->ready[index] |= mask;
            s->pending[index] |= s->level[index] & s->ready[index];
            plic_update(s);
        }
    }
}

static const MemoryRegionOps plic_common_ops = {
    .read = plic_common_read,
    .write = plic_common_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps plic_ie_ops = {
    .read = plic_ie_read,
    .write = plic_ie_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps plic_ctl_ops = {
    .read = plic_ctl_read,
    .write = plic_ctl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void plic_reset(DeviceState *d)
{
    PLICState *s = PLIC(d);
    int i;

    memset(s->pending, 0, sizeof(s->pending));
    memset(s->ready, 0xff, sizeof(s->ready));
    memset(s->level, 0, sizeof(s->level));

    for (i = 0; i < s->ntargets; i++)
    {
        PLICTarget *t = &s->targets[i];
        memset(t->enables, 0xff, sizeof(t->enables));

        t->interrupt_id = 0;
    }
}

static void plic_init(Object *obj)
{
    DeviceState *dev = DEVICE(obj);
    PLICState *s = PLIC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int i;

    s->ninputs = 32;
    s->ntargets = 2;

    s->targets = malloc(sizeof(PLICTarget) * s->ntargets);
    for (i = 0; i < s->ntargets; i++)
    {
        PLICTarget *t = &s->targets[i];
        sysbus_init_irq(sbd, &t->irq);
    }

    memory_region_init_io(&s->iomem, obj, &plic_common_ops, s, "plic", (MAX_SOURCES * 4) + (MAX_SOURCES / 8));
    sysbus_init_mmio(sbd, &s->iomem);

    memory_region_init_io(&s->iomem_ie, obj, &plic_ie_ops, s, "plic.target.ie", 0x80 * MAX_TARGETS);
    sysbus_init_mmio(sbd, &s->iomem_ie);

    memory_region_init_io(&s->iomem_ctl, obj, &plic_ctl_ops, s, "plic.target.ctl", 0x8 * MAX_TARGETS);
    sysbus_init_mmio(sbd, &s->iomem_ctl);

    qdev_init_gpio_in(dev, plic_set_irq, s->ninputs);
}

static const VMStateDescription vmstate_plic = {
    .name = "plic",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        // TODO
        VMSTATE_END_OF_LIST()
    }
};

static void plic_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = plic_reset;
    dc->vmsd = &vmstate_plic;
}

static const TypeInfo plic_info = {
    .name          = TYPE_PLIC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PLICState),
    .instance_init = plic_init,
    .class_init    = plic_class_init,
};

static void plic_register_types(void)
{
    type_register_static(&plic_info);
}

type_init(plic_register_types)
