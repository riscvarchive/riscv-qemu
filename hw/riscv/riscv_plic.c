/*
 * QEMU model of the SiFive and RISCVEMU PLICs
 *
 * Copyright (c) 2017 Stefan O'Rear
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
#include "trace.h"

/*
 * The RISC-V spec defines a "Platform level interrupt controller" and gives
 * functional behavior, but not a memory map.  The [SiFive documentation][1]
 * gives a memory map for their implementation; RISCVEMU's PLIC behaves as
 * a small window into the PLIC space and does not implement the whole thing.
 *
 * [1]: https://static.dev.sifive.com/SiFive-E3-Coreplex-v1.2.pdf#page=22
 */

#define TYPE_RISCV_PLIC "riscv.plic"
#define RISCV_PLIC(obj) \
    OBJECT_CHECK(RiscvPLIC, (obj), TYPE_RISCV_PLIC)

/*
 * TODO: Refactor this using subclasses to allow different PLIC instantiations
 * to coexist.
 */
#define RISCV_PLIC_MINI 1
#define RISCV_PLIC_TARGETS 1
#define RISCV_PLIC_SOURCES 31

#define RISCV_PLIC_FULL_SIZE (64*1024*1024)
#define RISCV_PLIC_MINI_SIZE 8
#define RISCV_PLIC_MINI_OFFSET 0x200000

#define RISCV_PLIC_O_PRIORITY 0x0
#define RISCV_PLIC_O_PENDING 0x1000
#define RISCV_PLIC_O_ENABLES 0x2000
#define RISCV_PLIC_O_DELEGATE 0x200000

#define RISCV_PLIC_SIZE_ENABLE 128
#define RISCV_PLIC_SIZE_DELEGATE 4096
#define RISCV_PLIC_O_D_THRESHOLD 0
#define RISCV_PLIC_O_D_CLAIM_COMPLETE 4

/* bit position 0 is reserved (PLIC thinks sources are 1-based) */
#define ENABLE_WORD(src) ((uint32_t)(src + 1) / 32)
#define ENABLE_MASK(src) (1 << ((uint32_t)(src + 1) % 32))
#define ENABLE_WORDS 32

typedef struct RiscvPLICTarget {
    qemu_irq target_irq;
    uint32_t prio_threshold;
    uint32_t enables[ENABLE_WORDS];
} RiscvPLICTarget;

typedef struct RiscvPLICSource {
    uint32_t priority;
    bool masked;
    bool raised;
} RiscvPLICSource;

typedef struct RiscvPLIC {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    uint32_t mini_layout;

    uint32_t n_sources;
    RiscvPLICSource *sources;
    uint32_t n_targets;
    RiscvPLICTarget *targets;
} RiscvPLIC;

static void riscv_plic_reset(DeviceState *d)
{
    RiscvPLIC *p = RISCV_PLIC(d);
    int s, t;

    for (s = 0; s < p->n_sources; s++) {
        p->sources[s].masked = false;
        p->sources[s].raised = false;
        p->sources[s].priority = 1;
    }

    for (t = 0; t < p->n_targets; t++) {
        p->targets[t].prio_threshold = 0;
        for (s = 0; s < ENABLE_WORDS; s++) {
            p->targets[t].enables[s] = ~0;
        }
    }
}

static uint32_t riscv_plic_unclaimed_id(RiscvPLIC *p, RiscvPLICTarget *tgt)
{
    uint32_t src_i;
    RiscvPLICSource *src;
    uint32_t best_prio = tgt->prio_threshold;
    uint32_t best_src = 0;

    for (src_i = 0; src_i < p->n_sources; src_i++) {
        src = &p->sources[src_i];
        if (src->raised && !src->masked && src->priority > best_prio && (tgt->enables[ENABLE_WORD(src_i)] & ENABLE_MASK(src_i))) {
            best_prio = src->priority;
            best_src = src_i + 1;
        }
    }

    return best_src;
}

static void riscv_plic_update_irq(RiscvPLIC *p)
{
    uint32_t tgt_i, unclaimed;
    RiscvPLICTarget *tgt;

    for (tgt_i = 0; tgt_i < p->n_targets; tgt_i++) {
        tgt = &p->targets[tgt_i];
        unclaimed = riscv_plic_unclaimed_id(p, tgt);
        trace_riscv_plic_unclaimed(tgt_i, unclaimed);

        if (unclaimed > 0) {
            qemu_irq_raise(tgt->target_irq);
        } else {
            qemu_irq_lower(tgt->target_irq);
        }
    }
}

static void riscv_plic_irq_handler(void *opaque, int irq, int level)
{
    RiscvPLIC *p = opaque;
    assert(irq < p->n_sources);
    trace_riscv_plic_interrupt(irq, level);

    p->sources[irq].raised = level != 0;
    riscv_plic_update_irq(p);
}

static uint64_t
riscv_plic_read(void *opaque, hwaddr addr, unsigned int size)
{
    RiscvPLIC *p = opaque;
    hwaddr abs_addr;
    uint32_t min_src_ix, src_ix, tgt_ix, retval = 0, pending_id;
    if (p->mini_layout) {
        addr += RISCV_PLIC_MINI_OFFSET;
    }
    assert(addr % 4 == 0);
    abs_addr = addr;

    if (addr < RISCV_PLIC_O_PENDING) {
        /* Read from priorities */
        src_ix = (addr / 4) - 1;
        if (src_ix < p->n_sources) {
            retval = p->sources[src_ix].priority;
        }
    } else if (addr < RISCV_PLIC_O_ENABLES) {
        /* Read from pending bitmask */
        addr -= RISCV_PLIC_O_PENDING;
        min_src_ix = addr * 8 - 1;
        for (src_ix = min_src_ix; src_ix < min_src_ix + 32; src_ix++) {
            if (src_ix < p->n_sources && p->sources[src_ix].raised && !p->sources[src_ix].masked) {
                retval |= 1 << ((src_ix - min_src_ix) % 32);
            }
        }
    } else if (addr < RISCV_PLIC_O_DELEGATE) {
        /* Read from per-target enables */
        addr -= RISCV_PLIC_O_ENABLES;
        tgt_ix = addr / RISCV_PLIC_SIZE_ENABLE;
        addr -= tgt_ix * RISCV_PLIC_SIZE_ENABLE;
        if (tgt_ix < p->n_targets) {
            assert(addr / 4 < ENABLE_WORDS);
            retval = p->targets[tgt_ix].enables[addr / 4];
        }
    } else {
        /* Read from delegated register */
        addr -= RISCV_PLIC_O_DELEGATE;
        tgt_ix = addr / RISCV_PLIC_SIZE_DELEGATE;
        addr -= tgt_ix * RISCV_PLIC_SIZE_DELEGATE;

        if (tgt_ix < p->n_targets) {
            if (addr == RISCV_PLIC_O_D_THRESHOLD) {
                retval = p->targets[tgt_ix].prio_threshold;
            } else if (addr == RISCV_PLIC_O_D_CLAIM_COMPLETE) {
                pending_id = riscv_plic_unclaimed_id(p, &p->targets[tgt_ix]);
                if (pending_id > 0) {
                    retval = pending_id;
                    p->sources[pending_id - 1].masked = true;
                    riscv_plic_update_irq(p);
                }
            }
        }
    }
    trace_riscv_plic_read(abs_addr, retval);
    return retval;
}

static void
riscv_plic_write(void *opaque, hwaddr addr, uint64_t val64, unsigned int size)
{
    RiscvPLIC *p = opaque;
    uint32_t min_src_ix, src_ix, tgt_ix, ok_mask = 0;

    if (p->mini_layout) {
        addr += RISCV_PLIC_MINI_OFFSET;
    }

    trace_riscv_plic_write(addr, val64);
    assert(addr % 4 == 0);

    if (addr < RISCV_PLIC_O_PENDING) {
        /* Write priorities */
        src_ix = (addr / 4) - 1;
        if (src_ix < p->n_sources) {
            p->sources[src_ix].priority = val64;
            riscv_plic_update_irq(p);
        }
    } else if (addr < RISCV_PLIC_O_ENABLES) {
        /* Write to pending bitmask - ignored */
    } else if (addr < RISCV_PLIC_O_DELEGATE) {
        /* Write to per-target enables */
        addr -= RISCV_PLIC_O_ENABLES;
        tgt_ix = addr / RISCV_PLIC_SIZE_ENABLE;
        addr -= tgt_ix * RISCV_PLIC_SIZE_ENABLE;

        min_src_ix = addr * 8 - 1;
        for (src_ix = min_src_ix; src_ix < min_src_ix + 32; src_ix++) {
            if (src_ix < p->n_sources) {
                ok_mask |= 1 << (src_ix - min_src_ix);
            }
        }

        assert(addr / 4 < ENABLE_WORDS);
        if (tgt_ix < p->n_targets) {
            p->targets[tgt_ix].enables[addr / 4] = val64 & ok_mask;
        }
    } else {
        /* Write to delegated register */
        addr -= RISCV_PLIC_O_DELEGATE;
        tgt_ix = addr / RISCV_PLIC_SIZE_DELEGATE;
        addr -= tgt_ix * RISCV_PLIC_SIZE_DELEGATE;

        if (tgt_ix < p->n_targets) {
            if (addr == RISCV_PLIC_O_D_THRESHOLD) {
                p->targets[tgt_ix].prio_threshold = val64;
                riscv_plic_update_irq(p);
            } else if (addr == RISCV_PLIC_O_D_CLAIM_COMPLETE) {
                src_ix = val64 - 1;
                /* Completes are ignored if the interrupt is disabled */
                if (src_ix < p->n_sources && (p->targets[tgt_ix].enables[ENABLE_WORD(src_ix)] & ENABLE_MASK(src_ix))) {
                    p->sources[src_ix].masked = false;
                    riscv_plic_update_irq(p);
                }
            }
        }
    }
}

static const MemoryRegionOps riscv_plic_ops = {
    .read = riscv_plic_read,
    .write = riscv_plic_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static void riscv_plic_init(Object *obj)
{
    RiscvPLIC *p = RISCV_PLIC(obj);
    DeviceState *dev = DEVICE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int i;

    p->mini_layout = RISCV_PLIC_MINI;
    p->n_sources = RISCV_PLIC_SOURCES;
    p->n_targets = RISCV_PLIC_TARGETS;

    p->targets = g_malloc0(sizeof p->targets[0] * p->n_targets);
    p->sources = g_malloc0(sizeof p->sources[0] * p->n_sources);

    qdev_init_gpio_in(dev, riscv_plic_irq_handler, p->n_sources);

    riscv_plic_reset(dev);

    for (i = 0; i < p->n_targets; i++) {
        sysbus_init_irq(sbd, &p->targets[i].target_irq);
    }

    memory_region_init_io(&p->mmio, obj, &riscv_plic_ops, p,
                          "riscv.riscv_plic", p->mini_layout ? RISCV_PLIC_MINI_SIZE : RISCV_PLIC_FULL_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &p->mmio);
}

static const VMStateDescription vmstate_riscv_plic_source = {
    .name = "riscv-plic-source",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(priority, RiscvPLICSource),
        VMSTATE_BOOL(masked, RiscvPLICSource),
        VMSTATE_BOOL(raised, RiscvPLICSource),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_riscv_plic_target = {
    .name = "riscv-plic-target",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(prio_threshold, RiscvPLICTarget),
        VMSTATE_UINT32_ARRAY(enables, RiscvPLICTarget, 32),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_riscv_plic = {
    .name = "riscv-plic",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_EQUAL(mini_layout, RiscvPLIC),
        VMSTATE_UINT32_EQUAL(n_sources, RiscvPLIC),
        VMSTATE_UINT32_EQUAL(n_targets, RiscvPLIC),
        VMSTATE_STRUCT_VARRAY_POINTER_UINT32(sources, RiscvPLIC, n_sources, vmstate_riscv_plic_source, RiscvPLICSource),
        VMSTATE_STRUCT_VARRAY_POINTER_UINT32(targets, RiscvPLIC, n_targets, vmstate_riscv_plic_target, RiscvPLICTarget),
        VMSTATE_END_OF_LIST()
    }
};

static void riscv_plic_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = riscv_plic_reset;
    dc->vmsd = &vmstate_riscv_plic;
}

static const TypeInfo riscv_plic_info = {
    .name          = TYPE_RISCV_PLIC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RiscvPLIC),
    .instance_init = riscv_plic_init,
    .class_init    = riscv_plic_class_init,
};

static void riscv_plic_register_types(void)
{
    type_register_static(&riscv_plic_info);
}

type_init(riscv_plic_register_types)
