/*
 * SiFive's RISC-V PLIC
 *
 * Copyright (c) 2017 SiFive, Inc.
 *
 * This provides a RISC-V PLIC device
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
#include "qemu/error-report.h"
#include "hw/sysbus.h"
#include "hw/riscv/cpudevs.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/riscv/sifive_plic.h"

static void riscv_plic_set_pending(RISCVPLICState *plic, int irq, bool pending)
{
    uint32_t word = irq >> 5;
    if (pending) {
        plic->pending[word] |= (1 << (irq & 31));
    } else {
        plic->pending[word] &= ~(1 << (irq & 31));
    }
}

static int riscv_plic_num_irqs_pending(RISCVPLICState *plic, uint32_t addrid)
{
    int i, j, count = 0;
    for (i = 0; i < (plic->num_sources >> 5); i++) {
        uint32_t pending_enabled = plic->pending[i] &
            plic->enable[addrid * (plic->num_sources >> 5) + i];
        if (!pending_enabled) continue;
        for (j = 0; j < 32; j++) {
            int irq = (i << 5) + j;
            uint32_t prio = plic->source_priority[irq];
            int enabled = pending_enabled & (1 << j);
            if (enabled && prio >= plic->target_priority[addrid]) {
                count++;
            }
        }
    }
    return count;
}

void riscv_plic_raise_irq(RISCVPLICState *plic, uint32_t irq)
{
    RISCVHartArrayState *soc = plic->soc;
    int i;
    uint32_t prio = plic->source_priority[irq];

    if (irq == 0 || irq >= plic->num_sources) return;

    riscv_plic_set_pending(plic, irq, true);

    /* raise irq on harts where this irq is enabled */
    for (i = 0; i < plic->num_addrs; i++) {
        uint32_t hartid = plic->addr_config[i].hartid;
        PLICMode mode = plic->addr_config[i].mode;
        int enabled = plic->enable[i * (plic->num_sources >> 5) + irq];
        if (enabled && prio >= plic->target_priority[i]) {
            CPURISCVState *env = &soc->harts[hartid].env;
            switch (mode) {
                case PLICMode_M:
                    if ((env->mip & MIP_MEIP) == 0) {
                        env->mip |= MIP_MEIP;
                        qemu_irq_raise(MEIP_IRQ);
                    }
                    break;
                case PLICMode_S:
                    if ((env->mip & MIP_SEIP) == 0) {
                        env->mip |= MIP_SEIP;
                        qemu_irq_raise(SEIP_IRQ);
                    }
                    break;
                default:
                    error_report("plic: raise irq invalid mode: %d", mode);
            }
        }
    }
}

void riscv_plic_lower_irq(RISCVPLICState *plic, uint32_t irq)
{
    RISCVHartArrayState *soc = plic->soc;
    int i;

    if (irq == 0 || irq >= plic->num_sources) return;

    riscv_plic_set_pending(plic, irq, false);

    /* only lower irq on harts with no irqs pending */
    for (i = 0; i < plic->num_addrs; i++) {
        uint32_t hartid = plic->addr_config[i].hartid;
        PLICMode mode = plic->addr_config[i].mode;
        CPURISCVState *env = &soc->harts[hartid].env;
        if (riscv_plic_num_irqs_pending(plic, i) > 0) continue;
        switch (mode) {
            case PLICMode_M:
                if (env->mip & MIP_MEIP) {
                    env->mip &= ~MIP_MEIP;
                    qemu_irq_lower(MEIP_IRQ);
                }
                break;
            case PLICMode_S:
                if (env->mip & MIP_SEIP) {
                    env->mip &= ~MIP_SEIP;
                    qemu_irq_lower(SEIP_IRQ);
                }
                break;
            default:
                error_report("plic: raise irq invalid mode: %d", mode);
        }
    }
}

static uint32_t riscv_plic_claim(RISCVPLICState *plic, uint32_t addrid)
{
    int i, j;
    for (i = 0; i < (plic->num_sources >> 5); i++) {
        uint32_t pending_enabled = plic->pending[i] &
            plic->enable[addrid * (plic->num_sources >> 5) + i];
        if (!pending_enabled) continue;
        for (j = 0; j < 32; j++) {
            int irq = (i << 5) + j;
            uint32_t prio = plic->source_priority[irq];
            int enabled = pending_enabled & (1 << j);
            if (enabled && prio >= plic->target_priority[addrid]) {
                riscv_plic_lower_irq(plic, irq);
                return irq;
            }
        }
    }
    return 0;
}

/* CPU wants to read rtc or timecmp register */
static uint64_t riscv_plic_read(void *opaque, hwaddr addr, unsigned size)
{
    RISCVPLICState *plic = opaque;

    /* writes must be 4 byte words */
    if ((addr & 0x3) != 0) goto err;

    if (addr >= plic->priority_base && /* 4 bytes per source */
        addr < plic->priority_base + (plic->num_sources << 2))
    {
        uint32_t irq = (addr - plic->priority_base) >> 2;
        return plic->source_priority[irq];
    }
    else if (addr >= plic->pending_base && /* 1 bit per source */
             addr < plic->pending_base + (plic->num_sources >> 3))
    {
        uint32_t word = (addr - plic->priority_base) >> 2;
        return plic->pending[word];
    }
    else if (addr >= plic->enable_base && /* 1 bit per source */
             addr < plic->enable_base + plic->num_addrs * 0x80)
    {
        if ((addr & 0x7f) < (plic->num_sources >> 3)) {
            uint32_t addrid = (addr - plic->enable_base) >> 7;
            uint32_t word = addrid * (plic->num_sources >> 5) + (addr & 0x7f);
            return plic->enable[word];
        }
    }
    else if (addr >= plic->claim_base && /* 1 bit per source */
             addr < plic->claim_base + plic->num_addrs * 0x1000)
    {
        uint32_t addrid = (addr - plic->claim_base) >> 12;
        if ((addr & 0xfff) == 0) {
            return plic->target_priority[addrid];
        } else if ((addr & 0xfff) == 4) {
            return riscv_plic_claim(plic, addrid);
        }
    }

err:
    error_report("plic: invalid register read: %08x", (uint32_t)addr);
    return 0;
}

/* CPU wrote to rtc or timecmp register */
static void riscv_plic_write(void *opaque, hwaddr addr, uint64_t value,
        unsigned size)
{
    RISCVPLICState *plic = opaque;

    /* writes must be 4 byte words */
    if ((addr & 0x3) != 0) goto err;

    if (addr >= plic->priority_base && /* 4 bytes per source */
        addr < plic->priority_base + (plic->num_sources << 2))
    {
        uint32_t irq = (addr - plic->priority_base) >> 2;
        plic->source_priority[irq] = value & 7;
        return;
    }
    else if (addr >= plic->pending_base && /* 1 bit per source */
             addr < plic->pending_base + (plic->num_sources >> 3))
    {
        error_report("plic: invalid pending write: %08x", (uint32_t)addr);
        return;
    }
    else if (addr >= plic->enable_base && /* 1 bit per source */
             addr < plic->enable_base + plic->num_addrs * 0x80)
    {
        if ((addr & 0x7f) < (plic->num_sources >> 3)) {
            uint32_t addrid = (addr - plic->enable_base) >> 7;
            uint32_t word = addrid * (plic->num_sources >> 5) + (addr & 0x7f);
            plic->enable[word] = value;
            return;
        }
    }
    else if (addr >= plic->claim_base && /* 1 bit per source */
             addr < plic->claim_base + plic->num_addrs * 0x1000)
    {
        uint32_t addrid = (addr - plic->claim_base) >> 12;
        if ((addr & 0xfff) == 0) {
            plic->target_priority[addrid] = value & 7;
            return;
        } else if ((addr & 0xfff) == 4) {
            error_report("plic: invalid claim register write: %08x",
                (uint32_t)addr);
            return;
        }
    }

err:
    error_report("plic: invalid register write: %08x", (uint32_t)addr);
}

static const MemoryRegionOps riscv_plic_ops = {
    .read = riscv_plic_read,
    .write = riscv_plic_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static Property riscv_plic_properties[] = {
    DEFINE_PROP_PTR("soc", RISCVPLICState, soc),
    DEFINE_PROP_STRING("hart-config", RISCVPLICState, hart_config),
    DEFINE_PROP_UINT32("num-sources", RISCVPLICState, num_sources, 0),
    DEFINE_PROP_UINT32("priority-base", RISCVPLICState, priority_base, 0),
    DEFINE_PROP_UINT32("pending-base", RISCVPLICState, pending_base, 0),
    DEFINE_PROP_UINT32("enable-base", RISCVPLICState, enable_base, 0),
    DEFINE_PROP_UINT32("claim-base", RISCVPLICState, claim_base, 0),
    DEFINE_PROP_UINT32("aperture-size", RISCVPLICState, aperture_size, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static PLICMode char_to_mode(char c)
{
    switch (c) {
        case 'U': return PLICMode_U;
        case 'S': return PLICMode_S;
        case 'H': return PLICMode_H;
        case 'M': return PLICMode_M;
        default:
            error_report("plic: invalid mode '%c'", c);
            exit(1);
    }
}

/*
 * parse PLIC hart/mode address offset config
 *
 * "M"              1 hart with M mode
 * "MS,MS"          2 harts, 0-1 with M and S mode
 * "M,MS,MS,MS,MS"  5 harts, 0 with M mode, 1-5 with M and S mode
 */
static void parse_hart_config(RISCVPLICState *plic)
{
    RISCVHartArrayState *soc = plic->soc;
    int addrid, hartid, modes;
    const char *p;
    char c;

    /* count and validate hart/mode combinations */
    addrid = 0, hartid = 0, modes = 0;
    p = plic->hart_config;
    while ((c = *p++)) {
        if (c == ',') {
            addrid += __builtin_popcount(modes);
            modes = 0;
            hartid++;
        } else {
            int m = char_to_mode(c);
            if ((modes | m) == m) {
                error_report("plic: duplicate mode '%c' in config: %s",
                             *p, plic->hart_config);
                exit(1);
            }
            modes |= m;
        }
    }
    if (modes) {
        addrid += __builtin_popcount(modes);
    }
    hartid++;
    if (hartid != soc->num_harts) {
        error_report("plic: found %d hart config items, require %d: %s",
                     hartid, soc->num_harts, plic->hart_config);
        exit(1);
    }

    /* store hart/mode combinations */
    plic->num_addrs = addrid;
    plic->addr_config = g_new(PLICAddr, plic->num_addrs);
    addrid = 0, hartid = 0;
    p = plic->hart_config;
    while ((c = *p++)) {
        if (c == ',') {
            hartid++;
        } else {
            plic->addr_config[addrid].addrid = addrid;
            plic->addr_config[addrid].hartid = hartid;
            plic->addr_config[addrid].mode = char_to_mode(*p);
            addrid++;
        }
    }
}

static void riscv_plic_realize(DeviceState *dev, Error **errp)
{
    RISCVPLICState *plic = RISCV_PLIC(dev);
    memory_region_init_io(&plic->mmio, OBJECT(dev), &riscv_plic_ops, plic,
                          TYPE_RISCV_PLIC, plic->aperture_size);
    parse_hart_config(plic);
    plic->source_priority = g_new(uint32_t, plic->num_sources);
    plic->target_priority = g_new(uint32_t, plic->num_addrs);
    plic->pending = g_new(uint32_t, plic->num_sources >> 5);
    plic->enable = g_new(uint32_t, (plic->num_sources * plic->num_addrs) >> 5);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &plic->mmio);
}

static void riscv_plic_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->props = riscv_plic_properties;
    dc->realize = riscv_plic_realize;
}

static const TypeInfo riscv_plic_info = {
    .name          = TYPE_RISCV_PLIC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RISCVPLICState),
    .class_init    = riscv_plic_class_init,
};

static void riscv_plic_register_types(void)
{
    type_register_static(&riscv_plic_info);
}

type_init(riscv_plic_register_types)

/*
 * Create PLIC device.
 */
DeviceState *riscv_plic_create(hwaddr addr, RISCVHartArrayState *soc,
    char *hart_config, uint32_t num_sources, uint32_t priority_base,
    uint32_t pending_base, uint32_t enable_base, uint32_t claim_base,
    uint32_t aperture_size)
{
    DeviceState *dev = qdev_create(NULL, TYPE_RISCV_PLIC);
    qdev_prop_set_ptr(dev, "soc", soc);
    qdev_prop_set_string(dev, "hart-config", hart_config);
    qdev_prop_set_uint32(dev, "num-sources", num_sources);
    qdev_prop_set_uint32(dev, "priority-base", priority_base);
    qdev_prop_set_uint32(dev, "pending-base", pending_base);
    qdev_prop_set_uint32(dev, "enable-base", enable_base);
    qdev_prop_set_uint32(dev, "claim-base", claim_base);
    qdev_prop_set_uint32(dev, "aperture-size", aperture_size);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, addr);
    return dev;
}
