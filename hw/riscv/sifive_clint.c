/*
 * SiFive CLINT (Core Local Interruptor)
 *
 * Copyright (c) 2016-2017 Sagar Karandikar, sagark@eecs.berkeley.edu
 * Copyright (c) 2017 SiFive, Inc.
 *
 * This provides real-time clock, timer and interprocessor interrupts.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "target/riscv/cpu.h"
#include "hw/riscv/sifive_clint.h"
#include "qemu/timer.h"

static uint64_t cpu_riscv_read_rtc(void)
{
    return muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
        SIFIVE_CLINT_TIMEBASE_FREQ, NANOSECONDS_PER_SECOND);
}

static void sifive_clint_mtimecmp_cb(void *cpu)
{
    riscv_cpu_update_mip((RISCVCPU *)cpu, MIP_MTIP, BOOL_TO_MASK(1));
}

/*
 * Called when timecmp is written to update the QEMU timer or immediately
 * trigger timer interrupt if mtimecmp <= current timer value.
 */
static void sifive_clint_write_timecmp(RISCVCPU *cpu, uint64_t value)
{
    CPURISCVState *env = &cpu->env;

    uint64_t rtc = cpu_riscv_read_rtc();
    uint64_t cmp = env->mtimecmp = value;
    uint64_t diff = cmp - rtc;
    uint64_t next_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
        muldiv64(diff, NANOSECONDS_PER_SECOND, SIFIVE_CLINT_TIMEBASE_FREQ);

    if (cmp <= rtc) {
        /* if we're setting a timecmp value in the "past",
           immediately raise the timer interrupt */
        riscv_cpu_update_mip(cpu, MIP_MTIP, BOOL_TO_MASK(1));
    } else {
        /* otherwise, set up the future timer interrupt */
        riscv_cpu_update_mip(cpu, MIP_MTIP, BOOL_TO_MASK(0));
        timer_mod(env->mtimer, next_ns);
    }
}

/* CPU wants to read rtc or timecmp register */
static uint64_t sifive_clint_read(void *opaque, hwaddr addr, unsigned size)
{
    SiFiveCLINTState *clint = opaque;

    /* reads must be 4 byte aligned words */
    if ((addr & 0x3) != 0 || size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
            "clint: invalid read size %u: 0x%" HWADDR_PRIx "\n", size, addr);
        return 0;
    }

    if (addr >= clint->sip_base &&
        addr < clint->sip_base + (clint->num_harts << 2)) {
        size_t hartid = (addr - clint->sip_base) >> 2;
        CPUState *cpu = qemu_get_cpu(hartid);
        CPURISCVState *env = cpu ? cpu->env_ptr : NULL;
        if (!env) {
            qemu_log_mask(LOG_GUEST_ERROR,
                "clint: invalid sip hartid: %zu\n", hartid);
        } else if ((addr & 0x3) == 0) {
            return (env->mip & MIP_MSIP) > 0;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                "clint: invalid sip read: 0x%" HWADDR_PRIx "\n", addr);
            return 0;
        }
    } else if (addr >= clint->timecmp_base &&
        addr < clint->timecmp_base + (clint->num_harts << 3)) {
        size_t hartid = (addr - clint->timecmp_base) >> 3;
        CPUState *cpu = qemu_get_cpu(hartid);
        CPURISCVState *env = cpu ? cpu->env_ptr : NULL;
        if (!env) {
            qemu_log_mask(LOG_GUEST_ERROR,
                "clint: invalid timecmp hartid: %zu\n", hartid);
        } else if ((addr & 0x7) == 0) {
            /* timecmp_lo */
            uint64_t timecmp = env->mtimecmp;
            return timecmp & 0xFFFFFFFF;
        } else if ((addr & 0x7) == 4) {
            /* timecmp_hi */
            uint64_t timecmp = env->mtimecmp;
            return (timecmp >> 32) & 0xFFFFFFFF;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                "clint: invalid read: 0x%" HWADDR_PRIx "\n", addr);
            return 0;
        }
    } else if (addr == clint->time_base) {
        /* time_lo */
        return cpu_riscv_read_rtc() & 0xFFFFFFFF;
    } else if (addr == clint->time_base + 4) {
        /* time_hi */
        return (cpu_riscv_read_rtc() >> 32) & 0xFFFFFFFF;
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
            "clint: invalid read: 0x%" HWADDR_PRIx "\n", addr);
    }

    return 0;
}

/* CPU wrote to rtc or timecmp register */
static void sifive_clint_write(void *opaque, hwaddr addr, uint64_t value,
        unsigned size)
{
    SiFiveCLINTState *clint = opaque;

    /* writes must be 4 byte aligned words */
    if ((addr & 0x3) != 0 || size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
            "clint: invalid write size %u: 0x%" HWADDR_PRIx "\n", size, addr);
        return;
    }

    if (addr >= clint->sip_base &&
        addr < clint->sip_base + (clint->num_harts << 2)) {
        size_t hartid = (addr - clint->sip_base) >> 2;
        CPUState *cpu = qemu_get_cpu(hartid);
        CPURISCVState *env = cpu ? cpu->env_ptr : NULL;
        if (!env) {
            qemu_log_mask(LOG_GUEST_ERROR,
                "clint: invalid sip hartid: %zu\n", hartid);
        } else if ((addr & 0x3) == 0) {
            riscv_cpu_update_mip(RISCV_CPU(cpu), MIP_MSIP, BOOL_TO_MASK(value));
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                "clint: invalid sip write: 0x%" HWADDR_PRIx "\n", addr);
        }
        return;
    } else if (addr >= clint->timecmp_base &&
        addr < clint->timecmp_base + (clint->num_harts << 3)) {
        size_t hartid = (addr - clint->timecmp_base) >> 3;
        CPUState *cpu = qemu_get_cpu(hartid);
        CPURISCVState *env = cpu ? cpu->env_ptr : NULL;
        if (!env) {
            qemu_log_mask(LOG_GUEST_ERROR,
                "clint: invalid timecmp hartid: %zu\n", hartid);
        } else if ((addr & 0x7) == 0) {
            /* timecmp_lo */
            uint64_t timecmp_hi = env->mtimecmp >> 32;
            sifive_clint_write_timecmp(RISCV_CPU(cpu),
                timecmp_hi << 32 | (value & 0xFFFFFFFF));
            return;
        } else if ((addr & 0x7) == 4) {
            /* timecmp_hi */
            uint64_t timecmp_lo = env->mtimecmp;
            sifive_clint_write_timecmp(RISCV_CPU(cpu),
                value << 32 | (timecmp_lo & 0xFFFFFFFF));
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                "clint: invalid timecmp write: 0x%" HWADDR_PRIx "\n", addr);
        }
        return;
    } else if (addr == clint->time_base) {
        /* time_lo */
        qemu_log_mask(LOG_UNIMP, "clint: time_lo write not implemented\n");
    } else if (addr == clint->time_base + 4) {
        /* time_hi */
        qemu_log_mask(LOG_UNIMP, "clint: time_hi write not implemented");
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
            "clint: invalid write: 0x%" HWADDR_PRIx "\n", addr);
    }
}

static const MemoryRegionOps sifive_clint_ops = {
    .read = sifive_clint_read,
    .write = sifive_clint_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static Property sifive_clint_properties[] = {
    DEFINE_PROP_UINT32("num-harts", SiFiveCLINTState, num_harts, 0),
    DEFINE_PROP_UINT32("sip-base", SiFiveCLINTState, sip_base, 0),
    DEFINE_PROP_UINT32("timecmp-base", SiFiveCLINTState, timecmp_base, 0),
    DEFINE_PROP_UINT32("time-base", SiFiveCLINTState, time_base, 0),
    DEFINE_PROP_UINT32("aperture-size", SiFiveCLINTState, aperture_size, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void sifive_clint_realize(DeviceState *dev, Error **errp)
{
    SiFiveCLINTState *s = SIFIVE_CLINT(dev);
    memory_region_init_io(&s->mmio, OBJECT(dev), &sifive_clint_ops, s,
                          TYPE_SIFIVE_CLINT, s->aperture_size);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);
}

static void sifive_clint_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = sifive_clint_realize;
    dc->props = sifive_clint_properties;
}

static const TypeInfo sifive_clint_info = {
    .name          = TYPE_SIFIVE_CLINT,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SiFiveCLINTState),
    .class_init    = sifive_clint_class_init,
};

static void sifive_clint_register_types(void)
{
    type_register_static(&sifive_clint_info);
}

type_init(sifive_clint_register_types)


/*
 * Create CLINT device.
 */
DeviceState *sifive_clint_create(hwaddr addr, hwaddr size, uint32_t num_harts,
    uint32_t sip_base, uint32_t timecmp_base, uint32_t time_base)
{
    int i;
    for (i = 0; i < num_harts; i++) {
        CPUState *cpu = qemu_get_cpu(i);
        CPURISCVState *env = cpu ? cpu->env_ptr : NULL;
        if (!env) {
            continue;
        }
        env->mtimer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                   &sifive_clint_mtimecmp_cb, cpu);
        env->mtimecmp = 0;
    }

    DeviceState *dev = qdev_create(NULL, TYPE_SIFIVE_CLINT);
    qdev_prop_set_uint32(dev, "num-harts", num_harts);
    qdev_prop_set_uint32(dev, "sip-base", sip_base);
    qdev_prop_set_uint32(dev, "timecmp-base", timecmp_base);
    qdev_prop_set_uint32(dev, "time-base", time_base);
    qdev_prop_set_uint32(dev, "aperture-size", size);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, addr);
    return dev;
}
