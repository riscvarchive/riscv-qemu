/*
 * SiFive CLIC (Core Local Interrupt Controller) interface
 *
 * Copyright (c) 2017 SiFive, Inc.
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

#ifndef HW_SIFIVE_CLIC_H
#define HW_SIFIVE_CLIC_H

#define TYPE_SIFIVE_CLIC "riscv.sifive.clic"

#define SIFIVE_CLIC(obj) \
    OBJECT_CHECK(SiFiveCLICState, (obj), TYPE_SIFIVE_CLIC)

typedef struct CLICActiveInterrupt {
    uint8_t intcfg;
    uint16_t irq;
} CLICActiveInterrupt;

typedef struct SiFiveCLICState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion mmio;
    uint32_t num_harts;
    uint32_t aperture_size;

    /* CLINT/CLIC shared state */
    uint32_t sip_base;
    uint32_t timecmp_base;
    uint32_t time_base;

    /* CLINT/CLIC base addresses */
    uint32_t clint_mmode_base;
    uint32_t clint_smode_base;
    uint32_t clic_mmode_base;
    uint32_t clic_smode_base;

    /* CLIC parameters */
    uint32_t num_sources;   /* 4-1024 */
    uint8_t int_bits;       /* 2-8 */
    uint8_t mode_bits;      /* 0-2 */
    uint8_t level_bits;     /* 0-4 */
    uint8_t vec_bits;       /* 0-1 */

    /* CLIC configuration (decoded from cliccfg) */
    uint8_t *nmbits;        /* nmbits (2-bit), M (0), M/U (0-1), M/S/U (0-2) */
    uint8_t *nlbits;        /* nlbits (3-bit), 0-4 */
    uint8_t *nvbits;        /* nvbits (1-bit), 0-1 */
    uint8_t *npbits;        /* min(8 - nmbits - nlbits, int_bits) */

    /* CLIC State */
    uint8_t *clicintip;     /* mode_base + hart * 0x1000 + 0x000 + i */
    uint8_t *clicintie;     /* mode_base + hart * 0x1000 + 0x400 + i */
    uint8_t *clicintcfg;    /* mode_base + hart * 0x1000 + 0x800 + i */

    /*
     * CLIC per hart active interrupts
     *
     * We maintain per hart lists of enabled interrupts sorted by
     * mode+level+priority. The sorting is done on the configuration path
     * so that the interrupt delivery fastpath can linear scan enabled
     * interrupts in priority order.
     */
    CLICActiveInterrupt *active_list;
    size_t *active_count;

    /* CLIC IRQ handlers */
    qemu_irq *irqs;

} SiFiveCLICState;

enum {
    SIFIVE_CLIC_CLINT_SIZE   = 0x10000,
    SIFIVE_CLIC_HART_SIZE    = 0x1000,

    SIFIVE_CLICINTIP_OFFSET  = 0x0,
    SIFIVE_CLICINTIE_OFFSET  = 0x400,
    SIFIVE_CLICINTCFG_OFFSET = 0x800,
    SIFIVE_CLICCFG_OFFSET    = 0xc00,

    SIFIVE_CLIC_CLINT_MMODE_OFFSET = 0x0,
    SIFIVE_CLIC_CLINT_SMODE_OFFSET = 0x20000,
    SIFIVE_CLIC_CLIC_MMODE_OFFSET = 0x800000,
    SIFIVE_CLIC_CLIC_SMODE_OFFSET = 0xc00000,

    SIFIVE_CLIC_MIN_SOURCES = 4,
    SIFIVE_CLIC_MAX_SOURCES = 1024,
    SIFIVE_CLIC_MIN_INT_BITS = 2,
    SIFIVE_CLIC_MAX_INT_BITS = 8,
    SIFIVE_CLIC_MAX_MODE_BITS = 8,
    SIFIVE_CLIC_MAX_LEVEL_BITS = 8,
    SIFIVE_CLIC_MAX_PRIORITY_BITS = 8,
    SIFIVE_CLIC_MAX_VEC_BITS = 1
};

DeviceState *sifive_clic_create(hwaddr addr, hwaddr size,
    uint32_t num_harts,
    uint32_t sip_base,
    uint32_t timecmp_base,
    uint32_t time_base,
    uint32_t num_sources,
    uint8_t int_bits,
    uint8_t mode_bits,
    uint8_t level_bits,
    uint8_t vec_bits,
    uint32_t clint_mmode_base,
    uint32_t clint_smode_base,
    uint32_t clic_mmode_base,
    uint32_t clic_smode_base);

qemu_irq sifive_clic_get_irq(DeviceState *dev, int hartid, int irq);

#endif
