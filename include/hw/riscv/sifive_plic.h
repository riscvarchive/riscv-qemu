/*
 * SiFive PLIC (Platform Level Interrupt Controller) interface
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

#ifndef HW_SIFIVE_PLIC_H
#define HW_SIFIVE_PLIC_H

#include "hw/irq.h"

#define TYPE_SIFIVE_PLIC "riscv.sifive.plic"

#define SIFIVE_PLIC(obj) \
    OBJECT_CHECK(SiFivePLICState, (obj), TYPE_SIFIVE_PLIC)

typedef enum PLICMode {
    PLICMode_U,
    PLICMode_S,
    PLICMode_H,
    PLICMode_M
} PLICMode;

typedef struct PLICAddr {
    uint32_t addrid;
    uint32_t hartid;
    PLICMode mode;
} PLICAddr;

typedef struct SiFivePLICState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion mmio;
    uint32_t num_addrs;
    uint32_t bitfield_words;
    PLICAddr *addr_config;
    uint32_t *source_priority;
    uint32_t *target_priority;
    uint32_t *pending;
    uint32_t *claimed;
    uint32_t *enable;
    QemuMutex lock;
    qemu_irq *irqs;

    /* config */
    char *hart_config;
    uint32_t num_sources;
    uint32_t num_priorities;
    uint32_t priority_base;
    uint32_t pending_base;
    uint32_t enable_base;
    uint32_t enable_stride;
    uint32_t context_base;
    uint32_t context_stride;
    uint32_t aperture_size;
} SiFivePLICState;

void sifive_plic_raise_irq(SiFivePLICState *plic, uint32_t irq);
void sifive_plic_lower_irq(SiFivePLICState *plic, uint32_t irq);

DeviceState *sifive_plic_create(hwaddr addr, char *hart_config,
    uint32_t num_sources, uint32_t num_priorities,
    uint32_t priority_base, uint32_t pending_base,
    uint32_t enable_base, uint32_t enable_stride,
    uint32_t context_base, uint32_t context_stride,
    uint32_t aperture_size);

#endif

