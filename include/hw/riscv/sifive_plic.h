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

#ifndef _RISCV_PLIC_H_
#define _RISCV_PLIC_H_
#include "target-riscv/cpu.h"

#define TYPE_RISCV_PLIC "riscv.plic"

#define RISCV_PLIC(obj) \
    OBJECT_CHECK(RISCVPLICState, (obj), TYPE_RISCV_HART_ARRAY)

typedef enum PLICMode {
    PLICMode_U,
    PLICMode_S,
    PLICMode_H,
    PLICMode_M
} PLICMode;

typedef struct PLICAddr {
    uint32_t  addrid;
    uint32_t  hartid;
    PLICMode mode;
} PLICAddr;

typedef struct RISCVPLICState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion mmio;
    uint32_t num_addrs;
    PLICAddr *addr_config;
    uint32_t *source_priority;
    uint32_t *target_priority;
    uint32_t *pending;
    uint32_t *enable;

    /* config */
    void *soc;
    char* hart_config;
    uint32_t num_sources;
    uint32_t priority_base;
    uint32_t pending_base;
    uint32_t enable_base;
    uint32_t claim_base;
    uint32_t aperture_size;
} RISCVPLICState;

void riscv_plic_raise_irq(RISCVPLICState *plic, uint32_t irq);
void riscv_plic_lower_irq(RISCVPLICState *plic, uint32_t irq);

DeviceState *riscv_plic_create(hwaddr addr, RISCVHartArrayState *soc,
    char *hart_config, uint32_t num_sources, uint32_t priority_base,
    uint32_t pending_base, uint32_t enable_base, uint32_t claim_base,
    uint32_t aperture_size);

#endif

