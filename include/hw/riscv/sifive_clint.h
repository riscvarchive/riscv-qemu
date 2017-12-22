/*
 * SiFive CLINT (Core Local Interruptor) interface
 *
 * Copyright (c) 2016-2017 Sagar Karandikar, sagark@eecs.berkeley.edu
 * Copyright (c) 2017 SiFive, Inc.
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

#ifndef HW_SIFIVE_CLINT_H
#define HW_SIFIVE_CLINT_H

#define TYPE_SIFIVE_CLINT "riscv.sifive.clint"

#define SIFIVE_CLINT(obj) \
    OBJECT_CHECK(SiFiveCLINTState, (obj), TYPE_SIFIVE_CLINT)

typedef struct SiFiveCLINTState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion mmio;
    uint32_t num_harts;
    uint32_t sip_base;
    uint32_t timecmp_base;
    uint32_t time_base;
    uint32_t aperture_size;
} SiFiveCLINTState;

DeviceState *sifive_clint_create(hwaddr addr, hwaddr size, uint32_t num_harts,
    uint32_t sip_base, uint32_t timecmp_base, uint32_t time_base);

enum {
    SIFIVE_SIP_BASE     = 0x0,
    SIFIVE_TIMECMP_BASE = 0x4000,
    SIFIVE_TIME_BASE    = 0xBFF8
};

#endif
