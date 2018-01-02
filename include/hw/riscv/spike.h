/*
 * SiFive U500 series machine interface
 *
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

#ifndef HW_SPIKE_H
#define HW_SPIKE_H

#define TYPE_RISCV_SPIKE_V1_09_1_BOARD "riscv.spike_v1_9"
#define TYPE_RISCV_SPIKE_V1_10_0_BOARD "riscv.spike_v1_10"

#define SPIKE(obj) \
    OBJECT_CHECK(SpikeState, (obj), TYPE_RISCV_SPIKE_BOARD)

typedef struct {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    RISCVHartArrayState soc;
    void *fdt;
    int fdt_size;
} SpikeState;


enum {
    SPIKE_MROM,
    SPIKE_CLINT,
    SPIKE_DRAM
};

#endif
