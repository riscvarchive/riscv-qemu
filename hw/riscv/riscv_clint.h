/*
 * QEMU RISC-V CLINT device emulator
 * Author: Daire McNamara, daire.mcnamara@emdalo.com
 *
 * This provides a RISC-V CLINT device:
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
#ifndef _RISCV_CLINT_H_
#define _RISCV_CLINT_H_

#include "target-riscv/cpu.h"

#define CLINT_MTIMECMP_OFFSET            (0x4000)
#define CLINT_MTIME_OFFSET               (0xBFF8)
#define CLINT_REL_MTIME_OFFSET           (CLINT_MTIME_OFFSET \
                                         - CLINT_MTIMECMP_OFFSET)

void clint_init(RISCVCPU *cpu, uint32_t num_harts);

#endif

