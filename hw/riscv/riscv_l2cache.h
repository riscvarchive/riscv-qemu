/*
 * QEMU RISC-V L2 Cache Driver
 * Author: Daire McNamara, daire.mcnamara@emdalo.com
 *
 * This provides a RISC-V L2 Cache driver
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
#ifndef _RISCV_L2_CACHE_H_
#define _RISCV_L2_CACHE_H_

#define CACHE_CTRL_WAY_CACHE_SIZE                       (64)

#define CACHE_CTRL_CONFIG_OFFSET                         (0)
#define CACHE_CFG_NUM_BANKS_OFFSET                       (0)
#define CACHE_CFG_NUM_WAYS_OFFSET                        (1)
#define CACHE_CFG_NUM_SETS_PER_BANK_OFFSET               (2)
#define CACHE_CFG_NUM_BYTES_PER_BLOCK_OFFSET             (3)
#define CACHE_CTRL_WAY_ENABLE_OFFSET                     (8)
#define CACHE_CTRL_ECC_INJECT_ERROR_OFFSET            (0x40)
#define CACHE_CTRL_ECC_DIR_FIX_ADDR_OFFSET           (0x100)
#define CACHE_CTRL_ECC_DIR_FIX_COUNT_OFFSET          (0x108)
#define CACHE_CTRL_ECC_DATA_FIX_ADDR_OFFSET          (0x140)
#define CACHE_CTRL_ECC_DATA_FIX_COUNT_OFFSET         (0x148)
#define CACHE_CTRL_ECC_DATA_FAIL_ADDR_OFFSET         (0x160)
#define CACHE_CTRL_ECC_DATA_FAIL_COUNT_OFFSET        (0x168)
#define CACHE_CTRL_FLUSH64_OFFSET                    (0x200)
#define CACHE_CTRL_FLUSH32_OFFSET                    (0x240)
#define CACHE_CTRL_WAY_MASK0_OFFSET                  (0x800)
#define CACHE_CTRL_WAY_MASK1_OFFSET                  (0x808)
#define CACHE_CTRL_WAY_MASK2_OFFSET                  (0x810)
#define CACHE_CTRL_WAY_MASK3_OFFSET                  (0x818)
#define CACHE_CTRL_WAY_MASK4_OFFSET                  (0x820)

#include "target-riscv/cpu.h"
void l2cache_init(RISCVCPU *cpu, uint32_t num_banks, uint32_t num_ways,
    uint32_t num_sets, uint32_t num_bytes_per_block);

void l2zero_dev_init(uint64_t base, uint32_t sz);

#endif
