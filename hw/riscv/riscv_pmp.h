/*
 * QEMU RISC-V PMP Driver
 * Author: Daire McNamara, daire.mcnamara@emdalo.com
 *         Ivan Griffin, ivan.griffin@emdalo.com
 *
 * This provides a RISC-V PMP driver
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
#ifndef _RISCV_PMP_H_
#define _RISCV_PMP_H_

#include "target-riscv/cpu.h"

typedef enum pmp_priv_s {
  PMP_READ  = 1 << 0,
  PMP_WRITE = 1 << 1,
  PMP_EXEC  = 1 << 2,
  PMP_LOCK  = 1 << 7
} pmp_priv_t;

void pmp_init(void);

void pmpcfg_handle_write(CPURISCVState *env, uint32_t reg_index,
    target_ulong val);
target_ulong pmpcfg_handle_read(CPURISCVState *env, uint32_t reg_index);
void pmpaddr_handle_write(CPURISCVState *env, uint32_t addr_index,
    target_ulong val);
target_ulong pmpaddr_handle_read(CPURISCVState *env, uint32_t addr_index);

bool pmp_hart_has_privs(CPURISCVState *env, target_ulong addr,
    target_ulong size, pmp_priv_t priv);

#endif
