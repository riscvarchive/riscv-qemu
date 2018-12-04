/*
 * QEMU RISC-V PMP (Physical Memory Protection)
 *
 * Author: Daire McNamara, daire.mcnamara@emdalo.com
 *         Ivan Griffin, ivan.griffin@emdalo.com
 *
 * This provides a RISC-V Physical Memory Protection interface
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

#ifndef _RISCV_PMP_H_
#define _RISCV_PMP_H_

typedef enum {
    PMP_READ  = 1 << 0,
    PMP_WRITE = 1 << 1,
    PMP_EXEC  = 1 << 2,
    PMP_LOCK  = 1 << 7
} pmp_priv_t;

typedef enum {
    PMP_AMATCH_OFF   = 0, /* Null (off)                            */
    PMP_AMATCH_TOR   = 1, /* Top of Range                          */
    PMP_AMATCH_NA4   = 2, /* Naturally aligned four-byte region    */
    PMP_AMATCH_NAPOT = 3  /* Naturally aligned power-of-two region */
} pmp_am_t;

typedef struct {
    target_ulong addr_reg;
    uint8_t  cfg_reg;
} pmp_entry_t;

typedef struct {
    pmp_entry_t pmp[MAX_RISCV_PMPS];
    size_t active_rules;
    size_t granularity;
} pmp_table_t;

void pmpcfg_csr_write(CPURISCVState *env, size_t cfg, target_ulong val);
target_ulong pmpcfg_csr_read(CPURISCVState *env, size_t cfg);
void pmpaddr_csr_write(CPURISCVState *env, size_t addr, target_ulong val);
target_ulong pmpaddr_csr_read(CPURISCVState *env, size_t addr);
bool pmp_has_access(CPURISCVState *env, target_ulong addr, int size, int rw,
                    target_ulong *tlb_entry_size);

#endif
