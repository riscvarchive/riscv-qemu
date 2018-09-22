/*
 * QEMU RISC-V PMP (Physical Memory Protection)
 *
 * Authors: Daire McNamara <daire.mcnamara@emdalo.com>
 *          Ivan Griffin <ivan.griffin@emdalo.com>
 *          Michael Clark <mjc@sifive.com>
 *
 * This provides a RISC-V Physical Memory Protection implementation
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

/*
 * PMP (Physical Memory Protection) is as-of-yet unused and needs testing.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "cpu.h"
#include "qemu-common.h"
#include "trace.h"

#ifndef CONFIG_USER_ONLY

#define PMP_DEBUG 0

/*
 * Accessor method to extract address matching type 'a field' from cfg reg
 */
static inline uint8_t pmp_get_a_field(uint8_t cfg)
{
    return (cfg >> 3) & 0x3;
}

/*
 * Check whether a PMP is locked or not.
 */
static inline int pmp_is_locked(CPURISCVState *env, uint32_t pmp_index)
{

    if (env->pmp_state.pmp[pmp_index].cfg_reg & PMP_LOCK) {
        return 1;
    }

    /* Top PMP has no 'next' to check */
    if ((pmp_index + 1u) >= MAX_RISCV_PMPS) {
        return 0;
    }

    /* In TOR mode, need to check the lock bit of the next entry */
    const uint8_t a_field =
        pmp_get_a_field(env->pmp_state.pmp[pmp_index + 1].cfg_reg);

    if ((env->pmp_state.pmp[pmp_index + 1].cfg_reg & PMP_LOCK) &&
         (PMP_AMATCH_TOR == a_field)) {
        return 1;
    }

    return 0;
}

/*
 * Convert cfg/addr reg values here into start address and end address values.
 * This function is on the slow path so we optimize for the fast path.
 */
static void pmp_update_rule(CPURISCVState *env, uint32_t pmp_index)
{
    uint8_t this_cfg = env->pmp_state.pmp[pmp_index].cfg_reg;
    target_ulong addr = env->pmp_state.pmp[pmp_index].addr_reg;
    target_ulong sa, ea, bits, base, mask;
    target_ulong prev_addr = 0u;

    env->pmp_state.num_rules = 0;

    if (pmp_index >= 1u) {
        prev_addr = env->pmp_state.pmp[pmp_index - 1].addr_reg;
    }

    switch (pmp_get_a_field(this_cfg)) {
    case PMP_AMATCH_OFF:
        sa = 0u;
        ea = -1;
        break;

    case PMP_AMATCH_TOR:
        sa = prev_addr << 2;
        ea = (addr << 2) - 1;
        break;

    case PMP_AMATCH_NA4:
        sa = addr << 2;
        ea = (addr + 4) - 1;
        break;

    case PMP_AMATCH_NAPOT:
        /*
         * FIXME: We have an issue with entries whose length is less
         *        than or equal to the 2 ^ TARGET_PAGE_BITS granularity.
         *        This is because we mask off the lower bits when the
         *        CSR is written, losing information, due to masking of
         *        lower bits, thus these entries end up with a different
         *        first 1 bit that defines the length of the range,
         *        erroneously expanding the range. This is an issue for
         *        any software that configures PMPs where the granularity
         *        is not 4 bytes. Also there is a problem for granule size
         *        itself, related to the granule size detection algorithm
         *        meaning the granule size cannot be encoded without an
         *        equivalent to NA4 i.e. NAG (naturally aligned granule).
         */
        bits = ctz64((addr >> (TARGET_PAGE_BITS - 2))) + TARGET_PAGE_BITS;
        mask = ((target_ulong)1 << bits) - 1;
        base = (addr << 2) & ~mask;
        /*
         * special case for all ones address, allow access to all memory
         */
        if (mask == ((1 << TARGET_PAGE_BITS)-1) &&
            (base | mask) == (target_ulong)-1) {
            sa = 0;
            ea = -1;
        } else {
            sa = base;
            ea = base + mask;
        }
#if PMP_DEBUG
        fprintf(stderr, "[%d]"
            " addr="TARGET_FMT_lx
            " base="TARGET_FMT_lx
            " mask="TARGET_FMT_lx
            " sa="TARGET_FMT_lx
            " ea="TARGET_FMT_lx"\n",
            pmp_index, addr, base, mask, sa, ea);
#endif
        break;
    default:
        sa = 0u;
        ea = 0u;
        break;
    }

    env->pmp_state.addr[pmp_index].sa = sa;
    env->pmp_state.addr[pmp_index].ea = ea;

    for (size_t i = 0; i < MAX_RISCV_PMPS; i++) {
        const uint8_t a_field =
            pmp_get_a_field(env->pmp_state.pmp[i].cfg_reg);
        if (PMP_AMATCH_OFF != a_field) {
            env->pmp_state.num_rules++;
        }
    }
}

/*
 * Public Interface
 */

/*
 * Check if the address has required RWX privs to complete desired operation
 */
bool pmp_has_access(CPURISCVState *env, target_ulong addr, int access_type)
{
    bool result = false;
    int access_priv;

    switch (access_type) {
    case MMU_INST_FETCH:
        access_priv = PMP_EXEC;
        break;
    case MMU_DATA_LOAD:
        access_priv = PMP_READ;
        break;
    case MMU_DATA_STORE:
        access_priv = PMP_WRITE;
        break;
    default:
        g_assert_not_reached();
    }

    /* if there are no rules, then only allow access to M-Mode */
    if (env->pmp_state.num_rules == 0) {
        return env->priv == PRV_M ? true : false;
    }

    /* loop through each rule and check access */
    for (size_t i = 0; i < MAX_RISCV_PMPS; i++) {
        if (pmp_get_a_field(env->pmp_state.pmp[i].cfg_reg) == PMP_AMATCH_OFF) {
            continue;
        }
        if ((addr >= env->pmp_state.addr[i].sa) &&
            (addr <= env->pmp_state.addr[i].ea)) {
            if (env->priv == PRV_M && !pmp_is_locked(env, i)) {
                result = true;
            } else {
                result = env->pmp_state.pmp[i].cfg_reg & access_priv;
                if (!result) {
                    continue; /* let another rule match */
                }
            }
            goto out;
        }
    }

    /* if no PMP entries matches then only allow M-Mode access */
    result = env->priv == PRV_M;

out:
    trace_pmp_has_access(env->mhartid, addr, access_priv, result);

    return result;
}


/*
 * Handle a write to a pmpcfg CSP
 */
void pmpcfg_csr_write(CPURISCVState *env, uint32_t reg_index,
    target_ulong val)
{
    if ((reg_index & 1) && (sizeof(target_ulong) == 8)) {
        return;
    }

    if (sizeof(target_ulong) == 8) {
        reg_index >>= 1;
    }

    for (size_t i = 0; i < sizeof(target_ulong); i++) {
        int pmp_index = reg_index * sizeof(target_ulong) + i;
        uint8_t cfg_val = (val >> 8 * i)  & 0xff;
        if (!pmp_is_locked(env, pmp_index)) {
            env->pmp_state.pmp[pmp_index].cfg_reg = cfg_val;
            pmp_update_rule(env, pmp_index);
        }
    }

    trace_pmpcfg_csr_write(env->mhartid, reg_index, val);
}


/*
 * Handle a read from a pmpcfg CSP
 */
target_ulong pmpcfg_csr_read(CPURISCVState *env, uint32_t reg_index)
{
    target_ulong cfg_val = 0;

    if ((reg_index & 1) && (sizeof(target_ulong) == 8)) {
        return 0;
    }

    if (sizeof(target_ulong) == 8) {
        reg_index >>= 1;
    }

    for (size_t  i = 0; i < sizeof(target_ulong); i++) {
        int pmp_index = reg_index * sizeof(target_ulong) + i;
        target_ulong val = env->pmp_state.pmp[pmp_index].cfg_reg;
        cfg_val |= (val << (i * 8));
    }

    trace_pmpcfg_csr_read(env->mhartid, reg_index, cfg_val);

    return cfg_val;
}


/*
 * Handle a write to a pmpaddr CSP
 */
void pmpaddr_csr_write(CPURISCVState *env, uint32_t addr_index,
    target_ulong val)
{
    if (addr_index >= MAX_RISCV_PMPS) return;

    /* mask for page granularity */
    val &= (((target_ulong)TARGET_PAGE_MASK) >> 2);

    trace_pmpaddr_csr_write(env->mhartid, addr_index, val);

    if (!pmp_is_locked(env, addr_index)) {
        env->pmp_state.pmp[addr_index].addr_reg = val;
        pmp_update_rule(env, addr_index);
    }
}


/*
 * Handle a read from a pmpaddr CSP
 */
target_ulong pmpaddr_csr_read(CPURISCVState *env, uint32_t addr_index)
{
    target_ulong val;

    if (addr_index >= MAX_RISCV_PMPS) return 0;

    val = env->pmp_state.pmp[addr_index].addr_reg;

    trace_pmpaddr_csr_read(env->mhartid, addr_index, val);

    return val;
}

#endif
