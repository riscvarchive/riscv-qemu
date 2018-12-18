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

/*
 * Round up to the next highest power of two
 *
 * Source: https://graphics.stanford.edu/~seander/bithacks.html
 */
static inline uint64_t roundpow2(uint64_t val)
{
    val--;
    val |= val >> 1;
    val |= val >> 2;
    val |= val >> 4;
    val |= val >> 8;
    val |= val >> 16;
    val |= val >> 32;
    val++;
    return val;
}

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
static inline int pmp_is_locked(CPURISCVState *env, size_t i)
{
    uint8_t cfg_reg = env->pmp_state.pmp[i].cfg_reg;

    if (cfg_reg & PMP_LOCK) {
        return 1;
    }

    /* In TOR mode, need to check the lock bit of the next entry */
    if (i + 1 < MAX_RISCV_PMPS) {
        cfg_reg = env->pmp_state.pmp[i + 1].cfg_reg;
        if (pmp_get_a_field(cfg_reg) == PMP_AMATCH_TOR) {
            if (cfg_reg & PMP_LOCK) {
                return 1;
            }
        }
    }

    return 0;
}

/*
 * Convert QEMU access type to PMP privileges
 */
static inline int pmp_access_priv(int access_type)
{
    switch (access_type) {
    case MMU_INST_FETCH:
        return PMP_EXEC;
    case MMU_DATA_LOAD:
        return PMP_READ;
    case MMU_DATA_STORE:
        return PMP_WRITE;
    default:
        g_assert_not_reached();
    }
    return 0;
}

static void pmp_decode_napot(target_ulong a, target_ulong *sa, target_ulong *ea)
{
    /*
       aaaa...aaa0   8-byte NAPOT range
       aaaa...aa01   16-byte NAPOT range
       aaaa...a011   32-byte NAPOT range
       ...
       aa01...1111   2^XLEN-byte NAPOT range
       a011...1111   2^(XLEN+1)-byte NAPOT range
       0111...1111   2^(XLEN+2)-byte NAPOT range
       1111...1111   Reserved
    */
    if (a == -1) {
        *sa = 0u;
        *ea = -1;
        return;
    } else {
        target_ulong t1 = ctz64(~a);
        target_ulong base = (a & ~(((target_ulong)1 << t1) - 1)) << 2;
        target_ulong range = ((target_ulong)1 << (t1 + 3)) - 1;
        *sa = base;
        *ea = base + range;
    }
}

/*
 * Count active rules when config changes
 */
static inline void pmp_count_active_rules(CPURISCVState *env)
{
    /* find last rule that is not PMP_AMATCH_OFF */
    env->pmp_state.active_rules = 0;
    for (size_t i = 0; i < MAX_RISCV_PMPS; i++) {
        if (pmp_get_a_field(env->pmp_state.pmp[i].cfg_reg) != PMP_AMATCH_OFF) {
            env->pmp_state.active_rules = i + 1;
        }
    }
}

/*
 * Public Interface
 */

/*
 * Check if the address has required RWX privs to complete desired operation
 */
bool pmp_has_access(CPURISCVState *env, target_ulong addr, int size, int rw,
                    target_ulong *tlb_size)
{
    int access = pmp_access_priv(rw);
    bool result;

    /* check physical memory protection rules */
    for (size_t i = 0; i < env->pmp_state.active_rules; i++)
    {
        uint8_t cfg_reg = env->pmp_state.pmp[i].cfg_reg;
        target_ulong addr_reg = env->pmp_state.pmp[i].addr_reg;
        target_ulong sa, ea;

        switch (pmp_get_a_field(cfg_reg)) {
        case PMP_AMATCH_TOR:
            sa = (i >= 1 ? env->pmp_state.pmp[i - 1].addr_reg : 0) << 2;
            ea = (addr_reg << 2) - 1;
            break;

        case PMP_AMATCH_NA4:
            /* The current spec wording for PMP granularity requires a
             * terminal bit for granule size spans; generalize NA4 -> NAG */
            sa = addr_reg << 2;
            ea = (addr_reg << 2) + (1 << (env->pmp_state.granularity + 2)) - 1;
            break;

        case PMP_AMATCH_NAPOT:
            pmp_decode_napot(addr_reg, &sa, &ea);
            break;

        case PMP_AMATCH_OFF:
        default:
            continue;
        }

        /* check address and privs, bypass for unlocked rule if PRV_M */
        result = (env->priv == PRV_M && !pmp_is_locked(env, i)) ||
                 (addr >= sa && addr + size - 1 <= ea && (cfg_reg & access));
        trace_pmp_rule_match(env->mhartid, addr, size, access, sa, ea,
                             cfg_reg, result);
        if (result) {
            /* return smaller page size if match has finer granularity */
            if ((sa & (*tlb_size - 1) || (ea + 1) & (*tlb_size - 1))) {
                /* breaks iff other covering entries (invalid config). Punt */
                *tlb_size = roundpow2(ea - sa);
            }
            goto match;
        }
    }

    /* only allow M mode if no rules are present */
    result = env->priv == PRV_M;

match:
    trace_pmp_has_access(env->mhartid, addr, size, access, result);

    return result;
}


/*
 * Handle a write to a pmpcfg CSR
 */
void pmpcfg_csr_write(CPURISCVState *env, size_t cfg, target_ulong val)
{
    if (TARGET_LONG_BITS == 64) {
        if (cfg & 1) {
            return;
        }
        cfg >>= 1;
    }

    for (size_t b = 0; b < sizeof(target_ulong); b++) {
        size_t i = cfg * sizeof(target_ulong) + b;
        if (!pmp_is_locked(env, i)) {
            env->pmp_state.pmp[i].cfg_reg = (val >> 8 * b)  & 0xff;
            pmp_count_active_rules(env);
        }
    }

    trace_pmpcfg_csr_write(env->mhartid, cfg, val);
}


/*
 * Handle a read from a pmpcfg CSR
 */
target_ulong pmpcfg_csr_read(CPURISCVState *env, size_t cfg)
{
    target_ulong cfg_val = 0;

    if (TARGET_LONG_BITS == 64) {
        if (cfg & 1) {
            return 0;
        }
        cfg >>= 1;
    }

    for (size_t b = 0; b < sizeof(target_ulong); b++) {
        size_t i = cfg * sizeof(target_ulong) + b;
        target_ulong val = env->pmp_state.pmp[i].cfg_reg;
        cfg_val |= (val << (b * 8));
    }

    trace_pmpcfg_csr_read(env->mhartid, cfg, cfg_val);

    return cfg_val;
}


/*
 * Handle a write to a pmpaddr CSR
 */
void pmpaddr_csr_write(CPURISCVState *env, size_t addr, target_ulong val)
{
    if (addr >= MAX_RISCV_PMPS) {
        return;
    }

    if (env->pmp_state.granularity) {
        val &= (((target_ulong)1 << env->pmp_state.granularity) - 1);
    }

    trace_pmpaddr_csr_write(env->mhartid, addr, val);

    if (!pmp_is_locked(env, addr)) {
        env->pmp_state.pmp[addr].addr_reg = val;
    }
}


/*
 * Handle a read from a pmpaddr CSR
 */
target_ulong pmpaddr_csr_read(CPURISCVState *env, size_t addr)
{
    target_ulong val;

    if (addr >= MAX_RISCV_PMPS) {
        return 0;
    }

    val = env->pmp_state.pmp[addr].addr_reg;

    trace_pmpaddr_csr_read(env->mhartid, addr, val);

    return val;
}

#endif
