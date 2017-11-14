/*
 * QEMU RISC-V PMP Device
 * Author: Daire McNamara, daire.mcnamara@emdalo.com
 *         Ivan Griffin, ivan.griffin@emdalo.com
 *
 * This provides a RISC-V PMP device model
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

#include "qemu/osdep.h"
#include "hw/riscv/riscv_pmp.h"
#include "hw/riscv/soc.h"
#include "exec/address-spaces.h"

/* #define DEBUG_PMP 1 */

#ifdef DEBUG_PMP
#define PMP_PRINTF(fmt, ...) \
do { fprintf(stderr, "pmp: " fmt, ## __VA_ARGS__); } while (0)
#else
#define PMP_PRINTF(fmt, ...) \
do {} while (0)
#endif

typedef enum pmp_am_s {
    PMP_AMATCH_OFF,  /* Null (off)                            */
    PMP_AMATCH_TOR,  /* Top of Range                          */
    PMP_AMATCH_NA4,  /* Naturally aligned four-byte region    */
    PMP_AMATCH_NAPOT /* Naturally aligned power-of-two region */
} pmp_am_t;

typedef struct pmp_entry_s {
    target_ulong addr_reg;
    uint8_t  cfg_reg;
} pmp_entry_t;

typedef struct pmp_addr_s {
    target_ulong sa;
    target_ulong ea;
} pmp_addr_t;

typedef struct pmp_table_s {
    pmp_entry_t pmp[NUM_HARTS][NUM_PMPS];
    pmp_addr_t  addr[NUM_HARTS][NUM_PMPS];
    uint32_t num_rules[NUM_HARTS];
} pmp_table_t;


static pmp_table_t pmp_table = { { { { 0 } } }, { { { 0 } } }, { 0 } };

static void pmp_write_cfg(uint32_t hart_index, uint32_t addr_index,
    uint8_t val);
static uint8_t pmp_read_cfg(uint32_t hart_index, uint32_t addr_index);
static void pmp_update_rule(uint32_t hart_index, uint32_t pmp_index);

/*
 * Accessor method to extract address matching type 'a field' from cfg reg
 */
static inline uint8_t pmp_get_a_field(uint8_t cfg)
{
    uint8_t a = cfg >> 3;
    return a & 0x3;
}

/*
 * Check whether a PMP is locked or not.
 */
static inline int pmp_is_locked(uint32_t hart_index, uint32_t pmp_index)
{

    if (pmp_table.pmp[hart_index][pmp_index].cfg_reg & PMP_LOCK) {
        return 1;
    }

    /* Top PMP has no 'next' to check */
    if ((pmp_index + 1u) >= NUM_PMPS) {
        return 0;
    }

    /* In TOR mode, need to check the lock bit of the next pmp
     * (if there is a next)
     */
    const uint8_t a_field =
        pmp_get_a_field(pmp_table.pmp[hart_index][pmp_index + 1].cfg_reg);
    if ((pmp_table.pmp[hart_index][pmp_index + 1u].cfg_reg & PMP_LOCK) &&
         (PMP_AMATCH_TOR == a_field)) {
        return 1;
    }

    return 0;
}

/*
 * Check if read bit is set or not
 */
static inline int pmp_can_read(uint32_t hart_index, uint32_t pmp_index)
{
    int result = 0;

    if (pmp_table.pmp[hart_index][pmp_index].cfg_reg & PMP_READ) {
        result = 1;
    }

    return result;
}

/*
 * Check if write bit is set or not
 */
static inline int pmp_can_write(uint32_t hart_index, uint32_t pmp_index)
{
    int result = 0;

    if (pmp_table.pmp[hart_index][pmp_index].cfg_reg & PMP_WRITE) {
        result = 1;
    }

    return result;
}

/*
 * Check if execute bit is set or not
 */
static inline int pmp_can_exec(uint32_t hart_index, uint32_t pmp_index)
{
    int result = 0;

    if (pmp_table.pmp[hart_index][pmp_index].cfg_reg & PMP_EXEC) {
        result = 1;
    }

    return result;
}

/*
 * Count the number of active rules.
 */
static inline uint32_t pmp_get_num_rules(uint32_t hart_index)
{
     return pmp_table.num_rules[hart_index];
}

/*
 * Accessor to get the cfg reg for a specific PMP/HART
 */
static inline uint8_t pmp_read_cfg(uint32_t hart_index, uint32_t pmp_index)
{
    if ((hart_index < NUM_HARTS) && (pmp_index < NUM_PMPS)) {
        return pmp_table.pmp[hart_index][pmp_index].cfg_reg;
    }

    return 0;
}


/*
 * Accessor to set the cfg reg for a specific PMP/HART
 * Bounds checks and relevant lock bit.
 */
static void pmp_write_cfg(uint32_t hart_index, uint32_t pmp_index, uint8_t val)
{
    if ((hart_index < NUM_HARTS) && (pmp_index < NUM_PMPS)) {
        if (!pmp_is_locked(hart_index, pmp_index)) {
            pmp_table.pmp[hart_index][pmp_index].cfg_reg = val;
            pmp_update_rule(hart_index, pmp_index);
        } else {
            PMP_PRINTF("Ignoring pmpcfg write - locked\n");
        }
    } else {
        PMP_PRINTF("Ignoring pmpcfg write - out of bounds\n");
    }
}

static target_ulong pmp_get_napot_base_and_range(target_ulong reg,
    target_ulong *range)
{
    /* construct a mask of all bits bar the top bit */
    target_ulong mask = 0u;
    target_ulong base = reg;
    target_ulong numbits = (sizeof(target_ulong) * 8u) + 2u;
    mask = (mask - 1u) >> 1;

    while (mask) {
        if ((reg & mask) == mask) {
            /* this is the mask to use */
            base = reg & ~mask;
            break;
        }
        mask >>= 1;
        numbits--;
    }

    *range = (1lu << numbits) - 1u;
    return base;
}


/* Convert cfg/addr reg values here into simple 'sa' --> start address and 'ea'
 *   end address values.
 *   This function is called relatively infrequently whereas the check that
 *   an address is within a pmp rule is called often, so optimise that one
 */
static void pmp_update_rule(uint32_t hart_index, uint32_t pmp_index)
{
    int i;

    pmp_table.num_rules[hart_index] = 0;

    uint8_t this_cfg = pmp_table.pmp[hart_index][pmp_index].cfg_reg;
    target_ulong this_addr = pmp_table.pmp[hart_index][pmp_index].addr_reg;
    target_ulong prev_addr = 0u;
    target_ulong sa = 0u;
    target_ulong ea = 0u;

    if (pmp_index >= 1u) {
        prev_addr = pmp_table.pmp[hart_index][pmp_index].addr_reg;
    }

    switch (pmp_get_a_field(this_cfg)) {
    case PMP_AMATCH_OFF:
        sa = 0u;
        ea = -1;
        break;

    case PMP_AMATCH_TOR:
        sa = prev_addr << 2; /* shift up from [xx:0] to [xx+2:2] */
        ea = (this_addr << 2) - 1u;
        break;

    case PMP_AMATCH_NA4:
        sa = this_addr << 2; /* shift up from [xx:0] to [xx+2:2] */
        ea = (this_addr + 4u) - 1u;
        break;

    case PMP_AMATCH_NAPOT:
        sa = pmp_get_napot_base_and_range(this_addr, &ea);
        sa = this_addr << 2; /* shift up from [xx:0] to [xx+2:2] */
        ea += sa;
        break;

    default:
        sa = 0u;
        ea = 0u;
        break;
    }

    pmp_table.addr[hart_index][pmp_index].sa = sa;
    pmp_table.addr[hart_index][pmp_index].ea = ea;

    for (i = 0; i < NUM_PMPS; i++) {
        const uint8_t a_field =
            pmp_get_a_field(pmp_table.pmp[hart_index][i].cfg_reg);
        if (PMP_AMATCH_OFF != a_field) {
            pmp_table.num_rules[hart_index]++;
        }
    }
}

static int pmp_is_in_range(int hart_index, int pmp_index, target_ulong addr)
{
    int result = 0;

    if ((addr >= pmp_table.addr[hart_index][pmp_index].sa)
        && (addr < pmp_table.addr[hart_index][pmp_index].ea)) {
        result = 1;
    } else {
        result = 0;
    }

    return result;
}


/*
 * Public Interface
 */

/*
 * Check if the address has required RWX privs to complete desired operation
 */
bool pmp_hart_has_privs(CPURISCVState *env, target_ulong addr,
    target_ulong size, pmp_priv_t privs)
{
    int i = 0;
    int ret = -1;
    target_ulong s = 0;
    target_ulong e = 0;
    pmp_priv_t allowed_privs = 0;
    uint32_t hart_index = env->hart_index;

    /* Short cut if no rules */
    if (0 == pmp_get_num_rules(env->hart_index)) {
        return true;
    }

    /* 1.10 draft priv spec states there is an implicit order
         from low to high */
    for (i = 0; i < NUM_PMPS; i++) {
        s = pmp_is_in_range(hart_index, i, addr);
        e = pmp_is_in_range(hart_index, i, addr + size);

        /* partially inside */
        if ((s + e) == 1) {
            PMP_PRINTF("pmp violation - access is partially in /"
                " partially out\n");
            ret = 0;
            break;
        }

        /* fully inside */
        const uint8_t a_field =
            pmp_get_a_field(pmp_table.pmp[hart_index][i].cfg_reg);
        if ((s + e) == 2) {
            if (PMP_AMATCH_OFF == a_field) {
                return 1;
            }

            allowed_privs = PMP_READ | PMP_WRITE | PMP_EXEC;
            if ((env->priv != PRV_M) || pmp_is_locked(hart_index, i)) {
                allowed_privs &= pmp_table.pmp[hart_index][i].cfg_reg;
            }

            if ((privs & allowed_privs) == privs) {
                ret = 1;
                break;
            } else {
                ret = 0;
                break;
            }
        }
    }

    /* No rule matched */
    if (ret == -1) {
        if (env->priv == PRV_M) {
            ret = 1; /* Draft 1.10 spec states if no PMP entry matches an
                      * M-Mode access, the access succeeds */
        } else {
            ret = 0; /* Other modes are not allowed succeed if they don't
                      * match a rule, but there are rules.  We've checked for
                      * no rule earlier in this function. */
        }
    }

    return ret == 1 ? true : false;
}


/*
 * Handle a write to a pmpcfg CSP
 */
void pmpcfg_handle_write(CPURISCVState *env, uint32_t reg_index,
    target_ulong val)
{
    int i;
    uint8_t cfg_val;

    PMP_PRINTF("hart%d pmpcfg_reg%d val: 0x" TARGET_FMT_lx "\n",
        env->hart_index, reg_index, val);

    if ((reg_index & 1) && (sizeof(target_ulong) == 8)) {
        PMP_PRINTF("Ignoring pmpcfg write - incorrect address\n");
        return;
    }

    for (i = 0; i < sizeof(target_ulong); i++) {
        cfg_val = (val >> 8 * i)  & 0xff;
        pmp_write_cfg(env->hart_index, (reg_index * sizeof(target_ulong)) + i,
            cfg_val);
    }
}


/*
 * Handle a read from a pmpcfg CSP
 */
target_ulong pmpcfg_handle_read(CPURISCVState *env, uint32_t reg_index)
{
    int i;
    target_ulong cfg_val = 0;
    uint8_t val = 0;

    for (i = 0; i < sizeof(target_ulong); i++) {
        val = pmp_read_cfg(env->hart_index,
            (reg_index * sizeof(target_ulong)) + i);
        cfg_val |= (val << (i * 8));
    }

    PMP_PRINTF("hart%d pmpcfg_reg%d, (rval: 0x" TARGET_FMT_lx ")\n",
        env->hart_index, reg_index, cfg_val);

    return cfg_val;
}


/*
 * Handle a write to a pmpaddr CSP
 */
void pmpaddr_handle_write(CPURISCVState *env, uint32_t addr_index,
    target_ulong val)
{
    PMP_PRINTF("hart%d addr%d val: 0x" TARGET_FMT_lx "\n",
        env->hart_index, addr_index, val);

    /* val &= 0x3ffffffffffffful; */

    if ((env->hart_index < NUM_HARTS) && (addr_index < NUM_PMPS)) {
        if (!pmp_is_locked(env->hart_index, addr_index)) {
            pmp_table.pmp[env->hart_index][addr_index].addr_reg = val;
            pmp_update_rule(env->hart_index, addr_index);
        } else {
            PMP_PRINTF("Ignoring pmpaddr write - locked\n");
        }
    } else {
        PMP_PRINTF("Ignoring pmpaddr write - out of bounds\n");
    }
}


/*
 * Handle a read from a pmpaddr CSP
 */
target_ulong pmpaddr_handle_read(CPURISCVState *env, uint32_t addr_index)
{
    PMP_PRINTF("hart%d addr%d (val: 0x" TARGET_FMT_lx ")\n",
        env->hart_index, addr_index,
        pmp_table.pmp[env->hart_index][addr_index].addr_reg);
    return pmp_table.pmp[env->hart_index][addr_index].addr_reg;
}


/*
 * Init the PMP.  Can be used to reset also.
 */
void pmp_init(void)
{
     memset(&pmp_table, 0, sizeof(pmp_table_t));
}

