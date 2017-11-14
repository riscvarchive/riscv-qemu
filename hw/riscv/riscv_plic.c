/*
 * QEMU RISC-V PLIC
 * Author: Daire McNamara, daire.mcnamara@emdalo.com
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

#include "qemu/osdep.h"
#include "qemu/queue.h"
#include "hw/riscv/riscv_plic.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/riscv/soc.h"
#include "exec/address-spaces.h"

uint32_t plic_regs[PLIC_REG_SZ] = { 0xffffffff };

#define SPAN_OF(x) (sizeof(x) / sizeof(x[0]))

/* #define DEBUG_PLIC */

#ifdef DEBUG_PLIC
#define PLIC_DEBUG(fmt, ...) \
do { fprintf(stderr, "plic: " fmt, ## __VA_ARGS__); } while (0)
#else
#define PLIC_DEBUG(fmt, ...) \
do {} while (0)
#endif

#define PLIC_WARN(fmt, ...) \
do { fprintf(stderr, "plic: WARNING " fmt, ## __VA_ARGS__); } while (0)

#define PLIC_ERR(fmt, ...) \
do { fprintf(stderr, "plic: ERROR " fmt, ## __VA_ARGS__); exit(-1); } while (0)

const char *plic_hart_strs[] = {
    "H0M",
    "H1M",
    "H1S",
    "H2M",
    "H2S",
    "H3M",
    "H3S",
    "H4M",
    "H4S",
};

typedef enum irq_state_s {
    PLIC_IRQ_CLAIMED,
    PLIC_IRQ_UNCLAIMED
} irq_state_t;

struct pending_irq_s {
    uint32_t irq;
    irq_state_t state;
    uint64_t index;
    QSIMPLEQ_ENTRY(pending_irq_s) next;
};

QSIMPLEQ_HEAD(requests, pending_irq_s) requests;

static uint32_t plic_get_source_priority(uint32_t irq);

static uint64_t pending_index;

static void plic_add_pending_irq(int irq)
{
    struct pending_irq_s *pending_irq = g_malloc(sizeof(struct pending_irq_s));
    pending_irq->irq = irq;
    pending_irq->state = PLIC_IRQ_UNCLAIMED;
    pending_irq->index = pending_index++;
    PLIC_DEBUG("queuing irq: %d (index: %ld)\n", irq, pending_irq->index);
    QSIMPLEQ_INSERT_TAIL(&requests, pending_irq, next);
}


static void plic_remove_pending_irq(int irq)
{
    struct pending_irq_s *req, *next, *p = 0;
    QSIMPLEQ_FOREACH_SAFE(req, &requests, next, next) {
        if (req->irq == irq) {
            p = req;
            PLIC_DEBUG("dequeueing irq: %d (%ld)\n", irq, p->index);
            QSIMPLEQ_REMOVE(&requests, req, pending_irq_s, next);
            if (p->state == PLIC_IRQ_CLAIMED) {
                break;
            }
        }
    }

    if (!p) {
        PLIC_WARN("failed to find irq\n");
    }
}


static uint32_t plic_get_pending_claim(bool modify)
{
    uint32_t curr_prio = 0;
    uint32_t next_prio = 0;
    struct pending_irq_s *req, *next, *claim = 0;
    uint32_t result = 0;

    QSIMPLEQ_FOREACH_SAFE(req, &requests, next, next) {
        if (req->state == PLIC_IRQ_CLAIMED) {
            continue;
        }

        next_prio = plic_get_source_priority(req->irq);
        if (next_prio > curr_prio) {
            claim = req;
            curr_prio = next_prio;
        } else if ((claim == 0) && (next_prio == curr_prio)) {
            claim = req;
        } else if ((claim->irq > req->irq) && (next_prio == curr_prio)) {
            claim = req;
        }
    }

    /* change state of claim */
    if (claim) {
        /* allow emulator self-debug */
        if (modify) {
            claim->state = PLIC_IRQ_CLAIMED;
        }
        PLIC_DEBUG("claiming %d (%ld)\n", claim->irq, claim->index);
        result = claim->irq;
    } else {
        result = 0;
    }

    return result;
}


static void plic_set_pending_claim(int irq)
{
    uint32_t base = PLIC_PENDING_BASE;
    uint32_t bit = irq % 32;
    base += irq / 32;
    plic_regs[base] |= bit;
    plic_add_pending_irq(irq);
}


static void plic_clear_pending_claim(int irq)
{
    uint32_t base = PLIC_PENDING_BASE;
    uint32_t bit = irq % 32;
    base += irq / 32;
    plic_regs[base] &= ~bit;
    plic_remove_pending_irq(irq);
}


static void plic_handle_write_source_priority(void *opaque, hwaddr addr,
    uint64_t val)
{
#if defined(DEBUG_PLIC)
    RISCVCPU *cpu = opaque;
    CPURISCVState *env = &cpu->env;
#endif
    uint64_t index = addr >> 2;

    val &= 7;

    PLIC_DEBUG("hart%d, setting source priority for interrupt %ld to %ld"
        " (%s)\n",
        env->hart_index, index, val, val ? "" : "disabled");
    plic_regs[index] = (uint32_t)val;
}


static void plic_handle_write_pending(void *opaque, hwaddr addr, uint64_t val)
{
    RISCVCPU *cpu = opaque;
    CPURISCVState *env = &cpu->env;
    hwaddr pending_base = (addr / PLIC_OFFSET_SIZE) * PLIC_OFFSET_SIZE;
    int hart_idx = ((addr - pending_base) / PLIC_HART_OFFSET_SIZE);
    int bit_base = pending_base + (hart_idx * PLIC_HART_OFFSET_SIZE);
    uint64_t index = addr >> 2;

    PLIC_WARN("ignoring unexpected write, hart%d:"
        " %s [%3ld..%3ld]: %x (addr=%lx)\n",
        env->hart_index, plic_hart_strs[hart_idx], (addr - bit_base) * 8,
        ((addr - bit_base) * 8) + 31, plic_regs[index], addr);
}


static void plic_handle_write_enb(void *opaque, hwaddr addr, uint64_t val)
{
#if defined(DEBUG_PLIC)
    RISCVCPU *cpu = opaque;
    CPURISCVState *env = &cpu->env;
    hwaddr enb_base = (addr / PLIC_OFFSET_SIZE) * PLIC_OFFSET_SIZE;
    int hart_idx = ((addr - enb_base) / PLIC_HART_OFFSET_SIZE);
    int bit_base = enb_base + (hart_idx * PLIC_HART_OFFSET_SIZE);
#endif
    uint64_t index = addr >> 2;

    plic_regs[index] = (uint32_t)val;
    PLIC_DEBUG("hart%d, %s [%3ld..%3ld]: %x (addr=%lx)\n", env->hart_index,
       plic_hart_strs[hart_idx], (addr - bit_base) * 8,
       ((addr - bit_base) * 8) + 31, plic_regs[index], addr);
}


static void plic_handle_write_priority_threshold(void *opaque, hwaddr addr,
    uint64_t val)
{
#if defined(DEBUG_PLIC)
    RISCVCPU *cpu = opaque;
    CPURISCVState *env = &cpu->env;
    hwaddr prio_thresh_base = addr - PLIC_PT_CC_OFFSET_SIZE;
    hwaddr hart_idx = prio_thresh_base / PLIC_OFFSET_SIZE;
#endif
    uint64_t index = addr >> 2;
    plic_regs[index] = (uint32_t)val;
    PLIC_DEBUG("hart%d, %s setting priority threshold to %lx\n",
        env->hart_index, plic_hart_strs[hart_idx], val);
}


static void plic_handle_write_claim_complete(void *opaque, hwaddr addr,
    uint64_t val)
{
#if defined(DEBUG_PLIC)
    RISCVCPU *cpu = opaque;
    CPURISCVState *env = &cpu->env;
    hwaddr thresh_base = addr - PLIC_PT_CC_OFFSET_SIZE;
    hwaddr hart_idx = thresh_base / PLIC_OFFSET_SIZE;
#endif
    PLIC_DEBUG("hart%d, %s completing %ld\n", env->hart_index,
        plic_hart_strs[hart_idx], val);
    plic_clear_pending_claim(val);
}

static const hwaddr plicEnbBaseAddrs[] = {
    PLIC_HART0_M_ENB_BASE,
    PLIC_HART1_M_ENB_BASE,
    PLIC_HART1_S_ENB_BASE,
    PLIC_HART2_M_ENB_BASE,
    PLIC_HART2_S_ENB_BASE,
    PLIC_HART3_M_ENB_BASE,
    PLIC_HART3_S_ENB_BASE,
    PLIC_HART4_M_ENB_BASE,
    PLIC_HART4_S_ENB_BASE
};

static const hwaddr plicEnbEndAddrs[] = {
    PLIC_HART0_M_ENB_END,
    PLIC_HART1_M_ENB_END,
    PLIC_HART1_S_ENB_END,
    PLIC_HART2_M_ENB_END,
    PLIC_HART2_S_ENB_END,
    PLIC_HART3_M_ENB_END,
    PLIC_HART3_S_ENB_END,
    PLIC_HART4_M_ENB_END,
    PLIC_HART4_S_ENB_END
};

static bool plic_is_an_enb_addr(hwaddr addr)
{
    bool result = false;
    int i;

    assert(sizeof(plicEnbBaseAddrs) == sizeof(plicEnbEndAddrs));

    for (i = 0; i < SPAN_OF(plicEnbBaseAddrs); i++) {
        if ((addr >= plicEnbBaseAddrs[i]) && (addr <= plicEnbEndAddrs[i])) {
            result = true;
            break;
        }
    }

    return result;
}


static const hwaddr prioThreshAddrs[] = {
    PLIC_HART0_M_PRIO_THRESH_ADDR,
    PLIC_HART1_M_PRIO_THRESH_ADDR,
    PLIC_HART1_S_PRIO_THRESH_ADDR,
    PLIC_HART2_M_PRIO_THRESH_ADDR,
    PLIC_HART2_S_PRIO_THRESH_ADDR,
    PLIC_HART3_M_PRIO_THRESH_ADDR,
    PLIC_HART3_S_PRIO_THRESH_ADDR,
    PLIC_HART4_M_PRIO_THRESH_ADDR,
    PLIC_HART4_S_PRIO_THRESH_ADDR
};

static bool plic_is_a_prio_thresh_addr(hwaddr addr)
{
    bool result = false;
    int i;

    for (i = 0; i < SPAN_OF(prioThreshAddrs); i++) {
        if (addr == prioThreshAddrs[i]) {
            result = true;
            break;
        }
    }

    return result;
}


static const hwaddr plicClaimCompleteAddrs[] = {
    PLIC_HART0_M_CLAIM_COMPLETE_ADDR,
    PLIC_HART1_M_CLAIM_COMPLETE_ADDR,
    PLIC_HART1_S_CLAIM_COMPLETE_ADDR,
    PLIC_HART2_M_CLAIM_COMPLETE_ADDR,
    PLIC_HART2_S_CLAIM_COMPLETE_ADDR,
    PLIC_HART3_M_CLAIM_COMPLETE_ADDR,
    PLIC_HART3_S_CLAIM_COMPLETE_ADDR,
    PLIC_HART4_M_CLAIM_COMPLETE_ADDR,
    PLIC_HART4_S_CLAIM_COMPLETE_ADDR
};

static bool plic_is_a_claim_complete_addr(hwaddr addr)
{
    bool result = false;
    int i;

    for (i = 0; i < SPAN_OF(plicClaimCompleteAddrs); i++) {
        if (addr == plicClaimCompleteAddrs[i]) {
            result = true;
            break;
        }
    }

    return result;
}


static void plic_write(void *opaque, hwaddr addr, uint64_t val64,
    unsigned int size)
{
    if (addr <= PLIC_SOURCE_PRIORITY_END) {
        plic_handle_write_source_priority(opaque, addr, val64);
    } else if ((addr >= PLIC_PENDING_BASE) && (addr <= PLIC_PENDING_END)) {
        plic_handle_write_pending(opaque, addr, val64);
    } else if (plic_is_an_enb_addr(addr)) {
        plic_handle_write_enb(opaque, addr, val64);
    } else if (plic_is_a_prio_thresh_addr(addr)) {
        plic_handle_write_priority_threshold(opaque, addr, val64);
    } else if (plic_is_a_claim_complete_addr(addr)) {
        plic_handle_write_claim_complete(opaque, addr, val64);
    } else {
        PLIC_DEBUG("unexpected write at %lx\n", addr);
    }
}


static uint32_t plic_get_priority_threshold(uint32_t hart_index, uint32_t mode)
{
    uint64_t addr = PLIC_PRIO_THRESH_ADDR;

    if (hart_index != 0) {
        addr += PLIC_OFFSET_SIZE;
        addr += (2 * PLIC_OFFSET_SIZE) * (hart_index - 1);
    }

    if (mode == PRV_M) {
        addr += PLIC_OFFSET_SIZE;
    }

    uint64_t index = addr >> 2;

    return plic_regs[index];
}


static uint32_t plic_get_source_priority(uint32_t irq)
{
    return plic_regs[irq];
}


static uint32_t plic_handle_read_source_priority(void *opaque, hwaddr addr)
{
    uint64_t index = addr >> 2;
#if defined(DEBUG_PLIC)
    RISCVCPU *cpu = opaque;
    CPURISCVState *env = &cpu->env;
#endif
    PLIC_DEBUG("hart%d, getting priority for interrupt %ld (%d)\n",
       env->hart_index, index, plic_regs[index]);
    return plic_regs[index];
}


static uint64_t plic_handle_read_pending(void *opaque, hwaddr addr)
{
    uint64_t index = addr >> 2;
#if defined(DEBUG_PLIC)
    RISCVCPU *cpu = opaque;
    CPURISCVState *env = &cpu->env;
    hwaddr pending_base = (addr / PLIC_OFFSET_SIZE) * PLIC_OFFSET_SIZE;
    int hart_idx = ((addr - pending_base) / PLIC_HART_OFFSET_SIZE);
    int bit_base = pending_base + (hart_idx * PLIC_HART_OFFSET_SIZE);
#endif

    PLIC_DEBUG("hart%d, %s [%3ld..%3ld]: %x (addr=%lx)\n", env->hart_index,
        plic_hart_strs[hart_idx], (addr - bit_base) * 8,
        ((addr - bit_base) * 8) + 31, plic_regs[index], addr);
    return plic_regs[index];
}


static uint64_t plic_handle_read_enb(void *opaque, hwaddr addr)
{
    uint64_t index = addr >> 2;
#if defined(DEBUG_PLIC)
    RISCVCPU *cpu = opaque;
    CPURISCVState *env = &cpu->env;
    hwaddr enb_base = (addr / PLIC_OFFSET_SIZE) * PLIC_OFFSET_SIZE;
    int hart_idx = ((addr - enb_base) / PLIC_HART_OFFSET_SIZE);
    int bit_base = enb_base + (hart_idx * PLIC_HART_OFFSET_SIZE);
#endif

    PLIC_DEBUG("hart%d, %s [%3ld..%3ld]: %x (addr=%lx)\n", env->hart_index,
        plic_hart_strs[hart_idx], (addr - bit_base) * 8,
        ((addr - bit_base) * 8) + 31, plic_regs[index], addr);

    return plic_regs[index];
}


static uint64_t plic_handle_read_priority_threshold(void *opaque, hwaddr addr)
{
    uint64_t index = addr >> 2;
#if defined(DEBUG_PLIC)
    RISCVCPU *cpu = opaque;
    CPURISCVState *env = &cpu->env;
    hwaddr prio_thresh_base = addr - PLIC_PT_CC_OFFSET_SIZE;
    hwaddr hart_idx = prio_thresh_base / PLIC_OFFSET_SIZE;
#endif
    PLIC_DEBUG("hart%d, %s: %s getting priority threshold (%x)\n",
        env->hart_index, __func__, plic_hart_strs[hart_idx], plic_regs[index]);
    return plic_regs[index];
}


static uint64_t plic_handle_read_claim_complete(void *opaque, uint32_t addr)
{
#if defined(DEBUG_PLIC)
    RISCVCPU *cpu = opaque;
    CPURISCVState *env = &cpu->env;
    hwaddr cc_base = addr - PLIC_PT_CC_OFFSET_SIZE;
    hwaddr hart_idx = cc_base / PLIC_OFFSET_SIZE;
#endif
    PLIC_DEBUG("hart%d, %s claiming %d\n", env->hart_index,
       plic_hart_strs[hart_idx], plic_get_pending_claim(false));
    return plic_get_pending_claim(true);
}


static uint64_t plic_read(void *opaque, hwaddr addr, unsigned int size)
{
  if (addr <= PLIC_SOURCE_PRIORITY_END) {
    return plic_handle_read_source_priority(opaque, addr);
  } else if ((addr >= PLIC_PENDING_BASE) && (addr <= PLIC_PENDING_END)) {
    return plic_handle_read_pending(opaque, addr);
  } else if (plic_is_an_enb_addr(addr)) {
    return plic_handle_read_enb(opaque, addr);
  } else if (plic_is_a_prio_thresh_addr(addr)) {
    return plic_handle_read_priority_threshold(opaque, addr);
  } else if (plic_is_a_claim_complete_addr(addr)) {
    return plic_handle_read_claim_complete(opaque, addr);
  } else {
    PLIC_WARN("unexpected read from %lx\n", addr);
  }

  return 0;
}


static const MemoryRegionOps plic_ops = {
  .read = plic_read,
  .write = plic_write,
  .endianness = DEVICE_NATIVE_ENDIAN,
  .valid = {
    .min_access_size = 1,
    .max_access_size = 8
  }
};


void plic_raise_irq(uint32_t irq)
{
    int hartid = 0;
    CPURISCVState *env = hart_get_env(hartid);
    uint32_t prio = plic_get_source_priority(irq);
    uint32_t prio_threshold = 0;

    plic_set_pending_claim(irq);

    while (env) {
        /* Don't set if priority set to 0 - effectively this disables the
         * interrupt */
        if (prio != 0) {
            /* Coreplex will mask all PLIC interrupts of a priority less than
             * or equal to threshold
             */
            prio_threshold = plic_get_priority_threshold(hartid, PRV_M);
            if (prio >= prio_threshold) { /* TODO: check this */
                if (env->mie & MIP_MEIP) {
                    PLIC_DEBUG("hart%d raising MEI\n", env->hart_index);
                    env->mip |= MIP_MEIP;
                    qemu_irq_raise(MEIP_IRQ);
                }
            }

            prio_threshold = plic_get_priority_threshold(hartid, PRV_S);
            if (prio >= prio_threshold) { /* TODO: check */
                if (env->mie & MIP_SEIP) {
                    PLIC_DEBUG("hart%d raising SEI\n", env->hart_index);
                    env->mip |= MIP_SEIP;
                    qemu_irq_raise(SEIP_IRQ);
                }
            }
       }

       env = hart_get_env(++hartid);
    }
}


void plic_lower_irq(uint32_t src)
{
    int hartid = 0;
    CPURISCVState *env = 0;

    env = hart_get_env(hartid);

    while (env) {
        if (env->mip & MIP_MEIP) {
            PLIC_DEBUG("hart%d lowering MEI\n", env->hart_index);
            env->mip &= ~MIP_MEIP;
            qemu_irq_lower(MEIP_IRQ);
        }

        if (env->mip & MIP_SEIP) {
            PLIC_DEBUG("hart%d lowering SEI\n", env->hart_index);
            env->mip &= ~MIP_SEIP;
            qemu_irq_lower(SEIP_IRQ);
        }

        env = hart_get_env(++hartid);
    }
}

void plic_init(RISCVCPU *cpu)
{
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *plic_io = g_new(MemoryRegion, 1);
    PLIC_DEBUG("registering PLIC as MMIO region\n");
    memory_region_init_io(plic_io, NULL, &plic_ops, cpu, "riscv_soc.plic",
        PLIC_REG_SZ);
    memory_region_add_subregion(system_memory, PLIC_BASE_ADDR, plic_io);
    QSIMPLEQ_INIT(&requests);
}

