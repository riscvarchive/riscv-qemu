/*
 * QEMU RISC-V L2 Cache Driver
 * Author: Daire McNamara, daire.mcnamara@emdalo.com
 *
 * This provides a RISC-V L2 Cache device model
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
#include "hw/riscv/riscv_l2cache.h"
#include "hw/riscv/soc.h"
#include "sysemu/arch_init.h"
#include "exec/address-spaces.h"

typedef struct cache_cfg_s {
    uint32_t banks;
    uint32_t ways;
    uint32_t sets;
    uint32_t bytes_per_block;
    uint32_t way_enb;
} cache_cfg_t;

cache_cfg_t cc = { 0 };

uint64_t way_mask_cache[NUM_HARTS];
MemoryRegion **lim_ram;

/* #define DEBUG_CACHE_CTRL */

#ifdef DEBUG_CACHE_CTRL
#define CC_DEBUG(fmt, ...) \
    do {\
        fprintf(stderr, "cachecfg: " fmt, ## __VA_ARGS__); \
    } while (0)
#else
#define CC_DEBUG(fmt, ...) \
    do {} while (0)
#endif

#define CC_WARN(fmt, ...) \
    do {\
        fprintf(stderr, "cachecfg: WARNING: " fmt, ## __VA_ARGS__); \
    } while (0)

#define CC_ERR(fmt, ...) \
    do {\
        fprintf(stderr, "cachecfg: ERROR: " fmt, ## __VA_ARGS__); exit(-1); \
    } while (0)

/*
 *   if way_enb is initialized to 0, only way indexes less than or
 *   equal to that value can be used by the cache.  Therefore only
 *   the first way (way 0) is in the cache by default
 *
 *   The model for LIM used here is that each of the 16 ways in the
 *   cache corresponds to a contiguous block of memory in LIM mode
 *   so, something like:
 *       way index       LIM base addr       LIM size
 *       0               0x0800_0000         0x20_0000
 *       1               0x0820_0000         0x20_0000
 *       ...
 *      14               0x081C_0000         0x20_0000
 *      15               0x081E_0000         0x20_0000
 *
 *   way 0 is always part of the cache so initially map in
 *   ways 1 - 15 into L2 LIM.
 */
static void l2cache_update_lim(uint8_t way_enb)
{
    int way;

    for (way = 0; way <= way_enb; way++) {
        /* now used by cache. take it out of lim */
        memory_region_set_enabled(lim_ram[way], false);
    }
}

static void l2cache_init_lim(uint64_t base, uint32_t way_sz)
{
    int ways = cc.ways;
    int way = 0;
    MemoryRegion *system_memory = get_system_memory();
    char memname[128];

    lim_ram = g_malloc0(sizeof(MemoryRegion *) * ways);

    for (way = 0; way < ways; way++) {
        lim_ram[way] = g_new(MemoryRegion, 1);
        memset(memname, 0, 128);
        snprintf(memname, 128, "riscv_soc.lim.way%d", way);
        memory_region_init_ram(lim_ram[way], NULL, memname, way_sz,
            &error_fatal);
        memory_region_add_subregion(system_memory, base + (way * way_sz),
            lim_ram[way]);
    }
}

static uint64_t ctrl_port_read(void *opaque, hwaddr addr, unsigned int size)
{
    uint64_t result;
    const hwaddr upperBound =
        CACHE_CTRL_WAY_MASK0_OFFSET + (sizeof(uint64_t) * NUM_HARTS);

    if ((addr >= CACHE_CTRL_WAY_MASK0_OFFSET) && (addr <= upperBound)) {
        uint8_t addr_cache = addr - CACHE_CTRL_WAY_MASK0_OFFSET;
        addr_cache >>= 3;
        return way_mask_cache[addr_cache];
    }

    switch (addr) {
    case CACHE_CFG_NUM_BANKS_OFFSET:
        /* 4 banks */
        result = cc.banks;
        break;

    case CACHE_CFG_NUM_WAYS_OFFSET:
        /* 16 ways */
        result = cc.ways;
        break;

    case CACHE_CFG_NUM_SETS_PER_BANK_OFFSET:
        /* 9 - 512 ways, return base - 2 log of 512 */
        result = cc.sets;
        break;

    case CACHE_CFG_NUM_BYTES_PER_BLOCK_OFFSET:
        /* 6 - 64 byte per block, return base - 2 log of 64 */
        result = cc.bytes_per_block;
        break;

    case CACHE_CTRL_WAY_ENABLE_OFFSET:
        result = cc.way_enb;
        break;

    case CACHE_CTRL_ECC_INJECT_ERROR_OFFSET:
    case CACHE_CTRL_FLUSH64_OFFSET:
    case CACHE_CTRL_FLUSH32_OFFSET:
        CC_DEBUG("register %lx is wo\n", addr);
        result = 0;
        break;

    case CACHE_CTRL_ECC_DIR_FIX_ADDR_OFFSET:
    case CACHE_CTRL_ECC_DIR_FIX_COUNT_OFFSET:
    case CACHE_CTRL_ECC_DATA_FIX_ADDR_OFFSET:
    case CACHE_CTRL_ECC_DATA_FIX_COUNT_OFFSET:
    case CACHE_CTRL_ECC_DATA_FAIL_ADDR_OFFSET:
    case CACHE_CTRL_ECC_DATA_FAIL_COUNT_OFFSET:
        result = 0;
        break;

    default:
        CC_WARN("read of unknown addr:  %lx\n", addr);
        result = 0;
        break;
    }

    return result;
}


static void ctrl_port_write(void *opaque, hwaddr addr, uint64_t val64,
    unsigned int size)
{
    const hwaddr upperBound =
        CACHE_CTRL_WAY_MASK0_OFFSET + (sizeof(uint64_t) * NUM_HARTS);

    if ((addr >= CACHE_CTRL_WAY_MASK0_OFFSET) && (addr <= upperBound)) {
        uint8_t addr_cache = addr - CACHE_CTRL_WAY_MASK0_OFFSET;
        addr_cache >>= 3;
        way_mask_cache[addr_cache] = val64;
        return;
    }

    switch (addr) {
    case CACHE_CTRL_WAY_ENABLE_OFFSET:
        /* For now, assume a mechanism for
         *  adding ways to the cache (and removing a 'way'
         *  from L2 LIM
         */
        if (val64 >= cc.way_enb) {
            cc.way_enb = val64;
            l2cache_update_lim(cc.way_enb);
        }
        break;

    case CACHE_CFG_NUM_BANKS_OFFSET:
    case CACHE_CFG_NUM_WAYS_OFFSET:
    case CACHE_CFG_NUM_SETS_PER_BANK_OFFSET:
    case CACHE_CFG_NUM_BYTES_PER_BLOCK_OFFSET:
    case CACHE_CTRL_ECC_DIR_FIX_ADDR_OFFSET:
    case CACHE_CTRL_ECC_DIR_FIX_COUNT_OFFSET:
    case CACHE_CTRL_ECC_DATA_FIX_ADDR_OFFSET:
    case CACHE_CTRL_ECC_DATA_FIX_COUNT_OFFSET:
    case CACHE_CTRL_ECC_DATA_FAIL_ADDR_OFFSET:
    case CACHE_CTRL_ECC_DATA_FAIL_COUNT_OFFSET:
        CC_WARN("register %lx is ro\n", addr);
        break;

    case CACHE_CTRL_ECC_INJECT_ERROR_OFFSET:
    case CACHE_CTRL_FLUSH64_OFFSET:
    case CACHE_CTRL_FLUSH32_OFFSET:
        CC_DEBUG("ignoring unsupported write to %lx\n", addr);
        break;

    default:
        CC_WARN("asked to write to unknown addr: %lx\n", addr);
        break;
    }
}

static void zero_dev_write(void *opaque, hwaddr addr, uint64_t val64,
    unsigned int size)
{
    CC_DEBUG("writing %lx to %lx\n", val64, addr);
}


static uint64_t zero_dev_read(void *opaque, hwaddr addr, unsigned int size)
{
    CC_ERR("reading from zero device (offset %lx)\n", addr);
}

static const MemoryRegionOps ctrl_port_ops = {
    .read = ctrl_port_read,
    .write = ctrl_port_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8
    }
};

static const MemoryRegionOps zero_dev_ops = {
    .read = zero_dev_read,
    .write = zero_dev_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8
    }
};

void l2cache_init(RISCVCPU *cpu, uint32_t banks, uint32_t ways, uint32_t sets,
    uint32_t bytes_per_block)
{
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *ctrl_port = g_new(MemoryRegion, 1);
    CC_DEBUG("registering Cache Control as  MMIO region\n");
    memory_region_init_io(ctrl_port, NULL, &ctrl_port_ops,
       cpu, "riscv_soc.ctrl_port_l2_cache", CACHE_CTRL_REG_SZ);
    memory_region_add_subregion(system_memory, CACHE_CTRL_BASE_ADDR,
       ctrl_port);

    memset(way_mask_cache, 0xff, NUM_HARTS * sizeof(uint64_t));
    cc.banks = banks; /* 4 */
    cc.ways = ways; /* 16 */
    cc.sets = 32 - clz32(sets); /* 9 (convert from 512) */
    cc.bytes_per_block = 32 - clz32(bytes_per_block); /* 6 (convert from 64) */
    cc.way_enb = 0;

    l2cache_init_lim(L2LIM_BASE_ADDR, L2LIM_WAY_SZ);
}

void l2zero_dev_init(uint64_t base, uint32_t sz)
{
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *zero_dev = g_new(MemoryRegion, 1);
    CC_DEBUG("registering L2 zero dev as MMIO region\n");
    memory_region_init_io(zero_dev, NULL, &zero_dev_ops,
        NULL, "riscv_soc.zero_dev", sz);
    memory_region_add_subregion(system_memory, base, zero_dev);
}

