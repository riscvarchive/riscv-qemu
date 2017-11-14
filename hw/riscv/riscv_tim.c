/*
 * QEMU RISC-V TIM device model
 * Author: Daire McNamara, daire.mcnamara@emdalo.com
 *
 * This provides a RISC-V TIM device model
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
#include "hw/riscv/riscv_tim.h"
#include "hw/riscv/soc.h"
#include "sysemu/arch_init.h"
#include "exec/address-spaces.h"
#include "hw/sysbus.h"             /* SysBusDevice */


typedef enum TimType_e {
    DTIM,
    ITIM
} TimType;

typedef struct TimWayState_s {
    uint32_t hartid;
    TimType  type;
    uint32_t way;
    uint64_t base;
} TimWayState;

typedef struct TimCfg_s {
    uint64_t base;
    uint32_t cache_sz;
    uint32_t way_sz;
    uint32_t num_ways;
} TimCfg;

/* #define DEBUG_TIM */

#ifdef DEBUG_TIM
#define TIM_DEBUG(fmt, ...) \
    do { \
        fprintf(stderr, "tim: " fmt, ## __VA_ARGS__);\
    } while (0)
#else
#define TIM_DEBUG(fmt, ...) \
    do {} while (0)
#endif

#define TIM_WARN(fmt, ...) \
    do { \
        fprintf(stderr, "tim: WARNING " fmt, ## __VA_ARGS__); \
    } while (0)

#define TIM_ERR(fmt, ...) \
    do { \
        fprintf(stderr, "tim: ERROR " fmt, ## __VA_ARGS__); exit(-1); \
    } while (0)

#define TIM_TYPE_TO_STRING(type) (type == DTIM ? "dtim" : "itim")


MemoryRegion **tim_io[NUM_HARTS][NUM_TIM_TYPES] = { 0 };
MemoryRegion **tim_ram[NUM_HARTS][NUM_TIM_TYPES] = { 0 } ;


TimCfg tim_cfg[5][2] = { 0 };

static void tim_io_write(void *opaque, hwaddr addr, uint64_t val64,
    unsigned int size)
{
    TimWayState *tws = (TimWayState *)opaque;

    if (opaque == NULL) {
        TIM_ERR("invalid write to %lx\n", addr);
    }

    uint32_t way  = tws->way;
    uint32_t type = tws->type;
    uint32_t hartid = tws->hartid;
    uint32_t active_way = 0;

    TIM_DEBUG("writing 0x%lx to offset 0x%lx %s way%d for hart%d"
            " (base: 0x%lx)\n", val64, addr, TIM_TYPE_TO_STRING(type), way,
            hartid, tws->base);

    if ((way == tim_cfg[hartid][type].num_ways - 1) && (addr == 0)
        && (val64 == 0)) {
        TIM_DEBUG("hart%d converting %s rams to io\n", hartid,
            TIM_TYPE_TO_STRING(type));

        /* remove all tim_rams from System Memory */
        for (active_way = 0; active_way < tim_cfg[hartid][type].num_ways - 1;
            active_way++) {
            memory_region_set_enabled(tim_ram[hartid][type][way], false);
            memory_region_set_enabled(tim_io[hartid][type][way], true);
        }
    } else {
        TIM_DEBUG("hart%d converting %s way%d io to ram\n", hartid,
            TIM_TYPE_TO_STRING(type), way);

        /* remove tim_io from System Memory */
        for (active_way = 0; active_way <= way; active_way++) {
            memory_region_set_enabled(tim_io[hartid][type][active_way], false);
            memory_region_set_enabled(tim_ram[hartid][type][active_way], true);
        }

        /* write into the System memory */
        /* memory_region_dispatch_write(get_system_memory(), tws->base+addr,
         *     val64, size,  MEMTXATTRS_UNSPECIFIED); */
        void *ptr = qemu_map_ram_ptr(tim_ram[hartid][type][way]->ram_block,
            addr);
        memcpy(ptr, &val64, size);
        /* invalidate_and_set_dirty(tim_ram[hartid][type][way], addr. size); */
    }
}

static uint64_t tim_io_read(void *opaque, hwaddr addr, unsigned int size)
{
    TIM_ERR("can't read before writing\n");
    return 0;
}

static const MemoryRegionOps tim_ops = {
    .read = tim_io_read,
    .write = tim_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8
    }
};


static void tim_allocate_cache(uint32_t hartid, uint32_t type,
    uint32_t num_ways)
{
    assert(tim_io[hartid][type] == NULL);
    assert(tim_ram[hartid][type] == NULL);

    tim_io[hartid][type] = g_malloc0(sizeof(MemoryRegion *) * num_ways);
    tim_ram[hartid][type] = g_malloc0(sizeof(MemoryRegion *) * num_ways);
}

static void tim_deallocate_cache(uint32_t hartid, uint32_t type,
    uint32_t num_ways)
{
    assert(tim_io[hartid][type]);
    assert(tim_ram[hartid][type]);

    g_free(tim_io[hartid][type]);
    g_free(tim_ram[hartid][type]);
}


static void tim_init(RISCVCPU *cpu, TimType type, uint64_t base,
     uint32_t cache_sz, uint32_t way_sz)
{
    /* allocate each way and one more way above the top way - this is used to
     * revert the [di]tim to L1 cache */
    uint32_t num_ways = cache_sz / way_sz + 1;
    CPURISCVState *env = &cpu->env;
    char memname[128] = { 0 };
    uint32_t hartid = env->hart_index;
    int way;
    MemoryRegion *system_memory = get_system_memory();

    tim_cfg[hartid][type].base = base;
    tim_cfg[hartid][type].cache_sz = cache_sz;
    tim_cfg[hartid][type].way_sz = way_sz;
    tim_cfg[hartid][type].num_ways = num_ways;

    tim_allocate_cache(hartid, type, num_ways);

    for (way = 0; way < num_ways; way++) {
        tim_io[hartid][type][way] = g_new(MemoryRegion, 1);
        tim_ram[hartid][type][way] = g_new(MemoryRegion, 1);
        TimWayState *tws = g_malloc0(sizeof(TimWayState));
        tws->hartid = hartid;
        tws->type = type;
        tws->way = way;
        tws->base = base + (way * way_sz);

        /* Register each way as I/O initially */
        memset(memname, 0, 128);
        snprintf(memname, 128, "riscv_soc.%s.hart%d.io.way%d",
            TIM_TYPE_TO_STRING(type), hartid, way);
        memory_region_init_io(tim_io[hartid][type][way], NULL, &tim_ops,
            tws, memname, way_sz);
        memory_region_add_subregion(system_memory, base + (way * way_sz),
            tim_io[hartid][type][way]);

        /* Create a corresponding RAM region for each I/O region - these will be
           swapped in/out as needed */
        snprintf(memname, 128, "riscv_soc.%s.hart%d.ram.way%d",
            TIM_TYPE_TO_STRING(type), hartid, way);
        memory_region_init_ram(tim_ram[hartid][type][way], NULL, memname,
            way_sz, &error_fatal);
        memory_region_add_subregion(system_memory, base + (way * way_sz),
           tim_ram[hartid][type][way]);

        /* Initialise to false */
        memory_region_set_enabled(tim_ram[hartid][type][way], false);
    }
}

static void tim_release(RISCVCPU *cpu, TimType type, uint64_t base,
    uint32_t cache_sz, uint32_t way_sz)
{
    uint32_t num_ways = cache_sz / way_sz + 1;
    CPURISCVState *env = &cpu->env;
    uint32_t hartid = env->hart_index;
    int way;

    for (way = 0; way < num_ways; way++) {
        g_free(tim_io[hartid][type][way]);
    }

    tim_deallocate_cache(hartid, type, num_ways);

}

void dtim_init(RISCVCPU *cpu, uint64_t base, uint32_t cache_sz,
    uint32_t way_sz)
{
    TIM_DEBUG("initialising DTIM for hart%d\n", cpu->env.hart_index);
    tim_init(cpu, DTIM, base, cache_sz, way_sz);
}

void itim_init(RISCVCPU *cpu, uint64_t base, uint32_t cache_sz,
    uint32_t way_sz)
{
    TIM_DEBUG("initialising ITIM for hart%d\n", cpu->env.hart_index);
    tim_init(cpu, ITIM, base, cache_sz, way_sz);
}

void dtim_release(RISCVCPU *cpu, uint64_t base, uint32_t cache_sz,
    uint32_t way_sz)
{
    tim_release(cpu, DTIM, base, cache_sz, way_sz);
}

void itim_release(RISCVCPU *cpu, uint64_t base, uint32_t cache_sz,
    uint32_t way_sz)
{
    tim_release(cpu, ITIM, base, cache_sz, way_sz);
}


