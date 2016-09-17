#ifndef HW_RISCV_TIMER_H
#define HW_RISCV_TIMER_H 1

#include "hw/hw.h"
#include "sysemu/sysemu.h"
#include "exec/memory.h"
#include "target-riscv/cpu.h"

typedef struct TIMERState TIMERState;

struct TIMERState {
    CPURISCVState *env;
    MemoryRegion io;
    uint32_t timecmp_lower;
    uint64_t temp_rtc_val;
};

extern const VMStateDescription vmstate_timer_rv;
extern const MemoryRegionOps timer_io_ops;

/* legacy pre qom */
TIMERState *timer_mm_init(MemoryRegion *address_space, hwaddr base,
                          CPURISCVState *env);

#endif
