#ifndef HW_RISCV_CPUDEVS_H
#define HW_RISCV_CPUDEVS_H

#include "target-riscv/cpu.h"

/* Definitions for RISCV CPU internal devices.  */

/* riscv_int.c */
void cpu_riscv_irq_init_cpu(CPURISCVState *env);

/* cputimer.c */
void cpu_riscv_clock_init(CPURISCVState *);

#endif
