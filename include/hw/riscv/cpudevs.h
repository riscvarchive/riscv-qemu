#ifndef HW_RISCV_CPUDEVS_H
#define HW_RISCV_CPUDEVS_H
/* Definitions for RISCV CPU internal devices.  */

/* mips_addr.c */
uint64_t cpu_mips_kseg0_to_phys(void *opaque, uint64_t addr);
uint64_t cpu_mips_phys_to_kseg0(void *opaque, uint64_t addr);

/* mips_int.c */
void cpu_riscv_irq_init_cpu(CPURISCVState *env);

/* mips_timer.c */
void cpu_riscv_clock_init(CPURISCVState *);

#endif
