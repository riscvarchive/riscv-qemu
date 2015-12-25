#ifndef HW_RISCV_CPUDEVS_H
#define HW_RISCV_CPUDEVS_H
/* Definitions for RISCV CPU internal devices.  */

/* riscv_board.c */
uint64_t identity_translate(void *opaque, uint64_t addr);

/* riscv_int.c */
void cpu_riscv_irq_init_cpu(CPURISCVState *env);
void cpu_riscv_hartid_init_cpu(CPURISCVState *env, unsigned int id);


/* cputimer.c */
void cpu_riscv_clock_init(CPURISCVState *);

#endif
