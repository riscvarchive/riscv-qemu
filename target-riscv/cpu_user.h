/* Return codes for riscv_cpu_do_userspace_amo */
#define RISCV_AMO_OK      0
#define RISCV_AMO_BADINSN 1
#define RISCV_AMO_BADADDR 2

/* not RISC-V exception codes - this is for qemu user-mode */
#define QEMU_USER_EXCP_ATOMIC              0xc
#define QEMU_USER_EXCP_FAULT               0xd

#define xRA 1   /* return address (aka link register) */
#define xSP 2   /* stack pointer */
#define xGP 3   /* global pointer */
#define xTP 4   /* thread pointer */

#define xA0 10  /* gpr[10-17] are syscall arguments */
#define xA1 11
#define xA2 12
#define xA3 13
#define xA4 14
#define xA5 15
#define xA6 16
#define xA7 17  /* syscall number goes here */

#ifdef CONFIG_USER_ONLY
int riscv_cpu_do_usermode_amo(CPUState* cs);

target_long riscv_arch_specific_syscall(CPURISCVState *env, int num,
        target_long cmd, target_long arg1, target_long arg2, target_long arg3);
#endif
