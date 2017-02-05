#ifndef RISCV_CPU_H
#define RISCV_CPU_H

/*#define DEBUG_OP */

/* uncomment for lots of debug printing */
/* #define RISCV_DEBUG_PRINT */

#define TARGET_HAS_ICE 1
#define ELF_MACHINE EM_RISCV
#define CPUArchState struct CPURISCVState

#include "qemu-common.h"

/* QEMU addressing/paging config */
#define TARGET_PAGE_BITS 12 /* 4 KiB Pages */
#if defined(TARGET_RISCV64)
#define TARGET_LONG_BITS 64
#define TARGET_PHYS_ADDR_SPACE_BITS 50
#define TARGET_VIRT_ADDR_SPACE_BITS 39
#elif defined(TARGET_RISCV32)
#define TARGET_LONG_BITS 32
#define TARGET_PHYS_ADDR_SPACE_BITS 34
#define TARGET_VIRT_ADDR_SPACE_BITS 32
#endif

#include "exec/cpu-defs.h"
#include "fpu/softfloat.h"

/* RISCV Exception Codes */
#define EXCP_NONE                       -1 /* not a real RISCV exception code */
#define RISCV_EXCP_INST_ADDR_MIS           0x0
#define RISCV_EXCP_INST_ACCESS_FAULT       0x1
#define RISCV_EXCP_ILLEGAL_INST            0x2
#define RISCV_EXCP_BREAKPOINT              0x3
#define RISCV_EXCP_LOAD_ADDR_MIS           0x4
#define RISCV_EXCP_LOAD_ACCESS_FAULT       0x5
#define RISCV_EXCP_STORE_AMO_ADDR_MIS      0x6
#define RISCV_EXCP_STORE_AMO_ACCESS_FAULT  0x7
#define RISCV_EXCP_U_ECALL                 0x8 /* for convenience, report all
                                                  ECALLs as this, handler
                                                  fixes */
#define RISCV_EXCP_S_ECALL                 0x9
#define RISCV_EXCP_H_ECALL                 0xa
#define RISCV_EXCP_M_ECALL                 0xb


#define TRANSLATE_FAIL 1
#define TRANSLATE_SUCCESS 0
#define NB_MMU_MODES 4

#define MMU_USER_IDX 3

struct CPURISCVState;

#define SSIP_IRQ (env->irq[0])
#define STIP_IRQ (env->irq[1])
#define MSIP_IRQ (env->irq[2])
#define TIMER_IRQ (env->irq[3])
#define HTIF_IRQ (env->irq[4])

typedef struct riscv_def_t riscv_def_t;

typedef struct CPURISCVState CPURISCVState;
struct CPURISCVState {
    target_ulong gpr[32];
    uint64_t fpr[32]; /* assume both F and D extensions */
    target_ulong pc;
    target_ulong load_res;

    target_ulong frm;
    target_ulong fstatus;
    target_ulong fflags;

    target_ulong badaddr;

    uint32_t mucounteren;

#ifdef CONFIG_USER_ONLY
    uint32_t amoinsn;
    target_long amoaddr;
    target_long amotest;
#else
    target_ulong priv;

    target_ulong misa;
    target_ulong max_isa;
    target_ulong mstatus;

    target_ulong mip;
    target_ulong mie;
    target_ulong mideleg;

    target_ulong sptbr;
    target_ulong sbadaddr;
    target_ulong mbadaddr;
    target_ulong medeleg;

    target_ulong stvec;
    target_ulong sepc;
    target_ulong scause;

    target_ulong mtvec;
    target_ulong mepc;
    target_ulong mcause;

    uint32_t mscounteren;

    target_ulong sscratch;
    target_ulong mscratch;

    /* temporary htif regs */
    uint64_t mfromhost;
    uint64_t mtohost;
    uint64_t timecmp;
#endif

    float_status fp_status;

    /* Internal CPU feature flags. */
    uint64_t features;

    /* QEMU */
    CPU_COMMON

    /* Fields from here on are preserved across CPU reset. */
    const riscv_def_t *cpu_model;
    size_t memsize;
    void *irq[8];
    QEMUTimer *timer; /* Internal timer */
};

#include "qom/cpu.h"

#define TYPE_RISCV_CPU "riscv-cpu"

#define RISCV_CPU_CLASS(klass) \
    OBJECT_CLASS_CHECK(RISCVCPUClass, (klass), TYPE_RISCV_CPU)
#define RISCV_CPU(obj) \
    OBJECT_CHECK(RISCVCPU, (obj), TYPE_RISCV_CPU)
#define RISCV_CPU_GET_CLASS(obj) \
    OBJECT_GET_CLASS(RISCVCPUClass, (obj), TYPE_RISCV_CPU)

/**
 * RISCVCPUClass:
 * @parent_realize: The parent class' realize handler.
 * @parent_reset: The parent class' reset handler.
 *
 * A RISCV CPU model.
 */
typedef struct RISCVCPUClass {
    /*< private >*/
    CPUClass parent_class;
    /*< public >*/
    DeviceRealize parent_realize;
    void (*parent_reset)(CPUState *cpu);
} RISCVCPUClass;

/**
 * RISCVCPU:
 * @env: #CPURISCVState
 *
 * A RISCV CPU.
 */
typedef struct RISCVCPU {
    /*< private >*/
    CPUState parent_obj;
    /*< public >*/
    CPURISCVState env;
} RISCVCPU;

static inline RISCVCPU *riscv_env_get_cpu(CPURISCVState *env)
{
    return container_of(env, RISCVCPU, env);
}

enum riscv_features {
    RISCV_FEATURE_RVM,
    RISCV_FEATURE_RVA,
    RISCV_FEATURE_RVF,
    RISCV_FEATURE_RVD,
    RISCV_FEATURE_RVC,
};

static inline int riscv_feature(CPURISCVState *env, int feature)
{
    return (env->features & (1ULL << feature)) != 0;
}

#include "cpu_user.h"
#include "cpu_bits.h"

#define ENV_GET_CPU(e) CPU(riscv_env_get_cpu(e))
#define ENV_OFFSET offsetof(RISCVCPU, env)

void riscv_cpu_do_interrupt(CPUState *cpu);
void riscv_cpu_dump_state(CPUState *cpu, FILE *f, fprintf_function cpu_fprintf,
                          int flags);
hwaddr riscv_cpu_get_phys_page_debug(CPUState *cpu, vaddr addr);
int riscv_cpu_gdb_read_register(CPUState *cpu, uint8_t *buf, int reg);
int riscv_cpu_gdb_write_register(CPUState *cpu, uint8_t *buf, int reg);
bool riscv_cpu_exec_interrupt(CPUState *cs, int interrupt_request);
void  riscv_cpu_do_unaligned_access(CPUState *cs, vaddr addr,
                                    MMUAccessType access_type, int mmu_idx,
                                    uintptr_t retaddr);
#if !defined(CONFIG_USER_ONLY)
void riscv_cpu_unassigned_access(CPUState *cpu, hwaddr addr, bool is_write,
        bool is_exec, int unused, unsigned size);
#endif

void riscv_cpu_list(FILE *f, fprintf_function cpu_fprintf);

#define cpu_signal_handler cpu_riscv_signal_handler
#define cpu_list riscv_cpu_list

void set_privilege(CPURISCVState *env, target_ulong newpriv);
unsigned int softfloat_flags_to_riscv(unsigned int flag);
uint_fast16_t float32_classify(uint32_t a, float_status *status);
uint_fast16_t float64_classify(uint64_t a, float_status *status);

/*
 * Compute mmu index
 * Adapted from Spike's mmu_t::translate
 */
#ifdef CONFIG_USER_ONLY
static inline int cpu_mmu_index(CPURISCVState *env, bool ifetch)
{
    return 0;
}
#else
static inline int cpu_mmu_index(CPURISCVState *env, bool ifetch)
{
    target_ulong mode = env->priv;
    if (!ifetch) {
        if (get_field(env->mstatus, MSTATUS_MPRV)) {
            mode = get_field(env->mstatus, MSTATUS_MPP);
        }
    }
    if (get_field(env->mstatus, MSTATUS_VM) == VM_MBARE) {
        mode = PRV_M;
    }
    return mode;
}
#endif

#ifndef CONFIG_USER_ONLY
/*
 * Return RISC-V IRQ number if an interrupt should be taken, else -1.
 * Used in cpu-exec.c
 *
 * Adapted from Spike's processor_t::take_interrupt()
 */
static inline int cpu_riscv_hw_interrupts_pending(CPURISCVState *env)
{
    target_ulong pending_interrupts = env->mip & env->mie;

    target_ulong mie = get_field(env->mstatus, MSTATUS_MIE);
    target_ulong m_enabled = env->priv < PRV_M || (env->priv == PRV_M && mie);
    target_ulong enabled_interrupts = pending_interrupts &
                                      ~env->mideleg & -m_enabled;

    target_ulong sie = get_field(env->mstatus, MSTATUS_SIE);
    target_ulong s_enabled = env->priv < PRV_S || (env->priv == PRV_S && sie);
    enabled_interrupts |= pending_interrupts & env->mideleg &
                          -s_enabled;

    if (enabled_interrupts) {
        target_ulong counted = ctz64(enabled_interrupts); /* since non-zero */
        if (counted == IRQ_HOST) {
            /* we're handing it to the cpu now, so get rid of the qemu irq */
            qemu_irq_lower(HTIF_IRQ);
        } else if (counted == IRQ_M_TIMER) {
            /* we're handing it to the cpu now, so get rid of the qemu irq */
            qemu_irq_lower(TIMER_IRQ);
        } else if (counted == IRQ_S_TIMER || counted == IRQ_H_TIMER) {
            /* don't lower irq here */
        }
        return counted;
    } else {
        return EXCP_NONE; /* indicates no pending interrupt */
    }
}
#endif

#include "exec/cpu-all.h"

void riscv_tcg_init(void);
RISCVCPU *cpu_riscv_init(const char *cpu_model);
int cpu_riscv_signal_handler(int host_signum, void *pinfo, void *puc);
void QEMU_NORETURN do_raise_exception_err(CPURISCVState *env,
                                          uint32_t exception, uintptr_t pc);
#define cpu_init(cpu_model) CPU(cpu_riscv_init(cpu_model))

/* hw/riscv/riscv_rtc.c  - supplies instret by approximating */
uint64_t cpu_riscv_read_instret(CPURISCVState *env);

int riscv_cpu_handle_mmu_fault(CPUState *cpu, vaddr address, int rw,
                              int mmu_idx);
#if !defined(CONFIG_USER_ONLY)
hwaddr cpu_riscv_translate_address(CPURISCVState *env, target_ulong address,
                                   int rw);
#endif

static inline void cpu_get_tb_cpu_state(CPURISCVState *env, target_ulong *pc,
                                        target_ulong *cs_base, uint32_t *flags)
{
    *pc = env->pc;
    *cs_base = 0;
    *flags = 0; /* necessary to avoid compiler warning */
}

void csr_write_helper(CPURISCVState *env, target_ulong val_to_write,
        target_ulong csrno);
target_ulong csr_read_helper(CPURISCVState *env, target_ulong csrno);

void validate_csr(CPURISCVState *env, uint64_t which, uint64_t write);

#include "exec/exec-all.h"

#endif /* RISCV_CPU_H */
