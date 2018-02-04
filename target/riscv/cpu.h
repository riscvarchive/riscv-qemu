#ifndef RISCV_CPU_H
#define RISCV_CPU_H

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

#define ELF_MACHINE EM_RISCV
#define CPUArchState struct CPURISCVState

#include "qemu-common.h"
#include "qom/cpu.h"
#include "exec/cpu-defs.h"
#include "fpu/softfloat.h"

/* #define DEBUG_OP */
/* #define RISCV_DEBUG_PRINT */

#define TYPE_RISCV_CPU                    "riscv"
#define TYPE_RISCV_CPU_ANY                "riscv-any"
#define TYPE_RISCV_CPU_IMAFDCSU_PRIV_1_09 "riscv-imafdcsu-priv1.9"
#define TYPE_RISCV_CPU_IMAFDCSU_PRIV_1_10 "riscv-imafdcsu-priv1.10"
#define TYPE_RISCV_CPU_IMACU_PRIV_1_10    "riscv-imacu-priv1.10"
#define TYPE_RISCV_CPU_IMAC_PRIV_1_10     "riscv-imac-priv1.10"

#define RISCV_CPU_TYPE_PREFIX TYPE_RISCV_CPU "-"
#define RISCV_CPU_TYPE_NAME(name) (RISCV_CPU_TYPE_PREFIX name)

#if defined(TARGET_RISCV32)
#define RVXLEN  ((target_ulong)1 << (TARGET_LONG_BITS - 2))
#elif defined(TARGET_RISCV64)
#define RVXLEN  ((target_ulong)2 << (TARGET_LONG_BITS - 2))
#endif
#define RV(x) ((target_ulong)1 << (x - 'A'))

#define RVI RV('I')
#define RVM RV('M')
#define RVA RV('A')
#define RVF RV('F')
#define RVD RV('D')
#define RVC RV('C')
#define RVS RV('S')
#define RVU RV('U')

#define USER_VERSION_2_02_0 0x00020200
#define PRIV_VERSION_1_09_1 0x00010901
#define PRIV_VERSION_1_10_0 0x00011000

#define TRANSLATE_FAIL 1
#define TRANSLATE_SUCCESS 0
#define NB_MMU_MODES 4
#define MMU_USER_IDX 3

#define SSIP_IRQ (env->irq[0])
#define STIP_IRQ (env->irq[1])
#define MSIP_IRQ (env->irq[2])
#define MTIP_IRQ (env->irq[3])
#define HTIF_IRQ (env->irq[4])
#define SEIP_IRQ (env->irq[5])
#define MEIP_IRQ (env->irq[6])

#define MAX_RISCV_IRQ (8)
#define MAX_RISCV_PMPS (16)

typedef struct CPURISCVState CPURISCVState;

#include "pmp.h"

struct CPURISCVState {
    target_ulong gpr[32];
    uint64_t fpr[32]; /* assume both F and D extensions */
    target_ulong pc;
    target_ulong load_res;
    target_ulong load_val;

    target_ulong frm;

    target_ulong badaddr;

    uint32_t mucounteren;

    target_ulong user_ver;
    target_ulong priv_ver;
    target_ulong misa;

#ifndef CONFIG_USER_ONLY
    target_ulong priv;

    target_ulong mhartid;
    target_ulong mstatus;
    target_ulong mip;
    target_ulong mie;
    target_ulong mideleg;

    target_ulong sptbr;  /* until: priv-1.9.1 */
    target_ulong satp;   /* since: priv-1.10.0 */
    target_ulong sbadaddr;
    target_ulong mbadaddr;
    target_ulong medeleg;

    target_ulong stvec;
    target_ulong sepc;
    target_ulong scause;

    target_ulong mtvec;
    target_ulong mepc;
    target_ulong mcause;
    target_ulong mtval;  /* since: priv-1.10.0 */

    uint32_t mscounteren;
    target_ulong scounteren; /* since: priv-1.10.0 */
    target_ulong mcounteren; /* since: priv-1.10.0 */

    target_ulong sscratch;
    target_ulong mscratch;

    /* temporary htif regs */
    uint64_t mfromhost;
    uint64_t mtohost;
    uint64_t timecmp;

    /* physical memory protection */
    pmp_table_t pmp_state;
#endif

    float_status fp_status;

    /* QEMU */
    CPU_COMMON

    /* Fields from here on are preserved across CPU reset. */
    qemu_irq irq[8];
    QEMUTimer *timer; /* Internal timer */
};

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

static inline int riscv_has_ext(CPURISCVState *env, target_ulong ext)
{
    return (env->misa & ext) != 0;
}

#include "cpu_user.h"
#include "cpu_bits.h"

extern const char * const riscv_int_regnames[];
extern const char * const riscv_fpr_regnames[];
extern const char * const riscv_excp_names[];
extern const char * const riscv_intr_names[];

#define ENV_GET_CPU(e) CPU(riscv_env_get_cpu(e))
#define ENV_OFFSET offsetof(RISCVCPU, env)

void riscv_cpu_do_interrupt(CPUState *cpu);
int riscv_cpu_gdb_read_register(CPUState *cpu, uint8_t *buf, int reg);
int riscv_cpu_gdb_write_register(CPUState *cpu, uint8_t *buf, int reg);
bool riscv_cpu_exec_interrupt(CPUState *cs, int interrupt_request);
int riscv_cpu_mmu_index(CPURISCVState *env, bool ifetch);
hwaddr riscv_cpu_get_phys_page_debug(CPUState *cpu, vaddr addr);
void  riscv_cpu_do_unaligned_access(CPUState *cs, vaddr addr,
                                    MMUAccessType access_type, int mmu_idx,
                                    uintptr_t retaddr);
int riscv_cpu_handle_mmu_fault(CPUState *cpu, vaddr address, int size,
                              int rw, int mmu_idx);

char *riscv_isa_string(RISCVCPU *cpu);
void riscv_cpu_list(FILE *f, fprintf_function cpu_fprintf);

#define cpu_init(cpu_model) cpu_generic_init(TYPE_RISCV_CPU, cpu_model)
#define cpu_signal_handler cpu_riscv_signal_handler
#define cpu_list riscv_cpu_list
#define cpu_mmu_index riscv_cpu_mmu_index

void riscv_set_mode(CPURISCVState *env, target_ulong newpriv);

void riscv_translate_init(void);
RISCVCPU *cpu_riscv_init(const char *cpu_model);
int cpu_riscv_signal_handler(int host_signum, void *pinfo, void *puc);
void QEMU_NORETURN do_raise_exception_err(CPURISCVState *env,
                                          uint32_t exception, uintptr_t pc);

/* hw/riscv/sifive_clint.c  - supplies instret by approximating */
uint64_t cpu_riscv_read_instret(CPURISCVState *env);
uint64_t cpu_riscv_read_rtc(void);

target_ulong cpu_riscv_get_fflags(CPURISCVState *env);
void cpu_riscv_set_fflags(CPURISCVState *env, target_ulong);

#define TB_FLAGS_MMU_MASK  3
#define TB_FLAGS_FP_ENABLE MSTATUS_FS

static inline void cpu_get_tb_cpu_state(CPURISCVState *env, target_ulong *pc,
                                        target_ulong *cs_base, uint32_t *flags)
{
    *pc = env->pc;
    *cs_base = 0;
#ifdef CONFIG_USER_ONLY
    *flags = TB_FLAGS_FP_ENABLE;
#else
    *flags = cpu_mmu_index(env, 0) | (env->mstatus & MSTATUS_FS);
#endif
}

void csr_write_helper(CPURISCVState *env, target_ulong val_to_write,
        target_ulong csrno);
target_ulong csr_read_helper(CPURISCVState *env, target_ulong csrno);

#include "exec/cpu-all.h"

#endif /* RISCV_CPU_H */
