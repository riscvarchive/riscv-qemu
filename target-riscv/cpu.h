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
#define TARGET_LONG_BITS 64 /* this defs TCGv as TCGv_i64 in tcg/tcg-op.h */
#define TARGET_PHYS_ADDR_SPACE_BITS 50
#define TARGET_VIRT_ADDR_SPACE_BITS 39
#elif defined(TARGET_RISCV32)
#define TARGET_LONG_BITS 32 /* this defs TCGv as TCGv_i64 in tcg/tcg-op.h */
#define TARGET_PHYS_ADDR_SPACE_BITS 34
#define TARGET_VIRT_ADDR_SPACE_BITS 32
#endif

#include "exec/cpu-defs.h"
#include "fpu/softfloat.h"

#define TRANSLATE_FAIL -1
#define TRANSLATE_SUCCESS 0
#define NB_MMU_MODES 4

struct CPURISCVState;

/* Below taken from Spike's decode.h and encoding.h.
 * Using these directly drastically simplifies updating to new versions of the
 * RISC-V privileged specification */

#define get_field(reg, mask) (((reg) & \
                 (target_ulong)(mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(target_ulong)(mask)) | \
                 (((target_ulong)(val) * ((mask) & ~((mask) << 1))) & \
                 (target_ulong)(mask)))

#define PGSHIFT 12

#define FP_RD_NE  0
#define FP_RD_0   1
#define FP_RD_DN  2
#define FP_RD_UP  3
#define FP_RD_NMM 4

#define FSR_RD_SHIFT 5
#define FSR_RD   (0x7 << FSR_RD_SHIFT)

#define FPEXC_NX 0x01
#define FPEXC_UF 0x02
#define FPEXC_OF 0x04
#define FPEXC_DZ 0x08
#define FPEXC_NV 0x10

#define FSR_AEXC_SHIFT 0
#define FSR_NVA  (FPEXC_NV << FSR_AEXC_SHIFT)
#define FSR_OFA  (FPEXC_OF << FSR_AEXC_SHIFT)
#define FSR_UFA  (FPEXC_UF << FSR_AEXC_SHIFT)
#define FSR_DZA  (FPEXC_DZ << FSR_AEXC_SHIFT)
#define FSR_NXA  (FPEXC_NX << FSR_AEXC_SHIFT)
#define FSR_AEXC (FSR_NVA | FSR_OFA | FSR_UFA | FSR_DZA | FSR_NXA)

#define CSR_FFLAGS 0x1
#define CSR_FRM 0x2
#define CSR_FCSR 0x3
#define CSR_CYCLE 0xc00
#define CSR_TIME 0xc01
#define CSR_INSTRET 0xc02
#define CSR_SSTATUS 0x100
#define CSR_SIE 0x104
#define CSR_STVEC 0x105
#define CSR_SSCRATCH 0x140
#define CSR_SEPC 0x141
#define CSR_SCAUSE 0x142
#define CSR_SBADADDR 0x143
#define CSR_SIP 0x144
#define CSR_SPTBR 0x180
#define CSR_SCYCLE 0xd00
#define CSR_STIME 0xd01
#define CSR_SINSTRET 0xd02
#define CSR_MSTATUS 0x300
#define CSR_MEDELEG 0x302
#define CSR_MIDELEG 0x303
#define CSR_MIE 0x304
#define CSR_MTVEC 0x305
#define CSR_MSCRATCH 0x340
#define CSR_MEPC 0x341
#define CSR_MCAUSE 0x342
#define CSR_MBADADDR 0x343
#define CSR_MIP 0x344
#define CSR_MUCOUNTEREN 0x310
#define CSR_MSCOUNTEREN 0x311
#define CSR_MUCYCLE_DELTA 0x700
#define CSR_MUTIME_DELTA 0x701
#define CSR_MUINSTRET_DELTA 0x702
#define CSR_MSCYCLE_DELTA 0x704
#define CSR_MSTIME_DELTA 0x705
#define CSR_MSINSTRET_DELTA 0x706
#define CSR_TDRSELECT 0x7a0
#define CSR_TDRDATA1 0x7a1
#define CSR_TDRDATA2 0x7a2
#define CSR_TDRDATA3 0x7a3
#define CSR_DCSR 0x7b0
#define CSR_DPC 0x7b1
#define CSR_DSCRATCH 0x7b2
#define CSR_MCYCLE 0xf00
#define CSR_MTIME 0xf01
#define CSR_MINSTRET 0xf02
#define CSR_MISA 0xf10
#define CSR_MVENDORID 0xf11
#define CSR_MARCHID 0xf12
#define CSR_MIMPID 0xf13
#define CSR_MHARTID 0xf14
#define CSR_MRESET 0x7c2
#define CSR_CYCLEH 0xc80
#define CSR_TIMEH 0xc81
#define CSR_INSTRETH 0xc82
#define CSR_MUCYCLE_DELTAH 0x780
#define CSR_MUTIME_DELTAH 0x781
#define CSR_MUINSTRET_DELTAH 0x782
#define CSR_MSCYCLE_DELTAH 0x784
#define CSR_MSTIME_DELTAH 0x785
#define CSR_MSINSTRET_DELTAH 0x786
#define CSR_MCYCLEH 0xf80
#define CSR_MTIMEH 0xf81
#define CSR_MINSTRETH 0xf82

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

#define IS_RV_INTERRUPT(ival) (ival & (0x1 << 31))

#define MSTATUS_UIE         0x00000001
#define MSTATUS_SIE         0x00000002
#define MSTATUS_HIE         0x00000004
#define MSTATUS_MIE         0x00000008
#define MSTATUS_UPIE        0x00000010
#define MSTATUS_SPIE        0x00000020
#define MSTATUS_HPIE        0x00000040
#define MSTATUS_MPIE        0x00000080
#define MSTATUS_SPP         0x00000100
#define MSTATUS_HPP         0x00000600
#define MSTATUS_MPP         0x00001800
#define MSTATUS_FS          0x00006000
#define MSTATUS_XS          0x00018000
#define MSTATUS_MPRV        0x00020000
#define MSTATUS_PUM         0x00040000
#define MSTATUS_MXR         0x00080000
#define MSTATUS_VM          0x1F000000

#define MSTATUS32_SD        0x80000000
#define MSTATUS64_SD        0x8000000000000000

#define SSTATUS_UIE         0x00000001
#define SSTATUS_SIE         0x00000002
#define SSTATUS_UPIE        0x00000010
#define SSTATUS_SPIE        0x00000020
#define SSTATUS_SPP         0x00000100
#define SSTATUS_FS          0x00006000
#define SSTATUS_XS          0x00018000
#define SSTATUS_PUM         0x00040000
#define SSTATUS32_SD        0x80000000
#define SSTATUS64_SD        0x8000000000000000

#define DCSR_XDEBUGVER      (3U << 30)
#define DCSR_NDRESET        (1 << 29)
#define DCSR_FULLRESET      (1 << 28)
#define DCSR_HWBPCOUNT      (0xfff << 16)
#define DCSR_EBREAKM        (1 << 15)
#define DCSR_EBREAKH        (1 << 14)
#define DCSR_EBREAKS        (1 << 13)
#define DCSR_EBREAKU        (1 << 12)
#define DCSR_STOPCYCLE      (1 << 10)
#define DCSR_STOPTIME       (1 << 9)
#define DCSR_CAUSE          (7 << 6)
#define DCSR_DEBUGINT       (1 << 5)
#define DCSR_HALT           (1 << 3)
#define DCSR_STEP           (1 << 2)
#define DCSR_PRV            (3 << 0)

#define DCSR_CAUSE_NONE     0
#define DCSR_CAUSE_SWBP     1
#define DCSR_CAUSE_HWBP     2
#define DCSR_CAUSE_DEBUGINT 3
#define DCSR_CAUSE_STEP     4
#define DCSR_CAUSE_HALT     5

#define MIP_SSIP            (1 << IRQ_S_SOFT)
#define MIP_HSIP            (1 << IRQ_H_SOFT)
#define MIP_MSIP            (1 << IRQ_M_SOFT)
#define MIP_STIP            (1 << IRQ_S_TIMER)
#define MIP_HTIP            (1 << IRQ_H_TIMER)
#define MIP_MTIP            (1 << IRQ_M_TIMER)
#define MIP_SEIP            (1 << IRQ_S_EXT)
#define MIP_HEIP            (1 << IRQ_H_EXT)
#define MIP_MEIP            (1 << IRQ_M_EXT)

#define SIP_SSIP MIP_SSIP
#define SIP_STIP MIP_STIP

#define PRV_U 0
#define PRV_S 1
#define PRV_H 2
#define PRV_M 3

#define VM_MBARE 0
#define VM_MBB   1
#define VM_MBBID 2
#define VM_SV32  8
#define VM_SV39  9
#define VM_SV48  10

#define IRQ_S_SOFT   1
#define IRQ_H_SOFT   2
#define IRQ_M_SOFT   3
#define IRQ_S_TIMER  5
#define IRQ_H_TIMER  6
#define IRQ_M_TIMER  7
#define IRQ_S_EXT    9
#define IRQ_H_EXT    10
#define IRQ_M_EXT    11
#define IRQ_COP      12
#define IRQ_HOST     13

#define DEFAULT_RSTVEC     0x00001000
#define DEFAULT_NMIVEC     0x00001004
#define DEFAULT_MTVEC      0x00001010
#define CONFIG_STRING_ADDR 0x0000100C
#define EXT_IO_BASE        0x40000000
#define DRAM_BASE          0x80000000

/* breakpoint control fields */
#define BPCONTROL_X           0x00000001
#define BPCONTROL_W           0x00000002
#define BPCONTROL_R           0x00000004
#define BPCONTROL_U           0x00000008
#define BPCONTROL_S           0x00000010
#define BPCONTROL_H           0x00000020
#define BPCONTROL_M           0x00000040
#define BPCONTROL_BPMATCH     0x00000780
#define BPCONTROL_BPAMASKMAX 0x0F80000000000000
#define BPCONTROL_TDRTYPE    0xF000000000000000

/* page table entry (PTE) fields */
#define PTE_V     0x001 /* Valid */
#define PTE_R     0x002 /* Read */
#define PTE_W     0x004 /* Write */
#define PTE_X     0x008 /* Execute */
#define PTE_U     0x010 /* User */
#define PTE_G     0x020 /* Global */
#define PTE_A     0x040 /* Accessed */
#define PTE_D     0x080 /* Dirty */
#define PTE_SOFT  0x300 /* Reserved for Software */

#define PTE_PPN_SHIFT 10

#define PTE_TABLE(PTE) (((PTE) & (PTE_V | PTE_R | PTE_W | PTE_X)) == PTE_V)
/* end Spike decode.h, encoding.h section */

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
    target_ulong PC;
    target_ulong load_res;

    target_ulong csr[4096]; /* RISCV CSR registers */
    target_ulong priv;
    target_ulong badaddr;

    /* temporary htif regs */
    uint64_t mfromhost;
    uint64_t mtohost;

    uint64_t timecmp;
    float_status fp_status;

    /* QEMU */
    CPU_COMMON

    /* Fields from here on are preserved across CPU reset. */
    const riscv_def_t *cpu_model;
    size_t memsize;
    void *irq[8];
    QEMUTimer *timer; /* Internal timer */
};

#ifndef QEMU_RISCV_CPU_QOM_H
#define QEMU_RISCV_CPU_QOM_H

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
#endif

#if !defined(CONFIG_USER_ONLY)
void riscv_cpu_unassigned_access(CPUState *cpu, hwaddr addr, bool is_write,
        bool is_exec, int unused, unsigned size);
#endif

void riscv_cpu_list(FILE *f, fprintf_function cpu_fprintf);

#define cpu_signal_handler cpu_riscv_signal_handler
#define cpu_list riscv_cpu_list

static int ctz(target_ulong val);
int validate_priv(target_ulong priv);
void set_privilege(CPURISCVState *env, target_ulong newpriv);
unsigned int softfloat_flags_to_riscv(unsigned int flag);
uint_fast16_t float32_classify(uint32_t a, float_status *status);
uint_fast16_t float64_classify(uint64_t a, float_status *status);

/*
 * Compute mmu index
 * Adapted from Spike's mmu_t::translate
 */
static inline int cpu_mmu_index(CPURISCVState *env, bool ifetch)
{
    target_ulong mode = env->priv;
    if (!ifetch) {
        if (get_field(env->csr[CSR_MSTATUS], MSTATUS_MPRV)) {
            mode = get_field(env->csr[CSR_MSTATUS], MSTATUS_MPP);
        }
    }
    if (get_field(env->csr[CSR_MSTATUS], MSTATUS_VM) == VM_MBARE) {
        mode = PRV_M;
    }
    return mode;
}

/*
 * ctz in Spike returns 0 if val == 0, wrap helper
 */
static int ctz(target_ulong val)
{
    return val ? ctz64(val) : 0;
}

/*
 * Return RISC-V IRQ number if an interrupt should be taken, else -1.
 * Used in cpu-exec.c
 *
 * Adapted from Spike's processor_t::take_interrupt()
 */
static inline int cpu_riscv_hw_interrupts_pending(CPURISCVState *env)
{
    target_ulong pending_interrupts = env->csr[CSR_MIP] & env->csr[CSR_MIE];

    target_ulong mie = get_field(env->csr[CSR_MSTATUS], MSTATUS_MIE);
    target_ulong m_enabled = env->priv < PRV_M || (env->priv == PRV_M && mie);
    target_ulong enabled_interrupts = pending_interrupts &
                                      ~env->csr[CSR_MIDELEG] & -m_enabled;

    target_ulong sie = get_field(env->csr[CSR_MSTATUS], MSTATUS_SIE);
    target_ulong s_enabled = env->priv < PRV_S || (env->priv == PRV_S && sie);
    enabled_interrupts |= pending_interrupts & env->csr[CSR_MIDELEG] &
                          -s_enabled;

    if (enabled_interrupts) {
        target_ulong counted = ctz(enabled_interrupts);
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

#include "exec/cpu-all.h"

void riscv_tcg_init(void);
RISCVCPU *cpu_riscv_init(const char *cpu_model);
int cpu_riscv_signal_handler(int host_signum, void *pinfo, void *puc);

#define cpu_init(cpu_model) CPU(cpu_riscv_init(cpu_model))

/* hw/riscv/riscv_rtc.c  - supplies instret by approximating */
uint64_t cpu_riscv_read_instret(CPURISCVState *env);

int riscv_cpu_handle_mmu_fault(CPUState *cpu, vaddr address, MMUAccessType rw,
                              int mmu_idx);
#if !defined(CONFIG_USER_ONLY)
hwaddr cpu_riscv_translate_address(CPURISCVState *env, target_ulong address,
                                   int rw);
#endif

static inline void cpu_get_tb_cpu_state(CPURISCVState *env, target_ulong *pc,
                                        target_ulong *cs_base, uint32_t *flags)
{
    *pc = env->PC;
    *cs_base = 0;
    *flags = 0; /* necessary to avoid compiler warning */
}

#ifndef CONFIG_USER_ONLY
void csr_write_helper(CPURISCVState *env, target_ulong val_to_write,
        target_ulong csrno);
target_ulong csr_read_helper(CPURISCVState *env, target_ulong csrno);
#endif

void validate_csr(CPURISCVState *env, uint64_t which, uint64_t write, uint64_t
        new_pc);

#include "exec/exec-all.h"

#endif /* RISCV_CPU_H */
