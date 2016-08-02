#if !defined (__RISCV_CPU_H__)
#define __RISCV_CPU_H__

//#define DEBUG_OP

#define TARGET_HAS_ICE 1

#define ELF_MACHINE	EM_RISCV

#define CPUArchState struct CPURISCVState

#define RISCV_START_PC 0x200

#include "config.h"
#include "qemu-common.h"
#include "riscv-defs.h"
#include "exec/cpu-defs.h"
#include "fpu/softfloat.h"

#define TRANSLATE_FAIL -1
#define TRANSLATE_SUCCESS 0

#define NB_MMU_MODES 4

struct CPURISCVState;

#define PGSHIFT 12

// uncomment for lots of debug printing
//#define RISCV_DEBUG_PRINT

#define get_field(reg, mask) (((reg) & (target_ulong)(mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(target_ulong)(mask)) | (((target_ulong)(val) * ((mask) & ~((mask) << 1))) & (target_ulong)(mask)))


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

#define NEW_CSR_FFLAGS 0x1
#define NEW_CSR_FRM 0x2
#define NEW_CSR_FCSR 0x3
#define NEW_CSR_CYCLE 0xc00
#define NEW_CSR_TIME 0xc01
#define NEW_CSR_INSTRET 0xc02
#define NEW_CSR_STATS 0xc0
#define NEW_CSR_UARCH0 0xcc0
#define NEW_CSR_UARCH1 0xcc1
#define NEW_CSR_UARCH2 0xcc2
#define NEW_CSR_UARCH3 0xcc3
#define NEW_CSR_UARCH4 0xcc4
#define NEW_CSR_UARCH5 0xcc5
#define NEW_CSR_UARCH6 0xcc6
#define NEW_CSR_UARCH7 0xcc7
#define NEW_CSR_UARCH8 0xcc8
#define NEW_CSR_UARCH9 0xcc9
#define NEW_CSR_UARCH10 0xcca
#define NEW_CSR_UARCH11 0xccb
#define NEW_CSR_UARCH12 0xccc
#define NEW_CSR_UARCH13 0xccd
#define NEW_CSR_UARCH14 0xcce
#define NEW_CSR_UARCH15 0xccf
#define NEW_CSR_SSTATUS 0x100
#define NEW_CSR_STVEC 0x101
#define NEW_CSR_SIE 0x104
#define NEW_CSR_SSCRATCH 0x140
#define NEW_CSR_SEPC 0x141
#define NEW_CSR_SIP 0x144
#define NEW_CSR_SPTBR 0x180
#define NEW_CSR_SASID 0x181
#define NEW_CSR_CYCLEW 0x900
#define NEW_CSR_TIMEW 0x901
#define NEW_CSR_INSTRETW 0x902
#define NEW_CSR_STIME 0xd01
#define NEW_CSR_SCAUSE 0xd42
#define NEW_CSR_SBADADDR 0xd43
#define NEW_CSR_STIMEW 0xa01
#define NEW_CSR_MSTATUS 0x300
#define NEW_CSR_MTVEC 0x301
#define NEW_CSR_MTDELEG 0x302
#define NEW_CSR_MIE 0x304
#define NEW_CSR_MTIMECMP 0x321
#define NEW_CSR_MSCRATCH 0x340
#define NEW_CSR_MEPC 0x341
#define NEW_CSR_MCAUSE 0x342
#define NEW_CSR_MBADADDR 0x343
#define NEW_CSR_MIP 0x344
#define NEW_CSR_MTIME 0x701
#define NEW_CSR_MCPUID 0xf00
#define NEW_CSR_MIMPID 0xf01
#define NEW_CSR_MHARTID 0xf10
#define NEW_CSR_MTOHOST 0x780
#define NEW_CSR_MFROMHOST 0x781
#define NEW_CSR_MRESET 0x782
#define NEW_CSR_MIPI 0x783
#define NEW_CSR_MIOBASE 0x784
#define NEW_CSR_CYCLEH 0xc80
#define NEW_CSR_TIMEH 0xc81
#define NEW_CSR_INSTRETH 0xc82
#define NEW_CSR_CYCLEHW 0x980
#define NEW_CSR_TIMEHW 0x981
#define NEW_CSR_INSTRETHW 0x982
#define NEW_CSR_STIMEH 0xd81
#define NEW_CSR_STIMEHW 0xa81
#define NEW_CSR_MTIMECMPH 0x361
#define NEW_CSR_MTIMEH 0x741


// RISCV Exception Codes
#define EXCP_NONE                       -1   // not a real RISCV exception code
#define NEW_RISCV_EXCP_INST_ADDR_MIS           0x0
#define NEW_RISCV_EXCP_INST_ACCESS_FAULT       0x1
#define NEW_RISCV_EXCP_ILLEGAL_INST            0x2
#define NEW_RISCV_EXCP_BREAKPOINT              0x3
#define NEW_RISCV_EXCP_LOAD_ADDR_MIS           0x4
#define NEW_RISCV_EXCP_LOAD_ACCESS_FAULT       0x5
#define NEW_RISCV_EXCP_STORE_AMO_ADDR_MIS      0x6
#define NEW_RISCV_EXCP_STORE_AMO_ACCESS_FAULT  0x7
#define NEW_RISCV_EXCP_U_ECALL                 0x8 // for convenience, report all
                                                   // ECALLs as this, handler fixes
#define NEW_RISCV_EXCP_S_ECALL                 0x9
#define NEW_RISCV_EXCP_H_ECALL                 0xa
#define NEW_RISCV_EXCP_M_ECALL                 0xb
// >= 12 reserved
// interrupts not listed here

#define IS_RV_INTERRUPT(ival) (ival & (0x1 << 31))

#define MSTATUS_IE          0x00000001
#define MSTATUS_PRV         0x00000006
#define MSTATUS_IE1         0x00000008
#define MSTATUS_PRV1        0x00000030
#define MSTATUS_IE2         0x00000040
#define MSTATUS_PRV2        0x00000180
#define MSTATUS_IE3         0x00000200
#define MSTATUS_PRV3        0x00000C00
#define MSTATUS_FS          0x00003000
#define MSTATUS_XS          0x0000C000
#define MSTATUS_MPRV        0x00010000
#define MSTATUS_VM          0x003E0000
#define MSTATUS32_SD        0x80000000
#define MSTATUS64_SD        0x8000000000000000

#define SSTATUS_IE          0x00000001
#define SSTATUS_PIE         0x00000008
#define SSTATUS_PS          0x00000010
#define SSTATUS_FS          0x00003000
#define SSTATUS_XS          0x0000C000
#define SSTATUS_MPRV        0x00010000
#define SSTATUS_TIE         0x01000000
#define SSTATUS32_SD        0x80000000
#define SSTATUS64_SD        0x8000000000000000

#define MIP_SSIP            0x00000002
#define MIP_HSIP            0x00000004
#define MIP_MSIP            0x00000008
#define MIP_STIP            0x00000020
#define MIP_HTIP            0x00000040
#define MIP_MTIP            0x00000080

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

#define UA_RV32  0
#define UA_RV64  4
#define UA_RV128 8

#define IRQ_SOFT   0
#define IRQ_TIMER  1
#define IRQ_HOST   2
#define IRQ_COP    3

#define IMPL_ROCKET 1

#define DEFAULT_MTVEC 0x100

// page table entry (PTE) fields
#define PTE_V     0x001 // Valid
#define PTE_TYPE  0x01E // Type
#define PTE_R     0x020 // Referenced
#define PTE_D     0x040 // Dirty
#define PTE_SOFT  0x380 // Reserved for Software

#define PTE_TYPE_TABLE        0x00
#define PTE_TYPE_TABLE_GLOBAL 0x02
#define PTE_TYPE_URX_SR       0x04
#define PTE_TYPE_URWX_SRW     0x06
#define PTE_TYPE_UR_SR        0x08
#define PTE_TYPE_URW_SRW      0x0A
#define PTE_TYPE_URX_SRX      0x0C
#define PTE_TYPE_URWX_SRWX    0x0E
#define PTE_TYPE_SR           0x10
#define PTE_TYPE_SRW          0x12
#define PTE_TYPE_SRX          0x14
#define PTE_TYPE_SRWX         0x16
#define PTE_TYPE_SR_GLOBAL    0x18
#define PTE_TYPE_SRW_GLOBAL   0x1A
#define PTE_TYPE_SRX_GLOBAL   0x1C
#define PTE_TYPE_SRWX_GLOBAL  0x1E

#define PTE_PPN_SHIFT 10

#define PTE_TABLE(PTE) ((0x0000000AU >> ((PTE) & 0x1F)) & 1)
#define PTE_UR(PTE)    ((0x0000AAA0U >> ((PTE) & 0x1F)) & 1)
#define PTE_UW(PTE)    ((0x00008880U >> ((PTE) & 0x1F)) & 1)
#define PTE_UX(PTE)    ((0x0000A0A0U >> ((PTE) & 0x1F)) & 1)
#define PTE_SR(PTE)    ((0xAAAAAAA0U >> ((PTE) & 0x1F)) & 1)
#define PTE_SW(PTE)    ((0x88888880U >> ((PTE) & 0x1F)) & 1)
#define PTE_SX(PTE)    ((0xA0A0A000U >> ((PTE) & 0x1F)) & 1)

#define PTE_CHECK_PERM(PTE, SUPERVISOR, STORE, FETCH) \
  ((STORE) ? ((SUPERVISOR) ? PTE_SW(PTE) : PTE_UW(PTE)) : \
   (FETCH) ? ((SUPERVISOR) ? PTE_SX(PTE) : PTE_UX(PTE)) : \
             ((SUPERVISOR) ? PTE_SR(PTE) : PTE_UR(PTE)))

typedef struct riscv_def_t riscv_def_t;

typedef struct TCState TCState;
struct TCState {
    target_ulong gpr[32];
    target_ulong fpr[32];
    target_ulong PC;
    target_ulong load_reservation;
};

typedef struct CPURISCVState CPURISCVState;
struct CPURISCVState {
    TCState active_tc;
    uint32_t current_tc;
    uint32_t SEGBITS;
    uint32_t PABITS;

    uint64_t csr[4096]; // RISCV CSR registers

    // TODO set defaults
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
void  riscv_cpu_do_unaligned_access(CPUState *cs,
                                              target_ulong addr, int rw,
                                              int is_user, uintptr_t retaddr);


#endif

#if !defined(CONFIG_USER_ONLY)
void riscv_cpu_unassigned_access(CPUState *cpu, hwaddr addr, bool is_write,
        bool is_exec, int unused, unsigned size);
#endif

void riscv_cpu_list (FILE *f, fprintf_function cpu_fprintf);

#define cpu_exec cpu_riscv_exec
#define cpu_signal_handler cpu_riscv_signal_handler
#define cpu_list riscv_cpu_list

// TODO I think this is related to VMState stuff
// commenting it out breaks stuff, and there's an #ifdef CPU_SAVE_VERSION
// in include/qemu-common.h
#define CPU_SAVE_VERSION 3

static inline int cpu_mmu_index (CPURISCVState *env, bool ifetch)
{
    int mode = get_field(env->csr[NEW_CSR_MSTATUS], MSTATUS_PRV);

    if (!ifetch && get_field(env->csr[NEW_CSR_MSTATUS], MSTATUS_MPRV)) {
        mode = get_field(env->csr[NEW_CSR_MSTATUS], MSTATUS_PRV1);
    }
    if (get_field(env->csr[NEW_CSR_MSTATUS], MSTATUS_VM) == VM_MBARE) {
        mode = PRV_M;
    }

    return mode;
}

/*
 * Return RISC-V IRQ number if an interrupt should be taken, else -1.
 * Used in cpu-exec.c
 */
static inline int cpu_riscv_hw_interrupts_pending(CPURISCVState *env)
{

    int priv = get_field(env->csr[NEW_CSR_MSTATUS], MSTATUS_PRV);
    int ie = get_field(env->csr[NEW_CSR_MSTATUS], MSTATUS_IE);
    target_ulong interrupts = env->csr[NEW_CSR_MIE] & env->csr[NEW_CSR_MIP];

    #ifdef RISCV_DEBUG_PRINT
    printf("checking interrupts: ie %d, priv %d, interrupts %ld\n", ie, priv,
            interrupts);
    #endif

    if (priv < PRV_M || (priv == PRV_M && ie)) {
        if (interrupts & MIP_MSIP) {
            #ifdef RISCV_DEBUG_PRINT
            fprintf(stderr, "taking soft interrupt M\n");
            #endif

            // no irq to lower, that is done by the CPU
            return IRQ_SOFT;
        }

        if (interrupts & MIP_MTIP) {
            #ifdef RISCV_DEBUG_PRINT
            fprintf(stderr, "taking timer interrupt M\n");
            #endif

            // we're handing it to the cpu now, so get rid of the qemu irq
            qemu_irq_lower(env->irq[7]); // get rid of the irq request
            return IRQ_TIMER;
        }

        if (env->csr[NEW_CSR_MFROMHOST]) {
            #ifdef RISCV_DEBUG_PRINT
            fprintf(stderr, "taking host interrupt\n");
            #endif

            // we're handing it to the cpu now, so get rid of the qemu irq
            qemu_irq_lower(env->irq[4]); // get rid of the irq request
            return IRQ_HOST;
        }

    }

    if (priv < PRV_S || (priv == PRV_S && ie)) {
        if (interrupts & MIP_SSIP) {
            #ifdef RISCV_DEBUG_PRINT
            fprintf(stderr, "taking soft interrupt S\n");
            #endif

            // no irq to lower, that is done by the CPU
            return IRQ_SOFT;
        }

        if (interrupts & MIP_STIP) {
            #ifdef RISCV_DEBUG_PRINT
            fprintf(stderr, "taking timer interrupt S\n");
            #endif

            // no irq to lower, that is done by the CPU
            return IRQ_TIMER;
        }
    }

    // indicates no pending interrupt to handler in cpu-exec.c
    return -1;
}

#include "exec/cpu-all.h"

int cpu_riscv_exec(CPUState *cpu);
void riscv_tcg_init(void);
RISCVCPU *cpu_riscv_init(const char *cpu_model);
int cpu_riscv_signal_handler(int host_signum, void *pinfo, void *puc);

#define cpu_init(cpu_model) CPU(cpu_riscv_init(cpu_model))

/* TODO QOM'ify CPU reset and remove */
void cpu_state_reset(CPURISCVState *s);

/* hw/riscv/cputimer.c */
uint64_t cpu_riscv_get_cycle (CPURISCVState *env);
uint32_t cpu_riscv_get_random (CPURISCVState *env);
void cpu_riscv_store_compare (CPURISCVState *env, uint64_t value);
void cpu_riscv_start_count(CPURISCVState *env);


void cpu_riscv_store_timew(CPURISCVState *env, uint64_t val_to_write);
uint64_t cpu_riscv_read_mtime(CPURISCVState *env);
uint64_t cpu_riscv_read_stime(CPURISCVState *env);
uint64_t cpu_riscv_read_time(CPURISCVState *env);

void cpu_riscv_store_instretw(CPURISCVState *env, uint64_t val_to_write);
uint64_t cpu_riscv_read_instretw(CPURISCVState *env);

/* hw/riscv/riscv_int.c */
void cpu_riscv_soft_irq(CPURISCVState *env, int irq, int level);

/* helper.c */
int riscv_cpu_handle_mmu_fault(CPUState *cpu, vaddr address, int rw,
                              int mmu_idx);
#if !defined(CONFIG_USER_ONLY)
hwaddr cpu_riscv_translate_address (CPURISCVState *env, target_ulong address,
		                               int rw);
#endif

static inline void cpu_get_tb_cpu_state(CPURISCVState *env, target_ulong *pc,
                                        target_ulong *cs_base, int *flags)
{
    *pc = env->active_tc.PC;
    *cs_base = 0;
    *flags = 0; // necessary to avoid compiler warning
}

target_ulong push_priv_stack(target_ulong start_mstatus);
target_ulong pop_priv_stack(target_ulong start_mstatus);

#ifndef CONFIG_USER_ONLY
void csr_write_helper(CPURISCVState *env, target_ulong val_to_write,
        target_ulong csrno);
target_ulong csr_read_helper(CPURISCVState *env, target_ulong csrno);
#endif

void validate_csr(CPURISCVState *env, uint64_t which, uint64_t write, uint64_t
        new_pc);

#include "exec/exec-all.h"

#endif /* !defined (__RISCV_CPU_H__) */
