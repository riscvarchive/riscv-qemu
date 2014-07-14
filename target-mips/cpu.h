#if !defined (__MIPS_CPU_H__)
#define __MIPS_CPU_H__

//#define DEBUG_OP

#define TARGET_HAS_ICE 1

#define ELF_MACHINE	EM_MIPS

#define CPUArchState struct CPUMIPSState

#include "config.h"
#include "qemu-common.h"
#include "riscv-defs.h"
#include "exec/cpu-defs.h"

struct CPUMIPSState;

// TODO LOOK HERE FOR TLB SETTINGS
typedef struct r4k_tlb_t r4k_tlb_t;
struct r4k_tlb_t {
    target_ulong VPN;
    uint32_t PageMask;
    uint_fast8_t ASID;
    uint_fast16_t G:1;
    uint_fast16_t C0:3;
    uint_fast16_t C1:3;
    uint_fast16_t V0:1;
    uint_fast16_t V1:1;
    uint_fast16_t D0:1;
    uint_fast16_t D1:1;
    target_ulong PFN[2];
};


// RISCV CSR mappings. These are not the "real" mappings defined by the isa.
// Instead, they are the indices into our csr array (ie the output given when
// calling translate.c:csr_regno(REAL_CSR_REGNO)).
#define CSR_SUP0       0x0
#define CSR_SUP1       0x1
#define CSR_EPC        0x2
#define CSR_BADVADDR   0x3
#define CSR_PTBR       0x4
#define CSR_ASID       0x5
#define CSR_COUNT      0x6
#define CSR_COMPARE    0x7
#define CSR_EVEC       0x8
#define CSR_CAUSE      0x9
#define CSR_STATUS     0xa
#define CSR_HARTID     0xb
#define CSR_IMPL       0xc
#define CSR_FATC       0xd
#define CSR_SEND_IPI   0xe
#define CSR_CLEAR_IPI  0xf
#define CSR_CYCLE     0x10
#define CSR_TIME      0x11
#define CSR_INSTRET   0x12
#define CSR_FFLAGS    0x13
#define CSR_FRM       0x14
#define CSR_FCSR      0x15
//...
#define CSR_TOHOST    0x1e
#define CSR_FROMHOST  0x1f

// RISCV Exception Codes
#define RISCV_EXCP_INST_ADDR_MIS        0x0
#define RISCV_EXCP_INST_ACCESS_FAULT    0x1
#define RISCV_EXCP_ILLEGAL_INST         0x2
#define RISCV_EXCP_PRIV_INST            0x3
#define RISCV_EXCP_FP_DISABLED          0x4
#define RISCV_EXCP_SCALL                0x6
#define RISCV_EXCP_BREAK                0x7
#define RISCV_EXCP_LOAD_ADDR_MIS        0x8
#define RISCV_EXCP_STORE_ADDR_MIS       0x9
#define RISCV_EXCP_LOAD_ACCESS_FAULT    0xa
#define RISCV_EXCP_STORE_ACCESS_FAULT   0xb
#define RISCV_EXCP_STORE_ACCEL_DISABLED 0xc
#define RISCV_EXCP_TIMER_INTERRUPT      (0x7  | (1 << 31)) // TODO: ALSO NEEDS interruptBit
#define RISCV_EXCP_HOST_INTERRUPT       (0x6  | (1 << 31)) // TODO: ALSO NEEDS interruptBit
#define RISCV_EXCP_SERIAL_INTERRUPT     ((0x4) | (1 << 31))


// RISCV Status Reg Bits
#define SR_S           0x1
#define SR_PS          0x2
#define SR_EI          0x4
#define SR_PEI         0x8
#define SR_EF         0x10
#define SR_U64        0x20
#define SR_S64        0x40
#define SR_VM         0x80
#define SR_EA        0x100
#define SR_IM     0xFF0000
#define SR_IP   0xFF000000

// RISCV pte bits
#define PTE_V    0x1
#define PTE_T    0x2
#define PTE_G    0x4
#define PTE_UR   0x8
#define PTE_UW  0x10
#define PTE_UX  0x20
#define PTE_SR  0x40
#define PTE_SW  0x80
#define PTE_SX 0x100

#if !defined(CONFIG_USER_ONLY)
typedef struct CPUMIPSTLBContext CPUMIPSTLBContext;
struct CPUMIPSTLBContext {
    uint32_t nb_tlb;
    uint32_t tlb_in_use;
    int (*map_address) (struct CPUMIPSState *env, hwaddr *physical, int *prot, target_ulong address, int rw, int access_type);
    void (*helper_tlbwi)(struct CPUMIPSState *env);
    void (*helper_tlbwr)(struct CPUMIPSState *env);
    void (*helper_tlbp)(struct CPUMIPSState *env);
    void (*helper_tlbr)(struct CPUMIPSState *env);
    union {
        struct {
            r4k_tlb_t tlb[MIPS_TLB_MAX];
        } r4k;
    } mmu;
};
#endif

#define NB_MMU_MODES 3

typedef struct mips_def_t mips_def_t;

#define MIPS_SHADOW_SET_MAX 16
#define MIPS_TC_MAX 5
#define MIPS_FPU_MAX 1
#define MIPS_DSP_ACC 4

typedef struct TCState TCState;
struct TCState {
    target_ulong gpr[32];
    target_ulong fpr[32];
    target_ulong PC;
    target_ulong HI[MIPS_DSP_ACC];
    target_ulong LO[MIPS_DSP_ACC];
    target_ulong ACX[MIPS_DSP_ACC];
    target_ulong DSPControl;
    int32_t CP0_TCStatus;
#define CP0TCSt_TCU3	31
#define CP0TCSt_TCU2	30
#define CP0TCSt_TCU1	29
#define CP0TCSt_TCU0	28
#define CP0TCSt_TMX	27
#define CP0TCSt_RNST	23
#define CP0TCSt_TDS	21
#define CP0TCSt_DT	20
#define CP0TCSt_DA	15
#define CP0TCSt_A	13
#define CP0TCSt_TKSU	11
#define CP0TCSt_IXMT	10
#define CP0TCSt_TASID	0
    int32_t CP0_TCBind;
#define CP0TCBd_CurTC	21
#define CP0TCBd_TBE	17
#define CP0TCBd_CurVPE	0
    target_ulong CP0_TCHalt;
    target_ulong CP0_TCContext;
    target_ulong CP0_TCSchedule;
    target_ulong CP0_TCScheFBack;
    int32_t CP0_Debug_tcstatus;
};

typedef struct CPUMIPSState CPUMIPSState;
struct CPUMIPSState {
    TCState active_tc;

    uint32_t current_tc;

    uint32_t SEGBITS;
    uint32_t PABITS;
    target_ulong SEGMask;
    target_ulong PAMask;

    uint64_t helper_csr[32]; // RISCV CSR registers



    int32_t CP0_Index;
    /* CP0_MVP* are per MVP registers. */
    int32_t CP0_Random;
    int32_t CP0_VPEControl;
#define CP0VPECo_YSI	21
#define CP0VPECo_GSI	20
#define CP0VPECo_EXCPT	16
#define CP0VPECo_TE	15
#define CP0VPECo_TargTC	0
    int32_t CP0_VPEConf0;
#define CP0VPEC0_M	31
#define CP0VPEC0_XTC	21
#define CP0VPEC0_TCS	19
#define CP0VPEC0_SCS	18
#define CP0VPEC0_DSC	17
#define CP0VPEC0_ICS	16
#define CP0VPEC0_MVP	1
#define CP0VPEC0_VPA	0
    int32_t CP0_VPEConf1;
#define CP0VPEC1_NCX	20
#define CP0VPEC1_NCP2	10
#define CP0VPEC1_NCP1	0
    target_ulong CP0_YQMask;
    target_ulong CP0_VPESchedule;
    target_ulong CP0_VPEScheFBack;
    int32_t CP0_VPEOpt;
#define CP0VPEOpt_IWX7	15
#define CP0VPEOpt_IWX6	14
#define CP0VPEOpt_IWX5	13
#define CP0VPEOpt_IWX4	12
#define CP0VPEOpt_IWX3	11
#define CP0VPEOpt_IWX2	10
#define CP0VPEOpt_IWX1	9
#define CP0VPEOpt_IWX0	8
#define CP0VPEOpt_DWX7	7
#define CP0VPEOpt_DWX6	6
#define CP0VPEOpt_DWX5	5
#define CP0VPEOpt_DWX4	4
#define CP0VPEOpt_DWX3	3
#define CP0VPEOpt_DWX2	2
#define CP0VPEOpt_DWX1	1
#define CP0VPEOpt_DWX0	0
    target_ulong CP0_EntryLo0;
    target_ulong CP0_EntryLo1;
    target_ulong CP0_Context;
    int32_t CP0_PageMask;
    int32_t CP0_PageGrain;
    int32_t CP0_Wired;
    int32_t CP0_SRSConf0_rw_bitmask;
    int32_t CP0_SRSConf0;
#define CP0SRSC0_M	31
#define CP0SRSC0_SRS3	20
#define CP0SRSC0_SRS2	10
#define CP0SRSC0_SRS1	0
    int32_t CP0_SRSConf1_rw_bitmask;
    int32_t CP0_SRSConf1;
#define CP0SRSC1_M	31
#define CP0SRSC1_SRS6	20
#define CP0SRSC1_SRS5	10
#define CP0SRSC1_SRS4	0
    int32_t CP0_SRSConf2_rw_bitmask;
    int32_t CP0_SRSConf2;
#define CP0SRSC2_M	31
#define CP0SRSC2_SRS9	20
#define CP0SRSC2_SRS8	10
#define CP0SRSC2_SRS7	0
    int32_t CP0_SRSConf3_rw_bitmask;
    int32_t CP0_SRSConf3;
#define CP0SRSC3_M	31
#define CP0SRSC3_SRS12	20
#define CP0SRSC3_SRS11	10
#define CP0SRSC3_SRS10	0
    int32_t CP0_SRSConf4_rw_bitmask;
    int32_t CP0_SRSConf4;
#define CP0SRSC4_SRS15	20
#define CP0SRSC4_SRS14	10
#define CP0SRSC4_SRS13	0
    int32_t CP0_HWREna;
    target_ulong CP0_BadVAddr;
    int32_t CP0_Count;
    target_ulong CP0_EntryHi;
    int32_t CP0_Compare;
    int32_t CP0_Status;
#define CP0St_CU3   31
#define CP0St_CU2   30
#define CP0St_CU1   29
#define CP0St_CU0   28
#define CP0St_RP    27
#define CP0St_FR    26
#define CP0St_RE    25
#define CP0St_MX    24
#define CP0St_PX    23
#define CP0St_BEV   22
#define CP0St_TS    21
#define CP0St_SR    20
#define CP0St_NMI   19
#define CP0St_IM    8
#define CP0St_KX    7
#define CP0St_SX    6
#define CP0St_UX    5
#define CP0St_KSU   3
#define CP0St_ERL   2
#define CP0St_EXL   1
#define CP0St_IE    0
    int32_t CP0_IntCtl;
#define CP0IntCtl_IPTI 29
#define CP0IntCtl_IPPC1 26
#define CP0IntCtl_VS 5
    int32_t CP0_SRSCtl;
#define CP0SRSCtl_HSS 26
#define CP0SRSCtl_EICSS 18
#define CP0SRSCtl_ESS 12
#define CP0SRSCtl_PSS 6
#define CP0SRSCtl_CSS 0
    int32_t CP0_SRSMap;
#define CP0SRSMap_SSV7 28
#define CP0SRSMap_SSV6 24
#define CP0SRSMap_SSV5 20
#define CP0SRSMap_SSV4 16
#define CP0SRSMap_SSV3 12
#define CP0SRSMap_SSV2 8
#define CP0SRSMap_SSV1 4
#define CP0SRSMap_SSV0 0
    int32_t CP0_Cause;
#define CP0Ca_BD   31
#define CP0Ca_TI   30
#define CP0Ca_CE   28
#define CP0Ca_DC   27
#define CP0Ca_PCI  26
#define CP0Ca_IV   23
#define CP0Ca_WP   22
#define CP0Ca_IP    8
#define CP0Ca_IP_mask 0x0000FF00
#define CP0Ca_EC    2
    target_ulong CP0_EPC;
    int32_t CP0_PRid;
    int32_t CP0_EBase;
    int32_t CP0_Config0;
#define CP0C0_M    31
#define CP0C0_K23  28
#define CP0C0_KU   25
#define CP0C0_MDU  20
#define CP0C0_MM   17
#define CP0C0_BM   16
#define CP0C0_BE   15
#define CP0C0_AT   13
#define CP0C0_AR   10
#define CP0C0_MT   7
#define CP0C0_VI   3
#define CP0C0_K0   0
    int32_t CP0_Config1;
#define CP0C1_M    31
#define CP0C1_MMU  25
#define CP0C1_IS   22
#define CP0C1_IL   19
#define CP0C1_IA   16
#define CP0C1_DS   13
#define CP0C1_DL   10
#define CP0C1_DA   7
#define CP0C1_C2   6
#define CP0C1_MD   5
#define CP0C1_PC   4
#define CP0C1_WR   3
#define CP0C1_CA   2
#define CP0C1_EP   1
#define CP0C1_FP   0
    int32_t CP0_Config2;
#define CP0C2_M    31
#define CP0C2_TU   28
#define CP0C2_TS   24
#define CP0C2_TL   20
#define CP0C2_TA   16
#define CP0C2_SU   12
#define CP0C2_SS   8
#define CP0C2_SL   4
#define CP0C2_SA   0
    int32_t CP0_Config3;
#define CP0C3_M    31
#define CP0C3_ISA_ON_EXC 16
#define CP0C3_DSPP 10
#define CP0C3_LPA  7
#define CP0C3_VEIC 6
#define CP0C3_VInt 5
#define CP0C3_SP   4
#define CP0C3_MT   2
#define CP0C3_SM   1
#define CP0C3_TL   0
    uint32_t CP0_Config4;
    uint32_t CP0_Config4_rw_bitmask;
#define CP0C4_M    31
    uint32_t CP0_Config5;
    uint32_t CP0_Config5_rw_bitmask;
#define CP0C5_M          31
#define CP0C5_K          30
#define CP0C5_CV         29
#define CP0C5_EVA        28
#define CP0C5_MSAEn      27
#define CP0C5_UFR        2
#define CP0C5_NFExists   0
    int32_t CP0_Config6;
    int32_t CP0_Config7;
    /* XXX: Maybe make LLAddr per-TC? */
    target_ulong lladdr;
    target_ulong llval;
    target_ulong llnewval;
    target_ulong llreg;
    target_ulong CP0_LLAddr_rw_bitmask;
    int CP0_LLAddr_shift;
    target_ulong CP0_WatchLo[8];
    int32_t CP0_WatchHi[8];
    target_ulong CP0_XContext;
    int32_t CP0_Framemask;
    int32_t CP0_Debug;
#define CP0DB_DBD  31
#define CP0DB_DM   30
#define CP0DB_LSNM 28
#define CP0DB_Doze 27
#define CP0DB_Halt 26
#define CP0DB_CNT  25
#define CP0DB_IBEP 24
#define CP0DB_DBEP 21
#define CP0DB_IEXI 20
#define CP0DB_VER  15
#define CP0DB_DEC  10
#define CP0DB_SSt  8
#define CP0DB_DINT 5
#define CP0DB_DIB  4
#define CP0DB_DDBS 3
#define CP0DB_DDBL 2
#define CP0DB_DBp  1
#define CP0DB_DSS  0
    target_ulong CP0_DEPC;
    int32_t CP0_Performance0;
    int32_t CP0_TagLo;
    int32_t CP0_DataLo;
    int32_t CP0_TagHi;
    int32_t CP0_DataHi;
    target_ulong CP0_ErrorEPC;
    int32_t CP0_DESAVE;
    /* We waste some space so we can handle shadow registers like TCs. */
    TCState tcs[MIPS_SHADOW_SET_MAX];
    /* QEMU */
    int error_code;
    target_ulong btarget;        /* Jump / branch target               */
    target_ulong bcond;          /* Branch condition (if needed)       */

    int SYNCI_Step; /* Address step size for SYNCI */
    int CCRes; /* Cycle count resolution/divisor */
    uint32_t CP0_Status_rw_bitmask; /* Read/write bits in CP0_Status */
    uint32_t CP0_TCStatus_rw_bitmask; /* Read/write bits in CP0_TCStatus */
    int insn_flags; /* Supported instruction set */

    target_ulong tls_value; /* For usermode emulation */

    CPU_COMMON

    /* Fields from here on are preserved across CPU reset. */
#if !defined(CONFIG_USER_ONLY)
    CPUMIPSTLBContext *tlb;
#endif

    const mips_def_t *cpu_model;
    void *irq[8];
    QEMUTimer *timer; /* Internal timer */
};

#include "cpu-qom.h"

#if !defined(CONFIG_USER_ONLY)
int r4k_map_address (CPUMIPSState *env, hwaddr *physical, int *prot,
                     target_ulong address, int rw, int access_type);
void r4k_helper_tlbwi(CPUMIPSState *env);
void r4k_helper_tlbwr(CPUMIPSState *env);
void r4k_helper_tlbp(CPUMIPSState *env);
void r4k_helper_tlbr(CPUMIPSState *env);

void mips_cpu_unassigned_access(CPUState *cpu, hwaddr addr,
                                bool is_write, bool is_exec, int unused,
                                unsigned size);
#endif

void mips_cpu_list (FILE *f, fprintf_function cpu_fprintf);

#define cpu_exec cpu_mips_exec
#define cpu_gen_code cpu_mips_gen_code
#define cpu_signal_handler cpu_mips_signal_handler
#define cpu_list mips_cpu_list

extern void cpu_wrdsp(uint32_t rs, uint32_t mask_num, CPUMIPSState *env);
extern uint32_t cpu_rddsp(uint32_t mask_num, CPUMIPSState *env);

#define CPU_SAVE_VERSION 3

/* MMU modes definitions. We carefully match the indices with our
   hflags layout. */
#define MMU_MODE0_SUFFIX _kernel
#define MMU_MODE1_SUFFIX _super
#define MMU_MODE2_SUFFIX _user
#define MMU_USER_IDX 2
static inline int cpu_mmu_index (CPUMIPSState *env)
{
//    return env->hflags & MIPS_HFLAG_KSU;
    return 0;
}

static inline int cpu_mips_hw_interrupts_pending(CPUMIPSState *env)
{
    int32_t pending;
    int32_t status;
    int r;

    /* first check if interrupts are disabled */
    if (!((env->helper_csr[CSR_STATUS] >> 2) & 0x1)) {
        // interrupts disabled
        return 0;
    }

    pending = (env->helper_csr[CSR_STATUS] >> 24) & 0xFF;
    status = (env->helper_csr[CSR_STATUS] >> 16) & 0xFF;

    // TODO handle priority here?

    r = pending & status;
    return r;
}

#include "exec/cpu-all.h"

/* Memory access type :
 * may be needed for precise access rights control and precise exceptions.
 */
enum {
    /* 1 bit to define user level / supervisor access */
    ACCESS_USER  = 0x00,
    ACCESS_SUPER = 0x01,
    /* 1 bit to indicate direction */
    ACCESS_STORE = 0x02,
    /* Type of instruction that generated the access */
    ACCESS_CODE  = 0x10, /* Code fetch access                */
    ACCESS_INT   = 0x20, /* Integer load/store access        */
    ACCESS_FLOAT = 0x30, /* floating point load/store access */
};

/* Exceptions */
enum {
    EXCP_NONE          = -1,
    EXCP_RESET         = 0,
    EXCP_SRESET,
    EXCP_DSS,
    EXCP_DINT,
    EXCP_DDBL,
    EXCP_DDBS,
    EXCP_NMI,
    EXCP_MCHECK,
    EXCP_EXT_INTERRUPT, /* 8 */
    EXCP_DFWATCH,
    EXCP_DIB,
    EXCP_IWATCH,
    EXCP_AdEL,
    EXCP_AdES,
    EXCP_TLBF,
    EXCP_IBE,
    EXCP_DBp, /* 16 */
    EXCP_SYSCALL,
    EXCP_BREAK,
    EXCP_CpU,
    EXCP_RI,
    EXCP_OVERFLOW,
    EXCP_TRAP,
    EXCP_FPE,
    EXCP_DWATCH, /* 24 */
    EXCP_LTLBL,
    EXCP_TLBL,
    EXCP_TLBS,
    EXCP_DBE,
    EXCP_THREAD,
    EXCP_MDMX,
    EXCP_C2E,
    EXCP_CACHE, /* 32 */
    EXCP_DSPDIS,

    EXCP_LAST = EXCP_DSPDIS,
};
/* Dummy exception for conditional stores.  */
#define EXCP_SC 0x100

/*
 * This is an interrnally generated WAKE request line.
 * It is driven by the CPU itself. Raised when the MT
 * block wants to wake a VPE from an inactive state and
 * cleared when VPE goes from active to inactive.
 */
#define CPU_INTERRUPT_WAKE CPU_INTERRUPT_TGT_INT_0

int cpu_mips_exec(CPUMIPSState *s);
void mips_tcg_init(void);
MIPSCPU *cpu_mips_init(const char *cpu_model);
int cpu_mips_signal_handler(int host_signum, void *pinfo, void *puc);

static inline CPUMIPSState *cpu_init(const char *cpu_model)
{
    MIPSCPU *cpu = cpu_mips_init(cpu_model);
    if (cpu == NULL) {
        return NULL;
    }
    return &cpu->env;
}

/* TODO QOM'ify CPU reset and remove */
void cpu_state_reset(CPUMIPSState *s);

/* mips_timer.c */
uint32_t cpu_mips_get_random (CPUMIPSState *env);
uint32_t cpu_mips_get_count (CPUMIPSState *env);
void cpu_mips_store_count (CPUMIPSState *env, uint32_t value);
void cpu_mips_store_compare (CPUMIPSState *env, uint32_t value);
void cpu_mips_start_count(CPUMIPSState *env);
void cpu_mips_stop_count(CPUMIPSState *env);

/* mips_int.c */
void cpu_mips_soft_irq(CPUMIPSState *env, int irq, int level);

/* helper.c */
int mips_cpu_handle_mmu_fault(CPUState *cpu, vaddr address, int rw,
                              int mmu_idx);
#if !defined(CONFIG_USER_ONLY)
void r4k_invalidate_tlb (CPUMIPSState *env, int idx, int use_extra);
hwaddr cpu_mips_translate_address (CPUMIPSState *env, target_ulong address,
		                               int rw);
#endif
target_ulong exception_resume_pc (CPUMIPSState *env);

static inline void cpu_get_tb_cpu_state(CPUMIPSState *env, target_ulong *pc,
                                        target_ulong *cs_base, int *flags)
{
    *pc = env->active_tc.PC;
    *cs_base = 0;
}

#include "exec/exec-all.h"

#endif /* !defined (__MIPS_CPU_H__) */
