/*
 *  MIPS emulation for qemu: CPU initialisation routines.
 *
 *  Copyright (c) 2004-2005 Jocelyn Mayer
 *  Copyright (c) 2007 Herve Poussineau
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

/* CPU / CPU family specific config register values. */

/* Have config1, uncached coherency */
#define MIPS_CONFIG0                                              \
  ((1U << CP0C0_M) | (0x2 << CP0C0_K0))

/* Have config2, no coprocessor2 attached, no MDMX support attached,
   no performance counters, watch registers present,
   no code compression, EJTAG present, no FPU */
#define MIPS_CONFIG1                                              \
((1U << CP0C1_M) |                                                \
 (0 << CP0C1_C2) | (0 << CP0C1_MD) | (0 << CP0C1_PC) |            \
 (1 << CP0C1_WR) | (0 << CP0C1_CA) | (1 << CP0C1_EP) |            \
 (0 << CP0C1_FP))

/* Have config3, no tertiary/secondary caches implemented */
#define MIPS_CONFIG2                                              \
((1U << CP0C2_M))

/* No config4, no DSP ASE, no large physaddr (PABITS),
   no external interrupt controller, no vectored interrupts,
   no 1kb pages, no SmartMIPS ASE, no trace logic */
#define MIPS_CONFIG3                                              \
((0 << CP0C3_M) | (0 << CP0C3_DSPP) | (0 << CP0C3_LPA) |          \
 (0 << CP0C3_VEIC) | (0 << CP0C3_VInt) | (0 << CP0C3_SP) |        \
 (0 << CP0C3_SM) | (0 << CP0C3_TL))

#define MIPS_CONFIG4                                              \
((0 << CP0C4_M))

#define MIPS_CONFIG5                                              \
((0 << CP0C5_M))

/* MMU types, the first four entries have the same layout as the
   CP0C0_MT field.  */
enum mips_mmu_types {
    MMU_TYPE_NONE,
    MMU_TYPE_R4000,
    MMU_TYPE_RESERVED,
    MMU_TYPE_FMT,
    MMU_TYPE_R3000,
    MMU_TYPE_R6000,
    MMU_TYPE_R8000
};

struct mips_def_t {
    const char *name;
    int32_t CP0_PRid;
    int32_t CP0_Config0;
    int32_t CP0_Config1;
    int32_t CP0_Config2;
    int32_t CP0_Config3;
    int32_t CP0_Config4;
    int32_t CP0_Config4_rw_bitmask;
    int32_t CP0_Config5;
    int32_t CP0_Config5_rw_bitmask;
    int32_t CP0_Config6;
    int32_t CP0_Config7;
    target_ulong CP0_LLAddr_rw_bitmask;
    int CP0_LLAddr_shift;
    int32_t SYNCI_Step;
    int32_t CCRes;
    int32_t CP0_Status_rw_bitmask;
    int32_t CP0_TCStatus_rw_bitmask;
    int32_t CP0_SRSCtl;
    int32_t CP1_fcr0;
    int32_t SEGBITS;
    int32_t PABITS;
    int32_t CP0_SRSConf0_rw_bitmask;
    int32_t CP0_SRSConf0;
    int32_t CP0_SRSConf1_rw_bitmask;
    int32_t CP0_SRSConf1;
    int32_t CP0_SRSConf2_rw_bitmask;
    int32_t CP0_SRSConf2;
    int32_t CP0_SRSConf3_rw_bitmask;
    int32_t CP0_SRSConf3;
    int32_t CP0_SRSConf4_rw_bitmask;
    int32_t CP0_SRSConf4;
    int insn_flags;
    enum mips_mmu_types mmu_type;
};

/*****************************************************************************/
/* MIPS CPU definitions */
static const mips_def_t mips_defs[] =
{
    {   ///////////////////// TODO: THIS ONE
        .name = "20Kc",
        .CP0_PRid = 0x000182a0,
        .CP0_Config0 = MIPS_CONFIG0 | (0x2 << CP0C0_AT) |
                    (MMU_TYPE_R4000 << CP0C0_MT) | (1 << CP0C0_VI),
        .CP0_Config1 = MIPS_CONFIG1 | (1 << CP0C1_FP) | (47 << CP0C1_MMU) |
                       (2 << CP0C1_IS) | (4 << CP0C1_IL) | (3 << CP0C1_IA) |
                       (2 << CP0C1_DS) | (4 << CP0C1_DL) | (3 << CP0C1_DA) |
                       (1 << CP0C1_PC) | (1 << CP0C1_WR) | (1 << CP0C1_EP),
        .CP0_Config2 = MIPS_CONFIG2,
        .CP0_Config3 = MIPS_CONFIG3,
        .CP0_LLAddr_rw_bitmask = 0,
        .CP0_LLAddr_shift = 0,
        .SYNCI_Step = 32,
        .CCRes = 1,
        .CP0_Status_rw_bitmask = 0x36FBFFFF,
        /* The 20Kc has F64 / L / W but doesn't use the fcr0 bits. */
        .SEGBITS = 40,
        .PABITS = 36,
//        .insn_flags = CPU_MIPS64 | ASE_MIPS3D,
        .mmu_type = MMU_TYPE_R4000,
    },
};

static const mips_def_t *cpu_mips_find_by_name (const char *name)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(mips_defs); i++) {
        if (strcasecmp(name, mips_defs[i].name) == 0) {
            return &mips_defs[i];
        }
    }
    return NULL;
}

void mips_cpu_list (FILE *f, fprintf_function cpu_fprintf)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(mips_defs); i++) {
        (*cpu_fprintf)(f, "MIPS '%s'\n",
                       mips_defs[i].name);
    }
}

#ifndef CONFIG_USER_ONLY
static void r4k_mmu_init (CPUMIPSState *env, const mips_def_t *def)
{
    env->tlb->nb_tlb = 1 + ((def->CP0_Config1 >> CP0C1_MMU) & 63);
    env->tlb->map_address = &r4k_map_address;
    env->tlb->helper_tlbwi = r4k_helper_tlbwi;
    env->tlb->helper_tlbwr = r4k_helper_tlbwr;
    env->tlb->helper_tlbp = r4k_helper_tlbp;
    env->tlb->helper_tlbr = r4k_helper_tlbr;
}

static void mmu_init (CPUMIPSState *env, const mips_def_t *def)
{
    MIPSCPU *cpu = mips_env_get_cpu(env);

    env->tlb = g_malloc0(sizeof(CPUMIPSTLBContext));

    switch (def->mmu_type) {
        case MMU_TYPE_R4000:
            r4k_mmu_init(env, def);
            break;
        default:
            cpu_abort(CPU(cpu), "MMU type not supported\n");
    }
}
#endif /* CONFIG_USER_ONLY */


