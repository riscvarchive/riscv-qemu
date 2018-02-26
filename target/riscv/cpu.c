/*
 * QEMU RISC-V CPU
 *
 * Copyright (c) 2016-2017 Sagar Karandikar, sagark@eecs.berkeley.edu
 * Copyright (c) 2017-2018 SiFive, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "cpu.h"
#include "exec/exec-all.h"
#include "qapi/error.h"
#include "migration/vmstate.h"

/* RISC-V CPU definitions */

const char * const riscv_int_regnames[] = {
  "zero", "ra  ", "sp  ", "gp  ", "tp  ", "t0  ", "t1  ", "t2  ",
  "s0  ", "s1  ", "a0  ", "a1  ", "a2  ", "a3  ", "a4  ", "a5  ",
  "a6  ", "a7  ", "s2  ", "s3  ", "s4  ", "s5  ", "s6  ", "s7  ",
  "s8  ", "s9  ", "s10 ", "s11 ", "t3  ", "t4  ", "t5  ", "t6  "
};

const char * const riscv_fpr_regnames[] = {
  "ft0 ", "ft1 ", "ft2 ", "ft3 ", "ft4 ", "ft5 ", "ft6 ",  "ft7 ",
  "fs0 ", "fs1 ", "fa0 ", "fa1 ", "fa2 ", "fa3 ", "fa4 ",  "fa5 ",
  "fa6 ", "fa7 ", "fs2 ", "fs3 ", "fs4 ", "fs5 ", "fs6 ",  "fs7 ",
  "fs8 ", "fs9 ", "fs10", "fs11", "ft8 ", "ft9 ", "ft10",  "ft11"
};

const char * const riscv_excp_names[] = {
    "misaligned_fetch",
    "fault_fetch",
    "illegal_instruction",
    "breakpoint",
    "misaligned_load",
    "fault_load",
    "misaligned_store",
    "fault_store",
    "user_ecall",
    "supervisor_ecall",
    "hypervisor_ecall",
    "machine_ecall",
    "exec_page_fault",
    "load_page_fault",
    "reserved",
    "store_page_fault"
};

const char * const riscv_intr_names[] = {
    "u_software",
    "s_software",
    "h_software",
    "m_software",
    "u_timer",
    "s_timer",
    "h_timer",
    "m_timer",
    "u_external",
    "s_external",
    "h_external",
    "m_external",
    "coprocessor",
    "host"
};

typedef struct RISCVCPUInfo {
    const char *name;
    void (*initfn)(Object *obj);
} RISCVCPUInfo;

static void riscv_any_cpu_init(Object *obj)
{
    CPURISCVState *env = &RISCV_CPU(obj)->env;
    env->misa = RVXLEN | RVI | RVM | RVA | RVF | RVD | RVC | RVU;
    env->user_ver = USER_VERSION_2_02_0;
    env->priv_ver = PRIV_VERSION_1_10_0;
}

static void riscv_imafdcsu_priv1_9_cpu_init(Object *obj)
{
    CPURISCVState *env = &RISCV_CPU(obj)->env;
    env->misa = RVXLEN | RVI | RVM | RVA | RVF | RVD | RVC | RVS | RVU;
    env->user_ver = USER_VERSION_2_02_0;
    env->priv_ver = PRIV_VERSION_1_09_1;
}

static void riscv_imafdcsu_priv1_10_cpu_init(Object *obj)
{
    CPURISCVState *env = &RISCV_CPU(obj)->env;
    env->misa = RVXLEN | RVI | RVM | RVA | RVF | RVD | RVC | RVS | RVU;
    env->user_ver = USER_VERSION_2_02_0;
    env->priv_ver = PRIV_VERSION_1_10_0;
}

static void riscv_imacu_priv1_10_cpu_init(Object *obj)
{
    CPURISCVState *env = &RISCV_CPU(obj)->env;
    env->misa = RVXLEN | RVI | RVM | RVA | RVC | RVU;
    env->user_ver = USER_VERSION_2_02_0;
    env->priv_ver = PRIV_VERSION_1_10_0;
}

static void riscv_imac_priv1_10_cpu_init(Object *obj)
{
    CPURISCVState *env = &RISCV_CPU(obj)->env;
    env->misa = RVXLEN | RVI | RVM | RVA | RVC;
    env->user_ver = USER_VERSION_2_02_0;
    env->priv_ver = PRIV_VERSION_1_10_0;
}

static const RISCVCPUInfo riscv_cpus[] = {
    { TYPE_RISCV_CPU_ANY,                riscv_any_cpu_init },
    { TYPE_RISCV_CPU_IMAFDCSU_PRIV_1_09, riscv_imafdcsu_priv1_9_cpu_init },
    { TYPE_RISCV_CPU_IMAFDCSU_PRIV_1_10, riscv_imafdcsu_priv1_10_cpu_init },
    { TYPE_RISCV_CPU_IMACU_PRIV_1_10,    riscv_imacu_priv1_10_cpu_init },
    { TYPE_RISCV_CPU_IMAC_PRIV_1_10,     riscv_imac_priv1_10_cpu_init },
    { NULL, NULL }
};

static ObjectClass *riscv_cpu_class_by_name(const char *cpu_model)
{
    ObjectClass *oc;
    char *typename;
    char **cpuname;

    cpuname = g_strsplit(cpu_model, ",", 1);
    typename = g_strdup_printf(RISCV_CPU_TYPE_NAME("%s"), cpuname[0]);
    oc = object_class_by_name(typename);
    g_strfreev(cpuname);
    g_free(typename);
    if (!oc || !object_class_dynamic_cast(oc, TYPE_RISCV_CPU) ||
        object_class_is_abstract(oc)) {
        return NULL;
    }
    return oc;
}

static void riscv_cpu_dump_state(CPUState *cs, FILE *f,
    fprintf_function cpu_fprintf, int flags)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;
    int i;

    cpu_fprintf(f, "pc=0x" TARGET_FMT_lx "\n", env->pc);
    for (i = 0; i < 32; i++) {
        cpu_fprintf(f, " %s " TARGET_FMT_lx,
            riscv_int_regnames[i], env->gpr[i]);
        if ((i & 3) == 3) {
            cpu_fprintf(f, "\n");
        }
    }

#ifndef CONFIG_USER_ONLY
    cpu_fprintf(f, " %s " TARGET_FMT_lx "\n", "MSTATUS ",
                env->mstatus);
    target_ulong mip = atomic_read(&env->mip);
    cpu_fprintf(f, " %s " TARGET_FMT_lx "\n", "MIP     ", mip);
    cpu_fprintf(f, " %s " TARGET_FMT_lx "\n", "MIE     ", env->mie);
#endif

    for (i = 0; i < 32; i++) {
        if ((i & 3) == 0) {
            cpu_fprintf(f, "FPR%02d:", i);
        }
        cpu_fprintf(f, " %s %016" PRIx64,
            riscv_fpr_regnames[i], env->fpr[i]);
        if ((i & 3) == 3) {
            cpu_fprintf(f, "\n");
        }
    }
}

static void riscv_cpu_set_pc(CPUState *cs, vaddr value)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;
    env->pc = value;
}

static void riscv_cpu_synchronize_from_tb(CPUState *cs, TranslationBlock *tb)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;
    env->pc = tb->pc;
}

static bool riscv_cpu_has_work(CPUState *cs)
{
#ifndef CONFIG_USER_ONLY
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;
    /*
     * Definition of the WFI instruction requires it to ignore the privilege
     * mode and delegation registers, but respect individual enables
     */
    return (atomic_read(&env->mip) & env->mie) != 0;
#else
    return true;
#endif
}

void restore_state_to_opc(CPURISCVState *env, TranslationBlock *tb,
                          target_ulong *data)
{
    env->pc = data[0];
}

static void riscv_cpu_reset(CPUState *cs)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    RISCVCPUClass *mcc = RISCV_CPU_GET_CLASS(cpu);
    CPURISCVState *env = &cpu->env;

    mcc->parent_reset(cs);
#ifndef CONFIG_USER_ONLY
    env->priv = PRV_M;
    env->mtvec = DEFAULT_MTVEC;
#endif
    env->pc = DEFAULT_RSTVEC;
    cs->exception_index = EXCP_NONE;
    set_default_nan_mode(1, &env->fp_status);
}

static void riscv_cpu_disas_set_info(CPUState *s, disassemble_info *info)
{
#if defined(TARGET_RISCV32)
    info->print_insn = print_insn_riscv32;
#elif defined(TARGET_RISCV64)
    info->print_insn = print_insn_riscv64;
#endif
}

static void riscv_cpu_realize(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    RISCVCPUClass *mcc = RISCV_CPU_GET_CLASS(dev);
    Error *local_err = NULL;

    cpu_exec_realizefn(cs, &local_err);
    if (local_err != NULL) {
        error_propagate(errp, local_err);
        return;
    }

    qemu_init_vcpu(cs);
    cpu_reset(cs);

    mcc->parent_realize(dev, errp);
}

static void riscv_cpu_init(Object *obj)
{
    CPUState *cs = CPU(obj);
    RISCVCPU *cpu = RISCV_CPU(obj);

    cs->env_ptr = &cpu->env;
}

static const VMStateDescription vmstate_riscv_cpu = {
    .name = "cpu",
    .unmigratable = 1,
};

static void riscv_cpu_class_init(ObjectClass *c, void *data)
{
    RISCVCPUClass *mcc = RISCV_CPU_CLASS(c);
    CPUClass *cc = CPU_CLASS(c);
    DeviceClass *dc = DEVICE_CLASS(c);

    mcc->parent_realize = dc->realize;
    dc->realize = riscv_cpu_realize;

    mcc->parent_reset = cc->reset;
    cc->reset = riscv_cpu_reset;

    cc->class_by_name = riscv_cpu_class_by_name;
    cc->has_work = riscv_cpu_has_work;
    cc->do_interrupt = riscv_cpu_do_interrupt;
    cc->cpu_exec_interrupt = riscv_cpu_exec_interrupt;
    cc->dump_state = riscv_cpu_dump_state;
    cc->set_pc = riscv_cpu_set_pc;
    cc->synchronize_from_tb = riscv_cpu_synchronize_from_tb;
    cc->gdb_read_register = riscv_cpu_gdb_read_register;
    cc->gdb_write_register = riscv_cpu_gdb_write_register;
    cc->gdb_num_core_regs = 65;
    cc->gdb_stop_before_watchpoint = true;
    cc->disas_set_info = riscv_cpu_disas_set_info;
#ifdef CONFIG_USER_ONLY
    cc->handle_mmu_fault = riscv_cpu_handle_mmu_fault;
#else
    cc->do_unaligned_access = riscv_cpu_do_unaligned_access;
    cc->get_phys_page_debug = riscv_cpu_get_phys_page_debug;
#endif
#ifdef CONFIG_TCG
    cc->tcg_initialize = riscv_translate_init;
#endif
    /* For now, mark unmigratable: */
    cc->vmsd = &vmstate_riscv_cpu;
}

static void cpu_register(const RISCVCPUInfo *info)
{
    TypeInfo type_info = {
        .name = info->name,
        .parent = TYPE_RISCV_CPU,
        .instance_size = sizeof(RISCVCPU),
        .instance_init = info->initfn,
    };

    type_register(&type_info);
}

static const TypeInfo riscv_cpu_type_info = {
    .name = TYPE_RISCV_CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(RISCVCPU),
    .instance_init = riscv_cpu_init,
    .abstract = false,
    .class_size = sizeof(RISCVCPUClass),
    .class_init = riscv_cpu_class_init,
};

char *riscv_isa_string(RISCVCPU *cpu)
{
    size_t len = 5 + ctz32(cpu->env.misa);
    char *isa_string = g_new(char, len);
    isa_string[0] = '\0';
#if defined(TARGET_RISCV32)
    strncat(isa_string, "rv32", len);
#elif defined(TARGET_RISCV64)
    strncat(isa_string, "rv64", len);
#endif
    if (cpu->env.misa & RVI) {
        strncat(isa_string, "i", len);
    }
    if (cpu->env.misa & RVM) {
        strncat(isa_string, "m", len);
    }
    if (cpu->env.misa & RVA) {
        strncat(isa_string, "a", len);
    }
    if (cpu->env.misa & RVF) {
        strncat(isa_string, "f", len);
    }
    if (cpu->env.misa & RVD) {
        strncat(isa_string, "d", len);
    }
    if (cpu->env.misa & RVC) {
        strncat(isa_string, "c", len);
    }
    return isa_string;
}

void riscv_cpu_list(FILE *f, fprintf_function cpu_fprintf)
{
    const RISCVCPUInfo *info = riscv_cpus;

    while (info->name) {
        (*cpu_fprintf)(f, "%s\n", info->name);
        info++;
    }
}

static void riscv_cpu_register_types(void)
{
    const RISCVCPUInfo *info = riscv_cpus;

    type_register_static(&riscv_cpu_type_info);

    while (info->name) {
        cpu_register(info);
        info++;
    }
}

type_init(riscv_cpu_register_types)
