/*
 * QEMU RISC-V CPU
 *
 * Author: Sagar Karandikar, sagark@eecs.berkeley.edu
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/lgpl-2.1.html>
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "cpu.h"
#include "qemu-common.h"
#include "migration/vmstate.h"

/* RISC-V CPU definitions */

typedef struct RISCVCPUInfo {
    const char *name;
    void (*initfn)(Object *obj);
} RISCVCPUInfo;

static void riscv_imafdcsu_priv1_9_cpu_init(Object *obj)
{
    CPURISCVState *env = &RISCV_CPU(obj)->env;
    env->misa = env->misa_mask = RVXLEN|RVI|RVM|RVA|RVF|RVD|RVC|RVS|RVU;
    env->user_ver = USER_VERSION_2_02_0;
    env->priv_ver = PRIV_VERSION_1_09_1;
}

static void riscv_imafdcsu_priv1_10_cpu_init(Object *obj)
{
    CPURISCVState *env = &RISCV_CPU(obj)->env;
    env->misa = env->misa_mask = RVXLEN|RVI|RVM|RVA|RVF|RVD|RVC|RVS|RVU;
    env->user_ver = USER_VERSION_2_02_0;
    env->priv_ver = PRIV_VERSION_1_10_0;
}

static void riscv_imacu_priv1_10_cpu_init(Object *obj)
{
    CPURISCVState *env = &RISCV_CPU(obj)->env;
    env->misa = env->misa_mask = RVXLEN|RVI|RVM|RVA|RVC|RVU;
    env->user_ver = USER_VERSION_2_02_0;
    env->priv_ver = PRIV_VERSION_1_10_0;
}

static void riscv_imac_priv1_10_cpu_init(Object *obj)
{
    CPURISCVState *env = &RISCV_CPU(obj)->env;
    env->misa = env->misa_mask = RVXLEN|RVI|RVM|RVA|RVC;
    env->user_ver = USER_VERSION_2_02_0;
    env->priv_ver = PRIV_VERSION_1_10_0;
}

static const RISCVCPUInfo riscv_cpus[] = {
    { TYPE_RISCV_CPU_IMAFDCSU_PRIV_1_09, riscv_imafdcsu_priv1_9_cpu_init },
    { TYPE_RISCV_CPU_IMAFDCSU_PRIV_1_10, riscv_imafdcsu_priv1_10_cpu_init },
    { TYPE_RISCV_CPU_IMACU_PRIV_1_10,    riscv_imacu_priv1_10_cpu_init },
    { TYPE_RISCV_CPU_IMAC_PRIV_1_10,     riscv_imac_priv1_10_cpu_init },
    { NULL, NULL }
};

static inline void set_feature(CPURISCVState *env, int feature)
{
    env->features |= 1ULL << feature;
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

#ifdef CONFIG_USER_ONLY
static bool riscv_cpu_has_work(CPUState *cs)
{
    return 0;
}
#else
static bool riscv_cpu_has_work(CPUState *cs)
{
    return (cs->interrupt_request & CPU_INTERRUPT_HARD);
}
#endif

void restore_state_to_opc(CPURISCVState *env, TranslationBlock *tb,
                          target_ulong *data)
{
    env->pc = data[0];
}

static void riscv_cpu_reset(CPUState *s)
{
    RISCVCPU *cpu = RISCV_CPU(s);
    RISCVCPUClass *mcc = RISCV_CPU_GET_CLASS(cpu);
    CPURISCVState *env = &cpu->env;
    CPUState *cs = CPU(cpu);

    mcc->parent_reset(s);
#ifndef CONFIG_USER_ONLY
    tlb_flush(s, 1);
    env->priv = PRV_M;
    env->mtvec = DEFAULT_MTVEC;
#endif
    env->pc = DEFAULT_RSTVEC;
    cs->exception_index = EXCP_NONE;
}

static void riscv_cpu_disas_set_info(CPUState *s, disassemble_info *info) {
    info->print_insn = print_insn_riscv;
}

static void riscv_cpu_realize(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    RISCVCPU *cpu = RISCV_CPU(dev);
    RISCVCPUClass *mcc = RISCV_CPU_GET_CLASS(dev);
    CPURISCVState *env = &cpu->env;

    if (env->misa & RVM) set_feature(env, RISCV_FEATURE_RVM);
    if (env->misa & RVA) set_feature(env, RISCV_FEATURE_RVA);
    if (env->misa & RVF) set_feature(env, RISCV_FEATURE_RVF);
    if (env->misa & RVD) set_feature(env, RISCV_FEATURE_RVD);
    if (env->misa & RVC) set_feature(env, RISCV_FEATURE_RVC);

    cpu_reset(cs);
    qemu_init_vcpu(cs);
    set_default_nan_mode(1, &env->fp_status);

    mcc->parent_realize(dev, errp);
}

static void riscv_cpu_init(Object *obj)
{
    CPUState *cs = CPU(obj);
    RISCVCPU *cpu = RISCV_CPU(obj);
    CPURISCVState *env = &cpu->env;

    cs->env_ptr = env;
    cpu_exec_init(cs, &error_abort);

    if (tcg_enabled()) {
        riscv_tcg_init();
    }
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
    cc->do_unassigned_access = riscv_cpu_unassigned_access;
    cc->do_unaligned_access = riscv_cpu_do_unaligned_access;
    cc->get_phys_page_debug = riscv_cpu_get_phys_page_debug;
#endif
    /* For now, mark unmigratable: */
    cc->vmsd = &vmstate_riscv_cpu;

    /*
     * Reason: riscv_cpu_init() calls cpu_exec_init(), which saves
     * the object in cpus -> dangling pointer after final
     * object_unref().
     */
    dc->cannot_destroy_with_object_finalize_yet = true;
}

static void cpu_register(const RISCVCPUInfo *info)
{
    TypeInfo type_info = {
        .name = g_strdup(info->name),
        .parent = TYPE_RISCV_CPU,
        .instance_size = sizeof(RISCVCPU),
        .instance_init = info->initfn,
    };

    type_register(&type_info);
    g_free((void *)type_info.name);
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

char* riscv_isa_string(RISCVCPU *cpu)
{
    size_t len = 5 + ctz32(cpu->env.misa);
    char *isa_string = g_new(char, len);
    isa_string[0] = '\0';
#if defined(TARGET_RISCV32)
    strncat(isa_string, "rv32", len);
#elif defined(TARGET_RISCV64)
    strncat(isa_string, "rv64", len);
#endif
    if (cpu->env.misa & RVI) strncat(isa_string, "i", len);
    if (cpu->env.misa & RVM) strncat(isa_string, "m", len);
    if (cpu->env.misa & RVA) strncat(isa_string, "a", len);
    if (cpu->env.misa & RVF) strncat(isa_string, "f", len);
    if (cpu->env.misa & RVD) strncat(isa_string, "d", len);
    if (cpu->env.misa & RVC) strncat(isa_string, "c", len);
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

RISCVCPU *cpu_riscv_init(const char *cpu_model)
{
    RISCVCPU *cpu = RISCV_CPU(object_new(TYPE_RISCV_CPU));
    CPURISCVState *env = &cpu->env;

    env->misa = env->misa_mask = RVXLEN|RVI|RVM|RVA|RVF|RVD|RVC;
    env->user_ver = USER_VERSION_2_02_0;
    env->priv_ver = PRIV_VERSION_1_09_1;

    object_property_set_bool(OBJECT(cpu), true, "realized", NULL);

    return cpu;
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
