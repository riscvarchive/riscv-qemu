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

static void riscv_cpu_set_pc(CPUState *cs, vaddr value)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;
    env->PC = value;
}

static void riscv_cpu_synchronize_from_tb(CPUState *cs, TranslationBlock *tb)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;
    env->PC = tb->pc;
}

static bool riscv_cpu_has_work(CPUState *cs)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;
    bool has_work = false;

    if (cs->interrupt_request & CPU_INTERRUPT_HARD) {
        int interruptno = cpu_riscv_hw_interrupts_pending(env);
        if (interruptno + 1) {
            has_work = true;
        }
    }

    return has_work;
}

static void riscv_cpu_reset(CPUState *s)
{
    RISCVCPU *cpu = RISCV_CPU(s);
    RISCVCPUClass *mcc = RISCV_CPU_GET_CLASS(cpu);
    CPURISCVState *env = &cpu->env;
    CPUState *cs = CPU(cpu);

    mcc->parent_reset(s);
    tlb_flush(s, 1);

    env->priv = PRV_M;
    env->PC = DEFAULT_RSTVEC;
    env->csr[CSR_MTVEC] = DEFAULT_MTVEC;
    cs->exception_index = EXCP_NONE;
}

static void riscv_cpu_realizefn(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    RISCVCPUClass *mcc = RISCV_CPU_GET_CLASS(dev);

    cpu_reset(cs);
    qemu_init_vcpu(cs);

    mcc->parent_realize(dev, errp);
}

static void riscv_cpu_initfn(Object *obj)
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
    dc->realize = riscv_cpu_realizefn;

    mcc->parent_reset = cc->reset;
    cc->reset = riscv_cpu_reset;

    cc->has_work = riscv_cpu_has_work;
    cc->do_interrupt = riscv_cpu_do_interrupt;
    cc->cpu_exec_interrupt = riscv_cpu_exec_interrupt;
    cc->dump_state = riscv_cpu_dump_state;
    cc->set_pc = riscv_cpu_set_pc;
    cc->synchronize_from_tb = riscv_cpu_synchronize_from_tb;
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
     * Reason: riscv_cpu_initfn() calls cpu_exec_init(), which saves
     * the object in cpus -> dangling pointer after final
     * object_unref().
     */
    dc->cannot_destroy_with_object_finalize_yet = true;
}

static const TypeInfo riscv_cpu_type_info = {
    .name = TYPE_RISCV_CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(RISCVCPU),
    .instance_init = riscv_cpu_initfn,
    .abstract = false,
    .class_size = sizeof(RISCVCPUClass),
    .class_init = riscv_cpu_class_init,
};

static void riscv_cpu_register_types(void)
{
    type_register_static(&riscv_cpu_type_info);
}

type_init(riscv_cpu_register_types)
