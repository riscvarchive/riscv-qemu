/*
 * QEMU RISC-V - QEMU IRQ Support
 *
 * Author: Sagar Karandikar, sagark@eecs.berkeley.edu
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "hw/hw.h"
#include "hw/riscv/cpudevs.h"
#include "cpu.h"

static void cpu_riscv_irq_request(void *opaque, int irq, int level)
{
    /* These are not the same irq numbers visible to the emulated processor. */
    RISCVCPU *cpu = opaque;
    CPURISCVState *env = &cpu->env;
    CPUState *cs = CPU(cpu);

    /* current irqs:
       4: Host Interrupt. mfromhost should have a nonzero value
       3: Machine Timer. MIP_MTIP should have already been set
       2, 1, 0: Interrupts triggered by the CPU. At least one of
       MIP_STIP, MIP_SSIP, MIP_MSIP should already be set */
    if (unlikely(!(irq < 5 && irq >= 0))) {
        printf("IRQNO: %d\n", irq);
        fprintf(stderr, "Unused IRQ was raised.\n");
        exit(1);
    }

    if (level) {
        cpu_interrupt(cs, CPU_INTERRUPT_HARD);
    } else {
        if (!env->mip && !env->mfromhost) {
            /* no interrupts pending, no host interrupt for HTIF, reset */
            cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
        }
    }
}

void cpu_riscv_irq_init_cpu(CPURISCVState *env)
{
    qemu_irq *qi;
    int i;

    qi = qemu_allocate_irqs(cpu_riscv_irq_request, riscv_env_get_cpu(env), 8);
    for (i = 0; i < 8; i++) {
        env->irq[i] = qi[i];
    }
}
