/*
 *  QEMU RISC-V interrupt support
 *
 *  Author: Sagar Karandikar, skarandikar@berkeley.edu
 *  Based on the MIPS target interrupt support
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

#include "hw/hw.h"
#include "hw/riscv/cpudevs.h"
#include "cpu.h"

// TODO: move constants to cpu.h

/* irq request function, called in hw/irq.h by qemu_irq_raise (level = 1), 
 * qemu_irq_lower (level = 0), qemu_irq_pulse (level = 1, then 0) 
 *
 * The device will call this once to raise the interrupt line and once to 
 * lower the interrupt line for level-trigerring
 *
 */
static void cpu_riscv_irq_request(void *opaque, int irq, int level)
{
    RISCVCPU *cpu = opaque;
    CPURISCVState *env = &cpu->env;
    CPUState *cs = CPU(cpu);

    if (unlikely(irq < 0 || irq > 7)) {
        return;
    }

    if (level) {
        // level high, set the interrupt in CSR_STATUS
        env->helper_csr[CSR_STATUS] |= (1 << (irq + 24));
    } else {
        // level low, turn off the interrupt in CSR_STATUS
        env->helper_csr[CSR_STATUS] &= ~(1 << (irq + 24));
    }

    if (env->helper_csr[CSR_STATUS] & (0xFF << (24))) {
        // call cpu_interrupt from include/qom/cpu.h
        // this will call cpu_interrupt_handler aka
        // tcg_handle_interrupt from translate-all.c
        cpu_interrupt(cs, CPU_INTERRUPT_HARD);
    } else {
        // call cpu_reset_interrupt from qom/cpu.c
        // this just turns off the relevant bits
        // in cpu->interrupt_request
        cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
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

void cpu_riscv_soft_irq(CPURISCVState *env, int irq, int level)
{
    if (irq != 0) {
        return;
    }
    qemu_set_irq(env->irq[irq], level);
}
