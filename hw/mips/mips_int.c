/*
 * QEMU MIPS interrupt support
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
#include "hw/mips/cpudevs.h"
#include "cpu.h"


// just for testing
#define BTBPat "%d%d%d%d%d%d%d%d"
#define BTBMac(byte)  \
      (byte & 0x80 ? 1 : 0), \
  (byte & 0x40 ? 1 : 0), \
  (byte & 0x20 ? 1 : 0), \
  (byte & 0x10 ? 1 : 0), \
  (byte & 0x08 ? 1 : 0), \
  (byte & 0x04 ? 1 : 0), \
  (byte & 0x02 ? 1 : 0), \
  (byte & 0x01 ? 1 : 0) 

static void cpu_mips_irq_request(void *opaque, int irq, int level)
{
    MIPSCPU *cpu = opaque;
    CPUMIPSState *env = &cpu->env;
    CPUState *cs = CPU(cpu);

    if (irq < 0 || irq > 7) {
        return;
    }

    // currently disable all irqs that are not 4
    if (irq != 4) {
        return;
    }

/*    if (env->active_tc.PC & 0xffff000000000000) {
        return;
    }*/

/*
    if (level) {
        env->CP0_Cause |= 1 << (irq + CP0Ca_IP);
    } else {
        env->CP0_Cause &= ~(1 << (irq + CP0Ca_IP));
    }
*/

/*
    printf("called with "BTBPat"\n", BTBMac(1 << irq));
    printf("enabled are "BTBPat"\n", BTBMac((uint8_t)((env->active_tc.csr[CSR_STATUS] >> 16) & 0xFF)));

*/

    if (level) {
        env->active_tc.csr[CSR_STATUS] |= (1 << (irq + 24));
    } else {
        env->active_tc.csr[CSR_STATUS] &= ~(1 << (irq + 24));
    }
/*
    if ((env->active_tc.csr[CSR_STATUS] >> 16) & (0x1 << irq)) {
        printf("doing interrupt\n");
        cpu_interrupt(cs, CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
    }

    */

    if (env->active_tc.csr[CSR_STATUS] & (0xFF << 24)) {
        cpu_interrupt(cs, CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
    }


/*
    if (env->CP0_Cause & CP0Ca_IP_mask) {
        cpu_interrupt(cs, CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
    }*/
}

void cpu_mips_irq_init_cpu(CPUMIPSState *env)
{
    qemu_irq *qi;
    int i;

    qi = qemu_allocate_irqs(cpu_mips_irq_request, mips_env_get_cpu(env), 8);
    for (i = 0; i < 8; i++) {
        env->irq[i] = qi[i];
    }
}

void cpu_mips_soft_irq(CPUMIPSState *env, int irq, int level)
{
    if (irq < 0 || irq > 2) {
        return;
    }

    qemu_set_irq(env->irq[irq], level);
}
