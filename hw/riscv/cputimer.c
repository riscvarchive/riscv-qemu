/*
 *  QEMU RISC-V timer support
 *
 *  Author: Sagar Karandikar, skarandikar@berkeley.edu
 *  Based on the MIPS target timer support
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
#include "qemu/timer.h"

static uint64_t last_count_update;

// should be the cpu freq
#define TIMER_FREQ	100 * 1000 * 1000

uint64_t cpu_riscv_get_cycle (CPURISCVState *env) {
    uint64_t now;
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    // first, convert _now_ to seconds by dividing by get_ticks_per_sec
    // and then multiply by the timer freq.
    return muldiv64(now, TIMER_FREQ, get_ticks_per_sec());
}

static void cpu_riscv_timer_update(CPURISCVState *env)
{
    uint64_t now, next;
    uint32_t diff;

    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    diff = (uint32_t)(env->helper_csr[CSR_COMPARE] - env->helper_csr[CSR_COUNT]);
    next = now + muldiv64(diff, get_ticks_per_sec(), TIMER_FREQ);
    timer_mod(env->timer, next);
}

static void cpu_riscv_timer_expire(CPURISCVState *env)
{
    cpu_riscv_timer_update(env);
    qemu_irq_raise(env->irq[7]);
}

uint32_t cpu_riscv_get_count (CPURISCVState *env)
{
    uint64_t diff;

    diff = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - last_count_update;
    return env->helper_csr[CSR_COUNT] +
        (uint32_t)muldiv64(diff, TIMER_FREQ, get_ticks_per_sec());
}

void cpu_riscv_store_count (CPURISCVState *env, uint32_t count)
{
    /* Store new count register */
    env->helper_csr[CSR_COUNT] = count;
    last_count_update = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /* Update timer timer */
    cpu_riscv_timer_update(env);
}

void cpu_riscv_store_compare (CPURISCVState *env, uint32_t value)
{
    env->helper_csr[CSR_COMPARE] = value;
    qemu_irq_lower(env->irq[7]);
    // according to RISCV spec, any write to compare clears timer interrupt
    cpu_riscv_timer_update(env);
}

static void riscv_timer_cb (void *opaque)
{
    CPURISCVState *env;
    env = opaque;

    env->helper_csr[CSR_COUNT]++;
    cpu_riscv_timer_expire(env);
    env->helper_csr[CSR_COUNT]--;
}

void cpu_riscv_clock_init (CPURISCVState *env)
{
    env->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, &riscv_timer_cb, env);
    env->helper_csr[CSR_COMPARE] = 0;
    cpu_riscv_store_count(env, 1);
}
