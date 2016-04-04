/*
 * QEMU RISC-V timer, instret counter support
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

#include "hw/hw.h"
#include "hw/riscv/cpudevs.h"
#include "hw/riscv/cputimer.h"
#include "qemu/timer.h"

//#define TIMER_DEBUGGING_RISCV

static uint64_t written_delta;
static uint64_t instret_delta;

// this is the "right value" for defaults in pk/linux
// see pk/sbi_entry.S and arch/riscv/kernel/time.c call to
// clockevents_config_and_register
#define TIMER_FREQ	10 * 1000 * 1000
// CPU_FREQ is for instret approximation - say we're running at 1 BIPS
#define CPU_FREQ    1000 * 1000 * 1000

inline uint64_t rtc_read(CPURISCVState *env) {
    return muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), TIMER_FREQ, get_ticks_per_sec());
}

inline uint64_t rtc_read_with_delta(CPURISCVState *env) {
    return rtc_read(env) + written_delta;
}

inline uint64_t instret_read(CPURISCVState *env) {
    return muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), CPU_FREQ, get_ticks_per_sec());
}

static inline uint64_t instret_read_with_delta(CPURISCVState *env) {
    return instret_read(env) + instret_delta;
}

/*
 * Called when mtimecmp is written to update the QEMU timer or immediately
 * trigger timer interrupt if mtimecmp <= current timer value.
 */
static inline void cpu_riscv_timer_update(CPURISCVState *env)
{
    uint64_t next;
    uint64_t diff;

    uint64_t rtc_r = rtc_read_with_delta(env);

    #ifdef TIMER_DEBUGGING_RISCV
    printf("timer update: mtimecmp %016lx, timew %016lx\n",
            env->csr[NEW_CSR_MTIMECMP], rtc_r);
    #endif

    if (env->csr[NEW_CSR_MTIMECMP] <= rtc_r) {
        // if we're setting an MTIMECMP value in the "past", immediately raise
        // the timer interrupt
        env->csr[NEW_CSR_MIP] |= MIP_MTIP;
        qemu_irq_raise(env->irq[7]);
        return;
    }

    // otherwise, set up the future timer interrupt
    diff = env->csr[NEW_CSR_MTIMECMP] - rtc_r;
    // back to ns (note args switched in muldiv64)
    next = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
        muldiv64(diff, get_ticks_per_sec(), TIMER_FREQ);
    timer_mod(env->timer, next);
}

/*
 * Called by the callback used when the timer set using timer_mod expires.
 * Should raise the timer interrupt line
 */
static inline void cpu_riscv_timer_expire(CPURISCVState *env)
{
    // do not call update here
    env->csr[NEW_CSR_MIP] |= MIP_MTIP;
    qemu_irq_raise(env->irq[7]);
}


inline void cpu_riscv_store_timew(CPURISCVState *env, uint64_t val_to_write) {
    #ifdef TIMER_DEBUGGING_RISCV
    printf("write timew: 0x%016lx\n", val_to_write);
    #endif

    written_delta = val_to_write - rtc_read(env);
}

inline void cpu_riscv_store_instretw(CPURISCVState *env, uint64_t val_to_write) {
    #ifdef TIMER_DEBUGGING_RISCV
    printf("write instretw: 0x%016lx\n", val_to_write);
    #endif

    written_delta = val_to_write - instret_read(env);
}

inline uint64_t cpu_riscv_read_instretw(CPURISCVState *env) {
    uint64_t retval = instret_read_with_delta(env);
    return retval;
}

inline uint64_t cpu_riscv_read_mtime(CPURISCVState *env) {
    uint64_t retval = rtc_read(env);
    return retval;
}

inline uint64_t cpu_riscv_read_stime(CPURISCVState *env) {
    uint64_t retval = rtc_read(env);
    return retval;
}

inline uint64_t cpu_riscv_read_time(CPURISCVState *env) {
    uint64_t retval = rtc_read_with_delta(env);
    return retval;
}

inline void cpu_riscv_store_compare (CPURISCVState *env, uint64_t value)
{
    #ifdef TIMER_DEBUGGING_RISCV
    uint64_t rtc_r = rtc_read_with_delta(env);
    printf("wrote mtimecmp %016lx, timew %016lx\n", value, rtc_r);
    #endif

    env->csr[NEW_CSR_MTIMECMP] = value;
    env->csr[NEW_CSR_MIP] &= ~MIP_MTIP;
    cpu_riscv_timer_update(env);
}

/*
 * Callback used when the timer set using timer_mod expires.
 */
static void riscv_timer_cb (void *opaque)
{
    CPURISCVState *env;
    env = opaque;
    cpu_riscv_timer_expire(env);
}

/*
 * Initialize clock mechanism.
 */
void cpu_riscv_clock_init (CPURISCVState *env)
{
    env->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, &riscv_timer_cb, env);
    env->csr[NEW_CSR_MTIMECMP] = 0;
    cpu_riscv_store_timew(env, 1);
    written_delta = 0;
    instret_delta = 0;
}
