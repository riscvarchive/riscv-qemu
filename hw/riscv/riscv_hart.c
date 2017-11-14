/*
 * QEMU RISC-V multi-hart emulation helpers
 * Author: Daire McNamara, daire.mcnamara@emdalo.com
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
#include "hw/char/serial.h"
#include "hw/riscv/riscv_rtc.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/boards.h"
#include "hw/riscv/cpudevs.h"
#include "sysemu/char.h"
#include "sysemu/arch_init.h"
#include "qemu/log.h"
#include "hw/loader.h"
#include "elf.h"
#include "exec/address-spaces.h"
#include "hw/sysbus.h"             /* SysBusDevice */
#include "qemu/host-utils.h"
#include "sysemu/qtest.h"
#include "qemu/error-report.h"
#include "hw/empty_slot.h"
#include "qemu/error-report.h"
#include "sysemu/block-backend.h"

typedef struct hart_share {
  CPURISCVState **envs;
  CPUState **cpu_states;
  uint32_t num_envs;
} hart_share_t;


hart_share_t hart_shared = { 0, 0, 0 };

void hart_shared_init(uint32_t num_harts)
{
    hart_shared.envs =
       (CPURISCVState **)malloc(sizeof(CPURISCVState *) * num_harts);
    memset(hart_shared.envs, 0, sizeof(CPURISCVState *) * num_harts);
    hart_shared.num_envs = num_harts;

    hart_shared.cpu_states =
        (CPUState **)malloc(sizeof(CPUState *) * num_harts);
    memset(hart_shared.cpu_states, 0, sizeof(CPUState *) * num_harts);
}

void hart_add_env_and_state(uint32_t index, CPURISCVState *env,
    CPUState *cpu_state)
{
    if (index < hart_shared.num_envs) {
        hart_shared.envs[index] = env;
        hart_shared.cpu_states[index] = cpu_state;
    }
}

CPUState *hart_get_cpu_state(uint32_t index)
{
    CPUState *result = NULL;

    if (index < hart_shared.num_envs) {
        result = hart_shared.cpu_states[index];
    }

    return result;
}

CPURISCVState *hart_get_env(uint32_t index)
{
    CPURISCVState *result = NULL;
    if (index < hart_shared.num_envs) {
        result = hart_shared.envs[index];
    }

    return result;
}

