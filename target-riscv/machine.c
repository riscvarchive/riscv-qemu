/*
 *  RISC-V CPU machine state helpers
 *
 *  Author: Sagar Karandikar, skarandikar@berkeley.edu
 *  Based on the MIPS target
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

#include "hw/hw.h"
#include "hw/boards.h"

#include "cpu.h"

static void save_tc(QEMUFile *f, TCState *tc)
{
    int i;
    /* Save active TC */
    for (i = 0; i < 32; i++) {
        qemu_put_betls(f, &tc->gpr[i]);
    }
    for (i = 0; i < 32; i++) {
        qemu_put_betls(f, &tc->fpr[i]);
    }
    qemu_put_betls(f, &tc->PC);
}

void cpu_save(QEMUFile *f, void *opaque)
{
    CPURISCVState *env = opaque;
    int i;

    /* Save active TC */
    save_tc(f, &env->active_tc);

    /* Save CPU metastate */
    qemu_put_be32s(f, &env->current_tc);

    for (i = 0; i < 32; i++) {
        qemu_put_betls(f, &env->helper_csr[i]);
    }
}

static void load_tc(QEMUFile *f, TCState *tc)
{
    int i;
    /* Save active TC */
    for(i = 0; i < 32; i++) {
        qemu_get_betls(f, &tc->gpr[i]);
    }
    for(i = 0; i < 32; i++) {
        qemu_get_betls(f, &tc->fpr[i]);
    }
    qemu_get_betls(f, &tc->PC);
}

int cpu_load(QEMUFile *f, void *opaque, int version_id)
{
    CPURISCVState *env = opaque;
    RISCVCPU *cpu = riscv_env_get_cpu(env);
    int i;

    if (version_id != 3)
        return -EINVAL;

    /* Load active TC */
    load_tc(f, &env->active_tc);

    /* Load CPU metastate */
    qemu_get_be32s(f, &env->current_tc);

    for (i = 0; i < 32; i++) {
        qemu_get_betls(f, &env->helper_csr[i]);
    }

    tlb_flush(CPU(cpu), 1);
    return 0;
}
