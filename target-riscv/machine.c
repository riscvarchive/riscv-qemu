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
