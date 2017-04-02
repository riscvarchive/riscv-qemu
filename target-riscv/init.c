#include "qemu/osdep.h"
#include "cpu.h"

struct riscv_def_t {
    const char *name;
    uint64_t init_misa_reg;
};

/* RISC-V CPU definitions */
static const riscv_def_t riscv_defs[] = {
    {
        .name = "any",
#if defined(TARGET_RISCV64)
        /* RV64G */
        .init_misa_reg = MISA_RV64I | MISA_S | MISA_U | MISA_I
            | MISA_M | MISA_A | MISA_F | MISA_D | MISA_C,
#else
        /* RV32G */
        .init_misa_reg = MISA_RV32I | MISA_S | MISA_U | MISA_I
            | MISA_M | MISA_A | MISA_F | MISA_D | MISA_C,
#endif
    },
};

static const riscv_def_t *cpu_riscv_find_by_name(const char *name)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(riscv_defs); i++) {
        if (strcasecmp(name, riscv_defs[i].name) == 0) {
            return &riscv_defs[i];
        }
    }
    return NULL;
}

void riscv_cpu_list(FILE *f, fprintf_function cpu_fprintf)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(riscv_defs); i++) {
        (*cpu_fprintf)(f, "RISCV '%s'\n", riscv_defs[i].name);
    }
}

RISCVCPU *cpu_riscv_init(const char *cpu_model)
{
    RISCVCPU *cpu;
    CPURISCVState *env;
    const riscv_def_t *def;

    def = cpu_riscv_find_by_name(cpu_model);
    if (!def) {
        return NULL;
    }
    cpu = RISCV_CPU(object_new(TYPE_RISCV_CPU));
    env = &cpu->env;
    env->cpu_model = def;

#ifndef CONFIG_USER_ONLY
    env->priv = PRV_M;
    /* set mcpuid from def */
    env->misa = def->init_misa_reg;
    env->max_isa = def->init_misa_reg;
    cpu_riscv_set_tb_flags(env);
#endif

    object_property_set_bool(OBJECT(cpu), true, "realized", NULL);

    /* fpu flags: */
    set_default_nan_mode(1, &env->fp_status);

    return cpu;
}
