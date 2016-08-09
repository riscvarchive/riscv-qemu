/*
 * RISC-V emulation for qemu: CPU Initialization Routines.
 *
 * Author: Sagar Karandikar, sagark@eecs.berkeley.edu
 *
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

#define MCPUID_RV64I   (2L << 62)
#define MCPUID_SUPER   (1L << ('S' - 'A'))
#define MCPUID_USER    (1L << ('U' - 'A'))
#define MCPUID_I       (1L << ('I' - 'A'))
#define MCPUID_M       (1L << ('M' - 'A'))
#define MCPUID_A       (1L << ('A' - 'A'))
#define MCPUID_F       (1L << ('F' - 'A'))
#define MCPUID_D       (1L << ('D' - 'A'))

struct riscv_def_t {
    const char *name;
    target_ulong init_misa_reg;
};

/* RISC-V CPU definitions */
static const riscv_def_t riscv_defs[] =
{
    {
        .name = "riscv-generic",
        // for now, hardcode RV64G:
        .init_misa_reg = MCPUID_RV64I | MCPUID_SUPER | MCPUID_USER | MCPUID_I
            | MCPUID_M | MCPUID_A | MCPUID_F | MCPUID_D,
    },
};

static const riscv_def_t *cpu_riscv_find_by_name (const char *name)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(riscv_defs); i++) {
        if (strcasecmp(name, riscv_defs[i].name) == 0) {
            return &riscv_defs[i];
        }
    }
    return NULL;
}

void riscv_cpu_list (FILE *f, fprintf_function cpu_fprintf)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(riscv_defs); i++) {
        (*cpu_fprintf)(f, "RISCV '%s'\n", riscv_defs[i].name);
    }
}
