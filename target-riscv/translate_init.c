/*
 *  RISC-V emulation for qemu: CPU initialisation routines.
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

struct riscv_def_t {
    const char *name;
    uint64_t init_status_reg;
};

/* RISC-V CPU definitions */
static const riscv_def_t riscv_defs[] =
{
    {  
        .name = "riscv-generic",
        .init_status_reg = SR_S64 | SR_U64 | SR_EF | SR_S,
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
