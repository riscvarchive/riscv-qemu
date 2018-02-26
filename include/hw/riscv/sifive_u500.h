/*
 * SiFive U500 series machine interface
 *
 * Copyright (c) 2017 SiFive, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HW_SIFIVE_U500_H
#define HW_SIFIVE_U500_H

#define TYPE_SIFIVE_U500 "riscv.sifive_u500"

#define SIFIVE_U500(obj) \
    OBJECT_CHECK(SiFiveU500State, (obj), TYPE_SIFIVE_U500)

typedef struct SiFiveU500State {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    RISCVHartArrayState soc;
    DeviceState *plic;
    void *fdt;
    int fdt_size;
} SiFiveU500State;

enum {
    SIFIVE_U500_DEBUG,
    SIFIVE_U500_MROM,
    SIFIVE_U500_CLINT,
    SIFIVE_U500_PLIC,
    SIFIVE_U500_UART0,
    SIFIVE_U500_UART1,
    SIFIVE_U500_DRAM
};

enum {
    SIFIVE_U500_UART0_IRQ = 3,
    SIFIVE_U500_UART1_IRQ = 4
};

#define SIFIVE_U500_PLIC_HART_CONFIG "MS"
#define SIFIVE_U500_PLIC_NUM_SOURCES 127
#define SIFIVE_U500_PLIC_NUM_PRIORITIES 7
#define SIFIVE_U500_PLIC_PRIORITY_BASE 0x0
#define SIFIVE_U500_PLIC_PENDING_BASE 0x1000
#define SIFIVE_U500_PLIC_ENABLE_BASE 0x2000
#define SIFIVE_U500_PLIC_ENABLE_STRIDE 0x80
#define SIFIVE_U500_PLIC_CONTEXT_BASE 0x200000
#define SIFIVE_U500_PLIC_CONTEXT_STRIDE 0x1000

#endif
