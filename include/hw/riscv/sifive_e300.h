/*
 * SiFive E300 series machine interface
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

#ifndef HW_SIFIVE_E300_H
#define HW_SIFIVE_E300_H

#define TYPE_SIFIVE_E300 "riscv.sifive.e300"

#define SIFIVE_E300(obj) \
    OBJECT_CHECK(SiFiveE300State, (obj), TYPE_SIFIVE_E300)

typedef struct SiFiveE300State {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    RISCVHartArrayState soc;
    DeviceState *plic;
} SiFiveE300State;

enum {
    SIFIVE_E300_DEBUG,
    SIFIVE_E300_MROM,
    SIFIVE_E300_OTP,
    SIFIVE_E300_CLINT,
    SIFIVE_E300_PLIC,
    SIFIVE_E300_AON,
    SIFIVE_E300_PRCI,
    SIFIVE_E300_OTP_CTRL,
    SIFIVE_E300_GPIO0,
    SIFIVE_E300_UART0,
    SIFIVE_E300_QSPI0,
    SIFIVE_E300_PWM0,
    SIFIVE_E300_UART1,
    SIFIVE_E300_QSPI1,
    SIFIVE_E300_PWM1,
    SIFIVE_E300_QSPI2,
    SIFIVE_E300_PWM2,
    SIFIVE_E300_XIP,
    SIFIVE_E300_DTIM
};

enum {
    SIFIVE_E300_UART0_IRQ = 3,
    SIFIVE_E300_UART1_IRQ = 4
};

#define SIFIVE_E300_PLIC_HART_CONFIG "M"
#define SIFIVE_E300_PLIC_NUM_SOURCES 127
#define SIFIVE_E300_PLIC_NUM_PRIORITIES 7
#define SIFIVE_E300_PLIC_PRIORITY_BASE 0x0
#define SIFIVE_E300_PLIC_PENDING_BASE 0x1000
#define SIFIVE_E300_PLIC_ENABLE_BASE 0x2000
#define SIFIVE_E300_PLIC_ENABLE_STRIDE 0x80
#define SIFIVE_E300_PLIC_CONTEXT_BASE 0x200000
#define SIFIVE_E300_PLIC_CONTEXT_STRIDE 0x1000

#endif
