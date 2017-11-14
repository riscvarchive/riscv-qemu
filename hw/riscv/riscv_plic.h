/*
 * QEMU RISC-V PLIC
 * Author: Daire McNamara, daire.mcnamara@emdalo.com
 *
 * This provides a RISC-V PLIC device
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

#ifndef _RISCV_PLIC_H_
#define _RISCV_PLIC_H_
#include "target-riscv/cpu.h"

#define PLIC_NUM_SOURCES (180)

#define PLIC_SOURCE_PRIORITY_BASE   (0)
#define PLIC_SOURCE_PRIORITY_SIZE   (4 * PLIC_NUM_SOURCES)
#define PLIC_SOURCE_PRIORITY_END    (PLIC_SOURCE_PRIORITY_BASE \
                                     + PLIC_SOURCE_PRIORITY_SIZE)

#define PLIC_COMMON_SIZE            ((PLIC_NUM_SOURCES / 32) * 4)

#define PLIC_PENDING_BASE           (0x1000)
#define PLIC_PENDING_END            (PLIC_PENDING_BASE + PLIC_COMMON_SIZE)

#define PLIC_HART0_M_ENB_BASE       (0x2000)
#define PLIC_HART0_M_ENB_END        (PLIC_HART0_M_ENB_BASE + PLIC_COMMON_SIZE)

#define PLIC_HART1_M_ENB_BASE       (0x2080)
#define PLIC_HART1_M_ENB_END        (PLIC_HART1_M_ENB_BASE + PLIC_COMMON_SIZE)
#define PLIC_HART1_S_ENB_BASE       (0x2100)
#define PLIC_HART1_S_ENB_END        (PLIC_HART1_S_ENB_BASE + PLIC_COMMON_SIZE)

#define PLIC_HART2_M_ENB_BASE       (0x2180)
#define PLIC_HART2_M_ENB_END        (PLIC_HART2_M_ENB_BASE + PLIC_COMMON_SIZE)
#define PLIC_HART2_S_ENB_BASE       (0x2200)
#define PLIC_HART2_S_ENB_END        (PLIC_HART2_S_ENB_BASE + PLIC_COMMON_SIZE)

#define PLIC_HART3_M_ENB_BASE       (0x2280)
#define PLIC_HART3_M_ENB_END        (PLIC_HART3_M_ENB_BASE + PLIC_COMMON_SIZE)
#define PLIC_HART3_S_ENB_BASE       (0x2300)
#define PLIC_HART3_S_ENB_END        (PLIC_HART3_S_ENB_BASE + PLIC_COMMON_SIZE)

#define PLIC_HART4_M_ENB_BASE       (0x2380)
#define PLIC_HART4_M_ENB_END        (PLIC_HART4_M_ENB_BASE + PLIC_COMMON_SIZE)
#define PLIC_HART4_S_ENB_BASE       (0x2400)
#define PLIC_HART4_S_ENB_END        (PLIC_HART4_S_ENB_BASE + PLIC_COMMON_SIZE)

#define PLIC_PRIO_THRESH_ADDR            (0x200000)
#define PLIC_HART0_M_PRIO_THRESH_ADDR    (0x200000)
#define PLIC_HART0_M_CLAIM_COMPLETE_ADDR (0x200004)

#define PLIC_HART1_M_PRIO_THRESH_ADDR    (0x201000)
#define PLIC_HART1_M_CLAIM_COMPLETE_ADDR (0x201004)
#define PLIC_HART1_S_PRIO_THRESH_ADDR    (0x202000)
#define PLIC_HART1_S_CLAIM_COMPLETE_ADDR (0x202004)

#define PLIC_HART2_M_PRIO_THRESH_ADDR    (0x203000)
#define PLIC_HART2_M_CLAIM_COMPLETE_ADDR (0x203004)
#define PLIC_HART2_S_PRIO_THRESH_ADDR    (0x204000)
#define PLIC_HART2_S_CLAIM_COMPLETE_ADDR (0x204004)

#define PLIC_HART3_M_PRIO_THRESH_ADDR    (0x205000)
#define PLIC_HART3_M_CLAIM_COMPLETE_ADDR (0x205004)
#define PLIC_HART3_S_PRIO_THRESH_ADDR    (0x206000)
#define PLIC_HART3_S_CLAIM_COMPLETE_ADDR (0x206004)

#define PLIC_HART4_M_PRIO_THRESH_ADDR    (0x207000)
#define PLIC_HART4_M_CLAIM_COMPLETE_ADDR (0x207004)
#define PLIC_HART4_S_PRIO_THRESH_ADDR    (0x208000)
#define PLIC_HART4_S_CLAIM_COMPLETE_ADDR (0x208004)

#define PLIC_OFFSET_SIZE                   (0x1000)
#define PLIC_HART_OFFSET_SIZE                (0x80)
#define PLIC_PT_CC_OFFSET_SIZE           (0x200000)


#define UART0_PLIC_SRC (41)

typedef void (*qemu_plic_irq)(uint32_t plic_src);

void plic_init(RISCVCPU *cpu);
void plic_raise_irq(uint32_t irq);
void plic_lower_irq(uint32_t irq);

#endif

