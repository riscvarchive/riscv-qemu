#ifndef _RISCV_SOC_H_
#define _RISCV_SOC_H_

#define NUM_HARTS                                 (5)

#define DEBUG_AREA_ADDR                  (0x00000100)
#define DEBUG_AREA_SZ                        (0xf000)

#define NUM_TIM_TYPES                             (2)
#define DEFAULT_DTIM_SZ                      (0x2000)
#define DEFAULT_ITIM_SZ                      (0x7000)
#define TIM_BLOCK_SZ                         (0x8000)

#define HART0_DTIM                       (0x01000000)
#define HART0_DTIM_SZ               (DEFAULT_DTIM_SZ)

#define HART0_ITIM                       (0x01800000)
#define HART0_ITIM_SZ                        (0x2000)
#define HART1_ITIM                       (0x01808000)
#define HART1_ITIM_SZ               (DEFAULT_ITIM_SZ)
#define HART2_ITIM                       (0x01810000)
#define HART2_ITIM_SZ               (DEFAULT_ITIM_SZ)
#define HART3_ITIM                       (0x01818000)
#define HART3_ITIM_SZ               (DEFAULT_ITIM_SZ)
#define HART4_ITIM                       (0x01820000)
#define HART4_ITIM_SZ               (DEFAULT_ITIM_SZ)

#define CLINT_BASE_ADDR                  (0x02000000)
#define CLINT_REG_SZ                         (0xc000)
#define CLINT_MSIP_REG_SZ                    (0x4000)
#define CLINT_TIME_REG_SZ                    (0x8000)

#define CACHE_CTRL_BASE_ADDR             (0x02010000)
#define CACHE_CTRL_REG_SZ                     (0xfff)

#define DMA_CTRL_BASE_ADDR               (0x03000000)
#define DMA_CTRL_REG_SZ                     (0xfffff)

#define L2LIM_BASE_ADDR                  (0x08000000)
#define L2LIM_REG_SZ                      (0x1ffffff)
#define L2LIM_WAY_SZ                      (0x0020000)

#define L2ZERO_DEV_BASE_ADDR             (0x0a000000)
#define L2ZERO_DEV_REG_SZ                 (0x1ffffff)

#define PLIC_BASE_ADDR                   (0x0c000000)
#define PLIC_REG_SZ                      (0x00400000)

#define SYSTEM_PORT0_AXI_DO_BASE_ADDR    (0x20000000)

#define UART0_BASE_ADDR                  (0x20000000)
#define UART0_REG_SZ                           (0x50)

#define SYSTEM_PORT1_AXI_D1_BASE_ADDR    (0x21000000)

#define SYSTEM_PORT0_AXI_D0_BASE_ADDR2   (0x30000000)
#define SYSTEM_PORT2_AXI_F0_BASE_ADDR    (0x60000000)

#define MEMORY_PORT_AXI_C_BASE_ADDR      (0x80000000)

#define SYSTEM_PORT4_AXI_NC_BASE_ADDR    (0xc0000000)

#define SYSTEM_PORT3_AXI_F1_BASE_ADDR    (0xe0000000)

#define MEMORY_PORT_AXI_C_BASE_ADDR1   (0x1000000000ll)
#define SYSTEM_PORT4_AXI_NC_BASE_ADDR1 (0x1400000000ll)
#define SYSTEM_PORT2_AXI_F0_BASE_ADDR1 (0x2000000000ll)
#define SYSTEM_PORT3_AXI_F1_BASE_ADDR1 (0x3000000000ll)

#define NUM_PMPS                               (16)

#endif
