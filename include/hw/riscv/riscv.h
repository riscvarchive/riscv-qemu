#ifndef HW_RISCV_H
#define HW_RISCV_H
/* Definitions for riscv board emulation.  */

/* Kernels can be configured with 64KB pages */
#define INITRD_PAGE_MASK (~((1 << 16) - 1))

#include "exec/memory.h"

#endif
