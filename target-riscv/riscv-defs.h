#if !defined (__QEMU_RISCV_DEFS_H__)
#define __QEMU_RISCV_DEFS_H__

#define TARGET_PAGE_BITS 12 // 4 KiB Pages
//#define RISCV_TLB_MAX 0 not used. was for MIPS tlb
// if you're looking to change the QEMU softmmu size, look for TLB_
// #define CPU_TLB_BITS 2 in /include/exec/cpu-defs.h

#define TARGET_LONG_BITS 64 // this defs TCGv as TCGv_i64 in tcg/tcg-op.h
// according to spec? 38 PPN + 12 Offset
#define TARGET_PHYS_ADDR_SPACE_BITS 50
#define TARGET_VIRT_ADDR_SPACE_BITS 39

#endif /* !defined (__QEMU_RISCV_DEFS_H__) */
