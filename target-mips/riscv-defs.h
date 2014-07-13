#if !defined (__QEMU_MIPS_DEFS_H__)
#define __QEMU_MIPS_DEFS_H__

/* Real pages are variable size... */
#define TARGET_PAGE_BITS 13 // MODIFIED FOR RISCV RV64 8 KiB Pages
#define MIPS_TLB_MAX 128

#define TARGET_LONG_BITS 64 // this defs TCGv as TCGv_i64 in tcg/tcg-op.h
#define TARGET_PHYS_ADDR_SPACE_BITS 32
#define TARGET_VIRT_ADDR_SPACE_BITS 43

#endif /* !defined (__QEMU_MIPS_DEFS_H__) */
