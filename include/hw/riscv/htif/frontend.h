#ifndef HW_RISCV_FRONTEND_H
#define HW_RISCV_FRONTEND_H 1

#include "hw/hw.h"
#include "sysemu/sysemu.h"
#include "exec/memory.h"
#include "target-riscv/cpu.h"


#define RV_FSYSCALL_sys_openat 56
#define RV_FSYSCALL_sys_close 57
#define RV_FSYSCALL_sys_write 64
#define RV_FSYSCALL_sys_pread 67
#define RV_FSYSCALL_sys_exit  93
#define RV_FSYSCALL_sys_getmainvars 2011

uint64_t sys_openat(HTIFState *htifstate, uint64_t dirfd, uint64_t pname,
                    uint64_t len, uint64_t flags, uint64_t mode);

uint64_t sys_close(HTIFState *htifstate, uint64_t fd);

uint64_t sys_write(HTIFState *htifstate, uint64_t fd, uint64_t pbuf,
                   uint64_t len);

uint64_t sys_pread(HTIFState *htifstate, uint64_t fd, uint64_t pbuf,
                   uint64_t len, uint64_t off);

uint64_t sys_exit(HTIFState *htifstate, uint64_t code);

int handle_frontend_syscall(HTIFState *htifstate, uint64_t payload);

uint64_t sys_getmainvars(HTIFState *htifstate, uint64_t pbuf, uint64_t limit);

#endif
