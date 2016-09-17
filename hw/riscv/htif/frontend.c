/*
 * QEMU RISC-V Syscall Proxy Emulation
 *
 * Author: Sagar Karandikar, sagark@eecs.berkeley.edu
 *
 * This provides a set of functions used by the HTIF Syscall Proxy device.
 * This is used by bbl and pk. Currently, only syscalls needed by bbl to
 * boot Linux are supported.
 *
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

#include "qemu/osdep.h"
#include "hw/riscv/htif/htif.h"
#include "hw/riscv/htif/frontend.h"
#include <inttypes.h>
#include <fcntl.h>
#include <unistd.h>

/*#define DEBUG_FRONTEND_RISCV */

/* only supports one fd right now, for the kernel we load */
int real_kernelfd = -1;

#define BBL_AT_FDCWD (-100)

uint64_t sys_openat(HTIFState *htifstate, uint64_t dirfd, uint64_t pname,
        uint64_t len, uint64_t flags, uint64_t mode) {

    void *base = htifstate->main_mem_ram_ptr + (uintptr_t)pname;

    char name[len];
    int i;
    for (i = 0; i < len; i++) {
        name[i] = ldub_p((void *)(base + i));
    }

    /* in case host OS has different val for AT_FDCWD, e.g. OS X
       TODO: removed to fix clang/osx build, sys_openat isn't used anymore
       by bbl anyway
       dirfd = dirfd == BBL_AT_FDCWD ? AT_FDCWD : dirfd; */

    #ifdef DEBUG_FRONTEND_RISCV
    fprintf(stderr, "openat: %s\n"
           "dirfd %ld, flags %ld, mode %ld\n", name, dirfd, flags, mode);
    #endif

    real_kernelfd = openat(dirfd, name, flags, mode);

    #ifdef DEBUG_FRONTEND_RISCV
    fprintf(stderr, "got real fd: %d\n", real_kernelfd);
    #endif

    if (real_kernelfd != -1) {
        /* always give fd 3 to bbl, until we have a better tracking mechanism */
        return 3;
    }
    return -1;
}


uint64_t sys_close(HTIFState *htifstate, uint64_t fd)
{
    if (fd != 3) {
        fprintf(stderr, "INVALID close fd: %ld. only 3 allowed\n", fd);
        fprintf(stderr, "Did you supply the right kernel using -append?\n");
        exit(1);
    }

    if (close(real_kernelfd) < 0) {
        return -1;
    }
    real_kernelfd = -1;
    return 0;
}

/*
 * Used by bbl to print.
 */
uint64_t sys_write(HTIFState *htifstate, uint64_t fd, uint64_t pbuf,
                   uint64_t len)
{

    int i;
    char *printbuf = malloc(sizeof(char) * (len + 1));
    printbuf[len] = '\0'; /* null term for easy printing */
    void *base = htifstate->main_mem_ram_ptr + (uintptr_t)pbuf;
    for (i = 0; i < len; i++) {
        printbuf[i] = ldub_p((void *)(base + i));
    }

    switch (fd) {
    case 1:
    case 2:
        printf("%s", printbuf);
        break;
    default:
        fprintf(stderr, "INVALID SYS_WRITE\n");
        exit(1);
    }
    free(printbuf);
    return len;
}

uint64_t sys_pread(HTIFState *htifstate, uint64_t fd, uint64_t pbuf,
                   uint64_t len, uint64_t off)
{
    #ifdef DEBUG_FRONTEND_RISCV
    fprintf(stderr, "read fd: %ld, len: %ld, off: %ld\n", fd, len, off);
    #endif
    if (fd != 3) {
        fprintf(stderr, "INVALID pread fd: %ld. only 3 allowed\n", fd);
        exit(1);
    }

    char *buf = malloc(sizeof(char) * len);
    size_t bytes_read = pread(real_kernelfd, buf, len, off);

    void *base = htifstate->main_mem_ram_ptr + (uintptr_t)pbuf;
    int i;
    for (i = 0; i < bytes_read; i++) {
        stb_p((void *)(base + i), buf[i]);
    }
    free(buf);
    return bytes_read;
}

uint64_t sys_exit(HTIFState *htifstate, uint64_t code)
{
    printf("sys_exit. code: %ld\n", code << 1 | 1);
    exit(code << 1 | 1);
}

uint64_t sys_getmainvars(HTIFState *htifstate, uint64_t pbuf, uint64_t limit)
{
    #ifdef DEBUG_FRONTEND_RISCV
    fprintf(stderr, "%s\n", htifstate->kernel_cmdline);
    #endif

    void *base = htifstate->main_mem_ram_ptr + (uintptr_t)pbuf;

    /* assume args are bbl + some kernel for now
       later, do the right thing */
    const char *arg0 = "bbl";
    const char *arg1 = htifstate->kernel_cmdline;

    #define WORDS_LEN 5
    #define START_ARGS (WORDS_LEN * 8)
    uint64_t words[WORDS_LEN] = {2, START_ARGS + pbuf, START_ARGS + pbuf + 4,
                                 0, 0};
    int i;
    for (i = 0; i < WORDS_LEN; i++) {
        stq_p((void *)(base + i * 8), words[i]);
    }
    for (i = 0; i < 4; i++) {
        stb_p((void *)(base + START_ARGS + i), arg0[i]);
    }
    for (i = 0; i < strlen(arg1) + 1; i++) {
        stb_p((void *)(base + START_ARGS + 4 + i), arg1[i]);
    }
    /* currently no support for > 2 args */
    return 0;
}


int handle_frontend_syscall(HTIFState *htifstate, uint64_t payload)
{
    uint64_t mm[8];
    int i;
    void *base = htifstate->main_mem_ram_ptr + (uintptr_t)payload;
    for (i = 0; i < 8; i++) {
        mm[i] = ldq_p((void *)(base + i * 8));
    }

    #ifdef DEBUG_FRONTEND_RISCV
    for (i = 0; i < 8; i++) {
        fprintf(stderr, "elem %d, val 0x%016lx\n", i, mm[i]);
    }
    #endif

    uint64_t retval = -1;
    switch (mm[0]) {
    case RV_FSYSCALL_sys_openat:
        retval = sys_openat(htifstate, mm[1], mm[2], mm[3], mm[4], mm[5]);
        break;
    case RV_FSYSCALL_sys_close:
        retval = sys_close(htifstate, mm[1]);
        break;
    case RV_FSYSCALL_sys_write:
        retval = sys_write(htifstate, mm[1], mm[2], mm[3]);
        break;
    case RV_FSYSCALL_sys_pread:
        retval = sys_pread(htifstate, mm[1], mm[2], mm[3], mm[4]);
        break;
    case RV_FSYSCALL_sys_exit:
        retval = sys_exit(htifstate, mm[1]);
        break;
    case RV_FSYSCALL_sys_getmainvars:
        retval = sys_getmainvars(htifstate, mm[1], mm[2]);
        break;
    default:
        fprintf(stderr, "FRONTEND SYSCALL %ld NOT IMPLEMENTED\n", mm[0]);
        exit(1);
    }

    /* write retval to mm */
    stq_p((void *)base, retval);
    return 1;
}
