#include "qemu/osdep.h"
#include "disas/bfd.h"

#define USE_SPIKE_DASM

int print_insn_riscv(bfd_vma pc, disassemble_info *info)
{
    int i, n = info->buffer_length;
    int j;
    uint8_t *buf = g_malloc(n);

    #ifdef USE_SPIKE_DASM
    int buflen = 100;
    char *runbuf = g_malloc(buflen); /* TODO len */
    #endif

    info->read_memory_func(pc, buf, n, info);

    const char *prefix = "RISC-V Inst";

    for (j = 0; j < n; j += 4) {
        info->fprintf_func(info->stream, "\n%s: 0x", prefix);

        /* little-endian */
        for (i = 3; i >= 0; --i) {
            info->fprintf_func(info->stream, "%02x", buf[j + i]);
        }

        /* use spike-dasm */
        #ifdef USE_SPIKE_DASM
        info->fprintf_func(info->stream, "\n");
        snprintf(runbuf, buflen, "echo 'DASM(%02x%02x%02x%02x)\n' \
                 | spike-dasm 1>&2", buf[j + 3], buf[j + 2], buf[j + 1],
                 buf[j + 0]);
        int res = system(runbuf);
        if (res) {
            printf("spike-dasm error\n");
            exit(1);
        }
        #endif
    }

    g_free(runbuf);
    g_free(buf);
    return n;
}
