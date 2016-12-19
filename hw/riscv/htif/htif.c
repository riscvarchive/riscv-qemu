/*
 * QEMU RISC-V Host Target Interface (HTIF) Emulation
 *
 * Author: Sagar Karandikar, sagark@eecs.berkeley.edu
 *
 * This provides HTIF device emulation for QEMU. At the moment this allows
 * for identical copies of bbl/linux to run on both spike and QEMU.
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
#include "hw/char/serial.h"
#include "sysemu/char.h"
#include "hw/riscv/htif/htif.h"
#include "qemu/timer.h"
#include "exec/address-spaces.h"
#include "qemu/error-report.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <inttypes.h>
#include "elf_symb.h"

#define ENABLE_CHARDEV
/*#define DEBUG_CHARDEV */
/*#define DEBUG_HTIF */

#ifdef ENABLE_CHARDEV
/*
 * Called by the char dev to see if HTIF is ready to accept input.
 */
static int htif_can_recv(void *opaque)
{
    return 1;
}

/*
 * Called by the char dev to supply input to HTIF console.
 * We assume that we will receive one character at a time.
 */
static void htif_recv(void *opaque, const uint8_t *buf, int size)
{
    if (size != 1) {
        return;
    }

    HTIFState *htifstate = opaque;

    #ifdef DEBUG_CHARDEV
    if (htifstate->env->mfromhost != 0x0) {
        fprintf(stderr, "recv handler: fromhost was not ready to \
                         accept input\n");
        fprintf(stderr, "recv handler: prev value was: %016lx\n",
                htifstate->env->mfromhost);
    }
    #endif

    uint64_t val_written = htifstate->pending_read;
    uint64_t resp = 0x100 | *buf;

    htifstate->env->mfromhost = (val_written >> 48 << 48) | (resp << 16 >> 16);
    qemu_irq_raise(htifstate->irq);
}

/*
 * Called by the char dev to supply special events to the HTIF console.
 * Not used for HTIF.
 */
static void htif_event(void *opaque, int event)
{
    #ifdef DEBUG_CHARDEV
    fprintf(stderr, "GOT EVENT: %d\n", event);
    #endif
}
#endif

static void htif_pre_save(void *opaque)
{
    return;
}

static int htif_post_load(void *opaque, int version_id)
{
    return 0;
}

const VMStateDescription vmstate_htif = {
    .name = "htif",
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = htif_pre_save,
    .post_load = htif_post_load,
    .fields      = (VMStateField []) { /* TODO what */
        VMSTATE_UINT64(tohost_offset, HTIFState),
        VMSTATE_UINT64(fromhost_offset, HTIFState),
        VMSTATE_UINT64(tohost_size, HTIFState),
        VMSTATE_UINT64(fromhost_size, HTIFState),
        VMSTATE_END_OF_LIST()
    },
};

static void dma_strcopy(HTIFState *htifstate, char *str, hwaddr phys_addr)
{
    int i = 0;
    void *base_copy_addr = htifstate->main_mem_ram_ptr + phys_addr;
    while (*(str + i)) {
        stb_p((void *)(base_copy_addr + i), *(str + i));
        i++;
    }
    stb_p((void *)(base_copy_addr + i), 0); /* store null term */
}

static void htif_handle_tohost_write(HTIFState *htifstate, uint64_t val_written)
{
    #ifdef DEBUG_HTIF
    fprintf(stderr, "TOHOST WRITE WITH val 0x%016lx\n", val_written);
    #endif

    uint8_t device = val_written >> 56;
    uint8_t cmd = val_written >> 48;
    uint64_t payload = val_written & 0xFFFFFFFFFFFFULL;

    uint64_t addr = payload >> 8;
    hwaddr real_addr = (hwaddr)addr;
    uint8_t what = payload & 0xFF;
    int resp;

    resp = 0; /* stop gcc complaining */
    #ifdef DEBUG_HTIF
    fprintf(stderr, "mtohost write:\n-device: %d\n-cmd: %d\n-what: %02lx\n\
                     -payload: %016lx\n", device, cmd, payload & 0xFF, payload);
    #endif

    /*
     * Currently, there is a fixed mapping of devices:
     * 0: riscv-tests Pass/Fail Reporting Only (no syscall proxy)
     * 1: Console
     */
    if (unlikely(device == 0x0)) {
        /* frontend syscall handler, only test pass/fail support */
        if (cmd == 0x0) {
            #ifdef DEBUG_HTIF
            fprintf(stderr, "frontend syscall handler\n");
            #endif
            if (payload & 0x1) {
                /* test result */
                if (payload >> 1) {
                    printf("*** FAILED *** (exitcode = %016" PRIx64 ")\n",
                           payload >> 1);
                } else {
                    printf("TEST PASSED\n");
                }
                exit(payload >> 1);
            }
            fprintf(stderr, "pk syscall proxy not supported\n");
        } else if (cmd == 0xFF) {
            /* use what */
            if (what == 0xFF) {
                #ifdef DEBUG_HTIF
                fprintf(stderr, "registering name\n");
                #endif
                dma_strcopy(htifstate, (char *)"syscall_proxy", real_addr);
            } else if (what == 0x0) {
                #ifdef DEBUG_HTIF
                fprintf(stderr, "registering syscall cmd\n");
                #endif
                dma_strcopy(htifstate, (char *)"syscall", real_addr);
            } else {
                #ifdef DEBUG_HTIF
                fprintf(stderr, "registering end of cmds list\n");
                #endif
                dma_strcopy(htifstate, (char *)"", real_addr);
            }
            resp = 0x1; /* write to indicate device name placed */
        } else {
            fprintf(stderr, "HTIF device %d: UNKNOWN COMMAND\n", device);
            exit(1);
        }
    } else if (likely(device == 0x1)) {
        /* HTIF Console */
        if (cmd == 0x0) {
            /* this should be a queue, but not yet implemented as such */
            htifstate->pending_read = val_written;
            htifstate->env->mtohost = 0; /* clear to indicate we read */
            return;
        } else if (cmd == 0x1) {
            #ifdef ENABLE_CHARDEV
            qemu_chr_fe_write(htifstate->chr, (uint8_t *)&payload, 1);
            #endif
            resp = 0x100 | (uint8_t)payload;
        } else if (cmd == 0xFF) {
            /* use what */
            if (what == 0xFF) {
                #ifdef DEBUG_HTIF
                fprintf(stderr, "registering name\n");
                #endif
                dma_strcopy(htifstate, (char *)"bcd", real_addr);
            } else if (what == 0x0) {
                #ifdef DEBUG_HTIF
                fprintf(stderr, "registering read cmd\n");
                #endif
                dma_strcopy(htifstate, (char *)"read", real_addr);
            } else if (what == 0x1) {
                #ifdef DEBUG_HTIF
                fprintf(stderr, "registering write cmd\n");
                #endif
                dma_strcopy(htifstate, (char *)"write", real_addr);
            } else {
                #ifdef DEBUG_HTIF
                fprintf(stderr, "registering end of cmds list\n");
                #endif
                dma_strcopy(htifstate, (char *)"", real_addr);
            }
            resp = 0x1; /* write to indicate device name placed */
        } else {
            fprintf(stderr, "HTIF device %d: UNKNOWN COMMAND\n", device);
            exit(1);
        }
    /* all other devices */
    } else if (device == 0x2 && cmd == 0xFF && what == 0xFF) {
        #ifdef DEBUG_HTIF
        fprintf(stderr, "registering no device as last\n");
        #endif
        stb_p((void *)(htifstate->main_mem_ram_ptr + real_addr), 0);
        resp = 0x1; /* write to indicate device name placed */
    } else {
        fprintf(stderr, "HTIF UNKNOWN DEVICE OR COMMAND!\n");
        fprintf(stderr, "-device: %d\n-cmd: %d\n-what: %02" PRIx64 "\n-payload: \
                         %016" PRIx64 "\n", device, cmd, payload & 0xFF, payload);
        exit(1);
    }
    while (!htifstate->fromhost_inprogress &&
            htifstate->env->mfromhost != 0x0) {
        /* wait */
    }
    htifstate->env->mfromhost = (val_written >> 48 << 48) | (resp << 16 >> 16);
    htifstate->env->mtohost = 0; /* clear to indicate we read */
    if (htifstate->env->mfromhost != 0) {
        /* raise HTIF interrupt */
        qemu_irq_raise(htifstate->irq);
    }
}

#define TOHOST_OFFSET1 (htifstate->tohost_offset)
#define TOHOST_OFFSET2 (htifstate->tohost_offset + 4)
#define FROMHOST_OFFSET1 (htifstate->fromhost_offset)
#define FROMHOST_OFFSET2 (htifstate->fromhost_offset + 4)

/* CPU wants to read an HTIF register */
static uint64_t htif_mm_read(void *opaque, hwaddr addr, unsigned size)
{
    HTIFState *htifstate = opaque;
    if (addr == TOHOST_OFFSET1) {
        return htifstate->env->mtohost & 0xFFFFFFFF;
    } else if (addr == TOHOST_OFFSET2) {
        return (htifstate->env->mtohost >> 32) & 0xFFFFFFFF;
    } else if (addr == FROMHOST_OFFSET1) {
        return htifstate->env->mfromhost & 0xFFFFFFFF;
    } else if (addr == FROMHOST_OFFSET2) {
        return (htifstate->env->mfromhost >> 32) & 0xFFFFFFFF;
    } else {
        printf("Invalid htif register address %016" PRIx64 "\n",
               (uint64_t)addr);
        exit(1);
    }
}

/* CPU wrote to an HTIF register */
static void htif_mm_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
    HTIFState *htifstate = opaque;
    if (addr == TOHOST_OFFSET1) {
        if (htifstate->env->mtohost == 0x0) {
            htifstate->allow_tohost = 1;
            htifstate->env->mtohost = value & 0xFFFFFFFF;
            if (unlikely(htifstate->tohost_size != 8)) {
#ifdef DEBUG_HTIF
                fprintf(stderr, "Using non-8 htif width\n");
#endif
                /* tests have a zero tohost size in elf symb tab and they
                   use sw to write to mm_write, so TOHOST_OFFSET2 will never
                   be written to. Thus, initiate side effects here. */
                htif_handle_tohost_write(htifstate, htifstate->env->mtohost);
            }
        } else {
            htifstate->allow_tohost = 0;
        }
    } else if (addr == TOHOST_OFFSET2) {
        if (htifstate->allow_tohost) {
            htifstate->env->mtohost |= value << 32;
            htif_handle_tohost_write(htifstate, htifstate->env->mtohost);
        }
    } else if (addr == FROMHOST_OFFSET1) {
        htifstate->fromhost_inprogress = 1;
        htifstate->env->mfromhost = value & 0xFFFFFFFF;
    } else if (addr == FROMHOST_OFFSET2) {
        htifstate->env->mfromhost |= value << 32;
        if (htifstate->env->mfromhost == 0x0) {
            qemu_irq_lower(htifstate->irq);
        }
        htifstate->fromhost_inprogress = 0;
    } else {
        printf("Invalid htif register address %016" PRIx64 "\n",
               (uint64_t)addr);
        exit(1);
    }
}

static const MemoryRegionOps htif_mm_ops[3] = {
    [DEVICE_LITTLE_ENDIAN] = {
        .read = htif_mm_read,
        .write = htif_mm_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
    },
};

HTIFState *htif_mm_init(MemoryRegion *address_space,
           const char *kernel_filename, qemu_irq irq, MemoryRegion *main_mem,
           CPURISCVState *env, CharDriverState *chr)
{
    uint64_t fromhost_addr = 0;
    uint64_t fromhost_size = 0; /* for pk vs tests */
    uint64_t tohost_addr = 0;
    uint64_t tohost_size = 0; /* for pk vs tests */

    /* get fromhost/tohost addresses from the ELF, as spike/fesvr do */
    if (NULL != kernel_filename) {
#if defined(TARGET_RISCV64)
        Elf_obj *e = elf_open(kernel_filename);
#else
        Elf_obj32 *e = elf_open32(kernel_filename);
#endif

        const char *fromhost = "fromhost";
        const char *tohost = "tohost";

#if defined(TARGET_RISCV64)
        Elf64_Sym *curr_sym = elf_firstsym(e);
#else
        Elf32_Sym *curr_sym = elf_firstsym32(e);
#endif
        while (curr_sym) {
#if defined(TARGET_RISCV64)
            char *symname = elf_symname(e, curr_sym);
#else
            char *symname = elf_symname32(e, curr_sym);
#endif

            if (strcmp(fromhost, symname) == 0) {
                /* get fromhost addr */
                fromhost_addr = curr_sym->st_value;
                fromhost_size = curr_sym->st_size; /* this is correctly set to 8
                                                      by pk */
            } else if (strcmp(tohost, symname) == 0) {
                /* get tohost addr */
                tohost_addr = curr_sym->st_value;
                tohost_size = curr_sym->st_size; /* this is correctly set to 8
                                                    by pk */
            }
#if defined(TARGET_RISCV64)
            curr_sym = elf_nextsym(e, curr_sym);
#else
            curr_sym = elf_nextsym32(e, curr_sym);
#endif
        }
    }

    /* now setup HTIF device */
    HTIFState *htifstate;

    htifstate = g_malloc0(sizeof(HTIFState));
    htifstate->irq = irq;
    htifstate->address_space = address_space;
    htifstate->main_mem = main_mem;
    htifstate->main_mem_ram_ptr = memory_region_get_ram_ptr(main_mem);
    htifstate->env = env;
    htifstate->chr = chr;
    htifstate->pending_read = 0;
    htifstate->allow_tohost = 0;
    htifstate->fromhost_inprogress = 0;
    htifstate->fromhost_size = fromhost_size;
    htifstate->tohost_size = tohost_size;

#ifdef ENABLE_CHARDEV
    qemu_chr_add_handlers(htifstate->chr, htif_can_recv, htif_recv, htif_event,
                          htifstate);
#endif

    uint64_t base = tohost_addr < fromhost_addr ? tohost_addr : fromhost_addr;
    uint64_t second = tohost_addr < fromhost_addr ? fromhost_addr : tohost_addr;
    uint64_t regionwidth = second - base + 8;

    htifstate->tohost_offset = base == tohost_addr ? 0 : tohost_addr -
                                                         fromhost_addr;
    htifstate->fromhost_offset = base == fromhost_addr ? 0 : fromhost_addr -
                                                             tohost_addr;

    vmstate_register(NULL, base, &vmstate_htif, htifstate);

    memory_region_init_io(&htifstate->io, NULL,
                          &htif_mm_ops[DEVICE_LITTLE_ENDIAN],
                           htifstate, "htif", regionwidth);
    memory_region_add_subregion(address_space, base, &htifstate->io);

    return htifstate;
}
