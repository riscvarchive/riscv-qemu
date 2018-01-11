/*
 * QEMU RISC-V Host Target Interface (HTIF) Emulation
 *
 * Author: Sagar Karandikar, sagark@eecs.berkeley.edu
 *
 * This provides HTIF device emulation for QEMU. At the moment this allows
 * for identical copies of bbl/linux to run on both spike and QEMU.
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
#include "qapi/error.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "hw/char/serial.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include "hw/riscv/riscv_htif.h"
#include "qemu/timer.h"
#include "exec/address-spaces.h"
#include "qemu/error-report.h"
#include "hw/riscv/riscv_elf.h"

#define ENABLE_CHARDEV

#define RISCV_DEBUG_HTIF 0
#define HTIF_DEBUG(fmt, ...)                                               \
    do {                                                                   \
        if (RISCV_DEBUG_HTIF) {                                            \
            qemu_log_mask(LOG_TRACE, "%s: " fmt, __func__, ##__VA_ARGS__); \
        }                                                                  \
    } while (0)

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
    HTIFState *htifstate = opaque;

    if (size != 1) {
        return;
    }

    /* TODO - we need to check whether mfromhost is zero which indicates
              the device is ready to receive. The current implementation
              will drop characters */

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

static int htif_be_change(void *opaque)
{
    HTIFState *s = opaque;

    qemu_chr_fe_set_handlers(&s->chr, htif_can_recv, htif_recv, htif_event,
        htif_be_change, s, NULL, true);

    return 0;
}
#endif

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
    HTIF_DEBUG("TOHOST WRITE WITH val 0x%016" PRIx64, val_written);

    uint8_t device = val_written >> 56;
    uint8_t cmd = val_written >> 48;
    uint64_t payload = val_written & 0xFFFFFFFFFFFFULL;

    uint64_t addr = payload >> 8;
    hwaddr real_addr = (hwaddr)addr;
    uint8_t what = payload & 0xFF;
    int resp;

    resp = 0; /* stop gcc complaining */
    HTIF_DEBUG("mtohost write: device: %d cmd: %d what: %02" PRIx64
        " -payload: %016" PRIx64, device, cmd, payload & 0xFF, payload);

    /*
     * Currently, there is a fixed mapping of devices:
     * 0: riscv-tests Pass/Fail Reporting Only (no syscall proxy)
     * 1: Console
     */
    if (unlikely(device == 0x0)) {
        /* frontend syscall handler, only test pass/fail support */
        if (cmd == 0x0) {
            HTIF_DEBUG("frontend syscall handler");
            if (payload & 0x1) {
                /* exit code */
                int exit_code = payload >> 1;
                if (exit_code) {
                    qemu_log("*** FAILED *** (tohost = %d)", exit_code);
                }
                exit(exit_code);
            }
            qemu_log_mask(LOG_UNIMP, "pk syscall proxy not supported");
        } else if (cmd == 0xFF) {
            /* use what */
            if (what == 0xFF) {
                HTIF_DEBUG("registering name");
                dma_strcopy(htifstate, (char *)"syscall_proxy", real_addr);
            } else if (what == 0x0) {
                HTIF_DEBUG("registering syscall cmd");
                dma_strcopy(htifstate, (char *)"syscall", real_addr);
            } else {
                HTIF_DEBUG("registering end of cmds list");
                dma_strcopy(htifstate, (char *)"", real_addr);
            }
            resp = 0x1; /* write to indicate device name placed */
        } else {
            qemu_log("HTIF device %d: UNKNOWN COMMAND", device);
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
            qemu_chr_fe_write(&htifstate->chr, (uint8_t *)&payload, 1);
            #endif
            resp = 0x100 | (uint8_t)payload;
        } else if (cmd == 0xFF) {
            /* use what */
            if (what == 0xFF) {
                HTIF_DEBUG("registering name");
                dma_strcopy(htifstate, (char *)"bcd", real_addr);
            } else if (what == 0x0) {
                HTIF_DEBUG("registering read cmd");
                dma_strcopy(htifstate, (char *)"read", real_addr);
            } else if (what == 0x1) {
                HTIF_DEBUG("registering write cmd");
                dma_strcopy(htifstate, (char *)"write", real_addr);
            } else {
                HTIF_DEBUG("registering end of cmds list");
                dma_strcopy(htifstate, (char *)"", real_addr);
            }
            resp = 0x1; /* write to indicate device name placed */
        } else {
            qemu_log("HTIF device %d: UNKNOWN COMMAND", device);
        }
    /* all other devices */
    } else if (device == 0x2 && cmd == 0xFF && what == 0xFF) {
        HTIF_DEBUG("registering no device as last");
        stb_p((void *)(htifstate->main_mem_ram_ptr + real_addr), 0);
        resp = 0x1; /* write to indicate device name placed */
    } else {
        qemu_log("HTIF UNKNOWN DEVICE OR COMMAND");
        HTIF_DEBUG("device: %d cmd: %d what: %02" PRIx64
            " payload: %016" PRIx64, device, cmd, payload & 0xFF, payload);
    }
    /*
     * - latest bbl does not set fromhost to 0 if there is a value in tohost
     * - with this code enabled, qemu hangs waiting for fromhost to go to 0
     * - with this code disabled, qemu works with bbl priv v1.9.1 and v1.10
     * - HTIF needs protocol documentation and a more complete state machine

        while (!htifstate->fromhost_inprogress &&
            htifstate->env->mfromhost != 0x0) {
        }
    */
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
        qemu_log("Invalid htif read: address %016" PRIx64,
            (uint64_t)addr);
        return 0;
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
        qemu_log("Invalid htif write: address %016" PRIx64,
            (uint64_t)addr);
    }
}

static const MemoryRegionOps htif_mm_ops = {
    .read = htif_mm_read,
    .write = htif_mm_write,
};

HTIFState *htif_mm_init(MemoryRegion *address_space,
    const char *kernel_filename, qemu_irq irq, MemoryRegion *main_mem,
    CPURISCVState *env, Chardev *chr)
{
    uint64_t fromhost_addr = 0, tohost_addr = 0;

    /* get fromhost/tohost addresses from the ELF, as spike/fesvr do */
    if (kernel_filename) {
#if defined(TARGET_RISCV64)
        Elf_obj64 *e = elf_open64(kernel_filename);
#else
        Elf_obj32 *e = elf_open32(kernel_filename);
#endif

        const char *fromhost = "fromhost";
        const char *tohost = "tohost";

#if defined(TARGET_RISCV64)
        Elf64_Sym *curr_sym = elf_firstsym64(e);
#else
        Elf32_Sym *curr_sym = elf_firstsym32(e);
#endif
        while (curr_sym) {
#if defined(TARGET_RISCV64)
            char *symname = elf_symname64(e, curr_sym);
#else
            char *symname = elf_symname32(e, curr_sym);
#endif

            if (strcmp(fromhost, symname) == 0) {
                /* get fromhost addr */
                fromhost_addr = curr_sym->st_value;
                if (curr_sym->st_size != 8) {
                    error_report("HTIF fromhost must be 8 bytes");
                    exit(1);
                }
            } else if (strcmp(tohost, symname) == 0) {
                /* get tohost addr */
                tohost_addr = curr_sym->st_value;
                if (curr_sym->st_size != 8) {
                    error_report("HTIF tohost must be 8 bytes");
                    exit(1);
                }
            }
#if defined(TARGET_RISCV64)
            curr_sym = elf_nextsym64(e, curr_sym);
#else
            curr_sym = elf_nextsym32(e, curr_sym);
#endif
        }

#if defined(TARGET_RISCV64)
        elf_close64(e);
#else
        elf_close32(e);
#endif
    }

    uint64_t base = MIN(tohost_addr, fromhost_addr);
    uint64_t size = MAX(tohost_addr + 8, fromhost_addr + 8) - base;
    uint64_t tohost_offset = tohost_addr - base;
    uint64_t fromhost_offset = fromhost_addr - base;

    HTIFState *s = g_malloc0(sizeof(HTIFState));
    s->irq = irq;
    s->address_space = address_space;
    s->main_mem = main_mem;
    s->main_mem_ram_ptr = memory_region_get_ram_ptr(main_mem);
    s->env = env;
    s->tohost_offset = tohost_offset;
    s->fromhost_offset = fromhost_offset;
    s->pending_read = 0;
    s->allow_tohost = 0;
    s->fromhost_inprogress = 0;
#ifdef ENABLE_CHARDEV
    qemu_chr_fe_init(&s->chr, chr, &error_abort);
    qemu_chr_fe_set_handlers(&s->chr, htif_can_recv, htif_recv, htif_event,
        htif_be_change, s, NULL, true);
#endif
    if (base) {
        memory_region_init_io(&s->mmio, NULL, &htif_mm_ops, s,
                            TYPE_HTIF_UART, size);
        memory_region_add_subregion(address_space, base, &s->mmio);
    }

    return s;
}
