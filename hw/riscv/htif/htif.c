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

#include "hw/char/serial.h"
#include "sysemu/char.h"
#include "hw/riscv/htif/htif.h"
#include "hw/riscv/htif/frontend.h"
#include "qemu/timer.h"
#include "exec/address-spaces.h"
#include "qemu/error-report.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <inttypes.h>
#include "elf_symb.h"

#define ENABLE_CHARDEV
//#define DEBUG_CHARDEV
//#define DEBUG_BLKDEV
//#define DEBUG_HTIF


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
        fprintf(stderr, "recv handler: fromhost was not ready to accept input\n");
        fprintf(stderr, "recv handler: prev value was: %016lx\n", htifstate->env->mfromhost);
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
    .fields      = (VMStateField []) { // TODO what
        VMSTATE_UINT64(tohost_offset, HTIFState),
        VMSTATE_UINT64(fromhost_offset, HTIFState),
        VMSTATE_UINT64(tohost_size, HTIFState),
        VMSTATE_UINT64(fromhost_size, HTIFState),
        VMSTATE_END_OF_LIST()
    },
};

static void dma_strcopy(HTIFState *htifstate, char *str, hwaddr phys_addr) {
    int i = 0;
    void* base_copy_addr = htifstate->main_mem_ram_ptr+phys_addr;
    while(*(str+i)) {
        stb_p((void*)(base_copy_addr + i), *(str + i));
        i++;
    }
    stb_p((void*)(base_copy_addr + i), 0); // store null term
}

static int htif_block_device_read(HTIFState *htifstate, uint64_t payload) {
    request_t req;
    int i;
    uint8_t* reqptr = (uint8_t*)&req;
    void *base = htifstate->main_mem_ram_ptr+payload;
    for (i = 0; i < sizeof(req); i++) {
        *(reqptr + i) = ldub_p((void*)(base + i));
    }

    #ifdef DEBUG_BLKDEV
    fprintf(stderr, "HTIF Block device read:\n");
    fprintf(stderr, "-addr: %016lx\n", req.addr);
    fprintf(stderr, "-offset: %016lx\n", req.offset);
    fprintf(stderr, "-size: %016lx\n", req.size);
    fprintf(stderr, "-tag: %016lx\n", req.tag);
    #endif

    uint8_t * copybuf = malloc(req.size * sizeof(uint8_t));
    if (pread(htifstate->block_fd, copybuf, req.size, req.offset) != req.size) {
        printf("FAILED READ\n");
        exit(1);
    }

    base = htifstate->main_mem_ram_ptr + req.addr;

    for (i = 0; i < req.size; i++) {
        stb_p((void*)(base + i), copybuf[i]);
    }
    free(copybuf);
    return req.tag;
}

static int htif_block_device_write(HTIFState *htifstate, uint64_t payload) {
    request_t req;
    int i;
    uint8_t* reqptr = (uint8_t*)&req;
    void* base = htifstate->main_mem_ram_ptr + payload;
    for (i = 0; i < sizeof(req); i++) {
        *(reqptr + i) = ldub_p((void*)(base + i));
    }

    uint8_t * copybuf = malloc(req.size * sizeof(uint8_t));

    base = htifstate->main_mem_ram_ptr + req.addr;
    for (i = 0; i < req.size; i++) {
        copybuf[i] = ldub_p((void*)(base + i));
    }

    if (pwrite(htifstate->block_fd, copybuf, req.size, req.offset) != req.size) {
        printf("FAILED WRITE\n");
        exit(1);
    }

    free(copybuf);
    return req.tag;
}

static void htif_handle_tohost_write(HTIFState *htifstate, uint64_t val_written) {

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

    resp = 0; // stop gcc complaining
    #ifdef DEBUG_HTIF
    fprintf(stderr, "mtohost write:\n-device: %d\n-cmd: %d\n-what: %02lx\n-payload: %016lx\n", device, cmd, payload & 0xFF, payload);
    #endif

    /*
     * Currently, there is a fixed mapping of devices:
     * 0: Syscall Proxy
     * 1: Console
     * 2: Block Device
     */
    if (unlikely(device == 0x0)) {
        // frontend syscall handler
        if (cmd == 0x0) {
            #ifdef DEBUG_HTIF
            fprintf(stderr, "frontend syscall handler\n");
            #endif
            if (payload & 0x1) {
                // test result
                if (payload >> 1) {
                    printf("*** FAILED *** (exitcode = %016lx)\n", payload >> 1);
                } else {
                    printf("TEST PASSED\n");
                }
                exit(payload >> 1);
            }
            resp = handle_frontend_syscall(htifstate, payload);
        } else if (cmd == 0xFF) {
            // use what
            if (what == 0xFF) {
                #ifdef DEBUG_HTIF
                fprintf(stderr, "registering name\n");
                #endif
                dma_strcopy(htifstate, (char*)"syscall_proxy", real_addr);
            } else if (what == 0x0) {
                #ifdef DEBUG_HTIF
                fprintf(stderr, "registering syscall cmd\n");
                #endif
                dma_strcopy(htifstate, (char*)"syscall", real_addr);
            } else {
                #ifdef DEBUG_HTIF
                fprintf(stderr, "registering end of cmds list\n");
                #endif
                dma_strcopy(htifstate, (char*)"", real_addr);
            }
            resp = 0x1; // write to indicate device name placed
        } else {
            fprintf(stderr, "HTIF device %d: UNKNOWN COMMAND\n", device);
            exit(1);
        }
    } else if (likely(device == 0x1)) {
        // HTIF Console
        if (cmd == 0x0) {
            // this should be a queue, but not yet implemented as such
            htifstate->pending_read = val_written;
            htifstate->env->mtohost = 0; // clear to indicate we read
            return;
        } else if (cmd == 0x1) {
            #ifdef ENABLE_CHARDEV
            qemu_chr_fe_write(htifstate->chr, (uint8_t*)&payload, 1);
            #endif
            resp = 0x100 | (uint8_t)payload;
        } else if (cmd == 0xFF) {
            // use what
            if (what == 0xFF) {
                #ifdef DEBUG_HTIF
                fprintf(stderr, "registering name\n");
                #endif
                dma_strcopy(htifstate, (char*)"bcd", real_addr);
            } else if (what == 0x0) {
                #ifdef DEBUG_HTIF
                fprintf(stderr, "registering read cmd\n");
                #endif
                dma_strcopy(htifstate, (char*)"read", real_addr);
            } else if (what == 0x1) {
                #ifdef DEBUG_HTIF
                fprintf(stderr, "registering write cmd\n");
                #endif
                dma_strcopy(htifstate, (char*)"write", real_addr);
            } else {
                #ifdef DEBUG_HTIF
                fprintf(stderr, "registering end of cmds list\n");
                #endif
                dma_strcopy(htifstate, (char*)"", real_addr);
            }
            resp = 0x1; // write to indicate device name placed
        } else {
            fprintf(stderr, "HTIF device %d: UNKNOWN COMMAND\n", device);
            exit(1);
        }
    } else if (device == 0x2 && htifstate->block_dev_present) {
        // HTIF Block Device
        if (unlikely(cmd == 0xFF)) {
            if (what == 0xFF) { // register
                dma_strcopy(htifstate, htifstate->real_name, real_addr);
            } else if (what == 0x0) {
                dma_strcopy(htifstate, (char*)"read", real_addr);
            } else if (what == 0x1) {
                dma_strcopy(htifstate, (char*)"write", real_addr);
            } else {
                dma_strcopy(htifstate, (char*)"", real_addr);
            }
            resp = 0x1; // write to indicate device name placed
        } else if (cmd == 0x0) {
            #ifdef DEBUG_HTIF
            fprintf(stderr, "DOING DISK READ\n");
            #endif
            resp = htif_block_device_read(htifstate, payload);
        } else if (cmd == 0x1) {
            #ifdef DEBUG_HTIF
            fprintf(stderr, "DOING DISK WRITE\n");
            #endif
            resp = htif_block_device_write(htifstate, payload);
        } else {
            fprintf(stderr, "HTIF device %d: UNKNOWN COMMAND\n", device);
            exit(1);
        }
    } else if (device == 0x3 && cmd == 0xFF && what == 0xFF) { // all other devices
        #ifdef DEBUG_HTIF
        fprintf(stderr, "registering no device as last\n");
        #endif
        stb_p((void*)(htifstate->main_mem_ram_ptr+real_addr), 0);
        resp = 0x1; // write to indicate device name placed
    } else {
        fprintf(stderr, "HTIF UNKNOWN DEVICE OR COMMAND!\n");
        fprintf(stderr, "-device: %d\n-cmd: %d\n-what: %02lx\n-payload: %016lx\n", device, cmd, payload & 0xFF, payload);
        exit(1);
    }
    while (!htifstate->fromhost_inprogress && htifstate->env->mfromhost != 0x0) {
        // wait
    }
    htifstate->env->mfromhost = (val_written >> 48 << 48) | (resp << 16 >> 16);
    htifstate->env->mtohost = 0; // clear to indicate we read
    if (htifstate->env->mfromhost != 0) {
        // raise HTIF interrupt
        qemu_irq_raise(htifstate->irq);
    }
}

#define TOHOST_OFFSET1 htifstate->tohost_offset
#define TOHOST_OFFSET2 (htifstate->tohost_offset+4)
#define FROMHOST_OFFSET1 htifstate->fromhost_offset
#define FROMHOST_OFFSET2 (htifstate->fromhost_offset+4)

// CPU wants to read an HTIF register
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
        printf("Invalid htif register address %016lx\n", (uint64_t)addr);
        exit(1);
    }
}

// CPU wrote to an HTIF register
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
                // tests have a zero tohost size in elf symb tab and they 
                // use sw to write to mm_write, so TOHOST_OFFSET2 will never
                // be written to. Thus, initiate side effects here.
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
        printf("Invalid htif register address %016lx\n", (uint64_t)addr);
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

HTIFState *htif_mm_init(MemoryRegion *address_space, const char* kernel_filename, 
           qemu_irq irq, MemoryRegion *main_mem, const char* htifbd_fname,
           const char *kernel_cmdline, CPURISCVState *env, CharDriverState *chr)
{
    // get fromhost/tohost addresses from the ELF, as spike/fesvr do
    Elf_obj * e = elf_open(kernel_filename);
    const char * fromhost = "fromhost";
    const char * tohost = "tohost";
    uint64_t fromhost_addr = 0;
    uint64_t fromhost_size = 0; // for pk vs tests
    uint64_t tohost_addr = 0;
    uint64_t tohost_size = 0; // for pk vs tests
    
    Elf64_Sym * curr_sym = elf_firstsym(e);
    while (curr_sym) {
        char * symname = elf_symname(e, curr_sym);
        if (strcmp(fromhost, symname) == 0) {
            // get fromhost addr
            fromhost_addr = curr_sym->st_value;
            fromhost_size = curr_sym->st_size; // this is correctly set to 8 by pk
        } else if (strcmp(tohost, symname) == 0) {
            // get tohost addr
            tohost_addr = curr_sym->st_value;
            tohost_size = curr_sym->st_size; // this is correctly set to 8 by pk
        }
        curr_sym = elf_nextsym(e, curr_sym);
    }

    // now setup HTIF device
    // TODO: cleanup the constant buffer sizes
    HTIFState *htifstate;
    size_t size;
    char *rname;
    char size_str_buf[400];

    htifstate = g_malloc0(sizeof(HTIFState));
    rname = g_malloc0(sizeof(char)*500);
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
    qemu_chr_add_handlers(htifstate->chr, htif_can_recv, htif_recv, htif_event, htifstate);
#endif

    uint64_t base = tohost_addr < fromhost_addr ? tohost_addr : fromhost_addr;
    uint64_t second = tohost_addr < fromhost_addr ? fromhost_addr : tohost_addr;
    uint64_t regionwidth = second - base + 8;

    htifstate->tohost_offset = base == tohost_addr ? 0 : tohost_addr - fromhost_addr;
    htifstate->fromhost_offset = base == fromhost_addr ? 0 : fromhost_addr - tohost_addr;

    vmstate_register(NULL, base, &vmstate_htif, htifstate);

    memory_region_init_io(&htifstate->io, NULL, &htif_mm_ops[DEVICE_LITTLE_ENDIAN],
            htifstate, "htif", regionwidth);
    memory_region_add_subregion(address_space, base, &htifstate->io);

    // save kernel_cmdline for sys_getmainvars
    htifstate->kernel_cmdline = malloc(strlen(kernel_cmdline)+1);
    strcpy(htifstate->kernel_cmdline, kernel_cmdline);

    if (NULL == htifbd_fname) { // NULL means no -hda specified
        htifstate->block_dev_present = 0;
        return htifstate;
    }

    htifstate->block_fname = htifbd_fname;
    htifstate->block_fd = open(htifstate->block_fname, O_RDWR);

    struct stat st;
    if (fstat(htifstate->block_fd, &st) < 0) {
        fprintf(stderr, "WARN: Could not stat %s, continuing without block device.\n",
                htifstate->block_fname);
        htifstate->block_dev_present = 0;
        return htifstate;
    }
    size = st.st_size;
    strcpy(rname, "disk size=");
    snprintf(size_str_buf, sizeof(size_str_buf), "%zu", size);
    strcat(rname, size_str_buf);
    htifstate->real_name = rname;
    htifstate->block_dev_present = 1;
    return htifstate;
}
