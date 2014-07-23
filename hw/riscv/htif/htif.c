/*
 *  QEMU RISC-V Host Target Interface (HTIF) Emulation
 *
 *  Author: Sagar Karandikar, skarandikar@berkeley.edu
 *
 * Since directly accessing CPU registers is not ideal, we map reads and writes 
 * to the tohost/fromhost registers onto memory addresses 0x400 and 0x408 
 * respectively (see the csr instructions in target-riscv/translate.c).
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

#include "hw/riscv/htif/htif.h"
#include "qemu/timer.h"
#include "exec/address-spaces.h"
#include "qemu/error-report.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

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
        VMSTATE_UINT64(tohost, HTIFState),
        VMSTATE_UINT64(fromhost, HTIFState),
        VMSTATE_UINT64(tohost_addr, HTIFState),
        VMSTATE_UINT64(fromhost_addr, HTIFState),
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
        // TODO: potential endianness issues here
        *(reqptr + i) = ldub_p((void*)(base + i));
    }

    uint8_t copybuf[req.size];
    if (pread(htifstate->block_fd, copybuf, req.size, req.offset) != req.size) {
        printf("FAILED READ\n");
        exit(1);
    }

    base = htifstate->main_mem_ram_ptr + req.addr;
    for (i = 0; i < req.size; i++) {
        stb_p((void*)(base + i), copybuf[i]);
    }
    return req.tag;
}

static int htif_block_device_write(HTIFState *htifstate, uint64_t payload) {
    request_t req;
    int i;
    uint8_t* reqptr = (uint8_t*)&req;
    void* base = htifstate->main_mem_ram_ptr + payload;
    for (i = 0; i < sizeof(req); i++) {
        // TODO: potential endianness issues here
        *(reqptr + i) = ldub_p((void*)(base + i));
    }

    uint8_t copybuf[req.size];

    base = htifstate->main_mem_ram_ptr + req.addr;
    for (i = 0; i < req.size; i++) {
        copybuf[i] = ldub_p((void*)(base + i));
    }

    if (pwrite(htifstate->block_fd, copybuf, req.size, req.offset) != req.size) {
        printf("FAILED WRITE\n");
        exit(1);
    }
    return req.tag;
}

static void htif_handle_tohost_write(HTIFState *htifstate, uint64_t val_written) {

    uint8_t device = val_written >> 56;
    uint8_t cmd = val_written >> 48;
    uint64_t payload = val_written & 0xFFFFFFFFFFFFULL;

    uint64_t addr = payload >> 8;
    hwaddr real_addr = (hwaddr)addr;
    uint8_t what = payload & 0xFF;
    int resp;

    resp = 0; // stop gcc complaining

    if (likely(device == 0x1 && htifstate->block_dev_present)) { 
        // assume device 0x1 is permanently hooked to block dev for now
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
            // handle disk read
            resp = htif_block_device_read(htifstate, payload);
        } else if (cmd == 0x1) {
            // handle disk write
            resp = htif_block_device_write(htifstate, payload);
        } else {
            printf("INVALID HTIFBD COMMAND. exiting\n");
            exit(1);
        }
    } else if (cmd == 0xFF && what == 0xFF) { // all other devices
        stb_p((void*)(htifstate->main_mem_ram_ptr+real_addr), 0);
        resp = 0x1; // write to indicate device name placed
    }
    htifstate->fromhost = (val_written >> 48 << 48) | (resp << 16 >> 16);
    htifstate->tohost = 0; // clear to indicate we read
}

// CPU wants to read an HTIF register
static uint64_t htif_mm_read(void *opaque, hwaddr addr, unsigned size)
{
    HTIFState *htifstate = opaque;
    if (addr == 0x0) {
        return htifstate->tohost & 0xFFFFFFFF;
    } else if (addr == 0x4) {
        return (htifstate->tohost >> 32) & 0xFFFFFFFF;
    } else if (addr == 0x8) {
        return htifstate->fromhost & 0xFFFFFFFF;
    } else if (addr == 0xc) {
        return (htifstate->fromhost >> 32) & 0xFFFFFFFF;
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
    if (addr == 0x0) {
        htifstate->tohost = value & 0xFFFFFFFF;
    } else if (addr == 0x4) {
        htif_handle_tohost_write(htifstate, htifstate->tohost | (value << 32));
    } else if (addr == 0x8) {
        htifstate->fromhost = value & 0xFFFFFFFF;
    } else if (addr == 0xc) {
        htifstate->fromhost |= value << 32;
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

HTIFState *htif_mm_init(MemoryRegion *address_space, hwaddr base, qemu_irq irq, 
                        MemoryRegion *main_mem, char* htifbd_fname)
{
    // TODO: cleanup the constant buffer sizes
    HTIFState *htifstate;
    size_t size;
    char *rname;
    char size_str_buf[400];

    htifstate = g_malloc0(sizeof(HTIFState));
    rname = g_malloc0(sizeof(char)*500);
    htifstate->tohost = 0;
    htifstate->fromhost = 0;
    htifstate->tohost_addr = base;
    htifstate->fromhost_addr = base + 0x8;
    htifstate->irq = irq;
    htifstate->address_space = address_space;
    htifstate->main_mem = main_mem;
    htifstate->main_mem_ram_ptr = memory_region_get_ram_ptr(main_mem);

    vmstate_register(NULL, base, &vmstate_htif, htifstate);

    memory_region_init_io(&htifstate->io, NULL, &htif_mm_ops[DEVICE_LITTLE_ENDIAN], 
            htifstate, "htif", 16 /* 2 64-bit registers */);
    memory_region_add_subregion(address_space, base, &htifstate->io);

    if (NULL == htifbd_fname) { // NULL means no -hda specified
        htifstate->block_dev_present = 0;
        return htifstate;
    }

    htifstate->block_fname = htifbd_fname;
    htifstate->block_fd = open(htifstate->block_fname, O_RDWR);

    struct stat st;
    if (fstat(htifstate->block_fd, &st) < 0) {
        printf("WARN: Could not stat %s, continuing without block device.\n", 
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
