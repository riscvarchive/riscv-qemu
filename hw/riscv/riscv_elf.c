/*
 * elf.c - A simple package for manipulating symbol tables in elf binaries.
 *
 * Taken from
 * https://www.cs.cmu.edu/afs/cs.cmu.edu/academic/class/15213-f03/www/
 * ftrace/elf.c
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <errno.h>
#include <fcntl.h>
#include <glib.h>

#include "hw/riscv/riscv_elf.h"

/*
 * elf_open - Map a binary into the address space and extract the
 * locations of the static and dynamic symbol tables and their string
 * tables. Return this information in a Elf object file handle that will
 * be passed to all of the other elf functions.
 */
Elf_obj64 *elf_open64(const char *filename)
{
    int i, fd;
    struct stat sbuf;
    Elf_obj64 *ep;
    Elf64_Shdr *shdr;

    ep = g_new(Elf_obj64, 1);

    /* Do some consistency checks on the binary */
    fd = open(filename, O_RDONLY);
    if (fd == -1) {
        fprintf(stderr, "Can't open \"%s\": %s\n", filename, strerror(errno));
        exit(1);
    }
    if (fstat(fd, &sbuf) == -1) {
        fprintf(stderr, "Can't stat \"%s\": %s\n", filename, strerror(errno));
        exit(1);
    }
    if (sbuf.st_size < sizeof(Elf64_Ehdr)) {
        fprintf(stderr, "\"%s\" is not an ELF binary object\n", filename);
        exit(1);
    }

    /* It looks OK, so map the Elf binary into our address space */
    ep->mlen = sbuf.st_size;
    ep->maddr = mmap(NULL, ep->mlen, PROT_READ, MAP_SHARED, fd, 0);
    if (ep->maddr == (void *)-1) {
        fprintf(stderr, "Can't mmap \"%s\": %s\n", filename, strerror(errno));
        exit(1);
    }
    close(fd);

    /* The Elf binary begins with the Elf header */
    ep->ehdr = ep->maddr;

    /* check we have a 64-bit little-endian RISC-V ELF object */
    if (ep->ehdr->e_ident[EI_MAG0] != ELFMAG0 ||
        ep->ehdr->e_ident[EI_MAG1] != ELFMAG1 ||
        ep->ehdr->e_ident[EI_MAG2] != ELFMAG2 ||
        ep->ehdr->e_ident[EI_MAG3] != ELFMAG3 ||
        ep->ehdr->e_ident[EI_CLASS] != ELFCLASS64 ||
        ep->ehdr->e_ident[EI_DATA] != ELFDATA2LSB ||
        ep->ehdr->e_machine != EM_RISCV)
    {
        fprintf(stderr, "\"%s\" is not a 64-bit RISC-V ELF object\n", filename);
        exit(1);
    }

    /*
     * Find the static and dynamic symbol tables and their string
     * tables in the the mapped binary. The sh_link field in symbol
     * table section headers gives the section index of the string
     * table for that symbol table.
     */
    shdr = (Elf64_Shdr *)(ep->maddr + ep->ehdr->e_shoff);
    for (i = 0; i < ep->ehdr->e_shnum; i++) {
        if (shdr[i].sh_type == SHT_SYMTAB) {   /* Static symbol table */
            ep->symtab = (Elf64_Sym *)(ep->maddr + shdr[i].sh_offset);
            ep->symtab_end = (Elf64_Sym *)((char *)ep->symtab +
                             shdr[i].sh_size);
            ep->strtab = (char *)(ep->maddr + shdr[shdr[i].sh_link].sh_offset);
        }
        if (shdr[i].sh_type == SHT_DYNSYM) {   /* Dynamic symbol table */
            ep->dsymtab = (Elf64_Sym *)(ep->maddr + shdr[i].sh_offset);
            ep->dsymtab_end = (Elf64_Sym *)((char *)ep->dsymtab +
                              shdr[i].sh_size);
            ep->dstrtab = (char *)(ep->maddr + shdr[shdr[i].sh_link].sh_offset);
        }
    }
    return ep;
}

Elf_obj32 *elf_open32(const char *filename)
{
    int i, fd;
    struct stat sbuf;
    Elf_obj32 *ep;
    Elf32_Shdr *shdr;

    ep = g_new(Elf_obj32, 1);

    /* Do some consistency checks on the binary */
    fd = open(filename, O_RDONLY);
    if (fd == -1) {
        fprintf(stderr, "Can't open \"%s\": %s\n", filename, strerror(errno));
        exit(1);
    }
    if (fstat(fd, &sbuf) == -1) {
        fprintf(stderr, "Can't stat \"%s\": %s\n", filename, strerror(errno));
        exit(1);
    }
    if (sbuf.st_size < sizeof(Elf32_Ehdr)) {
        fprintf(stderr, "\"%s\" is not an ELF binary object\n", filename);
        exit(1);
    }

    /* It looks OK, so map the Elf binary into our address space */
    ep->mlen = sbuf.st_size;
    ep->maddr = mmap(NULL, ep->mlen, PROT_READ, MAP_SHARED, fd, 0);
    if (ep->maddr == (void *)-1) {
        fprintf(stderr, "Can't mmap \"%s\": %s\n", filename, strerror(errno));
        exit(1);
    }
    close(fd);

    /* The Elf binary begins with the Elf header */
    ep->ehdr = ep->maddr;

    /* check we have a 32-bit little-endian RISC-V ELF object */
    if (ep->ehdr->e_ident[EI_MAG0] != ELFMAG0 ||
        ep->ehdr->e_ident[EI_MAG1] != ELFMAG1 ||
        ep->ehdr->e_ident[EI_MAG2] != ELFMAG2 ||
        ep->ehdr->e_ident[EI_MAG3] != ELFMAG3 ||
        ep->ehdr->e_ident[EI_CLASS] != ELFCLASS32 ||
        ep->ehdr->e_ident[EI_DATA] != ELFDATA2LSB ||
        ep->ehdr->e_machine != EM_RISCV)
    {
        fprintf(stderr, "\"%s\" is not a 32-bit RISC-V ELF object\n", filename);
        exit(1);
    }

    /*
     * Find the static and dynamic symbol tables and their string
     * tables in the the mapped binary. The sh_link field in symbol
     * table section headers gives the section index of the string
     * table for that symbol table.
     */
    shdr = (Elf32_Shdr *)(ep->maddr + ep->ehdr->e_shoff);
    for (i = 0; i < ep->ehdr->e_shnum; i++) {
        if (shdr[i].sh_type == SHT_SYMTAB) {   /* Static symbol table */
            ep->symtab = (Elf32_Sym *)(ep->maddr + shdr[i].sh_offset);
            ep->symtab_end = (Elf32_Sym *)((char *)ep->symtab +
                             shdr[i].sh_size);
            ep->strtab = (char *)(ep->maddr + shdr[shdr[i].sh_link].sh_offset);
        }
        if (shdr[i].sh_type == SHT_DYNSYM) {   /* Dynamic symbol table */
            ep->dsymtab = (Elf32_Sym *)(ep->maddr + shdr[i].sh_offset);
            ep->dsymtab_end = (Elf32_Sym *)((char *)ep->dsymtab +
                              shdr[i].sh_size);
            ep->dstrtab = (char *)(ep->maddr + shdr[shdr[i].sh_link].sh_offset);
        }
    }
    return ep;
}

/*
 * elf_close - Free up the resources of an  elf object
 */
void elf_close64(Elf_obj64 *ep)
{
    if (munmap(ep->maddr, ep->mlen) < 0) {
        perror("munmap");
        exit(1);
    }
    free(ep);
}

void elf_close32(Elf_obj32 *ep)
{
    if (munmap(ep->maddr, ep->mlen) < 0) {
        perror("munmap");
        exit(1);
    }
    free(ep);
}

/*
 * elf_symname - Return ASCII name of a static symbol
 */
char *elf_symname64(Elf_obj64 *ep, Elf64_Sym *sym)
{
    return &ep->strtab[sym->st_name];
}

char *elf_symname32(Elf_obj32 *ep, Elf32_Sym *sym)
{
    return &ep->strtab[sym->st_name];
}

/*
 * elf_firstsym - Return ptr to first symbol in static symbol table
 */
Elf64_Sym *elf_firstsym64(Elf_obj64 *ep)
{
    return ep->symtab;
}

Elf32_Sym *elf_firstsym32(Elf_obj32 *ep)
{
    return ep->symtab;
}


/*
 * elf_nextsym - Return ptr to next symbol in static symbol table,
 * or NULL if no more symbols.
 */
Elf64_Sym *elf_nextsym64(Elf_obj64 *ep, Elf64_Sym *sym)
{
    sym++;
    if (sym < ep->symtab_end) {
        return sym;
    } else {
        return NULL;
    }
}

Elf32_Sym *elf_nextsym32(Elf_obj32 *ep, Elf32_Sym *sym)
{
    sym++;
    if (sym < ep->symtab_end) {
        return sym;
    } else {
        return NULL;
    }
}
