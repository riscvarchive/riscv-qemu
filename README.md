riscv-qemu [![Build Status](https://travis-ci.org/ucb-bar/riscv-qemu.svg?branch=master)](https://travis-ci.org/ucb-bar/riscv-qemu)
=========

The riscv-softmmu target for full system emulation is currently supported. 
It supports booting [riscv-linux] \(currently requires building from the 
[qemu branch]\). A precompiled copy of the kernel is included in the "hacking" 
directory for convenience (see Method 1 under installation).

Installation 
--------------

### Method 1 \(the quick way\):

A sample kernel with an initramfs is included in the "hacking"
directory. You can easily test out riscv-qemu this way:

```sh
$ git clone git@github.com:ucb-bar/riscv-qemu.git
$ cd riscv-qemu
$ git submodule update --init pixman
$ ./configure --target-list=riscv-softmmu
$ make
$ cd riscv-softmmu
$ # now, start qemu
$ ./qemu-system-riscv -kernel ../hacking/vmlinux/vmlinux -nographic
```

To exit this system, hit `ctrl-a x`.

### Method 2 \(system with persistent storage\): 

Preliminary block device support is also present in riscv-qemu. A
more extensive guide for configuring the kernel will be available soon.

####Step 1:

```sh
$ git clone git@github.com:ucb-bar/riscv-qemu.git
$ cd riscv-qemu
$ git submodule update --init pixman
$ ./configure --target-list=riscv-softmmu
$ make
$ cd riscv-softmmu
```

####Step 2:

Instructions for the following two steps are coming soon:

**a)** Build linux kernel from the qemu branch of riscv-linux with htif block device support.

**b)** Build the `root.bin` root filesystem.

You should place both of these files (`vmlinux`, `root.bin`) in the riscv-softmmu directory. Due to a current limitation of riscv-qemu, only a file named `QEMU_DIR/riscv-softmmu/root.bin` is mounted. 

####Step 3:

```sh
$ # now from the riscv-softmmu/ directory, start qemu
$ ./qemu-system-riscv -kernel vmlinux -nographic
```

**IMPORTANT**: To cleanly exit this system, you must enter `halt -f` at the prompt
and then hit `ctrl-a x`. Otherwise, the root filesystem will likely be corrupted.


Notes
-----

- Qemu also supports a "linux-user" mode, however this is currently not implemented for RISC-V.
- Files/directories of interest:
  - target-riscv/
  - hw/riscv/

[riscv-linux]:https://github.com/ucb-bar/riscv-linux
[qemu branch]:https://github.com/ucb-bar/riscv-linux/tree/qemu
