riscv-qemu [![Build Status](https://travis-ci.org/ucb-bar/riscv-qemu.svg?branch=master)](https://travis-ci.org/ucb-bar/riscv-qemu)
=========

The riscv-softmmu target for full system emulation is currently supported. 
It supports booting [riscv-linux] \(currently requires building from the 
[qemu branch]\). A precompiled copy of the kernel is included in the 
"hacking_files" directory for convenience (see Method 1 under installation).

Installation 
--------------

### Method 1 \(the quick way\):

A sample kernel with an initramfs is included in the "hacking_files"
directory. You can easily test out riscv-qemu this way:

    $ git clone https://github.com/ucb-bar/riscv-qemu
    $ cd riscv-qemu
    $ git submodule update --init pixman
    $ ./configure --target-list=riscv-softmmu
    $ make
    $ cd riscv-softmmu
    $ # now, start qemu
    $ ./qemu-system-riscv -kernel ../hacking_files/vmlinux/vmlinux -nographic

To exit this system, hit `ctrl-a x`.

### Method 2 \(system with persistent storage\): 

Booting from a block device is also supported. A more extensive guide for 
configuring the kernel/building a root fs will be available soon.

####Step 1:

    $ git clone https://github.com/ucb-bar/riscv-qemu
    $ cd riscv-qemu
    $ git submodule update --init pixman
    $ ./configure --target-list=riscv-softmmu
    $ make
    $ cd riscv-softmmu

####Step 2:

Instructions for the following two steps are coming soon:

**a)** Build linux kernel from the qemu branch of riscv-linux with htif block device support.

**b)** Build the `root.bin` root filesystem.


####Step 3:

Now from the `riscv-softmmu/` directory, start `qemu-system-riscv`:

    $ ./qemu-system-riscv -hda [your root.bin location] -kernel [your vmlinux location] -nographic

**IMPORTANT**: To cleanly exit this system, you must enter `halt -f` at the prompt
and then hit `ctrl-a x`. Otherwise, the root filesystem will likely be corrupted.

Notes
-----

- Qemu also supports a "linux-user" mode, however this is currently not implemented for RISC-V.
- Once in a while, you may see a message from qemu of the form `main-loop: WARNING: I/O thread spun for N iterations`. You may safely ignore this message without consequence.
- Files/directories of interest:
  - target-riscv/
  - hw/riscv/

[riscv-linux]:https://github.com/ucb-bar/riscv-linux
[qemu branch]:https://github.com/ucb-bar/riscv-linux/tree/qemu
