riscv-qemu [![Build Status](https://travis-ci.org/ucb-bar/riscv-qemu.svg?branch=master)](https://travis-ci.org/ucb-bar/riscv-qemu)
=========

The riscv-softmmu target for full system emulation is currently supported. 
It supports booting [riscv-linux] \(currently requires building from the 
[qemu branch]\). A precompiled copy of the kernel is included in the 
"hacking_files" directory for convenience (see Method 1 under installation).

Prereqs:

    $ sudo apt-get install gcc libc6-dev pkg-config bridge-utils uml-utilities zlib1g-dev libglib2.0-dev autoconf automake libtool libsdl1.2-dev

Installation 
--------------

### Method 1 \(Devices: 8250 UART + HTIF Disk\): 

####Step 1: Build QEMU

    $ git clone https://github.com/ucb-bar/riscv-qemu
    $ cd riscv-qemu
    $ git submodule update --init pixman
    $ ./configure --target-list=riscv-softmmu [--prefix=INSTALL_LOCATION]
    $ make
    $ [make install] # if you supplied prefix above
    $ cd riscv-softmmu

####Step 2: Obtain Images

To get started, you may download the following kernel image and disk images from
the [RISC-V Getting Started Guide](http://riscv.org/getting-started.html).

**a)** [vmlinux](http://riscv.org/qemu/vmlinux)

**b)** [root.bin](http://riscv.org/qemu/root.bin)


####Step 3: Run QEMU

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
