riscv-qemu [![Build Status](https://travis-ci.org/riscv/riscv-qemu.svg?branch=master)](https://travis-ci.org/riscv/riscv-qemu)
=========

**About:**

The `riscv-softmmu` target for full system RV64G emulation is currently supported.
It supports booting Linux from the `priv-1.9` branch of [riscv-linux] and 
passes the compatibility tests from [riscv-tests].

**RISC-V Port Authors:**

* Sagar Karandikar (sagark@eecs.berkeley.edu)

**Upstream QEMU Version:**

* v2.5.0

**Privileged Specification Version:**

This version of QEMU adheres to the RISC-V v1.9 Privileged Specification as 
described in [Technical Report No. UCB/EECS-2016-129](https://www2.eecs.berkeley.edu/Pubs/TechRpts/2016/EECS-2016-129.pdf) and commit 65da94f84a2ba5a61a8bcf3ebdd8ca57f6d899ca of riscv-tools.

Installation
--------------

Prerequisites:

    $ sudo apt-get install gcc libc6-dev pkg-config bridge-utils uml-utilities zlib1g-dev libglib2.0-dev autoconf automake libtool libsdl1.2-dev

### Method 1 \(HTIF Devices\):

####Step 1: Build QEMU

    $ git clone https://github.com/riscv/riscv-qemu
    $ cd riscv-qemu
    $ git submodule update --init pixman
    $ ./configure --target-list=riscv-softmmu [--prefix=INSTALL_LOCATION]
    $ make
    $ [make install] # if you supplied prefix above

####Step 2: Obtain Images

You can build `vmlinux` from the `priv-1.9` branch of the [riscv-linux] repo and 
create an initramfs for your root filesystem, then supply the resulting vmlinux
as a payload for bbl. Alternatively, you can use the prebuilt copy linked 
below. This single file contains bbl with the Linux kernel as a payload. The
included copy of the Linux kernel also has an initramfs with busybox.

**a)** [bblvmlinuxinitramfs_dynamic](https://www.eecs.berkeley.edu/~skarandikar/host/qemu/1.9/bblvmlinuxinitramfs_dynamic)

####Step 3: Run QEMU

To boot Linux (assuming you are in the `riscv-qemu` directory):

    $ ./riscv-softmmu/qemu-system-riscv -kernel bblvmlinuxinitramfs_dynamic -nographic

Notes about arguments:
* `-kernel bblvmlinuxinitramfs_dynamic`: This is the path to the binary to run. In this case, it contains the bbl bootloader, vmlinux, and an initramfs containing busybox.

Useful optional arguments:
* `-m 2048M`: Set size of memory, in this example, 2048 MB

<!--**IMPORTANT**: To cleanly exit this system, you must enter `halt` at the prompt
and then hit `ctrl-a x`. Otherwise, the root filesystem will likely be corrupted.-->

####Current limitations:

* The current RISC-V board definition provides only an HTIF console device.
Support for other HTIF-based devices has been removed from [riscv-linux]; as a 
result, QEMU no longer supports them either.

### Method 2 \(Standard Devices\):

Coming soon!

Running RISC-V Tests:
---------------------

A script (`run-rv-tests.py`) for running the RV64 tests from [riscv-tests] is
included in the `hacking_files` directory. All RV64 tests (listed in 
`hacking_files/rv-tests-list`) are expected to pass, however you may need to 
increase  `TIMER_INTERVAL` in `riscv-tests/env/pt/riscv_test.h`. Also, see
the note in `target-riscv/TODO` about HTIF compatibility with tests.

Using QEMU to Debug RISC-V Code:
--------------------------------

**NOTE:** Support for QEMU + GDB is still under development for the v1.9 spec. See `target-riscv/TODO`.

QEMU works with riscv-gdb to enable remote debugging. This currently requires
building `gdb` from a special version of `riscv-gnu-toolchain`, available 
[here](https://github.com/riscv/riscv-gnu-toolchain/tree/binutils-submodule).

To use this, start QEMU with the additional flags `-S -s`:

    $ ./riscv-softmmu/qemu-system-riscv -S -s -kernel bbl -append vmlinux -drive file=rootfs.ext2,format=raw -nographic

This will start QEMU, but immediately pause and wait for a gdb connection.

Separately, start `riscv64-unknown-elf-gdb`:

    $ riscv64-unknown-elf-gdb [optional binary, e.g. vmlinux]

At the prompt, connect to QEMU:

    (gdb) target remote localhost:1234

At this point, you can use regular gdb commands to singlestep, set breakpoints, 
read/write registers, etc. If you type `continue` in gdb, you can return to QEMU 
and interact with the machine as if you were using it without GDB attached.

TODOs:
------

* See target-riscv/TODO

Notes
-----

- QEMU also supports a "linux-user" mode, however this is currently not implemented for RISC-V. For RISC-V, similar functionality can be obtained by using the [proxy kernel] instead of Linux.
- Files/directories of interest:
  - target-riscv/
  - hw/riscv/

[riscv-linux]:https://github.com/riscv/riscv-linux/tree/priv-1.9
[Buildroot]:https://github.com/a0u/buildroot
[riscv-tests]:https://github.com/riscv/riscv-tests
[proxy kernel]:https://github.com/riscv/riscv-pk

