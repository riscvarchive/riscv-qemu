riscv-qemu [![Build Status](https://travis-ci.org/riscv/riscv-qemu.svg?branch=master)](https://travis-ci.org/riscv/riscv-qemu)
=========

**About:**

The `riscv-softmmu` target for full system RV64G emulation is currently supported.
It supports booting [riscv-linux]. `riscv-qemu` now provides support for 
the updated privileged spec.

**RISC-V Port Authors:**

* Sagar Karandikar (sagark@eecs.berkeley.edu)

**Upstream QEMU Version:**

* v2.5.0

**Notes:**

* The pre-rebase version of QEMU has been moved to a different repo. Going 
forward, only this repo will be updated and the old version will be removed.
You can temporarily find the old version 
[here](https://github.com/ucb-bar/riscv-qemu-deprecated).

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

You can build vmlinux from the [riscv-linux] repo and a root filesystem using [Buildroot] or download the prebuilt copies below. You'll also need a copy of `bbl`.

**a)** [vmlinux](https://www.eecs.berkeley.edu/~skarandikar/host/qemu/vmlinux)

**b)** [rootfs.ext2](https://www.eecs.berkeley.edu/~skarandikar/host/qemu/rootfs.ext2)

**c)** [bbl](https://www.eecs.berkeley.edu/~skarandikar/host/qemu/bbl)

####Step 3: Run QEMU

To boot Linux (assuming you are in the `riscv-qemu` directory):

    $ ./riscv-softmmu/qemu-system-riscv -kernel bbl -append vmlinux -drive file=rootfs.ext2,format=raw -nographic

Notes about arguments:
* `-kernel bbl`: This is the path to the bbl bootloader, included when riscv-tools is built.
* `-append vmlinux`: The path to the linux kernel image.
* `-drive file=rootfs.ext2,format=raw`: Your root filesystem. You can build one using [Buildroot] or download one above.

Useful optional arguments:
* `-m 128M`: Set size of memory, in this example, 128 MB

**IMPORTANT**: To cleanly exit this system, you must enter `halt` at the prompt
and then hit `ctrl-a x`. Otherwise, the root filesystem will likely be corrupted.

####Current limitations:

* The current RISC-V board definition provides only HTIF devices (syscall 
proxy for `bbl`, console, block device). These devices are experimental and will 
be replaced with standard devices. The console especially can fall behind under
heavy use.

### Method 2 \(Standard Devices\): 

Coming soon!


Running RISC-V Tests:
---------------------

A script (`run-rv-tests.py`) for running the RV64 tests from [riscv-tests] is 
included in the `hacking_files` directory. All RV64 tests are expected to pass, 
however you will likely need to increase  `TIMER_INTERVAL` in 
`riscv-tests/env/pt/riscv_test.h`.

TODOs:
------

* Additional device support

Notes
-----

- QEMU also supports a "linux-user" mode, however this is currently not implemented for RISC-V. For RISC-V, similar functionality can be obtained by using the [proxy kernel] instead of Linux.
- Files/directories of interest:
  - target-riscv/
  - hw/riscv/

[riscv-linux]:https://github.com/riscv/riscv-linux
[Buildroot]:https://github.com/a0u/buildroot
[riscv-tests]:https://github.com/riscv/riscv-tests
[proxy kernel]:https://github.com/riscv/riscv-pk

