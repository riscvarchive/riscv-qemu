riscv-qemu [![Build Status](https://travis-ci.org/riscv/riscv-qemu.svg?branch=master)](https://travis-ci.org/riscv/riscv-qemu)
=========

**About:**

The `riscv-softmmu` target for full system RV64G emulation is currently supported.
It supports booting [riscv-linux]. `riscv-qemu` now provides beta support for 
the updated privileged spec.

**RISC-V Port Authors:**

* Sagar Karandikar (sagark@eecs.berkeley.edu)

**Upstream QEMU Version:**

* v2.0.0

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

    $ ./riscv-softmmu/qemu-system-riscv -m 2048M -kernel bbl -hda rootfs.ext2 -append vmlinux -nographic

Notes about arguments:
* `-kernel bbl`: This is the path to the bbl bootloader, included when riscv-tools is built.
* `-hda rootfs.ext2`: Your root filesystem. You can build one using [Buildroot] or download one above.
* `-append vmlinux`: The path to the linux kernel image.

**IMPORTANT**: To cleanly exit this system, you must enter `halt` at the prompt
and then hit `ctrl-a x`. Otherwise, the root filesystem will likely be corrupted.

####Current limitations:

* Must use 2048MB memory (due to hardcoded devicetree).
* The current RISC-V board definition provides only HTIF devices (syscall 
proxy for `bbl`, console, block device). These devices are experimental and will 
be replaced with standard devices. The console especially can fall behind under
heavy use.
* This is a beta release. As such, no performance tuning has been done.

### Method 2 \(Standard Devices\): 

Coming soon!


Running RISC-V Tests:
---------------------

A script (`run-rv-tests.py`) for running the RV64 tests from [riscv-tests] is included in the `hacking_files` directory. The following tests are expected to fail
due to unimplemented features:

* `rv64mi-p-csr`
* `rv64mi-pm-ipi`
* `rv64si-p-csr`


TODOs:
------

* Performance Tuning
* Additional device support
* Rebase to newer upstream QEMU

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

