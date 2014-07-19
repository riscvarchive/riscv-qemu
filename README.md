riscv-qemu
=========

The riscv-softmmu target for full system emulation is currently supported. 
It supports booting [riscv-linux] \(currently requires building from the 
[qemu branch]\). A precompiled copy of the kernel is included in the "hacking" 
directory for convenience.

Installation
--------------

```sh
$ git clone git@github.com:ucb-bar/riscv-qemu.git
$ cd riscv-qemu
$ git submodule update --init pixman
$ ./configure --target-list=riscv-softmmu
$ make
$ cd riscv-softmmu
$ ./qemu-system-riscv -kernel ../hacking/vmlinux/vmlinux -nographic
```

Notes
-----

- Qemu also supports a "linux-user" mode, however this is currently not implemented for RISC-V.
- Files/directories of interest:
  - target-riscv/
  - hw/riscv/

[riscv-linux]:https://github.com/ucb-bar/riscv-linux
[qemu branch]:https://github.com/ucb-bar/riscv-linux/tree/qemu
