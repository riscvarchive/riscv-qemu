#!/bin/bash

./qemu-system-mips64 -M malta -kernel ../hacking/vmlinux/vmlinux -nographic

#./qemu-system-mips64 -monitor stdio -M malta -kernel ../hacking/vmlinux/vmlinux -nographic

#./qemu-system-mips64 -M malta -kernel ../hacking/vmlinux/vmlinux -nographic
