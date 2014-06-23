#!/usr/bin/env python3

import os
import subprocess
import sys

t = os.listdir("tests/riscv/")
newT = []

for x in t:
    if "rv64ui" in x:
        newT.append(x)

for x in newT:
    print(x + ": ",end="",flush=True)
    subprocess.call("./mips64-linux-user/qemu-mips64 " + "tests/riscv/"+x, shell=True)
