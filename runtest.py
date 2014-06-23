#!/usr/bin/env python3

import os
import subprocess
import sys

t = os.listdir("tests/riscv/")
newT = []

successes = []
fails = []

for x in t:
    if "rv64ui" in x:
        newT.append(x)

for x in newT:
    print(x + ": ",end="",flush=True)
    o = subprocess.Popen("./mips64-linux-user/qemu-mips64 " + "tests/riscv/"+x, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True).communicate()[0]
    print(o)
    if "SUCCESS" in str(o):
        successes.append(x)
    else:
        fails.append(x)
    
print("Passing (" + str(len(successes)) + "):")
print(successes)
print("Failing (" + str(len(fails)) +"):")
print(fails)
