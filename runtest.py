#!/usr/bin/env python3

import os
import subprocess
import sys

t = os.listdir("tests/riscv/")
newT = []

successes = []
fails = []

for x in t:
    if "rv64ui" in x and "amo" not in x:
        newT.append(x)

for x in newT:
    print(x + ": ",end="",flush=True)
    o = subprocess.Popen("./mips64-linux-user/qemu-mips64 " + "tests/riscv/"+x, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True).communicate()[0]
    print(o)
    if "SUCCESS" in str(o):
        successes.append(x)
    else:
        fails.append(x)
  
# special test notes
note = "rv64ui-p-fence_i test will currently fail since there are issues with self-modifying code."

print("Passing (" + str(len(successes)) + "):")
print(successes)
print("Failing (" + str(len(fails)) +"):" + " " + note)
print(fails)
