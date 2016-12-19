import subprocess
import os
import sys


# with no test specified, will run all tests
# else run the specific test

bitslist = ['64', '32']
tests_dir = "/scratch/sagark/qemu-bump-latest/riscv-tools/riscv-tests/isa/"

if len(sys.argv) == 1:

    failures = 0
    failed = []
    for bits in bitslist:
        cmd = "../riscv" + bits + "-softmmu/qemu-system-riscv" + bits + \
                " -m 3840M -kernel " + tests_dir + "{} -nographic"
        devnull = open(os.devnull, 'w')

        f = open('rv' + bits + '-tests-list', 'r')
        tlist = f.readlines()
        f.close()

        for x in tlist:
            print cmd.format(x.strip())
            p = subprocess.Popen(cmd.format(x.strip()), shell=True,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            out, err = p.communicate()
            retval = p.returncode
            if retval != 0:
                failures += 1
                failed.append(x.strip())
                print "NONZERO RETVAL for " + x.strip()
                print retval
    print failed
    print "FAILED: " + str(failures)
    if (failures):
        exit(1)

else:
    bits = '64'
    cmd = "../riscv" + bits + "-softmmu/qemu-system-riscv" + bits + \
            " -m 3840M -kernel " + tests_dir + "{} -nographic"
    runcmd = cmd.format(sys.argv[1].strip())
    subprocess.call(runcmd, shell=True)
