import subprocess
import os
import sys


# with no test specified, will run all tests
# else run the specific test

tests_dir = "/scratch/sagark/qemu-newpriv1.9/riscv-tools/riscv-tests/isa/"
cmd = "../riscv-softmmu/qemu-system-riscv -m 3840M -kernel " + tests_dir + "{} -nographic"

if len(sys.argv) == 1:
    devnull = open(os.devnull, 'w')

    f = open('rv-tests-list', 'r')
    tlist = f.readlines()
    f.close()
	
    #tlist = filter(lambda x: "pt" not in x, tlist)

    failures = 0
    failed = []

    for x in tlist:
        print cmd.format(x.strip())
        p = subprocess.Popen(cmd.format(x.strip()), shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
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
    runcmd = cmd.format(sys.argv[1].strip())
    subprocess.call(runcmd, shell=True)
