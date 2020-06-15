import binascii
import os
import re
import sys

assert sys.version_info.major >= 3, "Python 3+ required"

infn, = sys.argv[1:]

print("Decoding ROM info from file %s" % infn)

rom = bytearray()

with open("rom.bin", "wb") as of:
    started = False
    n = 0
    for line in open(infn, "rt"):
        n += 1

        line = line.rstrip()
        if not len(line): continue

        # print(repr(line))
        if not started:
            if line.find("START DATA") != -1:
                print("found start")
                started = True
            continue
        if line.find("END DATA") != -1:
            print("hit END")
            break
        if line.strip() == "Timeout waiting for remote to send a byte":
            print("hit timeout")
            break

        try:
            b = binascii.a2b_hex(('0' * (8 - len(line))) + line)
        except binascii.Error:
            print("Unprocessed line %d: %s" % (n, repr(line)))
            continue

        assert len(b) == 4
        rom.append(b[3])
        rom.append(b[2])
        rom.append(b[1])
        rom.append(b[0])
    of.write(rom)
os.system("ls -al rom.bin")

ncos_fn = "risc_os_nc_5_13.rom"
ncos = open(ncos_fn, "rb").read()
if ncos == rom:
    print("rom == ncos!")
elif rom[:len(ncos)] == ncos:
    print("rom starts with ncos")
else:
    diffs = 0
    for i, c in enumerate(ncos):
        if i >= len(rom):
            print("%s and %s are identical up to the end of %s at %d (0x%x)" % (
                infn, ncos_fn, infn, i, i))
            break
        if c != rom[i]:
            print("Differing byte %d between %s (%02x) and %s (%02x) at index %d (0x%x)" % (
                diffs+1, infn, rom[i], ncos_fn, ncos[i], i, i))
            diffs += 1
            if diffs > 10: break
