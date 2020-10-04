from __future__ import print_function

import os
import re
import sys

PULSE_DETAIL = 0
COUNT_DETAIL = 0
BYTE_DETAIL = 0
BYTE_TIME = 1  # show when each char was received

class Processor:

    def __init__(self):
        self.last_byte = 0
        self.line_so_far = ''

    def decode_pulses(self, fn):
        print("Decoding pulses from %s" % fn)
        f = open(fn)

        headings = f.readline().split(",")
        def find_testreq_in_headings(headings):
            print(headings)
            for idx in range(len(headings)):
                if 'testreq' in headings[idx] or 'la21' in headings[idx].lower():
                    print("testreq is index %d" % idx)
                    return idx
            raise Exception("couldn't find testreq or la21 in headings")
        testreq_idx = find_testreq_in_headings(headings)

        lastval = 0
        lastts = lastpulse = pulsecount = 0

        reading_byte = 0

        for line in f:
            line = line.strip()
            #print(line)
            bits = line.split(",")
            ts = float(bits[0]) * 1000000
            val = int(bits[testreq_idx])

            if PULSE_DETAIL: print(ts, val, ts-lastts, val-lastval)

            if val and not lastval:
                gap = ts - lastpulse
                # print("rising edge on testreq at %f with gap %f" % (ts, gap))
                if gap > 10:  # > 10 us means separate group
                    if COUNT_DETAIL or PULSE_DETAIL:
                        print("space after %d pulses\n" % pulsecount)

                    if pulsecount == 3:
                        reading_byte = 8
                        shifter = 0
                    elif pulsecount < 3 and reading_byte:
                        bit = 1 if pulsecount == 1 else 0
                        shifter = (shifter << 1) | bit
                        reading_byte -= 1
                        if not reading_byte:
                            self.received_byte(shifter, ts)

                    pulsecount = 0
                if PULSE_DETAIL: print("pulse at %d (%d gap)" % (ts, gap))

                pulsecount += 1
                lastpulse = ts

            lastts = ts
            lastval = val

        assert not self.line_so_far

    def received_byte(self, shifter, ts=None):
        if not shifter:
            if self.line_so_far.strip():
                print(self.line_so_far)
                self.line_so_far = ''
        else:
            if BYTE_DETAIL: print("read byte: %02x (%s)" % (shifter, chr(shifter) if 32 <= shifter < 128 else '?'))
            if (shifter & 0xf) == 8:
                if (self.last_byte & 0xf) == 8:
                    c = (self.last_byte & 0xf0) | ((shifter & 0xf0) >> 4)
                    self.line_so_far += chr(c)
                    if BYTE_TIME:
                        print("<%s:%s>" % (ts, chr(c)))
                    shifter = 0  # hack to prevent double printing chars
                    #printed_something = 1
            elif shifter not in (0, 0x30, 0x20, 0x80):
                #print("%02x" % shifter)
                if self.line_so_far: self.line_so_far += ' '
            self.last_byte = shifter

    def decode_chars(self, fn):
        print("Decoding chars from %s" % fn)
        for line in open(fn, 'rt'):
            #print(line.strip())
            m = re.search(r'\(0x(..)\)', line)
            if m:
                ts, b = None, m.group(1)
            if not m:
                m = re.search(r'^(.*?),0x(..)$', line)
                if m:
                    ts, b = m.groups()
            if not m: continue
            b = int(b, 16)
            self.received_byte(b, ts)

p = Processor()

if sys.argv[1] == '-p':
    p.decode_pulses(sys.argv[2])
else:
    p.decode_chars(sys.argv[1])
