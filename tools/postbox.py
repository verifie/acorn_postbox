 # Copyright 2017 Google Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import hashlib
import re
import struct
import sys
import time
from typing import List

assert sys.version_info.major >= 3, "Python 3+ required"

import board

SHOW_ALL_DATA = 0  # output bytes and words read
SHOW_PROTOCOL = 0  # output notes about what we're sending/receiving

class ProtocolError(Exception): pass

class Timeout(ProtocolError): pass

class AlignmentError(ProtocolError): pass

CMD_WRITE = 0x08
CMD_WRITE_NEW_DATA = 0x04
CMD_WRITE_SAME_DATA = 0
CMD_WRITE_INCREMENT_ADDR = 0x02
CMD_WRITE_SAME_ADDR = 0
CMD_WRITE_BYTE = 0x01
CMD_WRITE_WORD = 0

CMD_READ = 0x10
CMD_READ_REPORT_ALL = 0x04
CMD_READ_REPORT_LAST = 0
CMD_READ_INCREMENT_ADDR = 0x02
CMD_READ_SAME_ADDR = 0
CMD_READ_BYTE = 0x01
CMD_READ_WORD = 0

class Postbox:

    def __init__(self):
        self.read_buffer = None
        self.input_checksum = 0
        self.output_checksum = 0
        self.tail = b""
        self.ser = None

    def read_n(self, n, timeout=0.5):
        start_t = time.time()
        while len(self.tail) < n:
            r = self.ser.read(1024)
            if r:
                self.tail += r
                # print(repr(self.tail), repr(r))
            else:
                if time.time() - start_t > 0.5:
                    raise Timeout()
                time.sleep(0.01)
        r, self.tail = self.tail[:n], self.tail[n:]
        # print("read_n(%d) returning %s with tail=%s" % (n, repr(r), repr(self.tail)))
        return r

    def read_until(self, match):
        if self.tail.find(match) == -1:
            while True:
                r = self.ser.read(1024)
                if r:
                    print("READ [%d b + %d b]: %s" % (len(self.tail), len(r), repr(r)))
                    if self.read_buffer:
                        self.read_buffer.write(r)
                    self.tail += r
                    if self.tail.find(match) != -1:
                        print("match found")
                        break
                    else:
                        time.sleep(0.01)

        p = self.tail.find(match)
        assert p != -1
        p += len(match)

        r, self.tail = self.tail[:p], self.tail[p:]
        return r

    def sendbyte(self, b):
        t = bytes([b])
        if SHOW_ALL_DATA:
            print("send %s" % repr(t))
        self.ser.write(t)

    def readbyte(self):
        b = self.read_n(1)[0]
        if SHOW_ALL_DATA:
            print("readbyte -> %02x" % b)
        return b

    def sendword(self, v):
        self.output_checksum = (self.output_checksum + v) & 0xFFFFFFFF
        if SHOW_ALL_DATA:
            print("send word %08x; checksum -> %08x" % (v, self.output_checksum))
        t = bytes([
            (v & 0xFF000000) >> 24,
            (v & 0xFF0000) >> 16,
            (v & 0xFF00) >> 8,
            v & 0xFF,
        ])
        # print("send %s" % repr(t))
        self.ser.write(t)

    def readword(self):
        w = self.read_n(4)
        v = (w[0] << 24) | (w[1] << 16) | (w[2] << 8) | (w[3])
        self.input_checksum = (self.input_checksum + v) & 0xFFFFFFFF
        if SHOW_ALL_DATA: print("readword -> %08x; checksum -> %08x" % (v, self.input_checksum))
        return v

    def start_command(self, cmd):
        assert not self.tail, "tail contains data at command start: %s" % repr(self.tail)
        self.last_command = cmd
        self.sendbyte(cmd)
        self.reset_checksum()

    def finish_command(self):
        fixup = (self.output_checksum ^ 0xFFFFFFFF) + 1
        self.sendword(fixup)

        r = self.readbyte()
        if r == 0xff:
            raise ProtocolError("Checksum error in data we sent")
        if r != self.last_command:
            raise ProtocolError("command byte should have been echoed; got %02x instead" % r)

    def reset_checksum(self):
        self.output_checksum = 0

    def reset_input_checksum(self):
        self.input_checksum = 0

    def verify_input_checksum(self):
        self.readword()  # read checksum complement, which should zero out the sum
        if self.input_checksum != 0:
            raise ProtocolError("Checksum invalid; total sum is %08x, should be zero" % self.input_checksum)

    def wait_ready(self):
        ready_byte = self.readbyte()
        if ready_byte != 0x90:
            raise ProtocolError("got 0x%02x instead of 0x90 at end of command" % ready_byte)

    def read_memory_words(self, start_addr, n_words):
        self.check_aligned(start_addr)
        print("reading %d words from %08x" % (n_words, start_addr))
        start_t = time.time()
        if SHOW_PROTOCOL: print("start command")
        self.start_command(CMD_READ | CMD_READ_REPORT_ALL | CMD_READ_INCREMENT_ADDR | CMD_READ_WORD)
        if SHOW_PROTOCOL: print("send n")
        self.sendword(n_words)
        if SHOW_PROTOCOL: print("send start")
        self.sendword(start_addr)
        if SHOW_PROTOCOL: print("finish command")
        self.finish_command()

        if SHOW_PROTOCOL: print("addr confirmation")
        confirmation = self.readword()
        assert confirmation == start_addr + n_words
        self.reset_input_checksum();
        #data = bytearray()
        data = []
        if SHOW_PROTOCOL: print("\n* End addr: %08x" % start_addr_confirmation)
        for i in range(n_words):
            w = self.readword()
            #print("read word %d -> %08x" % (i, w))
            #data += struct.pack("<I", w)
            data.append(w)
        self.verify_input_checksum()
        if SHOW_PROTOCOL: print("read %d bytes" % len(data))
        self.wait_ready()
        time_taken = time.time() - start_t
        print("read %d words from %08x in %.2f s (%.2f b/s)" % (n_words, start_addr, time_taken, n_words * 4 / time_taken))
        return data

    def read_memory_word(self, addr):
        return self.read_memory_words(addr, 1)[0]

    def read_memory_bytes(self, start_addr, n_bytes):
        print("reading %d bytes from %08x" % (n_bytes, start_addr))
        start_t = time.time()
        self.start_command(CMD_READ | CMD_READ_REPORT_ALL | CMD_READ_INCREMENT_ADDR | CMD_READ_BYTE)
        self.sendword(n_bytes)
        self.sendword(start_addr)
        self.finish_command()

        confirmation = self.readword()
        assert confirmation == start_addr + n_bytes
        self.reset_input_checksum();
        data = []
        for i in range(n_bytes):
            b = self.readword()
            data.append(b)
        self.verify_input_checksum()
        self.wait_ready()
        time_taken = time.time() - start_t
        print("read %d bytes from %08x in %.2f s (%.2f b/s)" % (n_bytes, start_addr, time_taken, n_bytes / time_taken))
        return data

    def read_memory_byte(self, addr):
        return self.read_memory_bytes(addr, 1)[0]

    def __enter__(self):
        self.ser = board.Port().ser

        print("* Port open.  Waiting for OK.")
        self.read_until(b"OK")

        print("* Waiting for 'alive!'")
        self.read_until(b"alive!")

        print("* Enter passthrough mode")
        self.ser.write(b"*")
        self.read_until(b"Enter passthrough mode")

        print("* Wait for 0x90")
        self.read_until(b"\x90")

        print("* POST box connected")
        return self

    def __exit__(self, type, value, traceback):
        if self.ser:
            self.ser.close()

    def read_memory(self, start_addr, n_bytes):
        assert not n_bytes % 4, "Only word reads are supported"
        data = bytearray()
        for w in self.read_memory_words(start_addr, n_bytes >> 2):
            data += struct.pack("<I", w)
        return data

    def read_memory_to_file(self, start_addr, n_bytes, f):
        print("\n* Start read")

        blksize = 16 * 1024

        rom = bytearray()
        for blk in range(0, n_bytes, blksize):
            d = self.read_memory(start_addr + blk, blksize)
            rom += d
            f.write(d)
            assert len(rom) == blk + blksize, "collected %d bytes, expected %d+%d = %d" % (len(rom), blk, blksize, blk+blksize)

        print("\n* Successfully read %d bytes from 0x%08x" % (n_bytes, start_addr))

    # write a word-aligned block of bytes to memory
    def write_memory(self, start_addr: int, data: bytes):
        print("writing %d bytes to memory at 0x%08x" % (len(data), start_addr))
        assert (len(data) % 4) == 0, "Byte write not supported (yet)"

        start_t = time.time()
        self.start_command(CMD_WRITE | CMD_WRITE_NEW_DATA | CMD_WRITE_INCREMENT_ADDR | CMD_WRITE_WORD)
        self.sendword(len(data) >> 2)
        self.sendword(start_addr)
        for ptr in range(0, len(data), 4):
            w = (data[ptr]
                 | (data[ptr+1] << 8)
                 | (data[ptr+2] << 16)
                 | (data[ptr+3] << 24))
            self.sendword(w)
        data_checksum = self.output_checksum
        self.finish_command()
        cs = self.readword()
        if cs != data_checksum:
            raise ProtocolError("Send error; Target reported checksum %08x, expected %08x" % (cs, data_checksum))
        self.wait_ready()
        time_taken = time.time() - start_t
        print("wrote %d bytes to %08x in %.2f s (%.2f b/s)" % (len(data), start_addr, time_taken, len(data) / time_taken))

    def check_aligned(self, addr):
        if addr & 3:
            raise AlignmentError("%08x is not word-aligned" % addr)

    # write a list of words to memory
    def write_memory_words(self, start_addr: int, data: List[int]):
        print("writing %d words to memory at 0x%08x" % (len(data), start_addr))
        self.check_aligned(start_addr)

        self.start_command(CMD_WRITE | CMD_WRITE_NEW_DATA | CMD_WRITE_INCREMENT_ADDR | CMD_WRITE_WORD)
        self.sendword(len(data))
        self.sendword(start_addr)
        for w in data:
            self.sendword(w)
        data_checksum = self.output_checksum
        self.finish_command()
        cs = self.readword()
        if cs != data_checksum:
            raise ProtocolError("Send error; Target reported checksum %08x, expected %08x" % (cs, data_checksum))
        self.wait_ready()

    def write_memory_word(self, addr, w):
        return self.write_memory_words(addr, [w])

    # write bytes to memory one at a time, using the byte write command.
    # this is mainly useful for writing IOC registers.
    def write_memory_bytes(self, start_addr, data: bytes):
        print("writing %d bytes to memory at 0x%08x" % (len(data), start_addr))

        self.start_command(CMD_WRITE | CMD_WRITE_NEW_DATA | CMD_WRITE_INCREMENT_ADDR | CMD_WRITE_BYTE)
        self.sendword(len(data))
        self.sendword(start_addr)
        for b in data:
            self.sendword(b)
        data_checksum = self.output_checksum
        self.finish_command()
        cs = self.readword()
        if cs != data_checksum:
            raise ProtocolError("Send error; Target reported checksum %08x, expected %08x" % (cs, data_checksum))
        self.wait_ready()

    def write_memory_byte(self, addr, b):
        return self.write_memory_bytes(addr, [b])

    def setup_memc(self, os_mode=0, sound_dma=0, video_dma=1, refresh=1, high_rom_time=0, low_rom_time=0, page_size=3):

        # high_rom_time=0 = four clocks (500 ns) per access
        # high_rom_time=1 = three clocks (375 ns) per access
        # high_rom_time=2 = two clocks (250 ns) per access
        # high_rom_time=3 = two clocks, then one clock for sequential reads (should work for RISC OS 2 roms, but not for RISC OS 3 roms, which are slower)

        # 036E = 0011 011X 111X
        # 05 = XX; 0 test; 0 os mode; 0 sound; 1 video; 01 refresh
        # (only refresh during flyback as memc uses the refresh register
        # as the dma pointer)
        # 0C = 00 high rom time; 00 low rom time; 11 page size; XX
        control = (0x036E0000
                   | os_mode << 12
                   | sound_dma << 11
                   | video_dma << 10
                   | refresh << 8
                   | high_rom_time << 6
                   | low_rom_time << 4
                   | page_size << 2)
        print("Configure MEMC: os_mode=%d sound_dma=%d video_dma=%d refresh=%d high_rom_time=%d low_rom_time=%d page_size=%d: control register write %08x" % (os_mode, sound_dma, video_dma, refresh, high_rom_time, low_rom_time, page_size, control))
        self.write_memory_words(control, [0])

    def main(self):
        with self as pb:

            # self.read_buffer = open("rom_read_buffer_%s.txt" % time.strftime("%Y%m%d_%H%M"), "wb")

            start_addr = 0
            n_bytes = 8 * 1024 * 1024
            with open("read.rom", "wb") as romf:
                pb.read_memory_to_file(start_addr, n_bytes, romf)

            print("Done!")

if __name__ == '__main__':
    Postbox().main()
