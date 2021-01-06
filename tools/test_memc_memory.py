# Copyright 2020 Google LLC
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

import random
import struct
import time
import traceback

import postbox

# postbox.SHOW_ALL_DATA = 1
postbox.SHOW_PROTOCOL = 1

video_setup = [
    # Init SFR reg to turn off test mode
    0xC0000100,
    # MODE 13: 640x256, 256 colors (8bpp), 163840 bytes
    0x807FC000,  # reg 80 = 0x7FC000 - horizontal cycle
    0x8408C000,  # reg 84 = 0x08C000 - horizontal sync width
    0x8810C000,  # reg 88 = 0x10C000 - horizontal border start
    0x8C1B4000,  # reg 8C = 0x1B4000 - horizontal display start
    0x906B4000,  # reg 90 = 0x6B4000 - horizontal display end
    0x9476C000,  # reg 94 = 0x76C000 - horizontal border end
    0x9C400000,  # reg 9C = 0x400000 - horizontal interlace
    0xA04DC000,  # reg A0 = 0x4DC000 - vertical cycle
    0xA4008000,  # reg A4 = 0x008000 - vertical sync width
    0xA8048000,  # reg A8 = 0x048000 - vertical border start
    0xAC08C000,  # reg AC = 0x08C000 - vertical display start
    0xB048C000,  # reg B0 = 0x48C000 - vertical display end
    0xB44D0000,  # reg B4 = 0x4D0000 - vertical border end
    0xE00000AE,  # reg E0 = 0x0000AE - control
    0xB8000000,  # reg B8 = 0x000000 - vertical cursor start
    0xBC000000,  # reg BC = 0x000000 - vertical cursor end
    # Palette
    0x00000000,
    0x04000111,
    0x08000222,
    0x0C000333,
    0x10000004,
    0x14000115,
    0x18000226,
    0x1C000337,
    0x20000400,
    0x24000511,
    0x28000622,
    0x2C000733,
    0x30000404,
    0x34000515,
    0x38000626,
    0x3C000737,
    # White screen border
    0x40000FFF,
]

PAGE_SIZE = 3  # 0 for 4k (256k or 512k), 1 for 8k (1M), 2 for 16k (2M), 3 for 32k (4M)
RAM_START = 0x2000000
RAM_SIZE = 4 * 1024 * 1024
SCREEN_SIZE = 160 * 1024
MEM_TEST_MAX = RAM_SIZE  # test all RAM
# MEM_TEST_MAX = SCREEN_SIZE  # only mess with screen memory


def run_test():
    with postbox.Postbox() as pb:
        print("Detecting ROM speed")
        rom_speed = 0
        # ROM wait state setup: 0 = 4 clocks per access; 1 = 3 clocks; 2 = two clocks; 3 = two clocks random, 1 clock sequential
        # ROM_SPEED = postbox.MEMC_ROM_SPEED_4N_4S  # Default; slower than any MEMC machine needs
        # ROM_SPEED = postbox.MEMC_ROM_SPEED_3N_3S  # 12 MHz bus (A3010/A3020/A4000/A5000/A540)
        # ROM_SPEED = postbox.MEMC_ROM_SPEED_2N_2S  # 8 MHz (A3xx/4xx, A3000)
        # ROM_SPEED = postbox.MEMC_ROM_SPEED_2N_1S  # 8 MHz + burst ROM
        for test_rom_speed in range(4):
            print("Trying %d" % test_rom_speed)
            try:
                pb.setup_memc(high_rom_time=test_rom_speed)
                rom_speed = test_rom_speed
            except (postbox.Timeout, postbox.ProtocolError, AssertionError) as e:
                traceback.print_exc()
                break
        print("Best rom speed = %d" % rom_speed)
        if rom_speed < 1:
            raise Exception(
                "This doesn't look right: every platform can support rom_speed > 0"
            )

    with postbox.Postbox() as pb:
        pb.setup_memc(high_rom_time=rom_speed, page_size=PAGE_SIZE)

        print("Setting up video")
        # Write VIDCR a bunch of times to set up the basics
        for w in video_setup:
            # print("Write %08x to VIDCR" % w)
            pb.write_memory_word(0x3400000, w)
        # Set up DMA registers in MEMC
        for addr in [
            0x3600000,  # vstart=0
            0x3620000,  # vinit=0
            0x364A000,  # vend=10240
        ]:
            pb.write_memory_word(addr, 0)

        print("Enabling video DMA")
        pb.setup_memc(high_rom_time=rom_speed, video_dma=1, page_size=PAGE_SIZE)

        print("Checking that we can read and write RAM")
        pb.write_memory_word(RAM_START, 0x12345678)  # start by writing two words
        pb.write_memory_word(RAM_START + 4, 0x4B534154)
        assert (
            pb.read_memory_word(RAM_START) == 0x12345678
        )  # make sure the first one is readable
        assert pb.read_memory_words(RAM_START, 2) == [
            0x12345678,
            0x4B534154,
        ]  # and both at once
        assert pb.read_memory_bytes(RAM_START, 4) == [
            0x78,
            0x56,
            0x34,
            0x12,
        ]  # and all bytes at once
        assert pb.read_memory_byte(RAM_START) == 0x78  # and each byte
        assert pb.read_memory_byte(RAM_START + 1) == 0x56
        assert pb.read_memory_byte(RAM_START + 2) == 0x34
        assert pb.read_memory_byte(RAM_START + 3) == 0x12
        assert pb.read_memory_bytes(RAM_START + 3, 4) == [
            0x12,
            0x54,
            0x41,
            0x53,
        ]  # test unaligned read
        pb.write_memory_byte(RAM_START + 1, 0x97)  # now try a single byte write
        assert pb.read_memory_word(RAM_START) == 0x12349778  # verify byte write
        # If we got this far, the read/write commands should be good enough to access IOC etc.

        print("IOC/IOEB check")
        print("IOEB ID reg - should be xxxxxxx5 if IOEB is present")
        # 0    3    3    5    0    0    5    0
        # 0000 0011 0011 0101 0000 0000 0101 0000
        #        ^^ ^^^^  ^^^ ^^^        ^^^ ^^
        #              T TBBB            AAA AA
        id_reg = pb.read_memory_word(0x03350050)  # IOEB ID; D3:0 should be 5
        print("- IOEB ID reg = %08x" % id_reg)
        if (id_reg & 0xF) == 5:
            print(
                "  IOEB present: this is an A5000 (discrete IOEB) or A3010/A3020/A4000 (ARM250 with built in IOEB)"
            )
            assert rom_speed == 1, (
                "Unexpected rom_speed %d for an IOEB machine (should be 1)" % rom_speed
            )
        else:
            print("  IOEB ID reg indicates no IOEB")
            assert rom_speed == 2, (
                "Unexpected rom_speed %d for a non-IOEB machine (should be 2)"
                % rom_speed
            )
            # This check might fail on an A540, although I think its PALs will identify it as an IOEB machine?

        if 0:
            # Debugging myelin's A5000 (this was used to find the bad latch)
            # Alternating reading these two words results in pin exp.C23 (LA7) always high when Siorq is low.
            while 1:
                pb.read_memory_word(0x03350000)
                pb.read_memory_word(0x033500FC)
        if 0:
            # Debugging myelin's A5000 (this was used to find the bad latch)
            while 1:
                for offset in range(0, 0x100, 4):
                    # why is LA2 always low and LA3 always high on IO requests?
                    # are there IO requests that aren't caused by my code?
                    # PHI2 is definitely held high while Siorq is low, so there's
                    # nothing weird going on there.
                    # So why am I seeing a request to a different address?
                    # LA7:0 = 1001 10(00) = 0x98
                    addr = 0x03200000 + offset
                    print("read %08x -> %08x" % (addr, pb.read_memory_word(addr)))
                    # pb.read_memory_word(0x03350050)  # alternate
                print()
        while 0:
            # Debugging myelin's A5000 (this was used to find the bad latch)
            # 0x50: LA7=0 LA6=1 LA5=0 LA4=1 LA3=0 LA2=0
            id_reg = pb.read_memory_word(0x03350050)  # IOEB ID; D3:0 should be 5
            # id_reg = pb.read_memory_word(0x03350000)  # IOEB ID; D3:0 should be 5
            # 11 0101 0000 0000 1001 1000
            #  3    5    0    0    9    8
            # to prove that, i should be able to see if S4 is asserted
            # but it isn't!  S5 is asserted!
            # check addresses at the IOC...
            # LA6:0 = 001 1000 = 18
            # LA19:16 should be 5 (0101), actually 0101
            # querying 03350000 gives LA7:2 = 1001 1000 = 98
            # is there some processor line that isn't connected to memc?
            # PHOTO:
            #   blue = PHI1
            #   pink = LA7
            #   yellow = Siorq
            #   -> Fiorq goes low *while* LA7 is low, but the latch is released(?) later
            #   looks like the latch is just always in transparent mode.
            #   what does the latch 'E' (LE in the datasheet, pin 11) input do?  74HC573
            #   LE high = transparent mode, LE low = latched
            #   looks like IC36 is stuck in transparent mode.
            print("- IOEB ID reg = %08x" % id_reg)
            if (id_reg & 0xF) != 5:
                print("  IOEB ID reg indicates no IOEB")
            # monitor_reg = pb.read_memory_word(0x03350070)
            # print("+ IOEB monitor type %08x" % monitor_reg)
        #        for c0 in (0, 1):
        #            b = 0xff if c0 else 0xc0  # drive c* low if c0==0 else undrive all
        #            print("IOC setting c0=%d (writing %02x)" % (c0, b))
        #            pb.write_memory_byte(0x03200000, b)
        # print("* IOC control reg (whole word) reads as %08x" % pb.read_memory_word(0x03200000))
        # this often reads as 00 or 20, so obviously IOC *can* drive BD* then

        if 1:
            print("IOC register test (like the one in the RISC OS self-test)")
            ioc_reg_access_ok = True
            for reg, addr in (
                ("IRQMSKA", 0x03200018),
                ("IRQMSKB", 0x03200028),
            ):
                print("Testing read/write %s register at %08x" % (reg, addr))
                for bit in range(8):
                    for v in (1 << bit, 0xFF ^ (1 << bit)):
                        v = 1 << bit
                        pb.write_memory_byte(addr, v)
                        r = pb.read_memory_byte(addr)
                        print(
                            "- Wrote %02x to %08x, read back as %02x (%s)"
                            % (v, addr, r, "ok" if v == r else "FAIL")
                        )
                        if v != r:
                            ioc_reg_access_ok = False
            print(
                "IOC register test %s" % ("PASSED" if ioc_reg_access_ok else "FAILED")
            )

        if 0:
            print("Stepping the IOC BAUD pin through various frequencies")
            for latch in range(1, 10):
                # baud_mhz = 1/(latch + 1)
                # so (latch + 1) = 1/baud_mhz
                # so latch = 1/baud_mhz - 1
                # i.e. for 500 kHz, latch = 1/0.5 - 1 = 1
                # for 250 kHz, latch = 1/.25 - 1 = 3
                baud_khz = 1000.0 / (latch + 1)
                print(
                    "Setting TIMER2 to output a %.3f kHz signal on BAUD (IOC pin 27) by setting latch=%d"
                    % (baud_khz, latch)
                )
                pb.write_memory_bytes(0x03200060, [latch])  # T2 latch low
                pb.write_memory_bytes(
                    0x03200064, [(latch & 0xFF00) >> 8]
                )  # T2 latch high
                pb.write_memory_bytes(0x03200068, [0])  # T2 go
                t2_count_low = pb.read_memory_bytes(0x03200060, 1)[0]  # T2 count low
                t2_count_high = pb.read_memory_bytes(0x03200064, 1)[0]  # T2 count high
                print("t2 count = %02x %02x" % (t2_count_high, t2_count_low))
                time.sleep(1)

        # rainbow pattern
        # screen_pattern = bytes([p & 0xFF for p in range(SCREEN_SIZE)])
        # blank screen
        # screen_pattern = bytes(SCREEN_SIZE)
        # solid color
        color = random.randint(0, 255)
        screen_pattern = bytes(color for _ in range(SCREEN_SIZE))
        print("Clearing the screen")
        # pb.write_memory(RAM_START, screen_pattern)

        # draw blocks
        while 1:
            # stress test MEMC setup by changing rom speed every time
            pb.setup_memc(
                high_rom_time=random.randint(0, rom_speed),
                video_dma=1,
                page_size=PAGE_SIZE,
            )
            sw = 640
            sh = 256
            w = 64
            h = 64 >> 1
            x0 = random.randint(0, sw - w)
            y0 = random.randint(0, sh - h)
            color = random.randint(0, 255)
            print(
                "Drawing a block in color %d at (%d,%d)-(%d,%d)"
                % (color, x0, y0, x0 + w, y0 + w)
            )
            for y in range(y0, y0 + h):
                ptr = RAM_START + y * sw + x0
                data = bytes(color for _ in range(w))
                pb.write_memory(ptr, data)

        # fill random blocks of memory with data and read it back
        while 1:
            print()
            blksize = 4 * 1024
            ptr = (RAM_START + random.randint(0, MEM_TEST_MAX - blksize)) & ~3
            # random pattern
            data = bytes([random.randint(0, 255) for _ in range(blksize)])
            # solid block
            # color = random.randint(0, 255)
            data = bytes([color for _ in range(blksize)])
            pb.write_memory(ptr, data)
            read_data = pb.read_memory(ptr, blksize)
            if read_data != data:
                print(
                    "Wrote %d bytes at %08x but data read back did not match"
                    % (blksize, ptr)
                )
                print("Sent: %s" % repr(data))
                print("Recv: %s" % repr(read_data))
                pb.write_memory_words(ptr, [0x12345678])
                bb = pb.read_memory_words(ptr, 1)
                print(repr(bb))
                print(hex(bb[0]))
                stop


def main():
    start_time = time.time()
    try:
        run_test()
    finally:
        end_time = time.time()
        print("Test ran for %.2f s" % (end_time - start_time))


if __name__ == "__main__":
    main()
