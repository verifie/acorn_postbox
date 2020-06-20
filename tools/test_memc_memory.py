import random
import struct

import postbox

# postbox.SHOW_ALL_DATA = 1

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

ROM_SPEED = 1  # 1 for A5000, 2 for A3000
PAGE_SIZE = 1  # 0 for 4k (256k or 512k), 1 for 8k (1M), 2 for 16k (2M), 3 for 32k (4M)
RAM_START = 0x2000000
RAM_SIZE = 4 * 1024 * 1024
SCREEN_SIZE = 160 * 1024
MEM_TEST_MAX = SCREEN_SIZE  # only mess with screen memory

with postbox.Postbox() as pb:
    pb.setup_memc(high_rom_time=ROM_SPEED, page_size=PAGE_SIZE)

    # Write VIDCR a bunch of times to set up the basics
    for w in video_setup:
        # print("Write %08x to VIDCR" % w)
        pb.write_memory_words(0x3400000, [w])
    # Set up DMA registers in MEMC
    for addr in [
            0x3600000,  # vstart=0
            0x3620000,  # vinit=0
            0x364a000,  # vend=10240
    ]:
        pb.write_memory_words(addr, [0])

    # Enable video DMA
    pb.setup_memc(high_rom_time=ROM_SPEED, video_dma=1, page_size=PAGE_SIZE)

    # rainbow pattern
    # screen_pattern = bytes([p & 0xFF for p in range(SCREEN_SIZE)])
    # blank screen
    # screen_pattern = bytes(SCREEN_SIZE)
    # solid color
    color = random.randint(0, 255)
    screen_pattern = bytes(color for _ in range(SCREEN_SIZE))

    pb.write_memory(RAM_START, screen_pattern)

    # draw blocks
    while 1:
        sw = 640
        sh = 256
        w = 64
        h = 64>>1
        x0 = random.randint(0, sw-w)
        y0 = random.randint(0, sh-h)
        color = random.randint(0, 255)
        for y in range(y0, y0+h):
            ptr = RAM_START + y * sw + x0
            data = bytes(color for _ in range(w))
            pb.write_memory(ptr, data)

    # set video, arcflash-style
    while 1:
        print()
        blksize = 4 * 1024
        ptr = (RAM_START + random.randint(0, MEM_TEST_MAX - blksize)) & ~3
        # random pattern
        #data = bytes([random.randint(0, 255) for _ in range(blksize)])
        # solid block
        color = random.randint(0, 255)
        data = bytes([color for _ in range(blksize)])
        pb.write_memory(ptr, data)
        read_data = pb.read_memory(ptr, blksize)
        if read_data != data:
            print("Wrote %d bytes at %08x but data read back did not match" % (blksize, ptr))
            print("Sent: %s" % repr(data))
            print("Recv: %s" % repr(read_data))
            pb.write_memory_words(ptr, [0x12345678])
            bb = pb.read_memory_words(ptr, 1)
            print(repr(bb))
            print(hex(bb[0]))
            stop
