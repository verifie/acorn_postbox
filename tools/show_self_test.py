import random
import struct
import time

import postbox

with postbox.Postbox() as pb:
    pb.sendbyte(0)

    have_half_char = False
    last_printed = None
    while 1:
        try:
            b = pb.readbyte()
        except postbox.Timeout:
            continue

        if (b & 0x0f) == 0:
            have_half_char = False
            if b == 0:
                if last_printed != '\n' and last_printed != ' ':
                  print()
                  last_printed = '\n'
            elif b == 0x90:
                Serial.println("ERROR: Got 0x90 during POST")
                break
            elif b in (0x30, 0x20, 0x80):
                pass
            else:
                if last_printed != ' ' and last_printed != '\n':
                  print(' ', end='')
                  last_printed = ' '
        elif (b & 0x0f) == 8:
            if have_half_char:
                char_so_far |= ((b & 0xf0) >> 4)
                print(chr(char_so_far), end='')
                last_printed = char_so_far
                have_half_char = False
            else:
                char_so_far = b & 0xf0
                have_half_char = True