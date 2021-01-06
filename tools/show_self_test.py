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

import postbox


def main():
    with postbox.Postbox() as pb:
        pb.sendbyte(0)

        have_half_char = False
        last_printed = None
        last_output = None
        while 1:
            try:
                b = pb.readbyte()
            except postbox.Timeout:
                if time.time() - last_output > 5:
                    print("No output for 5 seconds; exiting")
                    return
                continue
            last_output = time.time()
            # print(b)

            if (b & 0x0f) == 0:
                have_half_char = False
                if b == 0:
                    if last_printed != '\n' and last_printed != ' ':
                      print()
                      last_printed = '\n'
                elif b == 0x90:
                    print("ERROR: Got 0x90 during POST")
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
#            else:
#                print("[%02x]" % b)

if __name__ == '__main__':
    main()
