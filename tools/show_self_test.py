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


# Changes made by Paul:
#   Changed variable 'b' to 'post_output_char' so I can read the program easier (Not a needed change but I find verbose variable names easier!).
#   Added a function to detect a POST Fail notification after ARM ID to pickup the hex string and list out errors in human readable.


import random
import struct
import time
import binascii

import postbox


def main():
    with postbox.Postbox() as pb:

        pb.sendbyte(0)

        have_half_char              = False
        last_printed                = None        
        last_output                 = None
        post_output_all             = ""

        # POST Error Binary Lookup Table
        post_error_lut =   ["",
                            "Self-test due to power-on",
                            "Self-test due to interface hardware",
                            "Self-test due to test link",
                            "Long memory test performed",
                            "ARM 3 fitted/ARM ID read and not ARM2",
                            "Long memory test disabled",
                            "PC-style IO world detected",
                            "VRAM detected",
                            "FAIL: CMOS RAM checksum error",
                            "FAIL: ROM failed checksum test",
                            "FAIL: MEMC CAM mapping failed",
                            "FAIL: MEMC protection failed",
                            "FAIL: IOC register test failed",
                            "Reserved code (RISC PC)",
                            "FAIL: VIDC Virq (Video Interrupt) timing failed",
                            "FAIL: VIDC Sirq (Sound Interrupt) timing failed",
                            "FAIL: CMOS unreadable",
                            "FAIL: RAM control line failure",
                            "FAIL: Long RAM test failure",
                            "Reserved code (RISC PC)"]


        while 1:
            try:
                post_output_char = pb.readbyte()

            except postbox.Timeout:
                if time.time() - last_output > 5:
                    print("No output for 5 seconds; exiting")

                    # Test for FAIL report.
                    # If the 14th char from the end is "F" then POST (probably) failed.
                    if post_output_all[-14] == "F":
                        post_error_code = post_output_all[-8:]
                        #print("POST failed with error code: ", post_error_code)

                        # Convert the hex into binary. Each bit set represents an error code.
                        scale = 16 ## equals to hexadecimal
                        num_of_bits = 8
                        post_error_binary_code = bin(int(post_error_code, scale))[2:].zfill(num_of_bits)
                        #print("Binary output: ", post_error_binary_code, "\n")

                        # Decode the binary into error messages.
                        i = len(post_error_binary_code)
                        x = 0
                        fail_split = 0
                        # print("binary_position_count: ", i)

                        print("\nPOST Report:\n")
                        while i > 0:
                            i = i - 1
                            #print("binary_position: ", x, " : ", post_error_binary_code[i])
                            x = x + 1

                            # If the bit is true, print the error description.
                            if post_error_binary_code[i] == "1":
                                print(post_error_lut[x])
                            
                            # Once we pass post_error_lut 8, the rest are errors. Beforehand they are state notices. Lets split that up.
                            if x == 8:
                                print("\n")

                    # To make the output easier to read, lets add a space.            
                    print("\n")

                    return
                continue

            last_output = time.time()
            # print(post_output_char)



            if (post_output_char & 0x0f) == 0:

                have_half_char = False

                if post_output_char == 0:
                    if last_printed != '\n' and last_printed != ' ':
                      print()
                      last_printed = '\n'

                elif post_output_char == 0x90:
                    print("ERROR: Got 0x90 during POST")

                elif post_output_char in (0x30, 0x20, 0x80):
                    pass

                else:
                    if last_printed != ' ' and last_printed != '\n':
                      print(' ', end='')
                      last_printed = ' '

            elif (post_output_char & 0x0f) == 8:
                if have_half_char:
                    char_so_far |= ((post_output_char & 0xf0) >> 4)

                    # Code added to identify word FAIL, starting 14 chars from the end
                    post_output_all = post_output_all + chr(char_so_far)

                    # Print the character to the screen.
                    print(chr(char_so_far), end='')

                    last_printed = char_so_far
                    have_half_char = False

                else:
                    char_so_far = post_output_char & 0xf0
                    have_half_char = True

#            else:
#                print("[%02x]" % post_output_char)



if __name__ == '__main__':
    main()
