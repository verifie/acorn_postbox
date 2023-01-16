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
#   20230115 Changed variable 'b' to 'postbox_output' so I can read the program easier (Not a needed change but I find verbose variable names easier!).
#   20230115 Added a function to detect a POST Fail notification after ARM ID to pickup the hex string and list out errors in human readable.
#   20230116 Returned some of the changes back to original layout.
#   20230116 Added a memory read and interpretation for RISC OS 3.11 and before. I need to read up and expand for future RO versions.

import random
import struct
import time
import binascii

import postbox


def main():
    with postbox.Postbox() as pb:
        pb.sendbyte(0)

        have_half_char = False
        last_printed = None        
        last_output = None
        postbox_output_all = ""

        # POST Status and Error Binary Lookup Table
        post_status_lut =   ["",
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
                postbox_output = pb.readbyte()
            except postbox.Timeout:
                if time.time() - last_output > 5:
                    print("No output for 5 seconds; exiting")


                    # ---

                    # Test for FAIL report.
                    # If the 14th char from the end is "F" then POST (probably) failed.
                    if postbox_output_all[-14] == "F":
                        post_error_code = postbox_output_all[-8:]
                        #print("POST failed with error code: ", post_error_code)

                        # Convert the hex into binary. Each bit set represents an error code.
                        scale = 16 ## equals to hexadecimal
                        num_of_bits = 8
                        post_error_binary_code = bin(int(post_error_code, scale))[2:].zfill(num_of_bits)
                        #print("Binary output: ", post_error_binary_code, "\n")

                        # Decode the binary into error messages.
                        i = len(post_error_binary_code)
                        x = 0

                        print("\n------\nPOST Output Interpreted:\n")
                        while i > 0:
                            i = i - 1
                            #print("binary_position: ", x, " : ", post_error_binary_code[i])
                            x = x + 1

                            # If the bit is true, print the error description.
                            if post_error_binary_code[i] == "1":
                                print(post_status_lut[x])
                            
                            # Once we pass post_status_lut 8, the rest are errors. Beforehand they are state notices. Lets split that up.
                            if x == 8:
                                print("\n")
          
                    print("\n")


                    # ---
                    # Find the memory value and print it out.

                    # Find the part of the output where the description is appropriate.
                    # It is repeated twice, so add up all those characters, then determine the number start position.
                    memory_readout_position = (postbox_output_all.find("M Size ") + 14)

                    # Now find the next word in the post list. Assuming this doesn't change between machines or reports, it should be:
                    memory_readout_position_end = postbox_output_all.find("Data")

                    # Get the POST output report of memory, given in HEX. We use a function for this. Then print it.
                    post_memory_output = postbox_output_all[memory_readout_position:memory_readout_position_end]
                    print("Memory detected: ", memory_hex_conversion(post_memory_output))


                    # ---
                    # To make the output easier to read, lets add a space at the end.            
                    print("\n------\n")

                    return
                continue
            last_output = time.time()
            # print(postbox_output)

            if (postbox_output & 0x0f) == 0:
                have_half_char = False
                if postbox_output == 0:
                    if last_printed != '\n' and last_printed != ' ':
                      print()
                      last_printed = '\n'
                elif postbox_output == 0x90:
                    print("ERROR: Got 0x90 during POST")
                elif postbox_output in (0x30, 0x20, 0x80):
                    pass
                else:
                    if last_printed != ' ' and last_printed != '\n':
                      print(' ', end='')
                      last_printed = ' '
            elif (postbox_output & 0x0f) == 8:
                if have_half_char:
                    char_so_far |= ((postbox_output & 0xf0) >> 4)

                    # Code added to identify word FAIL, starting 14 chars from the end
                    postbox_output_all = postbox_output_all + chr(char_so_far)

                    # Print the character to the screen.
                    print(chr(char_so_far), end='')
                    last_printed = char_so_far
                    have_half_char = False
                else:
                    char_so_far = postbox_output & 0xf0
                    have_half_char = True
#            else:
#                print("[%02x]" % postbox_output)


def memory_hex_conversion(post_output_memory_hex):

    #print("memory_hex_conversion received: ", post_output_memory_hex)

    # POST Memory Lookup Table
    memory_human_readable_lut = {0:'0',
                                512:'512 KB',
                                1024:'1 MB',
                                2048:'2 MB',
                                4096:'4 MB',
                                6144:'6 MB',
                                8192:'8 MB',
                                10240:'10 MB',
                                12288:'12 MB',
                                14336:'14 MB',
                                16384:'16 MB',
                                18432:'18 MB',
                                20480:'20 MB',
                                21504:'21 MB',
                                22528:'22 MB',
                                26624:'26 MB',
                                28672:'28 MB',
                                32768:'32 MB',
                                65536:'64 MB',
                                98304:'96 MB',
                                131072:'128 MB',
                                262144:'256 MB'
                                }

    # Remove any decimal place and create an integer. Not sure why we get them, e.g. 800.1
    head, sep, tail = post_output_memory_hex.partition('.')
    #print("memory_hex:", head)

    # Convert the hex into a value, typically kilobytes.
    memory_integer = int(head, 16)
    #print("memory_integer:", memory_integer)

    # Use the lookup table to derive an easy, human readable value.
    memory_real_world = memory_human_readable_lut[memory_integer]
    #print("memory_real_world:", memory_real_world)
    
    return str(memory_real_world)

if __name__ == '__main__':
    main()


