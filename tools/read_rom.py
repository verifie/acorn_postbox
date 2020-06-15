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
import sys
import time

assert sys.version_info.major >= 3, "Python 3+ required"

import board

class Main:

    def __init__(self):
        self.read_buffer = None

    def read_until(self, ser, resp, match):
        if resp.find(match) == -1:
            while True:
                r = ser.read(1024)
                if r:
                    print("READ [%d b + %d b]: %s" % (len(resp), len(r), repr(r)))
                    if self.read_buffer:
                        self.read_buffer.write(r)
                    resp += r
                    if resp.find(match) != -1:
                        print("match found")
                        break
                    else:
                        time.sleep(0.1)
        p = resp.find(match)
        assert p != -1
        p += len(match)
        return resp[:p], resp[p:]

    def main(self):
        with board.Port() as ser:
            self.read_buffer = open("rom_read_buffer_%s.txt" % time.strftime("%Y%m%d_%H%M"), "wb")
            print("\n* Port open.  Waiting for OK.")
            r, tail = self.read_until(ser, b"", b"OK")

            print("\n* Waiting for 'alive!'")
            r, tail = self.read_until(ser, tail, b"alive!")

            if 1:
                # RISC PC, A7000, IBX*
                print("\n* Switch to RISC PC mode")
                ser.write(b"s\n")
                r, tail = self.read_until(ser, tail, b"Machine set to")
                r, tail = self.read_until(ser, tail, b"OK")
                if r.find(b"Archimedes") != -1:
                    print("\nStill in Archimedes mode -- do it again")
                    ser.write(b"s\n")
                    r, tail = self.read_until(ser, tail, b"RISC PC")
                print("Now in RISC PC mode")

            print("\n* Start read")
            ser.write(b"R\n")
            r, tail = self.read_until(ser, tail, b"Reading ROM data")

            r, tail = self.read_until(ser, tail, b"START DATA")
            start_time = time.time()
            input_buf, tail = self.read_until(ser, tail, b"END DATA")
            time_taken = time.time() - start_time
            contents = input_buf  # TODO
            #print("Saving")
            #open("read.rom", "wb").write(contents)
            print("%d bytes read in %.2f s - md5 %s - sha1 %s" % (
                len(contents),
                time_taken,
                hashlib.md5(contents).hexdigest(),
                hashlib.sha1(contents).hexdigest(),
            ))
            print("Done!")

if __name__ == '__main__':
    Main().main()
