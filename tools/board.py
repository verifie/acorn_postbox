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

import glob
import os
import serial
import sys
import time

assert sys.version_info[0] >= 3, "Python 3+ required"


def guess_port():
    # One day we'll have a proper USB VID:PID and be able to tell the
    # difference between a POST box and a random USB serial device, but for
    # now...
    port = os.environ.get("POSTBOX_PORT")
    if port:
        return port
    for pattern in [
        "/dev/ttyACM?",
        "/dev/ttyUSB?",
        "/dev/tty.usbserial*",
        "/dev/tty.usbmodem*",
        "/dev/tty.wchusbserial*",
    ]:
        matches = glob.glob(pattern)
        if matches:
            return matches[0]


class Port:
    def __init__(self):
        port = guess_port()
        if not port:
            raise Exception(
                "Could not guess serial port: please set the POSTBOX_PORT environment variable and try again."
            )

        print("Opening port %s" % port)
        start = time.time()
        self.ser = serial.Serial(port, timeout=0)
        print("Port open; took %.2f s: %s" % (
            time.time() - start,
            repr(self.ser),
        ))

    def __enter__(self):
        return self.ser

    def __exit__(self, type, value, traceback):
        self.ser.close()
