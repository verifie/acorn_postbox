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
import serial.tools.list_ports
import sys
import time

assert sys.version_info[0] >= 3, "Python 3+ required"


def guess_port():
    # Look for POSTBOX_PORT env var
    port = os.environ.get("POSTBOX_PORT")
    if port:
        print("Using %s from POSTBOX_PORT environment variable" % port)
        return port

    # Try to detect a connected POST Box or Circuit Playground
    postbox_port = circuitplay_port = None
    for port in serial.tools.list_ports.comports():
        print(port.device,
            port.product,
            port.hwid,
            port.vid,
            port.pid,
            port.manufacturer,
        )
        if port.vid == 0x1209 and port.pid == 0xFE06:
            print("Found a POST Box at %s" % port.device)
            postbox_port = port.device
        elif port.vid == 0x239A and port.pid in (0x0018, 0x8018):
            print("Found a Circuit Playground Express at %s" % port.device)
            circuitplay_port = port.device

    if postbox_port:
        print("Detected a POST Box on %s" % postbox_port)
        return postbox_port

    if circuitplay_port:
        print("Detected a Circuit Playground (probably a POST Box) on %s" % circuitplay_port)
        return circuitplay_port

    raise Exception("Could not find a connected POST Box")


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
