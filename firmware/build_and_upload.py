import os
import sys
import subprocess

import serial.tools.list_ports

import config

assert sys.version_info[0] >= 3, "Python 3+ required"

here = os.getcwd()
build_path = os.environ.get('POSTBOX_BUILD')
if not build_path:
    build_path = os.path.join(
        here,
        "build_output_%s" % sys.platform,  # just in case things differ between platforms
    )

arduino_cli = "arduino-cli"  # in case we need a path to it at some point
std_args = "--verbose --fqbn myelin:samd:myelin_postbox"

# Figure out where the POST Box is plugged in
upload_port = postbox_port = circuitplay_port = None
for port in serial.tools.list_ports.comports():
    print(port.device,
        port.product,
        port.hwid,
        port.vid,
        port.pid,
        port.manufacturer,  # "Adafruit"
    )
    if port.vid == 0x1209 and port.pid == 0xFE06:
        print("Found a POST Box at %s" % port.device)
        postbox_port = port.device
    elif port.vid == 0x239A and port.pid == 0x8018:
        print("Found a Circuit Playground Express at %s" % port.device)
        circuitplay_port = port.device

if "POSTBOX_PORT" in os.environ:
    upload_port = os.environ["POSTBOX_PORT"]
    print("Using %s from POSTBOX_PORT environment variable" % upload_port)
elif postbox_port:
    upload_port = postbox_port
elif circuitplay_port:
    upload_port = circuitplay_port

if not upload_port:
    raise Exception("Could not find a connected POST Box")

print("Using %s as the upload port" % upload_port)

# Make sure we have the correct version of the Adafruit library
subprocess.check_call("%s core install %s" % (
    arduino_cli,
    config.UPSTREAM_CORE,
), shell=True)

# Build sketch
subprocess.check_call("%s compile %s --libraries src --build-path %s" % (
    arduino_cli,
    std_args,
    build_path,
), shell=True)

# Upload sketch
if upload_port and upload_port != 'none':
    subprocess.check_call("%s upload %s --port %s --input-dir %s" % (
        arduino_cli,
        std_args,
        upload_port,
        build_path,
    ), shell=True)
