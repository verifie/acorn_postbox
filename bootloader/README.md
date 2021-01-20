# Bootloader for POST Box ATSAMD21E18A

This is the bootloader for the POST Box's MCU, built from [github.com/myelin/uf2-samdx1](https://github.com/myelin/uf2-samdx1).

To program into a freshly made or unresponsive POST Box:

~~~
python3 -m pip install --user https://github.com/adafruit/Adafruit_Adalink/archive/master.zip
python3 -m adalink.main -v atsamd21g18 -p jlink -w -h bootloader-*.bin
~~~

To program into an POST Box with a working bootloader, [install arduino-cli](https://arduino.github.io/arduino-cli/installation/) and run the following:

~~~
cp update-bootloader-*.ino bootloader.ino
arduino-cli compile --verbose --fqbn arduino:samd:adafruit_circuitplayground_m0
# Use /dev/ttyACM* (Linux) or COMn (Windows) here:
arduino-cli upload --verbose --fqbn arduino:samd:adafruit_circuitplayground_m0 --port /dev/tty.usbmodem*
~~~

To rebuild:

~~~
make clean build
~~~