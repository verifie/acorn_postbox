# Acorn POST interface test box

[Stardot thread](https://stardot.org.uk/forums/viewtopic.php?f=16&t=17478)

Based on philpem's work: https://github.com/philpem/acorn_postbox/

Acorn ARM computers (e.g. A-series, R-series and RiscPC) have a POST
code output port which outputs useful test diagnostic data during the
boot process. Unfortunately, in order to read these codes, an
Acorn/Atomwide "POST box". These boxes are extremely rare and hard to
find.

This is a Verilog module which implements the Acorn POST Box / POST
interface protocol and interfaces with a microcontroller.

In its current form, it requires one of [these PCBs from
myelin](https://stardot.org.uk/forums/viewtopic.php?f=8&t=19815), or
you can [make your
own](https://github.com/google/myelin-acorn-electron-hardware/tree/master/post_box_usb).

If you would rather output POST codes on an LCD, you want [philpem's
repository](https://stardot.org.uk/forums/viewtopic.php?f=16&t=17478)
and probably [one of anightin's
boards](https://stardot.org.uk/forums/viewtopic.php?f=16&t=17478&start=60#p271540).

**Note that RISC OS 4 does not include POST Box support**:

  * To test a StrongARM RISC PC, you will need a RISC OS 3.7 or 3.71 ROM set.
  * A RISC OS 3.6 ROM is also suitable if the machine has an ARM 610 processor.
  * For pre-RISC PC hardware, ideally use RISC OS 3.1 or 3.11.
  * I have not tested this with RISC OS 2 as I don't have a RO2 ROM set.
