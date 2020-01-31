setMode -bscan
addDevice -p 1 -file spi_bridge.jed

setCable -p svf -file spi_bridge.svf
program -e -v -p 1

setCable -p xsvf -file spi_bridge.xsvf
program -e -v -p 1

quit
