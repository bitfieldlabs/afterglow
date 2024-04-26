# Afterglow PCB v3.0

This is the newest generation of afterglow boards, supporting WPC, Data East, System 11 and Stern Whitestar/S.A.M. games.

## What's new

The new board is based on the RP2040 microprocessor. It is more robust than previous versions and it supports hats!

![afterglow](https://github.com/smyp/afterglow/blob/master/docu/images/ag_30_whatsnew.jpg "Afterglow PCB v3.0 What's New")

## Production
This board is designed for automated PCB production and assembly. The [gerber directory](https://github.com/bitfieldlabs/afterglow/tree/master/afterglow_pcb_v30/gerber) contains production files for [JLCPCB](https://jlcpcb.com/).

Components were chosen based on availability and price.

The connectors and some other parts are currently not included in the assembly files.

## Firmware
The RP2040 firmare is available in the [afterglow_rp2040_firmware](https://github.com/bitfieldlabs/afterglow/tree/master/afterglow_rp2040_firmware) directory. 

### Building the firmware
The firmware is built using [Visual Studio Code](https://code.visualstudio.com/) with [platformio](https://platformio.org/).
It uses [wizio-pico](https://github.com/maxgerhardt/wizio-pico) as a baremetal framework.

### Updating the firmware
There is also a [prebuilt binary UF2 file](https://github.com/bitfieldlabs/afterglow/tree/master/afterglow_rp2040_firmware/firmware_binary_uf2) available for direct upload to the chip. Push the boot select button SW2 when powering up and a mass storage device should appear on your computer. Just copy over the firmware uf2 file.

## Hats
The board supports the connection of hats, extending the functionality to other games, for debugging, modding etc.

### The debugger's hat
The [debugger's hat PCB project](https://github.com/bitfieldlabs/afterglow/tree/master/afterglow_pcb_v30/hat_debug) just visualizes the lamp matrix with LEDs and adds a connector for an I2C OLED display.

![afterglow](https://github.com/smyp/afterglow/blob/master/docu/images/ag_30_debug.jpg "Afterglow PCB v3.0 Debugger's hat")

### The pin bender
The [pin bender hat](https://github.com/bitfieldlabs/afterglow/tree/master/afterglow_pcb_v30/hat_stern_de_sys11) adds connectors for direct hookup of Stern Whitestar/S.A.M games as well as Data East and System 11 machines.

![afterglow](https://github.com/smyp/afterglow/blob/master/docu/images/ag_30_ws.jpg "Afterglow PCB v3.0 Pin Bender hat")

## Files

### Schematic
![afterglow](https://github.com/smyp/afterglow/blob/master/afterglow_pcb_v30/afterglow_schematic.pdf "Afterglow PCB v3.0 Schematic")

### BOM
![afterglow](https://github.com/smyp/afterglow/blob/master/afterglow_pcb_v30/afterglow_bom.csv "Afterglow PCB v3.0 BOM")
