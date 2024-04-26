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
The RP2040 firmare is available in the [afterglow_rp2040_firmware](https://github.com/bitfieldlabs/afterglow/tree/master/afterglow_rp2040_firmware) directory. There is also a [prebuilt binary UF2 file](https://github.com/bitfieldlabs/afterglow/tree/master/afterglow_rp2040_firmware/firmware_binary_uf2) available for direct upload to the chip. Push the boot select button SW2 when powering up and a mass storage device should appear on your computer. Just copy over the firmware uf2 file.
