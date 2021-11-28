![afterglow](https://github.com/smyp/afterglow/blob/master/artwork/afterglow.png "Afterglow")

AFTERGLOW is a pinball machine extension board that aims at softening the hard lamp on/off transitions which are the consequence of replacing the original incandescent bulbs with LEDs. It works with Williams WPC, Williams System 11, Data East and probably also with Sega games.

The brain of the board is a Arduino nano. It samples the original lamp matrix from the WPC Power Driver Board using two parallel to serial shift registers.

The basic principle involves resampling of the original lamp matrix signals at 4kHz. This allows for PWM LED dimming with 3 bits (8 brightness steps).

The resampled signal is output via serial to parallel shift registers driving 16 MOSFETs.

Make sure to check the ![wiki](https://github.com/smyp/afterglow/wiki) for detailed information and instructions.

![afterglow](https://github.com/smyp/afterglow/blob/master/docu/images/pcb_v13_populated.jpg "Afterglow PCB v1.3")
