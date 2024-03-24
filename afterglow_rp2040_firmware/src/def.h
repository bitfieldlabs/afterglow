/***********************************************************************
 *   ___  ___  ___  ___  ___  ___   _    ___  _ _ _ 
 *  | . || __>|_ _|| __>| . \/  _> | |  | . || | | |
 *  |   || _>  | | | _> |   /| <_/\| |_ | | || | | |
 *  |_|_||_|   |_| |___>|_\_\`____/|___|`___'|__/_/ 
 *                                                 rp2040
 *      Copyright (c) 2024 bitfield labs
 *
 ***********************************************************************
 *  This file is part of the afterglow pinball LED project:
 *  https://github.com/bitfieldlabs/afterglow
 *
 *  afterglow is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  afterglow is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with afterglow.
 *  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************/

// number of columns in the lamp matrix
#define NUM_COL 8

// number of rows in the lamp matrix
#define NUM_ROW 10

// LED driving frequency [Hz]
// This frequency must be a multiple of 8
#define LED_FREQ 1000

// PWM resolution (brightness steps)
#define PWM_RES 128

// Duration of anti ghosting [us] (turning off all lamps briefly)
#define ANTIGHOST_DURATION         20

// Turn debug output via serial on/off
#define DEBUG_SERIAL                1

// Afterglow RP2040 version number
#define AFTERGLOW_RP2040_VERSION  100

// Latest supported Afterglow board revision. Currently v1.3
#define BOARD_REV                  30
