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

// LED PWM frequency [Hz]
#define LED_FREQ 1000

// PWM resolution (brightness steps)
#define PWM_RES 128

// Lamp matrix update frequency [Hz]
#define MATRIX_UPDATE_FREQ 100

// Input sampling frequency [Hz]
#define INPUT_SAMPLING_FREQ 4000

// Duration of anti ghosting [us] (turning off all lamps briefly)
#define ANTIGHOST_DURATION 18

// Turn debug output via serial on/off
#define DEBUG_SERIAL 1

// Afterglow RP2040 version number
#define AFTERGLOW_RP2040_VERSION 100

// Latest supported Afterglow board revision. Currently v1.3
#define BOARD_REV 30


//------------------------------------------------------------------------------
// derived values
// DO NOT MODIFY

// Brightness matrix data update time interval [us]
#define MATRIX_UPDATE_INT (1000000 / MATRIX_UPDATE_FREQ)

// Output LED update time interval [us]
// Interval between two updates of the same LED
#define LED_UPDATE_INT (1000000 / LED_FREQ)

// Single LED update duration [us]
// Duration of a single column update
// This is the maximum time a LED can be lit. This is also the time which can be
// split into anti-ghosting off time and LED duty cycling.
#define LED_UPDATE_DUR (LED_UPDATE_INT / NUM_COL)

// Input sampling interval time interval [us]
#define INPUT_SAMPLE_INT (1000000 / INPUT_SAMPLING_FREQ)

// Input sampling to matrix update ratio
#define SAMPLE_TO_UPDATE_RATIO (INPUT_SAMPLING_FREQ / MATRIX_UPDATE_FREQ)
// This ratio must be integer
#if ((SAMPLE_TO_UPDATE_RATIO * MATRIX_UPDATE_FREQ) != INPUT_SAMPLING_FREQ)
#error "The input sampling frequency must be an integer multiple of the matrix update frequency!"
#endif