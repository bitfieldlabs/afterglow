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

// Number of columns in the lamp matrix
#define NUM_COL 8

// Number of rows in the lamp matrix
#define NUM_ROW 10

// LED PWM frequency [Hz]
// This is the update frequency of each individual lamp.
// The maximum duty cycle for each update is 1/NUM_COL.
// This frequency will affect the achievable brightness, as for each column
// transition all output needs to be disabled briefly (ANTIGHOST_DURATION)
// in order to avoid ghosting. E.g., at 1kHz the update interval for one column
// is 125us (1ms / NUM_COL). 20us anti-ghosting duration will consume 16% of
// the available duty cycle. At a lower frequency of 400Hz the anti-ghosting
// only makes for 6% of the duty cycle, therefore the maximum achievable
// brightness is higher.
#define LED_FREQ 250

// PWM resolution (brightness steps)
// This is the number of duty cycle steps within one LED
// update interval (LED_FREQ).
// A higher resolution means longer data preparation and higher PIO
// frequency, but also smoother brightness steps.
#define PWM_RES 256

// Lamp matrix update frequency [Hz]
// This is the frequency the output data is updated at.
// It should be high enough to allow for visually smooth transitions.
// Between updates the same data is repeatedly output at high rate
// (LED_FREQ * NUM_COL * PWM_RES).
#define MATRIX_UPDATE_FREQ 100

// Input sampling frequency [Hz]
// This is the frequency the input columns and rows are sampled at.
// It must be high enough to allow for multiple samples (SINGLE_UPDATE_CONS)
// of a single input data transition (2ms for WPC, 1ms for Whitestar).
#define INPUT_SAMPLING_FREQ 8000

// Number of consistent data samples required for matrix update. Helps prevent ghosting.
// The input sampling frequency (INPUT_SAMPLING_FREQ) must be high enough to
// produce enough samples for each original column duration (2ms for WPC, 1ms for Whitestar)
#define SINGLE_UPDATE_CONS 4

// Duration of anti ghosting [us] (turning off all lamps briefly)
#define ANTIGHOST_DURATION 20

// Turn debug output via serial on/off
#define DEBUG_SERIAL 1

// Afterglow RP2040 version number
#define AFTERGLOW_RP2040_VERSION 100

// Latest supported Afterglow board revision. Currently v3.0
#define BOARD_REV 30


//------------------------------------------------------------------------------
// derived values
// ** DO NOT MODIFY **

// Brightness matrix data update time interval [us]
#define MATRIX_UPDATE_INT (1000000 / MATRIX_UPDATE_FREQ)
#define MATRIX_UPDATE_INT_MS (MATRIX_UPDATE_INT / 1000) // [ms] version

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
