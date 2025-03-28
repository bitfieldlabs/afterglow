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

#include <stdio.h>

#ifndef __PARAMS_H__
#define __PARAMS_H__

// Main parameters structure
typedef struct AG_PARAMS_s
{
    // *************** CONFIGURABLE VALUES ****************

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
    uint32_t ledFreq;

    // PWM resolution (brightness steps)
    // This is the number of duty cycle steps within one LED
    // update interval (LED_FREQ).
    // A higher resolution means longer data preparation and higher PIO
    // frequency, but also smoother brightness steps.
    // Valid resolutions: 4, 8, 16, 32, 64, 128, 256
    uint32_t pwmRes;

    // Lamp matrix update frequency [Hz]
    // This is the frequency the output data is updated at.
    // It should be high enough to allow for visually smooth transitions.
    // Between updates the same data is repeatedly output at high rate
    uint32_t matrixUpdateFreq;

    // Duration of anti ghosting [us] (turning off all lamps briefly)
    uint32_t antiGhostDur;


    // *************** DERIVED VALUES ****************

    // Brightness matrix data update time interval [us]
    uint32_t matrixUpdateInt;
    uint32_t matrixUpdateIntMs; // [ms] version

    // Output LED update time interval [us]
    // Interval between two updates of the same LED
    uint32_t ledUpdateInt;

    // Single LED update duration [us]
    // Duration of a single column update
    // This is the maximum time a LED can be lit. This is also the time which can be
    // split into anti-ghosting off time and LED duty cycling.
    uint32_t ledUpdateDur;

    // Number of PIO steps used for anti-ghosting at the beginning of each
    // column output. During this period all outputs are set to LOW.
    uint32_t antiGhostSteps;

    // Number of total PIO steps (anti-ghosting + LED PWM duty cycle)
    uint32_t matrixPIOSteps;

    // Pointer to the brightness map
    const uint8_t *pkBrightnessMap;
} AG_PARAMS_t;

const AG_PARAMS_t * par_params();
void par_setDefault();
void par_setLEDFreq(uint32_t ledFreq);
void par_setPWMRes(uint32_t pwmRes);

#endif // __PARAMS_H__
