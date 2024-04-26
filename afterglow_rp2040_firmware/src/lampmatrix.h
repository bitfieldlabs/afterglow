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
 *  https://github.com/bitfieldlabs/afterglow_pico
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


//------------------------------------------------------------------------------
// Setup

#define MODE_DETECTION_THRESH      62     // Number of successful mode identifications need for mode detection


//------------------------------------------------------------------------------

// Initialize the afterglow engine
void lm_init();

// Sample and process the input pinball lamp matrix
void lm_inputUpdate(uint32_t ttag);

// Get the last lamp matrix input data
uint32_t lm_lastInputData();

// Get the invalid input data counter value
uint32_t lm_invalidDataCounter();

// Get a pointer to the raw lamp matrix data
const uint32_t *lm_rawLampMatrix();

// Query the input handling maximum duration and reset to zero [us]
uint32_t lm_inputMaxDurAndClear();
