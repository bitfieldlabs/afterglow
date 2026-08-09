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
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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

