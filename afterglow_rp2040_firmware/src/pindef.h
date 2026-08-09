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

//------------------------------------------------------------------------------
/* This code assumes following RP2040 pin layout:
 *
 *  +------------+-----------------+------------+-----------+
 *  | Name       | Function        | Pico Pin#  | Mode      |
 *  +------------+-----------------+------------+-----------+
 *  | CO1-CO8    | Column Output   | GPIO 0-7   | OUTPUT    |
 *  | RO1-RO10   | Row Output      | GPIO 8-17  | OUTPUT    |
 *  | OUT_CLK    | OUT SR Clock    | GPIO 20    | OUTPUT    |
 *  | OUT_DATA   | OUT SR Data     | GPIO 21    | OUTPUT    |
 *  | OUT_LOAD   | OUT SR Latch    | GPIO 22    | OUTPUT    |
 *  | IN_CLK     | IN SR Clock     | GPIO 23    | OUTPUT    |
 *  | IN_DATA    | IN SR Data      | GPIO 24    | INPUT     |
 *  | IN_LOAD    | IN SR Latch     | GPIO 25    | OUTPUT    |
 *  | STAT_LED   | Status LED      | GPIO 27    | OUTPUT    |
 *  | D_SCL      | Display SCL     | GPIO 28    | OUTPUT    |
 *  | D_SDA      | Display SDA     | GPIO 29    | OUTPUT    |
 *  +------------+-----------------+------------+-----------+
*/

#include <stdio.h>

#define NUM_COL_PINS     8             // Number of column pins
#define NUM_ROW_PINS    10             // Number of row pins

// Input
#define AGPIN_IN_DATA   24             // Shift register input data

// Output
#define AGPIN_CO1        0             // Column 1 output
#define AGPIN_CO2        1             // Column 2 output
#define AGPIN_CO3        2             // Column 3 output
#define AGPIN_CO4        3             // Column 4 output
#define AGPIN_CO5        4             // Column 5 output
#define AGPIN_CO6        5             // Column 6 output
#define AGPIN_CO7        6             // Column 7 output
#define AGPIN_CO8        7             // Column 8 output

#define AGPIN_RO1        8             // Row 1 output
#define AGPIN_RO2        9             // Row 2 output
#define AGPIN_RO3       10             // Row 3 output
#define AGPIN_RO4       11             // Row 4 output
#define AGPIN_RO5       12             // Row 5 output
#define AGPIN_RO6       13             // Row 6 output
#define AGPIN_RO7       14             // Row 7 output
#define AGPIN_RO8       15             // Row 8 output
#define AGPIN_RO9       16             // Row 9 output
#define AGPIN_RO10      17             // Row 10 output

#define AGPIN_OUT_CLK   20             // Output shift register clock
#define AGPIN_OUT_DATA  21             // Output shift register data
#define AGPIN_OUT_LOAD  22             // Output shift register latch

#define AGPIN_IN_CLK    23             // Input shift register clock
#define AGPIN_IN_LOAD   25             // Input shift register latch

#define AGPIN_STAT_LED  27             // Status LED

#define AGPIN_D_SCL     28             // Display SCL
#define AGPIN_D_SDA     29             // Display SDA

//------------------------------------------------------------------------------
// utility macros

#define AGPIN_OUT_ALL_MASK ((1 << AGPIN_CO1) | (1 << AGPIN_CO2) | (1 << AGPIN_CO3) | (1 << AGPIN_CO4) | \
                            (1 << AGPIN_CO5) | (1 << AGPIN_CO6) | (1 << AGPIN_CO7) | (1 << AGPIN_CO8) | \
                            (1 << AGPIN_RO1) | (1 << AGPIN_RO2) | (1 << AGPIN_RO3) | (1 << AGPIN_RO4) | \
                            (1 << AGPIN_RO5) | (1 << AGPIN_RO6) | (1 << AGPIN_RO7) | (1 << AGPIN_RO8) | \
                            (1 << AGPIN_RO9) | (1 << AGPIN_RO10))

#define AGPIN_OUT_COL_MASK ((1 << AGPIN_CO1) | (1 << AGPIN_CO2) | (1 << AGPIN_CO3) | (1 << AGPIN_CO4) | \
                            (1 << AGPIN_CO5) | (1 << AGPIN_CO6) | (1 << AGPIN_CO7) | (1 << AGPIN_CO8))


extern const uint8_t skAGColOutPins[NUM_COL_PINS];
extern const uint8_t skAGRowOutPins[NUM_ROW_PINS];
