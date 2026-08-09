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

#include "mod.h"

#if MODDING_OUTPUT

#include "pico/stdlib.h"
#include "pindef.h"
#include "lampmatrix.h"


//------------------------------------------------------------------------------
// local definitions

// Number of modding outputs
#define NUM_MOD_OUTPUTS 4



//------------------------------------------------------------------------------
// local variables

static bool sModOutputStates[NUM_MOD_OUTPUTS] = { false };



//------------------------------------------------------------------------------
void mod_evaluate()
{
    const uint32_t *pkRawLM = lm_rawLampMatrix();

    // Logic A
    sModOutputStates[0] = (pkRawLM[0] & (1 << 0)) ? true : false;

    // Logic B
    sModOutputStates[1] = (pkRawLM[1] & (1 << 1)) ? true : false;

    // Logic C
    sModOutputStates[2] = (pkRawLM[2] & (1 << 2)) ? true : false;

    // Logic D
    sModOutputStates[3] = (pkRawLM[3] & (1 << 3)) ? true : false;
}

//------------------------------------------------------------------------------
void mod_output()
{
    // output the modding GPIOs via the 74HC595 shift register

    // pull RCLK (AGPIN_OUT_LOAD) and CLK (AGPIN_OUT_CLK) low to start sending data
    gpio_put(AGPIN_OUT_CLK, false);
    gpio_put(AGPIN_OUT_LOAD, false);
    
    // send the 8 data bits
    for (int32_t i=7; i>=0; i--)
    {
        gpio_put(AGPIN_OUT_CLK, false);
    
        bool data = (i < NUM_MOD_OUTPUTS) ? sModOutputStates[i] : false;
        gpio_put(AGPIN_OUT_DATA, data);

        gpio_put(AGPIN_OUT_CLK, true);

        // wait some time
        uint32_t dummy = 0;
        dummy += 17;
        dummy -= 17;
    }

    // latch the data
    gpio_put(AGPIN_OUT_LOAD, true);
}

#endif











