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

#include "input.h"
#include "pico/stdlib.h"
#include "pindef.h"


//------------------------------------------------------------------------------
uint32_t input_dataRead()
{
    // drive CLK and LOAD low
    gpio_put(AGPIN_IN_CLK, false);
    gpio_put(AGPIN_IN_LOAD, false);
    
    // wait some time
    uint32_t data = 0;
    data+= 17;
    data-= 17;
    
    // drive LOAD high to save pin states
    gpio_put(AGPIN_IN_LOAD, true);
    data+= 17;
    data-= 17;

    // clock in all 24 data bits from the shift register
    // On 74HC165 D7 is shifted out first, D0 last
    uint32_t s = 0;
    for (uint i=0; i<3; i++)    // 3 shift registers
    {
        uint32_t dataSR = 0;
        for (uint i=0; i<8; i++) // 8 bits
        {
            dataSR <<= 1;
            gpio_put(AGPIN_IN_CLK, false);             // CLK low
            dataSR+= 17;
            dataSR-= 17;
            gpio_put(AGPIN_IN_CLK, true);              // CLK high
            s++;
            dataSR |= gpio_get(AGPIN_IN_DATA) ? 0x01 : 0;   // read data bit
        }
        data |= (dataSR << (s-8));
    }

    gpio_put(AGPIN_IN_CLK, false);
    gpio_put(AGPIN_IN_LOAD, false);

    return data;
}
