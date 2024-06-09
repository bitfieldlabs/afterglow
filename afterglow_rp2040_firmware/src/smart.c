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

#include "smart.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "def.h"
#include "pindef.h"


//------------------------------------------------------------------------------
// local variables

static uint32_t sLampCurrent[NUM_COL][NUM_ROW] = { 0 };
static uint32_t sLampCurrentMeas[NUM_COL][NUM_ROW] = { 0 };
static LAMP_TYPE_t sLampTypes[NUM_COL][NUM_ROW] = { LAMP_TYPE_UNKNOWN };


//------------------------------------------------------------------------------
LAMP_TYPE_t smart_lampType(uint32_t col, uint32_t row)
{
    return ((col < NUM_COL) && (row < NUM_ROW)) ? sLampTypes[col][row] : LAMP_TYPE_UNKNOWN;
}

//------------------------------------------------------------------------------
void smart_detect_lamps()
{
    // quickly cycle through each lamp and measure the current

    // cycle through all rows
    for (uint32_t r=0; r<NUM_ROW; r++)
    {
        // disable all output
        gpio_put_masked(AGPIN_OUT_ALL_MASK, false);

        // enable the row
        gpio_put(skAGRowOutPins[r], true);

        // repeat each row a few times to properly light the lamp
        for (uint32_t i=0; i<10; i++)
        {
            // light each lamp for 1ms
            for (uint32_t c=0; c<NUM_COL; c++)
            {
                // disable all columns for a short time (anti-ghosting)
                gpio_put_masked(AGPIN_OUT_COL_MASK, false);
                sleep_us(ANTIGHOST_DURATION);

                // enable the column
                gpio_put(skAGColOutPins[c], true);

                // measure the current for a millisecond
                uint64_t ts = to_us_since_boot(get_absolute_time());
                while ((to_us_since_boot(get_absolute_time()) - ts) < 1000)
                {
                    sLampCurrent[c][r] += adc_read();
                    sLampCurrentMeas[c][r]++;
                }

                // disable all columns again
                gpio_put_masked(AGPIN_OUT_COL_MASK, false);
            }
        }
    }

    // disable all output again
    gpio_put_masked(AGPIN_OUT_ALL_MASK, false);

    // classify the lamps
    for (uint32_t c=0; c<NUM_COL; c++)
    {
        for (uint32_t r=0; r<NUM_ROW; r++)
        {
            if (sLampCurrentMeas[c][r] > 0)
            {
                uint32_t v = (sLampCurrent[c][r] / sLampCurrentMeas[c][r]);
                if (v > 4090) sLampTypes[c][r] = LAMP_TYPE_SHORT;
                else if (v > 3400) sLampTypes[c][r] = LAMP_TYPE_INC;
                else if (v > 400) sLampTypes[c][r] = LAMP_TYPE_LED;
                else if (v > 200) sLampTypes[c][r] = LAMP_TYPE_TINYLED;
                else sLampTypes[c][r] = LAMP_TYPE_NONE;
            }
        }
    }
}

//------------------------------------------------------------------------------
void smart_detect_print()
{
    // output highest and lowest current
    for (uint32_t c=0; c<NUM_COL; c++)
    {
        for (uint32_t r=0; r<NUM_ROW; r++)
        {
            printf("CR %lu %lu %lu - %lu -> t %d\n", c, r,
                sLampCurrentMeas[c][r], sLampCurrent[c][r], sLampTypes[c][r]);
        }
    }
}
