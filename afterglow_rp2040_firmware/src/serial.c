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

#include "serial.h"
#include "pico/time.h"
#include "lampmatrix.h"
#include "matrixout.h"
#include "afterglow.h"
#include "config.h"
#include "def.h"


//------------------------------------------------------------------------------
// local variables

static uint32_t sLastDebugTTag = 0;


//------------------------------------------------------------------------------
void serial_debug(uint32_t ttag)
{
    uint32_t m = to_ms_since_boot(get_absolute_time());
    if ((m - sLastDebugTTag) > 2000)
    {
        // column/row data
        printf("data %08lx mode %d st %d\n", lm_lastInputData(), (int)ag_mode(), ag_status());

        // configuration
        AG_DIPSWITCH_t ds = cfg_dipSwitch();
        printf("cfg : %02x - tm %d pt %d\n", cfg_lastDipSwitchValue(), ds.testMode, ds.passThrough);

        // errors
        printf("err : inv %ld\n", lm_invalidDataCounter());

        // raw lamp matrix
        const uint32_t *pkRawLM = lm_rawLampMatrix();
        printf("raw :\n");
        for (uint c=0; c<NUM_COL; c++)
        {
            printf("   ");
            uint32_t cd = *pkRawLM;
            for (uint r=0; r<NUM_ROW; r++)
            {
                printf("%s", (cd & 0x01) ? "X" : ".");
                cd >>= 1;
            }
            pkRawLM++;
            printf("\n");
        }

        // brightness matrix
        const uint32_t *pkLM = matrixout_lampMatrix();
        printf("br  : %lu %lu\n", pkLM[0], pkLM[1]); 

        sLastDebugTTag = m;
    }
}
