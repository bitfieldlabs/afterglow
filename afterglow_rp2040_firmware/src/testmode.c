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

#include "testmode.h"
#include "def.h"


//------------------------------------------------------------------------------
// local definitions

#define TEST_MODE_WPC 1         // If set to 1, WPC input is simulated. If set to 0, Whitestar input is simulated.
#define TEST_MODE_NUMMODES 7    // number of test modes
#define TEST_MODE_DUR 8         // test duration per mode [s]
#define TEST_MODE_DUR_CYCLES    ((uint32_t)TEST_MODE_DUR * 1000000UL / INPUT_SAMPLE_INT) // number of cycles per testmode
#define TESTMODE_INT (500)      // test mode lamp switch interval [ms]
#define TESTMODE_CYCLES ((uint32_t)TESTMODE_INT * 1000UL / (uint32_t)INPUT_SAMPLE_INT) // number of cycles per testmode interval
#if (TEST_MODE_WPC == 1)
// Simulate WPC 16ms 8 columns lamp matrix cycle
#  define ORIG_CYCLES (2000 / INPUT_SAMPLE_INT)
#  define NUM_STROBE NUM_COL
#  define NUM_NONSTROBE NUM_ROW
#else
// Simulate Whitestar 10ms 10 rows lamp matrix cycle
#  define ORIG_CYCLES (1000 / INPUT_SAMPLE_INT)
#  define NUM_STROBE NUM_ROW
#  define NUM_NONSTROBE NUM_COL
#endif


//------------------------------------------------------------------------------
uint32_t tm_testModeData(uint32_t ttag)
{
    // Generate NUM_COL column bits and NUM_ROW row bits
    // Col 1 is the LSB, Row NUM_ROW is the MSB

    // simulate the original column cycle
    static uint32_t sCycleCounter = 0;
    static uint32_t sStrobeLine = 0;
    if (sCycleCounter == ORIG_CYCLES)
    {
        sCycleCounter = 0;
        sStrobeLine++;
    }
    if (sStrobeLine == NUM_STROBE)
    {
        sStrobeLine = 0;
    }
    sCycleCounter++;
    uint32_t strobeMask = ((uint32_t)1 << sStrobeLine);

    // populate the non strobed mask
    uint32_t nonStrobeMask = 0;

#if REPLAY_ENABLED
    // test switch 2 activates the replay mode
    if (cfg_dipSwitch().replay)
    {
        // replay from table
        nonStrobeMask = replay(sStrobeLine);
    }
#endif

    // loop through all available modes
    static uint32_t sModeCounter = 0;
    static uint32_t sMode = 0;
    static uint32_t sModeCycleCounter = 0;
    static uint32_t sModeCycle = 0;
    if (sModeCounter == TEST_MODE_DUR_CYCLES)
    {
        sModeCounter = 0;
        sModeCycleCounter = 0;
        sModeCycle = 0;
        sMode++;
    }
    if (sMode == TEST_MODE_NUMMODES)
    {
        sMode = 0;
    }
    sModeCounter++;
    if (sModeCycleCounter == TESTMODE_CYCLES)
    {
        sModeCycleCounter = 0;
        sModeCycle++;
    }
    sModeCycleCounter++;

    switch (sMode)
    {
        case 0:
        // cycle all strobe lines
        {
            uint8_t s = (sModeCycle % NUM_STROBE);
            if (s == sStrobeLine)
            {
                nonStrobeMask = 0xffff;
            }
        }
        break;
        case 1:
        // cycle all non strobe lines
        {
            uint8_t ns = (sModeCycle % NUM_NONSTROBE);
            nonStrobeMask |= ((uint32_t)1 << ns);
        }
        break;
        case 2:
        // cycle all strobe lines (inverted)
        {
            uint8_t s = (sModeCycle % NUM_STROBE);
            if (s != sStrobeLine)
            {
                nonStrobeMask = 0xffff;
            }
        }
        break;
        case 3:
        // cycle all non strobe lines (inverted)
        {
            uint8_t ns = (sModeCycle % NUM_NONSTROBE);
            nonStrobeMask = ~(1 << ns);
        }
        break;
        case 4:
        // blink all lamps
        {
            if (sModeCycle % 2)
            {
                nonStrobeMask = 0xffff;
            }
        }
        break;
        case 5:
        // switch between even and odd lamps
        // turn on every other strobe line
        {
            if (sStrobeLine % 2 == (sModeCycle % 2))
            {
                nonStrobeMask = 0xaaaa;
                if (sModeCycle % 3)
                {
                    nonStrobeMask <<= 1;
                }
            }
        }
        break;
        case 6:
        // cycle through all lamps individually with 4x speed
        {
            uint8_t l = (uint8_t)((sModeCycle * 4) % (NUM_COL * NUM_ROW));
            uint8_t c = (l / NUM_ROW);
            uint8_t r = (l % NUM_COL);
#if (TEST_MODE_WPC == 1)
            if (c == sStrobeLine)
            {
                nonStrobeMask = (1 << r);
            }
#else
            if (r == sStrobeLine)
            {
                nonStrobeMask = (1 << c);
            }
#endif
        }
        break;
        default:
        break;
    }

    // constrain the number of rows
    nonStrobeMask &= (~0 >> (32-NUM_NONSTROBE));

    // assign the column and row mask
#if (TEST_MODE_WPC == 1)
    uint32_t colMask = strobeMask;
    uint32_t rowMask = nonStrobeMask;
#else
    uint16_t colMask = nonStrobeMask;
    uint16_t rowMask = strobeMask;
#endif

    // invert the row mask as in the original input HIGH means off
    rowMask = ~rowMask;
    return (colMask | (rowMask << NUM_COL));
}
