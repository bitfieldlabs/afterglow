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

#include <string.h>
#include <math.h>
#include <stdio.h>
#include "lampmatrix.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "def.h"
#include "pindef.h"
#include "matrixout.h"
#include "config.h"
#include "utils.h"
#include "afterglow.h"
#include "testmode.h"
#include "input.h"


//------------------------------------------------------------------------------
// local variables

static uint32_t sRawLampMatrix[NUM_COL] = { 0 };
static uint32_t sLastData = 0;
static uint32_t sLastCol = 0xffffffff;
static uint32_t sLastRow = 0xffffffff; 
static uint32_t sLastValidCol = 0xffffffff;
static uint32_t sLastValidRow = 0xffffffff; 
static uint32_t sConsistentDataCount = 0;
static uint32_t sWPCModeCounter = 0;
static uint32_t sWhitestarModeCounter = 0;
static uint32_t sInvalidDataCounter = 0;
static uint32_t sMaxDur = 0;


//------------------------------------------------------------------------------
// function prototypes

static bool lm_dataValid(uint32_t c, uint32_t r, uint32_t *pCol, uint32_t *pRow);
static void lm_modeDetection(uint c, uint r);


//------------------------------------------------------------------------------
void lm_init()
{
    memset(sRawLampMatrix, 0, sizeof(sRawLampMatrix));
}

//------------------------------------------------------------------------------
void lm_inputUpdate(uint32_t ttag)
{
    // time is ticking
    uint64_t ts = to_us_since_boot(get_absolute_time());

    // sample the input data
    uint32_t dataIn = input_dataRead();

    // process the DIP switch information (bits 19-23 of the input)
    // Bits 0-3: CFG1 - CFG4
    cfg_updateDipSwitch((uint8_t)((dataIn>>18) & 0x0f));

    // extract the lamp matrix data (first 18 bits of the input)
    // Bits 0-7: column 1-8
    // Bits 8-17: row 1-10
    uint32_t lmData = (dataIn & 0x0003ffff);

    // if in test mode, replace the lamp data with simulated input
    if (cfg_dipSwitch().testMode)
    {
        lmData = tm_testModeData(ttag);
    }

    // check data consistency
    if (lmData == sLastData)
    {
        sConsistentDataCount++;
    }
    else
    {
        sConsistentDataCount = 1;
    }

    // read the current mode
    AFTERGLOW_MODE_t mode = ag_mode();

    // process new data
    if (sConsistentDataCount == SINGLE_UPDATE_CONS)
    {
        uint32_t colData = (lmData & 0x000000ff); // 8 column bits
        uint32_t rowData = ((lmData & 0x0003ff00) >> 8); // 10 row bits

        // Mode detection is active as long as the AG is in initialisation status
        if (mode == AG_MODE_UNKNOWN)
        {
            lm_modeDetection(colData, rowData);
        }
        else
        {
            // check data validity
            uint32_t col, row;
            if (lm_dataValid(colData, rowData, &col, &row))
            {
                // update the lamp matrix
                if (col != 0xffffffff)
                {
                    // WPC column update
                    sRawLampMatrix[col] = rowData;
                }
                else
                {
                    // Whitestar row update
                    uint32_t cd = colData;
                    uint32_t rowBit = (1 << row);
                    uint32_t rowMask = ~rowBit;
                    for (uint c=0; c<NUM_COL; c++)
                    {
                        if (cd & 0x01)
                        {
                            sRawLampMatrix[c] |= rowBit;
                        }
                        else
                        {
                            sRawLampMatrix[c] &= rowMask;
                        }
                        cd >>= 1;
                    }
                }
            }
        }

        // remember this data
        sLastCol = colData;
        sLastRow = rowData;
    }

    // count bad data
    if ((sLastValidCol == sLastCol) && (sLastValidRow == sLastRow))
    {
        sInvalidDataCounter = 0;
    }
    else if (mode != AG_MODE_UNKNOWN)
    {
        sInvalidDataCounter++;
    }

    sLastData = lmData;

    // measure time
    uint64_t te = to_us_since_boot(get_absolute_time());
    uint32_t dur = (uint32_t)(te - ts);
    if (dur > sMaxDur)
    {
        sMaxDur = dur;
    }
}

//------------------------------------------------------------------------------
void lm_modeDetection(uint c, uint r)
{
    // For WPC, DE, Sys11 the columns alternate, i.e. only one column bit can
    // be set at any time.
    if (skBitsPerByte[(uint8_t)c] == 1)
    {
        // check if this is the expected column value
        if (c != sLastCol)
        {
            uint expectedColValue = (sLastCol == 0x80) ? 0x01 : (sLastCol << 1);
            sWPCModeCounter = (c == expectedColValue) ? (sWPCModeCounter+1) : 0;
        }
    }

    // For S.A.M. and Whitestar, the rows are used for
    // multiplexing, therefore only one row bit may be set.
    uint bpr = (skBitsPerByte[(uint8_t)r] + skBitsPerByte[(uint8_t)(r>>8)]);
    if (bpr == 1)
    {
        // check if this is the expected row value
        if (r != sLastRow)
        {
            uint expectedRowValue = (sLastRow == 0x0200) ? 0x01 : (sLastRow << 1);
            sWhitestarModeCounter = (r == expectedRowValue) ? (sWhitestarModeCounter+1) : 0;
        }
    }

    // check if we have enough consistent information
    if (sWPCModeCounter > MODE_DETECTION_THRESH)
    {
        ag_setMode(AG_MODE_WPC);
        sWPCModeCounter = 0;
        sWhitestarModeCounter = 0;
    }
    if (sWhitestarModeCounter > MODE_DETECTION_THRESH)
    {
        ag_setMode(AG_MODE_WHITESTAR);
        sWPCModeCounter = 0;
        sWhitestarModeCounter = 0;
    }
}

//------------------------------------------------------------------------------
bool lm_dataValid(uint32_t c, uint32_t r, uint32_t *pCol, uint32_t *pRow)
{
    bool valid = false;

    if (ag_mode() == AG_MODE_WPC)
    {
        // For WPC, DE, Sys11 the columns alternate, i.e. only one column bit can
        // be set at any time.
        if (skBitsPerByte[(uint8_t)c] == 1)
        {
            switch (c)
            {
                case 0x00000001: *pCol = 0; break;
                case 0x00000002: *pCol = 1; break;
                case 0x00000004: *pCol = 2; break;
                case 0x00000008: *pCol = 3; break;
                case 0x00000010: *pCol = 4; break;
                case 0x00000020: *pCol = 5; break;
                case 0x00000040: *pCol = 6; break;
                case 0x00000080: *pCol = 7; break;
            }
            *pRow = 0xffffffff;
            valid = true;
        }
    }
    else if (ag_mode() == AG_MODE_WHITESTAR)
    {
        // For S.A.M. and Whitestar, the rows are used for
        // multiplexing, therefore only one row bit may be set.
        uint bpr = (skBitsPerByte[(uint8_t)r] + skBitsPerByte[(uint8_t)(r>>8)]);
        if (bpr == 1)
        {
            switch (r)
            {
                case 0x00000001: *pRow = 0; break;
                case 0x00000002: *pRow = 1; break;
                case 0x00000004: *pRow = 2; break;
                case 0x00000008: *pRow = 3; break;
                case 0x00000010: *pRow = 4; break;
                case 0x00000020: *pRow = 5; break;
                case 0x00000040: *pRow = 6; break;
                case 0x00000080: *pRow = 7; break;
                case 0x00000100: *pRow = 8; break;
                case 0x00000200: *pRow = 9; break;
            }
            *pCol = 0xffffffff;
            valid = true;
        }
    }

    if (valid)
    {
        sLastValidCol = c;
        sLastValidRow = r;
    }

    return valid;
}

//------------------------------------------------------------------------------
uint32_t lm_lastInputData()
{
    return sLastData;
}

//------------------------------------------------------------------------------
uint32_t lm_invalidDataCounter()
{
    return sInvalidDataCounter;
}

//------------------------------------------------------------------------------
const uint32_t *lm_rawLampMatrix()
{
    return sRawLampMatrix;
}

//------------------------------------------------------------------------------
uint32_t lm_inputMaxDurAndClear()
{
    uint32_t maxDur = sMaxDur;
    sMaxDur = 0;
    return maxDur;
}
