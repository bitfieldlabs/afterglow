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

#include "params.h"
#include "def.h"
#include "bmap.h"
#include "config.h"


//------------------------------------------------------------------------------
// local variables

static AG_PARAMS_t sParams;


//------------------------------------------------------------------------------
// local functions

void par_derive();


//------------------------------------------------------------------------------
const AG_PARAMS_t * par_params()
{
    return &sParams;
}

//------------------------------------------------------------------------------
void par_setDefault()
{
    // set the default parameters
    sParams.ledFreq = LED_FREQ;
    sParams.pwmRes = PWM_RES;
    sParams.matrixUpdateFreq = MATRIX_UPDATE_FREQ;
    sParams.antiGhostDur = ANTIGHOST_DURATION;

    // calculate the derived parameters
    par_derive();
}

//------------------------------------------------------------------------------
void par_setLEDFreq(uint32_t ledFreq)
{
    sParams.ledFreq = ledFreq;
    par_derive();
}

//------------------------------------------------------------------------------
void par_setPWMRes(uint32_t pwmRes)
{
    sParams.pwmRes = pwmRes;
    par_derive();
}

//------------------------------------------------------------------------------
void par_derive()
{
    sParams.matrixUpdateInt = (1000000UL / sParams.matrixUpdateFreq);
    sParams.matrixUpdateIntMs = (sParams.matrixUpdateInt / 1000);
    sParams.ledUpdateInt = (1000000UL / sParams.ledFreq);
    sParams.ledUpdateDur = (sParams.ledUpdateInt / NUM_COL);

    //
    //   ANTI       ROW
    //   GHOST      PWM
    // |---------|-----------------------------------------------------|
    //           |<------------------ PWM_RES steps ------------------>|
    // |<------->| ANTI_GHOSTING_STEPS
    // |<------------------ MATRIXOUT_PIO_STEPS ---------------------->|

    // Steps of the matrix PIO for anti-ghosting
    sParams.antiGhostSteps = ((sParams.antiGhostDur * sParams.pwmRes) / (sParams.ledUpdateDur - sParams.antiGhostDur));

    // Steps per matrix output PIO run
    sParams.matrixPIOSteps = (sParams.antiGhostSteps + sParams.pwmRes);

    // Choose the brightness map based on the selected PWM resolution
    switch (sParams.pwmRes)
    {
        case 4: sParams.pkBrightnessMap = skMap_256_4; break;
        case 8: sParams.pkBrightnessMap = skMap_256_8; break;
        case 16: sParams.pkBrightnessMap = skMap_256_16; break;
        case 32: sParams.pkBrightnessMap = skMap_256_32; break;
        case 64: sParams.pkBrightnessMap = skMap_256_64; break;
        case 128: sParams.pkBrightnessMap = skMap_256_128; break;
        default:
        case 256: sParams.pkBrightnessMap = skMap_256_256; break;
    }
}
