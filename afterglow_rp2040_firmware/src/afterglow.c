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

#include "pico/stdlib.h"
#include "afterglow.h"
#include "lampmatrix.h"
#include "pindef.h"
#include "config.h"
#include "record.h"


//------------------------------------------------------------------------------
// local definitions

// Maximum number of allowed invalid data readings before changing status to invalid
#define INV_DATA_THRES 255


//------------------------------------------------------------------------------
// local variables

AFTERGLOW_STATUS_t sStatus = AG_STATUS_INIT;
AFTERGLOW_MODE_t sMode = AG_MODE_UNKNOWN;


//------------------------------------------------------------------------------
AFTERGLOW_STATUS_t ag_status()
{
    return sStatus;
}

//------------------------------------------------------------------------------
void ag_setStatus(AFTERGLOW_STATUS_t status)
{
    sStatus = status;
}

//------------------------------------------------------------------------------
AFTERGLOW_MODE_t ag_mode()
{
    return sMode;
}

//------------------------------------------------------------------------------
void ag_setMode(AFTERGLOW_MODE_t mode)
{
    sMode = mode;
}

//------------------------------------------------------------------------------
static void ag_updateStatusLED()
{
    static uint32_t sLedCounter = 0;
    switch (sStatus)
    {
        case AG_STATUS_INIT:
            // always off
            gpio_put(AGPIN_STAT_LED, 0);
            break;
        case AG_STATUS_OK:
            // always on
            gpio_put(AGPIN_STAT_LED, 1);
            break;
        case AG_STATUS_PASSTHROUGH:
            // slow blinking
            gpio_put(AGPIN_STAT_LED, ((sLedCounter >> 4) & 0x01) ? true : false);
            break;
        case AG_STATUS_TESTMODE:
            // slow blinking
            gpio_put(AGPIN_STAT_LED, ((sLedCounter >> 4) & 0x01) ? true : false);
            break;
        case AG_STATUS_REPLAY:
            // slow blinking
            gpio_put(AGPIN_STAT_LED, ((sLedCounter >> 4) & 0x01) ? true : false);
            break;
        case AG_STATUS_RECORDREADY:
            // fast blinking
            gpio_put(AGPIN_STAT_LED, ((sLedCounter % 8) < 2) ? true : false);
            break;
        case AG_STATUS_RECORD:
            // faster blinking
            gpio_put(AGPIN_STAT_LED, ((sLedCounter >> 2) & 0x01) ? true : false);
            break;
        case AG_STATUS_INVINPUT:
        default:
            // fastest blinking
            gpio_put(AGPIN_STAT_LED, ((sLedCounter >> 1) & 0x01) ? true : false);
    }

    sLedCounter++;
}

//------------------------------------------------------------------------------
void ag_statusUpdate()
{
    // determine the current status
    AG_DIPSWITCH_t ds = cfg_dipSwitch();
    if (record_active())
    {
        sStatus = AG_STATUS_RECORD;
    }
    else if (ds.testMode && ds.replayMode)
    {
        sStatus = AG_STATUS_RECORDREADY;
    }
    else if (sMode == AG_MODE_UNKNOWN)
    {
        sStatus = AG_STATUS_INIT;
    }
    else if (lm_invalidDataCounter() < INV_DATA_THRES)
    {

        if (ds.testMode)
        {
            sStatus = AG_STATUS_TESTMODE;
        }
        else if (ds.replayMode)
        {
            sStatus = AG_STATUS_REPLAY;
        }
        else if (ds.passThrough)
        {
            sStatus = AG_STATUS_PASSTHROUGH;
        }
        else
        {
            sStatus = AG_STATUS_OK;
        }
    }
    else
    {
        sStatus = AG_STATUS_INVINPUT;
    }

    // every device needs a blinking LED
    ag_updateStatusLED();
}
