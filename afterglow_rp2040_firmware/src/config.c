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
#include "config.h"
#include "afterglow.h"
#include "def.h"


//------------------------------------------------------------------------------
// serial port protocol definition

// write buffer size [bytes]
#define AG_CMD_WRITE_BUF 32

// command terminator character
#define AG_CMD_TERMINATOR ':'

// version poll command string
#define AG_CMD_VERSION_POLL "AGV"

// configuration poll command string
#define AG_CMD_CFG_POLL "AGCP"

// configuration save command string
#define AG_CMD_CFG_SAVE "AGCS"

// configuration reset to default command string
#define AG_CMD_CFG_DEFAULT "AGCD"

// data ready string
#define AG_CMD_CFG_DATA_READY "AGDR"

// acknowledge string
#define AG_CMD_ACK "AGCACK"

// NOT acknowledge string
#define AG_CMD_NACK "AGCNACK"


//------------------------------------------------------------------------------

// afterglow configuration
static AFTERGLOW_CFG_t sCfg;

static AG_DIPSWITCH_t sDipSwitch;
static uint8_t sLastDipSwitchValue = 0;


//------------------------------------------------------------------------------
// local functions

void cfg_setDefault();
uint32_t calculateCRC32(const uint8_t *data, uint16_t length);


//------------------------------------------------------------------------------
void cfg_init()
{
    // set the default configuration
    cfg_setDefault();
}

//------------------------------------------------------------------------------
const AFTERGLOW_CFG_t * cfg_config()
{
    return &sCfg;
}

//------------------------------------------------------------------------------
AG_DIPSWITCH_t cfg_dipSwitch()
{
    return sDipSwitch;
}

//------------------------------------------------------------------------------
void cfg_updateDipSwitch(uint8_t rawBits)
{
    if (rawBits != sLastDipSwitchValue)
    {
        bool newTestMode = (rawBits & 0x01) ? true : false;
        if (sDipSwitch.testMode != newTestMode)
        {
            sDipSwitch.testMode = newTestMode;

            // reset the AG mode when the test mode configuration changes -
            // test mode and real input may use different modes
            ag_setMode(AG_MODE_UNKNOWN);
        }
        sDipSwitch.passThrough = (rawBits & 0x02) ? true : false;
    }
    sLastDipSwitchValue = rawBits;
}

//------------------------------------------------------------------------------
uint8_t cfg_lastDipSwitchValue()
{
    return sLastDipSwitchValue;
}

//------------------------------------------------------------------------------
void cfg_setDefault()
{
    // initialize configuration to default values
    memset(&sCfg, 0, sizeof(sCfg));
    sCfg.version = AFTERGLOW_CFG_VERSION;
    uint8_t *pGlowDur = &sCfg.lampGlowDur[0][0];
    uint8_t *pBrightness = &sCfg.lampBrightness[0][0];
    for (uint c=0; c<NUM_COL; c++)
    {
        for (uint r=0; r<NUM_ROW; r++)
        {
            *pGlowDur++ = (DEFAULT_GLOWDUR / GLOWDUR_CFG_SCALE);
            *pBrightness++ = DEFAULT_BRIGHTNESS;
        }
    }

    // calculate the crc
    uint16_t cfgSize = sizeof(sCfg);
    sCfg.crc = calculateCRC32((uint8_t*)&sCfg, cfgSize-sizeof(sCfg.crc));
}

//------------------------------------------------------------------------------
uint32_t calculateCRC32(const uint8_t *data, uint16_t length)
{
    uint32_t crc = 0xffffffff;
    while (length--)
    {
        uint8_t c = *data++;
        for (uint32_t i = 0x80; i > 0; i >>= 1)
        {
            bool bit = crc & 0x80000000;
            if (c & i)
            {
                bit = !bit;
            }
            crc <<= 1;
            if (bit)
            {
                crc ^= 0x04c11db7;
            }
        }
    }
    return crc;
}