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

#include "config.h"
#include "afterglow.h"
#include "def.h"

//------------------------------------------------------------------------------
// Some definitions

// Afterglow configuration version
#define AFTERGLOW_CFG_VERSION 3

// glow duration scaling in the configuration
#define GLOWDUR_CFG_SCALE 10


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

// afterglow configuration data definition
typedef struct AFTERGLOW_CFG_s
{
    uint16_t version;                         // afterglow version of the configuration
    uint16_t res;                             // reserved bytes
    uint8_t lampGlowDur[NUM_COL][NUM_ROW];    // Lamp matrix glow duration configuration [ms * GLOWDUR_CFG_SCALE]
    uint8_t lampBrightness[NUM_COL][NUM_ROW]; // Lamp matrix maximum brightness configuration (0-7)
    uint32_t crc;                             // data checksum
} AFTERGLOW_CFG_t;

// afterglow configuration
static AFTERGLOW_CFG_t sCfg;

static AG_DIPSWITCH_t sDipSwitch;
static uint8_t sLastDipSwitchValue = 0;

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
