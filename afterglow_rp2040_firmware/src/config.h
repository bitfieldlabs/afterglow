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

#include <stdio.h>
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
// configuration storage

#define AFTERGLOW_CFG_VERSION       3     // Afterglow configuration version
#define AFTERGLOW_CFG_SER_VERSION   2     // Afterglow configuration version used for configuration via serial port (config tool)
#define GLOWDUR_CFG_SCALE          10     // Glow duration scaling in the configuration

typedef struct AG_DIPSWITCH_s
{
    bool testMode;          // test mode
    bool passThrough;       // pass through mode (input replicated to output)
    bool highLEDFreq;       // high LED frequency mode
} AG_DIPSWITCH_t;

// afterglow configuration data definition
typedef struct AFTERGLOW_CFG_s
{
    uint16_t version;                         // afterglow version of the configuration
    uint16_t res;                             // reserved bytes
    uint8_t lampGlowDurOn[NUM_COL][NUM_ROW];  // Lamp glow duration turning on configuration [ms * GLOWDUR_CFG_SCALE]
    uint8_t lampGlowDurOff[NUM_COL][NUM_ROW]; // Lamp glow duration turning off configuration [ms * GLOWDUR_CFG_SCALE]
    uint8_t lampBrightness[NUM_COL][NUM_ROW]; // Lamp maximum brightness configuration (0-7)
    uint8_t lampDelay[NUM_COL][NUM_ROW];      // Lamp delay when turning on (skipping short on-times, anti-ghosting) [ms]
    uint32_t crc;                             // data checksum
} AFTERGLOW_CFG_t;

// afterglow configuration version 2 data definition
typedef struct AFTERGLOW_CFG_V2_s
{
    uint16_t version;                         // afterglow version of the configuration
    uint16_t res;                             // reserved bytes
    uint8_t lampGlowDur[NUM_COL][NUM_ROW];    // Lamp matrix glow duration configuration [ms * GLOWDUR_CFG_SCALE]
    uint8_t lampBrightness[NUM_COL][NUM_ROW]; // Lamp matrix maximum brightness configuration (0-7)
    uint32_t crc;                             // data checksum
} AFTERGLOW_CFG_V2_t;


void cfg_init();
void cfg_setDefault(bool directly);
const AFTERGLOW_CFG_t * cfg_config();
AG_DIPSWITCH_t cfg_dipSwitch();
void cfg_updateDipSwitch(uint8_t rawBits);
uint8_t cfg_lastDipSwitchValue();
void cfg_serialConfig(AFTERGLOW_CFG_V2_t *pCfg);
void cfg_setSerialConfig(const AFTERGLOW_CFG_V2_t *pkCfg);
uint32_t cfg_calculateCRC32(const uint8_t *data, uint16_t length);
bool cfg_newConfigAvailable();
bool cfg_applyNewConfig();
bool cfg_loadConfig();
