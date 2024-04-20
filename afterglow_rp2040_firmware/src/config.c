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
#include "input.h"


//------------------------------------------------------------------------------

// afterglow configuration
static AFTERGLOW_CFG_t sCfg;
static AFTERGLOW_CFG_t sNewCfg;
static bool sNewCfgAvailable = false;

static AG_DIPSWITCH_t sDipSwitch;
static uint8_t sLastDipSwitchValue = 0;


//------------------------------------------------------------------------------
void cfg_init()
{
    // set the default configuration
    cfg_setDefault();

    // sample the input data
    uint32_t dataIn = input_dataRead();

    // process the DIP switch information (bits 19-23 of the input)
    // Bits 0-3: CFG1 - CFG4
    cfg_updateDipSwitch((uint8_t)((dataIn>>18) & 0x0f));
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
        sDipSwitch.highLEDFreq = (rawBits & 0x04) ? true : false;
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
    uint8_t *pGlowDurOn = &sCfg.lampGlowDurOn[0][0];
    uint8_t *pGlowDurOff = &sCfg.lampGlowDurOff[0][0];
    uint8_t *pBrightness = &sCfg.lampBrightness[0][0];
    uint8_t *pDelay = &sCfg.lampDelay[0][0];
    for (uint c=0; c<NUM_COL; c++)
    {
        for (uint r=0; r<NUM_ROW; r++)
        {
            *pGlowDurOn++ = (DEFAULT_GLOWDUR_ON / GLOWDUR_CFG_SCALE);
            *pGlowDurOff++ = (DEFAULT_GLOWDUR_OFF / GLOWDUR_CFG_SCALE);
            *pBrightness++ = DEFAULT_BRIGHTNESS;
            *pDelay++ = DEFAULT_DELAY;
        }
    }

    // calculate the crc
    uint16_t cfgSize = sizeof(sCfg);
    sCfg.crc = cfg_calculateCRC32((uint8_t*)&sCfg, cfgSize-sizeof(sCfg.crc));
}

//------------------------------------------------------------------------------
uint32_t cfg_calculateCRC32(const uint8_t *data, uint16_t length)
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

//------------------------------------------------------------------------------
void cfg_serialConfig(AFTERGLOW_CFG_V2_t *pCfg)
{
    // initialize
    uint16_t cfgSize = sizeof(AFTERGLOW_CFG_V2_t);
    memset(pCfg, 0, cfgSize);

    // convert the internal configuration to the serial config version
    pCfg->version = AFTERGLOW_CFG_SER_VERSION;
    memcpy(pCfg->lampBrightness, sCfg.lampBrightness, sizeof(sCfg.lampBrightness));
    memcpy(pCfg->lampGlowDur, sCfg.lampGlowDurOn, sizeof(sCfg.lampGlowDurOn));
    pCfg->crc = cfg_calculateCRC32((uint8_t*)pCfg, cfgSize-sizeof(pCfg->crc));
}

//------------------------------------------------------------------------------
void cfg_setSerialConfig(const AFTERGLOW_CFG_V2_t *pkCfg)
{
    // Prepare to new configuration and mark it ready. It will be applied at the
    // next matrix thread run.
    memset(&sNewCfg, 0, sizeof(sNewCfg));
    memcpy(sNewCfg.lampBrightness, pkCfg->lampBrightness, sizeof(sNewCfg.lampBrightness));
    // apply the same brightness value to on and off glow duration
    memcpy(sNewCfg.lampGlowDurOn, pkCfg->lampGlowDur, sizeof(sNewCfg.lampGlowDurOn));
    memcpy(sNewCfg.lampGlowDurOff, pkCfg->lampGlowDur, sizeof(sNewCfg.lampGlowDurOff));
    sNewCfg.version = AFTERGLOW_CFG_VERSION;
    sNewCfg.crc = cfg_calculateCRC32((uint8_t*)&sNewCfg, sizeof(sNewCfg)-sizeof(sNewCfg.crc));
    sNewCfgAvailable = true;

    // store the configuration to flash
    // TUDUU
}

//------------------------------------------------------------------------------
bool cfg_applyNewConfig()
{
    bool applied = false;
    if (sNewCfgAvailable)
    {
        // copy the double buffered configuration to the active configuration set
        memcpy(&sCfg, &sNewCfg, sizeof(sCfg));
        sNewCfgAvailable = false;
        applied = true;
    }
    return applied;
}
