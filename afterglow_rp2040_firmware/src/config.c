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

#include <string.h>
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "config.h"
#include "afterglow.h"
#include "def.h"
#include "input.h"


//------------------------------------------------------------------------------
// Config definitions

// Use a flash region 252k from the flash start for storing the configuration
#define CFG_FLASH_OFFSET (252 * 1024)

// Pointer to the flash content for *reading*
const uint8_t *pkFlashCfg = (const uint8_t *) (XIP_BASE + CFG_FLASH_OFFSET);


//------------------------------------------------------------------------------

// afterglow configuration
static AFTERGLOW_CFG_t sCfg;
static AFTERGLOW_CFG_t sNewCfg;
static bool sNewCfgAvailable = false;
static uint8_t sFlashWriteBuf[FLASH_PAGE_SIZE*4] = { 0 };

static AG_DIPSWITCH_t sDipSwitch;
static uint8_t sLastDipSwitchValue = 0xff;


//------------------------------------------------------------------------------
// local functions

bool cfg_saveToFlash();


//------------------------------------------------------------------------------
void cfg_init()
{
    // try to load the configuration from flash
    if (cfg_loadConfig())
    {
        printf("Cfg loaded from flash");
    }
    else
    {
        // set the default configuration
        cfg_setDefault(true);
        printf("Default cfg set");
    }

    // sample the input data
    uint32_t dataIn = input_dataRead();

    // process the DIP switch information (bits 19-23 of the input)
    // Bits 0-3: CFG1 - CFG4
    cfg_updateDipSwitch((uint8_t)((dataIn>>18) & 0x0f));

    sNewCfgAvailable = false;
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
    // invert the raw bits as a HIGH input means the switch is OFF
    rawBits = (~rawBits & 0x0f);

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
        bool newReplaytMode = (rawBits & 0x02) ? true : false;
        if (sDipSwitch.replayMode != newReplaytMode)
        {
            sDipSwitch.replayMode = newReplaytMode;

            // reset the AG mode when the replay mode configuration changes -
            // replay mode and real input may use different modes
            ag_setMode(AG_MODE_UNKNOWN);
        }
        sDipSwitch.smartMode = (rawBits & 0x04) ? true : false;
        sDipSwitch.passThrough = (rawBits & 0x08) ? true : false;
    }
    sLastDipSwitchValue = rawBits;
}

//------------------------------------------------------------------------------
uint8_t cfg_lastDipSwitchValue()
{
    return sLastDipSwitchValue;
}

//------------------------------------------------------------------------------
void cfg_setDefault(bool directly)
{
    AFTERGLOW_CFG_t *pCfg = (directly) ? &sCfg : &sNewCfg;

    // initialize configuration to default values
    memset(pCfg, 0, sizeof(sCfg));
    pCfg->version = AFTERGLOW_CFG_VERSION;
    uint8_t *pGlowDurOn = &(pCfg->lampGlowDurOn[0][0]);
    uint8_t *pGlowDurOff = &(pCfg->lampGlowDurOff[0][0]);
    uint8_t *pBrightness = &(pCfg->lampBrightness[0][0]);
    uint8_t *pDelay = &(pCfg->lampDelay[0][0]);
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
    pCfg->crc = cfg_calculateCRC32((uint8_t*)pCfg, cfgSize-sizeof(pCfg->crc));

    // mark the buffered configuration as ready for application
    if (!directly)
    {
        sNewCfgAvailable = true;
    }
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
bool cfg_newConfigAvailable()
{
    return sNewCfgAvailable;
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

        // store the configuration to flash
        applied = cfg_saveToFlash();
    }   

    return applied;
}

//------------------------------------------------------------------------------
bool cfg_loadConfig()
{
    bool res = false;

    // load the configuration from flash
    memcpy(&sCfg, pkFlashCfg, sizeof(sCfg));

    // check the crc
    uint32_t crc = cfg_calculateCRC32((uint8_t*)&sCfg, sizeof(sCfg)-sizeof(sCfg.crc));
    if (crc == sCfg.crc)
    {
        res = true;
    }
    return res;
}

//------------------------------------------------------------------------------
bool cfg_saveToFlash()
{
    bool saved = false;

    // stop all interrupts
    uint32_t ints = save_and_disable_interrupts();

    // erase a 4kb flash sector
    flash_range_erase(CFG_FLASH_OFFSET, FLASH_SECTOR_SIZE);

    // Write the configuration to flash. The number of bytes written must be a
    // multiple of FLASH_PAGE_SIZE (256b)
    memcpy(sFlashWriteBuf, &sCfg, sizeof(sCfg));
    flash_range_program(CFG_FLASH_OFFSET, sFlashWriteBuf, sizeof(sFlashWriteBuf));

    // restore interrupts
    restore_interrupts(ints);

    // verify the data
    if (memcmp(&sCfg, pkFlashCfg, sizeof(sCfg)) == 0)
    {
        saved = true;
    }
    else
    {
#if DEBUG_SERIAL
        printf("Cfg save to flash failed!");
#endif
    }

    return saved;
}
