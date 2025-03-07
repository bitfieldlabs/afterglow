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
#include "serial.h"
#include "pico/time.h"
#include "pico/stdio.h"
#include "hardware/watchdog.h"
#include "lampmatrix.h"
#include "matrixout.h"
#include "afterglow.h"
#include "config.h"
#include "def.h"
#include "params.h"
#include "record.h"
#include "smart.h"


//------------------------------------------------------------------------------
// local definitions

#define SER_INPUT_BUF_SIZE 64   // serial input buffer size


//------------------------------------------------------------------------------
// local variables

static uint32_t sLastDebugTTag = 0;
static char sCmd[SER_INPUT_BUF_SIZE] = { 0 };
static uint32_t sCmdPos = 0;
static bool sCmdComplete = false;
static bool sSuspended = false;


//------------------------------------------------------------------------------
// function prototypes

void serial_debug(uint32_t ttag);
void serial_input();
void serial_receiveCfg();
void serial_recSizePoll();
void serial_recSend();


//------------------------------------------------------------------------------
void serial_comm(uint32_t ttag)
{
    // handle input
    serial_input();

#if DEBUG_SERIAL
    // debug output
    if (!sSuspended)
    {
        serial_debug(ttag);
    }
#endif
}

#if DEBUG_SERIAL
//------------------------------------------------------------------------------
void serial_debug(uint32_t ttag)
{
    uint32_t m = to_ms_since_boot(get_absolute_time());
    if ((m - sLastDebugTTag) > 5000)
    {
        // column/row data
        printf("data %08lx mode %d st %d\n", lm_lastInputData(), (int)ag_mode(), ag_status());

        // configuration
        AG_DIPSWITCH_t ds = cfg_dipSwitch();
        printf("cfg : %02x - tm %d rp %d sm %d pt %d\n", cfg_lastDipSwitchValue(),
            ds.testMode, ds.replayMode ? 1 : 0, ds.smartMode, ds.passThrough);

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
        //const uint32_t *pkLM = matrixout_lampMatrix();
        //printf("br  : %lu %lu\n", pkLM[0], pkLM[1]); 

        // update duration
        const AG_PARAMS_t *pkPar = par_params();
        printf("dur : i %lu/%u u %lu/%lu us\n", lm_inputMaxDurAndClear(), INPUT_SAMPLE_INT,
            matrixout_updateMaxDurAndClear(), pkPar->matrixUpdateInt);

        // print smart detection mode results once
        if ((sLastDebugTTag > 5000) && (sLastDebugTTag < 10000))
        {
            smart_detect_print();
        }

        sLastDebugTTag = m;
    }
}
#endif

//------------------------------------------------------------------------------
void serial_input()
{
    // read data from the serial port if available
    int c = 0;
    while ((c != PICO_ERROR_TIMEOUT) && !sCmdComplete)
    {
        c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT)
        {
            char character = (char)c;
            if (character != AG_CMD_TERMINATOR)
            {
                // add the character and wait for the command terminator
                sCmd[sCmdPos] = character;
                sCmdPos++;
                //printf("c - %c\n", character);
                if (sCmdPos >= SER_INPUT_BUF_SIZE)
                {
                    // clear the buffer
                    sCmdPos = 0;
                }
            }
            else
            {
                // command complete
                sCmdComplete = true;
            }           
        }
    }

    if (sCmdComplete)
    {
        // handle the commands

        // version poll
        if (strncmp(sCmd, AG_CMD_VERSION_POLL, 3) == 0)
        {
            // Output the version numbers
            printf("%s %d %d\n", AG_CMD_VERSION_POLL, AFTERGLOW_RP2040_VERSION, AFTERGLOW_CFG_SER_VERSION);
        }

        // configuration poll
        else if (strncmp(sCmd, AG_CMD_CFG_POLL, 4) == 0)
        {
#if (AFTERGLOW_CFG_SER_VERSION == 2)
            // ========= Configuration version 2 =========

            // create the serial configuration structure
            AFTERGLOW_CFG_V2_t cfg;
            cfg_serialConfigV2(&cfg);
            
            // send the full configuration
            uint16_t cfgSize = sizeof(cfg);
            const uint8_t *pkCfg = (const uint8_t*)&cfg;
            for (uint i=0; i<cfgSize; i++)
            {
                putchar_raw(*pkCfg++);
            }
#else
            // ========= Configuration version 3 =========

            // create the serial configuration structure
            AFTERGLOW_CFG_t cfg;
            cfg_serialConfig(&cfg);
            
            // send the full configuration
            uint16_t cfgSize = sizeof(cfg);
            const uint8_t *pkCfg = (const uint8_t*)&cfg;
            for (uint32_t i=0; i<cfgSize; i++)
            {
                putchar_raw((int)(*pkCfg));
                pkCfg++;
            }
#endif
        }

        // configuration reset
        else if (strncmp(sCmd, AG_CMD_CFG_DEFAULT, 4) == 0)
        {
            // reset the configuration to default
            cfg_setDefault(false);

            // acknowledge
            printf("%s\n", AG_CMD_ACK);
        }

        // configuration write
        else if (strncmp(sCmd, AG_CMD_CFG_SAVE, 4) == 0)
        {
            // receive a new configuration
            serial_receiveCfg();
        }

        // recording
        else if (strncmp(sCmd, AG_CMD_RECORD, 4) == 0)
        {
            // start a recording
            record_start();
        }

        // recording size poll
        else if (strncmp(sCmd, AG_CMD_REC_SIZE, 4) == 0)
        {
            // start a recording
            serial_recSizePoll();
        }

        // recording download
        else if (strncmp(sCmd, AG_CMD_REC_DOWNLOAD, 4) == 0)
        {
            // start a recording
            serial_recSend();
        }

        sCmdPos = 0;
        sCmdComplete = false;
    }
}

//------------------------------------------------------------------------------
void serial_receiveCfg()
{
    // wait for the full configuration data
    bool res = false;

#if (AFTERGLOW_CFG_SER_VERSION == 2)
    AFTERGLOW_CFG_V2_t cfg;
#else
    AFTERGLOW_CFG_t cfg;
#endif
    char *pCfg = (char*)&cfg;
    uint32_t cfgSize = sizeof(cfg);
    uint32_t size = 0;

    // read all data
    while (size < cfgSize)
    {
        // send data ready signal and wait for data
        printf("%s\n",AG_CMD_CFG_DATA_READY);
        sleep_ms(100);

        // read data
        uint32_t readBytes = 0;
        int c = 0;
        while ((c != PICO_ERROR_TIMEOUT) && (readBytes < AG_CMD_WRITE_BUF) && (size < cfgSize))
        {
            c = getchar_timeout_us(0);
            if (c != PICO_ERROR_TIMEOUT)
            {
                *pCfg++ = (char)c;
                readBytes++;
                size++;
            }
        }

        // still alive
        watchdog_update();
    }

    if (size == sizeof(cfg))
    {
        // check the crc
        uint32_t crc = cfg_calculateCRC32((uint8_t*)&cfg, size-sizeof(cfg.crc));
        if (crc == cfg.crc)
        {
            // set the new configuration and apply it
#if (AFTERGLOW_CFG_SER_VERSION == 2)
            cfg_setSerialConfigV2(&cfg);
#else
            cfg_setSerialConfig(&cfg);
#endif
            res = true;
        }
#if DEBUG_SERIAL
        else
        {
            printf("CRC FAIL %lu %lu", crc, cfg.crc);
        }
#endif
    }
#if DEBUG_SERIAL
    else
    {
            printf("SIZE MISMATCH: %lu", size);
    }
#endif

    // send ACK/NACK
    printf("%s\n", res ? AG_CMD_ACK : AG_CMD_NACK);
}

//------------------------------------------------------------------------------
void serial_recSizePoll()
{
    uint32_t recSize = record_replay_size();
    printf("%s %lu\n", AG_CMD_REC_SIZE, recSize);
}

//------------------------------------------------------------------------------
void serial_recSend()
{
    const uint8_t *pkRecData = record_data();
    uint32_t recSize = record_replay_size();
    uint32_t sent = 0;
    uint32_t chunkSize = 4096*8;

    // suspend serial output as we're sending the data non-blocking
    serial_suspend();

    // send all data in chunks
    while (sent < recSize)
    {
        // crop the last chunk
        uint32_t s = chunkSize;
        uint32_t newSent = (sent+s);
        if (newSent > recSize)
        {
            s -= (newSent-recSize);
        }

        // send the data
        //fwrite(pkRecData, sizeof(uint8_t), s, stdout);
        //pkRecData += chunkSize;
        for (uint i=0; i<s; i++)
        {
            putchar_raw(*pkRecData++);
        }
        
        sent += chunkSize;
        

        // still alive
        watchdog_update();
    }

    // resume serial output
    sleep_ms(200);
    serial_resume();
}

//------------------------------------------------------------------------------
void serial_suspend()
{
    sSuspended = true;
}

//------------------------------------------------------------------------------
void serial_resume()
{
    sSuspended = false;
}