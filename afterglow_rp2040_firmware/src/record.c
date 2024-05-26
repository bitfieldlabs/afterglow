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

#include "record.h"
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "pico/multicore.h"


//------------------------------------------------------------------------------
// Recording storage definitions

// Use a flash region 1Mb from the flash start for storing the configuration
#define REC_FLASH_OFFSET (1000 * 1024)

// Record size (500 * 4096b = 2Mb, records 64 seconds of data)
#define REC_FLASH_SIZE (500 * FLASH_SECTOR_SIZE)

// Pointer to the record storage
const uint8_t *pkFlashRec = (const uint8_t *) (XIP_BASE + REC_FLASH_OFFSET);


//------------------------------------------------------------------------------
// local variables

// data buffer in RAM (double buffering)
static uint32_t sDataBuffer[REC_RAMBUF_SIZE*2] = { 0 };

static uint32_t sRamBufPos = 0;
static uint32_t sReplayPos = 0;
static uint32_t sFlashPos = 0;
static bool sRecordActive = false;
static bool sRecordInit = false;
static bool sRecordInitStarted = false;


//------------------------------------------------------------------------------
void record_init()
{
    // erase the record sectors
    sRecordInitStarted =  true;
    flash_range_erase(REC_FLASH_OFFSET, REC_FLASH_SIZE);
    sRecordInit = true;
    sRecordInitStarted = false;

    printf("REC init\n");
}

//------------------------------------------------------------------------------
void record_start()
{
    sRecordActive = true;
    sRamBufPos = 0;
    sFlashPos = 0;

    printf("REC start\n");
}

//------------------------------------------------------------------------------
void record_stop()
{
    sRecordActive = false;
    printf("REC stop %lX\n", *((const uint32_t*)pkFlashRec));
}

//------------------------------------------------------------------------------
uint32_t record_replay()
{
    // return the current data record
    uint32_t rec = *((const uint32_t*)(pkFlashRec + sReplayPos));

    if (sReplayPos == 0)
    {
        printf("REP %lX\n", rec);
    }

    // advance to next record
    sReplayPos += sizeof(uint32_t);
    if (sReplayPos >= REC_FLASH_SIZE)
    {
        // rewind
        sReplayPos = 0;
    }

    return rec;
}

//------------------------------------------------------------------------------
bool record_active()
{
    return sRecordActive;
}

//------------------------------------------------------------------------------
bool record_isrecdata(const uint8_t *pkData)
{
    return ((pkData >= (const uint8_t*)&sDataBuffer[0]) &&
            (pkData <= (const uint8_t*)&sDataBuffer[REC_RAMBUF_SIZE*2-1]));
}

//------------------------------------------------------------------------------
bool record_add(uint32_t data)
{
    bool added = false;

    // check whether recording is active
    if (sRecordActive)
    {
        // check if storage has been initialized
        if (sRecordInit)
        {
            if ((sFlashPos == 0) && (sRamBufPos < 4))
            {
                printf("REC ADD %lu %lX\n", sRamBufPos, data);
            }

            // add the new data record to the RAM buffer
            sDataBuffer[sRamBufPos++] = data;
            added = true;

            // when the RAM buffer is half full, write the data to flash
            // this is done on CPU1 in order not to interfere with data
            // input
            if (sRamBufPos == REC_RAMBUF_SIZE)
            {
                static bool sPrint1 = false;
                if (!sPrint1)
                {
                    printf("REC PUSH 1 %lu %lX\n", sRamBufPos, sDataBuffer[0]);
                    sPrint1 = true;
                }
                
                if (!multicore_fifo_push_timeout_us((uint32_t)&sDataBuffer[0], 0))
                {
                    printf("!");
                }
            }
            if (sRamBufPos == (2*REC_RAMBUF_SIZE))
            {
                static bool sPrint2 = false;
                if (!sPrint2)
                {
                    printf("REC PUSH 2 %lu %lX\n", sRamBufPos, sDataBuffer[REC_RAMBUF_SIZE]);
                    sPrint2 = true;
                }

                // circular buffer, start again at beginning
                sRamBufPos = 0;

                if (!multicore_fifo_push_timeout_us((uint32_t)&sDataBuffer[REC_RAMBUF_SIZE], 0))
                {
                    printf("!");
                }
            }
        }
    }

    return added;
}

//------------------------------------------------------------------------------
bool record_write_flash(const uint8_t *pkData, uint32_t size)
{
    bool full = false;

    static uint32_t sWriteCounter = 0;

    // write to flash
    if (sRecordInit && (sFlashPos < (REC_FLASH_SIZE-size)))
    {
        if (sFlashPos == 0)
        {
            printf("REC FL %lX %lX %lu ", *((const uint32_t*)pkData),
                *((const uint32_t*)pkFlashRec), size);
        }

        flash_range_program(REC_FLASH_OFFSET+sFlashPos, pkData, size);

        if (sFlashPos == 0)
        {
            printf("PRG %lX\n", *((const uint32_t*)pkFlashRec));
        }

        sFlashPos += size;
    }
    else if (!sRecordInit)
    {
        // initialize the flash region first
        record_init();
    }

    if (sFlashPos >= (REC_FLASH_SIZE-size))
    {
        // recording full, stop
        full = true;
        record_stop();
    }

    return full;
}
