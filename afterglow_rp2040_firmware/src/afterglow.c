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
#include "afterglow.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pindef.h"

//------------------------------------------------------------------------------
// Some definitions

// Afterglow configuration version
#define AFTERGLOW_CFG_VERSION 3

// number of columns in the lamp matrix
#define NUM_COL 8

// number of rows in the lamp matrix
#define NUM_ROW 10

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
// Brightness maps

static const uint8_t skMap_256_4_log[256] =
    {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3};

static const uint8_t skMap_256_8_log[256] =
    {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
        3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5,
        5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7};

static const uint8_t skMap_256_16_log[256] =
    {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
        3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5,
        5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7,
        7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10,
        10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15};

//------------------------------------------------------------------------------
// function prototypes


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

// status enumeration
typedef enum AFTERGLOW_STATUS_e
{
    AG_STATUS_INIT = 0,    // initialising
    AG_STATUS_OK,          // up and running
    AG_STATUS_PASSTHROUGH, // ready in pass-through mode
    AG_STATUS_TESTMODE,    // ready in test mode
    AG_STATUS_REPLAY,      // ready in replay mode
    AG_STATUS_INVINPUT,    // invalid input
    AG_STATUS_OVERRUN      // interrupt overrun
} AFTERGLOW_STATUS_t;


//------------------------------------------------------------------------------
// local variables


static volatile uint32_t sTTag = 0;      // current time tag
static volatile uint32_t sCol = 0;       // currently active column

static uint8_t sLampMatrixState[NUM_COL][NUM_ROW] = { 0 };

static volatile AFTERGLOW_STATUS_t sStatus = AG_STATUS_INIT;
static volatile AFTERGLOW_STATUS_t sLastStatus = AG_STATUS_INIT;

static volatile uint32_t sLastData = 0;
static uint32_t sLastDebugTTag = 0;


//------------------------------------------------------------------------------
// function prototypes

static void ag_handleCol(uint32_t col);
static void ag_dataOutputOff();
static uint32_t ag_dataRead();


//------------------------------------------------------------------------------
void ag_init()
{
    memset(sLampMatrixState, 0, sizeof(sLampMatrixState));
    sTTag = 0;
    sCol = 0;

    // enable serial output at 115200 baudrate
    printf("afterglow RP2040 v%d  (c) 2024 bitfield labs\n", AFTERGLOW_RP2040_VERSION);
}

//------------------------------------------------------------------------------
// This is the realtime task update. All the afterglow magic happens here.
void ag_update()
{
    // handle one column with each update
    ag_handleCol(sCol);

    // sample the input data
    uint32_t dataIn = ag_dataRead();
    sLastData = dataIn;

    // time is flowing
    sTTag++;
    sCol++;
    if (sCol == NUM_COL)
    {
        sCol = 0;
    }
}

//------------------------------------------------------------------------------
void ag_handleCol(uint32_t col)
{
    // turn off everything briefly to avoid ghosting
    ag_dataOutputOff();
    busy_wait_us(ANTIGHOST_DURATION);

    // turn on current column
    gpio_put(skAGColOutPins[sCol], true);

    // enable the rows
    gpio_put(skAGRowOutPins[0], true);
    gpio_put(skAGRowOutPins[1], true);
    gpio_put(skAGRowOutPins[2], true);
    gpio_put(skAGRowOutPins[3], true);
    gpio_put(skAGRowOutPins[4], true);
    gpio_put(skAGRowOutPins[5], true);
    gpio_put(skAGRowOutPins[6], true);
    gpio_put(skAGRowOutPins[7], true);
}

//------------------------------------------------------------------------------
void ag_dataOutputOff()
{
    // turn all columns and row drives off
    gpio_put_masked(AGPIN_OUT_ALL_MASK, false);
}

//------------------------------------------------------------------------------
uint32_t ag_dataRead()
{
    // drive CLK and LOAD low
    gpio_put(AGPIN_IN_CLK, false);
    gpio_put(AGPIN_IN_LOAD, false);
    
    // wait some time
    uint32_t data = 0;
    data+= 17;
    data-= 17;
    
    // drive LOAD high to save pin states
    gpio_put(AGPIN_IN_LOAD, true);
    
    // clock in all 24 data bits from the shift register
    for (uint i=0; i<24; i++)
    {
        gpio_put(AGPIN_IN_CLK, false);             // CLK low
        data |= gpio_get(AGPIN_IN_DATA) ? 1 : 0;   // read data bit
        gpio_put(AGPIN_IN_CLK, true);              // CLK high
        data <<= 1;
    }

    return data;
}

//------------------------------------------------------------------------------
void ag_sercomm()
{
    uint32_t m = to_ms_since_boot(get_absolute_time());
    if ((m - sLastDebugTTag) > 5000)
    {
        printf("data: %d\n", sLastData);
        sLastDebugTTag = m;
    }
}
