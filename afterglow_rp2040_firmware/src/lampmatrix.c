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
#include "lampmatrix.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "def.h"
#include "pindef.h"
#include "matrixout.h"
#include "bmap.h"


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


static uint16_t sLampMatrix[NUM_COL][NUM_ROW] = { 0 };

static volatile AFTERGLOW_STATUS_t sStatus = AG_STATUS_INIT;
static volatile AFTERGLOW_STATUS_t sLastStatus = AG_STATUS_INIT;

static volatile uint32_t sLastData = 0;


//------------------------------------------------------------------------------
// function prototypes

static uint32_t lm_dataRead();


//------------------------------------------------------------------------------
void lm_init()
{
    memset(sLampMatrix, 0, sizeof(sLampMatrix));

    uint16_t b = 0;
    for (uint c=0; c<NUM_COL; c++)
    {
        for (uint r=0; r<NUM_ROW; r++)
        {
            sLampMatrix[c][r] = b;
            b += 1024;
        }
    }

    // enable serial output at 115200 baudrate
    printf("afterglow RP2040 v%d  (c) 2024 bitfield labs\n", AFTERGLOW_RP2040_VERSION);
}

//------------------------------------------------------------------------------
void lm_inputUpdate()
{
    // send the prepared data

    // sample the input data
    uint32_t dataIn = lm_dataRead();
    sLastData = dataIn;
}

//------------------------------------------------------------------------------
uint32_t lm_dataRead()
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
        data+= 17;
        data-= 17;
        data |= gpio_get(AGPIN_IN_DATA) ? 1 : 0;   // read data bit
        gpio_put(AGPIN_IN_CLK, true);              // CLK high
        data <<= 1;
    }

    return data;
}

//------------------------------------------------------------------------------
/*
void ag_sercomm()
{
    uint32_t m = to_ms_since_boot(get_absolute_time());
    if ((m - sLastDebugTTag) > 2000)
    {
        printf("data: %08lx\n", sLastData);
        sLastDebugTTag = m;
    }
}
*/
