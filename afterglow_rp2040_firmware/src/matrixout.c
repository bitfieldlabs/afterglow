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
#include "matrixout.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "def.h"
#include "pindef.h"
#include "matrixout.pio.h"
#include "afterglow.h"
#include "bmap.h"
#include "config.h"


//------------------------------------------------------------------------------
// Local definitions

//
//   ANTI       ROW
//   GHOST      PWM
// |---------|-----------------------------------------------------|
//           |<------------------ PWM_RES steps ------------------>|
// |<------->| ANTI_GHOSTING_STEPS
// |<------------------ MATRIXOUT_PIO_STEPS ---------------------->|

// Steps of the matrix PIO for anti-ghosting
#define ANTI_GHOSTING_STEPS ((ANTIGHOST_DURATION * PWM_RES) / (LED_UPDATE_DUR - ANTIGHOST_DURATION))

// Steps per matrix output PIO run
#define MATRIXOUT_PIO_STEPS (ANTI_GHOSTING_STEPS + PWM_RES)



//------------------------------------------------------------------------------
// Local data

// Matrix output on PIO 0
static PIO sPioMatrixOut = pio0;
static int sSmMatrixOut = -1;
static int sSmMatrixOutOffset = -1;

static int sDMAChan = -1;
static dma_channel_config sDMAChanConfig;

// The final lamp brightness matrix
static uint32_t sLampMatrix[NUM_COL][NUM_ROW] = { 0 };

// The lamp brightness steps matrix
static uint32_t sLampMatrixSteps[NUM_COL][NUM_ROW] = { 0 };

// The lamp maximum brightness matrix
static uint32_t sLampMatrixMaxBr[NUM_COL][NUM_ROW] = { 0 };

// local lamp matrix copy
static uint32_t sLampMatrixCopy[NUM_COL] = { 0 };

// use double buffering for the prepared output data
static uint32_t sMatrixDataBuf1[NUM_COL*MATRIXOUT_PIO_STEPS] = { 0 };
static uint32_t sMatrixDataBuf2[NUM_COL*MATRIXOUT_PIO_STEPS] = { 0 };
static volatile uint32_t *pMatrixDataBufOut = sMatrixDataBuf1;
static volatile uint32_t *pMatrixDataBufPrep = sMatrixDataBuf2;


//------------------------------------------------------------------------------
// local functions

void matrixout_prepareCol(uint col, uint8_t *pRowDur);
void matrixout_prepareData(const uint32_t *pkLM);
void matrixout_swapbuf();
void matrixout_updateLampMatrix(const uint32_t *pkRawLM);
void matrixout_prepareBrightnessSteps();


//------------------------------------------------------------------------------
void matrixout_thread()
{
    // initialize the data
    memset(sLampMatrix, 0, sizeof(sLampMatrix));

    // prepare the brightness steps matrix
    matrixout_prepareBrightnessSteps();

    while (true)
    {
        // wait until CPU0 triggers the data output preparation
        const uint32_t *pkRawLM = (const uint32_t*)multicore_fifo_pop_blocking();

        // make a local copy of the raw map matrix
        memcpy(sLampMatrixCopy, pkRawLM, sizeof(sLampMatrixCopy));

        // update the lamp brightness matrix based on the new raw matrix data
        matrixout_updateLampMatrix(sLampMatrixCopy);

        // prepare the PIO buffer
        matrixout_prepareData(&sLampMatrix[0][0]);
    }
}

//------------------------------------------------------------------------------
void matrixout_swapbuf()
{
    // swap between the two output data buffers
    if (pMatrixDataBufPrep == sMatrixDataBuf1)
    {
        pMatrixDataBufOut = sMatrixDataBuf1;
        pMatrixDataBufPrep = sMatrixDataBuf2;
    }
    else
    {
        pMatrixDataBufOut = sMatrixDataBuf2;
        pMatrixDataBufPrep = sMatrixDataBuf1;
    }
}

//------------------------------------------------------------------------------
void matrixout_prepareData(const uint32_t *pkLM)
{
    // No output with invalid data
    AFTERGLOW_STATUS_t s = ag_status();
    if ((s != AG_STATUS_INIT) && (s != AG_STATUS_INVINPUT))
    {
        uint8_t rowDur[NUM_ROW];

        // process column by column
        for (uint c=0; c<NUM_COL; c++)
        {
            uint8_t *pRD = rowDur;
            for (uint r=0; r<NUM_ROW; r++, pkLM++, pRD++)
            {
                // decimate the lamp brightness value to 8 bits
                uint8_t rb = (uint8_t)(*pkLM >> 24);

                // map the brightness value
                *pRD = skBrightnessMap[rb];
            }

            // prepare the output data
            matrixout_prepareCol(c, rowDur);
        }
    }
    else
    {
        memset(pMatrixDataBufPrep, 0, sizeof(sMatrixDataBuf1));
    }

    // swap the output/prepare buffers (double buffering)
    matrixout_swapbuf();
}

//------------------------------------------------------------------------------
void matrixout_prepareCol(uint col, uint8_t *pRowDur)
{
    // Anti-ghosting: turn off everything for some time
    // No need to do anything here - sMatrixDataBuf has already an empty buffer
    // of ANTI_GHOSTING_STEPS entries at the beginning

    // prepare the column and row data for all PWM steps
    uint32_t rbit = 0x00000100;
    uint32_t colBit = (1ul << col);
    uint32_t *pDS = &pMatrixDataBufPrep[col*MATRIXOUT_PIO_STEPS+ANTI_GHOSTING_STEPS];
    uint32_t *pD = pDS;
    for (uint s=0; s<PWM_RES; s++)
    {
        *pD++ = colBit;
    }
    for (uint r=0; r<NUM_ROW; r++)
    {
        uint rd = *pRowDur;
        pD = pDS;
        uint32_t rbitm = ~rbit;
        uint s = rd;
        while (s--)
        {
            *pD |= rbit;
            pD++;
        }
        s = (PWM_RES-rd);
        while (s--)
        {
            *pD &= rbitm;
            pD++;
        }
        pRowDur++;
        rbit <<= 1;
    }
}

//------------------------------------------------------------------------------
void matrix_transfer_complete()
{
    // clear the interrupt request
    dma_hw->ints0 = (1u << sDMAChan);

    // set the buffer address and retrigger
    dma_channel_set_read_addr(sDMAChan, pMatrixDataBufOut, true);
}

//------------------------------------------------------------------------------
bool matrixout_initpio()
{
    // find a place for the PIO program in the instruction memory
    sSmMatrixOutOffset = pio_add_program(sPioMatrixOut, &matrixout_program);
    // claim an unused state machine for the matrix output and run the program
    sSmMatrixOut = pio_claim_unused_sm(sPioMatrixOut, true);
    matrixout_program_init(sPioMatrixOut, sSmMatrixOut, sSmMatrixOutOffset);

    // claim and configure a free dma channel
    sDMAChan = dma_claim_unused_channel(false);
    sDMAChanConfig = dma_channel_get_default_config(sDMAChan);
    channel_config_set_transfer_data_size(&sDMAChanConfig, DMA_SIZE_32);
    channel_config_set_read_increment(&sDMAChanConfig, true);    // Read pointer increments with every step
    channel_config_set_write_increment(&sDMAChanConfig, false);  // Write pointer always points to PIO FIFO
    channel_config_set_dreq(&sDMAChanConfig, pio_get_dreq(sPioMatrixOut, sSmMatrixOut, true));  // Wait for the PIO to finish before writing

    dma_channel_configure(
        sDMAChan,                           // channel to be configured
        &sDMAChanConfig,                    // the channel's configuration
        &sPioMatrixOut->txf[sSmMatrixOut],  // write to the PIO TX FIFO
        NULL,                               // the read address will be set later
        count_of(sMatrixDataBuf1),          // number of transfers
        false                               // don't start yet
    );

    // raise IRQ 0 when the transfer finishes
    dma_channel_set_irq0_enabled(sDMAChan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, matrix_transfer_complete);
    irq_set_enabled(DMA_IRQ_0, true);

    // start the infinite loop
    matrix_transfer_complete();

    return ((sSmMatrixOut != -1) && (sDMAChan != -1));
}

//------------------------------------------------------------------------------
void matrixout_updateLampMatrix(const uint32_t *pkRawLM)
{
    for (uint c=0; c<NUM_COL; c++)
    {
        uint32_t rowBit = 0x01;
        for (uint r=0; r<NUM_ROW; r++)
        {
            // Increase or decrease the lamp brightness value depedning on its
            // current state in the raw matrix.
            if ((*pkRawLM) & rowBit)
            {
                // increase brightness
                if (sLampMatrix[c][r] < (sLampMatrixMaxBr[c][r] - sLampMatrixSteps[c][r]))
                {
                    sLampMatrix[c][r] += sLampMatrixSteps[c][r];
                }
                else
                {
                    sLampMatrix[c][r] = sLampMatrixMaxBr[c][r];
                }
            }
            else
            {
                // decrease brightness
                if (sLampMatrix[c][r] > sLampMatrixSteps[c][r])
                {
                    sLampMatrix[c][r] -= sLampMatrixSteps[c][r];
                }
                else
                {
                    sLampMatrix[c][r] = 0;
                }
            }
            rowBit <<= 1;
        }
        pkRawLM++;
    }
}

//------------------------------------------------------------------------------
void matrixout_prepareBrightnessSteps()
{
    // Prepare the brightness increase/decrease values per matrix update for
    // each lamp in the maxtrix. This value will be added in each update step if
    // the lamp is on in the raw matrix, and it will be subtracted from the
    // current brightness if the lamp is off.
    // The brightness is linear in the matrix, it will be translated to
    // non-linear PWM steps when preparing the PWM data.
    // This is the magic sauce for the afterglow effect.
    const AFTERGLOW_CFG_t *pkCfg = cfg_config();
    for (uint c=0; c<NUM_COL; c++)
    {
        for (uint r=0; r<NUM_ROW; r++)
        {
            // configured glow duration for this lamp [us]
            uint32_t gd = (pkCfg->lampGlowDur[c][r] * GLOWDUR_CFG_SCALE) * 1000;

            // maximum brightness (32 bits), 8 steps only
            uint32_t maxBr = ((uint32_t)0xffffffff >> 3) * (uint32_t)(pkCfg->lampBrightness[c][r] + 1);
            sLampMatrixMaxBr[c][r] = maxBr;

            // calculate the step size for this lamp
            float numSteps = ((float)gd / (float)MATRIX_UPDATE_INT);
            sLampMatrixSteps[c][r] = (uint32_t)((float)maxBr / numSteps);

            printf("ST %d %d %lu %lu\n", c, r, sLampMatrixMaxBr[c][r], sLampMatrixSteps[c][r]);
        }
    }
}

//------------------------------------------------------------------------------
const uint32_t * matrixout_lampMatrix()
{
    return &sLampMatrix[0][0];
}
