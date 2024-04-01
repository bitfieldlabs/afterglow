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
#include "bmap.h"


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
#define ANTI_GHOSTING_STEPS ((ANTIGHOST_DURATION * PWM_RES) / (MATRIX_UPDATE_INT - ANTIGHOST_DURATION))

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

// local lamp matrix copy
static uint16_t sLampMatrixCopy[NUM_COL][NUM_ROW] = { 0 };

// use double buffering for the prepared output data
static uint32_t sMatrixDataBuf1[NUM_COL*MATRIXOUT_PIO_STEPS] = { 0 };
static uint32_t sMatrixDataBuf2[NUM_COL*MATRIXOUT_PIO_STEPS] = { 0 };
static uint32_t * sMatrixDataBufOut = sMatrixDataBuf1;
static uint32_t * sMatrixDataBufPrep = sMatrixDataBuf2;


//------------------------------------------------------------------------------
// local functions

void matrixout_prepareCol(uint col, uint8_t *pRowDur);
void matrixout_prepareData(const uint16_t *pkLM);
void matrixout_swapbuf();


//------------------------------------------------------------------------------
void matrixout_thread()
{
    while (true)
    {
        // wait until CPU0 triggers the data output preparation
        const uint16_t *pkLM = (const uint16_t*)multicore_fifo_pop_blocking();

        // make a local copy of the map matrix
        memcpy(&sLampMatrixCopy[0][0], pkLM, sizeof(sLampMatrixCopy));

        // process the whole matrix data
        matrixout_prepareData(&sLampMatrixCopy[0][0]);
    }
}

//------------------------------------------------------------------------------
void matrixout_swapbuf()
{
    // swap between the two output data buffers
    if (sMatrixDataBufPrep == sMatrixDataBuf1)
    {
        sMatrixDataBufOut = sMatrixDataBuf1;
        sMatrixDataBufPrep = sMatrixDataBuf2;
    }
    else
    {
        sMatrixDataBufOut = sMatrixDataBuf2;
        sMatrixDataBufPrep = sMatrixDataBuf1;
    }
}

//------------------------------------------------------------------------------
void matrixout_prepareData(const uint16_t *pkLM)
{
    uint8_t rowDur[NUM_ROW];

    // process column by column
    for (uint c=0; c<NUM_COL; c++)
    {
        uint8_t *pRD = rowDur;
        for (uint r=0; r<NUM_ROW; r++, pkLM++, pRD++)
        {
            // decimate the lamp brightness value to 8 bits
            uint8_t rb = (*pkLM >> 8);

            // map the brightness value
            *pRD = skBrightnessMap[rb];
        }

        // prepare the output data
        matrixout_prepareCol(c, rowDur);

        // swap the output/prepare buffers (double buffering)
        matrixout_swapbuf();
    }
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
    uint32_t *pDS = &sMatrixDataBufPrep[col*MATRIXOUT_PIO_STEPS];
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
    dma_channel_set_read_addr(sDMAChan, sMatrixDataBufOut, true);
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
        sMatrixDataBufOut,                  // read from the output data buffer
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
