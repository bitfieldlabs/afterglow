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

#include "matrixout.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "pindef.h"
#include "matrixout.pio.h"


//------------------------------------------------------------------------------
// Local data

// Matrix output on PIO 0
static PIO sPioMatrixOut = pio0;
static int sSmMatrixOut = -1;
static int sSmMatrixOutOffset = -1;


//------------------------------------------------------------------------------
void matrixout_prepareData(uint col, uint8_t rowDur[NUM_ROW])
{

}

//------------------------------------------------------------------------------
bool matrixout_initpio()
{
    // Find a place for the PIO program in the instruction memory
    sSmMatrixOutOffset = pio_add_program(sPioMatrixOut, &matrixout_program);
    // Claim an unused state machine for the matrix output and run the program
    sSmMatrixOut = pio_claim_unused_sm(sPioMatrixOut, true);
    matrixout_program_init(sPioMatrixOut, sSmMatrixOut, sSmMatrixOutOffset);

    return (sSmMatrixOut != -1);
}
