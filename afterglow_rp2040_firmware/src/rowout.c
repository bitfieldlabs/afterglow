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

#include "rowout.h"
#include "hardware/pio.h"
#include "pindef.h"
#include "rowout.pio.h"


//------------------------------------------------------------------------------
// Local data

// Column/row reading, blanking and WS2812 on PIO 0
static PIO sPioRowOut = pio0;
static int sSmRowOut = -1;
static int sSmRowOutOffset = -1;


//------------------------------------------------------------------------------
void rowout_prepareData(uint8_t rowDur[NUM_ROW])
{

}

//------------------------------------------------------------------------------
bool rowout_initpio()
{
    // Find a place for the PIO program in the instruction memory
    sSmRowOutOffset = pio_add_program(sPioRowOut, &rowout_program);
    // Claim an unused state machine for the row output and run the program
    sSmRowOut = pio_claim_unused_sm(sPioRowOut, true);
    rowout_program_init(sPioRowOut, sSmRowOut, sSmRowOutOffset);

    return (sSmRowOut != -1);
}
