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

#include "pindef.h"

const uint8_t skAGColOutPins[NUM_COL_PINS] = { AGPIN_CO1, AGPIN_CO2, AGPIN_CO3, AGPIN_CO4, AGPIN_CO5, AGPIN_CO6, AGPIN_CO7, AGPIN_CO8 };
const uint8_t skAGRowOutPins[NUM_ROW_PINS] = { AGPIN_RO1, AGPIN_RO2, AGPIN_RO3, AGPIN_RO4, AGPIN_RO5, AGPIN_RO6, AGPIN_RO7, AGPIN_RO8, AGPIN_RO9, AGPIN_RO10 };
