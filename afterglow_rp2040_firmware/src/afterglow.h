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

#include <stdio.h>

// status enumeration
typedef enum AFTERGLOW_STATUS_e
{
    AG_STATUS_INIT = 0,    // initialising
    AG_STATUS_OK,          // up and running
    AG_STATUS_PASSTHROUGH, // ready in pass-through mode
    AG_STATUS_TESTMODE,    // ready in test mode
    AG_STATUS_REPLAY,      // ready in replay mode
    AG_STATUS_INVINPUT     // invalid input
} AFTERGLOW_STATUS_t;

typedef enum AFTERGLOW_MODE_e
{
    AG_MODE_UNKNOWN = 0,   // Mode not set
    AG_MODE_WPC = 1,       // WPC, Data East, Sys11 mode (8 column multiplexing)
    AG_MODE_WHITESTAR = 2  // Stern Whitestar / S.A.M. mode (10 rows multiplexing) 
} AFTERGLOW_MODE_t;


AFTERGLOW_STATUS_t ag_status();
void ag_statusUpdate();

AFTERGLOW_MODE_t ag_mode();
void ag_setMode(AFTERGLOW_MODE_t mode);
