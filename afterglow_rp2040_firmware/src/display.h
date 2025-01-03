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

#include <stdio.h>
#include "def.h"

#if DEBUG_OLED_I2C

// main display modes
typedef enum DISPLAY_MODES_e
{
    DISPLAY_MODE_BLANK = 0,
    DISPLAY_MODE_LOGO,
    DISPLAY_MODE_LAMPDETECT,
    DISPLAY_MODE_REPLAY,
    DISPLAY_MODE_RECORDREADY,
    DISPLAY_MODE_RECORD,
    DISPLAY_MODE_LAMPTYPE,
    DISPLAY_MODE_NOTICE,
    DISPLAY_MODE_INVINPUT
} DISPLAY_MODES_t;


bool display_init();
void display_update();
void display_setMode(DISPLAY_MODES_t mode);
void display_setNotice(const char *pkStr1, const char *pkStr2, const char *pkStr3, uint32_t duration);

#endif
