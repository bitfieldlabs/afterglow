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
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
