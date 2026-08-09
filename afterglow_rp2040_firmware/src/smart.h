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

#include "stdlib.h"


typedef enum LAMP_TYPE_e
{
    LAMP_TYPE_UNKNOWN = 0,
    LAMP_TYPE_NONE,         // no lamp
    LAMP_TYPE_TINYLED,      // small LED (debug hat)
    LAMP_TYPE_LED,          // LED
    LAMP_TYPE_INC,          // incandescent
    LAMP_TYPE_SHORT         // short detected
} LAMP_TYPE_t;


// Auto-detect incandescents and shorts
void smart_detect_lamps();
void smart_detect_print();
LAMP_TYPE_t smart_lampType(uint32_t col, uint32_t row);
