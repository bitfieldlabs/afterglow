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

extern const uint8_t skMap_256_4[256];
extern const uint8_t skMap_256_8[256];
extern const uint8_t skMap_256_16[256];
extern const uint8_t skMap_256_32[256];
extern const uint8_t skMap_256_64[256];
extern const uint8_t skMap_256_128[256];
extern const uint8_t skMap_256_256[256];

#if (PWM_RES == 4)
#define skBrightnessMap skMap_256_4
#elif (PWM_RES == 8)
#define skBrightnessMap skMap_256_8
#elif (PWM_RES == 16)
#define skBrightnessMap skMap_256_16
#elif (PWM_RES == 32)
#define skBrightnessMap skMap_256_32
#elif (PWM_RES == 64)
#define skBrightnessMap skMap_256_64
#elif (PWM_RES == 128)
#define skBrightnessMap skMap_256_128
#elif (PWM_RES == 256)
#define skBrightnessMap skMap_256_256
#else
#error "No brightness map defined for this PWM resolution!"
#endif
