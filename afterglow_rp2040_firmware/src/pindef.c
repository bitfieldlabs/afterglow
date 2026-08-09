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

#include "pindef.h"

const uint8_t skAGColOutPins[NUM_COL_PINS] = { AGPIN_CO1, AGPIN_CO2, AGPIN_CO3, AGPIN_CO4, AGPIN_CO5, AGPIN_CO6, AGPIN_CO7, AGPIN_CO8 };
const uint8_t skAGRowOutPins[NUM_ROW_PINS] = { AGPIN_RO1, AGPIN_RO2, AGPIN_RO3, AGPIN_RO4, AGPIN_RO5, AGPIN_RO6, AGPIN_RO7, AGPIN_RO8, AGPIN_RO9, AGPIN_RO10 };
