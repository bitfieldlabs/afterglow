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

// RAM data buffer [data words]
#define REC_RAMBUF_SIZE (1024)

void record_init();
void record_start();
void record_stop();
bool record_ready();
bool record_active();
bool record_add(uint32_t data);
bool record_write_flash(const uint8_t *pkData, uint32_t size);
bool record_isrecdata(const uint8_t *pkData);
uint32_t record_percentage();
uint32_t record_replay();
uint32_t replay_percentage();
uint32_t record_replay_size();
const uint8_t *record_data();
