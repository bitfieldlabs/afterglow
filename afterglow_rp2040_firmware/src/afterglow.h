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

// status enumeration
typedef enum AFTERGLOW_STATUS_e
{
    AG_STATUS_INIT = 0,    // initialising
    AG_STATUS_OK,          // up and running
    AG_STATUS_PASSTHROUGH, // ready in pass-through mode
    AG_STATUS_TESTMODE,    // ready in test mode
    AG_STATUS_REPLAY,      // ready in replay mode
    AG_STATUS_RECORDREADY, // ready to record
    AG_STATUS_RECORD,      // recording
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
