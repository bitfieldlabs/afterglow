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
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pindef.h"
#include "afterglow.h"
#include "rowout.h"


//------------------------------------------------------------------------------
// local variables

// The timer is the realtime task of this software
static repeating_timer_t sHeartbeatTimer;


//------------------------------------------------------------------------------
bool heartbeat(struct repeating_timer *t)
{
    // afterglow update
    ag_update();
    return true;
}

//------------------------------------------------------------------------------
void panic_mode()
{
    // endless panic
    while (true)
    {
        // blinking alert
        gpio_put(AGPIN_STAT_LED, 1);
        sleep_ms(200);
        gpio_put(AGPIN_STAT_LED, 0);
        sleep_ms(200);
    }
}

//------------------------------------------------------------------------------
int main(void)
{  
    stdio_init_all();

    printf("\n\nAfterglow RP2040 v0.1\n");

    // initialize the status LED
    gpio_init(AGPIN_STAT_LED);
    gpio_set_dir(AGPIN_STAT_LED, GPIO_OUT);

    // data input pin
    gpio_init(AGPIN_IN_DATA);
    gpio_set_dir(AGPIN_IN_DATA, GPIO_IN);
    gpio_set_pulls(AGPIN_IN_DATA, true, false); // pull up

    // output pins
    for (uint32_t p=0; p<NUM_COL_PINS; p++)
    {
        uint pin = skAGColOutPins[p];
        gpio_init(pin);
        gpio_put(pin, false);
        gpio_set_dir(pin, GPIO_OUT);
    }
    for (uint32_t p=0; p<NUM_ROW_PINS; p++)
    {
        uint pin = skAGRowOutPins[p];
        gpio_init(pin);
        gpio_put(pin, false);
        gpio_set_dir(pin, GPIO_OUT);
    }
    gpio_init(AGPIN_OUT_CLK);
    gpio_put(AGPIN_OUT_CLK, false);
    gpio_set_dir(AGPIN_OUT_CLK, GPIO_OUT);
    gpio_init(AGPIN_OUT_DATA);
    gpio_put(AGPIN_OUT_DATA, false);
    gpio_set_dir(AGPIN_OUT_DATA, GPIO_OUT);
    gpio_init(AGPIN_OUT_LOAD);
    gpio_put(AGPIN_OUT_LOAD, false);
    gpio_set_dir(AGPIN_OUT_LOAD, GPIO_OUT);

    gpio_init(AGPIN_IN_CLK);
    gpio_put(AGPIN_IN_CLK, false);
    gpio_set_dir(AGPIN_IN_CLK, GPIO_OUT);
    gpio_init(AGPIN_IN_LOAD);
    gpio_put(AGPIN_IN_LOAD, false);
    gpio_set_dir(AGPIN_IN_LOAD, GPIO_OUT);

    gpio_init(AGPIN_D_SCL);
    gpio_put(AGPIN_D_SCL, false);
    gpio_set_dir(AGPIN_D_SCL, GPIO_OUT);
    gpio_init(AGPIN_D_SDA);
    gpio_put(AGPIN_D_SDA, false);
    gpio_set_dir(AGPIN_D_SDA, GPIO_OUT);

    // set up the row output PIO
    if (!rowout_initpio())
    {
        panic_mode();
    }

    // afterglow init
    ag_init();

    // Heartbeat setup
    if (!add_repeating_timer_us(-TTAG_INT, heartbeat, NULL, &sHeartbeatTimer))
    {
        printf("Failed to start the heartbeat!\n");
        panic_mode();
    }

    // Eternal loop
    while (true)
    {
        // afterglow serial communication
        ag_sercomm();

        // every device needs a blinking LED
        gpio_put(AGPIN_STAT_LED, 1);
        sleep_ms(500);
        gpio_put(AGPIN_STAT_LED, 0);
        sleep_ms(500);
    }
}
