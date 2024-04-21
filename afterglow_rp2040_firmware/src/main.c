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

/*

              +------CPU 0-----+       +------CPU 1-----+        +-------PIO------+


              +----------------+
              | main()         |
              |  + Setup       |
              |  | Status      |
              |  + Serial      |
              |                |
              +----------------+                                   LED update: 1kHz
                                                                PIO out: 128*8*1kHz
                     Timer 4 kHz                   100 Hz                      1MHz
              +----------------+       +----------------+        +----------------+
              | sample_input() |       | matrixout()    |        | matrixout PIO  |
              |                |       |                |        |                |
Col/Row +---->+  o raw matrix  +-------+  o brightness  +--------+  o PWM raw data|
Input         |    1bit        |       |    matrix      |        |    PWM_RES *   |
              |                |       |    16bits      |        |    NUM_COL     |
              |                |       |                |        |                |
              |.sRawLampMatrix |       |   .sLampMatrix |        |.sMatrixDataBuf1|
              |                |       |                |        |.sMatrixDataBuf2|
              +----------------+       +----------------+        +----------------+

*/



 
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/watchdog.h"
#include "pindef.h"
#include "lampmatrix.h"
#include "matrixout.h"
#include "serial.h"
#include "afterglow.h"
#include "config.h"
#include "params.h"


//------------------------------------------------------------------------------
// local variables

// The timer is the trigger for the input sampling and the matrix update
static repeating_timer_t sInputSamplingTimer;
static volatile uint32_t sTtag = 0;
static uint32_t sMatrixUpdateCounter = 0;
static AG_DIPSWITCH_t sLastDIPSwitch = { 0 };


//------------------------------------------------------------------------------
// local functions

void checkForConfigChanges();


//------------------------------------------------------------------------------
bool sample_input(struct repeating_timer *t)
{
    // sample and process the the inputs
    lm_inputUpdate(sTtag);

    // trigger an update of the output data
    if (sMatrixUpdateCounter == 0)
    {
        // send the current matrix state to the thread on CPU1
        multicore_fifo_push_blocking((uint32_t)lm_rawLampMatrix());

        // restart the update counter
        sMatrixUpdateCounter = SAMPLE_TO_UPDATE_RATIO;
    }
    else
    {
        // no matrix update this time
        sMatrixUpdateCounter--;
    }

    // time is ticking
    sTtag++;

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
    stdio_usb_init();
    stdio_init_all();

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

    // configuration initialisation
    cfg_init();
    sLastDIPSwitch = cfg_dipSwitch();

    // parameters initialisation
    par_setDefault();

    // set up the matrix output DMA and PIO
    if (!matrixout_initpio())
    {
        panic_mode();
    }

    // prepare the brightness steps
    matrixout_prepareBrightnessSteps();

    // lamp matrix initialisation
    lm_init();

    // start a thread on CPU1, used for matrix data preparation
    multicore_launch_core1(matrixout_thread);

    // Input sampling setup
    if (!add_repeating_timer_us(-INPUT_SAMPLE_INT, sample_input, NULL, &sInputSamplingTimer))
    {
        printf("Failed to start the input handler timer!\n");
        panic_mode();
    }

    // enable serial output at 115200 baudrate
    printf("afterglow rp2040 v%d  (c) 2024 bitfield labs\n", AFTERGLOW_RP2040_VERSION);

    // Eternal loop
    while (true)
    {
        // handle configuration changes
        checkForConfigChanges();

        // afterglow serial communication
        serial_comm(sTtag);

        // status update
        ag_statusUpdate();

        // time for a nap
        sleep_ms(50);
    }
}

//------------------------------------------------------------------------------
void checkForConfigChanges()
{
    AG_DIPSWITCH_t dipSwitch = cfg_dipSwitch();

    // LED frequency configuration changes
    if (dipSwitch.highLEDFreq != sLastDIPSwitch.highLEDFreq)
    {
        // reboot, the new configuration will be applied at startup
        watchdog_reboot(0, 0, 0);
    }

    // Serial port configuration
    if (cfg_newConfigAvailable())
    {
        // stop afterglow operation
        cancel_repeating_timer(&sInputSamplingTimer);
        sleep_ms(100);
        multicore_reset_core1();
        sleep_ms(100);
        matrixout_stoppio();
        sleep_ms(100);

        // apply the new configuration and store it to flash
        if (cfg_applyNewConfig())
        {
#if DEBUG_SERIAL
            printf("Cfg saved to flash, rebooting...");
#endif
        }

        // reboot, the new configuration will be loaded from flash at startup
        watchdog_reboot(0, 0, 0);
    }

    sLastDIPSwitch = dipSwitch;
}
