/***********************************************************************
 *  afterglow:
 *      Copyright (c) 2018-2022 bitfield labs
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
 
//------------------------------------------------------------------------------
/* This code assumes following pin layout:
 *
 *  +----------+---------------+-----------+---------------+--------------+
 *  | Name     | Function      | Nano Pin# | Register, Bit | Mode         |
 *  +----------+---------------+-----------+---------------+--------------+
 *  | IN_DATA  | 74LS165 Q_H   | D2        | DDRD, 2       | Input        |
 *  | IN_CLK   | 74LS165 CLK   | D3        | DDRD, 3       | Output       |
 *  | IN_LOAD  | 74LS165 LD    | D4        | DDRD, 4       | Output       |
 *  | OUT_DATA | 74HC595 SER   | D5        | DDRD, 5       | Output       |
 *  | OUT_CLK  | 74LS595 SRCLK | D6        | DDRD, 6       | Output       |
 *  | OUT_LOAD | 74LS595 RCLK  | D7        | DDRD, 7       | Output       |
 *  | OE       | 74LS595 OE    | A1        | DDRC, 1       | Output       |
 *  | R9_IN    | Row 9 IN (WS) | D12       | DDRB, 5       | Input Pullup |
 *  | R10_IN   | Row 10 IN (WS)| A4        | DDRC, 5       | Input Pullup |
 *  | R9_OUT   | Row 9 OUT (WS)| A2        | DDRC, 3       | Output       |
 *  | R10_OUT  | Row 10 OUT(WS)| A3        | DDRC, 4       | Output       |
 *  | TEST1    | TESTMODE 1    | D8        | DDRB, 1       | Input Pullup |
 *  | TEST2    | TESTMODE 2    | D9        | DDRB, 2       | Input Pullup |
 *  | TEST3    | TESTMODE 3    | D10       | DDRB, 3       | Input Pullup |
 *  | TEST4    | TESTMODE 4    | D11       | DDRB, 4       | Input Pullup |
 *  | CM       | CURRENT MEAS  | A0        | DDRC, 0       | Input        |
 *  | WS2812   | RGB LED (WS)  | A0        | DDRC, 0       | Output       |
 *  +----------+---------------+-----------+---------------+--------------+
*/

#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/boot.h>

//------------------------------------------------------------------------------
// Setup

#define AFTERGLOW_VERSION       110     // Afterglow version number
#define AFTERGLOW_WHITESTAR       0     // Enable Afterglow Whitestar mode (PCB version >=2.0) when set to 1
#define AFTERGLOW_BOARD_REV      15     // Latest supported Afterglow WPC/Sys11/DE board revision
#define AFTERGLOW_WS_BOARD_REV   21     // Latest supported Afterglow Whitestar/S.A.M. board revision
#define SINGLE_UPDATE_CONS        2     // Number of consistent data samples required for matrix update. Helps prevent ghosting.
#define TTAG_INT_A              250     // Matrix update time interval, config A [us]
#define TTAG_INT_B              500     // Matrix update time interval, config B [us]
#define PWM_STEPS_A               8     // Number of brightness steps, config A
#define PWM_STEPS_B               4     // Number of brightness steps, config B
#define DEFAULT_GLOWDUR         140     // Default glow duration [ms]
#define DEFAULT_BRIGHTNESS        7     // Default maximum lamp brightness 0-7
#define CURRENT_MONITOR           0     // Monitor the current (unfinished featured, only on board rev >=1.3 and <2.0)   
#define DEBUG_SERIAL              0     // Turn debug output via serial on/off
#define REPLAY_ENABLED            0     // Enable lamp replay in test mode when set to 1
#define PROJECT_BUTTER            1     // Smooth as butter afterglow


//------------------------------------------------------------------------------
// Some definitions

// Afterglow configuration version
#if (AFTERGLOW_WHITESTAR == 0)
#  define AFTERGLOW_CFG_VERSION 1
#else
#  define AFTERGLOW_CFG_VERSION 2
#endif

// Afterglow board revision
#if (AFTERGLOW_WHITESTAR == 0)
#  define BOARD_REV AFTERGLOW_BOARD_REV
#else
#  define BOARD_REV AFTERGLOW_WS_BOARD_REV
#endif

// original matrix update interval [us]
#if (AFTERGLOW_WHITESTAR == 0)
#  define ORIG_INT (2000)
#else
#  define ORIG_INT (1000)
#endif

// cycles per original interval, config A
#define ORIG_CYCLES_A (ORIG_INT / TTAG_INT_A)

// cycles per original interval, config B
#define ORIG_CYCLES_B (ORIG_INT / TTAG_INT_B)

// number of columns in the lamp matrix
#define NUM_COL 8

// number of rows in the lamp matrix
#if (AFTERGLOW_WHITESTAR == 0)
#  define NUM_ROW 8
#else
#  define NUM_ROW 10
#endif

// number of strobed lines
#if (AFTERGLOW_WHITESTAR == 0)
// Most systems strobe the 8 columns
#  define NUM_STROBE NUM_COL
#  define NUM_NONSTROBE NUM_ROW
#else
// Stern Whitestar strobes the 10 rows
#  define NUM_STROBE NUM_ROW
#  define NUM_NONSTROBE NUM_COL
#endif

// glow duration scaling in the configuration
#define GLOWDUR_CFG_SCALE 10

#if (AFTERGLOW_WHITESTAR == 0)
// current supervision on pin A0
#  define CURR_MEAS_PIN A0
#else
// RGB status LED on A0
#  define RGB_LED_A0
#endif

// test mode setup
#define TEST_MODE_NUMMODES 7    // number of test modes
#define TEST_MODE_DUR 8         // test duration per mode [s]
#define TEST_MODE_DUR_CYCLES_A  ((uint32_t)TEST_MODE_DUR * 1000000UL / TTAG_INT_A) // number of cycles per testmode, config A
#define TEST_MODE_DUR_CYCLES_B  ((uint32_t)TEST_MODE_DUR * 1000000UL / TTAG_INT_B) // number of cycles per testmode, config B
#define TESTMODE_INT (500)      // test mode lamp switch interval [ms]
#define TESTMODE_CYCLES_A ((uint32_t)TESTMODE_INT * 1000UL / (uint32_t)TTAG_INT_A) // number of cycles per testmode interval, config A
#define TESTMODE_CYCLES_B ((uint32_t)TESTMODE_INT * 1000UL / (uint32_t)TTAG_INT_B) // number of cycles per testmode interval, config B

#if REPLAY_ENABLED
// Replay time scale [us]
#define REPLAY_TTAG_SCALE 16000

// Replay record
typedef struct AG_LAMP_SWITCH_s
{
    uint16_t col : 3;    // lamp column
    uint16_t row : 3;    // lamp row
    uint16_t dttag : 10; // delta time tag [16ms] to the last event
} AG_LAMP_SWITCH_t;

// Replay logic
byte replay(void);

// Number of replay records
int numReplays(void);
#endif // REPLAY_ENABLED


//------------------------------------------------------------------------------
// serial port protocol definition

// write buffer size [bytes]
#define AG_CMD_WRITE_BUF 32

// command terminator character
#define AG_CMD_TERMINATOR ':'

// version poll command string
#define AG_CMD_VERSION_POLL "AGV"

// configuration poll command string
#define AG_CMD_CFG_POLL "AGCP"

// configuration save command string
#define AG_CMD_CFG_SAVE "AGCS"

// configuration reset to default command string
#define AG_CMD_CFG_DEFAULT "AGCD"

// data ready string
#define AG_CMD_CFG_DATA_READY "AGDR"

// acknowledge string
#define AG_CMD_ACK "AGCACK"

// NOT acknowledge string
#define AG_CMD_NACK "AGCNACK"


//------------------------------------------------------------------------------
// global variables

// Lamp matrix 'memory'
static uint16_t sMatrixState[NUM_COL][NUM_ROW];

#if PROJECT_BUTTER
// Step per matrix update for all lamps
static int32_t sMatrixSteps[NUM_COL][NUM_ROW];

// Matrix value to brightness map
static uint16_t sBrightnessMap_A[PWM_STEPS_A];
static uint16_t sBrightnessMap_B[PWM_STEPS_B];
#endif

// local time
static uint32_t sTtag = 0;

// interrupt runtime counters [cycles]
static uint16_t sLastIntTime = 0;
static uint16_t sMaxIntTime = 0;
static volatile uint16_t sOverflowCount = 0;

// remember the last column and row samples
static uint16_t sLastColMask = 0;
static uint16_t sLastRowMask = 0;
static uint32_t sConsBadStrobeCounter = 0;

#if DEBUG_SERIAL
static uint16_t sLastOutColMask = 0;
static uint16_t sLastOutRowMask = 0;
static uint32_t sBadStrobeCounter = 0;
static uint32_t sBadStrobeOrderCounter = 0;
static uint16_t sLastBadStrobeMask = 0;
static byte sLastGoodStrobeLine = 0;
#if CURRENT_MONITOR
static int sMaxCurr = 0;
static int sLastCurr = 0;
#endif
#endif

// afterglow configuration data definition
typedef struct AFTERGLOW_CFG_s
{
    uint16_t version;                         // afterglow version of the configuration
    uint16_t res;                             // reserved bytes
    uint8_t lampGlowDur[NUM_COL][NUM_ROW];    // Lamp matrix glow duration configuration [ms * GLOWDUR_CFG_SCALE]
    uint8_t lampBrightness[NUM_COL][NUM_ROW]; // Lamp matrix maximum brightness configuration (0-7)
    uint32_t crc;                             // data checksum
} AFTERGLOW_CFG_t;

// afterglow configuration
static AFTERGLOW_CFG_t sCfg;

// precalculated glow steps for each lamp
static uint16_t sGlowSteps[NUM_COL][NUM_ROW];

// precalculated maximum subcycle for lamp activation (brightness)
static byte sMaxSubcycle[NUM_COL][NUM_ROW];

// last state of PINB
static uint8_t sLastPINB = 0;

// status enumeration
typedef enum AFTERGLOW_STATUS_e
{
    AG_STATUS_INIT = 0,     // initialising
    AG_STATUS_OK,           // up and running
    AG_STATUS_PASSTHROUGH,  // ready in pass-through mode
    AG_STATUS_TESTMODE,     // ready in test mode
    AG_STATUS_REPLAY,       // ready in replay mode
    AG_STATUS_INVINPUT,     // invalid input
    AG_STATUS_OVERRUN       // interrupt overrun
} AFTERGLOW_STATUS_t;

// afterglow status
static AFTERGLOW_STATUS_t sStatus = AG_STATUS_INIT;
static AFTERGLOW_STATUS_t sLastStatus = AG_STATUS_INIT;


//------------------------------------------------------------------------------
void setup()
{
    noInterrupts(); // disable all interrupts

    // I/O pin setup
    // 74LS165 LOAD and CLK are output, DATA is input
    // 74HC595 LOAD, CLK and DATA are output
    DDRD = B11111001;
    // Whitestar row 9 on pin D12, testmode config on pins 8-11
    DDRB = B00000000;
    // activate the pullups for the testmode and the whitestar pins
    PORTB |= B00011111;
    // OE on A1, Whitestar on A2 and A3
    // current meas on A0 for AG <v2.0, WS2812 on AG >=v2.0
    DDRC = B00001110;
 #ifdef RGB_LED_A0
    // WS2812 RGB LED on pin A0
    DDRC |= B00000001;
 #endif
    // Whitestar row 10 pin on A4 (enable pullup)
    PORTC |= B00010000;
    // keep OE high
    PORTC |= B00000010;

#if CURRENT_MONITOR
    // Configure the ADC clock to 1MHz by setting the prescaler to 16.
    // This should allow for fast analog pin sampling without much loss of precision.
    // defines for setting and clearing register bits.
    _SFR_BYTE(ADCSRA) |= _BV(ADPS2);
    _SFR_BYTE(ADCSRA) &= ~_BV(ADPS1);
    _SFR_BYTE(ADCSRA) &= ~_BV(ADPS0);
#endif

    // initialize the data
    memset(sMatrixState, 0, sizeof(sMatrixState));

    // load the configuration from EEPROM
    int err;
    bool cfgLoaded = loadCfg(&err);
    if (cfgLoaded == false)
    {
        // set default configuration
        setDefaultCfg();

        // store the configuration to EEPROM
        saveCfgToEEPROM();
    }

    // enable serial output at 115200 baudrate
    Serial.begin(115200);

    // Apply the configuration
    // This will prepare all values for the interrupt handlers.
    applyCfg();

    Serial.print("afterglow v");
    Serial.print(AFTERGLOW_VERSION);
#if AFTERGLOW_WHITESTAR
    Serial.print(" WS ");
#endif
    Serial.println("");
    Serial.println("-------------------------------");
    Serial.println("(c) 2018-2022 morbid cornflakes");

    // check the extended fuse for brown out detection level
    uint8_t efuse = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
    uint8_t bodBits = (efuse & 0x7);
    Serial.print("efuse BOD ");
    Serial.println((bodBits == 0x07) ? "OFF" : (bodBits == 0x04) ? "4.3V" : (bodBits == 0x05) ? "2.7V" : "1.8V");
#if REPLAY_ENABLED
    Serial.print("Replay Table Size: ");
    Serial.println(numReplays());
#endif
#if DEBUG_SERIAL
    Serial.print("CFG from ");
    Serial.print(cfgLoaded ? "EEPROM" : "DEFAULT");
    if (err)
    {
        Serial.print(" err ");
        Serial.print(err);
    }
    Serial.println("");
#endif

    // setup the timers
    timerSetup();

    // enable all interrupts
    interrupts();

    // enable a strict 15ms watchdog
    wdt_enable(WDTO_15MS);

    sLastPINB = PINB;

    // ready
    sStatus = AG_STATUS_OK;
}

//------------------------------------------------------------------------------
void timerSetup(void)
{
    // Use Timer1 to create an interrupt every TTAG_INT us.
    // This will be the heartbeat of our realtime task.
    TCCR1B = 0;
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS10 bit so timer runs at clock speed
    TCCR1B |= (1 << CS10);  
    // turn off other timer1 functions
    TCCR1A = 0;
    // set compare match register for TTAG_INT us increments
    // prescaler is at 1, so counting real clock cycles
    OCR1A = (PINB & B00000100) ?
        (TTAG_INT_A * 16) :  // [16MHz clock cycles]
        (TTAG_INT_B * 16);   // [16MHz clock cycles]
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
}

//------------------------------------------------------------------------------
void start()
{
    // enable the timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);

    // enable a strict 15ms watchdog
    wdt_enable(WDTO_15MS);
}

//------------------------------------------------------------------------------
void stop()
{
    // disable the watchdog
    wdt_disable();

    // disable the timer compare interrupt
    TIMSK1 &= ~(1 << OCIE1A);

    // pull OE high to disable all outputs
    PORTC |= B00000010;
}

//------------------------------------------------------------------------------
// Timer1 overflow handler
// 
ISR(TIMER1_OVF_vect)
{
    sOverflowCount++;
}

//------------------------------------------------------------------------------
// Timer1 interrupt handler
// This is the realtime task heartbeat. All the magic happens here.
ISR(TIMER1_COMPA_vect)
{   
    // time is running
    uint16_t startCnt = TCNT1;
    sTtag++;

    // kick the dog
    wdt_reset();

    // Drive the lamp matrix
    // This is done before updating the matrix to avoid having an irregular update
    // frequency due to varying update calculation times.
    if ((PINB & B00001000) == 0)
    {
        // pass-through mode
        driveLampMatrixPassThrough();
    }
    else
    {
        // afterglow mode
        driveLampMatrix();
    }

#if CURRENT_MONITOR
    monitorCurrent();
#endif

    // read/create the input data
    uint32_t inData;
    if ((PINB & B00000001) == 0)
    {
        // testmode input simulation (jumper J1 active)
        inData = testModeInput();
    }
    else
    {
        // 74HC165 input sampling
        uint32_t inData = sampleInput();
    }
    uint16_t inColMask = (uint16_t)(inData >> 16); // LSB is col 0, MSB is col 7
    uint16_t inRowMask = ~(uint16_t)inData; // high means OFF, LSB is row 0, bit 7 is row 7

    // evaluate the strobe line reading
    // only one bit should be set as only one strobe line can be active at a time
    uint32_t strobeLine;
    bool validInput = checkValidStrobeMask(inColMask, inRowMask, &strobeLine);

    // The input matrix values are updated only once per original strobe cycle. The code
    // waits for a number of consecutive consistent information before adopting the new data.
    validInput &= updateValid(inColMask, inRowMask);

    // Update the input state only with a valid input. If the input is invalid the current
    // input matrix state is left unchanged.
    if (validInput)
    {
        // update the input data with the current strobe line
        updateStrobe(strobeLine, inColMask, inRowMask);

#if DEBUG_SERIAL
        // monitor for bad strobe line input
        if ((strobeLine != (sLastGoodStrobeLine+1)) && (strobeLine!=(sLastGoodStrobeLine-NUM_STROBE+1)))
        {
            sBadStrobeOrderCounter++;
        }
        sLastGoodStrobeLine = strobeLine;
#endif
    }

#if PROJECT_BUTTER
    // update the current output column with the values from the step matrix
    updateOutCol();
#endif

    // remember the last column and row samples
    sLastColMask = inColMask;
    sLastRowMask = inRowMask;

    // status update
    statusUpdate();

    // how long did it take?
    sLastIntTime = (TCNT1 - startCnt);
    if ((sLastIntTime > sMaxIntTime) && (sLastIntTime < (1000 * 16)))
    {
        sMaxIntTime = sLastIntTime;
    }
}

//------------------------------------------------------------------------------
void loop()
{
    // The main loop is used for low priority serial communication only.
    // All the lamp matrix fun happens in the timer interrupt.

    // count the loops (used for debug output below)
    static uint32_t loopCounter = 0;
    loopCounter++;

    // check for serial data
    static String cmd = "";
    static bool complete = false;
    while (Serial.available() && (complete == false))
    {
        char character = Serial.read();
        if (character != AG_CMD_TERMINATOR)
        {
            // add the character and wait for the command terminator
            cmd.concat(character);
        }
        else
        {
            // command complete
            complete = true;
        }
    }

    // handle complete commands
    if (complete)
    {
        // version poll
        if (cmd == AG_CMD_VERSION_POLL)
        {
            // Output the version numbers
            Serial.print(AG_CMD_VERSION_POLL);
            Serial.print(" ");
            Serial.print(AFTERGLOW_VERSION);
            Serial.print(" ");
            Serial.println(AFTERGLOW_CFG_VERSION);
        }

        // configuration poll
        else if (cmd == AG_CMD_CFG_POLL)
        {
            // send the full confiuration
            sendCfg();
        }

        // configuration reset
        else if (cmd == AG_CMD_CFG_DEFAULT)
        {
            // reset the configuration to default
            defaultCfg();
        }

        // configuration write
        else if (cmd == AG_CMD_CFG_SAVE)
        {
            // stop the matrix updates
            stop();

            // receive a new configuration
            receiveCfg();

            // resume operation
            start();
        }

        cmd = "";
        complete = false;
    }

    // watch out for interval configuration changes
    if ((PINB & B00000100) != (sLastPINB & B00000100))
    {
        // reinitialize the timers
        noInterrupts();
        timerSetup();
        sTtag = 0;
        sLastPINB = PINB;
        interrupts();
#if DEBUG_SERIAL
        Serial.print("New TTAG_INT: ");
        Serial.println((PINB & B00000100) ? TTAG_INT_A : TTAG_INT_B);
#endif
    }

#if DEBUG_SERIAL
    if ((loopCounter % 10) == 0)
    {
        // print the maximum interrupt runtime
        if ((PINB & B00000001) == 0)
        {
            Serial.println("TESTMODE!");
        }
        if ((PINB & B0000100) == 0)
        {
            Serial.println("REPLAY!");
        }
        if ((PINB & B0001000) == 0)
        {
            Serial.println("PASS THROUGH!");
        }
        Serial.print("TTAG_INT ");
        Serial.println((PINB & B00000100) ? TTAG_INT_A : TTAG_INT_B);
        Serial.print("INT dt max ");
        Serial.print(sMaxIntTime / 16);
        Serial.print("us last ");
        Serial.print(sLastIntTime / 16);
        Serial.print("us ovfl ");
        Serial.println(sOverflowCount);
        Serial.print("Bad col: ");
        Serial.print(sBadStrobeCounter);
        Serial.print(" col ");
        Serial.print(sLastBadStrobeMask);
        Serial.print(" ord ");
        Serial.print(sBadStrobeOrderCounter);
        Serial.print(" last good: ");
        Serial.println(sLastGoodStrobeLine);
#if CURRENT_MONITOR
        Serial.print("CM ");
        Serial.print(sLastCurr);
        Serial.print(" max ");
        Serial.println(sMaxCurr);
#endif
        // data debugging
        debugInputs(sLastColMask, sLastRowMask);
        debugOutput(sLastOutColMask, sLastOutRowMask);
        // dump the full matrix
        for (uint32_t c=0; c<NUM_COL; c++)
        {
            Serial.print("C");
            Serial.print(c);
            Serial.print(" + ");
            for (uint32_t r=0; r<NUM_ROW; r++)
            {
                Serial.print(sMatrixState[c][r]);
                Serial.print(" ");
            }
            Serial.println("");
        }
    }
#endif

    // wait 500ms
    delay(500);
}

#if PROJECT_BUTTER
//------------------------------------------------------------------------------
void updateOutCol()
{
    // get a pointer to the matrix column
    uint8_t outCol = (sTtag % NUM_COL);
    uint16_t *pMx = &sMatrixState[outCol][0];
    int32_t *pMxSt = &sMatrixSteps[outCol][0];

    for (uint8_t r=0; r<NUM_ROW; r++)
    {
        // update the matrix value
        uint16_t step;
        bool on;
        if (*pMxSt >= 0)
        {
            step = (uint16_t)*pMxSt;
            on = true;
        }
        else
        {
            step = (uint16_t)(-(*pMxSt));
            on = false;
        }
        updateMx(pMx, on, step);

        // next row
        pMx++;
        pMxSt++;
    }
}
#endif

//------------------------------------------------------------------------------
inline void updateMx(uint16_t *pMx, bool on, uint16_t step)
{
    if (on)
    {
        // increase the stored brightness value
        if (*pMx < (65535 - step))
        {
            *pMx += step;
        }
        else
        {
            *pMx = 0xffff;
        }
    }
    else
    {
        // decrease the stored brightness value
        if (*pMx > step)
        {
            *pMx -= step;
        }
        else
        {
            *pMx = 0;
        }
    }
}

//------------------------------------------------------------------------------
void updateCol(uint32_t col, uint16_t rowMask)
{
    // paranoia check
    if (col >= NUM_COL)
    {
        return;
    }

#if (PROJECT_BUTTER == 0)
    // get a pointer to the matrix column
    uint16_t *pMx = &sMatrixState[col][0];
    const uint16_t *pkStep = &sGlowSteps[col][0];

    // update all row values
    for (uint8_t r=0; r<NUM_ROW; r++)
    {
        // update the matrix value
        updateMx(pMx, (rowMask & 0x0001), *pkStep);

        // next row
        pMx++;
        pkStep++;
        rowMask >>= 1;
    }
#else
    // get a pointer to the matrix steps column
    int32_t *pMxSt = &sMatrixSteps[col][0];
    const uint16_t *pkStep = &sGlowSteps[col][0];

    // update all row values
    for (uint8_t r=0; r<NUM_ROW; r++)
    {
        // set the matrix step value
        *pMxSt = (rowMask & 0x0001) ? *pkStep : -(*pkStep);

        // next row
        pMxSt++;
        pkStep++;
        rowMask >>= 1;
    }
#endif
}

//------------------------------------------------------------------------------
void updateRow(uint32_t row, uint16_t colMask)
{
    // paranoia check
    if (row >= NUM_ROW)
    {
        return;
    }

#if (PROJECT_BUTTER == 0)
    // get a pointer to the matrix row
    uint16_t *pMx = &sMatrixState[0][row];
    const uint16_t *pkStep = &sGlowSteps[0][row];

    // update all column values
    for (uint8_t c=0; c<NUM_COL; c++)
    {
        // update the matrix value
        updateMx(pMx, (colMask & 0x0001), *pkStep);

        // next row
        pMx += NUM_ROW;
        pkStep += NUM_ROW;
        colMask >>= 1;
    }
#else
    // get a pointer to the matrix row
    int32_t *pMxSt = &sMatrixSteps[0][row];
    const uint16_t *pkStep = &sGlowSteps[0][row];

    // update all column values
    for (uint8_t c=0; c<NUM_COL; c++)
    {
        // update the matrix step value
        *pMxSt = (colMask & 0x0001) ? *pkStep : -(*pkStep);

        // next row
        pMxSt += NUM_ROW;
        pkStep += NUM_ROW;
        colMask >>= 1;
    }
#endif
}

//------------------------------------------------------------------------------
void updateStrobe(uint32_t strobe, uint16_t colMask, uint16_t rowMask)
{
#if (AFTERGLOW_WHITESTAR == 0)
    updateCol(strobe, rowMask);
#else
    updateRow(strobe, colMask);
#endif
}

//------------------------------------------------------------------------------
uint32_t sampleInput(void)
{
    // drive CLK and LOAD low
    PORTD &= B11100111;
    
    // wait some time
    uint32_t data = 0;
    data+= 17;
    data-= 17;
    
    // drive LOAD high to save pin states
    PORTD |= B00010000;
    
    // clock in all 16 data bits from the shift register
    for (byte i=0; i<16; i++)
    {
        data <<= (i == 8) ? 9 : 1;         // make way for the new bit, shift columns up to bit 16
        PORTD &= B11110111;                // CLK low
        data |= ((PIND & B00000100) >> 2); // read data bit
        PORTD |= B00001000;                // CLK high
    }

#if (NUM_ROW > 8)
    // read the two extra rows
    data |= (((uint32_t)(PINB & B00010000)) << 4); // row 9 on D12
    data |= (((uint32_t)(PINC & B00010000)) << 5); // row 10 on A4
#endif

    return data;
}

//------------------------------------------------------------------------------
void driveLampMatrixPassThrough()
{
    static uint16_t sLastPassThroughColMask = 0;
    static uint16_t sLastPassThroughRowMask = 0;

    // only update when changed
    if ((sLastColMask != sLastPassThroughColMask) ||
        (sLastRowMask != sLastPassThroughRowMask))
    {
        // update the output
        dataOutput(sLastColMask, sLastRowMask);

        // remember the new state
        sLastPassThroughColMask = sLastColMask;
        sLastPassThroughRowMask = sLastRowMask;
    }
}

//------------------------------------------------------------------------------
void driveLampMatrix()
{
    // turn off everything briefly to avoid ghosting
    // the scope says this takes ~20us at 16MHz
    dataOutput(0x0000, 0x0000);

    // check which column we're currently updating
    uint32_t outCol = (sTtag % NUM_COL);

    // The update interval is divided into UPD_CYCLES column sub cycles.
    // These cycles are used to do PWM in order to adjust the lamp brightness.
    //
    // Illustration with UPD_CYCLES==4 and four brightness steps B1-B4 and off (B0):
    //
    // * Lamp on
    // afterglow col       012345670123456701234567012345670123456701234567012345
    // col cycle           0       1       2       3       0       1       2
    //
    // Brightness 1        *                               *
    // Brightness 2        *       *                       *       *
    // Brightness 3        *       *       *               *       *       *
    // Brightness 4        *       *       *       *       *       *       *
    uint32_t colCycle = (PINB & B00000100) ?
        ((sTtag / NUM_COL) % PWM_STEPS_A) :
        ((sTtag / NUM_COL) % PWM_STEPS_B);

    // prepare the data
    // LSB is row/col 0, MSB is row/col 7
    uint16_t colData = (1 << outCol);
    uint16_t rowData = 0;
    uint16_t *pMx = &sMatrixState[outCol][0];
    byte *pMaxSubCycle = &sMaxSubcycle[outCol][0];
    for (uint32_t r=0; r<NUM_ROW; r++)
    {
        // make room for the next bit
        rowData >>= 1;
        
        // nothing to do if the matrix value is zero (off)
        if (*pMx)
        {
            // Find the subcycle in the brightness map
            uint8_t subCycle = findSubCycle(*pMx);

            // limit to the configured maximum brightness
            if (subCycle > *pMaxSubCycle)
            {
                subCycle = *pMaxSubCycle;
            }

            // Lamps are turned on when the value in the matrix is not zero
            // and when the value is high enough for the current sub cycle.
            if (subCycle >= colCycle)
            {
                rowData |= ((uint16_t)1 << (NUM_ROW-1));
            }
        }
        pMx++;
        pMaxSubCycle++;
    }

    // output the data
    dataOutput(colData, rowData);
#if DEBUG_SERIAL
    sLastOutColMask = colData;
    sLastOutRowMask = rowData;
#endif
}

//------------------------------------------------------------------------------
uint8_t findSubCycle(uint16_t v)
{
#if (PROJECT_BUTTER == 0)
    uint8_t subCycle = (PINB & B00000100) ?
        (uint8_t)(v / (65536 / PWM_STEPS_A)) :
        (uint8_t)(v / (65536 / PWM_STEPS_B));
#else
    // This is done in a non optimal way in order to
    // maintain a constant runtime.
    uint8_t subCycle;
    const uint16_t *pkBM;
    uint8_t numSteps;
    if (PINB & B00000100)
    {
        pkBM = &sBrightnessMap_A[PWM_STEPS_A-1];
        numSteps = PWM_STEPS_A;
        subCycle = (PWM_STEPS_A-1);
    }
    else
    {
        pkBM = &sBrightnessMap_B[PWM_STEPS_B-1];
        numSteps = PWM_STEPS_B;
        subCycle = (PWM_STEPS_B-1);
    }
    for (int8_t i=(numSteps-2); i>=0; i--, pkBM--)
    {
        if (v < *pkBM)
        {
            subCycle = i;
        }
    }
#endif
    return subCycle;
}

//------------------------------------------------------------------------------
void dataOutput(uint16_t colData, uint16_t rowData)
{
    // This writes the 16bit column and row data to the two 74595 shift registers
    
    // pull RCLK (OUT_LOAD) and CLK low to start sending data
    PORTD &= B00111111;

    // prepare the data (8 bits from column and row)
    uint16_t data = ((rowData << 8) | (colData & 0x00ff));
    
    // clock out all data
    for (uint16_t bitMask=0x8000; bitMask>0; bitMask>>=1)
    {
        PORTD &= B10111111; // CLK low
        if (data & bitMask)
        {
            PORTD |= B00100000; // set data bit
        }
        else
        {
            PORTD &= B11011111; // clear data bit
        }
        PORTD |= B01000000; // CLK high
    }

    PORTD &= B10111111; // CLK low

    // pull RCLK high to latch the data
    PORTD |= B10000000;

#if (NUM_ROW > 8)
    // output the two additional rows directly on A2 and A3
    PORTC = ((PORTC & B11110011) | (uint8_t)(((rowData >> 8) & B00000011) << 2));
#endif

    // Enable by pulling OE low.
    // This is only done here to ensure that the LEDs are not turned on before
    // the columns are duty cycled.
    PORTC &= B11111101;
}

//------------------------------------------------------------------------------
uint32_t testModeInput(void)
{
    // simulate the original column cycle
    static uint8_t sCycleCounter = 0;
    static uint8_t sStrobeLine = 0;
    bool modeA = (PINB & B00000100) ? true : false;
    if ((modeA && (sCycleCounter == ORIG_CYCLES_A)) ||
        (!modeA && (sCycleCounter == ORIG_CYCLES_B)))
    {
        sCycleCounter = 0;
        sStrobeLine++;
    }
    if (sStrobeLine == NUM_STROBE)
    {
        sStrobeLine = 0;
    }
    sCycleCounter++;
    uint16_t strobeMask = ((uint16_t)1 << (uint16_t)sStrobeLine);

    // populate the non strobed mask
    uint16_t nonStrobeMask = 0;

#if REPLAY_ENABLED
    // test switch 2 activates the replay mode
    if ((PINB & B00000010) == 0)
    {
        // replay from table
        nonStrobeMask = replay(sStrobeLine);
    }
#endif

    // Start simulation if test switch 2 (replay mode) is inactive
    if ((PINB & B00000010) != 0)
    {       
        // loop through all available modes
        static uint32_t sModeCounter = 0;
        static uint32_t sMode = 0;
        static uint32_t sModeCycleCounter = 0;
        static uint32_t sModeCycle = 0;
        if ((modeA && (sModeCounter == TEST_MODE_DUR_CYCLES_A)) ||
            (!modeA && (sModeCounter == TEST_MODE_DUR_CYCLES_B)))
        {
            sModeCounter = 0;
            sModeCycleCounter = 0;
            sModeCycle = 0;
            sMode++;
        }
        if (sMode == TEST_MODE_NUMMODES)
        {
            sMode = 0;
        }
        sModeCounter++;
        if ((modeA && (sModeCycleCounter == TESTMODE_CYCLES_A)) ||
            (!modeA && (sModeCycleCounter == TESTMODE_CYCLES_B)))
        {
            sModeCycleCounter = 0;
            sModeCycle++;
        }
        sModeCycleCounter++;

        switch (sMode)
        {
            case 0:
            // cycle all strobe lines
            {
                uint8_t s = (sModeCycle % NUM_STROBE);
                if (s == sStrobeLine)
                {
                    nonStrobeMask = 0xffff;
                }
            }
            break;
            case 1:
            // cycle all non strobe lines
            {
                uint8_t ns = (sModeCycle % NUM_NONSTROBE);
                nonStrobeMask |= ((uint16_t)1 << ns);
            }
            break;
            case 2:
            // cycle all strobe lines (inverted)
            {
                uint8_t s = (sModeCycle % NUM_STROBE);
                if (s != sStrobeLine)
                {
                    nonStrobeMask = 0xffff;
                }
            }
            break;
            case 3:
            // cycle all non strobe lines (inverted)
            {
                uint8_t ns = (sModeCycle % NUM_NONSTROBE);
                nonStrobeMask = ~(1 << ns);
            }
            break;
            case 4:
            // blink all lamps
            {
                if (sModeCycle % 2)
                {
                    nonStrobeMask = 0xffff;
                }
            }
            break;
            case 5:
            // switch between even and odd lamps
            // turn on every other strobe line
            {
                if (sStrobeLine % 2 == (sModeCycle % 2))
                {
                    nonStrobeMask = 0xaaaa;
                    if (sModeCycle % 3)
                    {
                        nonStrobeMask <<= 1;
                    }
                }
            }
            break;
            case 6:
            // cycle through all lamps individually with 4x speed
            {
                uint8_t l = (uint8_t)((sModeCycle * 4) % (NUM_COL * NUM_ROW));
                uint8_t c = (l / NUM_ROW);
                uint8_t r = (l % NUM_COL);
#if (AFTERGLOW_WHITESTAR == 0)
                if (c == sStrobeLine)
                {
                    nonStrobeMask = (1 << r);
                }
#else
                if (r == sStrobeLine)
                {
                    nonStrobeMask = (1 << c);
                }
#endif
            }
            break;
            default:
            break;
        }
    }

    // assign the column and row mask
#if (AFTERGLOW_WHITESTAR == 0)
    uint16_t colMask = strobeMask;
    uint16_t rowMask = nonStrobeMask;
#else
    uint16_t colMask = nonStrobeMask;
    uint16_t rowMask = strobeMask;
#endif

    // invert the row mask as in the original input HIGH means off
    rowMask = ~rowMask;
    return (((uint32_t)colMask << 16) | (uint32_t)rowMask);
}

//------------------------------------------------------------------------------
bool checkValidStrobeMask(uint16_t inColMask, uint16_t inRowMask, uint32_t *pStrobeLine)
{
    bool validInput = true;
    *pStrobeLine = NUM_STROBE;

#if (AFTERGLOW_WHITESTAR == 0)
    uint16_t strobeMask = (inColMask & 0xff);
#else
    uint16_t strobeMask = (inRowMask & 0x03ff);
#endif

    switch (strobeMask)
    {
        case 0x0001: *pStrobeLine = 0; break;
        case 0x0002: *pStrobeLine = 1; break;
        case 0x0004: *pStrobeLine = 2; break;
        case 0x0008: *pStrobeLine = 3; break;
        case 0x0010: *pStrobeLine = 4; break;
        case 0x0020: *pStrobeLine = 5; break;
        case 0x0040: *pStrobeLine = 6; break;
        case 0x0080: *pStrobeLine = 7; break;
#if (NUM_STROBE > 8)
        case 0x0100: *pStrobeLine = 8; break;
        case 0x0200: *pStrobeLine = 9; break;
#endif
        default:
        {
            // This may happen if the sample is taken in between column transition.
            // Depending on the pinball ROM version the duration of this transition varies.
            // On a Whitewater with Home ROM LH6 (contains anti ghosting updates) this
            // gap was measured to be around 30us long.
            // Machines with anti-ghosting firmware will show a gap with no column enabled
            // for a while during the transition while older firmwares might have two
            // columns enabled at the same time due to slow transistor deactivation. Both
            // cases are caught here.
            // See also https://emmytech.com/arcade/led_ghost_busting/index.html for details.
            if (sConsBadStrobeCounter < 0xffffffff)
            {
                sConsBadStrobeCounter++;
            }
#if DEBUG_SERIAL
            if (sConsBadStrobeCounter < 0xffffffff)
            {
                sBadStrobeCounter++;
            }
            sLastBadStrobeMask = strobeMask;
#endif
            validInput = false;
        }
        break;
    }
    // restart the consecutive bad strobe counter
    if (validInput)
    {
        sConsBadStrobeCounter = 0;
    }
    return validInput;
}

//------------------------------------------------------------------------------
bool updateValid(uint16_t inColMask, uint16_t inRowMask)
{
    static byte sConsistentSamples = 0;
    static uint16_t sLastUpdStrobeMask = 0x0000;
    bool valid = false;

#if (AFTERGLOW_WHITESTAR == 0)
    uint16_t strobeMask = inColMask;
#else
    uint16_t strobeMask = inRowMask;
#endif

    // check if the current strobe line has not been handled already
    if (strobeMask != sLastUpdStrobeMask)
    {
        // reset the counter when the data changes
        if ((inColMask != sLastColMask) || (inRowMask != sLastRowMask))
        {
            sConsistentSamples = 0;
        }
        // count number of consecutive samples with consistent data
        else if (sConsistentSamples < 255)
        {
            sConsistentSamples++;
        }

        // The matrix is updated only once per original column cycle.
        // The code waits for a number of consecutive consistent information
        // before updating the matrix.
        // This also avoids ghosting issues, see
        // https://emmytech.com/arcade/led_ghost_busting/index.html for details.
        if (sConsistentSamples >= (SINGLE_UPDATE_CONS-1))
        {
            sLastUpdStrobeMask = strobeMask;
            valid = true;
        }
    }
    return valid;
}

//------------------------------------------------------------------------------
void applyCfg()
{
#if PROJECT_BUTTER
    // calculate the logarithmic brightness maps
    for (uint8_t i=0; i<PWM_STEPS_A; i++)
    {
        sBrightnessMap_A[i] = (uint16_t)(log10((float)i*10.0f/(float)(PWM_STEPS_A-1)) * 65535);
    }
    for (uint8_t i=0; i<PWM_STEPS_B; i++)
    {
        sBrightnessMap_B[i] = (uint16_t)(log10((float)i*10.0f/(float)(PWM_STEPS_B-1)) * 65535);
    }
#endif
    
    // calculate the glow steps and maximum subcycles
    uint16_t *pGS = &sGlowSteps[0][0];
    uint8_t *pGlowDur = &sCfg.lampGlowDur[0][0];
    uint8_t *pBrightness = &sCfg.lampBrightness[0][0];
    byte *pMaxSubCycle = &sMaxSubcycle[0][0];
    for (byte c=0; c<NUM_COL; c++)
    {
        for (byte r=0; r<NUM_ROW; r++)
        {
            uint32_t glowDur = (*pGlowDur * GLOWDUR_CFG_SCALE);

            // translate maximum brightness into maximum lamp driving subcycle
            *pMaxSubCycle = (PINB & B00000100) ?
                (*pBrightness >> (8/PWM_STEPS_A-1)) :
                (*pBrightness >> (8/PWM_STEPS_B-1));

#if (PROJECT_BUTTER == 0)
            // brightness step per lamp matrix update (assumes one update per original matrix step)
            *pGS++ = (glowDur > 0) ?
                ((uint16_t)(65535 / ((glowDur * 1000) / ORIG_INT)) * NUM_STROBE) : 0xffff;
#else
            // brightness step per lamp matrix update (assumes one update per matrix step)
            uint16_t maxVal = (PINB & B00000100) ?
                sBrightnessMap_A[*pMaxSubCycle] : sBrightnessMap_B[*pMaxSubCycle];
            *pGS++ = (glowDur > 0) ?
                (PINB & B00000100) ?
                 ((uint16_t)(maxVal / ((glowDur * 1000) / TTAG_INT_A)) * NUM_COL) : 
                 ((uint16_t)(maxVal / ((glowDur * 1000) / TTAG_INT_B)) * NUM_COL) : 0xffff;
#endif

            // next
            pMaxSubCycle++;
            pGlowDur++;
            pBrightness++;
        }
    }
}

//------------------------------------------------------------------------------
void setDefaultCfg()
{
    // initialize configuration to default values
    memset(&sCfg, 0, sizeof(sCfg));
    sCfg.version = AFTERGLOW_CFG_VERSION;
    uint8_t *pGlowDur = &sCfg.lampGlowDur[0][0];
    uint8_t *pBrightness = &sCfg.lampBrightness[0][0];
    for (byte c=0; c<NUM_COL; c++)
    {
        for (byte r=0; r<NUM_ROW; r++)
        {
            *pGlowDur++ = (DEFAULT_GLOWDUR / GLOWDUR_CFG_SCALE);
            *pBrightness++ = DEFAULT_BRIGHTNESS;
        }
    }

    // calculate the crc
    uint16_t cfgSize = sizeof(sCfg);
    sCfg.crc = calculateCRC32((uint8_t*)&sCfg, cfgSize-sizeof(sCfg.crc));
}

//------------------------------------------------------------------------------
void defaultCfg()
{
    // set the default configuration
    setDefaultCfg();

    // send the acknowledge
    Serial.print(AG_CMD_ACK);
}

//------------------------------------------------------------------------------
int loadCfg(int *pErr)
{
    bool valid = false;
    *pErr = 0;

    // load the configuration from the EEPROM
    uint16_t cfgSize = sizeof(sCfg);
    uint8_t *pCfg = (uint8_t*)&sCfg;
    for (uint16_t i=0; i<cfgSize; i++)
    {
        *pCfg++ = EEPROM.read(i);
    }

    // check the version
    if (sCfg.version == AFTERGLOW_CFG_VERSION)
    {
        // check the CRC of the data
        uint32_t crc = calculateCRC32((uint8_t*)&sCfg, cfgSize-sizeof(sCfg.crc));
        if (crc == sCfg.crc)
        {
            valid = true;
        }
        else
        {
            *pErr = 2;
        }
    }
    else
    {
        *pErr = 1;
    }

    return valid;
}

//------------------------------------------------------------------------------
uint32_t calculateCRC32(const uint8_t *data, uint16_t length)
{
    uint32_t crc = 0xffffffff;
    while (length--)
    {
        uint8_t c = *data++;
        for (uint32_t i = 0x80; i > 0; i >>= 1)
        {
            bool bit = crc & 0x80000000;
            if (c & i)
            {
                bit = !bit;
            }
            crc <<= 1;
            if (bit)
            {
                crc ^= 0x04c11db7;
            }
        }
    }
    return crc;
}

//------------------------------------------------------------------------------
void sendCfg()
{
    // send the whole configuration structure via serial port
    uint16_t cfgSize = sizeof(sCfg);
    const byte *pkCfg = (const byte*)&sCfg;
    Serial.write(pkCfg, cfgSize);
}

//------------------------------------------------------------------------------
void receiveCfg()
{
    // wait for the full configuration data
    bool res = false;
    AFTERGLOW_CFG_t cfg;
    uint8_t *pCfg = (uint8_t*)&cfg;
    uint16_t cfgSize = sizeof(cfg);
    uint16_t size = 0;

    // read all data
    while (size < cfgSize)
    {
        // send data ready signal and wait for data
        Serial.print(AG_CMD_CFG_DATA_READY);
        delay(200);

        // read data
        uint32_t readBytes = 0;
        while ((Serial.available()) && (readBytes < AG_CMD_WRITE_BUF) && (size < cfgSize))
        {
            *pCfg++ = Serial.read();
            readBytes++;
            size++;
        }
    }

    if (size == sizeof(cfg))
    {
        // check the crc
        uint32_t crc = calculateCRC32((uint8_t*)&cfg, size-sizeof(cfg.crc));
        if (crc == cfg.crc)
        {
             // set the new configuration and apply it
            memcpy(&sCfg, &cfg, size);
            applyCfg();

            // store the configuration to EEPROM
            saveCfgToEEPROM();

            res = true;
        }
#if DEBUG_SERIAL
        else
        {
            Serial.print("CRC FAIL ");
            Serial.print(crc);
            Serial.print(" ");
            Serial.println(cfg.crc);
        }
#endif
    }
#if DEBUG_SERIAL
    else
    {
            Serial.print("SIAG_STATUS_TESTMODEZE MISMATCH: ");
            Serial.println(size);
    }
#endif

    // send ACK/NACK
    Serial.print(res ? AG_CMD_ACK : AG_CMD_NACK);
}

//------------------------------------------------------------------------------
void saveCfgToEEPROM()
{
    const uint8_t *pkCfg = (const uint8_t*)&sCfg;
    for (uint16_t i=0; i<sizeof(sCfg); i++)
    {
        EEPROM.write(i, *pkCfg++);
    }
    Serial.print("EEPROM write ");
    Serial.println(sizeof(sCfg));
}

//------------------------------------------------------------------------------
void statusUpdate()
{
    // update the current status value
    if ((PINB & B00000001) == 0)
    {
        // testmode input simulation (jumper J1 active)
        sStatus = ((PINB & B00000010) == 0) ? AG_STATUS_REPLAY : AG_STATUS_TESTMODE;
    }
    else
    {
        // normal operation
        sStatus = (sOverflowCount < 100) ? 
                   ((sConsBadStrobeCounter < 100) ?
                    (((PINB & B00001000) == 0) ? AG_STATUS_PASSTHROUGH : AG_STATUS_OK) : AG_STATUS_INVINPUT) : AG_STATUS_OVERRUN;
    }

    // display the status
#ifdef RGB_LED_A0
    // use the RGB LED to display the status
    if (sStatus != sLastStatus)
    {
        switch (sStatus)
        {
            case AG_STATUS_INIT: ws2812Update(0x00222222); break;
            case AG_STATUS_OK: ws2812Update(0x00220000); break;
            case AG_STATUS_TESTMODE: ws2812Update(0x00220011); break;
            case AG_STATUS_INVINPUT: ws2812Update(0x00003300); break;
            case AG_STATUS_OVERRUN: ws2812Update(0x00003333); break;
            case AG_STATUS_PASSTHROUGH: ws2812Update(0x00333333); break;
            case AG_STATUS_REPLAY: ws2812Update(0x00110033); break;
            default: ws2812Update(0x00444444); break;
        }
        sLastStatus = sStatus;
    }
#endif
}

#if CURRENT_MONITOR
//------------------------------------------------------------------------------
void monitorCurrent()
{
    // Current measurement is only available boards starting from v1.3, but not on Whitestar boards
#if ((BOARD_REV >= 13) && (BOARD_REV < 20))
    // Measure the current flowing through the current measurement resistor
    // (R26 on AG v1.3).
    int cm = analogRead(CURR_MEAS_PIN);
#if DEBUG_SERIAL
    sLastCurr = cm;
    if (sLastCurr > sMaxCurr)
    {
        sMaxCurr = sLastCurr;
    }
#endif
#endif
}
#endif

#ifdef RGB_LED_A0
//------------------------------------------------------------------------------
void ws2812Update(uint32_t rgb)
{
    // turn interrupts off
    noInterrupts();

    // save current port state
    uint8_t pch = (PORTC | B00000001);
    uint8_t pcl = (PORTC & B11111110);

    // write all bits
    for (int i=0; i<24; i++)
    {
        // pull high
        PORTC = pch;
        //__asm__("nop\n\t""nop\n\t""nop\n\t"");
        // set bit
        PORTC = pcl | ((uint8_t)(rgb >> 23) & 0x01);
        rgb <<= 1;
        __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
        PORTC = pcl;
    }

    // enable all interrupts again
    interrupts();
}
#endif

#if DEBUG_SERIAL
//------------------------------------------------------------------------------
void debugInputs(byte inColMask, byte inRowMask)
{
    // output the data
    char msg[64];
    sprintf(msg, "IN C 0x%02X R 0x%02X\n", inColMask, inRowMask);
    Serial.print(msg);
}

//------------------------------------------------------------------------------
void debugOutput(byte outColMask, byte outRowMask)
{
    // output the data
    char msg[64];
    sprintf(msg, "OUT C 0x%02X R 0x%02X\n", outColMask, outRowMask);
    Serial.print(msg);
}
#endif


#if REPLAY_ENABLED
// Recording of the attract mode from a Creature of the Black Lagoon pinball.
// Recorded from a modified pinmame version.
const AG_LAMP_SWITCH_t kLampReplay[] PROGMEM =
{
{7, 7, 0},   // +0.000s 0
{7, 7, 25}, {0, 5, 4}, {1, 4, 0}, {2, 1, 0}, {2, 5, 0}, {3, 1, 0}, {3, 5, 0}, {4, 3, 0}, {4, 5, 0}, {5, 2, 0}, 
{5, 5, 0}, {0, 1, 2}, {1, 1, 0}, {1, 7, 0}, {6, 1, 0}, {7, 1, 0}, {0, 5, 4}, {0, 6, 0}, {1, 5, 0}, {2, 2, 0}, 
{2, 6, 0}, {3, 2, 0}, {3, 5, 0}, {3, 6, 0}, {4, 5, 0}, {4, 6, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {7, 5, 0}, 
{0, 1, 3}, {0, 2, 0}, {1, 1, 0}, {1, 2, 0}, {1, 4, 0}, {2, 1, 0}, {2, 5, 0}, {3, 1, 0}, {4, 3, 0}, {5, 2, 0}, 
{6, 2, 0}, {7, 1, 0}, {7, 2, 0}, {7, 4, 0}, {1, 7, 2}, {4, 7, 0}, {6, 1, 0}, {0, 4, 2}, {0, 6, 0}, {1, 5, 0},   // +7.067s 7072
{1, 6, 0}, {2, 3, 0}, {2, 4, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {3, 7, 0}, {4, 6, 0}, {5, 3, 0}, {5, 4, 0}, 
{5, 6, 0}, {5, 7, 0}, {7, 5, 0}, {7, 6, 0}, {0, 2, 2}, {0, 3, 0}, {1, 2, 0}, {1, 3, 0}, {2, 2, 0}, {2, 6, 0}, 
{2, 7, 0}, {5, 5, 0}, {6, 2, 0}, {6, 3, 0}, {7, 2, 0}, {7, 3, 0}, {0, 5, 2}, {3, 4, 0}, {4, 2, 0}, {4, 4, 0}, 
{4, 7, 0}, {5, 2, 0}, {5, 4, 0}, {5, 7, 0}, {7, 4, 0}, {0, 0, 2}, {0, 4, 0}, {1, 0, 0}, {1, 4, 0}, {1, 6, 0}, 
{2, 0, 0}, {2, 3, 0}, {2, 4, 0}, {2, 5, 0}, {3, 0, 0}, {3, 3, 0}, {3, 7, 0}, {4, 1, 0}, {7, 0, 0}, {7, 6, 0},   // +7.167s 7168
{0, 3, 2}, {1, 3, 0}, {2, 7, 0}, {4, 5, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {6, 3, 0}, {6, 4, 0}, {7, 3, 0}, 
{0, 5, 2}, {0, 6, 0}, {1, 5, 0}, {2, 1, 0}, {2, 6, 0}, {3, 1, 0}, {3, 4, 0}, {3, 5, 0}, {4, 2, 0}, {4, 3, 0}, 
{4, 4, 0}, {5, 2, 0}, {0, 0, 2}, {0, 1, 0}, {1, 0, 0}, {1, 1, 0}, {1, 4, 0}, {2, 0, 0}, {2, 5, 0}, {3, 0, 0}, 
{4, 1, 0}, {5, 1, 0}, {5, 7, 0}, {6, 4, 0}, {6, 5, 0}, {7, 0, 0}, {7, 1, 0}, {0, 4, 2}, {3, 6, 0}, {4, 5, 0}, 
{4, 6, 0}, {5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {7, 5, 0}, {0, 6, 2}, {1, 2, 0}, {1, 5, 0}, {1, 6, 0}, {2, 1, 0},   // +7.333s 7328
{2, 2, 0}, {2, 4, 0}, {2, 6, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0}, {4, 3, 0}, {6, 6, 0}, {7, 2, 0}, {0, 1, 2}, 
{0, 2, 0}, {0, 7, 0}, {1, 1, 0}, {4, 7, 0}, {5, 1, 0}, {5, 2, 0}, {5, 5, 0}, {5, 7, 0}, {6, 5, 0}, {7, 1, 0}, 
{7, 6, 0}, {0, 4, 3}, {0, 5, 0}, {1, 4, 0}, {2, 3, 0}, {2, 5, 0}, {3, 3, 0}, {3, 6, 0}, {3, 7, 0}, {4, 6, 0}, 
{5, 4, 0}, {7, 3, 0}, {7, 5, 0}, {0, 3, 2}, {1, 2, 0}, {1, 3, 0}, {1, 6, 0}, {2, 2, 0}, {2, 4, 0}, {3, 2, 0}, 
{5, 6, 0}, {6, 6, 0}, {6, 7, 0}, {7, 2, 0}, {0, 2, 2}, {0, 6, 0}, {0, 7, 0}, {3, 4, 0}, {4, 2, 0}, {4, 4, 0},   // +7.467s 7472
{4, 7, 0}, {5, 0, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {7, 6, 0}, {0, 0, 2}, {0, 5, 0}, {1, 0, 0}, {1, 4, 0}, 
{1, 5, 0}, {2, 0, 0}, {2, 3, 0}, {2, 5, 0}, {2, 6, 0}, {3, 0, 0}, {3, 3, 0}, {3, 7, 0}, {6, 0, 0}, {7, 0, 0}, 
{7, 3, 0}, {0, 3, 2}, {0, 4, 0}, {1, 3, 0}, {1, 7, 0}, {4, 5, 0}, {5, 0, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0}, 
{6, 7, 0}, {0, 6, 2}, {1, 6, 0}, {2, 1, 0}, {2, 4, 0}, {3, 1, 0}, {3, 4, 0}, {3, 5, 0}, {4, 2, 0}, {4, 3, 0}, 
{4, 4, 0}, {5, 3, 0}, {7, 1, 0}, {0, 0, 2}, {0, 1, 0}, {1, 0, 0}, {1, 1, 0}, {1, 5, 0}, {2, 0, 0}, {2, 6, 0},   // +7.600s 7600
{3, 0, 0}, {5, 5, 0}, {6, 0, 0}, {6, 1, 0}, {7, 0, 0}, {7, 4, 0}, {0, 4, 2}, {0, 5, 0}, {1, 4, 0}, {1, 7, 0}, 
{3, 6, 0}, {4, 5, 0}, {4, 6, 0}, {5, 2, 0}, {5, 4, 0}, {5, 7, 0}, {7, 5, 0}, {0, 2, 2}, {1, 1, 0}, {1, 2, 0}, 
{1, 6, 0}, {2, 1, 0}, {2, 2, 0}, {2, 4, 0}, {2, 5, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0}, {4, 3, 0}, {6, 2, 0}, 
{7, 1, 0}, {7, 2, 0}, {0, 1, 2}, {0, 6, 0}, {2, 7, 0}, {4, 7, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {6, 1, 0}, 
{7, 4, 0}, {7, 6, 0}, {0, 5, 2}, {1, 3, 0}, {1, 4, 0}, {1, 5, 0}, {2, 3, 0}, {2, 6, 0}, {3, 3, 0}, {3, 6, 0},   // +7.733s 7728
{3, 7, 0}, {4, 6, 0}, {5, 2, 0}, {7, 3, 0}, {7, 5, 0}, {0, 2, 2}, {0, 3, 0}, {1, 2, 0}, {2, 2, 0}, {2, 5, 0}, 
{3, 2, 0}, {4, 1, 0}, {5, 7, 0}, {6, 2, 0}, {6, 3, 0}, {7, 2, 0}, {0, 4, 3}, {0, 6, 0}, {1, 6, 0}, {2, 0, 0}, 
{2, 4, 0}, {2, 7, 0}, {3, 4, 0}, {4, 2, 0}, {4, 4, 0}, {4, 7, 0}, {5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {7, 6, 0}, 
{0, 0, 2}, {1, 0, 0}, {1, 3, 0}, {1, 5, 0}, {2, 3, 0}, {2, 6, 0}, {3, 0, 0}, {3, 3, 0}, {3, 7, 0}, {6, 4, 0}, 
{7, 0, 0}, {7, 3, 0}, {0, 3, 2}, {0, 5, 0}, {3, 5, 0}, {4, 1, 0}, {4, 5, 0}, {5, 1, 0}, {5, 2, 0}, {5, 5, 0},   // +7.867s 7872
{5, 7, 0}, {6, 3, 0}, {0, 4, 2}, {1, 1, 0}, {1, 4, 0}, {1, 6, 0}, {2, 0, 0}, {2, 1, 0}, {2, 4, 0}, {2, 5, 0}, 
{3, 0, 0}, {3, 1, 0}, {3, 4, 0}, {4, 2, 0}, {4, 3, 0}, {4, 4, 0}, {5, 4, 0}, {7, 1, 0}, {0, 0, 2}, {0, 1, 0}, 
{0, 7, 0}, {1, 0, 0}, {5, 6, 0}, {6, 4, 0}, {6, 5, 0}, {7, 0, 0}, {0, 5, 2}, {0, 6, 0}, {1, 5, 0}, {2, 2, 0}, 
{2, 6, 0}, {3, 2, 0}, {3, 5, 0}, {3, 6, 0}, {4, 5, 0}, {4, 6, 0}, {5, 1, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, 
{7, 5, 0}, {0, 2, 2}, {1, 1, 0}, {1, 2, 0}, {1, 4, 0}, {2, 1, 0}, {2, 5, 0}, {3, 1, 0}, {4, 3, 0}, {6, 6, 0},   // +8.000s 8000
{7, 1, 0}, {7, 2, 0}, {0, 1, 2}, {0, 4, 0}, {0, 7, 0}, {3, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 0, 0}, {5, 4, 0}, 
{5, 6, 0}, {5, 7, 0}, {6, 5, 0}, {7, 6, 0}, {0, 6, 2}, {1, 3, 0}, {1, 5, 0}, {1, 6, 0}, {2, 2, 0}, {2, 3, 0}, 
{2, 4, 0}, {2, 6, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {5, 3, 0}, {7, 3, 0}, {7, 5, 0}, {0, 2, 2}, {0, 3, 0}, 
{1, 2, 0}, {1, 7, 0}, {4, 4, 0}, {5, 2, 0}, {5, 5, 0}, {6, 6, 0}, {6, 7, 0}, {7, 2, 0}, {0, 4, 2}, {0, 5, 0}, 
{1, 4, 0}, {2, 0, 0}, {2, 5, 0}, {3, 0, 0}, {3, 4, 0}, {3, 7, 0}, {4, 2, 0}, {4, 7, 0}, {5, 0, 0}, {5, 4, 0},   // +8.133s 8128
{5, 7, 0}, {7, 6, 0}, {0, 0, 2}, {1, 0, 0}, {1, 3, 0}, {1, 6, 0}, {2, 3, 0}, {2, 4, 0}, {3, 3, 0}, {5, 6, 0}, 
{6, 0, 0}, {7, 0, 0}, {7, 3, 0}, {0, 3, 3}, {0, 6, 0}, {1, 7, 0}, {3, 5, 0}, {4, 3, 0}, {4, 4, 0}, {4, 5, 0}, 
{5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {6, 7, 0}, {7, 4, 0}, {0, 5, 2}, {1, 1, 0}, {1, 4, 0}, {1, 5, 0}, {2, 0, 0}, 
{2, 1, 0}, {2, 5, 0}, {2, 6, 0}, {3, 0, 0}, {3, 1, 0}, {3, 4, 0}, {4, 2, 0}, {6, 1, 0}, {7, 1, 0}, {0, 0, 2}, 
{0, 1, 0}, {1, 0, 0}, {2, 7, 0}, {4, 6, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0}, {6, 0, 0}, {7, 0, 0}, {0, 4, 2},   // +8.300s 8304
{0, 6, 0}, {1, 6, 0}, {2, 2, 0}, {2, 4, 0}, {3, 2, 0}, {3, 5, 0}, {3, 6, 0}, {4, 3, 0}, {4, 5, 0}, {5, 3, 0}, 
{7, 4, 0}, {7, 5, 0}, {0, 2, 2}, {1, 1, 0}, {1, 2, 0}, {1, 5, 0}, {2, 1, 0}, {2, 6, 0}, {3, 1, 0}, {4, 1, 0}, 
{5, 5, 0}, {6, 1, 0}, {6, 2, 0}, {7, 1, 0}, {7, 2, 0}, {0, 1, 2}, {0, 5, 0}, {2, 7, 0}, {3, 7, 0}, {4, 6, 0}, 
{4, 7, 0}, {5, 2, 0}, {5, 4, 0}, {5, 7, 0}, {7, 6, 0}, {0, 4, 2}, {1, 3, 0}, {1, 4, 0}, {1, 6, 0}, {2, 2, 0}, 
{2, 3, 0}, {2, 4, 0}, {2, 5, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {6, 3, 0}, {7, 2, 0}, {7, 3, 0}, {7, 5, 0},   // +8.400s 8400
{0, 2, 2}, {0, 3, 0}, {1, 2, 0}, {4, 1, 0}, {4, 4, 0}, {5, 1, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {6, 2, 0}, 
{0, 5, 2}, {0, 6, 0}, {1, 5, 0}, {2, 0, 0}, {2, 6, 0}, {3, 0, 0}, {3, 4, 0}, {3, 7, 0}, {4, 2, 0}, {4, 7, 0}, 
{5, 2, 0}, {7, 0, 0}, {7, 6, 0}, {0, 0, 2}, {0, 7, 0}, {1, 0, 0}, {1, 3, 0}, {1, 4, 0}, {2, 3, 0}, {2, 5, 0}, 
{3, 3, 0}, {5, 7, 0}, {6, 3, 0}, {6, 4, 0}, {7, 3, 0}, {0, 3, 2}, {0, 4, 0}, {0, 6, 0}, {3, 5, 0}, {4, 3, 0}, 
{4, 4, 0}, {4, 5, 0}, {5, 1, 0}, {5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {0, 1, 2}, {1, 1, 0}, {1, 5, 0}, {1, 6, 0},   // +8.567s 8560
{2, 0, 0}, {2, 1, 0}, {2, 4, 0}, {2, 6, 0}, {3, 0, 0}, {3, 1, 0}, {3, 4, 0}, {4, 2, 0}, {6, 5, 0}, {7, 0, 0}, 
{7, 1, 0}, {0, 0, 3}, {0, 5, 0}, {0, 7, 0}, {1, 0, 0}, {4, 6, 0}, {5, 0, 0}, {5, 2, 0}, {5, 5, 0}, {5, 7, 0}, 
{6, 4, 0}, {7, 5, 0}, {0, 4, 2}, {1, 2, 0}, {1, 4, 0}, {1, 6, 0}, {2, 2, 0}, {2, 5, 0}, {3, 2, 0}, {3, 5, 0}, 
{3, 6, 0}, {4, 3, 0}, {4, 5, 0}, {5, 4, 0}, {7, 2, 0}, {0, 1, 2}, {0, 2, 0}, {1, 1, 0}, {1, 7, 0}, {2, 1, 0}, 
{2, 4, 0}, {3, 1, 0}, {5, 6, 0}, {6, 5, 0}, {6, 6, 0}, {7, 1, 0}, {0, 5, 2}, {0, 6, 0}, {1, 5, 0}, {3, 7, 0},   // +8.700s 8704
{4, 6, 0}, {4, 7, 0}, {5, 0, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {7, 5, 0}, {7, 6, 0}, {0, 3, 2}, {1, 2, 0}, 
{1, 3, 0}, {1, 4, 0}, {2, 2, 0}, {2, 3, 0}, {2, 5, 0}, {2, 6, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {6, 7, 0}, 
{7, 2, 0}, {7, 3, 0}, {0, 2, 2}, {0, 4, 0}, {1, 7, 0}, {4, 4, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0}, {6, 6, 0}, 
{7, 4, 0}, {0, 6, 2}, {1, 0, 0}, {1, 5, 0}, {1, 6, 0}, {2, 0, 0}, {2, 3, 0}, {2, 4, 0}, {2, 6, 0}, {3, 0, 0}, 
{3, 4, 0}, {3, 7, 0}, {4, 2, 0}, {4, 7, 0}, {5, 3, 0}, {7, 0, 0}, {7, 6, 0}, {0, 0, 2}, {0, 3, 0}, {1, 3, 0},   // +8.833s 8832
{2, 7, 0}, {3, 3, 0}, {5, 5, 0}, {6, 0, 0}, {6, 7, 0}, {7, 3, 0}, {0, 4, 2}, {0, 5, 0}, {1, 4, 0}, {2, 1, 0}, 
{2, 5, 0}, {3, 1, 0}, {3, 5, 0}, {4, 3, 0}, {4, 4, 0}, {4, 5, 0}, {5, 2, 0}, {5, 4, 0}, {5, 7, 0}, {7, 4, 0}, 
{0, 1, 2}, {1, 0, 0}, {1, 1, 0}, {1, 6, 0}, {2, 0, 0}, {2, 4, 0}, {3, 0, 0}, {3, 4, 0}, {4, 2, 0}, {6, 1, 0}, 
{7, 0, 0}, {7, 1, 0}, {0, 0, 2}, {0, 6, 0}, {2, 7, 0}, {3, 6, 0}, {4, 1, 0}, {4, 6, 0}, {5, 3, 0}, {5, 5, 0}, 
{5, 6, 0}, {6, 0, 0}, {7, 5, 0}, {0, 5, 2}, {1, 2, 0}, {1, 4, 0}, {1, 5, 0}, {2, 1, 0}, {2, 2, 0}, {2, 5, 0},   // +8.967s 8960
{2, 6, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0}, {4, 3, 0}, {4, 5, 0}, {5, 2, 0}, {7, 2, 0}, {0, 1, 3}, {0, 2, 0}, 
{1, 1, 0}, {4, 7, 0}, {5, 1, 0}, {5, 7, 0}, {6, 1, 0}, {6, 2, 0}, {7, 1, 0}, {0, 4, 2}, {0, 6, 0}, {1, 6, 0}, 
{2, 3, 0}, {2, 4, 0}, {3, 3, 0}, {3, 6, 0}, {3, 7, 0}, {4, 1, 0}, {4, 6, 0}, {5, 3, 0}, {5, 4, 0}, {5, 6, 0}, 
{7, 5, 0}, {7, 6, 0}, {0, 3, 2}, {1, 2, 0}, {1, 3, 0}, {1, 5, 0}, {2, 2, 0}, {2, 6, 0}, {3, 2, 0}, {6, 3, 0}, 
{7, 2, 0}, {7, 3, 0}, {0, 2, 2}, {0, 5, 0}, {0, 7, 0}, {3, 4, 0}, {4, 2, 0}, {4, 4, 0}, {4, 7, 0}, {5, 1, 0},   // +9.100s 9104
{5, 2, 0}, {5, 5, 0}, {5, 7, 0}, {6, 2, 0}, {0, 4, 2}, {1, 0, 0}, {1, 4, 0}, {1, 6, 0}, {2, 0, 0}, {2, 3, 0}, 
{2, 4, 0}, {2, 5, 0}, {3, 0, 0}, {3, 3, 0}, {3, 7, 0}, {5, 4, 0}, {7, 0, 0}, {7, 6, 0}, {0, 0, 2}, {0, 3, 0}, 
{1, 3, 0}, {4, 5, 0}, {5, 0, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {6, 3, 0}, {6, 4, 0}, {7, 3, 0}, {0, 5, 2}, 
{0, 6, 0}, {0, 7, 0}, {1, 5, 0}, {2, 1, 0}, {2, 6, 0}, {3, 1, 0}, {3, 4, 0}, {3, 5, 0}, {4, 2, 0}, {4, 3, 0}, 
{4, 4, 0}, {5, 2, 0}, {0, 1, 2}, {1, 0, 0}, {1, 1, 0}, {1, 4, 0}, {2, 0, 0}, {2, 5, 0}, {3, 0, 0}, {5, 7, 0},   // +9.233s 9232
{6, 5, 0}, {7, 0, 0}, {7, 1, 0}, {0, 0, 2}, {0, 4, 0}, {1, 7, 0}, {3, 6, 0}, {4, 5, 0}, {4, 6, 0}, {5, 0, 0}, 
{5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {6, 4, 0}, {7, 5, 0}, {0, 6, 2}, {1, 2, 0}, {1, 5, 0}, {1, 6, 0}, {2, 1, 0}, 
{2, 2, 0}, {2, 4, 0}, {2, 6, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0}, {4, 3, 0}, {6, 6, 0}, {7, 2, 0}, {0, 1, 2}, 
{0, 2, 0}, {1, 1, 0}, {4, 7, 0}, {5, 2, 0}, {5, 5, 0}, {5, 7, 0}, {6, 5, 0}, {7, 1, 0}, {7, 4, 0}, {0, 4, 2}, 
{0, 5, 0}, {1, 4, 0}, {1, 7, 0}, {2, 3, 0}, {2, 5, 0}, {3, 3, 0}, {3, 6, 0}, {3, 7, 0}, {4, 6, 0}, {5, 4, 0},   // +9.367s 9360
{7, 3, 0}, {7, 5, 0}, {7, 6, 0}, {0, 3, 3}, {1, 2, 0}, {1, 3, 0}, {1, 6, 0}, {2, 2, 0}, {2, 4, 0}, {3, 2, 0}, 
{5, 6, 0}, {6, 6, 0}, {6, 7, 0}, {7, 2, 0}, {0, 2, 2}, {0, 6, 0}, {2, 7, 0}, {3, 4, 0}, {4, 2, 0}, {4, 4, 0}, 
{4, 7, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {7, 4, 0}, {7, 6, 0}, {0, 5, 2}, {1, 0, 0}, {1, 4, 0}, {1, 5, 0}, 
{2, 0, 0}, {2, 3, 0}, {2, 5, 0}, {2, 6, 0}, {3, 0, 0}, {3, 3, 0}, {3, 7, 0}, {6, 0, 0}, {7, 0, 0}, {7, 3, 0}, 
{0, 0, 2}, {0, 3, 0}, {0, 4, 0}, {1, 3, 0}, {4, 1, 0}, {4, 5, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0}, {6, 7, 0},   // +9.500s 9504
{0, 6, 2}, {1, 6, 0}, {2, 1, 0}, {2, 4, 0}, {2, 7, 0}, {3, 1, 0}, {3, 4, 0}, {3, 5, 0}, {4, 2, 0}, {4, 3, 0}, 
{4, 4, 0}, {5, 3, 0}, {7, 1, 0}, {0, 0, 2}, {0, 1, 0}, {1, 0, 0}, {1, 1, 0}, {1, 5, 0}, {2, 0, 0}, {2, 6, 0}, 
{3, 0, 0}, {5, 1, 0}, {5, 5, 0}, {6, 0, 0}, {6, 1, 0}, {7, 0, 0}, {0, 4, 2}, {0, 5, 0}, {1, 4, 0}, {3, 6, 0}, 
{4, 1, 0}, {4, 5, 0}, {4, 6, 0}, {5, 2, 0}, {5, 4, 0}, {5, 7, 0}, {7, 5, 0}, {0, 2, 2}, {1, 1, 0}, {1, 2, 0}, 
{1, 6, 0}, {2, 1, 0}, {2, 2, 0}, {2, 4, 0}, {2, 5, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0}, {4, 3, 0}, {6, 2, 0},   // +9.633s 9632
{7, 1, 0}, {7, 2, 0}, {0, 1, 2}, {0, 6, 0}, {0, 7, 0}, {4, 7, 0}, {5, 1, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, 
{6, 1, 0}, {7, 6, 0}, {0, 5, 2}, {1, 3, 0}, {1, 4, 0}, {1, 5, 0}, {2, 3, 0}, {2, 6, 0}, {3, 3, 0}, {3, 6, 0}, 
{3, 7, 0}, {4, 6, 0}, {5, 2, 0}, {7, 3, 0}, {7, 5, 0}, {0, 2, 2}, {0, 3, 0}, {1, 2, 0}, {2, 2, 0}, {2, 5, 0}, 
{3, 2, 0}, {5, 0, 0}, {5, 7, 0}, {6, 2, 0}, {6, 3, 0}, {7, 2, 0}, {0, 4, 2}, {0, 6, 0}, {0, 7, 0}, {1, 6, 0}, 
{2, 0, 0}, {2, 4, 0}, {3, 4, 0}, {4, 2, 0}, {4, 4, 0}, {4, 7, 0}, {5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {7, 6, 0},   // +9.767s 9760
{0, 0, 2}, {1, 0, 0}, {1, 3, 0}, {1, 5, 0}, {2, 3, 0}, {2, 6, 0}, {3, 0, 0}, {3, 3, 0}, {3, 7, 0}, {6, 4, 0}, 
{7, 0, 0}, {7, 3, 0}, {0, 3, 3}, {0, 5, 0}, {1, 7, 0}, {4, 5, 0}, {5, 0, 0}, {5, 2, 0}, {5, 5, 0}, {5, 7, 0}, 
{6, 3, 0}, {0, 4, 2}, {1, 1, 0}, {1, 4, 0}, {1, 6, 0}, {2, 0, 0}, {2, 1, 0}, {2, 4, 0}, {2, 5, 0}, {3, 1, 0}, 
{3, 4, 0}, {3, 5, 0}, {4, 2, 0}, {4, 3, 0}, {4, 4, 0}, {5, 4, 0}, {7, 1, 0}, {0, 0, 2}, {0, 1, 0}, {1, 0, 0}, 
{3, 0, 0}, {5, 6, 0}, {6, 4, 0}, {6, 5, 0}, {7, 0, 0}, {7, 4, 0}, {0, 5, 2}, {0, 6, 0}, {1, 5, 0}, {1, 7, 0},   // +9.933s 9936
{2, 2, 0}, {2, 6, 0}, {3, 2, 0}, {3, 5, 0}, {3, 6, 0}, {4, 5, 0}, {4, 6, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, 
{7, 5, 0}, {0, 2, 2}, {1, 1, 0}, {1, 2, 0}, {1, 4, 0}, {2, 1, 0}, {2, 5, 0}, {3, 1, 0}, {4, 3, 0}, {6, 6, 0}, 
{7, 1, 0}, {7, 2, 0}, {0, 1, 2}, {0, 4, 0}, {2, 7, 0}, {3, 7, 0}, {4, 7, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0}, 
{6, 5, 0}, {7, 4, 0}, {7, 6, 0}, {0, 6, 2}, {1, 3, 0}, {1, 5, 0}, {1, 6, 0}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, 
{2, 6, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {4, 6, 0}, {5, 3, 0}, {7, 3, 0}, {7, 5, 0}, {0, 2, 2}, {0, 3, 0},   // +10.067s 10064
{1, 2, 0}, {4, 1, 0}, {4, 4, 0}, {5, 5, 0}, {6, 6, 0}, {6, 7, 0}, {7, 2, 0}, {0, 4, 2}, {0, 5, 0}, {1, 4, 0}, 
{2, 0, 0}, {2, 5, 0}, {2, 7, 0}, {3, 0, 0}, {3, 4, 0}, {3, 7, 0}, {4, 2, 0}, {4, 7, 0}, {5, 2, 0}, {5, 4, 0}, 
{5, 7, 0}, {7, 6, 0}, {0, 0, 2}, {1, 0, 0}, {1, 3, 0}, {1, 6, 0}, {2, 3, 0}, {2, 4, 0}, {3, 3, 0}, {5, 1, 0}, 
{5, 6, 0}, {6, 0, 0}, {7, 0, 0}, {7, 3, 0}, {0, 3, 2}, {0, 6, 0}, {3, 5, 0}, {4, 1, 0}, {4, 3, 0}, {4, 4, 0}, 
{4, 5, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {6, 7, 0}, {0, 5, 2}, {1, 1, 0}, {1, 4, 0}, {1, 5, 0}, {2, 0, 0},   // +10.200s 10192
{2, 1, 0}, {2, 5, 0}, {2, 6, 0}, {3, 0, 0}, {3, 1, 0}, {3, 4, 0}, {4, 2, 0}, {7, 1, 0}, {0, 0, 3}, {0, 1, 0}, 
{0, 7, 0}, {1, 0, 0}, {4, 6, 0}, {5, 1, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0}, {6, 0, 0}, {6, 1, 0}, {7, 0, 0}, 
{0, 4, 2}, {0, 6, 0}, {1, 6, 0}, {2, 2, 0}, {2, 4, 0}, {3, 2, 0}, {3, 5, 0}, {3, 6, 0}, {4, 3, 0}, {4, 5, 0}, 
{5, 3, 0}, {7, 5, 0}, {0, 2, 2}, {1, 1, 0}, {1, 2, 0}, {1, 5, 0}, {2, 1, 0}, {2, 6, 0}, {3, 1, 0}, {5, 0, 0}, 
{5, 5, 0}, {6, 1, 0}, {6, 2, 0}, {7, 1, 0}, {7, 2, 0}, {0, 1, 2}, {0, 5, 0}, {0, 7, 0}, {3, 7, 0}, {4, 6, 0},   // +10.333s 10336
{4, 7, 0}, {5, 2, 0}, {5, 4, 0}, {5, 7, 0}, {7, 6, 0}, {0, 4, 2}, {1, 3, 0}, {1, 4, 0}, {1, 6, 0}, {2, 2, 0}, 
{2, 3, 0}, {2, 4, 0}, {2, 5, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {6, 3, 0}, {7, 3, 0}, {7, 5, 0}, {0, 2, 2}, 
{0, 3, 0}, {1, 2, 0}, {1, 7, 0}, {4, 4, 0}, {5, 0, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {6, 2, 0}, {7, 2, 0}, 
{0, 5, 2}, {0, 6, 0}, {1, 5, 0}, {2, 0, 0}, {2, 6, 0}, {3, 0, 0}, {3, 4, 0}, {3, 7, 0}, {4, 2, 0}, {4, 7, 0}, 
{5, 2, 0}, {7, 0, 0}, {7, 6, 0}, {0, 0, 2}, {1, 0, 0}, {1, 3, 0}, {1, 4, 0}, {2, 3, 0}, {2, 5, 0}, {3, 3, 0},   // +10.467s 10464
{5, 7, 0}, {6, 3, 0}, {6, 4, 0}, {7, 3, 0}, {7, 4, 0}, {0, 3, 2}, {0, 4, 0}, {1, 7, 0}, {3, 5, 0}, {4, 3, 0}, 
{4, 4, 0}, {4, 5, 0}, {5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {0, 1, 2}, {0, 6, 0}, {1, 1, 0}, {1, 5, 0}, {1, 6, 0}, 
{2, 0, 0}, {2, 1, 0}, {2, 4, 0}, {2, 6, 0}, {3, 0, 0}, {3, 1, 0}, {3, 4, 0}, {4, 2, 0}, {6, 5, 0}, {7, 0, 0}, 
{7, 1, 0}, {0, 0, 2}, {0, 5, 0}, {1, 0, 0}, {2, 7, 0}, {4, 6, 0}, {5, 2, 0}, {5, 5, 0}, {5, 7, 0}, {6, 4, 0}, 
{7, 4, 0}, {7, 5, 0}, {0, 4, 3}, {1, 2, 0}, {1, 4, 0}, {2, 2, 0}, {2, 5, 0}, {3, 2, 0}, {3, 5, 0}, {3, 6, 0},   // +10.600s 10608
{4, 3, 0}, {4, 5, 0}, {5, 4, 0}, {7, 2, 0}, {0, 1, 2}, {0, 2, 0}, {1, 1, 0}, {1, 6, 0}, {2, 1, 0}, {2, 4, 0}, 
{3, 1, 0}, {4, 1, 0}, {5, 6, 0}, {6, 5, 0}, {6, 6, 0}, {7, 1, 0}, {0, 5, 2}, {0, 6, 0}, {1, 5, 0}, {2, 7, 0}, 
{3, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {7, 5, 0}, {7, 6, 0}, {0, 3, 2}, {1, 2, 0}, 
{1, 3, 0}, {1, 4, 0}, {2, 2, 0}, {2, 3, 0}, {2, 5, 0}, {2, 6, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {6, 7, 0}, 
{7, 2, 0}, {7, 3, 0}, {0, 2, 2}, {0, 4, 0}, {4, 1, 0}, {4, 4, 0}, {5, 1, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0},   // +10.733s 10736
{6, 6, 0}, {0, 6, 2}, {1, 0, 0}, {1, 5, 0}, {1, 6, 0}, {2, 0, 0}, {2, 4, 0}, {2, 6, 0}, {3, 0, 0}, {3, 4, 0}, 
{3, 7, 0}, {4, 2, 0}, {4, 7, 0}, {5, 3, 0}, {7, 0, 0}, {7, 6, 0}, {0, 0, 2}, {0, 3, 0}, {0, 7, 0}, {1, 3, 0}, 
{2, 3, 0}, {3, 3, 0}, {5, 5, 0}, {6, 0, 0}, {6, 7, 0}, {7, 3, 0}, {0, 4, 2}, {0, 5, 0}, {1, 4, 0}, {2, 1, 0}, 
{2, 5, 0}, {3, 5, 0}, {4, 3, 0}, {4, 4, 0}, {4, 5, 0}, {5, 1, 0}, {5, 2, 0}, {5, 4, 0}, {5, 7, 0}, {0, 1, 2}, 
{1, 0, 0}, {1, 1, 0}, {1, 6, 0}, {2, 0, 0}, {2, 4, 0}, {3, 0, 0}, {3, 1, 0}, {3, 4, 0}, {4, 2, 0}, {6, 1, 0},   // +10.867s 10864
{7, 0, 0}, {7, 1, 0}, {0, 0, 2}, {0, 6, 0}, {0, 7, 0}, {3, 6, 0}, {4, 6, 0}, {5, 0, 0}, {5, 3, 0}, {5, 5, 0}, 
{5, 6, 0}, {6, 0, 0}, {7, 5, 0}, {0, 5, 2}, {1, 4, 0}, {1, 5, 0}, {2, 5, 0}, {2, 6, 0}, {3, 1, 0}, {3, 2, 0}, 
{3, 5, 0}, {4, 3, 0}, {4, 5, 0}, {5, 2, 0}, {0, 4, 5}, {0, 6, 0}, {1, 6, 0}, {2, 2, 0}, {2, 4, 0}, {3, 3, 0}, 
{3, 6, 0}, {3, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0}, {7, 6, 0}, {0, 1, 2}, 
{0, 2, 0}, {1, 1, 0}, {1, 2, 0}, {1, 5, 0}, {1, 7, 0}, {2, 1, 0}, {2, 6, 0}, {3, 2, 0}, {6, 2, 0}, {7, 1, 0},   // +11.033s 11040
{7, 2, 0}, {7, 5, 0}, {0, 5, 2}, {3, 4, 0}, {4, 2, 0}, {4, 4, 0}, {4, 7, 0}, {5, 0, 0}, {5, 2, 0}, {5, 5, 0}, 
{6, 1, 0}, {0, 3, 2}, {0, 4, 0}, {1, 3, 0}, {1, 4, 0}, {1, 6, 0}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {2, 5, 0}, 
{3, 0, 0}, {3, 3, 0}, {3, 7, 0}, {5, 4, 0}, {5, 7, 0}, {7, 3, 0}, {7, 6, 0}, {0, 2, 2}, {1, 2, 0}, {1, 7, 0}, 
{4, 5, 0}, {5, 3, 0}, {5, 6, 0}, {6, 2, 0}, {6, 3, 0}, {7, 2, 0}, {7, 4, 0}, {0, 5, 2}, {0, 6, 0}, {1, 5, 0}, 
{2, 6, 0}, {3, 1, 0}, {3, 4, 0}, {3, 5, 0}, {4, 2, 0}, {4, 3, 0}, {4, 4, 0}, {5, 2, 0}, {5, 5, 0}, {0, 0, 2},   // +11.200s 11200
{0, 3, 0}, {1, 0, 0}, {1, 3, 0}, {1, 4, 0}, {2, 0, 0}, {2, 3, 0}, {2, 5, 0}, {2, 7, 0}, {3, 0, 0}, {6, 4, 0}, 
{7, 0, 0}, {7, 3, 0}, {0, 4, 2}, {3, 6, 0}, {4, 5, 0}, {4, 6, 0}, {5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0}, 
{6, 3, 0}, {7, 4, 0}, {0, 6, 2}, {1, 1, 0}, {1, 5, 0}, {1, 6, 0}, {2, 1, 0}, {2, 4, 0}, {2, 6, 0}, {3, 1, 0}, 
{3, 2, 0}, {3, 5, 0}, {4, 3, 0}, {6, 5, 0}, {7, 1, 0}, {7, 5, 0}, {0, 0, 2}, {0, 1, 0}, {1, 0, 0}, {2, 0, 0}, 
{2, 7, 0}, {4, 1, 0}, {4, 7, 0}, {5, 2, 0}, {5, 5, 0}, {6, 4, 0}, {7, 0, 0}, {0, 4, 2}, {0, 5, 0}, {1, 4, 0},   // +11.333s 11328
{2, 2, 0}, {2, 5, 0}, {3, 3, 0}, {3, 6, 0}, {3, 7, 0}, {4, 6, 0}, {5, 4, 0}, {5, 7, 0}, {7, 6, 0}, {0, 2, 2}, 
{1, 1, 0}, {1, 2, 0}, {1, 6, 0}, {2, 1, 0}, {2, 4, 0}, {3, 2, 0}, {5, 1, 0}, {6, 5, 0}, {6, 6, 0}, {7, 1, 0}, 
{7, 2, 0}, {7, 5, 0}, {0, 1, 2}, {0, 6, 0}, {3, 4, 0}, {4, 1, 0}, {4, 2, 0}, {4, 4, 0}, {4, 7, 0}, {5, 2, 0}, 
{5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {0, 5, 3}, {1, 3, 0}, {1, 4, 0}, {1, 5, 0}, {2, 3, 0}, {2, 5, 0}, {2, 6, 0}, 
{3, 0, 0}, {3, 3, 0}, {3, 7, 0}, {6, 7, 0}, {7, 2, 0}, {7, 3, 0}, {7, 6, 0}, {0, 2, 2}, {0, 3, 0}, {0, 7, 0},   // +11.467s 11472
{1, 2, 0}, {2, 2, 0}, {4, 5, 0}, {5, 1, 0}, {5, 4, 0}, {5, 7, 0}, {6, 6, 0}, {0, 4, 2}, {0, 6, 0}, {1, 6, 0}, 
{2, 4, 0}, {3, 1, 0}, {3, 4, 0}, {3, 5, 0}, {4, 2, 0}, {4, 3, 0}, {4, 4, 0}, {5, 3, 0}, {5, 6, 0}, {7, 0, 0}, 
{0, 0, 2}, {0, 3, 0}, {0, 7, 0}, {1, 0, 0}, {1, 3, 0}, {1, 5, 0}, {2, 0, 0}, {2, 3, 0}, {2, 6, 0}, {3, 0, 0}, 
{5, 0, 0}, {6, 0, 0}, {6, 7, 0}, {7, 3, 0}, {0, 4, 2}, {0, 5, 0}, {3, 6, 0}, {4, 5, 0}, {4, 6, 0}, {5, 2, 0}, 
{5, 4, 0}, {5, 5, 0}, {5, 7, 0}, {7, 5, 0}, {0, 1, 2}, {1, 1, 0}, {1, 4, 0}, {1, 6, 0}, {1, 7, 0}, {2, 1, 0},   // +11.600s 11600
{2, 4, 0}, {2, 5, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0}, {4, 3, 0}, {6, 1, 0}, {7, 0, 0}, {7, 1, 0}, {0, 0, 2}, 
{0, 6, 0}, {1, 0, 0}, {2, 0, 0}, {4, 7, 0}, {5, 0, 0}, {5, 3, 0}, {5, 6, 0}, {6, 0, 0}, {0, 5, 2}, {1, 2, 0}, 
{1, 4, 0}, {1, 5, 0}, {2, 6, 0}, {3, 3, 0}, {3, 6, 0}, {3, 7, 0}, {4, 6, 0}, {5, 2, 0}, {5, 5, 0}, {7, 2, 0}, 
{7, 5, 0}, {7, 6, 0}, {0, 1, 2}, {0, 2, 0}, {1, 1, 0}, {1, 7, 0}, {2, 1, 0}, {2, 2, 0}, {2, 5, 0}, {3, 2, 0}, 
{6, 1, 0}, {6, 2, 0}, {7, 1, 0}, {7, 4, 0}, {0, 4, 2}, {0, 6, 0}, {1, 6, 0}, {2, 4, 0}, {3, 4, 0}, {4, 2, 0},   // +11.733s 11728
{4, 4, 0}, {4, 7, 0}, {5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0}, {0, 3, 2}, {1, 2, 0}, {1, 3, 0}, {1, 5, 0}, 
{2, 2, 0}, {2, 3, 0}, {2, 6, 0}, {2, 7, 0}, {3, 0, 0}, {3, 3, 0}, {3, 7, 0}, {6, 3, 0}, {7, 2, 0}, {7, 3, 0}, 
{7, 6, 0}, {0, 2, 2}, {0, 5, 0}, {4, 5, 0}, {5, 2, 0}, {5, 5, 0}, {6, 2, 0}, {7, 4, 0}, {0, 4, 3}, {1, 0, 0}, 
{1, 4, 0}, {1, 6, 0}, {2, 0, 0}, {2, 4, 0}, {2, 5, 0}, {3, 1, 0}, {3, 4, 0}, {3, 5, 0}, {4, 2, 0}, {4, 3, 0}, 
{4, 4, 0}, {5, 4, 0}, {5, 7, 0}, {7, 0, 0}, {0, 0, 2}, {0, 3, 0}, {1, 3, 0}, {2, 3, 0}, {2, 7, 0}, {3, 0, 0},   // +11.867s 11872
{4, 1, 0}, {6, 3, 0}, {6, 4, 0}, {7, 3, 0}, {0, 5, 2}, {0, 6, 0}, {1, 5, 0}, {2, 1, 0}, {2, 6, 0}, {3, 2, 0}, 
{3, 5, 0}, {3, 6, 0}, {4, 5, 0}, {4, 6, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {7, 5, 0}, {0, 1, 2}, 
{1, 0, 0}, {1, 1, 0}, {1, 4, 0}, {2, 0, 0}, {2, 5, 0}, {3, 1, 0}, {4, 3, 0}, {6, 5, 0}, {7, 0, 0}, {7, 1, 0}, 
{0, 0, 2}, {0, 4, 0}, {3, 7, 0}, {4, 1, 0}, {4, 7, 0}, {5, 1, 0}, {5, 4, 0}, {5, 7, 0}, {6, 4, 0}, {0, 6, 2}, 
{1, 2, 0}, {1, 5, 0}, {1, 6, 0}, {2, 1, 0}, {2, 2, 0}, {2, 4, 0}, {2, 6, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0},   // +12.000s 12000
{4, 6, 0}, {5, 3, 0}, {5, 6, 0}, {7, 2, 0}, {7, 5, 0}, {7, 6, 0}, {0, 1, 2}, {0, 2, 0}, {0, 7, 0}, {1, 1, 0}, 
{4, 4, 0}, {6, 5, 0}, {6, 6, 0}, {7, 1, 0}, {0, 4, 2}, {0, 5, 0}, {1, 4, 0}, {2, 5, 0}, {3, 0, 0}, {3, 4, 0}, 
{3, 7, 0}, {4, 2, 0}, {4, 7, 0}, {5, 1, 0}, {5, 2, 0}, {5, 4, 0}, {5, 5, 0}, {5, 7, 0}, {0, 3, 2}, {1, 2, 0}, 
{1, 3, 0}, {1, 6, 0}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {3, 3, 0}, {5, 0, 0}, {6, 7, 0}, {7, 2, 0}, {7, 3, 0}, 
{7, 6, 0}, {0, 2, 2}, {0, 6, 0}, {0, 7, 0}, {3, 5, 0}, {4, 3, 0}, {4, 4, 0}, {4, 5, 0}, {5, 2, 0}, {5, 3, 0},   // +12.133s 12128
{5, 5, 0}, {5, 6, 0}, {6, 6, 0}, {0, 5, 2}, {1, 0, 0}, {1, 4, 0}, {1, 5, 0}, {2, 0, 0}, {2, 5, 0}, {2, 6, 0}, 
{3, 0, 0}, {3, 1, 0}, {3, 4, 0}, {4, 2, 0}, {7, 0, 0}, {0, 0, 2}, {0, 3, 0}, {1, 3, 0}, {1, 7, 0}, {2, 3, 0}, 
{4, 6, 0}, {5, 0, 0}, {5, 4, 0}, {5, 7, 0}, {6, 0, 0}, {6, 7, 0}, {7, 3, 0}, {0, 4, 3}, {0, 6, 0}, {1, 6, 0}, 
{2, 1, 0}, {2, 4, 0}, {3, 2, 0}, {3, 5, 0}, {3, 6, 0}, {4, 3, 0}, {4, 5, 0}, {5, 3, 0}, {5, 6, 0}, {7, 5, 0}, 
{0, 1, 2}, {1, 0, 0}, {1, 1, 0}, {1, 5, 0}, {2, 0, 0}, {2, 6, 0}, {3, 1, 0}, {6, 0, 0}, {6, 1, 0}, {7, 0, 0},   // +12.267s 12272
{7, 1, 0}, {7, 4, 0}, {0, 0, 2}, {0, 5, 0}, {1, 7, 0}, {3, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 2, 0}, {5, 4, 0}, 
{5, 5, 0}, {5, 7, 0}, {0, 4, 2}, {1, 2, 0}, {1, 4, 0}, {1, 6, 0}, {2, 1, 0}, {2, 2, 0}, {2, 4, 0}, {2, 5, 0}, 
{3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {6, 2, 0}, {7, 2, 0}, {7, 5, 0}, {7, 6, 0}, {0, 1, 2}, {0, 2, 0}, {1, 1, 0}, 
{2, 7, 0}, {4, 4, 0}, {5, 3, 0}, {5, 6, 0}, {6, 1, 0}, {7, 1, 0}, {7, 4, 0}, {0, 5, 2}, {0, 6, 0}, {1, 5, 0}, 
{2, 6, 0}, {3, 0, 0}, {3, 4, 0}, {3, 7, 0}, {4, 2, 0}, {4, 7, 0}, {5, 2, 0}, {5, 5, 0}, {7, 3, 0}, {7, 6, 0},   // +12.400s 12400
{0, 3, 2}, {1, 2, 0}, {1, 3, 0}, {1, 4, 0}, {2, 2, 0}, {2, 3, 0}, {2, 5, 0}, {3, 3, 0}, {4, 1, 0}, {6, 2, 0}, 
{6, 3, 0}, {7, 2, 0}, {0, 2, 2}, {0, 4, 0}, {2, 7, 0}, {3, 5, 0}, {4, 3, 0}, {4, 4, 0}, {4, 5, 0}, {5, 3, 0}, 
{5, 4, 0}, {5, 6, 0}, {5, 7, 0}, {0, 0, 2}, {0, 6, 0}, {1, 0, 0}, {1, 5, 0}, {1, 6, 0}, {2, 0, 0}, {2, 4, 0}, 
{2, 6, 0}, {3, 0, 0}, {3, 1, 0}, {3, 4, 0}, {4, 2, 0}, {6, 4, 0}, {7, 0, 0}, {7, 3, 0}, {0, 3, 2}, {0, 5, 0}, 
{1, 3, 0}, {2, 3, 0}, {4, 1, 0}, {4, 6, 0}, {5, 1, 0}, {5, 2, 0}, {5, 5, 0}, {6, 3, 0}, {0, 4, 2}, {1, 4, 0},   // +12.567s 12560
{2, 1, 0}, {2, 5, 0}, {3, 2, 0}, {3, 5, 0}, {3, 6, 0}, {4, 3, 0}, {4, 5, 0}, {5, 4, 0}, {5, 7, 0}, {7, 1, 0}, 
{7, 5, 0}, {0, 0, 2}, {0, 1, 0}, {0, 7, 0}, {1, 0, 0}, {1, 1, 0}, {1, 6, 0}, {2, 0, 0}, {2, 4, 0}, {3, 1, 0}, 
{6, 4, 0}, {6, 5, 0}, {7, 0, 0}, {0, 5, 3}, {0, 6, 0}, {1, 5, 0}, {3, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 1, 0}, 
{5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {7, 6, 0}, {0, 2, 2}, {1, 1, 0}, {1, 2, 0}, {1, 4, 0}, {2, 1, 0}, 
{2, 2, 0}, {2, 5, 0}, {2, 6, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {6, 6, 0}, {7, 1, 0}, {7, 2, 0}, {7, 5, 0},   // +12.667s 12672
{0, 1, 2}, {0, 4, 0}, {0, 7, 0}, {4, 4, 0}, {5, 0, 0}, {5, 4, 0}, {5, 7, 0}, {6, 5, 0}, {0, 6, 2}, {1, 3, 0}, 
{1, 5, 0}, {1, 6, 0}, {2, 3, 0}, {2, 4, 0}, {3, 0, 0}, {3, 4, 0}, {3, 7, 0}, {4, 2, 0}, {4, 7, 0}, {5, 3, 0}, 
{5, 6, 0}, {7, 3, 0}, {7, 6, 0}, {0, 2, 2}, {0, 3, 0}, {1, 2, 0}, {1, 7, 0}, {2, 2, 0}, {2, 6, 0}, {3, 3, 0}, 
{6, 6, 0}, {6, 7, 0}, {7, 2, 0}, {0, 4, 2}, {0, 5, 0}, {1, 4, 0}, {2, 5, 0}, {3, 5, 0}, {4, 3, 0}, {4, 4, 0}, 
{4, 5, 0}, {5, 0, 0}, {5, 2, 0}, {5, 4, 0}, {5, 5, 0}, {5, 7, 0}, {0, 0, 2}, {1, 0, 0}, {1, 3, 0}, {1, 6, 0},   // +12.833s 12832
{2, 0, 0}, {2, 3, 0}, {2, 4, 0}, {3, 0, 0}, {3, 1, 0}, {3, 4, 0}, {4, 2, 0}, {6, 0, 0}, {7, 0, 0}, {7, 3, 0}, 
{7, 4, 0}, {0, 3, 2}, {1, 7, 0}, {5, 6, 0}, {6, 7, 0}, {0, 5, 2}, {0, 6, 0}, {1, 1, 0}, {1, 4, 0}, {1, 5, 0}, 
{2, 1, 0}, {2, 5, 0}, {2, 6, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0}, {3, 6, 0}, {4, 3, 0}, {4, 5, 0}, {4, 6, 0}, 
{5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {7, 1, 0}, {7, 5, 0}, {0, 0, 2}, {0, 1, 0}, {1, 0, 0}, {2, 0, 0}, {2, 7, 0}, 
{6, 0, 0}, {6, 1, 0}, {7, 0, 0}, {7, 4, 0}, {0, 4, 2}, {1, 6, 0}, {3, 3, 0}, {3, 7, 0}, {4, 6, 0}, {4, 7, 0},   // +12.967s 12960
{5, 4, 0}, {5, 6, 0}, {5, 7, 0}, {7, 6, 0}, {0, 2, 2}, {0, 6, 0}, {1, 1, 0}, {1, 2, 0}, {1, 5, 0}, {2, 1, 0}, 
{2, 2, 0}, {2, 4, 0}, {2, 6, 0}, {3, 2, 0}, {3, 6, 0}, {4, 1, 0}, {5, 3, 0}, {6, 2, 0}, {7, 1, 0}, {7, 2, 0}, 
{7, 5, 0}, {0, 1, 3}, {2, 7, 0}, {4, 2, 0}, {4, 4, 0}, {4, 7, 0}, {5, 5, 0}, {6, 1, 0}, {0, 4, 2}, {0, 5, 0}, 
{1, 4, 0}, {1, 6, 0}, {2, 3, 0}, {2, 5, 0}, {3, 0, 0}, {3, 3, 0}, {3, 4, 0}, {3, 7, 0}, {5, 2, 0}, {5, 4, 0}, 
{5, 7, 0}, {7, 6, 0}, {0, 2, 2}, {0, 3, 0}, {1, 2, 0}, {1, 3, 0}, {2, 2, 0}, {2, 4, 0}, {4, 1, 0}, {4, 5, 0},   // +13.100s 13104
{5, 1, 0}, {5, 6, 0}, {6, 2, 0}, {6, 3, 0}, {7, 2, 0}, {7, 3, 0}, {0, 6, 2}, {1, 5, 0}, {3, 1, 0}, {3, 5, 0}, 
{4, 2, 0}, {4, 3, 0}, {4, 4, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {0, 0, 2}, {0, 5, 0}, {1, 0, 0}, {1, 4, 0}, 
{2, 0, 0}, {2, 3, 0}, {2, 5, 0}, {2, 6, 0}, {3, 0, 0}, {3, 4, 0}, {7, 0, 0}, {0, 3, 2}, {0, 7, 0}, {1, 3, 0}, 
{4, 5, 0}, {4, 6, 0}, {5, 1, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0}, {6, 3, 0}, {6, 4, 0}, {7, 3, 0}, {0, 4, 2}, 
{0, 6, 0}, {1, 5, 0}, {1, 6, 0}, {2, 1, 0}, {2, 4, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0}, {3, 6, 0}, {4, 3, 0},   // +13.233s 13232
{5, 3, 0}, {7, 5, 0}, {0, 0, 2}, {0, 1, 0}, {1, 0, 0}, {1, 1, 0}, {2, 0, 0}, {2, 6, 0}, {4, 7, 0}, {5, 0, 0}, 
{5, 5, 0}, {6, 4, 0}, {6, 5, 0}, {7, 0, 0}, {7, 1, 0}, {0, 5, 2}, {0, 7, 0}, {3, 3, 0}, {3, 7, 0}, {4, 6, 0}, 
{5, 2, 0}, {5, 4, 0}, {5, 7, 0}, {7, 6, 0}, {0, 2, 2}, {0, 4, 0}, {1, 2, 0}, {1, 4, 0}, {1, 6, 0}, {2, 1, 0}, 
{2, 2, 0}, {2, 4, 0}, {2, 5, 0}, {3, 2, 0}, {3, 6, 0}, {6, 6, 0}, {7, 2, 0}, {7, 5, 0}, {0, 1, 2}, {1, 1, 0}, 
{1, 7, 0}, {4, 2, 0}, {4, 4, 0}, {4, 7, 0}, {5, 0, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {6, 5, 0}, {7, 1, 0},   // +13.367s 13360
{0, 5, 3}, {0, 6, 0}, {1, 4, 0}, {1, 5, 0}, {2, 3, 0}, {2, 6, 0}, {3, 0, 0}, {3, 3, 0}, {3, 4, 0}, {3, 7, 0}, 
{5, 2, 0}, {7, 3, 0}, {7, 6, 0}, {0, 2, 2}, {0, 3, 0}, {1, 2, 0}, {1, 3, 0}, {2, 2, 0}, {2, 5, 0}, {4, 5, 0}, 
{5, 7, 0}, {6, 6, 0}, {6, 7, 0}, {7, 2, 0}, {7, 4, 0}, {0, 4, 2}, {1, 6, 0}, {1, 7, 0}, {3, 1, 0}, {3, 5, 0}, 
{4, 2, 0}, {4, 3, 0}, {4, 4, 0}, {5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {0, 0, 2}, {0, 6, 0}, {1, 0, 0}, {1, 5, 0}, 
{2, 0, 0}, {2, 3, 0}, {2, 4, 0}, {2, 6, 0}, {3, 0, 0}, {3, 4, 0}, {6, 0, 0}, {7, 0, 0}, {7, 3, 0}, {0, 3, 2},   // +13.533s 13536
{0, 5, 0}, {1, 3, 0}, {2, 7, 0}, {4, 5, 0}, {4, 6, 0}, {5, 2, 0}, {5, 5, 0}, {5, 7, 0}, {6, 7, 0}, {7, 4, 0}, 
{7, 5, 0}, {0, 1, 2}, {0, 4, 0}, {1, 4, 0}, {1, 6, 0}, {2, 1, 0}, {2, 5, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0}, 
{3, 6, 0}, {4, 3, 0}, {5, 4, 0}, {7, 1, 0}, {0, 0, 2}, {1, 0, 0}, {1, 1, 0}, {2, 0, 0}, {2, 4, 0}, {4, 1, 0}, 
{4, 7, 0}, {5, 6, 0}, {6, 0, 0}, {6, 1, 0}, {7, 0, 0}, {0, 5, 2}, {0, 6, 0}, {1, 5, 0}, {2, 7, 0}, {3, 7, 0}, 
{4, 6, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {7, 5, 0}, {7, 6, 0}, {0, 2, 2}, {1, 1, 0}, {1, 2, 0}, {1, 4, 0},   // +13.667s 13664
{2, 2, 0}, {2, 6, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {6, 2, 0}, {7, 1, 0}, {7, 2, 0}, {0, 1, 2}, {0, 4, 0}, 
{2, 1, 0}, {2, 5, 0}, {4, 1, 0}, {4, 2, 0}, {4, 4, 0}, {4, 7, 0}, {5, 1, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0}, 
{6, 1, 0}, {0, 6, 2}, {1, 5, 0}, {1, 6, 0}, {2, 3, 0}, {2, 4, 0}, {3, 0, 0}, {3, 4, 0}, {3, 7, 0}, {5, 3, 0}, 
{7, 6, 0}, {0, 2, 2}, {0, 3, 0}, {0, 7, 0}, {1, 2, 0}, {1, 3, 0}, {2, 2, 0}, {2, 6, 0}, {3, 3, 0}, {4, 5, 0}, 
{5, 5, 0}, {6, 2, 0}, {6, 3, 0}, {7, 2, 0}, {7, 3, 0}, {0, 4, 2}, {0, 5, 0}, {3, 5, 0}, {4, 2, 0}, {4, 3, 0},   // +13.800s 13792
{4, 4, 0}, {5, 1, 0}, {5, 2, 0}, {5, 4, 0}, {5, 7, 0}, {1, 0, 3}, {1, 4, 0}, {1, 6, 0}, {2, 0, 0}, {2, 5, 0}, 
{3, 1, 0}, {3, 4, 0}, {7, 0, 0}, {0, 0, 2}, {0, 3, 0}, {0, 6, 0}, {0, 7, 0}, {1, 3, 0}, {2, 3, 0}, {2, 4, 0}, 
{3, 0, 0}, {3, 6, 0}, {4, 6, 0}, {5, 0, 0}, {5, 3, 0}, {5, 6, 0}, {6, 3, 0}, {6, 4, 0}, {7, 3, 0}, {7, 5, 0}, 
{0, 5, 2}, {3, 5, 0}, {4, 3, 0}, {4, 5, 0}, {5, 2, 0}, {5, 5, 0}, {1, 1, 2}, {1, 4, 0}, {1, 5, 0}, {2, 1, 0}, 
{2, 6, 0}, {3, 1, 0}, {3, 2, 0}, {7, 1, 0}, {0, 0, 2}, {0, 1, 0}, {0, 4, 0}, {1, 0, 0}, {1, 7, 0}, {2, 0, 0},   // +13.967s 13968
{2, 5, 0}, {3, 7, 0}, {4, 7, 0}, {5, 4, 0}, {5, 7, 0}, {6, 4, 0}, {6, 5, 0}, {7, 0, 0}, {0, 6, 2}, {3, 6, 0}, 
{4, 6, 0}, {5, 0, 0}, {5, 3, 0}, {5, 6, 0}, {7, 5, 0}, {7, 6, 0}, {1, 2, 2}, {1, 5, 0}, {1, 6, 0}, {2, 2, 0}, 
{2, 4, 0}, {3, 2, 0}, {3, 3, 0}, {7, 2, 0}, {0, 1, 2}, {0, 2, 0}, {0, 5, 0}, {1, 1, 0}, {1, 7, 0}, {2, 1, 0}, 
{2, 6, 0}, {3, 4, 0}, {4, 2, 0}, {4, 4, 0}, {4, 7, 0}, {5, 2, 0}, {5, 5, 0}, {6, 5, 0}, {6, 6, 0}, {7, 1, 0}, 
{7, 4, 0}, {0, 4, 2}, {3, 7, 0}, {5, 4, 0}, {5, 7, 0}, {7, 6, 0}, {1, 3, 2}, {1, 4, 0}, {1, 6, 0}, {2, 3, 0},   // +14.133s 14128
{2, 5, 0}, {3, 0, 0}, {3, 3, 0}, {7, 3, 0}, {0, 2, 2}, {0, 3, 0}, {0, 6, 0}, {1, 2, 0}, {2, 2, 0}, {2, 4, 0}, 
{2, 7, 0}, {3, 5, 0}, {4, 3, 0}, {4, 4, 0}, {4, 5, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {6, 6, 0}, 
{6, 7, 0}, {7, 2, 0}, {7, 4, 0}, {0, 5, 3}, {3, 4, 0}, {4, 2, 0}, {1, 0, 2}, {1, 4, 0}, {1, 5, 0}, {2, 0, 0}, 
{2, 6, 0}, {3, 0, 0}, {3, 1, 0}, {6, 0, 0}, {7, 0, 0}, {0, 0, 2}, {0, 3, 0}, {0, 4, 0}, {1, 3, 0}, {2, 3, 0}, 
{2, 5, 0}, {2, 7, 0}, {3, 6, 0}, {4, 1, 0}, {4, 5, 0}, {4, 6, 0}, {5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0},   // +14.267s 14272
{6, 7, 0}, {7, 3, 0}, {7, 5, 0}, {0, 6, 2}, {3, 5, 0}, {4, 3, 0}, {1, 1, 2}, {1, 5, 0}, {1, 6, 0}, {2, 1, 0}, 
{2, 4, 0}, {3, 1, 0}, {3, 2, 0}, {6, 1, 0}, {7, 1, 0}, {0, 0, 2}, {0, 1, 0}, {0, 5, 0}, {1, 0, 0}, {2, 0, 0}, 
{2, 6, 0}, {3, 7, 0}, {4, 1, 0}, {4, 6, 0}, {4, 7, 0}, {5, 1, 0}, {5, 2, 0}, {5, 4, 0}, {5, 5, 0}, {5, 7, 0}, 
{6, 0, 0}, {7, 0, 0}, {7, 6, 0}, {0, 4, 2}, {3, 6, 0}, {7, 5, 0}, {1, 2, 2}, {1, 4, 0}, {1, 6, 0}, {2, 2, 0}, 
{2, 5, 0}, {3, 2, 0}, {3, 3, 0}, {6, 2, 0}, {7, 1, 0}, {7, 2, 0}, {0, 1, 2}, {0, 2, 0}, {0, 6, 0}, {0, 7, 0},   // +14.467s 14464
{1, 1, 0}, {2, 1, 0}, {2, 4, 0}, {3, 4, 0}, {4, 2, 0}, {4, 4, 0}, {4, 7, 0}, {5, 1, 0}, {5, 2, 0}, {5, 3, 0}, 
{5, 5, 0}, {5, 6, 0}, {6, 1, 0}, {0, 5, 2}, {3, 7, 0}, {7, 6, 0}, {0, 3, 2}, {1, 3, 0}, {1, 4, 0}, {1, 5, 0}, 
{2, 3, 0}, {2, 6, 0}, {3, 0, 0}, {3, 3, 0}, {6, 3, 0}, {7, 2, 0}, {7, 3, 0}, {0, 2, 2}, {0, 4, 0}, {0, 6, 0}, 
{0, 7, 0}, {1, 2, 0}, {2, 2, 0}, {2, 5, 0}, {3, 5, 0}, {4, 3, 0}, {4, 4, 0}, {4, 5, 0}, {5, 0, 0}, {5, 3, 0}, 
{5, 4, 0}, {5, 6, 0}, {5, 7, 0}, {6, 2, 0}, {1, 6, 2}, {3, 4, 0}, {4, 2, 0}, {0, 0, 3}, {1, 0, 0}, {1, 3, 0},   // +14.633s 14640
{1, 5, 0}, {1, 7, 0}, {2, 0, 0}, {2, 4, 0}, {3, 0, 0}, {3, 1, 0}, {6, 4, 0}, {7, 0, 0}, {7, 3, 0}, {0, 3, 2}, 
{0, 4, 0}, {0, 5, 0}, {2, 3, 0}, {2, 6, 0}, {3, 6, 0}, {4, 5, 0}, {4, 6, 0}, {5, 0, 0}, {5, 2, 0}, {5, 4, 0}, 
{5, 5, 0}, {5, 7, 0}, {6, 3, 0}, {7, 5, 0}, {3, 5, 2}, {4, 3, 0}, {0, 0, 2}, {0, 5, 0}, {1, 0, 0}, {2, 0, 0}, 
{4, 6, 0}, {5, 0, 0}, {5, 2, 0}, {5, 5, 0}, {6, 4, 0}, {7, 5, 0}, {2, 4, 2}, {3, 1, 0}, {3, 6, 0}, {7, 1, 0}, 
{7, 2, 0}, {1, 5, 2}, {2, 0, 0}, {7, 3, 0}, {1, 0, 2}, {1, 4, 0}, {2, 1, 0}, {7, 4, 0}, {1, 1, 2}, {1, 2, 0},   // +14.867s 14864
{2, 2, 0}, {2, 3, 0}, {3, 0, 0}, {5, 5, 0}, {1, 3, 2}, {2, 4, 0}, {2, 5, 0}, {2, 7, 0}, {3, 1, 0}, {3, 2, 0}, 
{2, 6, 2}, {3, 3, 0}, {4, 2, 0}, {7, 6, 0}, {4, 0, 2}, {4, 3, 0}, {5, 6, 0}, {7, 5, 0}, {3, 4, 2}, {4, 1, 0}, 
{5, 7, 0}, {3, 5, 3}, {3, 6, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {5, 1, 0}, {0, 5, 2}, {0, 6, 0}, 
{3, 7, 0}, {0, 4, 2}, {5, 2, 0}, {5, 3, 0}, {5, 4, 2}, {0, 0, 2}, {0, 1, 0}, {0, 2, 2}, {0, 3, 0}, {0, 7, 2}, 
{6, 0, 2}, {6, 1, 2}, {6, 2, 0}, {6, 3, 0}, {6, 4, 2}, {6, 5, 0}, {6, 6, 2}, {6, 7, 3}, {6, 7, 8}, {6, 5, 2},   // +15.567s 15568
{6, 6, 0}, {6, 3, 2}, {6, 4, 0}, {6, 1, 2}, {6, 2, 0}, {0, 7, 2}, {6, 0, 0}, {0, 2, 2}, {0, 3, 0}, {0, 0, 2}, 
{0, 1, 0}, {0, 4, 4}, {5, 3, 0}, {5, 4, 0}, {0, 5, 3}, {0, 6, 0}, {5, 2, 0}, {3, 6, 2}, {3, 7, 0}, {4, 4, 0}, 
{4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {3, 4, 2}, {3, 5, 0}, {4, 1, 0}, {5, 1, 0}, {5, 7, 2}, {2, 6, 2}, {3, 3, 0}, 
{4, 0, 0}, {4, 3, 0}, {5, 5, 0}, {5, 6, 0}, {7, 5, 0}, {1, 3, 2}, {2, 4, 0}, {2, 5, 0}, {2, 7, 0}, {3, 1, 0}, 
{3, 2, 0}, {4, 2, 0}, {7, 6, 0}, {1, 1, 2}, {1, 2, 0}, {2, 2, 0}, {2, 3, 0}, {3, 0, 0}, {1, 0, 2}, {1, 4, 2},   // +16.100s 16096
{1, 5, 0}, {2, 0, 0}, {2, 1, 0}, {7, 4, 0}, {7, 3, 2}, {7, 1, 2}, {7, 2, 0}, {1, 6, 2}, {1, 7, 0}, {5, 0, 0}, 
{7, 0, 0}, {6, 6, 7}, {6, 7, 0}, {6, 4, 2}, {6, 5, 0}, {6, 2, 2}, {6, 3, 0}, {6, 0, 2}, {6, 1, 0}, {0, 7, 2}, 
{0, 2, 2}, {0, 3, 0}, {0, 0, 2}, {0, 1, 0}, {5, 4, 2}, {0, 4, 2}, {5, 2, 0}, {5, 3, 0}, {0, 5, 2}, {0, 6, 0}, 
{3, 7, 0}, {3, 5, 3}, {3, 6, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {5, 1, 0}, {3, 4, 2}, {4, 1, 0}, 
{5, 6, 2}, {5, 7, 0}, {2, 6, 2}, {3, 3, 0}, {4, 0, 0}, {4, 2, 0}, {4, 3, 0}, {5, 5, 0}, {7, 5, 0}, {7, 6, 0},   // +16.733s 16736
{1, 3, 2}, {2, 4, 0}, {2, 5, 0}, {2, 7, 0}, {3, 2, 0}, {1, 1, 2}, {1, 2, 0}, {2, 2, 0}, {2, 3, 0}, {3, 0, 0}, 
{3, 1, 0}, {1, 0, 2}, {1, 4, 0}, {2, 1, 0}, {1, 5, 2}, {2, 0, 0}, {7, 4, 0}, {7, 3, 2}, {1, 6, 2}, {1, 7, 0}, 
{7, 1, 0}, {7, 2, 0}, {5, 0, 2}, {7, 0, 0}, {1, 6, 7}, {1, 7, 0}, {5, 0, 0}, {7, 0, 2}, {7, 1, 0}, {7, 2, 2}, 
{7, 3, 0}, {1, 4, 2}, {1, 5, 0}, {2, 0, 0}, {2, 1, 0}, {7, 4, 0}, {1, 0, 2}, {1, 2, 0}, {2, 2, 0}, {5, 5, 0}, 
{1, 1, 2}, {1, 3, 0}, {2, 3, 0}, {2, 4, 0}, {3, 0, 0}, {3, 1, 0}, {2, 5, 2}, {2, 7, 0}, {3, 2, 0}, {4, 2, 0},   // +17.267s 17264
{2, 6, 2}, {3, 3, 0}, {4, 0, 0}, {4, 3, 0}, {5, 6, 0}, {7, 5, 0}, {7, 6, 0}, {5, 7, 2}, {3, 4, 2}, {3, 5, 0}, 
{4, 1, 0}, {5, 1, 0}, {3, 6, 3}, {3, 7, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {0, 4, 2}, {0, 5, 0}, 
{0, 6, 0}, {5, 2, 0}, {5, 3, 2}, {5, 4, 0}, {0, 0, 2}, {0, 1, 2}, {0, 2, 0}, {0, 3, 0}, {0, 7, 2}, {6, 0, 2}, 
{6, 1, 2}, {6, 2, 0}, {6, 3, 2}, {6, 4, 0}, {6, 5, 2}, {6, 6, 0}, {6, 7, 2}, {1, 7, 25}, {3, 3, 0}, {3, 1, 2}, 
{3, 2, 0}, {6, 0, 0}, {1, 4, 3}, {1, 5, 0}, {3, 0, 0}, {6, 1, 0}, {6, 2, 0}, {1, 6, 2}, {1, 0, 2}, {1, 1, 0},   // +18.267s 18272
{1, 2, 2}, {1, 3, 0}, {4, 2, 2}, {4, 3, 0}, {5, 1, 0}, {5, 2, 0}, {5, 3, 0}, {6, 3, 0}, {7, 0, 0}, {5, 4, 2}, 
{3, 4, 2}, {3, 5, 0}, {3, 6, 2}, {3, 7, 0}, {2, 7, 2}, {4, 4, 0}, {6, 4, 0}, {7, 1, 0}, {4, 5, 2}, {4, 6, 0}, 
{0, 0, 2}, {4, 1, 0}, {4, 7, 0}, {6, 5, 0}, {7, 2, 0}, {0, 1, 2}, {0, 2, 0}, {0, 4, 0}, {0, 5, 0}, {0, 6, 0}, 
{0, 7, 0}, {2, 0, 0}, {4, 0, 0}, {6, 6, 0}, {6, 7, 0}, {7, 3, 0}, {7, 5, 0}, {7, 6, 0}, {2, 1, 2}, {2, 2, 0}, 
{5, 5, 0}, {5, 6, 3}, {5, 7, 0}, {2, 3, 2}, {7, 4, 0}, {0, 3, 2}, {2, 4, 0}, {2, 5, 0}, {2, 6, 2}, {0, 3, 6},   // +18.833s 18832
{2, 5, 0}, {2, 6, 0}, {0, 2, 2}, {2, 3, 0}, {2, 4, 0}, {7, 4, 2}, {2, 2, 2}, {5, 6, 0}, {5, 7, 0}, {0, 1, 2}, 
{0, 4, 0}, {0, 5, 0}, {0, 6, 0}, {0, 7, 0}, {2, 0, 0}, {2, 1, 0}, {5, 5, 0}, {0, 0, 2}, {4, 0, 0}, {6, 6, 0}, 
{6, 7, 0}, {7, 3, 0}, {7, 5, 0}, {7, 6, 0}, {4, 1, 3}, {4, 6, 0}, {4, 7, 0}, {6, 5, 0}, {7, 2, 0}, {4, 4, 2}, 
{4, 5, 0}, {2, 7, 2}, {3, 7, 0}, {6, 4, 0}, {7, 1, 0}, {3, 5, 2}, {3, 6, 0}, {3, 4, 2}, {5, 3, 2}, {5, 4, 0}, 
{1, 3, 2}, {4, 2, 0}, {4, 3, 0}, {5, 1, 0}, {5, 2, 0}, {6, 3, 0}, {7, 0, 0}, {1, 1, 2}, {1, 2, 0}, {1, 0, 2},   // +19.300s 19296
{1, 5, 2}, {1, 6, 0}, {1, 4, 2}, {3, 0, 0}, {3, 1, 0}, {6, 1, 0}, {6, 2, 0}, {1, 7, 3}, {3, 2, 0}, {3, 3, 0}, 
{6, 0, 0}, {0, 3, 6}, {2, 5, 0}, {2, 6, 0}, {0, 2, 2}, {2, 3, 0}, {2, 4, 0}, {7, 4, 0}, {5, 7, 2}, {0, 5, 2}, 
{0, 6, 0}, {2, 2, 0}, {5, 5, 0}, {5, 6, 0}, {0, 0, 2}, {0, 1, 0}, {0, 4, 0}, {0, 7, 0}, {2, 0, 0}, {2, 1, 0}, 
{6, 7, 0}, {4, 0, 2}, {4, 1, 0}, {4, 7, 0}, {6, 5, 0}, {6, 6, 0}, {7, 2, 0}, {7, 3, 0}, {7, 5, 0}, {7, 6, 0}, 
{4, 6, 2}, {2, 7, 2}, {4, 4, 0}, {4, 5, 0}, {3, 7, 2}, {6, 4, 0}, {7, 1, 0}, {3, 5, 3}, {3, 6, 0}, {3, 4, 2},   // +19.833s 19840
{5, 3, 2}, {5, 4, 0}, {1, 2, 2}, {1, 3, 0}, {4, 2, 0}, {4, 3, 0}, {5, 1, 0}, {5, 2, 0}, {6, 3, 0}, {7, 0, 0}, 
{1, 0, 2}, {1, 1, 0}, {1, 6, 2}, {1, 4, 2}, {1, 5, 0}, {3, 0, 0}, {3, 1, 2}, {3, 2, 0}, {6, 1, 0}, {6, 2, 0}, 
{1, 7, 2}, {3, 3, 0}, {6, 0, 0}, {1, 7, 6}, {3, 3, 0}, {3, 1, 2}, {3, 2, 0}, {6, 0, 0}, {1, 4, 3}, {1, 5, 0}, 
{3, 0, 0}, {6, 1, 0}, {6, 2, 0}, {1, 6, 2}, {1, 0, 2}, {1, 1, 0}, {1, 2, 2}, {1, 3, 0}, {4, 2, 2}, {4, 3, 0}, 
{5, 1, 0}, {5, 2, 0}, {5, 3, 0}, {6, 3, 0}, {7, 0, 0}, {5, 4, 2}, {3, 4, 2}, {3, 5, 0}, {3, 6, 2}, {3, 7, 0},   // +20.467s 20464
{2, 7, 2}, {4, 4, 0}, {6, 4, 0}, {7, 1, 0}, {4, 5, 2}, {4, 6, 0}, {0, 0, 2}, {4, 1, 0}, {4, 7, 0}, {6, 5, 0}, 
{7, 2, 0}, {0, 1, 2}, {0, 2, 0}, {0, 4, 0}, {0, 5, 0}, {0, 6, 0}, {0, 7, 0}, {2, 0, 0}, {4, 0, 0}, {6, 6, 0}, 
{6, 7, 0}, {7, 3, 0}, {7, 5, 0}, {7, 6, 0}, {2, 1, 3}, {2, 2, 0}, {5, 5, 0}, {5, 6, 2}, {5, 7, 0}, {2, 3, 2}, 
{7, 4, 0}, {0, 3, 2}, {2, 4, 0}, {2, 5, 0}, {2, 6, 2}, {2, 7, 2}, {2, 7, 6}, {2, 7, 4}, {2, 7, 7}, {2, 7, 4}, 
{2, 7, 6}, {2, 7, 4}, {2, 7, 7}, {2, 7, 4}, {2, 7, 6}, {1, 2, 2}, {1, 3, 0}, {1, 1, 2}, {2, 5, 0}, {3, 0, 0},   // +21.633s 21632
{3, 2, 0}, {3, 4, 0}, {4, 1, 0}, {4, 2, 0}, {4, 3, 0}, {4, 5, 0}, {5, 5, 0}, {5, 6, 0}, {7, 5, 0}, {7, 6, 0}, 
{0, 6, 2}, {1, 0, 0}, {1, 4, 0}, {1, 5, 0}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {3, 1, 0}, {3, 5, 0}, {3, 6, 0}, 
{3, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 1, 0}, {5, 7, 0}, {0, 4, 2}, {0, 5, 0}, {1, 6, 0}, {2, 0, 0}, {2, 1, 0}, 
{4, 4, 0}, {5, 3, 0}, {5, 4, 0}, {7, 0, 0}, {7, 4, 0}, {0, 0, 2}, {0, 1, 0}, {0, 2, 0}, {7, 1, 0}, {7, 2, 0}, 
{7, 3, 0}, {0, 3, 2}, {0, 7, 0}, {2, 6, 0}, {2, 7, 0}, {5, 2, 0}, {3, 3, 2}, {5, 0, 0}, {6, 0, 0}, {6, 1, 0},   // +21.800s 21792
{6, 2, 0}, {6, 3, 3}, {6, 4, 0}, {6, 5, 0}, {6, 6, 0}, {6, 7, 2}, {6, 7, 20}, {6, 3, 3}, {6, 4, 0}, {6, 5, 0}, 
{6, 6, 0}, {0, 3, 2}, {0, 7, 0}, {2, 6, 0}, {2, 7, 0}, {3, 3, 0}, {5, 0, 0}, {5, 2, 0}, {6, 0, 0}, {6, 1, 0}, 
{6, 2, 0}, {0, 0, 2}, {0, 1, 0}, {0, 2, 0}, {1, 6, 2}, {5, 3, 0}, {7, 1, 0}, {7, 2, 0}, {7, 3, 0}, {0, 4, 2}, 
{0, 5, 0}, {0, 6, 0}, {1, 0, 0}, {1, 4, 0}, {1, 5, 0}, {2, 0, 0}, {2, 1, 0}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, 
{3, 1, 0}, {3, 5, 0}, {3, 6, 0}, {3, 7, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {5, 1, 0}, {5, 4, 0},   // +22.367s 22368
{7, 0, 0}, {7, 4, 0}, {1, 1, 2}, {2, 5, 0}, {3, 0, 0}, {3, 2, 0}, {3, 4, 0}, {4, 1, 0}, {5, 5, 0}, {5, 6, 0}, 
{5, 7, 0}, {1, 2, 2}, {1, 3, 0}, {4, 2, 0}, {4, 3, 0}, {7, 5, 0}, {7, 6, 0}, {1, 2, 19}, {1, 3, 0}, {4, 2, 0}, 
{4, 3, 0}, {7, 5, 0}, {7, 6, 0}, {1, 1, 2}, {2, 5, 0}, {3, 0, 0}, {3, 2, 0}, {3, 4, 0}, {4, 1, 0}, {4, 5, 0}, 
{5, 5, 0}, {5, 6, 0}, {0, 6, 2}, {1, 0, 0}, {1, 4, 0}, {1, 5, 0}, {2, 0, 0}, {2, 1, 0}, {2, 2, 0}, {2, 3, 0}, 
{2, 4, 0}, {3, 1, 0}, {3, 5, 0}, {3, 6, 0}, {3, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 1, 0}, {5, 7, 0}, {0, 4, 2},   // +22.833s 22832
{0, 5, 0}, {1, 6, 0}, {4, 4, 0}, {5, 3, 0}, {5, 4, 0}, {7, 0, 0}, {7, 4, 0}, {0, 0, 2}, {0, 1, 0}, {0, 2, 0}, 
{7, 1, 0}, {7, 2, 0}, {7, 3, 0}, {0, 3, 2}, {0, 7, 0}, {2, 6, 0}, {2, 7, 0}, {3, 3, 0}, {5, 0, 2}, {5, 2, 0}, 
{6, 0, 0}, {6, 1, 0}, {6, 2, 0}, {6, 3, 2}, {6, 4, 0}, {6, 5, 0}, {6, 6, 0}, {6, 7, 3}, {6, 5, 20}, {6, 6, 0}, 
{6, 7, 0}, {6, 3, 2}, {6, 4, 0}, {0, 3, 2}, {0, 7, 0}, {2, 6, 0}, {2, 7, 0}, {3, 3, 0}, {5, 0, 0}, {5, 2, 0}, 
{6, 0, 0}, {6, 1, 0}, {6, 2, 0}, {0, 0, 3}, {0, 1, 0}, {0, 2, 0}, {7, 3, 0}, {1, 6, 2}, {5, 3, 0}, {7, 0, 0},   // +23.467s 23472
{7, 1, 0}, {7, 2, 0}, {0, 4, 2}, {0, 5, 0}, {0, 6, 0}, {1, 4, 0}, {1, 5, 0}, {2, 0, 0}, {2, 1, 0}, {4, 4, 0}, 
{5, 4, 0}, {7, 4, 0}, {1, 0, 2}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {3, 1, 0}, {3, 5, 0}, {3, 6, 0}, {3, 7, 0}, 
{4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {5, 1, 0}, {5, 7, 0}, {1, 1, 2}, {1, 2, 0}, {1, 3, 0}, {2, 5, 0}, {3, 0, 0}, 
{3, 2, 0}, {3, 4, 0}, {4, 1, 0}, {4, 2, 0}, {4, 3, 0}, {5, 5, 0}, {5, 6, 0}, {7, 5, 0}, {7, 6, 0}, {1, 2, 19}, 
{1, 3, 0}, {1, 1, 2}, {2, 5, 0}, {3, 0, 0}, {3, 2, 0}, {3, 4, 0}, {4, 1, 0}, {4, 2, 0}, {4, 3, 0}, {4, 5, 0},   // +23.900s 23904
{5, 5, 0}, {5, 6, 0}, {7, 5, 0}, {7, 6, 0}, {0, 6, 2}, {1, 0, 0}, {1, 4, 0}, {1, 5, 0}, {2, 0, 0}, {2, 1, 0}, 
{2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {3, 1, 0}, {3, 5, 0}, {3, 6, 0}, {3, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 1, 0}, 
{5, 7, 0}, {0, 4, 2}, {0, 5, 0}, {1, 6, 0}, {4, 4, 0}, {5, 3, 0}, {5, 4, 0}, {7, 0, 0}, {7, 4, 0}, {0, 0, 2}, 
{0, 1, 0}, {0, 2, 0}, {7, 1, 0}, {7, 2, 0}, {7, 3, 0}, {0, 3, 2}, {0, 7, 0}, {2, 6, 0}, {2, 7, 0}, {3, 3, 0}, 
{5, 2, 0}, {5, 0, 2}, {6, 0, 0}, {6, 1, 0}, {6, 2, 0}, {6, 3, 2}, {6, 4, 0}, {6, 5, 0}, {6, 6, 0}, {6, 7, 2},   // +24.133s 24128
{6, 5, 21}, {6, 6, 0}, {6, 7, 0}, {6, 1, 2}, {6, 2, 0}, {6, 3, 0}, {6, 4, 0}, {0, 1, 2}, {0, 2, 0}, {0, 3, 0}, 
{0, 7, 0}, {2, 6, 0}, {2, 7, 0}, {3, 3, 0}, {5, 0, 0}, {5, 2, 0}, {6, 0, 0}, {0, 0, 2}, {7, 3, 0}, {0, 4, 2}, 
{0, 5, 0}, {1, 6, 0}, {5, 3, 0}, {7, 0, 0}, {7, 1, 0}, {7, 2, 0}, {0, 6, 3}, {1, 0, 0}, {1, 4, 0}, {1, 5, 0}, 
{2, 0, 0}, {2, 1, 0}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {3, 1, 0}, {3, 5, 0}, {3, 6, 0}, {3, 7, 0}, {4, 4, 0}, 
{4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {5, 1, 0}, {5, 4, 0}, {7, 4, 0}, {1, 1, 2}, {1, 2, 0}, {1, 3, 0}, {2, 5, 0},   // +24.667s 24672
{3, 0, 0}, {3, 2, 0}, {3, 4, 0}, {4, 1, 0}, {5, 5, 0}, {5, 6, 0}, {5, 7, 0}, {4, 2, 2}, {4, 3, 0}, {7, 5, 0}, 
{7, 6, 0}, {1, 7, 18}, {2, 0, 0}, {2, 1, 0}, {2, 2, 0}, {7, 2, 0}, {7, 3, 0}, {2, 3, 3}, {2, 4, 0}, {2, 5, 0}, 
{2, 6, 0}, {7, 4, 0}, {4, 0, 2}, {4, 1, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {5, 5, 0}, {5, 6, 0}, 
{5, 7, 0}, {7, 5, 0}, {7, 6, 0}, {0, 0, 2}, {0, 1, 0}, {0, 4, 0}, {0, 5, 0}, {0, 6, 0}, {0, 2, 2}, {0, 3, 0}, 
{0, 7, 0}, {6, 0, 2}, {6, 1, 0}, {6, 2, 0}, {6, 3, 2}, {6, 4, 0}, {6, 5, 0}, {6, 6, 0}, {3, 7, 2}, {5, 2, 0},   // +25.233s 25232
{5, 3, 0}, {5, 4, 0}, {6, 7, 0}, {2, 7, 2}, {3, 1, 0}, {3, 2, 0}, {3, 3, 0}, {3, 4, 0}, {3, 5, 0}, {3, 6, 0}, 
{4, 2, 0}, {4, 3, 0}, {5, 1, 0}, {1, 0, 2}, {1, 1, 0}, {1, 2, 0}, {1, 3, 0}, {3, 0, 0}, {1, 4, 2}, {1, 5, 0}, 
{1, 6, 0}, {5, 0, 0}, {7, 0, 0}, {7, 1, 0}, {1, 6, 21}, {1, 0, 2}, {1, 1, 0}, {1, 4, 0}, {1, 5, 0}, {5, 0, 0}, 
{7, 0, 0}, {7, 1, 0}, {1, 2, 2}, {1, 3, 0}, {3, 0, 0}, {2, 7, 2}, {3, 1, 0}, {3, 2, 0}, {3, 3, 0}, {3, 4, 0}, 
{3, 5, 0}, {3, 6, 0}, {4, 2, 0}, {4, 3, 0}, {5, 1, 0}, {3, 7, 2}, {5, 2, 0}, {5, 3, 0}, {5, 4, 0}, {6, 5, 3},   // +25.833s 25840
{6, 6, 0}, {6, 7, 0}, {0, 7, 2}, {6, 1, 0}, {6, 2, 0}, {6, 3, 0}, {6, 4, 0}, {0, 0, 2}, {0, 1, 0}, {0, 2, 0}, 
{0, 3, 0}, {6, 0, 0}, {0, 4, 2}, {0, 5, 0}, {0, 6, 0}, {2, 5, 2}, {2, 6, 0}, {4, 0, 0}, {4, 1, 0}, {4, 4, 0}, 
{4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {5, 5, 0}, {2, 1, 2}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {5, 6, 0}, {5, 7, 0}, 
{7, 5, 0}, {7, 6, 0}, {1, 7, 2}, {2, 0, 0}, {7, 2, 0}, {7, 3, 0}, {7, 4, 0}, {1, 4, 33}, {1, 5, 0}, {1, 6, 0}, 
{5, 0, 0}, {7, 0, 0}, {7, 1, 0}, {1, 0, 3}, {1, 1, 0}, {1, 2, 0}, {1, 3, 0}, {3, 0, 0}, {2, 7, 2}, {3, 1, 0},   // +26.633s 26640
{3, 2, 0}, {3, 3, 0}, {3, 4, 0}, {4, 2, 0}, {3, 5, 4}, {3, 6, 0}, {4, 3, 0}, {5, 1, 0}, {3, 7, 2}, {5, 2, 0}, 
{5, 3, 0}, {5, 4, 0}, {6, 7, 2}, {6, 5, 4}, {6, 6, 0}, {0, 7, 2}, {6, 1, 0}, {6, 2, 0}, {6, 3, 0}, {6, 4, 0}, 
{0, 0, 2}, {0, 1, 0}, {0, 2, 0}, {0, 3, 0}, {6, 0, 0}, {0, 4, 2}, {0, 5, 0}, {0, 6, 0}, {2, 5, 2}, {2, 6, 0}, 
{4, 0, 0}, {4, 1, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {5, 5, 0}, {2, 1, 3}, {2, 2, 0}, {2, 3, 0}, 
{2, 4, 0}, {5, 6, 0}, {5, 7, 0}, {7, 5, 0}, {7, 6, 0}, {1, 7, 2}, {2, 0, 0}, {7, 2, 0}, {7, 3, 0}, {7, 4, 0},   // +27.033s 27040
{1, 7, 20}, {2, 0, 0}, {2, 1, 3}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {7, 2, 0}, {7, 3, 0}, {7, 4, 0}, {2, 5, 2}, 
{2, 6, 0}, {4, 0, 0}, {4, 1, 0}, {4, 7, 0}, {5, 5, 0}, {5, 6, 0}, {5, 7, 0}, {7, 5, 0}, {7, 6, 0}, {0, 4, 2}, 
{0, 5, 0}, {0, 6, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 0}, {0, 0, 2}, {0, 1, 0}, {0, 2, 0}, {0, 3, 0}, {0, 7, 2}, 
{6, 0, 0}, {6, 1, 0}, {6, 2, 0}, {6, 3, 2}, {6, 4, 0}, {6, 5, 0}, {6, 6, 0}, {5, 2, 2}, {5, 3, 0}, {6, 7, 0}, 
{3, 5, 2}, {3, 6, 0}, {3, 7, 0}, {4, 3, 0}, {5, 1, 0}, {5, 4, 0}, {1, 0, 2}, {1, 1, 0}, {1, 2, 0}, {1, 3, 0},   // +27.667s 27664
{2, 7, 0}, {3, 0, 0}, {3, 1, 0}, {3, 2, 0}, {3, 3, 0}, {3, 4, 0}, {4, 2, 0}, {1, 4, 2}, {1, 5, 0}, {1, 6, 0}, 
{7, 0, 0}, {7, 1, 0}, {5, 0, 2}, {1, 7, 19}, {2, 0, 0}, {2, 1, 0}, {2, 2, 0}, {7, 2, 0}, {7, 3, 0}, {2, 3, 2}, 
{2, 4, 0}, {2, 5, 0}, {2, 6, 0}, {7, 4, 0}, {4, 0, 2}, {4, 1, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 0}, {4, 7, 0}, 
{5, 5, 0}, {5, 6, 0}, {5, 7, 0}, {7, 5, 0}, {7, 6, 0}, {0, 0, 2}, {0, 1, 0}, {0, 4, 0}, {0, 5, 0}, {0, 6, 0}, 
{0, 2, 2}, {0, 3, 0}, {0, 7, 0}, {6, 0, 3}, {6, 1, 0}, {6, 2, 0}, {6, 3, 2}, {6, 4, 0}, {6, 5, 0}, {6, 6, 0},   // +28.233s 28240
{6, 7, 0}, {3, 7, 2}, {5, 2, 0}, {5, 3, 0}, {5, 4, 0}, {2, 7, 2}, {3, 1, 0}, {3, 2, 0}, {3, 3, 0}, {3, 4, 0}, 
{3, 5, 0}, {3, 6, 0}, {4, 2, 0}, {4, 3, 0}, {5, 1, 0}, {1, 0, 2}, {1, 1, 0}, {1, 2, 0}, {1, 3, 0}, {3, 0, 0}, 
{1, 4, 2}, {1, 5, 0}, {1, 6, 0}, {5, 0, 0}, {7, 0, 0}, {7, 1, 0}, {1, 6, 21}, {1, 0, 2}, {1, 1, 0}, {1, 4, 0}, 
{1, 5, 0}, {5, 0, 0}, {7, 0, 0}, {7, 1, 0}, {1, 2, 2}, {1, 3, 0}, {2, 7, 0}, {3, 0, 0}, {3, 1, 2}, {3, 2, 0}, 
{3, 3, 0}, {3, 4, 0}, {3, 5, 0}, {3, 6, 0}, {4, 2, 0}, {4, 3, 0}, {5, 1, 0}, {3, 7, 2}, {5, 2, 0}, {5, 3, 0},   // +28.833s 28832
{5, 4, 0}, {6, 5, 2}, {6, 6, 0}, {6, 7, 0}, {0, 7, 2}, {6, 1, 0}, {6, 2, 0}, {6, 3, 0}, {6, 4, 0}, {0, 0, 2}, 
{0, 1, 0}, {0, 2, 0}, {0, 3, 0}, {6, 0, 0}, {0, 4, 2}, {0, 5, 0}, {0, 6, 0}, {2, 5, 3}, {2, 6, 0}, {4, 0, 0}, 
{4, 1, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {5, 5, 0}, {2, 1, 2}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, 
{5, 6, 0}, {5, 7, 0}, {7, 5, 0}, {7, 6, 0}, {1, 7, 2}, {2, 0, 0}, {7, 2, 0}, {7, 3, 0}, {7, 4, 0}, {1, 6, 18}, 
{5, 0, 0}, {1, 0, 3}, {1, 1, 0}, {1, 4, 0}, {1, 5, 0}, {7, 0, 0}, {7, 1, 0}, {1, 2, 2}, {1, 3, 0}, {2, 7, 0},   // +29.433s 29440
{3, 0, 0}, {3, 1, 0}, {3, 2, 0}, {3, 3, 0}, {3, 4, 0}, {4, 2, 0}, {3, 5, 2}, {3, 6, 0}, {3, 7, 0}, {4, 3, 0}, 
{5, 1, 0}, {5, 4, 0}, {5, 2, 2}, {5, 3, 0}, {6, 7, 0}, {6, 3, 2}, {6, 4, 0}, {6, 5, 0}, {6, 6, 0}, {0, 2, 2}, 
{0, 3, 0}, {0, 7, 0}, {6, 0, 0}, {6, 1, 0}, {6, 2, 0}, {0, 0, 2}, {0, 1, 0}, {0, 4, 0}, {0, 5, 0}, {0, 6, 0}, 
{4, 4, 2}, {4, 5, 0}, {4, 6, 0}, {2, 5, 2}, {2, 6, 0}, {4, 0, 0}, {4, 1, 0}, {4, 7, 0}, {5, 5, 0}, {5, 6, 0}, 
{5, 7, 0}, {7, 5, 0}, {7, 6, 0}, {1, 7, 2}, {2, 1, 0}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {7, 4, 0}, {2, 0, 2},   // +29.733s 29728
{7, 2, 0}, {7, 3, 0}, {1, 7, 21}, {2, 0, 0}, {2, 1, 0}, {2, 2, 0}, {7, 2, 0}, {7, 3, 0}, {2, 3, 2}, {2, 4, 0}, 
{2, 5, 0}, {2, 6, 0}, {7, 4, 0}, {4, 0, 2}, {4, 1, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {5, 5, 0}, 
{5, 6, 0}, {5, 7, 0}, {7, 5, 0}, {7, 6, 0}, {0, 0, 2}, {0, 1, 0}, {0, 4, 0}, {0, 5, 0}, {0, 6, 0}, {0, 2, 3}, 
{0, 3, 0}, {0, 7, 0}, {6, 0, 2}, {6, 1, 0}, {6, 2, 0}, {6, 3, 2}, {6, 4, 0}, {6, 5, 0}, {6, 6, 0}, {3, 7, 6}, 
{5, 2, 0}, {5, 3, 0}, {5, 4, 0}, {6, 7, 0}, {2, 7, 2}, {3, 1, 0}, {3, 2, 0}, {3, 3, 0}, {3, 4, 0}, {3, 5, 0},   // +30.400s 30400
{3, 6, 0}, {4, 2, 0}, {4, 3, 0}, {5, 1, 0}, {1, 0, 6}, {1, 1, 0}, {1, 2, 0}, {1, 3, 0}, {3, 0, 0}, {1, 4, 2}, 
{1, 5, 0}, {1, 6, 0}, {5, 0, 0}, {7, 0, 0}, {7, 1, 0}, {5, 5, 21}, {0, 0, 2}, {0, 5, 0}, {1, 1, 0}, {1, 4, 0}, 
{2, 1, 0}, {2, 5, 0}, {3, 1, 0}, {3, 5, 0}, {4, 3, 0}, {4, 5, 0}, {5, 2, 0}, {7, 1, 0}, {0, 0, 2}, {0, 1, 0}, 
{1, 7, 0}, {5, 0, 0}, {6, 1, 0}, {0, 6, 2}, {1, 5, 0}, {3, 2, 0}, {3, 6, 0}, {4, 5, 0}, {4, 6, 0}, {5, 0, 0}, 
{5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {7, 5, 0}, {0, 2, 2}, {0, 5, 0}, {1, 1, 0}, {1, 2, 0}, {1, 4, 0}, {2, 1, 0},   // +31.000s 30992
{2, 2, 0}, {2, 5, 0}, {2, 6, 0}, {3, 1, 0}, {3, 5, 0}, {4, 3, 0}, {5, 2, 0}, {6, 2, 0}, {7, 1, 0}, {7, 2, 0}, 
{7, 4, 0}, {0, 1, 3}, {1, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 7, 0}, {6, 1, 0}, {0, 4, 2}, {0, 6, 0}, {1, 6, 0}, 
{2, 3, 0}, {2, 4, 0}, {3, 3, 0}, {3, 6, 0}, {3, 7, 0}, {5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {7, 5, 0}, {7, 6, 0}, 
{0, 2, 2}, {0, 3, 0}, {1, 2, 0}, {1, 3, 0}, {1, 5, 0}, {2, 2, 0}, {2, 6, 0}, {2, 7, 0}, {3, 2, 0}, {4, 4, 0}, 
{5, 5, 0}, {6, 2, 0}, {6, 3, 0}, {7, 2, 0}, {7, 3, 0}, {0, 5, 2}, {1, 4, 0}, {3, 0, 0}, {3, 4, 0}, {4, 2, 0},   // +31.133s 31136
{4, 7, 0}, {5, 2, 0}, {5, 4, 0}, {5, 7, 0}, {7, 4, 0}, {0, 0, 2}, {0, 4, 0}, {1, 0, 0}, {1, 6, 0}, {2, 0, 0}, 
{2, 3, 0}, {2, 4, 0}, {2, 5, 0}, {3, 3, 0}, {3, 7, 0}, {4, 1, 0}, {7, 0, 0}, {7, 6, 0}, {0, 3, 2}, {1, 3, 0}, 
{2, 7, 0}, {4, 3, 0}, {4, 4, 0}, {4, 5, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {6, 3, 0}, {6, 4, 0}, {7, 3, 0}, 
{0, 5, 2}, {0, 6, 0}, {1, 4, 0}, {1, 5, 0}, {2, 1, 0}, {2, 6, 0}, {3, 0, 0}, {3, 1, 0}, {3, 4, 0}, {3, 5, 0}, 
{4, 2, 0}, {5, 2, 0}, {0, 0, 2}, {0, 1, 0}, {1, 0, 0}, {1, 1, 0}, {2, 0, 0}, {2, 5, 0}, {4, 6, 0}, {5, 1, 0},   // +31.267s 31264
{5, 7, 0}, {6, 4, 0}, {6, 5, 0}, {7, 0, 0}, {7, 1, 0}, {0, 4, 2}, {1, 6, 0}, {3, 2, 0}, {3, 6, 0}, {4, 1, 0}, 
{4, 3, 0}, {4, 5, 0}, {5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {7, 5, 0}, {0, 2, 2}, {0, 6, 0}, {1, 2, 0}, {1, 5, 0}, 
{2, 1, 0}, {2, 2, 0}, {2, 4, 0}, {2, 6, 0}, {3, 1, 0}, {3, 5, 0}, {6, 6, 0}, {7, 2, 0}, {0, 1, 2}, {0, 7, 0}, 
{1, 1, 0}, {4, 6, 0}, {4, 7, 0}, {5, 1, 0}, {5, 2, 0}, {5, 5, 0}, {5, 7, 0}, {6, 5, 0}, {7, 1, 0}, {7, 6, 0}, 
{0, 4, 3}, {0, 5, 0}, {1, 4, 0}, {1, 6, 0}, {2, 3, 0}, {2, 5, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {3, 7, 0},   // +31.400s 31408
{5, 4, 0}, {7, 3, 0}, {7, 5, 0}, {0, 2, 2}, {0, 3, 0}, {1, 2, 0}, {1, 3, 0}, {2, 2, 0}, {2, 4, 0}, {4, 4, 0}, 
{5, 0, 0}, {5, 6, 0}, {6, 6, 0}, {6, 7, 0}, {7, 2, 0}, {0, 6, 2}, {0, 7, 0}, {1, 5, 0}, {3, 0, 0}, {3, 4, 0}, 
{4, 2, 0}, {4, 7, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {7, 6, 0}, {0, 0, 2}, {0, 5, 0}, {1, 0, 0}, {1, 4, 0}, 
{2, 0, 0}, {2, 3, 0}, {2, 5, 0}, {2, 6, 0}, {3, 3, 0}, {3, 7, 0}, {6, 0, 0}, {7, 0, 0}, {7, 3, 0}, {0, 3, 2}, 
{0, 4, 0}, {1, 3, 0}, {1, 7, 0}, {4, 3, 0}, {4, 4, 0}, {4, 5, 0}, {5, 0, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0},   // +31.533s 31536
{6, 7, 0}, {0, 1, 2}, {0, 6, 0}, {1, 6, 0}, {2, 1, 0}, {2, 4, 0}, {3, 1, 0}, {3, 4, 0}, {3, 5, 0}, {4, 2, 0}, 
{5, 3, 0}, {7, 1, 0}, {0, 0, 2}, {1, 0, 0}, {1, 1, 0}, {1, 5, 0}, {2, 0, 0}, {2, 6, 0}, {3, 0, 0}, {4, 6, 0}, 
{5, 5, 0}, {6, 0, 0}, {6, 1, 0}, {7, 0, 0}, {7, 4, 0}, {0, 4, 2}, {0, 5, 0}, {1, 4, 0}, {1, 7, 0}, {3, 6, 0}, 
{4, 3, 0}, {4, 5, 0}, {5, 2, 0}, {5, 4, 0}, {5, 7, 0}, {7, 5, 0}, {0, 1, 2}, {0, 2, 0}, {1, 1, 0}, {1, 2, 0}, 
{1, 6, 0}, {2, 1, 0}, {2, 2, 0}, {2, 4, 0}, {2, 5, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0}, {6, 2, 0}, {7, 1, 0},   // +31.667s 31664
{7, 2, 0}, {0, 6, 2}, {2, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {6, 1, 0}, {7, 4, 0}, 
{7, 6, 0}, {0, 3, 2}, {0, 5, 0}, {1, 3, 0}, {1, 4, 0}, {1, 5, 0}, {2, 3, 0}, {2, 6, 0}, {3, 3, 0}, {3, 6, 0}, 
{3, 7, 0}, {5, 2, 0}, {7, 3, 0}, {7, 5, 0}, {0, 2, 2}, {1, 2, 0}, {2, 2, 0}, {2, 5, 0}, {2, 7, 0}, {3, 2, 0}, 
{4, 1, 0}, {4, 4, 0}, {5, 7, 0}, {6, 2, 0}, {6, 3, 0}, {7, 2, 0}, {0, 4, 3}, {0, 6, 0}, {1, 6, 0}, {2, 0, 0}, 
{2, 4, 0}, {3, 4, 0}, {4, 2, 0}, {4, 7, 0}, {5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {7, 6, 0}, {0, 0, 2}, {0, 3, 0},   // +31.833s 31840
{1, 0, 0}, {1, 3, 0}, {1, 5, 0}, {2, 3, 0}, {2, 6, 0}, {3, 0, 0}, {3, 3, 0}, {3, 7, 0}, {5, 1, 0}, {6, 4, 0}, 
{7, 0, 0}, {7, 3, 0}, {0, 5, 2}, {3, 5, 0}, {4, 1, 0}, {4, 3, 0}, {4, 4, 0}, {4, 5, 0}, {5, 2, 0}, {5, 5, 0}, 
{5, 7, 0}, {6, 3, 0}, {0, 1, 2}, {0, 4, 0}, {1, 1, 0}, {1, 4, 0}, {1, 6, 0}, {2, 0, 0}, {2, 1, 0}, {2, 4, 0}, 
{2, 5, 0}, {3, 0, 0}, {3, 1, 0}, {3, 4, 0}, {4, 2, 0}, {5, 4, 0}, {7, 1, 0}, {0, 0, 2}, {0, 7, 0}, {1, 0, 0}, 
{4, 6, 0}, {5, 1, 0}, {5, 6, 0}, {6, 4, 0}, {6, 5, 0}, {7, 0, 0}, {0, 5, 2}, {0, 6, 0}, {1, 5, 0}, {2, 2, 0},   // +31.967s 31968
{2, 6, 0}, {3, 2, 0}, {3, 5, 0}, {3, 6, 0}, {4, 3, 0}, {4, 5, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {7, 5, 0}, 
{0, 1, 2}, {0, 2, 0}, {1, 1, 0}, {1, 2, 0}, {1, 4, 0}, {2, 1, 0}, {2, 5, 0}, {3, 1, 0}, {4, 7, 0}, {5, 0, 0}, 
{6, 6, 0}, {7, 1, 0}, {7, 2, 0}, {0, 4, 2}, {0, 7, 0}, {3, 7, 0}, {4, 6, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0}, 
{6, 5, 0}, {7, 6, 0}, {0, 3, 2}, {0, 6, 0}, {1, 3, 0}, {1, 5, 0}, {1, 6, 0}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, 
{2, 6, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {5, 3, 0}, {7, 3, 0}, {7, 5, 0}, {0, 2, 2}, {1, 2, 0}, {1, 7, 0},   // +32.100s 32096
{4, 2, 0}, {4, 4, 0}, {4, 7, 0}, {5, 0, 0}, {5, 2, 0}, {5, 5, 0}, {6, 6, 0}, {6, 7, 0}, {7, 2, 0}, {0, 4, 2}, 
{0, 5, 0}, {1, 4, 0}, {2, 0, 0}, {2, 5, 0}, {3, 0, 0}, {3, 4, 0}, {3, 7, 0}, {5, 4, 0}, {5, 7, 0}, {7, 6, 0}, 
{0, 0, 2}, {0, 3, 0}, {1, 0, 0}, {1, 3, 0}, {1, 6, 0}, {2, 3, 0}, {2, 4, 0}, {3, 3, 0}, {4, 5, 0}, {5, 6, 0}, 
{6, 0, 0}, {7, 0, 0}, {7, 3, 0}, {7, 4, 0}, {0, 6, 2}, {1, 7, 0}, {3, 5, 0}, {4, 2, 0}, {4, 3, 0}, {4, 4, 0}, 
{5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {6, 7, 0}, {0, 1, 3}, {0, 5, 0}, {1, 1, 0}, {1, 4, 0}, {1, 5, 0}, {2, 0, 0},   // +32.233s 32240
{2, 1, 0}, {2, 5, 0}, {2, 6, 0}, {3, 0, 0}, {3, 1, 0}, {3, 4, 0}, {6, 1, 0}, {7, 1, 0}, {0, 0, 2}, {1, 0, 0}, 
{2, 7, 0}, {4, 5, 0}, {4, 6, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0}, {6, 0, 0}, {7, 0, 0}, {7, 4, 0}, {0, 4, 2}, 
{0, 6, 0}, {1, 6, 0}, {2, 2, 0}, {2, 4, 0}, {3, 2, 0}, {3, 5, 0}, {3, 6, 0}, {4, 3, 0}, {5, 3, 0}, {7, 5, 0}, 
{0, 1, 2}, {0, 2, 0}, {1, 1, 0}, {1, 2, 0}, {1, 5, 0}, {2, 1, 0}, {2, 6, 0}, {3, 1, 0}, {4, 1, 0}, {4, 7, 0}, 
{5, 5, 0}, {6, 1, 0}, {6, 2, 0}, {7, 1, 0}, {7, 2, 0}, {0, 5, 2}, {2, 7, 0}, {3, 7, 0}, {4, 6, 0}, {5, 2, 0},   // +32.367s 32368
{5, 4, 0}, {5, 7, 0}, {7, 6, 0}, {0, 3, 2}, {0, 4, 0}, {1, 3, 0}, {1, 4, 0}, {1, 6, 0}, {2, 2, 0}, {2, 3, 0}, 
{2, 4, 0}, {2, 5, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {5, 1, 0}, {6, 3, 0}, {7, 2, 0}, {7, 3, 0}, {7, 5, 0}, 
{0, 2, 2}, {1, 2, 0}, {4, 1, 0}, {4, 2, 0}, {4, 4, 0}, {4, 7, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {6, 2, 0}, 
{0, 5, 2}, {0, 6, 0}, {1, 5, 0}, {2, 0, 0}, {2, 6, 0}, {3, 0, 0}, {3, 4, 0}, {3, 7, 0}, {5, 2, 0}, {7, 0, 0}, 
{7, 6, 0}, {0, 0, 2}, {0, 3, 0}, {0, 7, 0}, {1, 0, 0}, {1, 3, 0}, {1, 4, 0}, {2, 3, 0}, {2, 5, 0}, {3, 3, 0},   // +32.500s 32496
{4, 5, 0}, {5, 1, 0}, {5, 7, 0}, {6, 3, 0}, {6, 4, 0}, {7, 3, 0}, {0, 4, 2}, {0, 6, 0}, {3, 5, 0}, {4, 2, 0}, 
{4, 3, 0}, {4, 4, 0}, {5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {0, 0, 2}, {0, 1, 0}, {1, 1, 0}, {1, 5, 0}, {1, 6, 0}, 
{2, 0, 0}, {2, 1, 0}, {2, 4, 0}, {2, 6, 0}, {3, 0, 0}, {3, 1, 0}, {3, 4, 0}, {5, 0, 0}, {6, 5, 0}, {7, 0, 0}, 
{7, 1, 0}, {0, 5, 3}, {0, 7, 0}, {1, 0, 0}, {4, 5, 0}, {4, 6, 0}, {5, 2, 0}, {5, 5, 0}, {5, 7, 0}, {6, 4, 0}, 
{7, 5, 0}, {0, 2, 2}, {0, 4, 0}, {1, 2, 0}, {1, 4, 0}, {1, 6, 0}, {2, 2, 0}, {2, 5, 0}, {3, 2, 0}, {3, 5, 0},   // +32.633s 32640
{3, 6, 0}, {4, 3, 0}, {5, 4, 0}, {7, 2, 0}, {0, 1, 2}, {1, 1, 0}, {1, 7, 0}, {2, 1, 0}, {2, 4, 0}, {3, 1, 0}, 
{4, 7, 0}, {5, 0, 0}, {5, 6, 0}, {6, 5, 0}, {6, 6, 0}, {7, 1, 0}, {0, 5, 2}, {0, 6, 0}, {1, 5, 0}, {3, 7, 0}, 
{4, 6, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {7, 5, 0}, {7, 6, 0}, {0, 2, 2}, {0, 3, 0}, {1, 2, 0}, {1, 3, 0}, 
{1, 4, 0}, {2, 2, 0}, {2, 3, 0}, {2, 5, 0}, {2, 6, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {6, 7, 0}, {7, 2, 0}, 
{7, 3, 0}, {7, 4, 0}, {0, 4, 2}, {1, 7, 0}, {4, 2, 0}, {4, 4, 0}, {4, 7, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0},   // +32.767s 32768
{6, 6, 0}, {0, 0, 2}, {0, 6, 0}, {1, 0, 0}, {1, 5, 0}, {1, 6, 0}, {2, 0, 0}, {2, 3, 0}, {2, 4, 0}, {2, 6, 0}, 
{3, 0, 0}, {3, 4, 0}, {3, 7, 0}, {5, 3, 0}, {7, 0, 0}, {7, 6, 0}, {0, 3, 2}, {1, 3, 0}, {2, 7, 0}, {3, 3, 0}, 
{4, 5, 0}, {5, 5, 0}, {6, 0, 0}, {6, 7, 0}, {7, 3, 0}, {7, 4, 0}, {0, 4, 2}, {0, 5, 0}, {1, 4, 0}, {2, 1, 0}, 
{2, 5, 0}, {3, 1, 0}, {3, 5, 0}, {4, 2, 0}, {4, 3, 0}, {4, 4, 0}, {5, 2, 0}, {5, 4, 0}, {5, 7, 0}, {0, 0, 2}, 
{0, 1, 0}, {1, 0, 0}, {1, 1, 0}, {1, 6, 0}, {2, 0, 0}, {2, 4, 0}, {3, 0, 0}, {3, 4, 0}, {4, 1, 0}, {6, 1, 0},   // +32.900s 32896
{7, 0, 0}, {7, 1, 0}, {0, 6, 2}, {2, 7, 0}, {3, 6, 0}, {4, 5, 0}, {4, 6, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, 
{6, 0, 0}, {7, 5, 0}, {0, 2, 2}, {0, 5, 0}, {1, 2, 0}, {1, 4, 0}, {1, 5, 0}, {2, 1, 0}, {2, 2, 0}, {2, 5, 0}, 
{2, 6, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0}, {4, 3, 0}, {5, 2, 0}, {7, 2, 0}, {0, 1, 2}, {1, 1, 0}, {4, 1, 0}, 
{4, 7, 0}, {5, 1, 0}, {5, 7, 0}, {6, 1, 0}, {6, 2, 0}, {7, 1, 0}, {0, 4, 3}, {0, 6, 0}, {1, 6, 0}, {2, 3, 0}, 
{2, 4, 0}, {3, 3, 0}, {3, 6, 0}, {3, 7, 0}, {4, 6, 0}, {5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {7, 5, 0}, {7, 6, 0},   // +33.033s 33040
{0, 2, 2}, {0, 3, 0}, {0, 7, 0}, {1, 2, 0}, {1, 3, 0}, {1, 5, 0}, {2, 2, 0}, {2, 6, 0}, {3, 2, 0}, {4, 4, 0}, 
{6, 3, 0}, {7, 2, 0}, {7, 3, 0}, {0, 5, 2}, {3, 4, 0}, {4, 2, 0}, {4, 7, 0}, {5, 1, 0}, {5, 2, 0}, {5, 5, 0}, 
{5, 7, 0}, {6, 2, 0}, {0, 0, 2}, {0, 4, 0}, {1, 0, 0}, {1, 4, 0}, {1, 6, 0}, {2, 0, 0}, {2, 3, 0}, {2, 4, 0}, 
{2, 5, 0}, {3, 0, 0}, {3, 3, 0}, {3, 7, 0}, {5, 0, 0}, {5, 4, 0}, {7, 0, 0}, {7, 6, 0}, {0, 3, 2}, {0, 7, 0}, 
{1, 3, 0}, {4, 3, 0}, {4, 4, 0}, {4, 5, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {6, 3, 0}, {6, 4, 0}, {7, 3, 0},   // +33.167s 33168
{0, 5, 2}, {0, 6, 0}, {1, 5, 0}, {2, 1, 0}, {2, 6, 0}, {3, 1, 0}, {3, 4, 0}, {3, 5, 0}, {4, 2, 0}, {5, 2, 0}, 
{0, 0, 2}, {0, 1, 0}, {1, 0, 0}, {1, 1, 0}, {1, 4, 0}, {1, 7, 0}, {2, 0, 0}, {2, 5, 0}, {3, 0, 0}, {4, 6, 0}, 
{5, 0, 0}, {5, 7, 0}, {6, 5, 0}, {7, 0, 0}, {7, 1, 0}, {0, 4, 2}, {3, 6, 0}, {4, 3, 0}, {4, 5, 0}, {5, 3, 0}, 
{5, 4, 0}, {5, 6, 0}, {6, 4, 0}, {7, 5, 0}, {0, 2, 2}, {0, 6, 0}, {1, 2, 0}, {1, 5, 0}, {1, 6, 0}, {2, 1, 0}, 
{2, 2, 0}, {2, 4, 0}, {2, 6, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0}, {6, 6, 0}, {7, 2, 0}, {0, 1, 2}, {1, 1, 0},   // +33.333s 33328
{1, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 2, 0}, {5, 5, 0}, {5, 7, 0}, {6, 5, 0}, {7, 1, 0}, {7, 4, 0}, {0, 4, 2}, 
{0, 5, 0}, {1, 4, 0}, {2, 3, 0}, {2, 5, 0}, {3, 3, 0}, {3, 6, 0}, {3, 7, 0}, {5, 4, 0}, {7, 3, 0}, {7, 5, 0}, 
{7, 6, 0}, {0, 2, 3}, {0, 3, 0}, {1, 2, 0}, {1, 3, 0}, {1, 6, 0}, {2, 2, 0}, {2, 4, 0}, {2, 7, 0}, {3, 2, 0}, 
{4, 4, 0}, {5, 6, 0}, {6, 6, 0}, {6, 7, 0}, {7, 2, 0}, {7, 4, 0}, {0, 6, 2}, {3, 4, 0}, {4, 2, 0}, {4, 7, 0}, 
{5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {7, 6, 0}, {0, 0, 2}, {0, 5, 0}, {1, 0, 0}, {1, 4, 0}, {1, 5, 0}, {2, 0, 0},   // +33.467s 33472
{2, 3, 0}, {2, 5, 0}, {2, 6, 0}, {3, 0, 0}, {3, 3, 0}, {3, 7, 0}, {4, 1, 0}, {6, 0, 0}, {7, 0, 0}, {7, 3, 0}, 
{0, 3, 2}, {1, 3, 0}, {2, 7, 0}, {4, 4, 0}, {4, 5, 0}, {5, 6, 0}, {5, 7, 0}, {6, 7, 0}, {0, 4, 4}, {0, 6, 0}, 
{1, 6, 0}, {3, 1, 0}, {3, 5, 0}, {4, 3, 0}, {5, 3, 0}, {5, 4, 0}, {7, 1, 0}, {0, 0, 2}, {0, 1, 0}, {1, 1, 0}, 
{1, 5, 0}, {2, 0, 0}, {2, 1, 0}, {2, 4, 0}, {2, 6, 0}, {3, 0, 0}, {3, 4, 0}, {4, 2, 0}, {4, 6, 0}, {5, 1, 0}, 
{5, 5, 0}, {6, 0, 0}, {6, 1, 0}, {7, 0, 0}, {1, 0, 2}, {4, 1, 0}, {4, 5, 0}, {5, 7, 0}, {0, 4, 4}, {0, 5, 0},   // +33.700s 33696
{1, 2, 0}, {1, 4, 0}, {1, 6, 0}, {2, 5, 0}, {3, 2, 0}, {3, 6, 0}, {4, 3, 0}, {5, 2, 0}, {5, 4, 0}, {7, 2, 0}, 
{7, 5, 0}, {0, 1, 2}, {0, 2, 0}, {0, 7, 0}, {1, 1, 0}, {2, 1, 0}, {2, 2, 0}, {2, 4, 0}, {3, 1, 0}, {3, 5, 0}, 
{4, 7, 0}, {5, 6, 0}, {6, 1, 0}, {6, 2, 0}, {7, 1, 0}, {4, 6, 2}, {5, 1, 0}, {5, 5, 0}, {0, 6, 3}, {1, 5, 0}, 
{2, 6, 0}, {5, 3, 0}, {7, 6, 0}, {0, 3, 2}, {0, 5, 0}, {1, 2, 0}, {1, 3, 0}, {1, 4, 0}, {2, 2, 0}, {2, 3, 0}, 
{2, 5, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {3, 7, 0}, {5, 2, 0}, {6, 3, 0}, {7, 2, 0}, {7, 3, 0}, {7, 5, 0},   // +33.833s 33840
{0, 2, 2}, {0, 7, 0}, {4, 4, 0}, {4, 7, 0}, {5, 0, 0}, {5, 6, 0}, {5, 7, 0}, {6, 2, 0}, {0, 4, 2}, {0, 6, 0}, 
{1, 0, 0}, {1, 5, 0}, {1, 6, 0}, {2, 0, 0}, {2, 4, 0}, {3, 0, 0}, {3, 3, 0}, {3, 4, 0}, {3, 7, 0}, {4, 2, 0}, 
{5, 3, 0}, {5, 4, 0}, {7, 0, 0}, {7, 6, 0}, {0, 0, 2}, {0, 3, 0}, {1, 3, 0}, {1, 7, 0}, {2, 3, 0}, {2, 6, 0}, 
{4, 5, 0}, {5, 5, 0}, {6, 3, 0}, {6, 4, 0}, {7, 3, 0}, {0, 5, 2}, {1, 4, 0}, {2, 5, 0}, {3, 1, 0}, {3, 5, 0}, 
{4, 3, 0}, {4, 4, 0}, {5, 0, 0}, {5, 2, 0}, {5, 7, 0}, {0, 1, 2}, {0, 4, 0}, {1, 1, 0}, {1, 6, 0}, {2, 0, 0},   // +34.000s 34000
{2, 1, 0}, {2, 4, 0}, {3, 0, 0}, {3, 4, 0}, {4, 2, 0}, {4, 6, 0}, {5, 4, 0}, {6, 5, 0}, {7, 0, 0}, {7, 1, 0}, 
{0, 0, 2}, {1, 0, 0}, {1, 7, 0}, {4, 5, 0}, {5, 5, 0}, {5, 6, 0}, {6, 4, 0}, {7, 4, 0}, {0, 5, 2}, {0, 6, 0}, 
{1, 2, 0}, {1, 4, 0}, {1, 5, 0}, {2, 2, 0}, {2, 5, 0}, {2, 6, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0}, {3, 6, 0}, 
{4, 3, 0}, {5, 2, 0}, {5, 3, 0}, {7, 2, 0}, {7, 5, 0}, {0, 1, 2}, {0, 2, 0}, {1, 1, 0}, {2, 1, 0}, {2, 7, 0}, 
{4, 6, 0}, {4, 7, 0}, {5, 7, 0}, {6, 5, 0}, {6, 6, 0}, {7, 1, 0}, {0, 4, 2}, {1, 6, 0}, {3, 3, 0}, {3, 7, 0},   // +34.133s 34128
{5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {7, 4, 0}, {7, 6, 0}, {0, 3, 2}, {0, 6, 0}, {1, 2, 0}, {1, 3, 0}, {1, 5, 0}, 
{2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {2, 6, 0}, {3, 2, 0}, {3, 6, 0}, {4, 4, 0}, {5, 5, 0}, {6, 7, 0}, {7, 2, 0}, 
{7, 3, 0}, {7, 5, 0}, {0, 2, 2}, {2, 7, 0}, {4, 1, 0}, {4, 2, 0}, {4, 7, 0}, {5, 2, 0}, {5, 7, 0}, {6, 6, 0}, 
{0, 4, 3}, {0, 5, 0}, {1, 0, 0}, {1, 4, 0}, {1, 6, 0}, {2, 0, 0}, {2, 5, 0}, {3, 0, 0}, {3, 3, 0}, {3, 4, 0}, 
{3, 7, 0}, {5, 4, 0}, {6, 0, 0}, {7, 0, 0}, {7, 6, 0}, {0, 0, 2}, {0, 3, 0}, {1, 3, 0}, {2, 3, 0}, {2, 4, 0},   // +34.267s 34272
{4, 4, 0}, {4, 5, 0}, {5, 1, 0}, {5, 5, 0}, {5, 6, 0}, {6, 7, 0}, {7, 3, 0}, {0, 6, 2}, {1, 5, 0}, {3, 1, 0}, 
{3, 5, 0}, {4, 1, 0}, {4, 2, 0}, {4, 3, 0}, {5, 2, 0}, {5, 3, 0}, {0, 1, 2}, {0, 5, 0}, {1, 0, 0}, {1, 1, 0}, 
{1, 4, 0}, {2, 0, 0}, {2, 1, 0}, {2, 5, 0}, {2, 6, 0}, {3, 0, 0}, {3, 4, 0}, {4, 6, 0}, {5, 7, 0}, {6, 0, 0}, 
{6, 1, 0}, {7, 0, 0}, {7, 1, 0}, {0, 0, 2}, {0, 7, 0}, {4, 5, 0}, {5, 1, 0}, {5, 4, 0}, {5, 6, 0}, {7, 5, 0}, 
{0, 4, 2}, {0, 6, 0}, {1, 2, 0}, {1, 5, 0}, {1, 6, 0}, {2, 2, 0}, {2, 4, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0},   // +34.400s 34400
{3, 6, 0}, {4, 3, 0}, {5, 3, 0}, {6, 2, 0}, {7, 1, 0}, {7, 2, 0}, {0, 1, 2}, {0, 2, 0}, {1, 1, 0}, {2, 1, 0}, 
{2, 6, 0}, {4, 6, 0}, {4, 7, 0}, {5, 0, 0}, {5, 5, 0}, {5, 7, 0}, {6, 1, 0}, {0, 5, 2}, {0, 7, 0}, {1, 4, 0}, 
{2, 5, 0}, {3, 3, 0}, {3, 7, 0}, {5, 2, 0}, {5, 4, 0}, {7, 3, 0}, {7, 5, 0}, {7, 6, 0}, {0, 3, 2}, {0, 4, 0}, 
{1, 2, 0}, {1, 3, 0}, {1, 6, 0}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {3, 2, 0}, {3, 6, 0}, {4, 4, 0}, {5, 6, 0}, 
{6, 2, 0}, {6, 3, 0}, {7, 2, 0}, {0, 2, 2}, {0, 6, 0}, {1, 7, 0}, {4, 2, 0}, {4, 7, 0}, {5, 0, 0}, {5, 3, 0},   // +34.533s 34528
{5, 5, 0}, {0, 0, 2}, {0, 5, 0}, {1, 0, 0}, {1, 4, 0}, {1, 5, 0}, {2, 0, 0}, {2, 5, 0}, {2, 6, 0}, {3, 0, 0}, 
{3, 3, 0}, {3, 4, 0}, {3, 7, 0}, {5, 2, 0}, {6, 4, 0}, {7, 0, 0}, {7, 3, 0}, {7, 6, 0}, {0, 3, 3}, {1, 3, 0}, 
{2, 3, 0}, {4, 4, 0}, {4, 5, 0}, {5, 6, 0}, {5, 7, 0}, {6, 3, 0}, {7, 4, 0}, {0, 4, 2}, {0, 6, 0}, {1, 1, 0}, 
{1, 5, 0}, {1, 6, 0}, {1, 7, 0}, {3, 1, 0}, {3, 5, 0}, {4, 2, 0}, {4, 3, 0}, {5, 3, 0}, {5, 4, 0}, {7, 1, 0}, 
{0, 0, 2}, {0, 1, 0}, {1, 0, 0}, {2, 0, 0}, {2, 1, 0}, {2, 4, 0}, {2, 6, 0}, {3, 0, 0}, {3, 4, 0}, {4, 6, 0},   // +34.667s 34672
{5, 5, 0}, {6, 4, 0}, {6, 5, 0}, {7, 0, 0}, {0, 5, 2}, {1, 4, 0}, {2, 7, 0}, {4, 5, 0}, {5, 2, 0}, {5, 7, 0}, 
{7, 4, 0}, {7, 5, 0}, {0, 2, 2}, {0, 4, 0}, {1, 1, 0}, {1, 2, 0}, {1, 6, 0}, {2, 2, 0}, {2, 5, 0}, {3, 1, 0}, 
{3, 2, 0}, {3, 5, 0}, {3, 6, 0}, {4, 3, 0}, {5, 4, 0}, {6, 6, 0}, {7, 1, 0}, {7, 2, 0}, {0, 1, 2}, {2, 1, 0}, 
{2, 4, 0}, {2, 7, 0}, {4, 1, 0}, {4, 6, 0}, {4, 7, 0}, {5, 5, 0}, {5, 6, 0}, {6, 5, 0}, {0, 5, 2}, {0, 6, 0}, 
{1, 4, 0}, {1, 5, 0}, {2, 3, 0}, {2, 6, 0}, {3, 7, 0}, {5, 2, 0}, {5, 3, 0}, {7, 5, 0}, {7, 6, 0}, {0, 2, 2},   // +34.833s 34832
{0, 3, 0}, {1, 2, 0}, {1, 3, 0}, {2, 2, 0}, {2, 5, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {4, 4, 0}, {5, 1, 0}, 
{5, 7, 0}, {6, 6, 0}, {6, 7, 0}, {7, 2, 0}, {7, 3, 0}, {0, 4, 2}, {1, 6, 0}, {3, 0, 0}, {3, 4, 0}, {4, 1, 0}, 
{4, 2, 0}, {4, 7, 0}, {5, 4, 0}, {5, 6, 0}, {0, 0, 2}, {0, 6, 0}, {1, 0, 0}, {1, 5, 0}, {2, 0, 0}, {2, 3, 0}, 
{2, 4, 0}, {2, 6, 0}, {3, 3, 0}, {3, 7, 0}, {5, 3, 0}, {7, 0, 0}, {7, 6, 0}, {0, 3, 2}, {0, 7, 0}, {1, 3, 0}, 
{4, 5, 0}, {5, 1, 0}, {5, 5, 0}, {5, 7, 0}, {6, 0, 0}, {6, 7, 0}, {7, 3, 0}, {0, 4, 2}, {0, 5, 0}, {1, 4, 0},   // +34.967s 34960
{1, 6, 0}, {2, 1, 0}, {2, 5, 0}, {3, 0, 0}, {3, 1, 0}, {3, 4, 0}, {3, 5, 0}, {4, 2, 0}, {4, 3, 0}, {4, 4, 0}, 
{5, 2, 0}, {5, 4, 0}, {0, 0, 2}, {0, 1, 0}, {1, 0, 0}, {1, 1, 0}, {2, 0, 0}, {2, 4, 0}, {4, 6, 0}, {5, 6, 0}, 
{6, 1, 0}, {7, 0, 0}, {7, 1, 0}, {0, 6, 3}, {0, 7, 0}, {1, 5, 0}, {3, 2, 0}, {3, 6, 0}, {4, 3, 0}, {4, 5, 0}, 
{5, 0, 0}, {5, 3, 0}, {5, 5, 0}, {6, 0, 0}, {7, 5, 0}, {0, 2, 2}, {0, 5, 0}, {1, 2, 0}, {1, 4, 0}, {2, 1, 0}, 
{2, 2, 0}, {2, 5, 0}, {2, 6, 0}, {3, 1, 0}, {3, 5, 0}, {4, 7, 0}, {5, 2, 0}, {7, 2, 0}, {0, 1, 2}, {1, 1, 0},   // +35.100s 35104
{1, 7, 0}, {4, 6, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0}, {6, 1, 0}, {6, 2, 0}, {7, 1, 0}, {0, 4, 2}, {0, 6, 0}, 
{1, 5, 0}, {1, 6, 0}, {2, 3, 0}, {2, 4, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {3, 7, 0}, {5, 0, 0}, {5, 3, 0}, 
{7, 5, 0}, {7, 6, 0}, {0, 2, 2}, {0, 3, 0}, {1, 2, 0}, {1, 3, 0}, {2, 2, 0}, {2, 6, 0}, {4, 4, 0}, {4, 7, 0}, 
{5, 5, 0}, {5, 7, 0}, {6, 3, 0}, {7, 2, 0}, {7, 3, 0}, {0, 5, 2}, {1, 4, 0}, {1, 7, 0}, {2, 5, 0}, {3, 0, 0}, 
{3, 4, 0}, {4, 2, 0}, {5, 2, 0}, {5, 4, 0}, {6, 2, 0}, {7, 4, 0}, {0, 0, 2}, {0, 4, 0}, {1, 0, 0}, {1, 6, 0},   // +35.233s 35232
{2, 0, 0}, {2, 3, 0}, {2, 4, 0}, {3, 3, 0}, {3, 7, 0}, {4, 5, 0}, {5, 6, 0}, {6, 4, 0}, {7, 0, 0}, {7, 6, 0}, 
{0, 3, 2}, {1, 3, 0}, {2, 7, 0}, {4, 4, 0}, {5, 5, 0}, {6, 3, 0}, {7, 3, 0}, {4, 3, 2}, {5, 3, 0}, {7, 4, 0}, 
{0, 5, 2}, {0, 6, 0}, {1, 1, 0}, {1, 4, 0}, {1, 5, 0}, {2, 1, 0}, {2, 6, 0}, {3, 0, 0}, {3, 1, 0}, {3, 4, 0}, 
{3, 5, 0}, {4, 2, 0}, {5, 2, 0}, {6, 5, 0}, {7, 1, 0}, {0, 0, 2}, {0, 1, 0}, {1, 0, 0}, {2, 0, 0}, {2, 5, 0}, 
{2, 7, 0}, {4, 1, 0}, {4, 5, 0}, {4, 6, 0}, {5, 6, 0}, {5, 7, 0}, {6, 4, 0}, {7, 0, 0}, {0, 4, 3}, {1, 6, 0},   // +35.400s 35408
{3, 2, 0}, {3, 6, 0}, {4, 3, 0}, {5, 3, 0}, {5, 4, 0}, {7, 2, 0}, {7, 5, 0}, {0, 2, 2}, {0, 6, 0}, {1, 1, 0}, 
{1, 2, 0}, {1, 5, 0}, {2, 1, 0}, {2, 2, 0}, {2, 4, 0}, {2, 6, 0}, {3, 1, 0}, {3, 5, 0}, {4, 7, 0}, {5, 1, 0}, 
{5, 5, 0}, {6, 5, 0}, {6, 6, 0}, {7, 1, 0}, {0, 1, 2}, {4, 1, 0}, {4, 6, 0}, {5, 2, 0}, {5, 7, 0}, {7, 6, 0}, 
{0, 3, 2}, {0, 4, 0}, {0, 5, 0}, {1, 4, 0}, {1, 6, 0}, {2, 3, 0}, {2, 5, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, 
{3, 7, 0}, {5, 4, 0}, {7, 3, 0}, {7, 5, 0}, {0, 2, 2}, {0, 7, 0}, {1, 2, 0}, {1, 3, 0}, {2, 2, 0}, {2, 4, 0},   // +35.533s 35536
{4, 4, 0}, {5, 1, 0}, {5, 5, 0}, {5, 6, 0}, {6, 6, 0}, {6, 7, 0}, {7, 2, 0}, {0, 5, 2}, {0, 6, 0}, {1, 5, 0}, 
{3, 0, 0}, {3, 4, 0}, {4, 2, 0}, {4, 7, 0}, {5, 2, 0}, {5, 3, 0}, {7, 6, 0}, {0, 0, 2}, {0, 3, 0}, {1, 0, 0}, 
{1, 4, 0}, {2, 0, 0}, {2, 3, 0}, {2, 5, 0}, {2, 6, 0}, {3, 3, 0}, {3, 7, 0}, {4, 5, 0}, {5, 7, 0}, {6, 0, 0}, 
{7, 0, 0}, {7, 3, 0}, {0, 4, 2}, {0, 7, 0}, {1, 3, 0}, {1, 6, 0}, {4, 3, 0}, {4, 4, 0}, {5, 0, 0}, {5, 4, 0}, 
{5, 6, 0}, {6, 7, 0}, {0, 1, 2}, {0, 6, 0}, {1, 1, 0}, {1, 5, 0}, {2, 1, 0}, {2, 4, 0}, {3, 0, 0}, {3, 1, 0},   // +35.667s 35664
{3, 4, 0}, {3, 5, 0}, {4, 2, 0}, {5, 3, 0}, {7, 1, 0}, {0, 0, 2}, {1, 0, 0}, {1, 7, 0}, {2, 0, 0}, {2, 6, 0}, 
{4, 5, 0}, {4, 6, 0}, {5, 5, 0}, {5, 7, 0}, {6, 0, 0}, {6, 1, 0}, {7, 0, 0}, {0, 4, 2}, {0, 5, 0}, {1, 4, 0}, 
{1, 6, 0}, {2, 2, 0}, {2, 5, 0}, {3, 2, 0}, {3, 6, 0}, {4, 3, 0}, {5, 0, 0}, {5, 2, 0}, {5, 4, 0}, {7, 5, 0}, 
{0, 1, 2}, {0, 2, 0}, {1, 1, 0}, {1, 2, 0}, {2, 1, 0}, {2, 4, 0}, {3, 1, 0}, {3, 5, 0}, {4, 7, 0}, {5, 6, 0}, 
{6, 2, 0}, {7, 1, 0}, {7, 2, 0}, {0, 6, 2}, {1, 5, 0}, {1, 7, 0}, {4, 6, 0}, {5, 3, 0}, {5, 5, 0}, {6, 1, 0},   // +35.800s 35792
{7, 4, 0}, {7, 6, 0}, {0, 3, 3}, {0, 5, 0}, {1, 3, 0}, {1, 4, 0}, {2, 2, 0}, {2, 3, 0}, {2, 5, 0}, {2, 6, 0}, 
{3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {3, 7, 0}, {5, 2, 0}, {7, 3, 0}, {7, 5, 0}, {0, 2, 2}, {1, 2, 0}, {2, 7, 0}, 
{4, 4, 0}, {4, 7, 0}, {5, 6, 0}, {5, 7, 0}, {6, 2, 0}, {6, 3, 0}, {7, 2, 0}, {0, 4, 2}, {0, 6, 0}, {1, 5, 0}, 
{1, 6, 0}, {2, 0, 0}, {2, 4, 0}, {3, 0, 0}, {3, 3, 0}, {3, 4, 0}, {3, 7, 0}, {4, 2, 0}, {5, 3, 0}, {5, 4, 0}, 
{7, 4, 0}, {7, 6, 0}, {0, 0, 2}, {0, 3, 0}, {1, 0, 0}, {1, 3, 0}, {2, 3, 0}, {2, 6, 0}, {4, 5, 0}, {5, 5, 0},   // +35.933s 35936
{6, 4, 0}, {7, 0, 0}, {7, 3, 0}, {0, 5, 2}, {1, 4, 0}, {2, 7, 0}, {3, 1, 0}, {3, 5, 0}, {4, 1, 0}, {4, 3, 0}, 
{4, 4, 0}, {5, 2, 0}, {5, 7, 0}, {6, 3, 0}, {0, 1, 2}, {0, 4, 0}, {1, 1, 0}, {1, 6, 0}, {2, 0, 0}, {2, 1, 0}, 
{2, 4, 0}, {2, 5, 0}, {3, 0, 0}, {3, 4, 0}, {4, 2, 0}, {4, 6, 0}, {5, 4, 0}, {7, 1, 0}, {0, 0, 2}, {1, 0, 0}, 
{4, 1, 0}, {4, 5, 0}, {5, 1, 0}, {5, 5, 0}, {5, 6, 0}, {6, 4, 0}, {6, 5, 0}, {7, 0, 0}, {0, 5, 2}, {0, 6, 0}, 
{1, 4, 0}, {1, 5, 0}, {2, 2, 0}, {2, 6, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0}, {3, 6, 0}, {4, 3, 0}, {5, 2, 0},   // +36.067s 36064
{5, 3, 0}, {7, 5, 0}, {0, 1, 2}, {0, 2, 0}, {1, 1, 0}, {1, 2, 0}, {2, 1, 0}, {2, 5, 0}, {4, 6, 0}, {4, 7, 0}, 
{5, 7, 0}, {6, 6, 0}, {7, 1, 0}, {7, 2, 0}, {0, 4, 2}, {0, 7, 0}, {1, 6, 0}, {3, 3, 0}, {3, 7, 0}, {5, 1, 0}, 
{5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {6, 5, 0}, {7, 6, 0}, {0, 3, 2}, {0, 6, 0}, {1, 3, 0}, {1, 5, 0}, {2, 2, 0}, 
{2, 3, 0}, {2, 4, 0}, {2, 6, 0}, {3, 2, 0}, {3, 6, 0}, {4, 4, 0}, {5, 5, 0}, {7, 3, 0}, {7, 5, 0}, {0, 2, 3}, 
{1, 2, 0}, {4, 7, 0}, {5, 0, 0}, {5, 2, 0}, {5, 7, 0}, {6, 6, 0}, {6, 7, 0}, {7, 2, 0}, {0, 4, 2}, {0, 5, 0},   // +36.233s 36240
{0, 7, 0}, {1, 4, 0}, {1, 6, 0}, {2, 0, 0}, {2, 5, 0}, {3, 0, 0}, {3, 3, 0}, {3, 4, 0}, {3, 7, 0}, {4, 2, 0}, 
{5, 4, 0}, {7, 6, 0}, {0, 0, 2}, {0, 3, 0}, {1, 0, 0}, {1, 3, 0}, {1, 7, 0}, {2, 3, 0}, {2, 4, 0}, {4, 4, 0}, 
{4, 5, 0}, {5, 5, 0}, {5, 6, 0}, {6, 0, 0}, {6, 7, 0}, {7, 0, 0}, {7, 3, 0}, {0, 6, 2}, {1, 5, 0}, {3, 1, 0}, 
{3, 5, 0}, {4, 2, 0}, {4, 3, 0}, {5, 0, 0}, {5, 2, 0}, {5, 3, 0}, {0, 1, 2}, {0, 5, 0}, {1, 1, 0}, {1, 4, 0}, 
{2, 0, 0}, {2, 1, 0}, {2, 5, 0}, {2, 6, 0}, {3, 0, 0}, {3, 4, 0}, {4, 6, 0}, {5, 7, 0}, {6, 1, 0}, {7, 1, 0},   // +36.333s 36336
{0, 0, 2}, {1, 0, 0}, {1, 7, 0}, {4, 5, 0}, {5, 4, 0}, {5, 6, 0}, {6, 0, 0}, {7, 0, 0}, {7, 4, 0}, {7, 5, 0}, 
{0, 4, 2}, {0, 6, 0}, {1, 6, 0}, {2, 2, 0}, {2, 4, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0}, {3, 6, 0}, {4, 3, 0}, 
{5, 3, 0}, {7, 2, 0}, {0, 1, 2}, {0, 2, 0}, {1, 1, 0}, {1, 2, 0}, {1, 5, 0}, {2, 1, 0}, {2, 6, 0}, {2, 7, 0}, 
{4, 6, 0}, {4, 7, 0}, {5, 5, 0}, {5, 7, 0}, {6, 1, 0}, {6, 2, 0}, {7, 1, 0}, {0, 5, 2}, {1, 4, 0}, {3, 3, 0}, 
{3, 7, 0}, {5, 2, 0}, {5, 4, 0}, {7, 4, 0}, {7, 5, 0}, {7, 6, 0}, {0, 3, 2}, {0, 4, 0}, {1, 3, 0}, {1, 6, 0},   // +36.500s 36496
{2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {2, 5, 0}, {3, 2, 0}, {3, 6, 0}, {4, 4, 0}, {5, 6, 0}, {6, 3, 0}, {7, 2, 0}, 
{7, 3, 0}, {0, 2, 2}, {0, 6, 0}, {1, 2, 0}, {2, 7, 0}, {4, 1, 0}, {4, 2, 0}, {4, 7, 0}, {5, 3, 0}, {5, 5, 0}, 
{6, 2, 0}, {0, 0, 2}, {0, 5, 0}, {1, 4, 0}, {1, 5, 0}, {2, 0, 0}, {2, 6, 0}, {3, 0, 0}, {3, 3, 0}, {3, 4, 0}, 
{3, 7, 0}, {5, 2, 0}, {7, 0, 0}, {7, 6, 0}, {0, 3, 3}, {1, 0, 0}, {1, 3, 0}, {2, 3, 0}, {2, 5, 0}, {4, 4, 0}, 
{4, 5, 0}, {5, 1, 0}, {5, 6, 0}, {5, 7, 0}, {6, 3, 0}, {6, 4, 0}, {7, 3, 0}, {0, 4, 2}, {0, 6, 0}, {1, 5, 0},   // +36.633s 36640
{1, 6, 0}, {3, 1, 0}, {3, 5, 0}, {4, 1, 0}, {4, 2, 0}, {4, 3, 0}, {5, 3, 0}, {5, 4, 0}, {0, 0, 2}, {0, 1, 0}, 
{1, 0, 0}, {1, 1, 0}, {2, 0, 0}, {2, 1, 0}, {2, 4, 0}, {2, 6, 0}, {3, 0, 0}, {3, 4, 0}, {4, 6, 0}, {5, 5, 0}, 
{6, 5, 0}, {7, 0, 0}, {7, 1, 0}, {0, 5, 2}, {0, 7, 0}, {1, 4, 0}, {4, 5, 0}, {5, 1, 0}, {5, 2, 0}, {5, 7, 0}, 
{6, 4, 0}, {7, 5, 0}, {0, 2, 2}, {0, 4, 0}, {1, 2, 0}, {1, 6, 0}, {2, 2, 0}, {2, 5, 0}, {3, 1, 0}, {3, 2, 0}, 
{3, 5, 0}, {3, 6, 0}, {4, 3, 0}, {5, 4, 0}, {7, 2, 0}, {0, 1, 2}, {1, 1, 0}, {2, 1, 0}, {2, 4, 0}, {4, 6, 0},   // +36.767s 36768
{4, 7, 0}, {5, 0, 0}, {5, 5, 0}, {5, 6, 0}, {6, 5, 0}, {6, 6, 0}, {7, 1, 0}, {0, 5, 2}, {0, 6, 0}, {0, 7, 0}, 
{1, 4, 0}, {1, 5, 0}, {2, 3, 0}, {2, 6, 0}, {3, 3, 0}, {3, 7, 0}, {5, 2, 0}, {5, 3, 0}, {7, 5, 0}, {7, 6, 0}, 
{0, 2, 2}, {0, 3, 0}, {1, 2, 0}, {1, 3, 0}, {1, 7, 0}, {2, 2, 0}, {2, 5, 0}, {3, 2, 0}, {3, 6, 0}, {4, 4, 0}, 
{5, 7, 0}, {6, 7, 0}, {7, 2, 0}, {7, 3, 0}, {0, 4, 2}, {1, 6, 0}, {3, 0, 0}, {3, 4, 0}, {4, 2, 0}, {4, 7, 0}, 
{5, 0, 0}, {5, 4, 0}, {5, 6, 0}, {6, 6, 0}, {0, 0, 2}, {0, 6, 0}, {1, 0, 0}, {1, 5, 0}, {2, 0, 0}, {2, 3, 0},   // +36.900s 36896
{2, 4, 0}, {2, 6, 0}, {3, 3, 0}, {3, 7, 0}, {5, 3, 0}, {7, 0, 0}, {7, 6, 0}, {0, 3, 2}, {1, 3, 0}, {1, 7, 0}, 
{4, 4, 0}, {4, 5, 0}, {5, 5, 0}, {5, 7, 0}, {6, 0, 0}, {6, 7, 0}, {7, 3, 0}, {7, 4, 0}, {0, 4, 2}, {0, 5, 0}, 
{1, 4, 0}, {1, 6, 0}, {2, 1, 0}, {2, 5, 0}, {3, 0, 0}, {3, 1, 0}, {3, 4, 0}, {3, 5, 0}, {4, 2, 0}, {4, 3, 0}, 
{5, 2, 0}, {5, 4, 0}, {0, 0, 3}, {0, 1, 0}, {1, 0, 0}, {1, 1, 0}, {2, 0, 0}, {2, 4, 0}, {2, 7, 0}, {4, 6, 0}, 
{5, 6, 0}, {6, 1, 0}, {7, 0, 0}, {7, 1, 0}, {0, 6, 2}, {1, 5, 0}, {3, 2, 0}, {3, 6, 0}, {4, 3, 0}, {4, 5, 0},   // +37.033s 37040
{5, 3, 0}, {5, 5, 0}, {6, 0, 0}, {7, 4, 0}, {7, 5, 0}, {0, 2, 2}, {0, 5, 0}, {1, 2, 0}, {1, 4, 0}, {2, 1, 0}, 
{2, 2, 0}, {2, 5, 0}, {2, 6, 0}, {3, 1, 0}, {3, 5, 0}, {4, 7, 0}, {5, 2, 0}, {7, 2, 0}, {0, 1, 2}, {1, 1, 0}, 
{2, 7, 0}, {4, 1, 0}, {4, 6, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0}, {6, 1, 0}, {6, 2, 0}, {7, 1, 0}, {0, 4, 2}, 
{0, 6, 0}, {1, 5, 0}, {1, 6, 0}, {2, 3, 0}, {2, 4, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {3, 7, 0}, {5, 3, 0}, 
{7, 5, 0}, {7, 6, 0}, {0, 2, 2}, {0, 3, 0}, {1, 2, 0}, {1, 3, 0}, {2, 2, 0}, {2, 6, 0}, {4, 4, 0}, {4, 7, 0},   // +37.167s 37168
{5, 1, 0}, {5, 5, 0}, {5, 7, 0}, {6, 3, 0}, {7, 2, 0}, {7, 3, 0}, {0, 5, 2}, {1, 4, 0}, {3, 0, 0}, {3, 4, 0}, 
{4, 1, 0}, {4, 2, 0}, {5, 2, 0}, {5, 4, 0}, {6, 2, 0}, {0, 0, 2}, {0, 4, 0}, {1, 0, 0}, {1, 6, 0}, {2, 0, 0}, 
{2, 3, 0}, {2, 4, 0}, {2, 5, 0}, {3, 3, 0}, {3, 7, 0}, {4, 5, 0}, {5, 6, 0}, {6, 4, 0}, {7, 0, 0}, {7, 6, 0}, 
{0, 3, 2}, {0, 7, 0}, {1, 3, 0}, {4, 3, 0}, {4, 4, 0}, {5, 1, 0}, {5, 3, 0}, {5, 5, 0}, {6, 3, 0}, {7, 3, 0}, 
{0, 5, 2}, {0, 6, 0}, {1, 4, 0}, {1, 5, 0}, {2, 1, 0}, {2, 6, 0}, {3, 0, 0}, {3, 1, 0}, {3, 4, 0}, {3, 5, 0},   // +37.300s 37296
{4, 2, 0}, {5, 2, 0}, {0, 0, 2}, {0, 1, 0}, {1, 0, 0}, {1, 1, 0}, {2, 0, 0}, {2, 5, 0}, {4, 5, 0}, {4, 6, 0}, 
{5, 0, 0}, {5, 6, 0}, {5, 7, 0}, {6, 4, 0}, {6, 5, 0}, {7, 0, 0}, {7, 1, 0}, {0, 4, 2}, {0, 7, 0}, {1, 6, 0}, 
{3, 2, 0}, {3, 6, 0}, {4, 3, 0}, {5, 3, 0}, {5, 4, 0}, {7, 5, 0}, {0, 2, 3}, {0, 6, 0}, {1, 2, 0}, {1, 5, 0}, 
{2, 1, 0}, {2, 2, 0}, {2, 4, 0}, {2, 6, 0}, {3, 1, 0}, {3, 5, 0}, {4, 7, 0}, {5, 5, 0}, {6, 6, 0}, {7, 1, 0}, 
{7, 2, 0}, {0, 1, 2}, {1, 1, 0}, {1, 7, 0}, {4, 6, 0}, {5, 0, 0}, {5, 2, 0}, {5, 7, 0}, {6, 5, 0}, {7, 6, 0},   // +37.433s 37440
{0, 4, 2}, {0, 5, 0}, {1, 4, 0}, {1, 6, 0}, {2, 3, 0}, {2, 5, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {3, 7, 0}, 
{5, 4, 0}, {7, 3, 0}, {7, 5, 0}, {0, 2, 2}, {0, 3, 0}, {1, 2, 0}, {1, 3, 0}, {2, 2, 0}, {2, 4, 0}, {4, 4, 0}, 
{4, 7, 0}, {5, 5, 0}, {5, 6, 0}, {6, 6, 0}, {6, 7, 0}, {7, 2, 0}, {7, 4, 0}, {0, 5, 2}, {0, 6, 0}, {1, 5, 0}, 
{1, 7, 0}, {3, 0, 0}, {3, 4, 0}, {4, 2, 0}, {5, 2, 0}, {5, 3, 0}, {7, 6, 0}, {0, 0, 2}, {0, 3, 0}, {1, 0, 0}, 
{1, 4, 0}, {2, 0, 0}, {2, 3, 0}, {2, 5, 0}, {2, 6, 0}, {3, 3, 0}, {3, 7, 0}, {4, 5, 0}, {5, 7, 0}, {6, 0, 0},   // +37.567s 37568
{7, 0, 0}, {7, 3, 0}, {0, 4, 2}, {1, 3, 0}, {1, 6, 0}, {2, 7, 0}, {4, 3, 0}, {4, 4, 0}, {5, 4, 0}, {5, 6, 0}, 
{6, 7, 0}, {7, 4, 0}, {0, 1, 2}, {0, 6, 0}, {1, 1, 0}, {1, 5, 0}, {2, 1, 0}, {2, 4, 0}, {3, 0, 0}, {3, 1, 0}, 
{3, 4, 0}, {3, 5, 0}, {4, 2, 0}, {5, 3, 0}, {7, 1, 0}, {0, 0, 2}, {1, 0, 0}, {2, 0, 0}, {2, 6, 0}, {4, 1, 0}, 
{4, 5, 0}, {4, 6, 0}, {5, 5, 0}, {5, 7, 0}, {6, 0, 0}, {6, 1, 0}, {7, 0, 0}, {0, 4, 2}, {0, 5, 0}, {1, 4, 0}, 
{1, 6, 0}, {2, 7, 0}, {3, 2, 0}, {3, 6, 0}, {4, 3, 0}, {5, 2, 0}, {5, 4, 0}, {7, 5, 0}, {0, 1, 2}, {0, 2, 0},   // +37.733s 37728
{1, 1, 0}, {1, 2, 0}, {2, 1, 0}, {2, 2, 0}, {2, 4, 0}, {2, 5, 0}, {3, 1, 0}, {3, 5, 0}, {4, 7, 0}, {5, 6, 0}, 
{6, 2, 0}, {7, 1, 0}, {7, 2, 0}, {0, 6, 2}, {1, 5, 0}, {4, 1, 0}, {4, 6, 0}, {5, 1, 0}, {5, 3, 0}, {5, 5, 0}, 
{6, 1, 0}, {7, 6, 0}, {0, 3, 2}, {0, 5, 0}, {1, 3, 0}, {1, 4, 0}, {2, 2, 0}, {2, 3, 0}, {2, 5, 0}, {2, 6, 0}, 
{3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {3, 7, 0}, {5, 2, 0}, {7, 3, 0}, {7, 5, 0}, {0, 2, 3}, {0, 7, 0}, {1, 2, 0}, 
{4, 4, 0}, {4, 7, 0}, {5, 6, 0}, {5, 7, 0}, {6, 2, 0}, {6, 3, 0}, {7, 2, 0}, {0, 4, 2}, {0, 6, 0}, {1, 5, 0},   // +37.867s 37872
{1, 6, 0}, {2, 0, 0}, {2, 4, 0}, {3, 0, 0}, {3, 4, 0}, {4, 2, 0}, {5, 1, 0}, {5, 3, 0}, {5, 4, 0}, {7, 6, 0}, 
{0, 0, 2}, {0, 3, 0}, {1, 0, 0}, {1, 3, 0}, {2, 3, 0}, {2, 6, 0}, {3, 3, 0}, {3, 7, 0}, {4, 5, 0}, {5, 5, 0}, 
{6, 4, 0}, {7, 0, 0}, {7, 3, 0}, {0, 5, 2}, {0, 7, 0}, {1, 4, 0}, {3, 1, 0}, {3, 5, 0}, {4, 3, 0}, {4, 4, 0}, 
{5, 0, 0}, {5, 2, 0}, {5, 7, 0}, {6, 3, 0}, {0, 1, 2}, {0, 4, 0}, {1, 1, 0}, {1, 6, 0}, {2, 0, 0}, {2, 1, 0}, 
{2, 4, 0}, {2, 5, 0}, {3, 0, 0}, {3, 4, 0}, {4, 2, 0}, {5, 4, 0}, {7, 1, 0}, {0, 0, 2}, {1, 0, 0}, {1, 7, 0},   // +38.000s 38000
{4, 5, 0}, {4, 6, 0}, {5, 5, 0}, {5, 6, 0}, {6, 4, 0}, {6, 5, 0}, {7, 0, 0}, {0, 5, 2}, {0, 6, 0}, {1, 4, 0}, 
{1, 5, 0}, {2, 2, 0}, {2, 6, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0}, {3, 6, 0}, {4, 3, 0}, {5, 0, 0}, {5, 2, 0}, 
{5, 3, 0}, {7, 5, 0}, {0, 1, 2}, {0, 2, 0}, {1, 1, 0}, {1, 2, 0}, {2, 1, 0}, {2, 5, 0}, {7, 1, 0}, {7, 2, 0}, 
{0, 4, 2}, {3, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 4, 0}, {5, 7, 0}, {0, 3, 2}, {0, 6, 0}, {1, 3, 0}, {1, 5, 0}, 
{1, 6, 0}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {2, 6, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {5, 3, 0}, {5, 6, 0},   // +38.133s 38128
{7, 3, 0}, {7, 5, 0}, {7, 6, 0}, {0, 2, 2}, {1, 2, 0}, {1, 7, 0}, {4, 4, 0}, {5, 2, 0}, {5, 5, 0}, {6, 5, 0}, 
{6, 6, 0}, {7, 2, 0}, {7, 4, 0}, {0, 4, 2}, {0, 5, 0}, {1, 4, 0}, {2, 0, 0}, {2, 5, 0}, {3, 0, 0}, {3, 4, 0}, 
{3, 7, 0}, {4, 2, 0}, {4, 7, 0}, {5, 4, 0}, {5, 7, 0}, {0, 0, 3}, {0, 3, 0}, {1, 0, 0}, {1, 3, 0}, {1, 6, 0}, 
{2, 3, 0}, {2, 4, 0}, {2, 7, 0}, {3, 3, 0}, {6, 7, 0}, {7, 0, 0}, {7, 3, 0}, {7, 6, 0}, {0, 6, 2}, {3, 5, 0}, 
{4, 3, 0}, {4, 4, 0}, {4, 5, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {6, 6, 0}, {7, 4, 0}, {0, 1, 2},   // +38.300s 38304
{0, 5, 0}, {1, 1, 0}, {1, 4, 0}, {1, 5, 0}, {2, 1, 0}, {2, 5, 0}, {2, 6, 0}, {3, 0, 0}, {3, 1, 0}, {3, 4, 0}, 
{4, 2, 0}, {6, 0, 0}, {7, 1, 0}, {0, 0, 2}, {1, 0, 0}, {2, 0, 0}, {2, 7, 0}, {4, 1, 0}, {4, 6, 0}, {5, 4, 0}, 
{5, 7, 0}, {6, 7, 0}, {7, 0, 0}, {0, 4, 2}, {0, 6, 0}, {1, 6, 0}, {2, 2, 0}, {2, 4, 0}, {3, 2, 0}, {3, 5, 0}, 
{3, 6, 0}, {4, 3, 0}, {4, 5, 0}, {5, 3, 0}, {5, 6, 0}, {7, 2, 0}, {7, 5, 0}, {0, 2, 2}, {1, 1, 0}, {1, 2, 0}, 
{1, 5, 0}, {2, 1, 0}, {2, 6, 0}, {3, 1, 0}, {5, 1, 0}, {6, 0, 0}, {6, 1, 0}, {7, 1, 0}, {0, 1, 2}, {0, 5, 0},   // +38.433s 38432
{3, 7, 0}, {4, 1, 0}, {4, 6, 0}, {4, 7, 0}, {5, 2, 0}, {5, 4, 0}, {5, 5, 0}, {5, 7, 0}, {7, 6, 0}, {0, 4, 2}, 
{1, 3, 0}, {1, 4, 0}, {1, 6, 0}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {2, 5, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, 
{6, 2, 0}, {7, 2, 0}, {7, 3, 0}, {7, 5, 0}, {0, 2, 2}, {0, 3, 0}, {0, 6, 0}, {0, 7, 0}, {1, 2, 0}, {4, 4, 0}, 
{5, 1, 0}, {5, 3, 0}, {5, 6, 0}, {6, 1, 0}, {0, 5, 2}, {1, 5, 0}, {2, 6, 0}, {3, 0, 0}, {3, 4, 0}, {3, 7, 0}, 
{4, 2, 0}, {4, 7, 0}, {5, 2, 0}, {5, 5, 0}, {7, 0, 0}, {7, 6, 0}, {0, 0, 2}, {0, 3, 0}, {0, 7, 0}, {1, 0, 0},   // +38.567s 38560
{1, 3, 0}, {1, 4, 0}, {2, 0, 0}, {2, 3, 0}, {2, 5, 0}, {3, 3, 0}, {5, 0, 0}, {6, 2, 0}, {6, 3, 0}, {7, 3, 0}, 
{0, 4, 2}, {0, 6, 0}, {1, 6, 0}, {3, 5, 0}, {4, 3, 0}, {4, 4, 0}, {4, 5, 0}, {5, 3, 0}, {5, 4, 0}, {5, 6, 0}, 
{5, 7, 0}, {0, 1, 3}, {1, 0, 0}, {1, 1, 0}, {1, 5, 0}, {1, 7, 0}, {2, 1, 0}, {2, 4, 0}, {2, 6, 0}, {3, 0, 0}, 
{3, 1, 0}, {3, 4, 0}, {4, 2, 0}, {6, 4, 0}, {7, 0, 0}, {7, 1, 0}, {0, 0, 2}, {0, 5, 0}, {2, 0, 0}, {4, 6, 0}, 
{5, 0, 0}, {5, 2, 0}, {5, 5, 0}, {6, 3, 0}, {0, 4, 2}, {1, 2, 0}, {1, 4, 0}, {1, 6, 0}, {2, 2, 0}, {2, 5, 0},   // +38.700s 38704
{3, 2, 0}, {3, 5, 0}, {3, 6, 0}, {4, 3, 0}, {4, 5, 0}, {5, 4, 0}, {5, 7, 0}, {7, 2, 0}, {7, 5, 0}, {0, 1, 2}, 
{0, 2, 0}, {1, 1, 0}, {1, 7, 0}, {2, 1, 0}, {2, 4, 0}, {3, 1, 0}, {6, 4, 0}, {6, 5, 0}, {7, 1, 0}, {7, 4, 0}, 
{0, 2, 2}, {0, 5, 0}, {1, 2, 0}, {1, 4, 0}, {1, 6, 0}, {1, 7, 0}, {2, 2, 0}, {2, 5, 0}, {3, 2, 0}, {3, 6, 0}, 
{4, 6, 0}, {5, 0, 0}, {5, 2, 0}, {5, 5, 0}, {6, 5, 0}, {7, 0, 0}, {7, 2, 0}, {7, 4, 0}, {7, 5, 0}, {7, 1, 2}, 
{7, 2, 0}, {1, 5, 2}, {2, 0, 0}, {7, 3, 0}, {1, 0, 2}, {1, 4, 0}, {2, 1, 0}, {7, 4, 0}, {1, 1, 2}, {1, 2, 0},   // +38.900s 38896
{2, 2, 0}, {2, 3, 0}, {3, 0, 0}, {5, 5, 0}, {1, 3, 2}, {2, 4, 0}, {2, 5, 0}, {2, 7, 0}, {3, 1, 0}, {3, 2, 0}, 
{2, 6, 2}, {3, 3, 0}, {4, 2, 0}, {7, 6, 0}, {4, 0, 2}, {4, 3, 0}, {5, 6, 0}, {7, 5, 0}, {3, 4, 3}, {4, 1, 0}, 
{5, 7, 0}, {3, 5, 2}, {3, 6, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {5, 1, 0}, {0, 5, 2}, {0, 6, 0}, 
{3, 7, 0}, {0, 4, 2}, {5, 2, 0}, {5, 3, 0}, {5, 4, 2}, {0, 0, 2}, {0, 1, 0}, {0, 2, 2}, {0, 3, 0}, {0, 7, 2}, 
{6, 0, 2}, {6, 1, 0}, {6, 2, 2}, {6, 3, 0}, {6, 4, 2}, {6, 5, 0}, {6, 6, 3}, {6, 7, 0}, {6, 7, 10}, {6, 5, 2},   // +39.600s 39600
{6, 6, 0}, {6, 3, 2}, {6, 4, 0}, {6, 1, 2}, {6, 2, 0}, {0, 7, 2}, {6, 0, 0}, {0, 2, 2}, {0, 3, 0}, {0, 0, 2}, 
{0, 1, 0}, {0, 4, 5}, {5, 3, 0}, {5, 4, 0}, {0, 5, 2}, {0, 6, 0}, {5, 2, 0}, {3, 5, 2}, {3, 6, 0}, {3, 7, 0}, 
{4, 4, 0}, {4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {3, 4, 2}, {4, 1, 0}, {5, 1, 0}, {5, 7, 2}, {2, 6, 2}, {3, 3, 0}, 
{4, 0, 0}, {4, 3, 0}, {5, 5, 0}, {5, 6, 0}, {7, 5, 0}, {1, 3, 2}, {2, 4, 0}, {2, 5, 0}, {2, 7, 0}, {3, 1, 0}, 
{3, 2, 0}, {4, 2, 0}, {7, 6, 0}, {1, 1, 2}, {1, 2, 0}, {2, 2, 0}, {2, 3, 0}, {3, 0, 0}, {1, 0, 2}, {1, 4, 0},   // +40.100s 40096
{2, 1, 0}, {1, 5, 2}, {2, 0, 0}, {7, 4, 0}, {7, 3, 2}, {1, 6, 2}, {1, 7, 0}, {7, 1, 0}, {7, 2, 0}, {5, 0, 3}, 
{7, 0, 0}, {6, 6, 6}, {6, 7, 0}, {6, 4, 2}, {6, 5, 0}, {6, 2, 2}, {6, 3, 0}, {6, 0, 2}, {6, 1, 0}, {0, 7, 2}, 
{0, 1, 2}, {0, 2, 0}, {0, 3, 0}, {0, 0, 2}, {5, 4, 2}, {0, 4, 2}, {0, 5, 0}, {0, 6, 0}, {5, 2, 0}, {5, 3, 0}, 
{3, 7, 3}, {3, 5, 2}, {3, 6, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {5, 1, 0}, {3, 4, 2}, {4, 1, 0}, 
{2, 6, 2}, {5, 6, 0}, {5, 7, 0}, {2, 5, 2}, {2, 7, 0}, {3, 3, 0}, {4, 0, 0}, {4, 2, 0}, {4, 3, 0}, {5, 5, 0},   // +40.767s 40768
{7, 5, 0}, {7, 6, 0}, {1, 1, 2}, {1, 2, 0}, {1, 3, 0}, {2, 3, 0}, {2, 4, 0}, {3, 1, 0}, {3, 2, 0}, {2, 2, 2}, 
{3, 0, 0}, {1, 0, 2}, {1, 4, 2}, {1, 5, 0}, {2, 0, 0}, {2, 1, 0}, {7, 4, 2}, {7, 2, 2}, {7, 3, 0}, {1, 6, 3}, 
{1, 7, 0}, {7, 0, 0}, {7, 1, 0}, {5, 0, 2}, {1, 6, 6}, {1, 7, 0}, {5, 0, 0}, {7, 0, 0}, {7, 1, 2}, {7, 2, 0}, 
{1, 5, 2}, {2, 0, 0}, {7, 3, 0}, {1, 0, 2}, {1, 4, 0}, {2, 1, 0}, {5, 5, 0}, {7, 4, 0}, {1, 1, 2}, {1, 2, 0}, 
{2, 2, 0}, {2, 3, 0}, {3, 0, 0}, {1, 3, 2}, {2, 4, 0}, {2, 5, 0}, {2, 7, 0}, {3, 1, 0}, {3, 2, 0}, {2, 6, 2},   // +41.333s 41328
{3, 3, 0}, {4, 0, 0}, {4, 2, 0}, {4, 3, 0}, {7, 6, 0}, {5, 6, 2}, {5, 7, 0}, {7, 5, 0}, {3, 4, 2}, {4, 1, 0}, 
{3, 5, 3}, {3, 6, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {5, 1, 0}, {3, 7, 2}, {0, 4, 2}, {0, 5, 0}, 
{0, 6, 0}, {5, 2, 0}, {5, 3, 0}, {5, 4, 2}, {0, 0, 2}, {0, 1, 0}, {0, 2, 0}, {0, 3, 2}, {0, 7, 2}, {6, 0, 2}, 
{6, 1, 0}, {6, 2, 2}, {6, 3, 0}, {6, 4, 2}, {6, 5, 0}, {6, 6, 2}, {6, 7, 0}, {1, 7, 28}, {3, 2, 0}, {3, 3, 0}, 
{1, 4, 2}, {3, 0, 0}, {3, 1, 0}, {6, 0, 0}, {6, 1, 0}, {6, 2, 0}, {1, 5, 2}, {1, 6, 0}, {1, 0, 2}, {1, 1, 2},   // +42.333s 42336
{1, 2, 0}, {1, 3, 2}, {4, 2, 0}, {4, 3, 0}, {5, 1, 0}, {5, 2, 0}, {6, 3, 0}, {7, 0, 0}, {5, 3, 2}, {5, 4, 0}, 
{3, 4, 2}, {3, 5, 2}, {3, 6, 0}, {2, 7, 2}, {3, 7, 0}, {6, 4, 0}, {7, 1, 0}, {4, 4, 2}, {4, 5, 0}, {4, 6, 2}, 
{4, 7, 0}, {0, 0, 3}, {0, 1, 0}, {0, 4, 0}, {0, 7, 0}, {4, 0, 0}, {4, 1, 0}, {6, 5, 0}, {6, 6, 0}, {7, 2, 0}, 
{7, 3, 0}, {7, 5, 0}, {7, 6, 0}, {0, 2, 2}, {0, 5, 0}, {0, 6, 0}, {2, 0, 0}, {2, 1, 0}, {6, 7, 0}, {2, 2, 2}, 
{5, 5, 0}, {5, 6, 0}, {5, 7, 2}, {2, 3, 2}, {2, 4, 0}, {2, 5, 0}, {7, 4, 0}, {0, 3, 2}, {2, 6, 0}, {0, 3, 6},   // +42.867s 42864
{2, 6, 0}, {2, 4, 2}, {2, 5, 0}, {0, 2, 2}, {2, 3, 0}, {7, 4, 0}, {5, 7, 2}, {0, 5, 3}, {0, 6, 0}, {2, 1, 0}, 
{2, 2, 0}, {5, 5, 0}, {5, 6, 0}, {0, 0, 2}, {0, 1, 0}, {0, 4, 0}, {0, 7, 0}, {2, 0, 0}, {4, 0, 0}, {6, 7, 0}, 
{4, 1, 2}, {4, 7, 0}, {6, 5, 0}, {6, 6, 0}, {7, 2, 0}, {7, 3, 0}, {7, 5, 0}, {7, 6, 0}, {4, 5, 2}, {4, 6, 0}, 
{2, 7, 2}, {4, 4, 0}, {3, 6, 2}, {3, 7, 0}, {6, 4, 0}, {7, 1, 0}, {3, 4, 2}, {3, 5, 0}, {5, 4, 2}, {4, 2, 2}, 
{4, 3, 0}, {5, 1, 0}, {5, 2, 0}, {5, 3, 0}, {6, 3, 0}, {1, 2, 2}, {1, 3, 0}, {7, 0, 0}, {1, 0, 2}, {1, 1, 0},   // +43.333s 43328
{1, 6, 2}, {1, 4, 2}, {1, 5, 0}, {3, 0, 0}, {6, 2, 0}, {3, 1, 3}, {3, 2, 0}, {6, 0, 0}, {6, 1, 0}, {1, 7, 2}, 
{3, 3, 0}, {0, 3, 4}, {2, 6, 0}, {0, 2, 2}, {2, 4, 0}, {2, 5, 0}, {2, 3, 2}, {7, 4, 0}, {5, 7, 2}, {5, 6, 2}, 
{0, 1, 2}, {0, 4, 0}, {0, 5, 0}, {0, 6, 0}, {0, 7, 0}, {2, 1, 0}, {2, 2, 0}, {5, 5, 0}, {0, 0, 2}, {2, 0, 0}, 
{4, 0, 0}, {6, 6, 0}, {6, 7, 0}, {7, 3, 0}, {7, 5, 0}, {7, 6, 0}, {4, 1, 2}, {4, 7, 0}, {6, 5, 0}, {7, 2, 0}, 
{4, 5, 3}, {4, 6, 0}, {2, 7, 4}, {3, 7, 0}, {4, 4, 0}, {6, 4, 0}, {7, 1, 0}, {3, 5, 2}, {3, 6, 0}, {3, 4, 2},   // +43.933s 43936
{5, 4, 2}, {1, 3, 2}, {4, 2, 0}, {4, 3, 0}, {5, 1, 0}, {5, 2, 0}, {5, 3, 0}, {6, 3, 0}, {7, 0, 0}, {1, 1, 2}, 
{1, 2, 0}, {1, 0, 2}, {1, 5, 2}, {1, 6, 0}, {1, 4, 2}, {3, 0, 0}, {3, 1, 0}, {6, 2, 0}, {1, 7, 2}, {3, 2, 0}, 
{3, 3, 0}, {6, 0, 0}, {6, 1, 0}, {1, 7, 9}, {3, 2, 0}, {3, 3, 0}, {6, 0, 0}, {1, 4, 2}, {3, 0, 0}, {3, 1, 0}, 
{6, 1, 0}, {6, 2, 0}, {1, 5, 2}, {1, 6, 0}, {1, 0, 2}, {1, 1, 2}, {1, 2, 0}, {1, 3, 2}, {4, 2, 0}, {4, 3, 0}, 
{5, 1, 0}, {5, 2, 0}, {6, 3, 0}, {7, 0, 0}, {5, 3, 2}, {5, 4, 0}, {3, 4, 2}, {3, 5, 2}, {3, 6, 0}, {2, 7, 3},   // +44.600s 44608
{3, 7, 0}, {6, 4, 0}, {7, 1, 0}, {4, 4, 2}, {4, 5, 0}, {4, 6, 2}, {4, 7, 0}, {0, 0, 2}, {0, 1, 0}, {0, 4, 0}, 
{0, 7, 0}, {4, 0, 0}, {4, 1, 0}, {6, 5, 0}, {6, 6, 0}, {7, 2, 0}, {7, 3, 0}, {7, 5, 0}, {7, 6, 0}, {0, 2, 2}, 
{0, 5, 0}, {0, 6, 0}, {2, 0, 0}, {2, 1, 0}, {6, 7, 0}, {2, 2, 2}, {5, 5, 0}, {5, 6, 0}, {2, 3, 2}, {5, 7, 0}, 
{2, 4, 2}, {2, 5, 0}, {7, 4, 0}, {0, 3, 2}, {2, 6, 0}, {2, 7, 4}, {2, 7, 5}, {2, 7, 4}, {2, 7, 6}, {2, 7, 4}, 
{2, 7, 6}, {2, 7, 5}, {2, 7, 6}, {2, 7, 4}, {2, 7, 6}, {1, 1, 4}, {1, 2, 0}, {1, 3, 0}, {2, 5, 0}, {4, 2, 0},   // +45.733s 45728
{4, 3, 0}, {7, 5, 0}, {7, 6, 0}, {1, 0, 2}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {3, 0, 0}, {3, 2, 0}, {3, 4, 0}, 
{4, 1, 0}, {4, 5, 0}, {5, 5, 0}, {5, 6, 0}, {5, 7, 0}, {0, 4, 2}, {0, 5, 0}, {0, 6, 0}, {1, 4, 0}, {1, 5, 0}, 
{1, 6, 0}, {2, 0, 0}, {2, 1, 0}, {3, 1, 0}, {3, 5, 0}, {3, 6, 0}, {3, 7, 0}, {4, 4, 0}, {4, 6, 0}, {4, 7, 0}, 
{5, 1, 0}, {5, 4, 0}, {7, 4, 0}, {0, 0, 3}, {5, 3, 0}, {7, 0, 0}, {7, 1, 0}, {7, 2, 0}, {0, 1, 2}, {0, 2, 0}, 
{0, 3, 0}, {7, 3, 0}, {0, 7, 2}, {2, 6, 0}, {2, 7, 0}, {3, 3, 0}, {5, 0, 0}, {5, 2, 0}, {6, 0, 0}, {6, 1, 2},   // +45.933s 45936
{6, 2, 0}, {6, 3, 0}, {6, 4, 0}, {6, 5, 2}, {6, 6, 0}, {6, 7, 0}, {6, 7, 21}, {6, 3, 2}, {6, 4, 0}, {6, 5, 0}, 
{6, 6, 0}, {0, 7, 2}, {2, 6, 0}, {2, 7, 0}, {3, 3, 0}, {5, 0, 0}, {6, 0, 0}, {6, 1, 0}, {6, 2, 0}, {0, 1, 2}, 
{0, 2, 0}, {0, 3, 0}, {5, 2, 0}, {0, 0, 2}, {7, 1, 0}, {7, 2, 0}, {7, 3, 0}, {0, 4, 2}, {0, 5, 0}, {1, 4, 0}, 
{1, 5, 0}, {1, 6, 0}, {2, 0, 0}, {2, 1, 0}, {4, 4, 0}, {5, 3, 0}, {5, 4, 0}, {7, 0, 0}, {7, 4, 0}, {0, 6, 2}, 
{1, 0, 0}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {3, 1, 0}, {3, 5, 0}, {3, 6, 0}, {3, 7, 0}, {4, 5, 0}, {4, 6, 0},   // +46.500s 46496
{4, 7, 0}, {5, 1, 0}, {5, 7, 0}, {1, 1, 2}, {1, 2, 0}, {1, 3, 0}, {2, 5, 0}, {3, 0, 0}, {3, 2, 0}, {3, 4, 0}, 
{4, 1, 0}, {4, 2, 0}, {4, 3, 0}, {5, 5, 0}, {5, 6, 0}, {7, 5, 0}, {7, 6, 0}, {1, 2, 19}, {1, 3, 0}, {1, 1, 2}, 
{2, 5, 0}, {3, 0, 0}, {3, 2, 0}, {3, 4, 0}, {4, 1, 0}, {4, 2, 0}, {4, 3, 0}, {4, 5, 0}, {5, 5, 0}, {5, 6, 0}, 
{7, 5, 0}, {7, 6, 0}, {1, 0, 2}, {1, 4, 0}, {1, 5, 0}, {2, 0, 0}, {2, 1, 0}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, 
{3, 1, 0}, {3, 5, 0}, {3, 6, 0}, {3, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 1, 0}, {5, 7, 0}, {0, 4, 2}, {0, 5, 0},   // +46.933s 46928
{0, 6, 0}, {1, 6, 0}, {4, 4, 0}, {5, 3, 0}, {5, 4, 0}, {7, 0, 0}, {7, 4, 0}, {0, 0, 2}, {0, 1, 0}, {0, 2, 0}, 
{7, 1, 0}, {7, 2, 0}, {7, 3, 0}, {0, 3, 3}, {0, 7, 0}, {2, 6, 0}, {2, 7, 0}, {3, 3, 0}, {5, 2, 0}, {5, 0, 2}, 
{6, 0, 0}, {6, 1, 0}, {6, 2, 0}, {6, 3, 2}, {6, 4, 0}, {6, 5, 0}, {6, 6, 0}, {6, 7, 2}, {6, 5, 21}, {6, 6, 0}, 
{6, 7, 0}, {6, 1, 2}, {6, 2, 0}, {6, 3, 0}, {6, 4, 0}, {0, 7, 2}, {2, 6, 0}, {2, 7, 0}, {3, 3, 0}, {5, 0, 0}, 
{5, 2, 0}, {6, 0, 0}, {0, 0, 2}, {0, 1, 0}, {0, 2, 0}, {0, 3, 0}, {7, 3, 0}, {1, 6, 2}, {5, 3, 0}, {7, 0, 0},   // +47.567s 47568
{7, 1, 0}, {7, 2, 0}, {0, 4, 2}, {0, 5, 0}, {0, 6, 0}, {1, 0, 0}, {1, 4, 0}, {1, 5, 0}, {2, 0, 0}, {2, 1, 0}, 
{2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {3, 1, 0}, {3, 5, 0}, {3, 6, 0}, {3, 7, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 0}, 
{4, 7, 0}, {5, 1, 0}, {5, 4, 0}, {7, 4, 0}, {1, 1, 2}, {1, 2, 0}, {1, 3, 0}, {2, 5, 0}, {3, 0, 0}, {3, 2, 0}, 
{3, 4, 0}, {4, 1, 0}, {5, 5, 0}, {5, 6, 0}, {5, 7, 0}, {4, 2, 2}, {4, 3, 0}, {7, 5, 0}, {7, 6, 0}, {1, 1, 19}, 
{1, 2, 0}, {1, 3, 0}, {2, 5, 0}, {3, 0, 0}, {3, 2, 0}, {3, 4, 0}, {4, 2, 0}, {4, 3, 0}, {7, 5, 0}, {7, 6, 0},   // +47.967s 47968
{1, 0, 2}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {3, 1, 0}, {3, 5, 0}, {3, 6, 0}, {3, 7, 0}, {4, 1, 0}, {4, 5, 0}, 
{4, 6, 0}, {4, 7, 0}, {5, 5, 0}, {5, 6, 0}, {5, 7, 0}, {0, 4, 2}, {0, 5, 0}, {0, 6, 0}, {1, 4, 0}, {1, 5, 0}, 
{1, 6, 0}, {2, 0, 0}, {2, 1, 0}, {4, 4, 0}, {5, 1, 0}, {5, 4, 0}, {7, 4, 0}, {0, 0, 2}, {5, 3, 0}, {7, 0, 0}, 
{7, 1, 0}, {7, 2, 0}, {0, 1, 2}, {0, 2, 0}, {0, 3, 0}, {5, 2, 0}, {7, 3, 0}, {0, 7, 2}, {2, 6, 0}, {2, 7, 0}, 
{3, 3, 0}, {5, 0, 0}, {6, 0, 0}, {6, 1, 2}, {6, 2, 0}, {6, 3, 0}, {6, 4, 0}, {6, 5, 3}, {6, 6, 0}, {6, 7, 0},   // +48.200s 48208
{6, 7, 20}, {6, 3, 2}, {6, 4, 0}, {6, 5, 0}, {6, 6, 0}, {0, 3, 3}, {0, 7, 0}, {2, 6, 0}, {2, 7, 0}, {3, 3, 0}, 
{5, 0, 0}, {6, 0, 0}, {6, 1, 0}, {6, 2, 0}, {0, 0, 2}, {0, 1, 0}, {0, 2, 0}, {5, 2, 0}, {1, 6, 2}, {7, 1, 0}, 
{7, 2, 0}, {7, 3, 0}, {0, 4, 2}, {0, 5, 0}, {0, 6, 0}, {1, 0, 0}, {1, 4, 0}, {1, 5, 0}, {2, 0, 0}, {2, 1, 0}, 
{4, 4, 0}, {5, 3, 0}, {5, 4, 0}, {7, 0, 0}, {7, 4, 0}, {1, 1, 2}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {3, 1, 0}, 
{3, 5, 0}, {3, 6, 0}, {3, 7, 0}, {4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {5, 1, 0}, {5, 7, 0}, {1, 2, 2}, {1, 3, 0},   // +48.767s 48768
{2, 5, 0}, {3, 0, 0}, {3, 2, 0}, {3, 4, 0}, {4, 1, 0}, {4, 2, 0}, {4, 3, 0}, {5, 5, 0}, {5, 6, 0}, {7, 5, 0}, 
{7, 6, 0}, {1, 7, 19}, {2, 0, 0}, {2, 1, 2}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {7, 2, 0}, {7, 3, 0}, {7, 4, 0}, 
{2, 5, 2}, {2, 6, 0}, {4, 0, 0}, {4, 1, 0}, {4, 7, 0}, {5, 5, 0}, {5, 6, 0}, {5, 7, 0}, {7, 5, 0}, {7, 6, 0}, 
{0, 4, 2}, {0, 5, 0}, {0, 6, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 0}, {0, 0, 2}, {0, 1, 0}, {0, 2, 0}, {0, 3, 0}, 
{0, 7, 2}, {6, 0, 0}, {6, 1, 0}, {6, 2, 0}, {6, 3, 2}, {6, 4, 0}, {6, 5, 0}, {6, 6, 0}, {5, 2, 2}, {5, 3, 0},   // +49.300s 49296
{6, 7, 0}, {3, 5, 2}, {3, 6, 0}, {3, 7, 0}, {4, 3, 0}, {5, 1, 0}, {5, 4, 0}, {1, 2, 2}, {1, 3, 0}, {2, 7, 0}, 
{3, 0, 0}, {3, 1, 0}, {3, 2, 0}, {3, 3, 0}, {3, 4, 0}, {4, 2, 0}, {1, 0, 2}, {1, 1, 0}, {1, 4, 0}, {1, 5, 0}, 
{7, 0, 0}, {7, 1, 0}, {1, 6, 3}, {5, 0, 0}, {1, 4, 20}, {1, 5, 0}, {1, 6, 0}, {5, 0, 0}, {7, 0, 2}, {7, 1, 0}, 
{1, 0, 5}, {1, 1, 0}, {1, 2, 0}, {1, 3, 0}, {2, 7, 2}, {3, 0, 0}, {3, 1, 0}, {3, 2, 0}, {3, 3, 0}, {3, 4, 0}, 
{3, 5, 0}, {3, 6, 0}, {4, 2, 0}, {3, 7, 2}, {4, 3, 0}, {5, 1, 0}, {5, 4, 0}, {5, 2, 2}, {5, 3, 0}, {6, 7, 0},   // +49.967s 49968
{6, 3, 2}, {6, 4, 0}, {6, 5, 0}, {6, 6, 0}, {0, 2, 2}, {0, 3, 0}, {0, 7, 0}, {6, 0, 0}, {6, 1, 0}, {6, 2, 0}, 
{0, 0, 2}, {0, 1, 0}, {0, 4, 0}, {0, 5, 0}, {0, 6, 0}, {4, 0, 2}, {4, 1, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 0}, 
{4, 7, 0}, {2, 3, 2}, {2, 4, 0}, {2, 5, 0}, {2, 6, 0}, {5, 5, 0}, {5, 6, 0}, {5, 7, 0}, {7, 5, 0}, {7, 6, 0}, 
{1, 7, 2}, {2, 0, 0}, {2, 1, 0}, {2, 2, 0}, {7, 4, 0}, {7, 2, 2}, {7, 3, 0}, {1, 4, 19}, {1, 5, 0}, {1, 6, 0}, 
{5, 0, 0}, {7, 0, 0}, {7, 1, 0}, {1, 0, 2}, {1, 1, 0}, {1, 2, 0}, {1, 3, 0}, {3, 0, 0}, {2, 7, 2}, {3, 1, 0},   // +50.567s 50560
{3, 2, 0}, {3, 3, 0}, {3, 4, 0}, {3, 5, 0}, {3, 6, 0}, {4, 2, 0}, {4, 3, 0}, {5, 1, 0}, {3, 7, 3}, {5, 2, 0}, 
{5, 3, 0}, {5, 4, 0}, {6, 5, 2}, {6, 6, 0}, {6, 7, 0}, {0, 7, 2}, {6, 1, 0}, {6, 2, 0}, {6, 3, 0}, {6, 4, 0}, 
{0, 0, 2}, {0, 1, 0}, {0, 2, 0}, {0, 3, 0}, {6, 0, 0}, {0, 4, 2}, {0, 5, 0}, {0, 6, 0}, {2, 5, 2}, {2, 6, 0}, 
{4, 0, 0}, {4, 1, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {5, 5, 0}, {2, 1, 2}, {2, 2, 0}, {2, 3, 0}, 
{2, 4, 0}, {5, 6, 0}, {5, 7, 0}, {7, 5, 0}, {7, 6, 0}, {1, 7, 2}, {2, 0, 0}, {7, 2, 0}, {7, 3, 0}, {7, 4, 0},   // +50.833s 50832
{1, 7, 21}, {2, 0, 0}, {2, 1, 2}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {7, 2, 0}, {7, 3, 0}, {7, 4, 0}, {2, 5, 2}, 
{2, 6, 0}, {4, 0, 0}, {4, 1, 0}, {4, 7, 0}, {5, 5, 0}, {5, 6, 0}, {5, 7, 0}, {7, 5, 0}, {7, 6, 0}, {0, 4, 2}, 
{0, 5, 0}, {0, 6, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 0}, {0, 0, 2}, {0, 1, 0}, {0, 2, 0}, {0, 3, 0}, {0, 7, 2}, 
{6, 0, 0}, {6, 1, 0}, {6, 2, 0}, {6, 3, 2}, {6, 4, 0}, {6, 5, 0}, {6, 6, 0}, {5, 2, 3}, {5, 3, 0}, {6, 7, 0}, 
{3, 5, 2}, {3, 6, 0}, {3, 7, 0}, {4, 3, 0}, {5, 1, 0}, {5, 4, 0}, {1, 2, 2}, {1, 3, 0}, {2, 7, 0}, {3, 0, 0},   // +51.467s 51472
{3, 1, 0}, {3, 2, 0}, {3, 3, 0}, {3, 4, 0}, {4, 2, 0}, {1, 0, 2}, {1, 1, 0}, {1, 4, 0}, {1, 5, 0}, {7, 0, 0}, 
{7, 1, 0}, {1, 6, 2}, {5, 0, 0}, {1, 7, 19}, {2, 0, 0}, {2, 1, 0}, {2, 2, 0}, {7, 2, 0}, {7, 3, 0}, {2, 3, 2}, 
{2, 4, 0}, {2, 5, 0}, {2, 6, 0}, {7, 4, 0}, {4, 0, 2}, {4, 1, 0}, {4, 7, 0}, {5, 5, 0}, {5, 6, 0}, {5, 7, 0}, 
{7, 5, 0}, {7, 6, 0}, {0, 0, 2}, {0, 1, 0}, {0, 4, 0}, {0, 5, 0}, {0, 6, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 0}, 
{0, 2, 2}, {0, 3, 0}, {0, 7, 0}, {6, 0, 2}, {6, 1, 0}, {6, 2, 0}, {6, 3, 2}, {6, 4, 0}, {6, 5, 0}, {6, 6, 0},   // +52.033s 52032
{3, 7, 2}, {5, 2, 0}, {5, 3, 0}, {6, 7, 0}, {2, 7, 2}, {3, 1, 0}, {3, 2, 0}, {3, 3, 0}, {3, 4, 0}, {3, 5, 0}, 
{3, 6, 0}, {4, 2, 0}, {4, 3, 0}, {5, 1, 0}, {5, 4, 0}, {1, 0, 2}, {1, 1, 0}, {1, 2, 0}, {1, 3, 0}, {3, 0, 0}, 
{1, 4, 2}, {1, 5, 0}, {1, 6, 0}, {5, 0, 0}, {7, 0, 0}, {7, 1, 0}, {1, 4, 23}, {1, 5, 0}, {1, 6, 0}, {5, 0, 0}, 
{7, 0, 0}, {7, 1, 0}, {1, 0, 2}, {1, 1, 0}, {1, 2, 0}, {1, 3, 0}, {3, 0, 0}, {2, 7, 3}, {3, 1, 0}, {3, 2, 0}, 
{3, 3, 0}, {3, 4, 0}, {3, 5, 0}, {3, 6, 0}, {4, 2, 0}, {4, 3, 0}, {5, 1, 0}, {3, 7, 2}, {5, 2, 0}, {5, 3, 0},   // +52.633s 52640
{5, 4, 0}, {6, 5, 2}, {6, 6, 0}, {6, 7, 0}, {0, 7, 2}, {6, 1, 0}, {6, 2, 0}, {6, 3, 0}, {6, 4, 0}, {0, 0, 2}, 
{0, 1, 0}, {0, 2, 0}, {0, 3, 0}, {6, 0, 0}, {0, 4, 2}, {0, 5, 0}, {0, 6, 0}, {2, 5, 2}, {2, 6, 0}, {4, 0, 0}, 
{4, 1, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {5, 5, 0}, {2, 1, 2}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, 
{5, 6, 0}, {5, 7, 0}, {7, 5, 0}, {7, 6, 0}, {1, 7, 2}, {2, 0, 0}, {7, 2, 0}, {7, 3, 0}, {7, 4, 0}, {1, 6, 19}, 
{5, 0, 0}, {1, 0, 2}, {1, 1, 0}, {1, 4, 0}, {1, 5, 0}, {7, 0, 0}, {7, 1, 0}, {1, 2, 2}, {1, 3, 0}, {2, 7, 0},   // +53.233s 53232
{3, 0, 0}, {3, 1, 0}, {3, 2, 0}, {3, 3, 0}, {3, 4, 0}, {4, 2, 0}, {3, 5, 2}, {3, 6, 0}, {3, 7, 0}, {4, 3, 0}, 
{5, 1, 0}, {5, 4, 0}, {5, 2, 2}, {5, 3, 0}, {6, 7, 0}, {6, 3, 2}, {6, 4, 0}, {6, 5, 0}, {6, 6, 0}, {0, 7, 2}, 
{6, 0, 0}, {6, 1, 0}, {6, 2, 0}, {0, 0, 3}, {0, 1, 0}, {0, 2, 0}, {0, 3, 0}, {0, 4, 2}, {0, 5, 0}, {0, 6, 0}, 
{4, 4, 0}, {4, 5, 0}, {4, 6, 0}, {2, 5, 2}, {2, 6, 0}, {4, 0, 0}, {4, 1, 0}, {4, 7, 0}, {5, 5, 0}, {5, 6, 0}, 
{5, 7, 0}, {7, 5, 0}, {7, 6, 0}, {2, 1, 2}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {7, 4, 0}, {1, 7, 2}, {2, 0, 0},   // +53.533s 53536
{7, 2, 0}, {7, 3, 0}, {1, 7, 21}, {2, 0, 0}, {2, 1, 0}, {2, 2, 0}, {7, 2, 0}, {7, 3, 0}, {2, 3, 2}, {2, 4, 0}, 
{2, 5, 0}, {2, 6, 0}, {7, 4, 0}, {4, 0, 2}, {4, 1, 0}, {4, 7, 0}, {5, 5, 0}, {5, 6, 0}, {5, 7, 0}, {7, 5, 0}, 
{7, 6, 0}, {0, 0, 2}, {0, 1, 0}, {0, 4, 0}, {0, 5, 0}, {0, 6, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 0}, {0, 2, 2}, 
{0, 3, 0}, {0, 7, 0}, {6, 0, 2}, {6, 1, 0}, {6, 2, 0}, {6, 3, 4}, {6, 4, 0}, {6, 5, 0}, {6, 6, 0}, {3, 7, 2}, 
{5, 2, 0}, {5, 3, 0}, {5, 4, 0}, {6, 7, 0}, {2, 7, 2}, {3, 1, 0}, {3, 2, 0}, {3, 3, 0}, {3, 4, 0}, {3, 5, 0},   // +54.167s 54160
{3, 6, 0}, {4, 2, 0}, {4, 3, 0}, {5, 1, 0}, {1, 0, 3}, {1, 1, 0}, {1, 2, 0}, {1, 3, 0}, {3, 0, 0}, {1, 4, 2}, 
{1, 5, 0}, {1, 6, 0}, {5, 0, 0}, {7, 0, 0}, {7, 1, 0}, {0, 4, 20}, {5, 5, 0}, {0, 0, 2}, {0, 1, 0}, {0, 4, 0}, 
{0, 5, 0}, {1, 1, 0}, {1, 4, 0}, {2, 0, 0}, {2, 5, 0}, {3, 1, 0}, {3, 5, 0}, {4, 3, 0}, {4, 5, 0}, {5, 2, 0}, 
{7, 1, 0}, {0, 0, 3}, {1, 7, 0}, {2, 0, 0}, {2, 1, 0}, {5, 0, 0}, {6, 1, 0}, {0, 6, 2}, {1, 5, 0}, {4, 5, 0}, 
{4, 6, 0}, {5, 0, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {7, 5, 0}, {0, 2, 2}, {0, 5, 0}, {1, 1, 0}, {1, 2, 0},   // +54.700s 54704
{1, 4, 0}, {2, 2, 0}, {2, 6, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0}, {3, 6, 0}, {4, 3, 0}, {5, 2, 0}, {6, 2, 0}, 
{7, 1, 0}, {7, 2, 0}, {7, 4, 0}, {0, 1, 2}, {1, 7, 0}, {2, 1, 0}, {2, 5, 0}, {4, 7, 0}, {5, 7, 0}, {6, 1, 0}, 
{0, 4, 2}, {0, 6, 0}, {1, 6, 0}, {2, 3, 0}, {2, 4, 0}, {3, 7, 0}, {4, 6, 0}, {5, 3, 0}, {5, 4, 0}, {5, 6, 0}, 
{7, 5, 0}, {7, 6, 0}, {0, 2, 2}, {0, 3, 0}, {1, 2, 0}, {1, 3, 0}, {1, 5, 0}, {2, 2, 0}, {2, 6, 0}, {2, 7, 0}, 
{3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {6, 2, 0}, {6, 3, 0}, {7, 2, 0}, {7, 3, 0}, {0, 5, 2}, {1, 4, 0}, {4, 2, 0},   // +54.833s 54832
{4, 4, 0}, {4, 7, 0}, {5, 2, 0}, {5, 5, 0}, {5, 7, 0}, {7, 4, 0}, {0, 0, 2}, {0, 4, 0}, {1, 0, 0}, {1, 6, 0}, 
{2, 0, 0}, {2, 3, 0}, {2, 4, 0}, {2, 5, 0}, {3, 0, 0}, {3, 3, 0}, {3, 4, 0}, {3, 7, 0}, {5, 4, 0}, {7, 0, 0}, 
{7, 6, 0}, {0, 3, 2}, {1, 3, 0}, {2, 7, 0}, {4, 1, 0}, {4, 5, 0}, {5, 6, 0}, {6, 3, 0}, {6, 4, 0}, {7, 3, 0}, 
{0, 5, 2}, {0, 6, 0}, {1, 4, 0}, {1, 5, 0}, {2, 1, 0}, {2, 6, 0}, {3, 0, 0}, {3, 1, 0}, {3, 4, 0}, {3, 5, 0}, 
{4, 2, 0}, {4, 3, 0}, {4, 4, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {0, 0, 2}, {0, 1, 0}, {1, 0, 0}, {1, 1, 0},   // +54.967s 54960
{2, 0, 0}, {2, 5, 0}, {6, 5, 0}, {7, 0, 0}, {7, 1, 0}, {0, 4, 3}, {1, 6, 0}, {3, 2, 0}, {3, 6, 0}, {4, 1, 0}, 
{4, 5, 0}, {4, 6, 0}, {5, 1, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0}, {6, 4, 0}, {7, 5, 0}, {0, 2, 2}, {0, 6, 0}, 
{1, 2, 0}, {1, 5, 0}, {2, 1, 0}, {2, 2, 0}, {2, 4, 0}, {2, 6, 0}, {3, 1, 0}, {3, 5, 0}, {4, 3, 0}, {5, 3, 0}, 
{7, 2, 0}, {0, 1, 2}, {0, 7, 0}, {1, 1, 0}, {4, 6, 0}, {4, 7, 0}, {5, 1, 0}, {5, 5, 0}, {6, 5, 0}, {6, 6, 0}, 
{7, 1, 0}, {0, 4, 2}, {0, 5, 0}, {1, 4, 0}, {1, 6, 0}, {2, 3, 0}, {2, 5, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0},   // +55.100s 55104
{3, 7, 0}, {5, 2, 0}, {5, 4, 0}, {5, 7, 0}, {7, 5, 0}, {7, 6, 0}, {0, 2, 2}, {0, 3, 0}, {1, 2, 0}, {1, 3, 0}, 
{2, 2, 0}, {2, 4, 0}, {4, 4, 0}, {5, 0, 0}, {5, 6, 0}, {6, 7, 0}, {7, 2, 0}, {7, 3, 0}, {0, 6, 2}, {0, 7, 0}, 
{1, 5, 0}, {3, 0, 0}, {3, 4, 0}, {4, 2, 0}, {4, 7, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {6, 6, 0}, {0, 0, 2}, 
{0, 5, 0}, {1, 0, 0}, {1, 4, 0}, {2, 0, 0}, {2, 3, 0}, {2, 5, 0}, {2, 6, 0}, {3, 3, 0}, {3, 7, 0}, {7, 0, 0}, 
{7, 6, 0}, {0, 3, 2}, {1, 3, 0}, {1, 7, 0}, {4, 3, 0}, {4, 4, 0}, {4, 5, 0}, {5, 0, 0}, {5, 4, 0}, {5, 6, 0},   // +55.233s 55232
{5, 7, 0}, {6, 0, 0}, {6, 7, 0}, {7, 3, 0}, {0, 4, 2}, {0, 6, 0}, {1, 6, 0}, {2, 1, 0}, {2, 4, 0}, {3, 1, 0}, 
{3, 4, 0}, {3, 5, 0}, {4, 2, 0}, {5, 3, 0}, {0, 0, 2}, {0, 1, 0}, {1, 0, 0}, {1, 1, 0}, {1, 5, 0}, {2, 0, 0}, 
{2, 6, 0}, {3, 0, 0}, {4, 6, 0}, {5, 5, 0}, {6, 0, 0}, {6, 1, 0}, {7, 0, 0}, {7, 1, 0}, {7, 4, 0}, {0, 5, 2}, 
{1, 7, 0}, {3, 6, 0}, {4, 3, 0}, {4, 5, 0}, {5, 2, 0}, {5, 4, 0}, {5, 7, 0}, {7, 5, 0}, {0, 2, 2}, {0, 4, 0}, 
{1, 2, 0}, {1, 4, 0}, {1, 6, 0}, {2, 1, 0}, {2, 2, 0}, {2, 4, 0}, {2, 5, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0},   // +55.367s 55360
{6, 2, 0}, {7, 2, 0}, {0, 1, 3}, {1, 1, 0}, {2, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, 
{6, 1, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {0, 5, 2}, {0, 6, 0}, {1, 5, 0}, {2, 3, 0}, {2, 6, 0}, {3, 3, 0}, 
{3, 6, 0}, {3, 7, 0}, {5, 2, 0}, {7, 3, 0}, {7, 5, 0}, {0, 2, 2}, {0, 3, 0}, {1, 2, 0}, {1, 3, 0}, {1, 4, 0}, 
{2, 2, 0}, {2, 5, 0}, {3, 2, 0}, {4, 1, 0}, {4, 4, 0}, {5, 7, 0}, {6, 2, 0}, {6, 3, 0}, {7, 2, 0}, {0, 4, 2}, 
{2, 7, 0}, {3, 4, 0}, {4, 2, 0}, {4, 7, 0}, {5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {7, 6, 0}, {0, 0, 2}, {0, 3, 0},   // +55.533s 55536
{0, 6, 0}, {1, 0, 0}, {1, 5, 0}, {1, 6, 0}, {2, 0, 0}, {2, 3, 0}, {2, 4, 0}, {2, 6, 0}, {3, 0, 0}, {3, 3, 0}, 
{3, 7, 0}, {5, 1, 0}, {6, 4, 0}, {7, 0, 0}, {7, 3, 0}, {0, 5, 2}, {1, 3, 0}, {4, 1, 0}, {4, 3, 0}, {4, 4, 0}, 
{4, 5, 0}, {5, 2, 0}, {5, 5, 0}, {5, 7, 0}, {6, 3, 0}, {0, 1, 2}, {0, 4, 0}, {1, 1, 0}, {1, 4, 0}, {2, 1, 0}, 
{2, 5, 0}, {3, 1, 0}, {3, 4, 0}, {3, 5, 0}, {4, 2, 0}, {5, 4, 0}, {7, 1, 0}, {0, 0, 2}, {0, 7, 0}, {1, 0, 0}, 
{1, 6, 0}, {2, 0, 0}, {2, 4, 0}, {3, 0, 0}, {4, 6, 0}, {5, 1, 0}, {5, 6, 0}, {6, 4, 0}, {6, 5, 0}, {7, 0, 0},   // +55.633s 55632
{0, 5, 2}, {0, 6, 0}, {1, 5, 0}, {3, 6, 0}, {4, 3, 0}, {4, 5, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {7, 5, 0}, 
{0, 1, 2}, {0, 2, 0}, {1, 1, 0}, {1, 2, 0}, {1, 4, 0}, {2, 1, 0}, {2, 2, 0}, {2, 5, 0}, {2, 6, 0}, {3, 1, 0}, 
{3, 2, 0}, {3, 5, 0}, {5, 0, 0}, {6, 6, 0}, {7, 1, 0}, {7, 2, 0}, {0, 4, 2}, {0, 7, 0}, {4, 6, 0}, {4, 7, 0}, 
{5, 4, 0}, {5, 6, 0}, {5, 7, 0}, {6, 5, 0}, {7, 6, 0}, {0, 3, 2}, {0, 6, 0}, {1, 3, 0}, {1, 5, 0}, {1, 6, 0}, 
{2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {2, 6, 0}, {3, 3, 0}, {3, 6, 0}, {3, 7, 0}, {5, 3, 0}, {7, 3, 0}, {7, 5, 0},   // +55.767s 55760
{0, 2, 3}, {1, 2, 0}, {1, 7, 0}, {3, 2, 0}, {4, 4, 0}, {5, 0, 0}, {5, 5, 0}, {6, 6, 0}, {6, 7, 0}, {7, 2, 0}, 
{0, 4, 2}, {0, 5, 0}, {1, 4, 0}, {2, 0, 0}, {2, 5, 0}, {3, 4, 0}, {4, 2, 0}, {4, 7, 0}, {5, 2, 0}, {5, 4, 0}, 
{5, 7, 0}, {7, 6, 0}, {0, 0, 2}, {0, 3, 0}, {1, 0, 0}, {1, 3, 0}, {1, 6, 0}, {2, 3, 0}, {2, 4, 0}, {3, 0, 0}, 
{3, 3, 0}, {3, 7, 0}, {6, 0, 0}, {7, 0, 0}, {7, 3, 0}, {7, 4, 0}, {0, 6, 2}, {1, 7, 0}, {3, 5, 0}, {4, 3, 0}, 
{4, 4, 0}, {4, 5, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {6, 7, 0}, {0, 1, 2}, {0, 5, 0}, {1, 1, 0}, {1, 4, 0},   // +55.933s 55936
{1, 5, 0}, {2, 0, 0}, {2, 1, 0}, {2, 5, 0}, {2, 6, 0}, {3, 0, 0}, {3, 1, 0}, {3, 4, 0}, {4, 2, 0}, {5, 2, 0}, 
{7, 1, 0}, {0, 0, 2}, {1, 0, 0}, {2, 7, 0}, {4, 6, 0}, {5, 7, 0}, {6, 0, 0}, {6, 1, 0}, {7, 0, 0}, {7, 4, 0}, 
{0, 4, 2}, {0, 6, 0}, {1, 6, 0}, {2, 2, 0}, {2, 4, 0}, {3, 2, 0}, {3, 5, 0}, {3, 6, 0}, {4, 3, 0}, {4, 5, 0}, 
{5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {7, 5, 0}, {0, 1, 2}, {0, 2, 0}, {1, 1, 0}, {1, 2, 0}, {1, 5, 0}, {2, 1, 0}, 
{2, 6, 0}, {3, 1, 0}, {7, 1, 0}, {7, 2, 0}, {4, 7, 2}, {5, 5, 0}, {0, 3, 2}, {0, 4, 0}, {0, 5, 0}, {1, 3, 0},   // +56.100s 56096
{1, 4, 0}, {1, 6, 0}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {2, 5, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {3, 7, 0}, 
{4, 6, 0}, {5, 2, 0}, {5, 4, 0}, {5, 7, 0}, {7, 3, 0}, {7, 5, 0}, {7, 6, 0}, {0, 2, 2}, {1, 2, 0}, {2, 7, 0}, 
{4, 1, 0}, {4, 4, 0}, {5, 6, 0}, {6, 1, 0}, {6, 2, 0}, {7, 2, 0}, {0, 6, 2}, {1, 5, 0}, {3, 0, 0}, {3, 4, 0}, 
{4, 2, 0}, {4, 7, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {0, 0, 2}, {0, 5, 0}, {1, 0, 0}, {1, 3, 0}, {1, 4, 0}, 
{2, 0, 0}, {2, 3, 0}, {2, 5, 0}, {2, 6, 0}, {3, 3, 0}, {3, 7, 0}, {5, 1, 0}, {6, 3, 0}, {7, 0, 0}, {7, 3, 0},   // +56.200s 56192
{7, 6, 0}, {0, 3, 3}, {4, 1, 0}, {4, 3, 0}, {4, 4, 0}, {4, 5, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0}, {6, 2, 0}, 
{0, 4, 2}, {0, 6, 0}, {1, 1, 0}, {1, 5, 0}, {1, 6, 0}, {2, 1, 0}, {2, 4, 0}, {3, 0, 0}, {3, 1, 0}, {3, 4, 0}, 
{3, 5, 0}, {4, 2, 0}, {5, 3, 0}, {6, 4, 0}, {7, 1, 0}, {0, 0, 2}, {0, 1, 0}, {0, 7, 0}, {1, 0, 0}, {2, 0, 0}, 
{2, 6, 0}, {4, 6, 0}, {5, 1, 0}, {5, 5, 0}, {6, 3, 0}, {7, 0, 0}, {0, 5, 2}, {1, 4, 0}, {3, 2, 0}, {3, 6, 0}, 
{4, 3, 0}, {4, 5, 0}, {5, 2, 0}, {5, 4, 0}, {5, 7, 0}, {7, 5, 0}, {0, 2, 2}, {0, 4, 0}, {1, 1, 0}, {1, 2, 0},   // +56.367s 56368
{1, 6, 0}, {2, 1, 0}, {2, 2, 0}, {2, 4, 0}, {2, 5, 0}, {3, 1, 0}, {3, 5, 0}, {5, 0, 0}, {6, 4, 0}, {6, 5, 0}, 
{7, 1, 0}, {7, 2, 0}, {0, 1, 2}, {0, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {7, 6, 0}, 
{0, 5, 2}, {0, 6, 0}, {1, 4, 0}, {1, 5, 0}, {2, 3, 0}, {2, 6, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {3, 7, 0}, 
{5, 2, 0}, {7, 3, 0}, {7, 5, 0}, {0, 2, 2}, {0, 3, 0}, {1, 2, 0}, {1, 3, 0}, {1, 7, 0}, {2, 2, 0}, {2, 5, 0}, 
{4, 4, 0}, {5, 0, 0}, {5, 7, 0}, {6, 5, 0}, {6, 6, 0}, {7, 2, 0}, {0, 4, 2}, {1, 6, 0}, {3, 0, 0}, {3, 4, 0},   // +56.500s 56496
{4, 2, 0}, {4, 7, 0}, {5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {7, 6, 0}, {0, 0, 2}, {0, 3, 0}, {0, 6, 0}, {1, 0, 0}, 
{1, 5, 0}, {2, 0, 0}, {2, 3, 0}, {2, 4, 0}, {2, 6, 0}, {3, 3, 0}, {3, 7, 0}, {6, 7, 0}, {7, 0, 0}, {7, 3, 0}, 
{7, 4, 0}, {0, 5, 2}, {1, 3, 0}, {1, 7, 0}, {4, 3, 0}, {4, 4, 0}, {4, 5, 0}, {5, 2, 0}, {5, 5, 0}, {5, 7, 0}, 
{6, 6, 0}, {0, 1, 3}, {0, 4, 0}, {1, 1, 0}, {1, 4, 0}, {1, 6, 0}, {2, 1, 0}, {2, 5, 0}, {3, 0, 0}, {3, 1, 0}, 
{3, 4, 0}, {3, 5, 0}, {4, 2, 0}, {5, 4, 0}, {7, 1, 0}, {0, 0, 2}, {1, 0, 0}, {2, 0, 0}, {2, 4, 0}, {2, 7, 0},   // +56.633s 56640
{4, 6, 0}, {5, 6, 0}, {6, 0, 0}, {6, 7, 0}, {7, 0, 0}, {7, 4, 0}, {0, 5, 2}, {0, 6, 0}, {1, 4, 0}, {1, 5, 0}, 
{3, 2, 0}, {3, 6, 0}, {4, 3, 0}, {4, 5, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {7, 5, 0}, {0, 1, 2}, {0, 2, 0}, 
{1, 1, 0}, {1, 2, 0}, {2, 1, 0}, {2, 2, 0}, {2, 5, 0}, {2, 6, 0}, {3, 1, 0}, {3, 5, 0}, {6, 1, 0}, {7, 1, 0}, 
{7, 2, 0}, {0, 4, 2}, {1, 6, 0}, {2, 7, 0}, {4, 1, 0}, {4, 6, 0}, {4, 7, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0}, 
{6, 0, 0}, {7, 6, 0}, {0, 3, 2}, {0, 6, 0}, {1, 3, 0}, {1, 5, 0}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {2, 6, 0},   // +56.767s 56768
{3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {3, 7, 0}, {5, 3, 0}, {7, 3, 0}, {7, 5, 0}, {0, 2, 2}, {1, 2, 0}, {4, 1, 0}, 
{4, 4, 0}, {5, 1, 0}, {5, 5, 0}, {6, 1, 0}, {6, 2, 0}, {7, 2, 0}, {0, 4, 2}, {0, 5, 0}, {1, 4, 0}, {1, 6, 0}, 
{2, 0, 0}, {2, 5, 0}, {3, 0, 0}, {3, 4, 0}, {4, 2, 0}, {4, 7, 0}, {5, 2, 0}, {5, 4, 0}, {5, 7, 0}, {7, 6, 0}, 
{0, 0, 2}, {0, 3, 0}, {0, 7, 0}, {1, 0, 0}, {1, 3, 0}, {2, 3, 0}, {2, 4, 0}, {3, 3, 0}, {3, 7, 0}, {6, 3, 0}, 
{7, 0, 0}, {7, 3, 0}, {0, 6, 2}, {1, 5, 0}, {3, 1, 0}, {3, 5, 0}, {4, 3, 0}, {4, 4, 0}, {4, 5, 0}, {5, 1, 0},   // +56.900s 56896
{5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {6, 2, 0}, {0, 1, 2}, {0, 5, 0}, {1, 1, 0}, {1, 4, 0}, {2, 0, 0}, {2, 1, 0}, 
{2, 5, 0}, {2, 6, 0}, {3, 0, 0}, {3, 4, 0}, {4, 2, 0}, {5, 2, 0}, {7, 1, 0}, {0, 0, 2}, {0, 7, 0}, {1, 0, 0}, 
{4, 6, 0}, {5, 0, 0}, {5, 7, 0}, {6, 3, 0}, {6, 4, 0}, {7, 0, 0}, {0, 4, 3}, {0, 6, 0}, {1, 5, 0}, {1, 6, 0}, 
{2, 2, 0}, {2, 4, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0}, {3, 6, 0}, {4, 3, 0}, {4, 5, 0}, {5, 3, 0}, {5, 4, 0}, 
{5, 6, 0}, {7, 5, 0}, {0, 1, 2}, {0, 2, 0}, {1, 1, 0}, {1, 2, 0}, {1, 7, 0}, {2, 1, 0}, {2, 6, 0}, {4, 7, 0},   // +57.033s 57040
{6, 5, 0}, {7, 1, 0}, {7, 2, 0}, {0, 5, 2}, {1, 4, 0}, {3, 3, 0}, {3, 7, 0}, {4, 6, 0}, {5, 0, 0}, {5, 2, 0}, 
{5, 5, 0}, {5, 7, 0}, {6, 4, 0}, {7, 6, 0}, {0, 3, 2}, {0, 4, 0}, {1, 3, 0}, {1, 6, 0}, {2, 2, 0}, {2, 3, 0}, 
{2, 4, 0}, {2, 5, 0}, {3, 2, 0}, {3, 6, 0}, {5, 4, 0}, {7, 3, 0}, {7, 5, 0}, {0, 2, 2}, {1, 2, 0}, {1, 7, 0}, 
{4, 2, 0}, {4, 4, 0}, {4, 7, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {6, 5, 0}, {6, 6, 0}, {7, 2, 0}, {7, 4, 0}, 
{0, 5, 2}, {0, 6, 0}, {1, 4, 0}, {1, 5, 0}, {2, 0, 0}, {2, 6, 0}, {3, 0, 0}, {3, 3, 0}, {3, 4, 0}, {3, 7, 0},   // +57.167s 57168
{5, 2, 0}, {7, 6, 0}, {0, 0, 2}, {0, 3, 0}, {1, 0, 0}, {1, 3, 0}, {2, 3, 0}, {2, 5, 0}, {2, 7, 0}, {4, 5, 0}, 
{5, 7, 0}, {6, 7, 0}, {7, 0, 0}, {7, 3, 0}, {0, 4, 2}, {1, 6, 0}, {3, 1, 0}, {3, 5, 0}, {4, 2, 0}, {4, 3, 0}, 
{4, 4, 0}, {5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {6, 6, 0}, {7, 4, 0}, {0, 1, 2}, {0, 6, 0}, {1, 1, 0}, {1, 5, 0}, 
{2, 0, 0}, {2, 1, 0}, {2, 4, 0}, {2, 6, 0}, {3, 0, 0}, {3, 4, 0}, {4, 1, 0}, {6, 0, 0}, {7, 1, 0}, {0, 0, 2}, 
{1, 0, 0}, {2, 7, 0}, {4, 5, 0}, {4, 6, 0}, {5, 2, 0}, {5, 5, 0}, {5, 7, 0}, {6, 7, 0}, {7, 0, 0}, {0, 4, 2},   // +57.333s 57328
{0, 5, 0}, {1, 4, 0}, {1, 6, 0}, {2, 2, 0}, {2, 5, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0}, {3, 6, 0}, {4, 3, 0}, 
{5, 4, 0}, {7, 5, 0}, {0, 1, 2}, {0, 2, 0}, {1, 1, 0}, {1, 2, 0}, {2, 1, 0}, {2, 4, 0}, {4, 7, 0}, {5, 1, 0}, 
{5, 6, 0}, {6, 0, 0}, {6, 1, 0}, {7, 1, 0}, {7, 2, 0}, {0, 6, 3}, {1, 5, 0}, {3, 3, 0}, {3, 7, 0}, {4, 1, 0}, 
{4, 6, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {7, 5, 0}, {7, 6, 0}, {0, 3, 2}, {0, 5, 0}, {1, 3, 0}, {1, 4, 0}, 
{2, 2, 0}, {2, 3, 0}, {2, 5, 0}, {2, 6, 0}, {3, 2, 0}, {3, 6, 0}, {6, 2, 0}, {7, 2, 0}, {7, 3, 0}, {0, 2, 2},   // +57.467s 57472
{0, 7, 0}, {1, 2, 0}, {4, 2, 0}, {4, 4, 0}, {4, 7, 0}, {5, 1, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0}, {6, 1, 0}, 
{0, 0, 2}, {0, 4, 0}, {0, 6, 0}, {1, 5, 0}, {1, 6, 0}, {2, 0, 0}, {2, 4, 0}, {3, 0, 0}, {3, 3, 0}, {3, 4, 0}, 
{3, 7, 0}, {5, 3, 0}, {7, 0, 0}, {7, 6, 0}, {0, 3, 2}, {0, 7, 0}, {1, 0, 0}, {1, 3, 0}, {2, 3, 0}, {2, 6, 0}, 
{4, 5, 0}, {5, 0, 0}, {5, 5, 0}, {6, 2, 0}, {6, 3, 0}, {7, 3, 0}, {0, 4, 2}, {0, 5, 0}, {1, 4, 0}, {3, 1, 0}, 
{3, 5, 0}, {4, 2, 0}, {4, 3, 0}, {4, 4, 0}, {5, 2, 0}, {5, 4, 0}, {5, 7, 0}, {0, 0, 2}, {0, 1, 0}, {1, 1, 0},   // +57.600s 57600
{1, 6, 0}, {1, 7, 0}, {2, 0, 0}, {2, 1, 0}, {2, 4, 0}, {2, 5, 0}, {3, 0, 0}, {3, 4, 0}, {6, 4, 0}, {7, 0, 0}, 
{7, 1, 0}, {0, 6, 2}, {1, 0, 0}, {1, 5, 0}, {4, 5, 0}, {4, 6, 0}, {5, 0, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, 
{6, 3, 0}, {7, 5, 0}, {0, 2, 2}, {0, 5, 0}, {1, 2, 0}, {1, 4, 0}, {2, 2, 0}, {2, 6, 0}, {3, 1, 0}, {3, 2, 0}, 
{3, 5, 0}, {3, 6, 0}, {4, 3, 0}, {5, 2, 0}, {7, 2, 0}, {0, 1, 2}, {1, 1, 0}, {1, 7, 0}, {2, 1, 0}, {2, 5, 0}, 
{4, 7, 0}, {5, 7, 0}, {6, 4, 0}, {6, 5, 0}, {7, 1, 0}, {7, 4, 0}, {0, 4, 2}, {0, 6, 0}, {1, 5, 0}, {1, 6, 0},   // +57.733s 57728
{2, 3, 0}, {2, 4, 0}, {3, 3, 0}, {3, 7, 0}, {4, 6, 0}, {5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {7, 5, 0}, {7, 6, 0}, 
{0, 2, 2}, {0, 3, 0}, {1, 2, 0}, {1, 3, 0}, {2, 2, 0}, {2, 6, 0}, {2, 7, 0}, {3, 2, 0}, {3, 6, 0}, {6, 6, 0}, 
{7, 2, 0}, {7, 3, 0}, {0, 5, 2}, {1, 4, 0}, {4, 2, 0}, {4, 4, 0}, {4, 7, 0}, {5, 2, 0}, {5, 5, 0}, {5, 7, 0}, 
{6, 5, 0}, {7, 4, 0}, {0, 0, 3}, {0, 4, 0}, {1, 0, 0}, {1, 6, 0}, {2, 0, 0}, {2, 3, 0}, {2, 4, 0}, {2, 5, 0}, 
{3, 0, 0}, {3, 3, 0}, {3, 4, 0}, {3, 7, 0}, {5, 4, 0}, {7, 0, 0}, {7, 6, 0}, {0, 3, 2}, {1, 3, 0}, {2, 7, 0},   // +57.867s 57872
{4, 1, 0}, {4, 5, 0}, {5, 6, 0}, {6, 6, 0}, {6, 7, 0}, {7, 3, 0}, {0, 5, 2}, {0, 6, 0}, {1, 4, 0}, {1, 5, 0}, 
{2, 1, 0}, {2, 6, 0}, {3, 0, 0}, {3, 1, 0}, {3, 4, 0}, {3, 5, 0}, {4, 2, 0}, {4, 3, 0}, {4, 4, 0}, {5, 2, 0}, 
{5, 3, 0}, {5, 5, 0}, {0, 0, 2}, {0, 1, 0}, {1, 0, 0}, {1, 1, 0}, {2, 0, 0}, {2, 5, 0}, {5, 1, 0}, {6, 0, 0}, 
{7, 0, 0}, {7, 1, 0}, {0, 4, 2}, {1, 6, 0}, {3, 2, 0}, {3, 6, 0}, {4, 1, 0}, {4, 6, 0}, {5, 4, 0}, {5, 6, 0}, 
{5, 7, 0}, {6, 7, 0}, {7, 5, 0}, {0, 2, 2}, {0, 6, 0}, {1, 2, 0}, {1, 5, 0}, {2, 1, 0}, {2, 2, 0}, {2, 4, 0},   // +58.000s 58000
{2, 6, 0}, {3, 1, 0}, {3, 5, 0}, {4, 3, 0}, {4, 5, 0}, {5, 3, 0}, {7, 2, 0}, {0, 1, 2}, {0, 7, 0}, {1, 1, 0}, 
{4, 7, 0}, {5, 1, 0}, {5, 5, 0}, {6, 0, 0}, {6, 1, 0}, {7, 1, 0}, {0, 4, 2}, {0, 5, 0}, {1, 4, 0}, {1, 6, 0}, 
{2, 3, 0}, {2, 5, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {3, 7, 0}, {4, 6, 0}, {5, 2, 0}, {5, 4, 0}, {5, 7, 0}, 
{7, 5, 0}, {7, 6, 0}, {0, 2, 2}, {0, 3, 0}, {1, 2, 0}, {1, 3, 0}, {2, 2, 0}, {2, 4, 0}, {5, 0, 0}, {5, 6, 0}, 
{6, 2, 0}, {7, 2, 0}, {7, 3, 0}, {0, 6, 2}, {0, 7, 0}, {3, 4, 0}, {4, 2, 0}, {4, 4, 0}, {4, 7, 0}, {5, 2, 0},   // +58.133s 58128
{5, 3, 0}, {5, 5, 0}, {6, 1, 0}, {0, 0, 2}, {0, 5, 0}, {1, 0, 0}, {1, 4, 0}, {1, 5, 0}, {2, 0, 0}, {2, 3, 0}, 
{2, 5, 0}, {2, 6, 0}, {3, 0, 0}, {3, 3, 0}, {3, 7, 0}, {7, 0, 0}, {7, 6, 0}, {0, 3, 3}, {1, 3, 0}, {1, 7, 0}, 
{4, 5, 0}, {5, 0, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0}, {6, 2, 0}, {6, 3, 0}, {7, 3, 0}, {0, 4, 2}, {0, 6, 0}, 
{1, 6, 0}, {2, 1, 0}, {2, 4, 0}, {3, 1, 0}, {3, 4, 0}, {3, 5, 0}, {4, 2, 0}, {4, 3, 0}, {4, 4, 0}, {5, 3, 0}, 
{0, 0, 2}, {0, 1, 0}, {1, 0, 0}, {1, 1, 0}, {1, 5, 0}, {2, 0, 0}, {2, 6, 0}, {3, 0, 0}, {5, 5, 0}, {6, 3, 0},   // +58.267s 58272
{6, 4, 0}, {7, 0, 0}, {7, 1, 0}, {7, 4, 0}, {0, 5, 2}, {1, 7, 0}, {3, 6, 0}, {4, 5, 0}, {4, 6, 0}, {5, 2, 0}, 
{5, 4, 0}, {5, 7, 0}, {7, 5, 0}, {0, 2, 2}, {0, 4, 0}, {1, 2, 0}, {1, 4, 0}, {1, 6, 0}, {2, 1, 0}, {2, 2, 0}, 
{2, 4, 0}, {2, 5, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0}, {4, 3, 0}, {6, 5, 0}, {7, 2, 0}, {0, 1, 2}, {1, 1, 0}, 
{2, 7, 0}, {4, 7, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {6, 4, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {0, 5, 2}, 
{0, 6, 0}, {1, 5, 0}, {2, 3, 0}, {2, 6, 0}, {3, 3, 0}, {3, 6, 0}, {3, 7, 0}, {4, 6, 0}, {5, 2, 0}, {7, 3, 0},   // +58.400s 58400
{7, 5, 0}, {0, 2, 2}, {0, 3, 0}, {1, 2, 0}, {1, 3, 0}, {1, 4, 0}, {2, 2, 0}, {2, 5, 0}, {3, 2, 0}, {4, 1, 0}, 
{5, 7, 0}, {6, 5, 0}, {6, 6, 0}, {7, 2, 0}, {0, 4, 2}, {2, 7, 0}, {3, 4, 0}, {4, 2, 0}, {4, 4, 0}, {4, 7, 0}, 
{5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {7, 6, 0}, {0, 0, 2}, {0, 6, 0}, {1, 0, 0}, {1, 5, 0}, {1, 6, 0}, {2, 0, 0}, 
{2, 3, 0}, {2, 4, 0}, {2, 6, 0}, {3, 0, 0}, {3, 3, 0}, {3, 7, 0}, {6, 7, 0}, {7, 0, 0}, {7, 3, 0}, {0, 3, 2}, 
{0, 5, 0}, {1, 3, 0}, {4, 1, 0}, {4, 5, 0}, {5, 1, 0}, {5, 2, 0}, {5, 5, 0}, {5, 7, 0}, {6, 6, 0}, {0, 1, 2},   // +58.567s 58560
{0, 4, 0}, {1, 4, 0}, {2, 1, 0}, {2, 5, 0}, {3, 1, 0}, {3, 4, 0}, {3, 5, 0}, {4, 2, 0}, {4, 3, 0}, {4, 4, 0}, 
{5, 4, 0}, {7, 1, 0}, {0, 0, 3}, {0, 7, 0}, {1, 0, 0}, {1, 1, 0}, {1, 6, 0}, {2, 0, 0}, {2, 4, 0}, {3, 0, 0}, 
{5, 6, 0}, {6, 0, 0}, {6, 7, 0}, {7, 0, 0}, {0, 5, 2}, {0, 6, 0}, {1, 5, 0}, {3, 6, 0}, {4, 5, 0}, {4, 6, 0}, 
{5, 1, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {7, 5, 0}, {0, 1, 2}, {0, 2, 0}, {1, 1, 0}, {1, 2, 0}, {1, 4, 0}, 
{2, 1, 0}, {2, 2, 0}, {2, 5, 0}, {2, 6, 0}, {3, 1, 0}, {3, 2, 0}, {3, 5, 0}, {4, 3, 0}, {6, 1, 0}, {7, 1, 0},   // +58.667s 58672
{7, 2, 0}, {0, 4, 2}, {0, 7, 0}, {4, 7, 0}, {5, 0, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0}, {6, 0, 0}, {7, 6, 0}, 
{0, 6, 2}, {1, 6, 0}, {2, 3, 0}, {2, 4, 0}, {3, 7, 0}, {4, 6, 0}, {5, 3, 0}, {7, 5, 0}, {0, 2, 2}, {0, 3, 0}, 
{1, 2, 0}, {1, 3, 0}, {1, 5, 0}, {1, 7, 0}, {2, 2, 0}, {2, 6, 0}, {3, 2, 0}, {3, 3, 0}, {3, 6, 0}, {5, 5, 0}, 
{6, 1, 0}, {6, 2, 0}, {7, 2, 0}, {7, 3, 0}, {0, 4, 2}, {0, 5, 0}, {4, 2, 0}, {4, 4, 0}, {4, 7, 0}, {5, 0, 0}, 
{5, 2, 0}, {5, 4, 0}, {5, 7, 0}, {7, 6, 0}, {0, 0, 2}, {1, 0, 0}, {1, 4, 0}, {1, 6, 0}, {2, 0, 0}, {2, 3, 0},   // +58.833s 58832
{2, 4, 0}, {2, 5, 0}, {3, 0, 0}, {3, 4, 0}, {3, 7, 0}, {7, 0, 0}, {0, 3, 2}, {0, 6, 0}, {1, 3, 0}, {1, 7, 0}, 
{3, 3, 0}, {4, 5, 0}, {5, 3, 0}, {5, 5, 0}, {5, 6, 0}, {6, 2, 0}, {6, 3, 0}, {7, 3, 0}, {7, 4, 0}, {0, 5, 2}, 
{1, 5, 0}, {3, 1, 0}, {3, 5, 0}, {4, 2, 0}, {4, 3, 0}, {4, 4, 0}, {5, 2, 0}, {0, 1, 2}, {1, 1, 0}, {1, 4, 0}, 
{2, 0, 0}, {2, 1, 0}, {2, 5, 0}, {2, 6, 0}, {2, 7, 0}, {3, 0, 0}, {3, 4, 0}, {6, 4, 0}, {7, 1, 0}, {0, 0, 2}, 
{0, 4, 0}, {0, 6, 0}, {1, 0, 0}, {4, 5, 0}, {4, 6, 0}, {5, 4, 0}, {5, 6, 0}, {5, 7, 0}, {6, 3, 0}, {7, 0, 0},   // +58.967s 58960
{7, 4, 0}, {7, 5, 0}, {3, 2, 2}, {3, 6, 0}, {4, 3, 0}, {5, 3, 0}, {0, 2, 3}, {1, 2, 0}, {1, 5, 0}, {1, 6, 0}, 
{2, 1, 0}, {2, 2, 0}, {2, 4, 0}, {2, 6, 0}, {3, 1, 0}, {3, 5, 0}, {4, 1, 0}, {6, 5, 0}, {7, 2, 0}, {0, 1, 2}, 
{0, 4, 0}, {0, 5, 0}, {1, 1, 0}, {2, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 2, 0}, {5, 5, 0}, {5, 7, 0}, {6, 4, 0}, 
{7, 1, 0}, {7, 6, 0}, {3, 3, 2}, {3, 7, 0}, {5, 4, 0}, {7, 5, 0}, {0, 3, 2}, {1, 3, 0}, {1, 4, 0}, {1, 6, 0}, 
{2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {2, 5, 0}, {3, 2, 0}, {3, 6, 0}, {5, 1, 0}, {5, 6, 0}, {6, 6, 0}, {7, 3, 0},   // +59.133s 59136
{0, 2, 2}, {0, 5, 0}, {0, 6, 0}, {1, 2, 0}, {4, 1, 0}, {4, 2, 0}, {4, 4, 0}, {4, 7, 0}, {5, 2, 0}, {5, 3, 0}, 
{5, 5, 0}, {6, 5, 0}, {7, 2, 0}, {1, 5, 2}, {3, 4, 0}, {7, 6, 0}, {0, 0, 2}, {1, 0, 0}, {1, 4, 0}, {2, 0, 0}, 
{2, 3, 0}, {2, 5, 0}, {2, 6, 0}, {3, 0, 0}, {3, 3, 0}, {3, 7, 0}, {5, 7, 0}, {6, 7, 0}, {7, 0, 0}, {0, 3, 2}, 
{0, 4, 0}, {0, 6, 0}, {0, 7, 0}, {1, 3, 0}, {4, 2, 0}, {4, 3, 0}, {4, 4, 0}, {4, 5, 0}, {5, 1, 0}, {5, 3, 0}, 
{5, 4, 0}, {5, 6, 0}, {6, 6, 0}, {7, 3, 0}, {3, 1, 2}, {3, 5, 0}, {0, 1, 2}, {1, 1, 0}, {1, 5, 0}, {1, 6, 0},   // +59.333s 59328
{2, 0, 0}, {2, 1, 0}, {2, 4, 0}, {2, 6, 0}, {3, 0, 0}, {3, 4, 0}, {5, 0, 0}, {5, 5, 0}, {6, 0, 0}, {7, 1, 0}, 
{0, 0, 2}, {0, 4, 0}, {0, 5, 0}, {0, 7, 0}, {1, 0, 0}, {4, 3, 0}, {4, 5, 0}, {4, 6, 0}, {5, 2, 0}, {5, 4, 0}, 
{5, 7, 0}, {6, 7, 0}, {7, 0, 0}, {7, 5, 0}, {3, 2, 3}, {3, 6, 0}, {0, 2, 2}, {1, 2, 0}, {1, 4, 0}, {1, 6, 0}, 
{1, 7, 0}, {2, 1, 0}, {2, 2, 0}, {2, 4, 0}, {2, 5, 0}, {3, 1, 0}, {3, 5, 0}, {5, 6, 0}, {6, 1, 0}, {7, 1, 0}, 
{7, 2, 0}, {0, 1, 2}, {0, 5, 0}, {0, 6, 0}, {1, 1, 0}, {4, 6, 0}, {4, 7, 0}, {5, 0, 0}, {5, 2, 0}, {5, 3, 0},   // +59.467s 59472
{5, 5, 0}, {6, 0, 0}, {7, 5, 0}, {7, 6, 0}, {1, 5, 2}, {3, 3, 0}, {3, 7, 0}, {0, 2, 2}, {0, 3, 0}, {0, 4, 0}, 
{1, 3, 0}, {1, 4, 0}, {2, 2, 0}, {2, 3, 0}, {2, 5, 0}, {2, 6, 0}, {3, 2, 0}, {3, 6, 0}, {5, 7, 0}, {6, 1, 0}, 
{6, 2, 0}, {7, 2, 0}, {7, 3, 0}, {7, 4, 0}, {0, 6, 2}, {1, 2, 0}, {1, 7, 0}, {4, 2, 0}, {4, 4, 0}, {4, 7, 0}, 
{5, 3, 0}, {5, 4, 0}, {5, 6, 0}, {7, 6, 0}, {1, 6, 2}, {3, 4, 0}, {0, 0, 2}, {0, 3, 0}, {0, 5, 0}, {1, 0, 0}, 
{1, 3, 0}, {1, 5, 0}, {2, 0, 0}, {2, 3, 0}, {2, 4, 0}, {2, 6, 0}, {2, 7, 0}, {3, 0, 0}, {3, 3, 0}, {3, 7, 0},   // +59.633s 59632
{4, 5, 0}, {5, 5, 0}, {6, 2, 0}, {6, 3, 0}, {7, 0, 0}, {7, 3, 0}, {0, 4, 2}, {4, 2, 0}, {4, 3, 0}, {4, 4, 0}, 
{5, 2, 0}, {5, 4, 0}, {5, 7, 0}, {7, 4, 0}, {1, 4, 2}, {3, 1, 0}, {3, 5, 0}, {7, 1, 0}, {0, 0, 2}, {0, 1, 0}, 
{0, 6, 0}, {1, 0, 0}, {1, 1, 0}, {1, 6, 0}, {2, 0, 0}, {2, 1, 0}, {2, 4, 0}, {2, 5, 0}, {3, 0, 0}, {3, 4, 0}, 
{4, 1, 0}, {5, 6, 0}, {6, 3, 0}, {6, 4, 0}, {7, 0, 0}, {0, 5, 2}, {2, 7, 0}, {4, 3, 0}, {4, 5, 0}, {4, 6, 0}, 
{5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {7, 5, 0}, {1, 5, 3}, {2, 2, 0}, {2, 6, 0}, {3, 2, 0}, {3, 6, 0}, {0, 1, 2},   // +59.833s 59840
{0, 2, 0}, {0, 4, 0}, {1, 1, 0}, {1, 2, 0}, {1, 4, 0}, {2, 1, 0}, {2, 5, 0}, {3, 1, 0}, {3, 5, 0}, {5, 1, 0}, 
{5, 7, 0}, {6, 4, 0}, {6, 5, 0}, {7, 1, 0}, {7, 2, 0}, {0, 6, 2}, {4, 1, 0}, {4, 6, 0}, {4, 7, 0}, {5, 3, 0}, 
{5, 4, 0}, {5, 6, 0}, {7, 5, 0}, {7, 6, 0}, {1, 5, 2}, {1, 6, 0}, {2, 3, 0}, {2, 4, 0}, {3, 2, 0}, {3, 3, 0}, 
{3, 6, 0}, {3, 7, 0}, {0, 2, 2}, {0, 3, 0}, {0, 5, 0}, {0, 7, 0}, {1, 2, 0}, {1, 3, 0}, {2, 2, 0}, {2, 6, 0}, 
{5, 5, 0}, {6, 5, 0}, {6, 6, 0}, {7, 2, 0}, {7, 3, 0}, {0, 4, 2}, {4, 2, 0}, {4, 4, 0}, {4, 7, 0}, {5, 1, 0},   // +59.967s 59968
{5, 2, 0}, {5, 4, 0}, {5, 7, 0}, {7, 6, 0}, {1, 4, 2}, {2, 0, 0}, {2, 5, 0}, {3, 0, 0}, {3, 3, 0}, {3, 4, 0}, 
{3, 7, 0}, {0, 0, 2}, {0, 3, 0}, {0, 6, 0}, {1, 0, 0}, {1, 3, 0}, {1, 6, 0}, {2, 3, 0}, {2, 4, 0}, {4, 3, 0}, 
{4, 5, 0}, {5, 0, 0}, {5, 6, 0}, {6, 6, 0}, {6, 7, 0}, {7, 0, 0}, {7, 3, 0}, {0, 5, 2}, {0, 7, 0}, {4, 2, 0}, 
{4, 4, 0}, {5, 2, 0}, {5, 3, 0}, {5, 5, 0}, {1, 5, 2}, {2, 1, 0}, {2, 6, 0}, {3, 1, 0}, {3, 4, 0}, {3, 5, 0}
};

//------------------------------------------------------------------------------
int numReplays(void)
{
    return sizeof(kLampReplay)/sizeof(kLampReplay[0]);
}

//------------------------------------------------------------------------------
uint16_t replay(byte col)
{
    static byte replayLamps[NUM_COL] = {0};
    static uint32_t lastUpdTtag = 0;
    static uint32_t replayPos = 0;
    //static uint32_t numReplays = (sizeof(kLampReplay) / sizeof(AG_LAMP_SWITCH_t));
    static uint16_t currData = 0;
    AG_LAMP_SWITCH_t *pEv = (AG_LAMP_SWITCH_t*)&currData;
    int nr = numReplays();

    // update the lamp matrix
    uint32_t replayTtag = (PINB & B00000100) ?
        (sTtag >> 6) : //(sTtag / (REPLAY_TTAG_SCALE / TTAG_INT_A));
        (sTtag >> 5);  //(sTtag / (REPLAY_TTAG_SCALE / TTAG_INT_B));
    if (lastUpdTtag == 0)
    {
        lastUpdTtag = replayTtag;
    }
    uint32_t dTtag = (replayTtag - lastUpdTtag);
    if (dTtag >= pEv->dttag)
    {
        // handle all events of this ttag
        currData = pgm_read_word_near(kLampReplay + replayPos);
        do
        {
            replayLamps[pEv->col] ^= (1 << pEv->row);
            replayPos++;
            if (replayPos > nr)
            {
                // start over again
                replayPos = 0;
                lastUpdTtag = 0;
                memset(replayLamps, 0, sizeof(replayLamps));
            }
            currData = pgm_read_word_near(kLampReplay + replayPos);
        } while (pEv->dttag == 0);
        lastUpdTtag = replayTtag;
    }

    // return the current row from the replay lamp matrix
    return replayLamps[col];
}

#endif // REPLAY_ENABLED
