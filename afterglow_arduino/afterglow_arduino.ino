/***********************************************************************
 *  afterglow:
 *      Copyright (c) 2018 Christoph Schmid
 *
 ***********************************************************************
 *  This file is part of the afterglow pinball LED project:
 *  https://github.com/smyp/afterglow
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
/* This script assumes following pin layout:
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
 *  | LED      | LED           | D8        | DDRB, 1       | Output       |
 *  | OE       | 74LS595 OE    | A1        | DDRC, 1       | Output       |
 *  | CFG0     | DIP CFG 0     | D10       | DDRB, 3       | Input Pullup |
 *  | CFG1     | DIP CFG 1     | D11       | DDRB, 4       | Input Pullup |
 *  | CFG2     | DIP CFG 2     | D12       | DDRB, 5       | Input Pullup |
 *  | CFG3**   | DIP CFG 3     | D13       | DDRB, 6       | Input Pullup |
 *  +----------+---------------+-----------+---------------+--------------+

** CFG3 not working on boards revision <= 1.1 as it's connected to D13 (LED) 

*/

//------------------------------------------------------------------------------
// Setup

// turn debug output via serial on/off
#define DEBUG_SERIAL 0

// Number of consistent data samples required for matrix update
#define SINGLE_UPDATE_CONS 2

// local time interval (us)
#define TTAG_INT (250)

// original matrix update interval (us)
#define ORIG_INT (2000)

// cycles per original interval
#define ORIG_CYCLES (ORIG_INT / TTAG_INT)

// number of columns in the lamp matrix
#define NUM_COL 8

// number of rows in the lamp matrix
#define NUM_ROW 8

// afterglow duration step size [ms]
// glow duration = glowCfg * GLOWDUR_STEP
#define GLOWDUR_STEP (75)

// afterglow LED glow duration [ms]
#define AFTERGLOW_LED_DUR (2000)

// test mode lamp switch interval [ms]
#define TESTMODE_INT (500)

// number of cycles per testmode interval
#define TESTMODE_CYCLES ((uint32_t)TESTMODE_INT * 1000UL / (uint32_t)TTAG_INT)


//------------------------------------------------------------------------------
// global variables

// Lamp matrix 'memory'
static uint16_t sMatrixState[NUM_COL][NUM_ROW];
    
// local time
static uint32_t sTtag = 0;

// maximum interrupt runtime counter [cycles]
static uint16_t sMaxIntTime = 0;

// remember the last column and row samples
static byte sLastColMask = 0;
static byte sLastRowMask = 0;

#if DEBUG_SERIAL
static byte sLastOutColMask = 0;
static byte sLastOutRowMask = 0;
static uint32_t sBadColCounter = 0;
static byte sLastBadCol = 0;
static byte sLastGoodCol = 0;
#endif

// Lamp matrix configuration
// Bit 0-7=column 1, row 1-8
// 0=LED, 1=incandescent
static uint64_t sLampCfg = 0;


//------------------------------------------------------------------------------
void setup()
{
    // Use Timer1 to create an interrupt every TTAG_INT us.
    // This will be the heartbeat of our realtime task.
    noInterrupts(); // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    // set compare match register for TTAG_INT us increments
    // prescaler is at 1, so counting real clock cycles
    OCR1A = (TTAG_INT * 16);  // [16MHz clock cycles]
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS10 bit so timer runs at clock speed
    TCCR1B |= (1 << CS10);  
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);

    // I/O pin setup
    // 74LS165 LOAD and CLK are output, DATA is input
    // 74HC595 LOAD, CLK and DATA are output
    DDRD = B11111001;
    // LED output on pin 8, configuration input on pins 10-13
    DDRB = B00000001;
    // activate the pullups for the configuration pins
    PORTB |= B00011100;
    // OE on A1, DBG on A2
    DDRC = B00000110;
    // turn the LED on, keep OE high
    PORTC |= B00000011;

    // initialize the data
    memset(sMatrixState, 0, sizeof(sMatrixState));

    // enable all interrupts
    interrupts();

#if DEBUG_SERIAL
    // enable serial output at 115200 baudrate
    Serial.begin(115200);
    Serial.println("afterglow 1.1 (c) 2018 morbid cornflakes");
#endif
}

//------------------------------------------------------------------------------
// Timer1 interrupt handler
// This is the realtime task heartbeat. All the magic happens here.
ISR(TIMER1_COMPA_vect)
{
    // time is running
    uint16_t startCnt = TCNT1;
    sTtag++;

    // Drive the lamp matrix
    // This is done before updating the matrix to avoid having an irregular update
    // frequency due to varying update calculation times.
    driveLampMatrix();

    // 74HC165 16bit sampling
    uint16_t inData = sampleInput();
    bool validInput = true;

    // testmode input simulation (CFG3 active)
    if ((PINB & B00000100) == 0)
    {
        // test mode
        inData = testModeInput();
    }
    byte inColMask = (inData >> 8); // LSB is col 0, MSB is col 7
    byte inRowMask = ~(byte)inData; // high means OFF, LSB is row 0, MSB is row 7

    // evaluate the column reading
    // only one bit should be set as only one column can be active at a time
    uint32_t inCol = NUM_COL;
    switch (inColMask)
    {
        case 0x01: inCol = 0; break;
        case 0x02: inCol = 1; break;
        case 0x04: inCol = 2; break;
        case 0x08: inCol = 3; break;
        case 0x10: inCol = 4; break;
        case 0x20: inCol = 5; break;
        case 0x40: inCol = 6; break;
        case 0x80: inCol = 7; break;
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
#if DEBUG_SERIAL
            sBadColCounter++;
            sLastBadCol = inColMask;
#endif
            validInput = false;
        }
        break;
    }

    // The matrix is updated only once per original column cycle. The code
    // waits for a number of consecutive consistent information before updating the matrix.
    validInput &= updateValid(inColMask, inRowMask);

    // Update only with a valid input. If the input is invalid the current
    // matrix state is left unchanged.
    if (validInput)
    {
        // update the current column
        updateCol(inCol, inRowMask);

#if DEBUG_SERIAL
        sLastGoodCol = inCol;
#endif
    }

    // remember the last column and row samples
    sLastColMask = inColMask;
    sLastRowMask = inRowMask;

    // update the funky afterglow LED
    afterglowLED(sTtag);

    // how long did it take?
    uint16_t dt = (TCNT1 - startCnt);
    if (dt > sMaxIntTime)
    {
        sMaxIntTime = dt;
    }
}

//------------------------------------------------------------------------------
void loop()
{
    // debug output only - wait for interrupts
    // all the fun stuff happens in the timer interrupt

#if DEBUG_SERIAL
    // print the maximum interrupt runtime
    if ((PINB & B00000100) == 0)
    {
        Serial.println("TESTMODE!");
    }
    Serial.print("DUR ");
    Serial.println((PINB & B00011000) >> 3);
    Serial.print("INT dt max ");
    Serial.print(sMaxIntTime / 16);
    Serial.println("us");
    Serial.print("Bad col: # ");
    Serial.print(sBadColCounter);
    Serial.print(" col ");
    Serial.print(sLastBadCol);
    Serial.print(" last good: ");
    Serial.println(sLastGoodCol);
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
    delay(5000);
#endif
}

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
void updateCol(uint32_t col, byte rowMask)
{
    // paranoia check
    if (col >= NUM_COL)
    {
        return;
    }
    
    // get a pointer to the matrix column
    uint16_t *pMx = &sMatrixState[col][0];

    // evaluate the glow configuration (CFG0-1)
    // brightness filter from dark to full [ms]
    static byte sLastGlowCfg = 0xff;
    uint32_t glowCfg = ((~PINB & B00011000) >> 3); // invert as a closed switch means 0

    // Update the glow step when the configuration changes only as this is an
    // expensive calculation
    static uint16_t glowStep = 0xffff;
    if (glowCfg != sLastGlowCfg)
    {
        uint32_t glowDur = (glowCfg * GLOWDUR_STEP);

        // brightness step per lamp matrix update (assumes one update per original matrix step)
        glowStep = (glowDur > 0) ?
            ((uint16_t)((uint32_t)65536 / (glowDur * 1000 / (uint32_t)ORIG_INT)) * NUM_COL) : 0xffff;
        sLastGlowCfg = glowCfg;
    }

    // get the row configuration
    byte rowCfg = (byte)(sLampCfg >> (col * NUM_ROW));

    // update all row values
    for (uint32_t r=0; r<NUM_ROW; r++)
    {
        // determine the step size based on configuration
        // todo: make a lookup table for this
        uint16_t step;
        if ((rowCfg & (1 << r)) == 0)
        {
            // LEDs glow :-)
            step = glowStep;
        }
        else
        {
            // incandescents turn on immediately
            step = 0xffff;
        }
        
        // update the matrix value
        updateMx(pMx, (rowMask & 0x01), step);

        // next row
        pMx++;
        rowMask >>= 1;
    }
}

//------------------------------------------------------------------------------
uint16_t sampleInput(void)
{
    // drive CLK and LOAD low
    PORTD &= B11100111;
    
    // wait some time
    uint16_t data = 0;
    data+= 17;
    data-= 3;
    
    // drive LOAD high to save pin states
    PORTD |= B00010000;
    
    // clock in all data
    for (byte i=0; i<16; i++)
    {
        data <<= 1;                        // make way for the new bit
        PORTD &= B11110111;                // CLK low
        data |= ((PIND & B00000100) >> 2); // read data bit
        PORTD |= B00001000;                // CLK high
    }
    return data;
}

//------------------------------------------------------------------------------
void driveLampMatrix()
{   
    // turn off everything briefly to avoid ghosting
    // the scope says this takes ~20us at 16MHz
    dataOutput(0x00, 0x00);

    // check which column we're currently updating
    uint32_t outCol = (sTtag % NUM_COL);

    // The original cycle is divided into ORIG_CYCLES column sub cycles.
    // These cycles are used to do PWM in order to adjust the lamp brightness.
    //
    // Illustration with ORIG_CYCLES==4 and four brightness steps B1-B4 and off (B0):
    //
    // * = Lamp on
    //                       2ms
    //        +-------------------------------+
    //        |                               |        Original lamp duty cycle
    //        |                               |
    //  ------+         500us                 +-------
    //        ^       ^       ^       ^       ^        Afterglow duty cycles
    //  B0                                             Lamp matrix value == 0
    //  B1    ********                                 Lamp matrix value < 16384
    //  B2    ****************                         Lamp matrix value < 32768
    //  B3    ************************                 Lamp matrix value < 49152
    //  B4    ********************************         Lamp matrix value < 65536
    uint32_t colCycle = (sTtag / NUM_COL) % ORIG_CYCLES;

    // prepare the data
    // LSB is row/col 0, MSB is row/col 7
    byte colData = (1 << outCol);
    byte rowData = 0;
    uint16_t *pMx = &sMatrixState[outCol][0];
    for (uint32_t r=0; r<NUM_ROW; r++)
    {
        // make room for the next bit
        rowData >>= 1;
        
        // LEDs are turned on when the value in the matrix is not zero
        // and when the value is high enough for the current sub cycle.
        if ((*pMx) &&
            ((*pMx / (65536 / ORIG_CYCLES)) >= colCycle))
        {
            rowData |= 0x80;
        }
        pMx++;
    }

    // output the data
    dataOutput(colData, rowData);
#if DEBUG_SERIAL
    sLastOutColMask = colData;
    sLastOutRowMask = rowData;
#endif
}

//------------------------------------------------------------------------------
void dataOutput(byte colData, byte rowData)
{
    // This writes the 16bit column and row data to the two 74595 shift registers
    
    // pull RCLK (OUT_LOAD) and CLK low to start sending data
    PORTD &= B00111111;

    // prepare the data
    uint16_t data = ((rowData << 8) | colData);
    
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

    // Enable by pulling OE low.
    // This is only done here to ensure that the LEDs are not turned on before
    // the columns are duty cycled.
    PORTC &= B11111101;
}
//------------------------------------------------------------------------------
void afterglowLED(uint32_t ttag)
{
    static const uint32_t kDutyCycleMin = 0;
    static const uint32_t kDutyCycleMax = 48;
    static uint32_t dttag = 0;
    static bool positive = true;
    static uint32_t dutyCycle = kDutyCycleMin;
    static uint32_t dutyCyleDTtag = 0;
    static const uint32_t dutyCycleUpdateInt = ((uint32_t)AFTERGLOW_LED_DUR * 1000 / (kDutyCycleMax - kDutyCycleMin) / (uint32_t)TTAG_INT);

    // update the counters
    dttag++;
    dutyCyleDTtag++;

    // update the duty cycle
    if (dutyCyleDTtag >= dutyCycleUpdateInt)
    {
        if (positive)
        {
            dutyCycle++;
            // start dimming
            if (dutyCycle > kDutyCycleMax)
            {
                dutyCycle = (kDutyCycleMax - 1);
                positive = false;
            }
        }
        else
        {
            if (dutyCycle > kDutyCycleMin)
            {
                dutyCycle--;
            }
            else
            {
                dutyCycle = (kDutyCycleMin + 1);
                positive = true;
            }
        }
        dutyCyleDTtag = 0;   
    }

    // update the LED
    if (dttag >= dutyCycle)
    {
        // turn the LED on
        PORTB |= B00000001;
        dttag = 0;
    }
    else
    {
        // turn the LED off
        PORTB &= B11111110;
    }
}

//------------------------------------------------------------------------------
uint16_t testModeInput(void)
{
    // simulate the original column cycle
    byte col = ((sTtag / ORIG_CYCLES) % NUM_COL);
    byte colMask = (1 << col);

    // simulate one active lamp at a time, cycling through the full matrix
    byte rowMask = 0;
    byte lampIx = (byte)((sTtag / TESTMODE_CYCLES) % 64);
    if ((lampIx / NUM_COL) == col)
    {
        rowMask = (1 << (lampIx % NUM_COL));
    }

    // invert the row mask as in the original input HIGH means off
    rowMask = ~rowMask;
    
    return ((colMask << 8) | rowMask);
}

//------------------------------------------------------------------------------
bool updateValid(byte inColMask, byte inRowMask)
{
    static byte sConsistentSamples = 0;
    static byte sLastUpdColMask = 0x00;
    bool valid = false;

    // check if the current column has not been handled already
    if (inColMask != sLastUpdColMask)
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
            sLastUpdColMask = inColMask;
            valid = true;
        }
    }
    return valid;
}

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

