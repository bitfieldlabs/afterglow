//------------------------------------------------------------------------------
// Setup

// local time interval (us)
#define TTAG_INT (500)

// original matrix update interval (us)
#define ORIG_INT (2000)

// cycles per original interval
#define ORIG_CYCLES (ORIG_INT / TTAG_INT)

// brightness filter from dark to full (ms)
#define GLOW_DUR (200)

// brightness step per local time step
#define GLOW_STEP ((uint16_t)((uint32_t)65536 / (((uint32_t)GLOW_DUR * 1000) / (uint32_t)TTAG_INT)))


//------------------------------------------------------------------------------
// global variables

// Lamp matrix 'memory'
uint16_t sMatrixState[8][8] = {0};
    
// local time
uint32_t sTtag = 0;


//------------------------------------------------------------------------------
void setup()
{
    // Use Timer1 to create an interrupt every 500us.
    // This will be the heartbeat of our realtime task.
    noInterrupts(); // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    // set compare match register for 500us increments
    // prescaler is at 1, so counting real clock cycles
    OCR1A = (TTAG_INT * 16);  // [16MHz clock cycles]
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS10 bit so timer runs at clock speed
    TCCR1B |= (1 << CS10);  
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    interrupts();   // enable all interrupts

    // I/O pin setup
    DDRC |= 0x03;   // output
    DDRD = 0x00;    // input
    DDRB = 0x00;    // input
}

//------------------------------------------------------------------------------
// Timer1 interrupt handler
ISR(TIMER1_COMPA_vect)
{   
    // time is running
    sTtag++;
 
    // sample the columns, digital pins 2-9

    // sample the rows, digital pins 10-

    // update the matrix
    uint16_t *pMx = &sMatrixState[0][0];
    bool on = (!(PIND & 0x04) && (PINB & 0x04)) ? true : false;
    updateMx(pMx, on);

    // drive the lamp matrix
    uint32_t col = (sTtag % 8);    // current colum
    uint32_t colCycle = (sTtag / 8) % ORIG_CYCLES;
    uint16_t v = sMatrixState[0][0];
    bool turnOn = (v && ((v / (65536 / ORIG_CYCLES)) >= colCycle));
    if ((col == 0) && (turnOn))
    {
        PORTC |= 0x02;
    }
    else
    {
        PORTC &= ~0x02;
    }

    // demo mode: change lamp every second
    uint32_t ttagS = sTtag / (1000000 / TTAG_INT);
    bool ledOn = (ttagS % 2);
    if (ledOn)
    {
        PORTC |= 0x01;
    }
    else
    {
        PORTC &= ~0x01;
    }

    // trigger the row interrupt
    
}

//------------------------------------------------------------------------------
void loop()
{
    // do nothing at all - wait for interrupts
}

//------------------------------------------------------------------------------
void setDataPins(uint8_t v)
{
    
}

//------------------------------------------------------------------------------
void updateMx(uint16_t *pMx, bool on)
{
    if (on)
    {
        if (*pMx < (0xffff - GLOW_STEP))
        {
            *pMx += GLOW_STEP;
        }
        else
        {
            *pMx = 0xffff;
        }
    }
    else
    {
        if (*pMx > GLOW_STEP)
        {
            *pMx -= GLOW_STEP;
        }
        else
        {
            *pMx = 0;
        }
    }
}

