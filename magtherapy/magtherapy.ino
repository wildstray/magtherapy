/*
 Magnetoterapia

 Timer0 used to burst 40 pulses (100uS) at 400KHz (2.5uS)
 Timer1 used to cycle programs and restart Timer0

 V 1.00 initial working version (only start/stop auto mode)
 V 1.01 manual mode, start with selected frequency
 V 1.02 INT0/INT1 visual feedback on LEDs A,B
 V 1.03 audible feedbacks (buzzer)
 V 1.04 better buttons management
 V 1.05 manual mode change frequency while running
 TDB: better 1 second time counting / timer
 TDB: setup and user programmable timers
 TDB: cleanup (eg. timers initialization)

 (C) 2017-2018 Andrea Tuccia GPL V.3
*/

#include <EEPROM.h>
#include <TM1638.h>

TM1638 module(8, 9, 10, 1, 1);

// Globals
//uint8_t  pulse0;
uint32_t pulse1;
uint32_t ticks;
uint16_t seconds;
uint16_t frequency;
uint8_t  program;
//uint8_t  cycles;
uint8_t  time;
bool     running;
bool     manual;
uint8_t  feed0;
uint8_t  feed1;
bool     ledA;
bool     ledB;

// Timer1 presets for divisor, cycles for 120 seconds and frequency
struct preset
{
    uint16_t counter;
    uint32_t ticks;
    uint16_t frequency;
};

struct preset t1_preset[] = 
{
    { 25600,   938,   78 }, 
    { 12800,  1875,  156 }, 
    {  6400,  3750,  312 }, 
    {  3200,  7500,  625 }, 
    {  1600, 15000, 1250 }, 
    {   800, 30000, 2500 }, 
}; 

// Version (w/o decimals)
#define VERSION 105
// Buzzer (Arduino) pin
#define BUZ 11
// Buzzer tone (D7)
#define TONE 2349
// Pulses to be generated (40 pulses @ 400KHz)
#define BURST  (40 * 2 - 1)
// Max programs
#define PMAX  (sizeof(t1_preset) / sizeof(t1_preset[0]) - 1)
// Max cycles of programs
//#define CMAX  (5 - 1)
// Max total time (seconds)
#define TMAX  (3600 - 1)
#define TMAX  (36 - 1)

void setup() 
{
    // Set up the built-in LED pin as an output:
    DDRB |= _BV(PB5);         // PB5 mode output
    DDRD |= _BV(PD6);         // PD6 mode output
    DDRD &= ~_BV(PD2);        // PD2 mode input
    DDRD &= ~_BV(PD3);        // PD3 mode input

    module.setLEDs(255);
    module.setDisplayToString("88888888",255);
    tone(BUZ, TONE, 500);
    delay(500);
    
    cli();

    EIMSK |= _BV(INT0)  |  _BV(INT1);     // Enable external interrupt INT0 & INT1
    EICRA |= _BV(ISC00) | _BV(ISC10);    // Trigger INT0 & INT1 on any logical change 

    TCCR0A = 0;
    TCCR0B = 0;
    OCR0A = 40;   // 16MHz / 1 / 40 = 400KHz
    TIMSK0 |= _BV(OCIE0A);  // enable timer compare interrupt

    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 |= _BV(OCIE1A);

    sei();

}

// T0 interrupt routine: generate one burst on OC0A one shot
ISR(TIMER0_COMPA_vect)
{
    static byte pulse0 = 0;
    pulse0++;
    if (pulse0 > BURST)
    {
        pulse0 = 0;
        if (feed0 > BURST) {
            ledA = true;
        }
        if (feed1 > BURST) {
            ledB = true;
        }
        feed0 = 0;
        feed1 = 0;
        TCCR0A = 0;
        TCCR0B = 0;
    }
}

// T1 interrupt routine: restart T0 
ISR(TIMER1_COMPA_vect)
{
//    static byte pulse1 = 0;
    if (!running) 
        return;
    TCCR0A |= _BV(WGM01) | _BV(COM0A0);
    TCCR0B |= _BV(CS00);
    pulse1++;
    if (pulse1 & 64)
    {
        PORTB ^= _BV(PB5);
    }
    if ((pulse1 % frequency) == 0)
    {
        timestep();
    }
    if (pulse1 > ticks)
    {
        pulse1 = 0;
        progstep();
    }
}

ISR(INT0_vect)
{
    feed0++;
}

ISR(INT1_vect)
{
    feed1++;
}

void timestep()
{
    seconds++;
    if (seconds > TMAX) 
    {
        stop();
        tone(BUZ, TONE, 1500);
    }
}

void progstep()
{
    if (manual) 
        return;
    program++;
    if (program > PMAX)
    {
        program = 0;
//        cycles++;
    }
    ticks = t1_preset[program].ticks;
    frequency = t1_preset[program].frequency;
    TCCR1B = 0;
    OCR1A = t1_preset[program].counter;
    TCCR1B |= _BV(WGM12) | _BV(CS11);
//    if (cycles > CMAX) 
//    {
//        stop();
//        tone(BUZ, TONE, 1500);
//    }
}

void stop()
{
    if (!running) 
        return;
    cli();
    TCCR1A = 0;
    TCCR1B = 0;
    PORTB &= ~_BV(PB5);
    running = 0;
    sei();
}

void _change(int8_t _program = 0)
{
    if (!manual) 
        return;
    cli();
    program = _program;
    ticks = t1_preset[program].ticks;
    frequency = t1_preset[program].frequency;
    TCCR1A = 0;
    TCCR1B = 0;
    OCR1A = t1_preset[program].counter;
    TCCR1B |= _BV(WGM12) | _BV(CS11);
    sei();
}

void start(int8_t _program = 0, bool automatic = true)
{
    if (running) 
    {
        _change(_program);
        return;
    }
    cli();
    manual = !automatic;
    program = _program;
//    pulse0 = 0;
    pulse1 = 0;
//    cycles = 0;
    seconds = 0;
    ticks = t1_preset[program].ticks;
    frequency = t1_preset[program].frequency;
    TCCR1A = 0;
    TCCR1B = 0;
    OCR1A = t1_preset[program].counter;
    TCCR1B |= _BV(WGM12) | _BV(CS11);
    running = 1;
    sei();
}

void loop() 
{
    static byte last;
    static byte tenminutes;
    char text[8];
    
    byte keys = module.getButtons();
    if (keys && !last) 
    {
        tone(BUZ, 1000, 50);
        switch (keys) 
        {
            case _BV(0):
                start(0, false);
                break;
            case _BV(1):
                start(1, false);
                break;
            case _BV(2):
                start(2, false);
                break;
            case _BV(3):
                start(3, false);
                break;
            case _BV(4):
                start(4, false);
                break;
            case _BV(5):
                start(5, false);
                break;
            case _BV(6):
                break;
            case _BV(7): 
                tone(BUZ, 1000, 50);
                if (running) 
                {
                    stop();
                } else {
                    start();
                }
                break;
        }
    }
    last = keys;
    if (running) 
    {
        tenminutes = seconds / 60;
        uint8_t leds = _BV(tenminutes);
        leds |= (ledA) ? _BV(6) : 0;
        leds |= (ledB) ? _BV(7) : 0;
        module.setLEDs(leds);
        ledA = false;
        ledB = false;
        sprintf(text, "P%i F%4i", program + 1, frequency);
        sprintf(text, "P%i F%4i", program + 1, seconds); // debug
        module.setDisplayToString(text);
    } else {
        module.setLEDs(0);
        sprintf(text, "MAG %3i ", VERSION);
        //sprintf(text, "TIME %3i ", VERSION);
        module.setDisplayToString(text, _BV(3));
    }
}

