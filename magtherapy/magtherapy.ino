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
 V 1.06 setup for program and total time, less times approximation
 V 1.07 better definitions and code cleanup
 
 (C) 2017-2018 Andrea Tuccia GPL V.3
*/

#include <EEPROM.h>
#include <TM1638.h>

// TM1638 Led & Key definitions
#define LED1 _BV(0) // 10 minutes
#define LED2 _BV(1) // 20 minutes
#define LED3 _BV(2) // 30 minutes
#define LED4 _BV(3) // 40 minutes
#define LED5 _BV(4) // 50 minutes
#define LED6 _BV(5) // 60 minutes
#define LED7 _BV(6) // A
#define LED8 _BV(7) // B
#define S1   _BV(0) // F1 = 78.125Hz / -
#define S2   _BV(1) // F2 = 156.25Hz / +
#define S3   _BV(2) // F3 = 312.5Hz
#define S4   _BV(3) // F4 = 625.0Hz
#define S5   _BV(4) // F5 = 1250.0Hz
#define S6   _BV(5) // F6 = 2500.0Hz
#define S7   _BV(6) // Set t1, t2
#define S8   _BV(7) // Start/Stop
#define DIO   8 
#define CLK   9
#define STB  10

TM1638 module(DIO, CLK, STB, 1, 1);

// Globals
//uint8_t  pulse0;
uint32_t pulse1;
uint32_t ticks;
uint16_t seconds;
uint16_t frequency;
uint8_t  program;
uint16_t tmax;
bool     running;
bool     manual;
uint8_t  feed0;
uint8_t  feed1;
bool     ledA;
bool     ledB;

// EEPROM settings
struct settings
{
    uint16_t tmax;
    uint16_t tprog;
};

struct settings mysettings;

// Timer1 presets for divisor, ticks for 600 seconds and frequency
// (not 60 to avoid floating point or approximation)
struct preset
{
    uint16_t counter;
    uint32_t ticks;
    uint16_t frequency;
};

struct preset t1_preset[] = 
{
    { 25600,   46875,   78 }, 
    { 12800,   93750,  156 }, 
    {  6400,  187500,  312 }, 
    {  3200,  375000,  625 }, 
    {  1600,  750000, 1250 }, 
    {   800, 1500000, 2500 }, 
}; 

// Version (w/o decimals)
#define VERSION 107
// Buzzer (Arduino) pin
#define BUZ 11
// Buzzer tone (D7)
#define TONE 2349
// Pulses to be generated (40 pulses @ 400KHz)
#define BURST  (40 * 2 - 1)
// Max programs
#define PMAX  (sizeof(t1_preset) / sizeof(t1_preset[0]) - 1)
// Max total time default (seconds)
#define TMAX_DEFAULT   3600
// Min total time default (seconds)
#define TMIN_DEFAULT     60
// Default single preset time (seconds)
#define TPROG_DEFAULT    60
// Max single preset time (seconds)
#define TPROG_MAX       300
// Min single preset time (seconds)
#define TPROG_MIN        60

#define beep() tone(BUZ, 1000, 50)

/* fast integer MSB of single byte */
uint8_t _msb(uint8_t n)
{
    uint8_t r = 0;

    while (n >>= 1) 
        r++;
    return r;
}

void setup() 
{   
    DDRB |= _BV(PB5);         // PB5 mode output
    DDRD |= _BV(PD6);         // PD6 mode output
    DDRD &= ~_BV(PD2);        // PD2 mode input
    DDRD &= ~_BV(PD3);        // PD3 mode input

    module.setLEDs(255);
    module.setDisplayToString("88888888",255);
    tone(BUZ, TONE, 500);
    _delay_ms(500);

    EEPROM.get(0, mysettings);
    if (mysettings.tmax == 0 || mysettings.tprog == 0)
    {
        mysettings.tmax = TMAX_DEFAULT;
        mysettings.tprog = TPROG_DEFAULT;
        EEPROM.put(0, mysettings);
    }
    
    cli();

    EIMSK |= _BV(INT0)  |  _BV(INT1);    // Enable external interrupt INT0 & INT1
    EICRA |= _BV(ISC00) | _BV(ISC10);    // Trigger INT0 & INT1 on any logical change 

    TCCR0A = 0;
    TCCR0B = 0;
    OCR0A = 40;   // 16MHz / 1 / 40 = 400KHz
    TIMSK0 |= _BV(OCIE0A);               // enable timer compare interrupt

    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 |= _BV(OCIE1A);               // enable timer compare interrupt

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
    if (pulse1 & 64) // debug
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
    if (seconds > tmax) 
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
    }
//    ticks = (uint32_t) t1_preset[program].ticks;
    ticks = (t1_preset[program].ticks * mysettings.tprog / 600);
    frequency = t1_preset[program].frequency;
    TCCR1A = 0;
    TCCR1B = 0;
    OCR1A = t1_preset[program].counter;
    TCCR1B |= _BV(WGM12) | _BV(CS11);
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

void change(int8_t _program = 0)
{
    if (!running) 
        return;
    cli();
    program = _program;
    //ticks = (uint32_t) t1_preset[program].ticks;
    tmax = mysettings.tmax - 1;
    ticks = (t1_preset[program].ticks * mysettings.tprog / 600);
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
        return;
    cli();
    manual = !automatic;
    program = _program;
//    pulse0 = 0;
    pulse1 = 0;
    seconds = 0;
    //ticks = (uint32_t) t1_preset[program].ticks;
    tmax = mysettings.tmax - 1;
    ticks = (t1_preset[program].ticks * mysettings.tprog / 600);
    frequency = t1_preset[program].frequency;
    TCCR1A = 0;
    TCCR1B = 0;
    OCR1A = t1_preset[program].counter;
    TCCR1B |= _BV(WGM12) | _BV(CS11);
    running = 1;
    sei();
}

void settingmenu()
{
    byte _min, _sec, setting = 1;
    char text[8];
    uint16_t _tprog = mysettings.tprog;
    uint16_t _tmax = mysettings.tmax;
    byte last = module.getButtons();
    while (setting)
    {
        volatile byte keys = module.getButtons();

        switch (setting)
        {
            case 1:
                switch (keys) 
                {
                    case S1:
                        if (keys != last && _tprog > TPROG_MIN)
                            beep();
                        if (_tprog > TPROG_MIN)
                            _tprog--;
                        break;
                    case S2:
                        if (keys != last && _tprog < TPROG_MAX)
                            beep();
                        if (_tprog < TPROG_MAX)
                            _tprog++;
                        break;
                    case S7:
                        if (keys != last)
                        {
                            beep();
                            setting = 2;
                        }
                        break;
                }
                last = keys;
                _sec = _tprog % 60;
                _min = _tprog / 60;
                sprintf(text, "T%1i  %02d%02d", 1, _min, _sec);
                break;
            case 2:
                switch (keys) 
                {
                    case S1:
                        if (keys != last && _tmax > TMIN_DEFAULT)
                            beep();
                        if (_tmax > TMIN_DEFAULT)
                            _tmax--;
                        break;
                    case S2:
                        if (keys != last && _tmax < TMAX_DEFAULT)
                            beep();
                        if (_tmax < TMAX_DEFAULT)
                            _tmax++;
                        break;
                    case S7:
                        if (keys != last)
                        {
                            beep();
                            setting = 0;
                        }
                        break;
                }
                last = keys;
                _sec = _tmax % 60;
                _min = _tmax / 60;
                sprintf(text, "T%1i  %02d%02d", 2,  _min, _sec);
                break;
        }
        module.setLEDs(0);
        module.setDisplayToString(text, _BV(2));
        _delay_ms(100);
    }
    mysettings.tprog = _tprog;
    mysettings.tmax = _tmax;
    EEPROM.put(0, mysettings);
}

void loop() 
{
    static byte last;
    static byte tenminutes;
    char text[8];
    
    byte keys = module.getButtons();
    if (keys && !last) 
    {
        if (!running)
        {
            switch (keys) 
            {
                case S1:
                case S2:
                case S3:
                case S4:
                case S5:
                case S6:
                    beep();
                    start(_msb(keys), false);
                    break;
                case S7:
                    beep();
                    settingmenu();
                    break;
                case S8: 
                    beep();
                    start();
                    break;
            }
        } else {
            switch (keys) 
            {
                case S1:
                case S2:
                case S3:
                case S4:
                case S5:
                case S6:
                    if (!manual) 
                        break;
                    beep();
                    change(_msb(keys));
                    break;
                case S8: 
                    beep();
                    stop();
                    break;
            }
        }
    }
    last = keys;
    if (running) 
    {
        tenminutes = seconds / 600;
        uint8_t leds = _BV(tenminutes);
        leds |= (ledA) ? LED7 : 0;
        leds |= (ledB) ? LED8 : 0;
        ledA = false;
        ledB = false;
        sprintf(text, "P%i F%4i", program + 1, frequency);
        //sprintf(text, "P%i F%4i", program + 1, seconds); //debug
        module.setLEDs(leds);
        module.setDisplayToString(text);
    } else {
        sprintf(text, " MAG%3i ", VERSION);
        module.setLEDs(0);
        module.setDisplayToString(text, _BV(3));
    }
}

