#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h> 
#include <stdbool.h> 
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/eeprom.h> 

#define CHARGE_PIN      PB3
#define ADMUX_SENSE_PIN   (1<<MUX1);

#define AVERAGE_BUFFER_LENGTH  10
#define POSITIVE_READINGS_TRESHOLD  5
#define POSITIVE_READINGS_HYSTERESIS  3

#define ADC_READINGS_MIN_DELTA     4

static uint8_t currentIdleValue = 0x00;
static uint8_t positiveReadings = 0x00;

static bool touchSensed = false;
static bool latchValue = true;

typedef enum {
    ModePulse,
    ModePulseLong,
    ModeSelfLatch
} OperationMode;

typedef enum {
    TouchEventIdle,
    TouchEventPulse,
    TouchEventContinuous,    
    TouchEventDiscard    
} TouchEvent;

static TouchEvent currentEvent = TouchEventIdle;
static OperationMode currentMode = ModePulse;

static void each_100ms();

static void init_ports() {
    DDRB = 0x00;
    PORTB = 0x00;

    //debug LED
    DDRB |= (1<<PB0); //debug pin
    DDRB |= (1<<PB3); //charge pin

    //ADC setup
    ADMUX = 0x00;
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1);

    DDRB &= ~(1<<PB1);
    DDRB &= ~(1<<PB2);    
}

static uint8_t virtual_timer = 0x00;

static void init_timer() {
    TCCR0A = 0x00; //normal mode
}

static void init_mode() {
    uint8_t mode = 0x00;

    if ( PINB & (1<<PB1) ) {
        mode |= 0b01;
    }

    if ( PINB & (1<<PB2) ) { 
        mode |= 0b10;
    }

    switch ( mode ) {
        case 0b11:
        case 0b00:
            currentMode = ModePulse;
            break;
        case 0b01:
            currentMode = ModePulseLong;

            break;
        case 0b10:
            currentMode = ModeSelfLatch;
            break;
    }
}

static void start_timer() {
    TCNT0 = 0x00; //reset timer
    virtual_timer = 0x00;

    TIFR0 &= ~_BV(OCF0A); //reset flag    
    TIMSK0 = _BV(OCIE0A); //overflow interrupt enabled    

    TCCR0B = _BV(CS02) | _BV(CS00); //start with 1024 prescaler
} 

static void stop_timer() {
    TCNT0 = 0x00;
    virtual_timer = 0x00;
    TCCR0B = 0x00; //turn off prescaler
    TIMSK0 = 0x00; //dissable interrupts
}

ISR(TIM0_COMPA_vect) {
    if ( ++virtual_timer >= F_CPU/1024/255/10 ) {
        each_100ms();
        virtual_timer = 0;
    }
}

static void setOutput(bool output) {
    bool res = output;

    if ( currentMode == ModeSelfLatch ) {
        if ( output ) {
            latchValue = !latchValue;
        }
        res = latchValue;        
    } 
    else  {
        res = output;
    }

    if ( res ) {
        PORTB |= (1<<PB0);
    } else {
        PORTB &= ~(1<<PB0);
    }
}

static inline uint16_t adc_get(void){
    ADCSRA |= (1<<ADSC); //start conversion
    while(!(ADCSRA & (1<<ADIF))); //wait for conversion to finish
    ADCSRA |= (1<<ADIF);
    return ADCW;
}

static uint8_t read_pin() {
    uint16_t low, high;

    //charge up    
    {
        DDRB |= (1<<CHARGE_PIN); 
        PORTB |= (1<<CHARGE_PIN);
        _delay_us(5); 

        //read high value
        ADMUX = 0b00000010;
        high = adc_get();
    }

    //enter high-z state, disable pull-up
    {
        DDRB &= ~(1<<CHARGE_PIN);
        PORTB &= ~(1<<CHARGE_PIN);

        //wait for discharge a bit
        _delay_us(5);

        //read low value
        ADMUX = 0b00000010;
        low = adc_get();
    }

    //return the difference
    uint16_t diff = high-low;

    if ( diff > 0xFF ) {
        return 0xFF;
    } else {
        return diff & 0xFF;
    }
}

static uint8_t read_pin_times(uint8_t times) {
    uint16_t retValue = 0x00;

       for ( int i=0; i<times; i++ ) {
        retValue += read_pin();
    }

    return (uint8_t)(retValue/times); 
}

static uint8_t average_readings(uint8_t v) {
    static uint8_t idle_average_readings[AVERAGE_BUFFER_LENGTH] = { 0x00 };    
    static uint8_t cnt = 0;

    if ( ++cnt >= AVERAGE_BUFFER_LENGTH ) {
        cnt = 0;
    }

    idle_average_readings[cnt] = v;
    
    uint16_t average = 0;

    for (uint8_t i=0; i<AVERAGE_BUFFER_LENGTH; i++ ) {
        average += idle_average_readings[i];
    }

    return (uint8_t)(average/AVERAGE_BUFFER_LENGTH);
}

static void each_100ms() {
    switch( currentEvent ) {
        case TouchEventPulse:
            if ( touchSensed && currentMode == ModePulseLong ) {
                currentEvent = TouchEventContinuous;
            } else {
                setOutput(false);
                currentEvent = TouchEventDiscard;
            }

            break;

        case TouchEventDiscard:
            if ( touchSensed && (currentMode == ModePulse || currentMode == ModeSelfLatch) ) {
                currentEvent = TouchEventDiscard;
            }
            else {
                currentEvent = TouchEventIdle;
            }

            break;

        case TouchEventIdle:
        case TouchEventContinuous: 
        default:            
            break;
    }

    if ( currentEvent != TouchEventDiscard ) {
        stop_timer(); 
    }   
}

static void slowly_fill_idle_buffer() {
    for ( uint8_t i=0; i<AVERAGE_BUFFER_LENGTH; i++ ) {
        currentIdleValue = average_readings(read_pin());
        _delay_ms(30);
        wdt_reset();
    }
}

int main() {
    MCUSR &= ~_BV(WDRF); //clear watchdog reset flag     
    wdt_disable(); //disable watchdog

    cli(); {
        //init PORTs, DDRs and PINs
        init_ports();
        init_timer();
        init_mode();
    }; sei();

    wdt_enable(WDTO_8S); //restore watchdog timer

    slowly_fill_idle_buffer();

    //main loop
    while(1) {
        wdt_reset();

        uint8_t currentValue = read_pin_times(2);
        uint8_t delta = abs(currentIdleValue - currentValue);

        if ( delta >= ADC_READINGS_MIN_DELTA ) {
            if ( positiveReadings > POSITIVE_READINGS_TRESHOLD + POSITIVE_READINGS_HYSTERESIS ) {
                touchSensed = true;

                if ( currentEvent == TouchEventIdle ) {
                    currentEvent = TouchEventPulse;
                    setOutput(true);

                    start_timer();                    
                }
            }

            if ( positiveReadings < POSITIVE_READINGS_TRESHOLD*3 ) {
                positiveReadings++;
            }

        } else {
            if ( positiveReadings < POSITIVE_READINGS_TRESHOLD - POSITIVE_READINGS_HYSTERESIS ) {
                touchSensed = false;

                if ( currentEvent == TouchEventContinuous ) {
                    setOutput(false);
                    currentEvent = TouchEventDiscard;

                    start_timer();
                }
            }

            if ( positiveReadings > 0 ) {
                positiveReadings--;
            } else if ( currentEvent == TouchEventIdle ) {
                static uint8_t slow_average = 0x00;
                if ( ++slow_average == 0x0F ) {
                    currentIdleValue = average_readings(currentValue);               
                    slow_average = 0x00;
                }
            }
   
        }

        //variable delay during active mode
        if ( positiveReadings > 0 ) {
            _delay_ms(2);
         } else {
            _delay_ms(20);
        }
    }

    return 0;  
}


