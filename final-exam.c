/*
*-------------------------------------------------------------------------
* Authors:	Matthew Mancuso & Tyler Ressler
* Date:		December 18, 2017
* Course:	Processor Systems(ECE 3612), Temple University - Fall 2017
* Project:	Final Exam
*-------------------------------------------------------------------------
*/

#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "LCD_driver.h"

// -- Internal Reference Voltage (1.1V) for 169P --
#define VCC 1100UL

// -- Analog to Digital Conversion Mux & Control Selection --
#define ADC_VOLTAGE	(1 << REFS1) | (1 << REFS0)  // selects internal voltage reference
#define ADC_MUXSEL (1 << MUX0) // selects ADC0
#define ADC_PRESCALE (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // division factor of 128
#define ADC_ENABLE (1 << ADEN)
#define ADC_START (1 << ADSC)

// -- I/O Pin Definitions --
#define PING_MASK (1 << PB0)
#define LED_MASK (1 << PB1)
#define SERVO_MASK (1 << PB7)

// -- Timer Values --
#define PING_PRESCALE 1024
#define LED_PRESCALE 1024
#define SERVO_PRESCALE 1024
#define SERVO_PWM_DISABLED ~((1 << COM2A1) | (1 << COM2A0))
#define SERVO_PWM_ENABLED (1 << COM2A1) | (0 << COM2A0)

// -- Servo PWM Pulse Times --
#define SERVO_PULSE_CW 1.3
#define SERVO_PULSE_CCW 1.7
#define SERVO_PULSE_DELAY 20

// -- Servo Direction Constants for setServoDirection() --
#define SERVO_DIR_STOP 0
#define SERVO_DIR_CW 1
#define SERVO_DIR_CCW 2

// -- Calculations --
#define PROC_PERIOD 0.000000125		// Take 1/F_CPU
#define PING_INCH_PER_COUNT 0.856	// Take PING_PRESCALE*0.0008359375
#define LED_COUNT_PER_SEC 7812		// Take 1/(LED_PRESCALE*PROC_PERIOD)
#define SERVO_COUNT_PER_MS 7.8125	// Take 0.001/(SERVO_PRESCALE*PROC_PERIOD)
#define SERVO_DELAY_COMPARE 99		// Take 255 - (SERVO_COUNT_PER_MS*SERVO_PULSE_DELAY)

// -- Function Prototypes --
void ADCInit(void);							// Turns on ADC inputs
uint16_t ADCRead(void);						// Returns ADC level in mV
void sendPINGStartPulse();					// Sends initial 5ms PING pulse
void listenForPINGPulse(uint8_t listen);	// Turns interrupt on for PING return; boolean input
void setServoDirection(uint8_t direction);	// Sets servo direction; uses SERVO_DIR_ constants as input

// -- Global Variables --
char lcd_str[7];					// Used to store string to put to LCD
volatile float dist = 0;			// Calculated PING distance in inches
volatile uint8_t led_status = 0;	// 0 for stopped; 1 for solid on; 2 for blinking
volatile int led_rate = 0;			// Blinking speed of LED in ms
volatile float servo_pulse = 1.7;	// Current variable servo PWM pulse in ms

int main(void) {
	// -- Initializations --
	LCD_Init();
	ADCInit();
	
	// -- Sets I/O Values --
	DDRB |= LED_MASK | SERVO_MASK;	// Ensure LED and servo pins are outputs
	PORTB |= LED_MASK;				// Turn on LED to begin
	PORTB &= ~SERVO_MASK;			// Turn off servo to begin
	
	// -- PORTB Pin Change Interrupt Init --
	PCMSK1 = PING_MASK;	// Ensure only PING sensor is tracked
						// Don't enable interrupts yet; sendPINGStartPulse() will do that
	
	// -- Timer0: PING Counter/Timer --
	TCCR0A |= (1 << CS02 | 0 << CS01 | 1 << CS00);	// Prescale Clk*1024
	TIMSK0 |= (1 << TOIE0);							// Enable overflow interrupt
	
	// -- Timer1A: LED Blink Timer --
	TCCR1B |= (1 << CS12 | 0 << CS11 | 1 << CS10)	// Prescale Clk*1024
		   |  (1 << WGM12);							// Enable CTC mode
	TIMSK1 |= (1 << OCIE1A);						// Enable compare match interrupt
	
	TCCR2A |= (1 << CS22 | 1 << CS21 | 1 << CS20)	// Prescale Clk*1024
		   |  (1 << WGM21 | 1 << WGM20);			// Enable Fast PWM mode
	TIMSK2 |= (1 << TOIE2);							// Enable overflow interrupt
													// Don't enable PWM output to OC2A yet
	OCR2A = SERVO_DELAY_COMPARE;					// Length of delay pulse (20ms), 
													// since OC2A goes low on match
	
	sei();	// Enable all global interrupts
	
	while (1) {
		// Read in mV from ADC
		uint16_t adc_out = ADCRead();
		
		// Continuously display distance on LCD; force to right of display
		sprintf(lcd_str, "%4dIN", (int)dist);
		LCD_puts(lcd_str);
		
		// Test ADC value per assignment instructions
		if (adc_out <= 1000) {
		
			if (dist < 10) {
				led_status = 1;						// LED solid on
				setServoDirection(SERVO_DIR_CCW);	// Rotate servo counterclockwise
			} else if (dist < 30) {
				led_status = 2;						// LED blinking
				led_rate = ((dist - 10)*80);		// Varies blinking from 0-1.6 seconds
				setServoDirection(SERVO_DIR_STOP);	// Stop servo
			} else {
				led_status = 0;						// LED solid off
				setServoDirection(SERVO_DIR_CW);	// Rotate servo clockwise
			}
			
		} else if (adc_out <= 2000) {
			
			led_status = 2;							// Led blinking
			led_rate = 2000;						// Blinks every 2 seconds
			setServoDirection(SERVO_DIR_STOP);		// Ensure servo is stopped
			
		} else {
			
			led_status = 2;							// Led blinking
			led_rate = 5000;						// Blinks every 5 seconds
			setServoDirection(SERVO_DIR_STOP);		// Ensure servo is stopped
			
		}
		
	}
}

// Turns on ADC inputs
void ADCInit() {
	// ADC Control and Status Register A
	ADCSRA |= ADC_START | ADC_ENABLE | ADC_PRESCALE;
	ADMUX |= ADC_VOLTAGE | ADC_MUXSEL;
}

// Returns ADC level in mV
uint16_t ADCRead() {
	ADCSRA |= (1 << ADSC);			// Test for result
	while (ADCSRA & (1 << ADSC));	// Wait for result
	return (ADC * VCC * 6) / 1024;	// Convert to mV
}

// Sends initial 5ms PING pulse; waits for PING return interrupt
void sendPINGStartPulse() {
	listenForPINGPulse(FALSE);	// Ensure interrupts are off
	
	DDRB |= PING_MASK;			// Set PING pin to output
	
	PORTB &= ~PING_MASK;		// Ensure output is low for 2us
	_delay_us(2);
	PORTB |= PING_MASK;			// Output high for 5us
	_delay_us(5);
	PORTB &= ~PING_MASK;		// Set output back to low
	
	DDRB &= ~PING_MASK;			// Set PING pin back to input
	
	listenForPINGPulse(TRUE);	// Enable interrupts
}

// Turns interrupt on for PING return; boolean input
void listenForPINGPulse(uint8_t listen) {
	if (listen) {
		EIMSK |= (1<<PCIE1);	// Enable PORTB pin change interrupts
	} else {
		EIMSK &= ~(1<<PCIE1);	// Disable PORTB pin change interrupts
	}
}

// Sets servo direction; uses SERVO_DIR_ constants as input
void setServoDirection(uint8_t direction) {
	switch(direction) {
		case SERVO_DIR_STOP:
			TCCR2A &= SERVO_PWM_DISABLED;	// Turn OC2A off to turn PWM off
			PORTB &= ~SERVO_MASK;			// Ensure servo is low
			break;
		case SERVO_DIR_CW:
			TCCR2A |= SERVO_PWM_ENABLED;	// Turn OC2A on to turn PWM on
			servo_pulse = SERVO_PULSE_CW;	// Set direction pulse to 1.3ms
			break;
		case SERVO_DIR_CCW:
			TCCR2A |= SERVO_PWM_ENABLED;	// Turn OC2A on to turn PWM on
			servo_pulse = SERVO_PULSE_CCW;	// Set direction pulse to 1.7ms
			break;
	}
}

// PORTB pin change interrupt; after PING start pulse, counts return delay
ISR(PCINT1_vect) {
	if (PINB & PING_MASK) {	// If PING went high,
		TCNT0 = 0;			// start count
	} else if (TCNT0 > 0) {					// If PING went low and counting has begun,
		dist = TCNT0 * PING_INCH_PER_COUNT;	// calculate distance
		listenForPINGPulse(FALSE);			// Disable future interrupts until next start pulse
	}
}

// PING timer overflow interrupt; gives PING time to prepare for next measurement
ISR(TIMER0_OVF_vect) {
	sendPINGStartPulse();
}

// LED timer compare match; allows various LED blinking rates
ISR(TIMER1_COMPA_vect) {
	if (led_status == 2) {
		// If LED is blinking, alternate LED status
		if (PORTB & LED_MASK) {
			PORTB &= ~LED_MASK;
		} else {
			PORTB |= LED_MASK;
		}
	} else {
		// If LED is solid on, turn it on, otherwise off
		if (led_status) {
			PORTB |= LED_MASK;
		} else {
			PORTB &= ~LED_MASK;
		}
	}
	
	// Set 16-bit output compare register to vary LED blinking rate
	OCR1A = LED_COUNT_PER_SEC * ((float)led_rate / 1000.0);
}

// Servo PWM timer overflow; resets TCNT2 to vary high pulse
ISR(TIMER2_OVF_vect) {
	// Start the counter 1.3ms or 1.7ms before compare match.
	// This allows precise high and low pulses.
	// TCNT2 to OCR2A will be high, OCR2A to MAX (255) will be low.
	TCNT2 = OCR2A - (servo_pulse * SERVO_COUNT_PER_MS);
}
