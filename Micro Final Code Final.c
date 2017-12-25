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

// Internal reference voltage (1.1V) for 169P
#define VCC 1100UL

// -- Analog to Digital Conversion Mux & Control Selection --
#define ADC_VOLTAGE	(1 << REFS1) | (1 << REFS0)  // selects internal voltage reference
#define ADC_MUXSEL (1 << MUX0) // selects ADC0
#define ADC_PRESCALE (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // division factor of 128
#define ADC_ENABLE (1 << ADEN)
#define ADC_START (1 << ADSC)

#define PING_MASK (1 << PB0)
#define LED_MASK (1 << PB1)
#define SERVO_MASK (1 << PB7)

#define SERVO_DIR_STOP 0
#define SERVO_DIR_CW 1
#define SERVO_DIR_CCW 2

#define SERVO_PULSE_CW 1.3
#define SERVO_PULSE_CCW 1.7

// prots
void ADCInit(void);
uint16_t ADCRead(void);

volatile float dist = 0;
volatile uint8_t led_status = 0;
volatile int led_rate = 0;
volatile float servo_pulse = 1.7;

int main(void) {
	LCD_Init();
	ADCInit();
	
	DDRB |= LED_MASK | SERVO_MASK;
	PORTB |= LED_MASK;
	PORTB &= ~SERVO_MASK;
	
	PCMSK1 = PING_MASK;
	
	TCCR0A |= (1 << CS02 | 1 << CS00);
	TIMSK0 |= (1 << TOIE0);
	
	TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10) | (1 << WGM12);
	TIMSK1 |= (1 << OCIE1A);
	
	TCCR2A |= (1 << CS22) | (0 << CS21) | (1 << CS20) | (1 << WGM21) | (1 << WGM20) | (0 << COM2A1) | (0 << COM2A0);
	TIMSK2 |= (1 << TOIE2);
	OCR2A = 156;
	
	sendStartPulse();
	
	sei();
	
	while (1) {
		// read in from ADC, convert data to mV
		uint16_t adc_result = ADCRead();
		uint16_t adc_out = (adc_result * VCC * 6) / 1024; // ADC output in mV
		
		char str[7];
		sprintf(str, "%3dIN", (int)dist);
		LCD_puts(str);
		
		// test ADC output
		if (adc_out <= 1000) { // between 0V & 1V
		
			if (dist < 10) {
				led_status = 1;
				setServoDirection(SERVO_DIR_CCW);
			} else if (dist < 30) {
				led_status = 2;
				led_rate = ((dist - 10)*63);
				setServoDirection(SERVO_DIR_STOP);
			} else {
				led_status = 0;
				setServoDirection(SERVO_DIR_CW);
			}
			
		} else if (adc_out <= 2000) { // between 1V & 2V
			
			led_status = 2;
			led_rate = 2000;
			setServoDirection(SERVO_DIR_STOP);
			
		} else { // greater than 2V
			
			led_rate = 5000;
			setServoDirection(SERVO_DIR_STOP);
			
		}
		
	}
}

void ADCInit() {
	// ADC Control and Status Register A
	ADCSRA |= ADC_START | ADC_ENABLE | ADC_PRESCALE;
	ADMUX |= ADC_VOLTAGE | ADC_MUXSEL;
}

uint16_t ADCRead() {
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

void sendStartPulse() {
	listenForPulse(FALSE);
	
	DDRB |= PING_MASK;
	
	PORTB &= ~PING_MASK;
	_delay_us(2);
	PORTB |= PING_MASK;
	_delay_us(5);
	PORTB &= ~PING_MASK;
	
	DDRB &= ~PING_MASK;
	
	listenForPulse(TRUE);
}

void listenForPulse(uint8_t listen) {
	if (listen) {
		EIMSK |= (1<<PCIE1);
	} else {
		EIMSK &= ~(1<<PCIE1);
	}
}

void setServoDirection(uint8_t direction) {
	switch(direction) {
		case SERVO_DIR_STOP:
			TCCR2A &= ~((1 << COM2A1) | (1 << COM2A0));
			PORTB &= ~SERVO_MASK;
			break;
		case SERVO_DIR_CW:
			TCCR2A |= (1 << COM2A1) | (0 << COM2A0);
			servo_pulse = SERVO_PULSE_CW;
			break;
		case SERVO_DIR_CCW:
			TCCR2A |= (1 << COM2A1) | (0 << COM2A0);
			servo_pulse = SERVO_PULSE_CCW;
			break;
	}
}

ISR(PCINT1_vect) {
	if (PINB & PING_MASK) {
		TCNT0 = 0;
	} else if (TCNT0 > 0) {
		dist = TCNT0 * 0.856;
		listenForPulse(FALSE);
	}
}

ISR(TIMER0_OVF_vect) {
	sendStartPulse();
}

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
	
	OCR1A = 7812 * ((float)led_rate / 1000.0);
}

ISR(TIMER2_OVF_vect) {
	TCNT2 = OCR2A - (servo_pulse * 7.8125);
}