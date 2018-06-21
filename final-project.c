/*
-------------------------------------------------------------------------
Authors:	Matthew Mancuso & Tyler Ressler
Date:		December 11, 2017
Course:		Processor Systems(ECE 3612), Temple University - Fall 2017
Project:	Disk RPM Calculation
-------------------------------------------------------------------------
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "LCD_driver.h"

// -- PORTB I/O Masks --
#define JOYSTICK_UP (1 << PB6)
#define JOYSTICK_CENTER (1 << PB4)
#define JOYSTICK_DOWN (1 << PB7)
#define JOYSTICK_MASK (JOYSTICK_UP | JOYSTICK_CENTER | JOYSTICK_DOWN)
#define SENSOR_MASK (1 << PB1)
#define MOTOR_MASK (1 << PB0)

// -- PWM Timer -- Timer0 in CTC Mode --
#define PWM_INCREMENT_PERIOD 0.000025 // 25us
#define PWM_PERIOD 0.00025 // 250us (increment period * increments)
#define PWM_PRESCALE (0 << CS02 | 0 << CS01 | 1 << CS00) // clk*1
#define PWM_COMPARE 199
#define PWM_INCREMENT 10 // 10%, 20%, etc
#define PWM_TOTAL_INCREMENTS (100 / PWM_INCREMENT)

// -- RPM Timer -- Timer1A in CTC Mode --
#define RPM_POLL 1 // 1s
#define RPM_PRESCALE (1 << CS12 | 0 << CS11 | 0 << CS10) // clk*256
#define RPM_COMPARE 31249 // Timer1 is 16-bit

// -- Sensor Timer -- Timer2 in CTC Mode --
#define SENSOR_POLL 0.00001 // 10us
#define SENSOR_PRESCALE (0 << CS22 | 1 << CS21 | 0 << CS20) // clk*8
#define SENSOR_COMPARE 9
#define SENSOR_DEBOUNCE 2 // Polls high twice to count

// -- Global Variables --
// Used to prevent erroneous joystick interrupts (i.e. debounce)
volatile uint8_t ignore_joystick = 0;

// Used to temporarily display duty cycle, "RUN" or "STOP" on LCD
volatile uint8_t lcd_block = 0;

// Used to track current division in PWM period
volatile uint8_t pwm_count = 0;

// Used to track current duty cycle; must be multiple of PWM_INCREMENT
volatile uint8_t duty_cycle = PWM_INCREMENT;

// Used to track running status of motor
volatile uint8_t motor_running = 1;

// Used to debounce sensor pulses
volatile uint8_t sensor_high_count = 0;

// Tracks current rotation count
volatile unsigned int count = 0;

// Updated every second, defined by RPM_POLL
volatile unsigned int rpm = 0;

int main(void) {
	// Turns LCD on
	LCD_Init();
	
	// Sets outputs as outputs
	DDRB |= MOTOR_MASK;

	// Pull up inputs
	PORTB |= JOYSTICK_MASK | SENSOR_MASK;
	
	// Joystick Interrupts
	PCMSK1 |= JOYSTICK_MASK;	// Only track joystick PORTB inputs
	EIMSK |= (1 << PCIE1);		// Turn on PORTB Pin Change Interrupts
	
	// PWM Timer -- Timer0 in CTC Mode
	TCCR0A = (0 << WGM01 | 1 << WGM00) | PWM_PRESCALE; // CTC/prescale settings
	OCR0A = PWM_COMPARE;		// Max TCNT0 before interrupting
	TIMSK0 |= (1 << OCIE0A);	// Enable interrupt on compare match
	
	// RPM Timer -- Timer1A in CTC Mode
	TCCR1B = (0 << WGM13 | 1 << WGM12) | RPM_PRESCALE; // CTC/prescale settings
	OCR1A = RPM_COMPARE;		// Max TCNT1 before interrupting
	TIMSK1 |= (1 << OCIE1A);	// Enable interrupt on compare match
	
	// Sensor Timer -- Timer2 in CTC Mode
	TCCR2A = (0 << WGM21 | 1 << WGM20) | SENSOR_PRESCALE; // CTC/prescale settings
	OCR2A = SENSOR_COMPARE;		// Max TCNT2 before interrupting
	TIMSK2 |= (1 << OCIE2A);	// Enable interrupt on compare match
	
	sei(); // Turn on all interrupts
	
	while (1) {
		// If increment_duty, decrement_duty or toggle_running have been called,
		// joystick_block will be > 0, so don't show RPM
		if (lcd_block == 0) {
			// Convert RPM to string
			char str[5];
			itoa(rpm, str, 10);
			
			// Display RPM on LCD
			LCD_puts(str);
		}
	}
}

void increment_duty() {
	if (duty_cycle < 100) duty_cycle += 10;
	
	// Forces LCD to display duty cycle for ~2 seconds
	lcd_block = 2;
	char str[5];
	itoa(duty_cycle, str, 10);
	LCD_puts(str);
}

void decrement_duty() {
	if (duty_cycle > 0) duty_cycle -= 10;
	
	// Forces LCD to display duty cycle for ~2 seconds
	lcd_block = 2;
	char str[5];
	itoa(duty_cycle, str, 10);
	LCD_puts(str);
}

void toggle_running() {
	motor_running = !motor_running;
	
	// Forces LCD to display running status for ~2 seconds
	lcd_block = 2;
	if (motor_running) LCD_puts("RUN");
	else LCD_puts("STOP");
}

// ISR for joystick pin changes on PORTB
ISR(PCINT1_vect) {
	// Take only joystick pins from PORTB
	uint8_t joystick = ~PINB & JOYSTICK_MASK;
	
	// Ensure one joystick button is pressed, and not debouncing
	if (joystick > 0 && ignore_joystick == 0) {
		if (joystick == JOYSTICK_CENTER) {
			toggle_running();
			} else if (joystick == JOYSTICK_DOWN) {
			decrement_duty();
			} else if (joystick == JOYSTICK_UP) {
			increment_duty();
		}
		
		// Set debouncing flag
		ignore_joystick = 1;
	}
}

// PWM Timer
ISR(TIMER0_COMP_vect) {
	// Generates pulse of duty_cycle % by switching motor on duty_cycle/100 times,
	// and off (1 - duty_cycle)/100 times. multiplying by motor_running sets duty
	// cycle to 0 if false, or duty_cycle if true
	if (pwm_count < motor_running*(duty_cycle / PWM_TOTAL_INCREMENTS)) {
		// Turn on motor; leave rest alone
		PORTB |= MOTOR_MASK;
		} else {
		// Turn off motor; leave rest alone
		PORTB &= ~MOTOR_MASK;
	}
	
	// Tracks total PWM divisions; resets every PWM_TOTAL_INCREMENTS
	if (pwm_count == (PWM_TOTAL_INCREMENTS - 1)) {
		pwm_count = 0;
		} else {
		pwm_count++;
	}
}

// RPM Timer
ISR(TIMER1_COMPA_vect) {
	// This timer runs for 1 second, giving RPS. Multiply by 60 for RPM
	rpm = count * 60.0;
	count = 0; // Reset count for next calculation
	
	// Every second, decrease the LCD flag until 0
	if (lcd_block > 0) lcd_block--;
	
	// Turn off joystick debounce flag every second
	ignore_joystick = 0;
}

// Sensor Timer
ISR(TIMER2_COMP_vect) {
	// Take only sensor pins from PORTB
	uint8_t sensor = ~PINB & SENSOR_MASK;
	
	// Debounce sensor by tracking how many times sensor is high per time increment.
	// Time sensor must be high is defined by SENSOR_DEBOUNCE*SENSOR_POLL.
	if (sensor) {
		// If sensor is high for this poll, increment the debounce count
		sensor_high_count++;
		} else {
		// If sensor goes low, check to see if it was high long enough
		if (sensor_high_count >= SENSOR_DEBOUNCE) {
			// If it was, count a rotation
			count++;
		}
		
		// Reset the debounce count
		sensor_high_count = 0;
	}
}
