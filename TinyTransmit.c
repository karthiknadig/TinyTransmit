/*
 * TinyTransmit.c
 *
 * Collects data from sensor and transmits wirelessly.
 *
 * Created: 6/21/2015 1:23:14 PM
 *  Author: karthik
 */ 

#include <inttypes.h>
#include <avr/io.h>
#include <util/crc16.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>

#define F_CPU 8000000
#include <util/delay.h>

// this has to change for each device
#define DEVICE_IDL 0xB6
#define DEVICE_IDH 0xF1
#define DEVICE_ID ((DEVICE_IDH<<0x8) & 0xFF00 | DEVICE_IDL)

#define TRANSMIT_DELAY 0x1A00
#define TRANSMIT_GAP 0x00FF
void inline transmit_bit(uint8_t bit)
{
	// Transmit the data first.
	
	// Set OCR0A Transmit delay
	OCR0A = TRANSMIT_DELAY;
	// Set Counter to zero
	TCNT0 = 0x0000;
	
	// Set the output value to PB3
	if(bit)
	{
		PORTB |= _BV(PORTB3); //Transmit a 1
	}
	else
	{
		PORTB &= ~_BV(PORTB3); // Transmit a 0
	}

	// Start the counter at full clock speed
	TCCR0B |= _BV(CS00);
	
	// wait for the timer to finish
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_mode();
	
	// Stop the clock
	//TCCR0B &= ~_BV(CS02) & ~_BV(CS01) & ~_BV(CS00);
	TCCR0B &= 0xF8;
	
	// inserting a short gap
	// Set OCR0A Transmit delay
	OCR0A = TRANSMIT_GAP;
	// Set Counter to zero
	TCNT0 = 0x0000;
	
	// Gap always transmits a 0
	PORTB &= ~_BV(PORTB3);
	
	// Start the counter at full clock speed
	TCCR0B |= _BV(CS00);
	
	// wait for the timer to finish
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_mode();
	
	// Stop the clock
	//TCCR0B &= ~_BV(CS02) & ~_BV(CS01) & ~_BV(CS00);
	TCCR0B &= 0xF8;
}

ISR(TIM0_COMPA_vect)
{
	// this clears the interrupt flag. OCF0A
}

void transmit(uint8_t tval)
{
	// power up timer/counter
	PRR &= ~_BV(PRTIM0);

	// Configure Timer to be disconnected from OC0A pin.
	TCCR0A = 0;
	
	// set the timer to CTC (Clear Timer on Compare) mode
	TCCR0B = _BV(WGM02);

	// enable timer/counter interrupt on compare for OCR0A register
	TIMSK0 = _BV(OCIE0A);

	// Reset interrupt flag
	TIFR0 |= _BV(OCF0A);
	
	for(uint8_t i = 0;i < 8; ++i)
	{
		transmit_bit(tval & _BV(i));		
	}
	
	// Reset interrupt flag
	TIFR0 |= _BV(OCF0A);
	
	// clear up the interrupts
	TIMSK0 &= ~_BV(OCIE0A);
	
	// stop the timer/counter
	TCCR0B = 0;
	TCCR0A = 0;
	
	// power down timer/counter
	PRR |= _BV(PRTIM0);
}

uint8_t getSensorValue()
{
	// start square wave
	OCR0A = 0;
	TCCR0A = _BV(COM0A0) ;
	TCCR0B = _BV(CS00) | _BV(WGM02);
	
	// wait for square wave to the capacitor
	//_delay_ms(10);
	
	// Start ADC process
	ADCSRA |= _BV(ADEN) | _BV(ADSC);
	
	// Wait for ADC to finish
	set_sleep_mode(SLEEP_MODE_ADC);
	sleep_mode();
	
    // wait for a bit 
	//_delay_ms(10);
	
	// save result
	uint8_t result = ADCL;

	// stop square wave
	TCCR0B = 0;
	TCCR0A = 0;
	
	//  Make port 0 low
	PORTB &= ~_BV(PORTB0);

	// Disable  ADC
	ADCSRA &=~ _BV(ADEN);

	return result;
}

ISR(ADC_vect)
{
	// do nothing just wake up
}

ISR(WDT_vect)
{
	// do nothing just wake up
}

uint8_t crc8(uint8_t crc, uint8_t data)
{
	return _crc8_ccitt_update(crc, data);
}

int main(void)
{
	// set-up IO
	//DDRB |= _BV(DDB0);   // Square wave output
	//PORTB &= ~_BV(PORTB0);
	//DDRB |= _BV(DDB1);   // Sensor input
	//PORTB &= ~_BV(PORTB1);
	//DDRB |= _BV(DDB2);   // To LED
	//PORTB &= ~_BV(PORTB2);
	//DDRB |= _BV(DDB3);   // To Transmitter
	//PORTB &= ~_BV(PORTB3);
	DDRB = 0x0f;
	PORTB = 0x00;
	
	
	//  Set the clock to the inter 8MHz oscillator
	//CLKMSR &= ~_BV(CLKMS1) & ~_BV(CLKMS0);
	CLKMSR = 0x00;
	
	// Set clock scaling to divide by 1 (no scaling)
	//CLKPSR &=  ~_BV(CLKPS3) & ~_BV(CLKPS2) & ~_BV(CLKPS1) & ~_BV(CLKPS0);
	CLKPSR = 0x00;	
	
	// Disable ADC triggers and Enable ACD Interrupt, and ADC
	//ADCSRA &= ~_BV(ADSC) & ~_BV(ADATE) & ~_BV(ADPS0) & ~_BV(ADPS1) & ~_BV(ADPS2);
	//ADCSRA |= _BV(ADIF) | _BV(ADIE) | _BV(ADEN);
	ADCSRA = 0x98;
	
	// Set ADC to free running mode (Disabling ADATE has the same effect)
	//ADCSRB &= ~_BV(ADTS2) & ~_BV(ADTS1) & ~_BV(ADTS0);
	ADCSRB = 0x00;
	
	// Set ADC 0 (PB0) as the input
	// ADMUX &= ~_BV(MUX0) & ~_BV(MUX1); // 0x00
	ADMUX = 0x00;
	
	// we want all values so clear the pin disable register and set ADC3 for analog
	//DIDR0 = 0x00 | _BV(ADC0D); 
	DIDR0 = 0x01;
	
	// Turn off timer counter and ADC
	//PRR |= _BV(PRTIM0) | _BV(PRADC);
	PRR = 0x03;
	
	// Watchdog timer interrupt and set WDT pre-scaled to  1024K cycles (8seconds)
	//WDTCSR &= ~_BV(WDE) & ~_BV(WDP2) & ~_BV(WDP1);
	//WDTCSR |= _BV(WDIE) | _BV(WDP3) | _BV(WDP0);
	WDTCSR = 0x61;
	
	// enable interrupts globally
	sei();
	
	// store Sensor result in this 
	uint8_t result = 0;	
	// store CRC8 result in this
	uint8_t crc8result = 0;

	while(1)
	{
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		sleep_mode();
		
		// Turn LED on
		PORTB |= _BV(PORTB2);
		PUEB |= _BV(PUEB2);
		
		// power up ADC and timer
		//PRR &= ~_BV(PRADC) & ~_BV(PRTIM0);
		PRR = 0x00;
		
		// Get Sensor reading
		result = getSensorValue();
		
		// Power down ADC
		//PRR |= _BV(PRADC);
		PRR = 0x02;
		
		// Calculate CRC value
		crc8result = crc8(0, DEVICE_IDL);
		crc8result = crc8(crc8result, DEVICE_IDH);
		crc8result = crc8(crc8result, result);
			
		transmit(DEVICE_IDL);
		transmit(DEVICE_IDH);
		transmit(result);
		transmit(crc8result);
			
		// Turn LED off
		PORTB &= ~_BV(PORTB2);
		PUEB &= ~_BV(PUEB2);
		
		// Power down Timer/counter
		// PRR |= _BV(PRTIM0);
		PRR = 0x03;
	}
}