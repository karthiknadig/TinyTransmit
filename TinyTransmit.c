/*
 * TinyTransmit.c
 *
 * Collects data from sensor and transmits wirelessly.
 *
 * Created: 6/21/2015 1:23:14 PM
 *  Author: karthik
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay_basic.h>

#define F_CPU 8000000
#include <util/delay.h>

// this has to change for each device
#define DEVICE_IDL 0xB6
#define DEVICE_IDH 0xF1
#define DEVICE_ID ((DEVICE_IDH<<0x8) & 0xFF00 | DEVICE_IDL)

#define TX_BAUD 800

uint8_t txMode = 0;
uint8_t txValue = 0;
uint8_t txDone = 0;

void txByte(uint8_t val)
{
	txValue = val;
	txMode = 0;
	txDone = 0;
	
	TIMSK0 = 0x02;
	TIFR0 = 0x02;
	OCR0A = TX_BAUD;
	TCNT0 = TX_BAUD - 0x80;
	TCCR0A = 0x80;
	TCCR0B = 0x09;

	for(;txMode<8;++txMode)
	{
		set_sleep_mode(SLEEP_MODE_IDLE);
		sleep_mode();
	}
	TCCR0A = 0;
	TCCR0B = 0;
}

ISR(TIM0_COMPA_vect)
{
	switch(txMode)
	{
		default:
		if(txValue & 0x01)
		{
			TCCR0A = 0xC0;
		}
		else
		{
			TCCR0A = 0x80;
		}
		txValue = txValue>>1;
		break;
	}
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

static uint8_t crc8(uint8_t __crc, uint8_t __data)
{
	uint8_t __i, __pattern;
	__asm__ __volatile__ (
	"	eor	%0, %4" "\n\t"
	"	ldi	%1, 8" "\n\t"
	"	ldi	%2, 0x8C" "\n\t"
	"1:	lsr	%0" "\n\t"
	"	brcc	2f" "\n\t"
	"	eor	%0, %2" "\n\t"
	"2:	dec	%1" "\n\t"
	"	brne	1b" "\n\t"
	: "=r" (__crc), "=d" (__i), "=d" (__pattern)
	: "0" (__crc), "r" (__data));
	return __crc;
}

void delay()
{
	uint8_t _low = 0x55; // alias %0
	uint8_t _high = 0x55; // alias %1
	__asm__ volatile (
	"2: ldi %0,0x55" "\n\t"
	"1: subi %0,1" "\n\t"
	"brne 1b" "\n\t"
	"subi %1,1" "\n\t"
	"brne 2b"
	: "=r" (_low), "=r" (_high)
	);
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
			
		txByte(DEVICE_IDL);
		txByte(DEVICE_IDH);
		txByte(result);
		txByte(crc8result);
			
		// Turn LED off
		PORTB &= ~_BV(PORTB2);
		PUEB &= ~_BV(PUEB2);
		
		// Power down Timer/counter
		// PRR |= _BV(PRTIM0);
		PRR = 0x03;
	}
}