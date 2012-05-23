/*	LPD8806 object has:
	3 bytes per pixel
	numLEDs

	write to SPI:
	simple function hooks for hardware/bit-bang SPI with same call/return format

	Clocks SPI at 2 MHz.  With proper shielding it should work up to 20 MHz.

	writeStrip function:
	- write out 24 bits / pixel
	- write out latch sequence

	Byte order is G,R,B.  Possibly need to |= 0x80

	setPixelColor function
	clearStrip function

*/

#define	F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>

#include "LPD8806.h"

pixel_t	strip[NUM_LEDS];
pixel_t nullstrip[NUM_LEDS];

int 	main(void) {
	//init_ports();
	//init_spi();
	//init_uart();

	DDRB = 0xFF;
	PORTB = 0xFF;
	DDRC	= 0x00;
	DIDR0	= 0xFF;
	DDRD 	= 0b11111010;	// PD2 as button input, PD0 as UART RX

	init_spi();

/*	for ( ; ; ){
		//PORTB = 0xFF;
		//PORTB = 0;
		hardware_spi_write(0xF0);
	}
*/

	clearStrip(nullstrip, NUM_LEDS);

	clearStrip(strip, NUM_LEDS);
	writeStrip(strip, NUM_LEDS, hardware_spi_write);
	latchStrip(hardware_spi_write, NUM_LEDS);

/*	uint8_t	stripAddress = 0x00;

	uint8_t	serial_inBuffer[SERIAL_INBUFFER_LEN];
	uint8_t	serial_inPos = 0;
	uint8_t	serial_rxState = 0;
	uint8_t	serial_mask = 0;
	uint8_t serial_packetLen = 0; */

//	init_adc();
//	uint8_t offset = 0;

	uint16_t	buttonTimer = 0;
	uint16_t	resetTimer = 0;
	uint8_t 	displayState = 0;

	for ( ; ; ) {
		if (PORTD & BUTTONMASK) { // Button is pressed
			buttonTimer++;
			resetTimer = 0;
		} else {	// Button is not pressed
			if (buttonTimer > 1000) // holding the button
				displayState = 0;
			else if (buttonTimer > 30) // quick press
				displayState++;
			buttonTimer = 0;
		}

		switch (displayState) {
			case 0:
				effect_colorFade(strip, 20, 32);
				break;
			case 1:
				effect_colorFade(strip, 10, 32);
				break;
			case 2:
				effect_rgbChecker(strip, NUM_LEDS, 32);
				writeStrip(strip, NUM_LEDS, hardware_spi_write);
				latchStrip(hardware_spi_write, NUM_LEDS);
				_delay_ms(300);
				break;
			case 3:
				effect_colorStrobe(strip, NUM_LEDS, 20, 20, 32);
				break;
			case 4:
				effect_rgbStrobe(strip, NUM_LEDS, 20, 20, 32, 3);
				break;
			default:
				displayState = 0;
				break;
		}

	}

	/*	uint8_t j;
		for (j=0; j<3; j++) {
			ADMUX = (j | 0xC0);	// Red on AN0, Green on AN1, Blue on AN2
			ADC_START();
			//_delay_ms(10);
			loop_until_bit_is_clear(ADCSRA, ADSC);
			uint16_t newsample = ADCL;
			newsample += (ADCH << 8);
			switch (j) {
				case 0:	// Red
					r = newsample >> 4;// * 10;
					break;
				case 1:	// Green
					g = newsample >> 3;// * 14;
					break;
				case 2:	// Blue
					b = newsample >> 3;// * 17;
					break;
			}
		}

		setStripColor(strip, NUM_LEDS, r, g, b); */
		//effect_rgbChecker(strip, NUM_LEDS, 32);
		//effect_colorFade(strip, 10, 32);
		effect_rgbStrobe(strip, NUM_LEDS, 10, 20, 40, 5);
		//if (offset >= 3) offset = 0;

		//_delay_ms(50);

	/*	clearStrip(strip, NUM_LEDS);
		writeStrip(strip, NUM_LEDS, hardware_spi_write);
		latchStrip(hardware_spi_write, NUM_LEDS);
		_delay_ms(500);

		if (offset == 3) offset = 0;  */

	


/*	for ( ; ; ) {
		if (checkflag(SERIAL_FLAG)) {
			switch (serial_rxState) {
				case RX_State_first_byte:
					serial_packetLen = 0;
					serial_inPos = 0;
					if (serial_in < 128) {	// Strip address
						if (stripAddress == serial_in)
							serial_rxState = RX_State_packet_len;
						else
							serial_rxState = RX_State_packet_len_skip;
					}	else {
						serial_mask = (serial_in & 0x7F);
						serial_rxState = RX_State_mask_match;
					}
					break;
				
				case RX_State_packet_len:
					serial_packetLen = serial_in;
					serial_rxState = RX_State_payload;
					break;

				case RX_State_packet_len_skip:
					serial_packetLen = serial_in;
					serial_rxState = RX_State_skip;
					break;

				case RX_State_mask_match:
					if (serial_in == (serial_mask & stripAddress))
						serial_rxState = RX_State_packet_len;
					else
						serial_rxState = RX_State_packet_len_skip;
					break;

				case RX_State_payload:
					serial_inBuffer[serial_inPos++] = serial_in;
					if (serial_inPos == serial_packetLen) {
						// process packet
						serial_processPacket(serial_inBuffer, strip, NUM_LEDS);
						serial_rxState = RX_State_first_byte;
					}
					else if (serial_inPos == SERIAL_INBUFFER_LEN)
						serial_inPos = 0;
					break;

				case RX_State_skip:
					serial_inPos++;
					if (serial_inPos == serial_packetLen)
						serial_rxState = RX_State_first_byte;
					break;

			}
		}
	}
*/

/*	for ( ; ; ){
		//	Main code here

		effect_rgbChecker(strip, NUM_LEDS, offset++);
		writeStrip(strip, NUM_LEDS, hardware_spi_write);
		latchStrip(hardware_spi_write, NUM_LEDS);
		_delay_ms(50);
		if (offset == 3) offset = 0;

 	} */
}

void	serial_processPacket(uint8_t const *buffer, pixel_t strip[], uint8_t const stripLen) {
	switch (buffer[0]) {
		case CMD_SET_PIXEL_COLOR:
			setPixelColor(&strip[buffer[0]], buffer[1], buffer[2], buffer[3]);
			break;
		case CMD_SET_STRIP_COLOR:
			setStripColor(strip, stripLen, buffer[0], buffer[1], buffer[2]);
			break;
		case CMD_LATCH:
			latchStrip(hardware_spi_write, NUM_LEDS);
			break;
	}
}


void	setPixelColor(pixel_t *pixel, uint8_t r, uint8_t g, uint8_t b) {
	pixel->r = r | 0x80;
	pixel->g = g | 0x80;
	pixel->b = b | 0x80;
}


void	clearStrip(pixel_t strip[], uint8_t stripLen) {
	uint8_t j;
	for (j=0; j<stripLen; j++) {
		setPixelColor(&strip[j], 0, 0, 0);
		//strip[j].r = 0;
		//strip[j].g = 0;
		//strip[j].b = 0;
	}
}

void	effect_rgbStrobe(pixel_t strip[], uint8_t stripLen, uint8_t up, uint8_t down, uint8_t amplitude, uint8_t repeat) {

	effect_rgbChecker(strip, stripLen, amplitude);	
	uint8_t j;
	for (j=0; j<repeat; j++){
		writeStrip(strip, stripLen, hardware_spi_write);
		latchStrip(hardware_spi_write, stripLen);
		_delay_ms(up);
		writeStrip(nullstrip, stripLen, hardware_spi_write);
		latchStrip(hardware_spi_write, stripLen);
		_delay_ms(down);
	}

}

void	effect_colorStrobe(pixel_t strip[], uint8_t stripLen, uint8_t up, uint8_t down, uint8_t amplitude) {
	//TODO strobe a single color at a time across the whole strip
	static uint8_t offset = 0;
	if (amplitude > 0x7f) amplitude = 0x7f;
	switch (offset) {
		case 0:
			setStripColor(strip, stripLen, amplitude, 0, 0);
			break;
		case 1:
			setStripColor(strip, stripLen, 0, amplitude, 0);
			break;
		case 2:
			setStripColor(strip, stripLen, 0, 0, amplitude);
			break;
		default:
			offset = 0;
			break;
	}
	writeStrip(strip, stripLen, hardware_spi_write);
	latchStrip(hardware_spi_write, stripLen);
	_delay_ms(up);
	writeStrip(nullstrip, stripLen, hardware_spi_write);
	latchStrip(hardware_spi_write, stripLen);
	_delay_ms(down);
}

int8_t	effect_rgbChecker(pixel_t strip[], uint8_t stripLen, uint8_t amplitude) {
	
	static uint8_t index = 0;

	if ((stripLen < 0)) {//} || (offset > 2) || (offset < 0)) {
		return -1;
	}

//	if (offset > 2) offset = 2;
//	else if (offset < 0) offset = 0;
	uint8_t offset = index;
	if (amplitude > 0x7f) amplitude = 0x7f;
	uint8_t i = 0;
	do {
		switch (offset) {
			case 0:
				setPixelColor(&strip[i], amplitude, 0x00, 0x00);
				break;
			case 1:
				setPixelColor(&strip[i], 0x00, amplitude, 0x00);
				break;
			case 2:
				setPixelColor(&strip[i], 0x00, 0x00, amplitude);
				break;
		}
		offset++;
		if (offset > 2) offset = 0;
		i++;
	}	while (i < stripLen);
	index++;
	if (index > 2) index = 0;

//	writeStrip(strip, NUM_LEDS, hardware_spi_write);
//	latchStrip(hardware_spi_write, NUM_LEDS);

	return 0;
}

void	effect_colorFade(pixel_t strip[], uint16_t delay, uint8_t amplitude) {
	static uint8_t mode = 0;
	static uint8_t r;
	static uint8_t g;
	static uint8_t b;

	switch (mode) {
		case 0:
			r = amplitude;
			g = 0;
			b = 0;
			mode = 1;
			break;
		case 1:	// red state
			r--;
			g++;
			if (g == amplitude) mode++;
			break;
		case 2: // green state
			g--;
			b++;
			if (b == amplitude) mode++;
			break;
		case 3:	// blue state
			b--;
			r++;
			if (r == amplitude) mode = 1;
			break;
		default:
			mode = 0;
	}
	setStripColor(strip, NUM_LEDS, r, g, b);
	writeStrip(strip, NUM_LEDS, hardware_spi_write);
	latchStrip(hardware_spi_write, NUM_LEDS);
	_delay_ms(delay);

}

int8_t	setStripColor(pixel_t strip[], uint8_t stripLen, uint8_t r, uint8_t g, uint8_t b) {
	if (stripLen < 0) return -1;
	uint8_t	i = 0;
	do {
		setPixelColor(&strip[i++], r, g, b);
	}	while (i < stripLen);
	return 0;
}

void	latchStrip(void (*spi_write_p)(uint8_t), uint8_t stripLen ) {
	uint16_t n;
	n = ((stripLen+63)/64) * 3;

	while (n--) spi_write_p(0);
}

void	writeStrip(pixel_t strip[], uint8_t stripLen, void (*spi_write_p)(uint8_t)) {
	uint8_t	j;
	for (j=0; j<stripLen; j++) {
		spi_write_p(strip[j].g);
		spi_write_p(strip[j].r);
		spi_write_p(strip[j].b);
	}
}

void	init_spi(void) {
	/*	SPCR - SPI control register

	7	SPIE: SPI interrupt enable
	6 	SPE: SPI enable
	5	DORD: data order (1 - LSB first, 0 - MSB first)
	4	MSTR: master(1)/slave(0) select
	3	CPOL: clock idle & polarity
	2 	CPHA: Clock phase
	1,0 SPR1:0: SPI Clock Rate
	*/

	SPCR = 0b01010000;
	/*	SPSR - SPI status register

	7	SPIF: SPI interrupt flag
	6 	WCOL: Write collision flag
	5-1 x
	0 	SP2X: double-rate SPI
	*/

	//SPSR = 0b00000000;
}

void	hardware_spi_write(uint8_t const byte) {
	//init_spi();
	SPDR = byte;
	while (!(SPSR & (1<<SPIF)));
}


ISR(USART_RX_vect) {
	serial_in = UDR0;
	setflag(SERIAL_FLAG);
}


void			init_adc(void) {
//	DDRC	=	0x00;	// Use PORTC as analog inputs
//	DIDR0	=	0x3F;	// Disable digital inputs

	/*	ADMUX - ADC multiplexer control reg.

	BIT	VAL	FCN
	7:6	b01	Reference select.  01 -> use AVCC.
	5	0	Left-adjust output control (off)
	4	x	unused
	3:0	0x0	Channel select (0-7, internal temp. sens. on 8)
	*/
	ADMUX	=	0xC0;

	/*	ADCSRA - ADC ctrl reg A

	BIT	VAL	FCN
	7	1	ADEN: ADC enable
	6	0	ADSC: start converison. 1->0 for conversion done.
	5	0	ADC auto-trigger enable
	4	0	ADIF: interrupt flag
	3	0	ADIE: interrupt enable
	2:0	110	Prescaler select. 110= div by 64
			Gives Fadc = 125kHz
	*/
	ADCSRA	=	0x86; 

	//	DIDR0 - digital input disable reg 
	DIDR0	=	0x3F;
	
	/*	ADCSRB - ADC ctrl reg B

	BIT	VAL	FCN
	7:3	0	don't care
	2:0	xxx	Auto-trigger source.  101 = Timer1 match B
	*/
	ADCSRB = 0;

	//	Example sets ADSC during init and waits here for a sample
	//	to 'prime' the system.
	ADCSRA	|=	_BV(ADSC);
	loop_until_bit_is_clear(ADCSRA, ADSC);

}

