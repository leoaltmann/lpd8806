/*	LPD8806.h

*/

#define ATTR_PACKED      __attribute__ ((packed))

#define	NUM_LEDS	6
#define	SERIAL_INBUFFER_LEN		32

#define	RX_State_first_byte			0
#define	RX_State_packet_len			1
#define	RX_State_packet_len_skip	2
#define	RX_State_mask_match			3
#define	RX_State_payload			4
#define	RX_State_skip 				5

#define	CMD_SET_PIXEL_COLOR			0x01
#define	CMD_SET_STRIP_COLOR			0x02
#define	CMD_MUTE_PIXEL				0x03
#define	CMD_UNMUTE_PIXEL			0x04
#define	CMD_MUTE_STRIP				0x05
#define	CMD_UNMUTE_STRIP			0x06
#define	CMD_SET_PIXEL_MUTE_COLOR	0xF1
#define	CMD_SET_STRIP_MUTE_COLOR	0xF2
#define	CMD_LATCH					0xFF


#define	ADMUX_R		0x40
#define	ADMUX_G		0x41
#define	ADMUX_B		0x42
#define	ADC_START()			ADCSRA |= _BV(ADSC)

#define	BUTTONMASK	0x04

typedef struct {
	uint8_t	pixel;
	uint8_t	r;
	uint8_t	g;
	uint8_t	b;
}	ATTR_PACKED PKT_SET_PIXEL_COLOR_t;


#define	FLAGS0	GPIOR0

#define	SERIAL_FLAG			(0x01),(&FLAGS0)

typedef struct {
	uint8_t	r;
	uint8_t	g;
	uint8_t	b;
}	ATTR_PACKED pixel_t;

extern pixel_t strip[];
extern pixel_t nullstrip[];

void	clearStrip(pixel_t strip[], uint8_t stripLen);
int8_t	effect_rgbChecker(pixel_t strip[], uint8_t stripLen, uint8_t amplitude);
void	effect_rgbStrobe(pixel_t strip[], uint8_t stripLen, uint8_t up, uint8_t down, uint8_t amplitude, uint8_t repeat);
void	effect_colorFade(pixel_t strip[], uint16_t delay, uint8_t amplitude);
void	effect_colorStrobe(pixel_t strip[], uint8_t stripLen, uint8_t up, uint8_t down, uint8_t amplitude);
void	latchStrip(void (*spi_write_p)(uint8_t), uint8_t stripLen );
void	writeStrip(pixel_t strip[], uint8_t stripLen, void (*spi_write_p)(uint8_t));
void	init_spi(void);
void	hardware_spi_write(uint8_t const byte);
void	serial_processPacket(uint8_t const *buffer, pixel_t strip[], uint8_t stripLen);
void	setPixelColor(pixel_t *pixel, uint8_t r, uint8_t g, uint8_t b);
int8_t	setStripColor(pixel_t strip[], uint8_t stripLen, uint8_t r, uint8_t g, uint8_t b);
void	init_adc(void);

static inline	void setflag(uint8_t flag, volatile uint8_t *reg) {
	*reg |= flag;
} __attribute__((always_inline));

static inline	void clearflag(uint8_t flag, volatile uint8_t *reg) {
	*reg &= (~flag);
} __attribute__((always_inline));

volatile static inline	int8_t checkflag(uint8_t flag, volatile uint8_t *reg) {
	return (*reg & flag);
} __attribute__((always_inline));

uint8_t	serial_in;
