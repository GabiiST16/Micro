#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#define CMD_START       0x01
#define CMD_STOP        0x02
#define CMD_SPEED       0x03
#define CMD_PISTON      0x04
#define CMD_EMERGENCY   0x05

#define TEMP_UMBRAL     50


#define PIN_SS          PB2
#define PIN_MOSI        PB3
#define PIN_MISO        PB4
#define PIN_SCK         PB5

#define PIN_BOTON       PD2
#define PIN_DHT11       PD3
#define CANAL_ADC_LDR   0
#define CANAL_ADC_POT   1
#define LCD_PORT        PORTD
#define LCD_DDR         DDRD
#define LCD_RS          PD4
#define LCD_EN          PD5
#define LCD_D4          PD6
#define LCD_D5          PD7
#define LCD_PORT_B      PORTB
#define LCD_DDR_B       DDRB
#define LCD_D6          PB0
#define LCD_D7          PB1

uint8_t estado_sistema = 0;
uint16_t contador_piezas = 0;
uint8_t temp_motor = 25;
uint8_t velocidad_pwm = 0;
uint8_t ldr_bloqueado = 0;

static void lcd_pulso_en(void) {
	LCD_PORT |= (1 << LCD_EN);
	_delay_us(1);
	LCD_PORT &= ~(1 << LCD_EN);
	_delay_us(50);
}

static void lcd_enviar_nibble(uint8_t nibble) {
	if (nibble & 0x01) LCD_PORT |= (1 << LCD_D4); else LCD_PORT &= ~(1 << LCD_D4);
	if (nibble & 0x02) LCD_PORT |= (1 << LCD_D5); else LCD_PORT &= ~(1 << LCD_D5);
	if (nibble & 0x04) LCD_PORT_B |= (1 << LCD_D6); else LCD_PORT_B &= ~(1 << LCD_D6);
	if (nibble & 0x08) LCD_PORT_B |= (1 << LCD_D7); else LCD_PORT_B &= ~(1 << LCD_D7);
	lcd_pulso_en();
}

void lcd_comando(uint8_t cmd) {
	LCD_PORT &= ~(1 << LCD_RS);
	lcd_enviar_nibble(cmd >> 4);
	lcd_enviar_nibble(cmd & 0x0F);
	_delay_ms(2);
}

void lcd_dato(char dato) {
	LCD_PORT |= (1 << LCD_RS);
	lcd_enviar_nibble(dato >> 4);
	lcd_enviar_nibble(dato & 0x0F);
	_delay_us(50);
}

void lcd_init(void) {
	LCD_DDR |= (1 << LCD_RS) | (1 << LCD_EN) | (1 << LCD_D4) | (1 << LCD_D5);
	LCD_DDR_B |= (1 << LCD_D6) | (1 << LCD_D7);
	_delay_ms(20);

	LCD_PORT &= ~(1 << LCD_RS);
	lcd_enviar_nibble(0x03);
	_delay_ms(5);
	lcd_enviar_nibble(0x03);
	_delay_us(150);
	lcd_enviar_nibble(0x03);
	lcd_enviar_nibble(0x02);

	lcd_comando(0x28);
	lcd_comando(0x0C);
	lcd_comando(0x06);
	lcd_comando(0x01);
	_delay_ms(2);
}

void lcd_set_cursor(uint8_t fila, uint8_t col) {
	uint8_t addr = (fila == 0) ? (0x80 + col) : (0xC0 + col);
	lcd_comando(addr);
}

void lcd_imprimir(const char *str) {
	while (*str) {
		lcd_dato(*str++);
	}
}

void adc_init(void) {
	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_leer(uint8_t canal) {
	ADMUX = (ADMUX & 0xF0) | (canal & 0x07);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

void spi_maestro_init(void) {
	DDRB |= (1 << PIN_MOSI) | (1 << PIN_SCK) | (1 << PIN_SS);
	DDRB &= ~(1 << PIN_MISO);
	PORTB |= (1 << PIN_SS);
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

void spi_enviar_byte(uint8_t dato) {
	SPDR = dato;
	while (!(SPSR & (1 << SPIF)));
}

void enviar_trama_esclavo(uint8_t cmd, uint8_t val) {
	PORTB &= ~(1 << PIN_SS);
	_delay_us(10);
	spi_enviar_byte(cmd);
	_delay_us(25);
	spi_enviar_byte(val);
	_delay_us(10);
	PORTB |= (1 << PIN_SS);
}

uint8_t dht11_leer_temp(void) {
	uint8_t bits[5] = {0, 0, 0, 0, 0};
	uint8_t i, j;

	DDRD |= (1 << PIN_DHT11);
	PORTD &= ~(1 << PIN_DHT11);
	_delay_ms(18);
	PORTD |= (1 << PIN_DHT11);
	_delay_us(30);

	DDRD &= ~(1 << PIN_DHT11);
	PORTD |= (1 << PIN_DHT11);

	uint16_t timeout = 10000;
	while (PIND & (1 << PIN_DHT11)) { if (--timeout == 0) return temp_motor; }
	timeout = 10000;
	while (!(PIND & (1 << PIN_DHT11))) { if (--timeout == 0) return temp_motor; }
	timeout = 10000;
	while (PIND & (1 << PIN_DHT11)) { if (--timeout == 0) return temp_motor; }

	for (i = 0; i < 5; i++) {
		for (j = 0; j < 8; j++) {
			timeout = 10000;
			while (!(PIND & (1 << PIN_DHT11))) { if (--timeout == 0) return temp_motor; }
			_delay_us(35);
			if (PIND & (1 << PIN_DHT11)) {
				bits[i] |= (1 << (7 - j));
				timeout = 10000;
				while (PIND & (1 << PIN_DHT11)) { if (--timeout == 0) return temp_motor; }
			}
		}
	}
	return bits[2];
}

int main(void) {
	DDRD &= ~(1 << PIN_BOTON);
	PORTD |= (1 << PIN_BOTON);

	adc_init();
	spi_maestro_init();
	lcd_init();

	char buffer[17];
	uint8_t ultimo_boton = 1;
	uint8_t ciclos_refresco = 0;

	lcd_set_cursor(0, 0);
	lcd_imprimir("SISTEMA INICIADO");
	_delay_ms(1000);
	lcd_comando(0x01);

	while (1) {
		uint8_t lectura_boton = (PIND & (1 << PIN_BOTON)) ? 1 : 0;
		if (ultimo_boton == 1 && lectura_boton == 0) {
			_delay_ms(20);
			if (!(PIND & (1 << PIN_BOTON))) {
				if (estado_sistema == 0) {
					estado_sistema = 1;
					enviar_trama_esclavo(CMD_START, 0x00);
					} else if (estado_sistema == 1) {
					estado_sistema = 0;
					enviar_trama_esclavo(CMD_STOP, 0x00);
				}
			}
		}
		ultimo_boton = lectura_boton;

		uint16_t lectura_pot = adc_leer(CANAL_ADC_POT);
		velocidad_pwm = (uint8_t)(lectura_pot >> 2);
		if (estado_sistema == 1) {
			enviar_trama_esclavo(CMD_SPEED, velocidad_pwm);
		}

		uint16_t lectura_ldr = adc_leer(CANAL_ADC_LDR);
		if (lectura_ldr < 400) {
			if (!ldr_bloqueado && estado_sistema == 1) {
				ldr_bloqueado = 1;
				contador_piezas++;
				enviar_trama_esclavo(CMD_PISTON, 0x01);
			}
			} else if (lectura_ldr > 600) {
			ldr_bloqueado = 0;
		}

		ciclos_refresco++;
		if (ciclos_refresco >= 5) {
			ciclos_refresco = 0;
			uint8_t t = dht11_leer_temp();
			if (t > 0 && t < 90) {
				temp_motor = t;
			}

			if (temp_motor >= TEMP_UMBRAL && estado_sistema != 2) {
				estado_sistema = 2;
				enviar_trama_esclavo(CMD_EMERGENCY, 0xFF);
			}

			lcd_set_cursor(0, 0);
			if (estado_sistema == 1) {
				sprintf(buffer, "RUN V:%3u T:%2uC", velocidad_pwm, temp_motor);
				} else if (estado_sistema == 0) {
				sprintf(buffer, "STOP    T:%2uC", temp_motor);
				} else {
				sprintf(buffer, "EMERGENCIA T:%2u", temp_motor);
			}
			lcd_imprimir(buffer);

			lcd_set_cursor(1, 0);
			sprintf(buffer, "PIEZAS: %-5u   ", contador_piezas);
			lcd_imprimir(buffer);
		}

		_delay_ms(100);
	}
}
