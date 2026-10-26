#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

// Comandos SPI - Problema B (Trama de 2 bytes: [COMANDO][VALOR])
#define CMD_START       0x01
#define CMD_STOP        0x02
#define CMD_SPEED       0x03
#define CMD_PISTON      0x04
#define CMD_EMERGENCY   0x05

#define TEMP_LIMITE     50

// Pines en Nodo Maestro (ATmega328P)
#define PIN_LDR_DIG     PD0   // Pata 30 (Divisor LDR1 + R1)
#define PIN_UART_TX     PD1   // Pata 31 (TXD hacia Virtual Terminal)
#define PIN_BOTON       PD2   // Pata 32 (Boton Start/Stop - INT0)
#define PIN_DHT11       PD3   // Pata 1  (Sensor DHT11 DATA)

#define PIN_SS          PB2   // Pata 14 (PB2/SS)
#define PIN_MOSI        PB3   // Pata 15 (PB3/MOSI)
#define PIN_MISO        PB4   // Pata 16 (PB4/MISO)
#define PIN_SCK         PB5   // Pata 17 (PB5/SCK)

// Pines Pantalla LCD 16x2 (Modo 4 bits)
#define LCD_RS          PD4   // Pata 2
#define LCD_EN          PD5   // Pata 9
#define LCD_D4          PD6   // Pata 10
#define LCD_D5          PD7   // Pata 11
#define LCD_D6          PB0   // Pata 12
#define LCD_D7          PB1   // Pata 13

// Definicion de la Maquina de Estados del Nodo Maestro
typedef enum {
	ESTADO_PARO = 0,
	ESTADO_MARCHA,
	ESTADO_DESVIO_PIEZA,
	ESTADO_EMERGENCIA
} estado_maestro_t;

estado_maestro_t estado_actual = ESTADO_PARO;

uint16_t contador_piezas = 0;
uint8_t velocidad_pwm = 0;
uint8_t velocidad_enviada = 255;
uint8_t temp_motor = 25;
uint8_t ldr_base = 0;
uint8_t ldr_bloqueado = 0;
volatile uint8_t evento_boton = 0;
const char *ultimo_cmd_str = "STOP ";

/* UART (9600 bps a 16 MHz) para Virtual Terminal */
void uart_init(void) {
	UBRR0H = 0;
	UBRR0L = 103;
	UCSR0B = (1 << TXEN0); // Solo TX (PD1) para dejar PD0 libre al LDR
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_enviar_char(char c) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;
}

void uart_imprimir(const char *str) {
	while (*str) {
		uart_enviar_char(*str++);
	}
}

/* Interrupcion externa INT0 para el boton Start/Stop */
ISR(INT0_vect) {
	evento_boton = 1;
}

/* Control de Pantalla LCD 16x2 */
static void lcd_pulso(void) {
	PORTD |= (1 << LCD_EN);
	_delay_us(5);
	PORTD &= ~(1 << LCD_EN);
	_delay_us(40);
}

static void lcd_enviar_nibble(uint8_t n) {
	if (n & 1) PORTD |= (1 << LCD_D4); else PORTD &= ~(1 << LCD_D4);
	if (n & 2) PORTD |= (1 << LCD_D5); else PORTD &= ~(1 << LCD_D5);
	if (n & 4) PORTB |= (1 << LCD_D6); else PORTB &= ~(1 << LCD_D6);
	if (n & 8) PORTB |= (1 << LCD_D7); else PORTB &= ~(1 << LCD_D7);
	lcd_pulso();
}

void lcd_cmd(uint8_t cmd) {
	PORTD &= ~(1 << LCD_RS);
	lcd_enviar_nibble(cmd >> 4);
	lcd_enviar_nibble(cmd & 0x0F);
	_delay_ms(2);
}

void lcd_data(char d) {
	PORTD |= (1 << LCD_RS);
	lcd_enviar_nibble(d >> 4);
	lcd_enviar_nibble(d & 0x0F);
	_delay_us(40);
}

void lcd_init(void) {
	DDRD |= (1 << LCD_RS) | (1 << LCD_EN) | (1 << LCD_D4) | (1 << LCD_D5);
	DDRB |= (1 << LCD_D6) | (1 << LCD_D7);
	_delay_ms(20);
	PORTD &= ~(1 << LCD_RS);
	lcd_enviar_nibble(0x03); _delay_ms(5);
	lcd_enviar_nibble(0x03); _delay_us(150);
	lcd_enviar_nibble(0x03);
	lcd_enviar_nibble(0x02);
	lcd_cmd(0x28);
	lcd_cmd(0x0C);
	lcd_cmd(0x06);
	lcd_cmd(0x01);
	_delay_ms(2);
}

void lcd_texto(uint8_t fila, uint8_t col, const char *str) {
	lcd_cmd((fila == 0) ? (0x80 + col) : (0xC0 + col));
	while (*str) lcd_data(*str++);
}

/* Conversor Analogico Digital (ADC) */
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

/* Comunicacion SPI Maestro */
void spi_init(void) {
	DDRB |= (1 << PIN_MOSI) | (1 << PIN_SCK) | (1 << PIN_SS);
	DDRB &= ~(1 << PIN_MISO);
	PORTB |= (1 << PIN_SS);
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1); // Fosc/64
}

void spi_tx_byte(uint8_t b) {
	SPDR = b;
	while (!(SPSR & (1 << SPIF)));
}

void enviar_trama(uint8_t cmd, uint8_t val) {
	PORTB &= ~(1 << PIN_SS);
	_delay_us(200);
	spi_tx_byte(cmd);
	_delay_us(300);
	spi_tx_byte(val);
	_delay_us(200);
	PORTB |= (1 << PIN_SS);
	_delay_ms(2);

	uart_imprimir("jacinto\r\n");
}

/* Lectura de Temperatura del Sensor DHT11 */
uint8_t dht11_leer(void) {
	uint8_t bits[5] = {0, 0, 0, 0, 0};
	uint8_t i, j;
	uint16_t timeout;

	DDRD |= (1 << PIN_DHT11);
	PORTD &= ~(1 << PIN_DHT11);
	_delay_ms(18);
	PORTD |= (1 << PIN_DHT11);
	_delay_us(30);

	DDRD &= ~(1 << PIN_DHT11);
	PORTD |= (1 << PIN_DHT11);

	timeout = 1000;
	while (PIND & (1 << PIN_DHT11)) { if (--timeout == 0) return temp_motor; }
	timeout = 1000;
	while (!(PIND & (1 << PIN_DHT11))) { if (--timeout == 0) return temp_motor; }
	timeout = 1000;
	while (PIND & (1 << PIN_DHT11)) { if (--timeout == 0) return temp_motor; }

	for (i = 0; i < 5; i++) {
		for (j = 0; j < 8; j++) {
			timeout = 1000;
			while (!(PIND & (1 << PIN_DHT11))) { if (--timeout == 0) return temp_motor; }
			_delay_us(35);
			if (PIND & (1 << PIN_DHT11)) {
				bits[i] |= (1 << (7 - j));
				timeout = 1000;
				while (PIND & (1 << PIN_DHT11)) { if (--timeout == 0) return temp_motor; }
			}
		}
	}
	return bits[2];
}

int main(void) {
	DDRD &= ~((1 << PIN_BOTON) | (1 << PIN_LDR_DIG));
	PORTD |= (1 << PIN_BOTON);

	EICRA = (1 << ISC01);
	EIMSK = (1 << INT0);

	uart_init();
	adc_init();
	spi_init();
	lcd_init();
	sei();

	char buf[17];
	uint8_t btn_prev = 1;
	uint8_t ciclos_dht = 0;

	lcd_texto(0, 0, "SISTEMA LISTO   ");
	_delay_ms(400);
	lcd_cmd(0x01);

	ldr_base = (PIND & (1 << PIN_LDR_DIG)) ? 1 : 0;

	estado_actual = ESTADO_PARO;
	enviar_trama(CMD_STOP, 0);
	ultimo_cmd_str = "STOP ";

	while (1) {
		// Lectura de entradas y sensores comunes
		uint8_t btn_actual = (PIND & (1 << PIN_BOTON)) ? 1 : 0;
		uint8_t boton_presionado = 0;
		if (evento_boton || (btn_prev == 1 && btn_actual == 0)) {
			_delay_ms(25);
			evento_boton = 0;
			boton_presionado = 1;
		}
		btn_prev = btn_actual;

		velocidad_pwm = (uint8_t)(adc_leer(1) >> 2);
		uint8_t ldr_actual = (PIND & (1 << PIN_LDR_DIG)) ? 1 : 0;

		ciclos_dht++;
		if (ciclos_dht >= 15) {
			ciclos_dht = 0;
			uint8_t t = dht11_leer();
			if (t > 0 && t < 95) {
				temp_motor = t;
			}
		}

		// =====================================================
		// MAQUINA DE ESTADOS FINITOS (NODO MAESTRO)
		// =====================================================
		switch (estado_actual) {
			case ESTADO_PARO:
				if (temp_motor >= TEMP_LIMITE) {
					enviar_trama(CMD_EMERGENCY, 0xFF);
					ultimo_cmd_str = "EMERG";
					estado_actual = ESTADO_EMERGENCIA;
				} 
				else if (boton_presionado) {
					ldr_base = ldr_actual;
					ldr_bloqueado = 0;
					enviar_trama(CMD_START, 0);
					ultimo_cmd_str = "START";
					enviar_trama(CMD_SPEED, velocidad_pwm);
					velocidad_enviada = velocidad_pwm;
					estado_actual = ESTADO_MARCHA;
				}
				break;

			case ESTADO_MARCHA:
				if (temp_motor >= TEMP_LIMITE) {
					enviar_trama(CMD_EMERGENCY, 0xFF);
					ultimo_cmd_str = "EMERG";
					estado_actual = ESTADO_EMERGENCIA;
				} 
				else if (boton_presionado) {
					enviar_trama(CMD_STOP, 0);
					ultimo_cmd_str = "STOP ";
					estado_actual = ESTADO_PARO;
				} 
				else {
					// Verificar si se detecta una pieza en el LDR
					if (ldr_actual != ldr_base) {
						if (!ldr_bloqueado) {
							estado_actual = ESTADO_DESVIO_PIEZA;
						}
					} else {
						ldr_bloqueado = 0;
					}

					// Ajuste dinamico de velocidad por potenciometro
					int16_t diff = (int16_t)velocidad_pwm - (int16_t)velocidad_enviada;
					if (diff >= 2 || diff <= -2) {
						velocidad_enviada = velocidad_pwm;
						enviar_trama(CMD_SPEED, velocidad_pwm);
						ultimo_cmd_str = "SPEED";
					}
				}
				break;

			case ESTADO_DESVIO_PIEZA:
				ldr_bloqueado = 1;
				contador_piezas++;
				enviar_trama(CMD_PISTON, 1);
				ultimo_cmd_str = "PISTN";
				estado_actual = ESTADO_MARCHA;
				break;

			case ESTADO_EMERGENCIA:
				if (boton_presionado && temp_motor < TEMP_LIMITE) {
					enviar_trama(CMD_STOP, 0);
					ultimo_cmd_str = "STOP ";
					estado_actual = ESTADO_PARO;
				}
				break;

			default:
				estado_actual = ESTADO_PARO;
				break;
		}

		// Actualizacion de interfaz LCD segun el estado actual
		if (estado_actual == ESTADO_MARCHA || estado_actual == ESTADO_DESVIO_PIEZA) {
			sprintf(buf, "RUN  V:%3u T:%2uC", velocidad_pwm, temp_motor);
		} else if (estado_actual == ESTADO_PARO) {
			sprintf(buf, "STOP V:%3u T:%2uC", velocidad_pwm, temp_motor);
		} else {
			sprintf(buf, "EMERGENCIA T:%2uC", temp_motor);
		}
		lcd_texto(0, 0, buf);

		sprintf(buf, "P:%-3u CMD:%s ", contador_piezas, ultimo_cmd_str);
		lcd_texto(1, 0, buf);

		_delay_ms(30);
	}
}
