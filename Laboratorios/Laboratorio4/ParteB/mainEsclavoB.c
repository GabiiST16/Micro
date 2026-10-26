#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Comandos SPI - Problema B
#define CMD_START       0x01
#define CMD_STOP        0x02
#define CMD_SPEED       0x03
#define CMD_PISTON      0x04
#define CMD_EMERGENCY   0x05

// Pines en Nodo Esclavo (ATmega328P)
#define PIN_UART_TX     PD1   // Pata 31 (TXD hacia Virtual Terminal)
#define PIN_LED_VERDE   PD2   // Pata 32 (Indicador de Marcha)
#define PIN_LED_ROJO    PD3   // Pata 1  (Indicador de Paro / Falla)
#define PIN_BUZZER      PD4   // Pata 2  (Alarma sonora)
#define PIN_MOTOR_IN1   PD5   // Pata 9  (L293D IN1)
#define PIN_MOTOR_EN    PD6   // Pata 10 (L293D EN1 - OC0A Timer 0 PWM)
#define PIN_MOTOR_IN2   PD7   // Pata 11 (L293D IN2)
#define PIN_SERVO       PB1   // Pata 13 (Servomotor - OC1A Timer 1 PWM)

// Maquina de Estados de Recepcion de Trama SPI (2 Bytes)
typedef enum {
	SPI_ESPERANDO_CMD = 0,
	SPI_ESPERANDO_VAL
} estado_trama_spi_t;

// Maquina de Estados Operativa del Nodo Esclavo
typedef enum {
	ESC_PARO = 0,
	ESC_MARCHA,
	ESC_ACCION_PISTON,
	ESC_EMERGENCIA
} estado_esclavo_t;

volatile estado_trama_spi_t estado_spi = SPI_ESPERANDO_CMD;
volatile estado_esclavo_t estado_esclavo = ESC_PARO;

volatile uint8_t cmd_rx = 0;
volatile uint8_t val_rx = 0;
volatile uint16_t timer_servo = 0;
volatile uint8_t pendientes_gabied = 0;

/* UART (9600 bps a 16 MHz) para Virtual Terminal del Esclavo */
void uart_init(void) {
	UBRR0H = 0;
	UBRR0L = 103;
	UCSR0B = (1 << TXEN0);
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

/* Control de Motor DC */
static inline void motor_detener(void) {
	TCCR0A = 0; // Desconecta OC0A del pin PD6
	OCR0A = 0;
	PORTD &= ~((1 << PIN_MOTOR_IN1) | (1 << PIN_MOTOR_IN2) | (1 << PIN_MOTOR_EN));
}

static inline void motor_velocidad(uint8_t pwm) {
	if (pwm == 0) {
		TCCR0A = 0;
		PORTD &= ~(1 << PIN_MOTOR_EN);
	} else {
		PORTD |= (1 << PIN_MOTOR_IN1);
		PORTD &= ~(1 << PIN_MOTOR_IN2);
		TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
		OCR0A = pwm;
	}
}

/* Inicializacion de Perifericos */
void timer0_init(void) {
	TCCR0A = 0;
	TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 64
	OCR0A = 0;
}

void timer1_init(void) {
	TCCR1A = (1 << COM1A1) | (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Modo 14 Fast PWM, 50 Hz
	ICR1 = 39999;
	OCR1A = 2000;
}

void spi_esclavo_init(void) {
	DDRB |= (1 << PB4);
	DDRB &= ~((1 << PB2) | (1 << PB3) | (1 << PB5));
	SPCR = (1 << SPE) | (1 << SPIE);
}

/* Interrupcion SPI: Maquina de estados de recepcion y transicion de actuadores */
ISR(SPI_STC_vect) {
	uint8_t d = SPDR;

	switch (estado_spi) {
		case SPI_ESPERANDO_CMD:
			cmd_rx = d;
			estado_spi = SPI_ESPERANDO_VAL;
			break;

		case SPI_ESPERANDO_VAL:
			val_rx = d;
			estado_spi = SPI_ESPERANDO_CMD;

			// Evaluacion de transiciones del Esclavo segun el comando recibido
			if (cmd_rx == CMD_EMERGENCY) {
				estado_esclavo = ESC_EMERGENCIA;
				motor_detener();
				PORTD &= ~(1 << PIN_LED_VERDE);
				PORTD |= (1 << PIN_LED_ROJO);
				PORTD |= (1 << PIN_BUZZER);
				OCR1A = 2000;
			} 
			else if (cmd_rx == CMD_STOP) {
				estado_esclavo = ESC_PARO;
				motor_detener();
				PORTD &= ~(1 << PIN_LED_VERDE);
				PORTD |= (1 << PIN_LED_ROJO);
				PORTD &= ~(1 << PIN_BUZZER);
				OCR1A = 2000;
			} 
			else if (estado_esclavo != ESC_EMERGENCIA) {
				if (cmd_rx == CMD_START) {
					estado_esclavo = ESC_MARCHA;
					PORTD |= (1 << PIN_LED_VERDE);
					PORTD &= ~(1 << PIN_LED_ROJO);
					PORTD &= ~(1 << PIN_BUZZER);
					PORTD |= (1 << PIN_MOTOR_IN1);
					PORTD &= ~(1 << PIN_MOTOR_IN2);
				} 
				else if (cmd_rx == CMD_SPEED && (estado_esclavo == ESC_MARCHA || estado_esclavo == ESC_ACCION_PISTON)) {
					motor_velocidad(val_rx);
				} 
				else if (cmd_rx == CMD_PISTON && (estado_esclavo == ESC_MARCHA || estado_esclavo == ESC_ACCION_PISTON)) {
					estado_esclavo = ESC_ACCION_PISTON;
					OCR1A = 4000;      // Acciona servo a +90 grados
					timer_servo = 700; // 700 ms extendido
				}
			}

			pendientes_gabied++;
			break;
	}
}

int main(void) {
	DDRD |= (1 << PIN_LED_VERDE) | (1 << PIN_LED_ROJO) | (1 << PIN_BUZZER);
	DDRD |= (1 << PIN_MOTOR_IN1) | (1 << PIN_MOTOR_IN2) | (1 << PIN_MOTOR_EN);
	DDRB |= (1 << PIN_SERVO);

	motor_detener();
	PORTD &= ~(1 << PIN_LED_VERDE);
	PORTD |= (1 << PIN_LED_ROJO);
	PORTD &= ~(1 << PIN_BUZZER);

	uart_init();
	timer0_init();
	timer1_init();
	spi_esclavo_init();
	sei();

	while (1) {
		// Sincronizacion por hardware: si SS esta en alto, reiniciar estado de trama SPI
		if (PINB & (1 << PB2)) {
			estado_spi = SPI_ESPERANDO_CMD;
		}

		// Confirmacion por terminal cada vez que se recibe una trama
		if (pendientes_gabied > 0) {
			cli();
			pendientes_gabied--;
			sei();
			uart_imprimir("gabied\r\n");
		}

		// Ejecucion continua de la Maquina de Estados del Esclavo
		switch (estado_esclavo) {
			case ESC_PARO:
				break;

			case ESC_MARCHA:
				break;

			case ESC_ACCION_PISTON:
				if (timer_servo > 0) {
					timer_servo--;
					if (timer_servo == 0) {
						OCR1A = 2000; // Retorno del servo a 0 grados
						estado_esclavo = ESC_MARCHA;
					}
				}
				break;

			case ESC_EMERGENCIA:
				break;
		}

		_delay_ms(1);
	}
}
