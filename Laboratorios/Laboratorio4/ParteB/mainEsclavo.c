#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define CMD_START       0x01
#define CMD_STOP        0x02
#define CMD_SPEED       0x03
#define CMD_PISTON      0x04
#define CMD_EMERGENCY   0x05

#define PIN_UART_TX     PD1
#define PIN_LED_VERDE   PD2
#define PIN_LED_ROJO    PD3
#define PIN_BUZZER      PD4
#define PIN_MOTOR_IN1   PD5
#define PIN_MOTOR_EN    PD6
#define PIN_MOTOR_IN2   PD7
#define PIN_SERVO       PB1

volatile uint8_t byte_estado = 0;
volatile uint8_t cmd_rx = 0;
volatile uint8_t val_rx = 0;
volatile uint8_t marcha = 0;
volatile uint16_t timer_servo = 0;
volatile uint8_t pendientes_gabied = 0;

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

static inline void motor_detener(void) {
	TCCR0A = 0;
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

void timer0_init(void) {
	TCCR0A = 0;
	TCCR0B = (1 << CS01) | (1 << CS00);
	OCR0A = 0;
}

void timer1_init(void) {
	TCCR1A = (1 << COM1A1) | (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
	OCR1A = 2000;
}

void spi_esclavo_init(void) {
	DDRB |= (1 << PB4);
	DDRB &= ~((1 << PB2) | (1 << PB3) | (1 << PB5));
	SPCR = (1 << SPE) | (1 << SPIE);
}

ISR(SPI_STC_vect) {
	uint8_t d = SPDR;

	if (byte_estado == 0) {
		cmd_rx = d;
		byte_estado = 1;
	} else {
		val_rx = d;
		byte_estado = 0;

		uint8_t c = cmd_rx;
		uint8_t v = val_rx;

		if (c == CMD_START) {
			marcha = 1;
			PORTD |= (1 << PIN_LED_VERDE);
			PORTD &= ~(1 << PIN_LED_ROJO);
			PORTD &= ~(1 << PIN_BUZZER);
			PORTD |= (1 << PIN_MOTOR_IN1);
			PORTD &= ~(1 << PIN_MOTOR_IN2);
		} 
		else if (c == CMD_STOP) {
			marcha = 0;
			motor_detener();
			PORTD &= ~(1 << PIN_LED_VERDE);
			PORTD |= (1 << PIN_LED_ROJO);
			PORTD &= ~(1 << PIN_BUZZER);
			OCR1A = 2000;
		} 
		else if (c == CMD_SPEED) {
			if (marcha) {
				motor_velocidad(v);
			}
		} 
		else if (c == CMD_PISTON) {
			if (marcha) {
				OCR1A = 4000;
				timer_servo = 700;
			}
		} 
		else if (c == CMD_EMERGENCY) {
			marcha = 0;
			motor_detener();
			PORTD &= ~(1 << PIN_LED_VERDE);
			PORTD |= (1 << PIN_LED_ROJO);
			PORTD |= (1 << PIN_BUZZER);
			OCR1A = 2000;
		}

		pendientes_gabied++;
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
		if (PINB & (1 << PB2)) {
			byte_estado = 0;
		}

		if (pendientes_gabied > 0) {
			cli();
			pendientes_gabied--;
			sei();
			uart_imprimir("gabied\r\n");
		}

		if (timer_servo > 0) {
			timer_servo--;
			if (timer_servo == 0) {
				OCR1A = 2000;
			}
		}

		_delay_ms(1);
	}
}
