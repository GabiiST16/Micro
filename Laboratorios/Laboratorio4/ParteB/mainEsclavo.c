#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define CMD_START       0x01
#define CMD_STOP        0x02
#define CMD_SPEED       0x03
#define CMD_PISTON      0x04
#define CMD_EMERGENCY   0x05

#define PIN_LED_VERDE   PD2
#define PIN_LED_ROJO    PD3
#define PIN_BUZZER      PD4
#define PIN_MOTOR_IN1   PD5
#define PIN_MOTOR_IN2   PD7
#define PIN_MOTOR_EN    PD6 
#define PIN_SERVO       PB1 

volatile uint8_t cmd_recibido = 0;
volatile uint8_t val_recibido = 0;
volatile uint8_t indice_byte = 0;
volatile uint8_t trama_completa = 0;

uint8_t modo_paro_emergencia = 0;
uint8_t ejecucion_marcha = 0;
uint16_t timer_servo_retorno = 0;

void spi_esclavo_init(void) {
	DDRB |= (1 << PB4);
	DDRB &= ~((1 << PB2) | (1 << PB3) | (1 << PB5));
	SPCR = (1 << SPE) | (1 << SPIE);
}

ISR(SPI_STC_vect) {
	uint8_t byte_in = SPDR;
	if (indice_byte == 0) {
		cmd_recibido = byte_in;
		indice_byte = 1;
		} else {
		val_recibido = byte_in;
		indice_byte = 0;
		trama_completa = 1;
	}
}

void timer0_pwm_motor_init(void) {
	TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
	TCCR0B = (1 << CS01) | (1 << CS00);
	OCR0A = 0;
}

void timer1_servo_init(void) {

	TCCR1A = (1 << COM1A1) | (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
	ICR1 = 39999;
	OCR1A = 2000;
}

void mover_servo(uint8_t desviado) {
	if (desviado) {
		OCR1A = 3000;
		} else {
		OCR1A = 2000;
	}
}

int main(void) {
	DDRD |= (1 << PIN_LED_VERDE) | (1 << PIN_LED_ROJO) | (1 << PIN_BUZZER);
	DDRD |= (1 << PIN_MOTOR_IN1) | (1 << PIN_MOTOR_IN2) | (1 << PIN_MOTOR_EN);
	DDRB |= (1 << PIN_SERVO);

	PORTD |= (1 << PIN_MOTOR_IN1);
	PORTD &= ~(1 << PIN_MOTOR_IN2);

	timer0_pwm_motor_init();
	timer1_servo_init();
	spi_esclavo_init();
	sei();

	PORTD &= ~(1 << PIN_LED_VERDE);
	PORTD |= (1 << PIN_LED_ROJO);
	PORTD &= ~(1 << PIN_BUZZER);
	OCR0A = 0;

	while (1) {
		if (trama_completa) {
			uint8_t c = cmd_recibido;
			uint8_t v = val_recibido;
			trama_completa = 0;

			if (c == CMD_EMERGENCY) {
				modo_paro_emergencia = 1;
				ejecucion_marcha = 0;
				OCR0A = 0;
				PORTD &= ~(1 << PIN_LED_VERDE);
				PORTD |= (1 << PIN_LED_ROJO);
				PORTD |= (1 << PIN_BUZZER);
				mover_servo(0);
			}
			else if (!modo_paro_emergencia) {
				switch (c) {
					case CMD_START:
					ejecucion_marcha = 1;
					PORTD |= (1 << PIN_LED_VERDE);
					PORTD &= ~(1 << PIN_LED_ROJO);
					PORTD &= ~(1 << PIN_BUZZER);
					break;

					case CMD_STOP:
					ejecucion_marcha = 0;
					OCR0A = 0;
					PORTD &= ~(1 << PIN_LED_VERDE);
					PORTD |= (1 << PIN_LED_ROJO);
					PORTD &= ~(1 << PIN_BUZZER);
					mover_servo(0);
					break;

					case CMD_SPEED:
					if (ejecucion_marcha) {
						OCR0A = v;
					}
					break;

					case CMD_PISTON:
					if (ejecucion_marcha) {
						mover_servo(1);
						timer_servo_retorno = 15;
					}
					break;

					default:
					break;
				}
			}
		}

		if (timer_servo_retorno > 0) {
			timer_servo_retorno--;
			if (timer_servo_retorno == 0) {
				mover_servo(0);
			}
		}

		_delay_ms(50);
	}
}
