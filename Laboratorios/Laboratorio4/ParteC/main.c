
#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

// Definiciones de Sensores y Actuadores Locales (Nodo Planta)
#define PIN_LDR_VENTANA 0     // Sensor LDR (apertura ventana) en ADC0 (PC0)
#define PIN_PIR_MOV     PD2   // Sensor PIR o botón (presencia) en PD2 (Pin 2)

#define PIN_BUZZER      PD5   // Sirena / Buzzer en PD5 (Pin 5)
#define PIN_LED_VERDE   PD6   // LED Verde (Sistema Armado) en PD6 (Pin 6)
#define PIN_LED_ROJO    PD7   // LED Rojo (Alarma Activa) en PD7 (Pin 7)

// Botones físicos adicionales para control local/remoto
#define PIN_BTN_ARMAR   PB0   // Botón Armar/Desarmar en PB0 (Pin 8)
#define PIN_BTN_SILENC  PB1   // Botón Silenciar sirena en PB1 (Pin 9)
#define PIN_BTN_MODO    PB2   // Botón cambiar modo AUTO/MANUAL en PB2 (Pin 10)

// Umbral del LDR para detectar ventana abierta (0 a 255)
#define UMBRAL_LDR      140

// Definiciones del LCD 16x2 I2C (0x27 en PC4=SDA, PC5=SCL)
#define LCD_ADDR 0x27
#define RS_BM    (1 << 0)
#define RW_BM    (1 << 1)
#define E_BM     (1 << 2)
#define BL_BM    (1 << 3)

// Estados de la Máquina de Estados de la Alarma
#define ESTADO_DESARMADO   0
#define ESTADO_ARMADO      1
#define ESTADO_ALARMA      2
#define ESTADO_SILENCIADO  3

// Modos de operación
#define MODO_AUTO          0
#define MODO_MANUAL        1

// Zonas de detección
#define ZONA_NINGUNA       0
#define ZONA_VENTANA       1
#define ZONA_PRESENCIA     2
#define ZONA_AMBAS         3
#define ZONA_FORZADA       4

// LUT (Look-Up Table) de máscaras de LEDs (PD6=Verde, PD7=Rojo) según el estado
const uint8_t LUT_LEDS_ESTADO[4] = {
	0x00,                                      // ESTADO_DESARMADO: Ambos apagados
	(1 << PIN_LED_VERDE),                      // ESTADO_ARMADO: Verde encendido
	(1 << PIN_LED_VERDE) | (1 << PIN_LED_ROJO),// ESTADO_ALARMA: Verde y Rojo encendidos
	(1 << PIN_LED_VERDE) | (1 << PIN_LED_ROJO) // ESTADO_SILENCIADO: Verde y Rojo encendidos
};

// LUT de valores OCR0A para los 2 tonos de la sirena (Timer0 CTC con prescaler 256)
// f = 16MHz / (2 * 256 * (1 + OCR0A)) -> OCR0A=11 (~2604 Hz), OCR0A=15 (~1953 Hz)
const uint8_t LUT_SIRENA_OCR0A[2] = {11, 15};

// Variables volátiles para Timers e Interrupciones
volatile uint32_t milisegundos_contador = 0;
volatile uint8_t sirena_activa = 0;

// Buffer de recepción Bluetooth (UART RX por interrupción)
volatile char rx_buffer[32];
volatile uint8_t rx_idx = 0;
volatile uint8_t comando_listo = 0;

// Prototipos de Funciones
void Timer1_Millis_Init(void);
uint32_t get_millis(void);
void Timer0_Sirena_Init(void);
void Sirena_Configurar(uint8_t activar, uint8_t indice_tono);

void ADC_Iniciar(void);
uint8_t ADC_Leer(uint8_t canal);

void UART_Init(void);
void UART_EnviarChar(char c);
void UART_EnviarTexto(const char* texto);

void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Escribir(uint8_t data);
void LCD_Init(void);
void LCD_Comando(uint8_t cmd);
void LCD_Dato(uint8_t data);
void LCD_Cursor(uint8_t fila, uint8_t col);
void LCD_Texto(const char* s);
void LCD_Limpiar(void);
void LCD_ActualizarPantalla(uint8_t estado, uint8_t modo, uint8_t zona);

void procesarComandoBluetooth(const char* cmd, uint8_t* estado, uint8_t* modo, uint8_t* zona);
void reportarEstadoBluetooth(uint8_t estado, uint8_t modo, uint8_t zona);

int main(void){
	// Configuración de pines de Salida (LEDs y Buzzer)
	DDRD |= (1 << PIN_LED_VERDE) | (1 << PIN_LED_ROJO) | (1 << PIN_BUZZER);
	PORTD &= ~((1 << PIN_LED_VERDE) | (1 << PIN_LED_ROJO) | (1 << PIN_BUZZER));

	// Configuración de pines de Entrada (PIR en PD2 y Botones en PB0, PB1, PB2)
	DDRD &= ~(1 << PIN_PIR_MOV);
	PORTD |= (1 << PIN_PIR_MOV);

	DDRB &= ~((1 << PIN_BTN_ARMAR) | (1 << PIN_BTN_SILENC) | (1 << PIN_BTN_MODO));
	PORTB |= (1 << PIN_BTN_ARMAR) | (1 << PIN_BTN_SILENC) | (1 << PIN_BTN_MODO);

	// Iniciar periféricos y Timers
	ADC_Iniciar();
	UART_Init();
	I2C_Init();
	LCD_Init();
	Timer1_Millis_Init();
	Timer0_Sirena_Init();
	_delay_ms(20);

	// Guardar estado inicial de las entradas para detectar flancos sin bloquear
	uint8_t reposo_pir   = (PIND & (1 << PIN_PIR_MOV)) ? 1 : 0;
	uint8_t ant_armar    = (PINB & (1 << PIN_BTN_ARMAR)) ? 1 : 0;
	uint8_t ant_silenc   = (PINB & (1 << PIN_BTN_SILENC)) ? 1 : 0;
	uint8_t ant_modo     = (PINB & (1 << PIN_BTN_MODO)) ? 1 : 0;

	// Habilitar interrupciones globales (Timer1, Timer0 y USART RX)
	sei();

	uint8_t estado = ESTADO_ARMADO;
	uint8_t modo   = MODO_AUTO;
	uint8_t zona   = ZONA_NINGUNA;

	uint8_t estado_prev = 0xFF;
	uint8_t modo_prev   = 0xFF;
	uint8_t zona_prev   = 0xFF;

	uint32_t t_ultimo_reporte = 0;
	uint32_t t_ultimo_tono    = 0;
	uint8_t indice_tono       = 0;

	UART_EnviarTexto("\r\n--- SISTEMA DE ALARMA ANTIRROBO HC-05 ---\r\n");
	UART_EnviarTexto("Comandos: ARMAR (A), DESARMAR (D), SILENCIAR (S), AUTO (U), MANUAL (M), FORZAR (F)\r\n");
	reportarEstadoBluetooth(estado, modo, zona);
	LCD_ActualizarPantalla(estado, modo, zona);

	while (1){
		uint32_t t_actual = get_millis();

		// 1. Leer sensores (LDR de apertura de ventana y PIR/botón de presencia)
		uint8_t valor_ldr = ADC_Leer(PIN_LDR_VENTANA);
		uint8_t act_pir   = (PIND & (1 << PIN_PIR_MOV)) ? 1 : 0;

		bool ventana_abierta = (valor_ldr > UMBRAL_LDR);
		bool presencia_det   = (act_pir != reposo_pir);

		uint8_t zona_detectada = ZONA_NINGUNA;
		if (ventana_abierta && presencia_det) {
			zona_detectada = ZONA_AMBAS;
		} else if (ventana_abierta) {
			zona_detectada = ZONA_VENTANA;
		} else if (presencia_det) {
			zona_detectada = ZONA_PRESENCIA;
		}

		// 2. Leer botones físicos locales
		uint8_t act_armar  = (PINB & (1 << PIN_BTN_ARMAR)) ? 1 : 0;
		uint8_t act_silenc = (PINB & (1 << PIN_BTN_SILENC)) ? 1 : 0;
		uint8_t act_modo   = (PINB & (1 << PIN_BTN_MODO)) ? 1 : 0;

		if (act_armar != ant_armar) {
			_delay_ms(30);
			uint16_t esp = 0;
			while((((PINB & (1 << PIN_BTN_ARMAR)) ? 1 : 0) == act_armar) && esp < 600) { _delay_ms(10); esp += 10; }
			ant_armar = (PINB & (1 << PIN_BTN_ARMAR)) ? 1 : 0;

			if (modo == MODO_MANUAL) {
				if (estado == ESTADO_DESARMADO) {
					estado = ESTADO_ARMADO;
					zona = ZONA_NINGUNA;
				} else {
					estado = ESTADO_DESARMADO;
					zona = ZONA_NINGUNA;
				}
			}
		}

		if (act_silenc != ant_silenc) {
			_delay_ms(30);
			uint16_t esp = 0;
			while((((PINB & (1 << PIN_BTN_SILENC)) ? 1 : 0) == act_silenc) && esp < 600) { _delay_ms(10); esp += 10; }
			ant_silenc = (PINB & (1 << PIN_BTN_SILENC)) ? 1 : 0;

			if (modo == MODO_MANUAL && estado == ESTADO_ALARMA) {
				estado = ESTADO_SILENCIADO;
			}
		}

		if (act_modo != ant_modo) {
			_delay_ms(30);
			uint16_t esp = 0;
			while((((PINB & (1 << PIN_BTN_MODO)) ? 1 : 0) == act_modo) && esp < 600) { _delay_ms(10); esp += 10; }
			ant_modo = (PINB & (1 << PIN_BTN_MODO)) ? 1 : 0;

			modo = (modo == MODO_AUTO) ? MODO_MANUAL : MODO_AUTO;
			if (modo == MODO_AUTO && estado == ESTADO_DESARMADO) {
				estado = ESTADO_ARMADO;
			}
		}

		// 3. Procesar comandos recibidos por interrupción Bluetooth (HC-05)
		if (comando_listo) {
			char cmd_copia[32];
			cli();
			strncpy(cmd_copia, (const char*)rx_buffer, sizeof(cmd_copia));
			rx_idx = 0;
			comando_listo = 0;
			sei();

			procesarComandoBluetooth(cmd_copia, &estado, &modo, &zona);
		}

		// 4. Máquina de Estados del Sistema de Alarma
		switch (estado) {

			case ESTADO_DESARMADO:
				Sirena_Configurar(0, 0);
				if (modo == MODO_AUTO) {
					// En modo AUTO el nodo se mantiene armado automáticamente
					estado = ESTADO_ARMADO;
					zona = ZONA_NINGUNA;
				}
				break;

			case ESTADO_ARMADO:
				Sirena_Configurar(0, 0);
				if (modo == MODO_AUTO) {
					// En modo AUTO cualquier intrusión dispara la sirena sola
					if (zona_detectada != ZONA_NINGUNA) {
						zona = zona_detectada;
						estado = ESTADO_ALARMA;
					}
				} else {
					// En modo MANUAL solo reporta la zona; el operador decide
					zona = zona_detectada;
				}
				break;

			case ESTADO_ALARMA:
				if (zona_detectada != ZONA_NINGUNA) {
					zona = zona_detectada;
				}
				// Alternar los 2 tonos de la LUT cada 150 ms usando Timer1 sin bloquear
				if (t_actual - t_ultimo_tono >= 150) {
					t_ultimo_tono = t_actual;
					indice_tono ^= 1;
					Sirena_Configurar(1, indice_tono);
				}
				break;

			case ESTADO_SILENCIADO:
				Sirena_Configurar(0, 0);
				if (modo == MODO_AUTO && zona_detectada == ZONA_NINGUNA) {
					zona = ZONA_NINGUNA;
					estado = ESTADO_ARMADO;
				}
				break;
		}

		// Actualizar LEDs aplicando la LUT de estados
		PORTD = (PORTD & ~((1 << PIN_LED_VERDE) | (1 << PIN_LED_ROJO))) | LUT_LEDS_ESTADO[estado];

		// 5. Si cambió el estado, modo o zona, actualizar LCD y enviar trama por Bluetooth
		if (estado != estado_prev || modo != modo_prev || zona != zona_prev) {
			LCD_ActualizarPantalla(estado, modo, zona);
			reportarEstadoBluetooth(estado, modo, zona);
			estado_prev = estado;
			modo_prev   = modo;
			zona_prev   = zona;
		}

		// Reporte periódico por Bluetooth cada 3000 ms medidos con Timer1
		if (t_actual - t_ultimo_reporte >= 3000) {
			t_ultimo_reporte = t_actual;
			reportarEstadoBluetooth(estado, modo, zona);
		}
	}
}

// Funciones de Timers e Interrupciones

// Timer1 en modo CTC a 1 kHz (interrupción cada 1 ms)
void Timer1_Millis_Init(void) {
	TCCR1A = 0x00;
	TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC, Prescaler 64
	OCR1A  = 249;                                      // 16MHz / (64 * 250) = 1000 Hz
	TIMSK1 = (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect) {
	milisegundos_contador++;
}

uint32_t get_millis(void) {
	uint32_t ms;
	cli();
	ms = milisegundos_contador;
	sei();
	return ms;
}

// Timer0 en modo CTC para generar la frecuencia de la sirena por interrupción
void Timer0_Sirena_Init(void) {
	TCCR0A = (1 << WGM01); // Modo CTC
	TCCR0B = 0x00;         // Apagado inicialmente
	OCR0A  = LUT_SIRENA_OCR0A[0];
	TIMSK0 = (1 << OCIE0A);
}

void Sirena_Configurar(uint8_t activar, uint8_t indice_tono) {
	if (activar) {
		OCR0A = LUT_SIRENA_OCR0A[indice_tono & 0x01];
		sirena_activa = 1;
		TCCR0B = (1 << CS02); // Prescaler 256 encendido
	} else {
		sirena_activa = 0;
		TCCR0B = 0x00;        // Detener Timer0
		PORTD &= ~(1 << PIN_BUZZER);
	}
}

ISR(TIMER0_COMPA_vect) {
	if (sirena_activa) {
		PORTD ^= (1 << PIN_BUZZER);
	}
}

// Interrupción de recepción Serial / Bluetooth HC-05
ISR(USART_RX_vect) {
	char c = UDR0;
	if (c == '\r' || c == '\n') {
		if (rx_idx > 0) {
			rx_buffer[rx_idx] = '\0';
			comando_listo = 1;
		}
	} else {
		if (rx_idx < sizeof(rx_buffer) - 1) {
			rx_buffer[rx_idx++] = c;
			if (rx_idx == 1 && (c == 'A' || c == 'a' || c == 'D' || c == 'd' ||
			                    c == 'S' || c == 's' || c == 'M' || c == 'm' ||
			                    c == 'U' || c == 'u' || c == 'F' || c == 'f')) {
				rx_buffer[1] = '\0';
				comando_listo = 1;
			}
		}
	}
}

// Funciones de Procesamiento de Tramas Bluetooth

void procesarComandoBluetooth(const char* cmd, uint8_t* estado, uint8_t* modo, uint8_t* zona) {
	bool es_auto   = (strcmp(cmd, "U") == 0 || strcmp(cmd, "u") == 0 || strstr(cmd, "AUTO") != NULL);
	bool es_manual = (strcmp(cmd, "M") == 0 || strcmp(cmd, "m") == 0 || strstr(cmd, "MANUAL") != NULL);

	if (es_auto) {
		*modo = MODO_AUTO;
		if (*estado == ESTADO_DESARMADO) {
			*estado = ESTADO_ARMADO;
		}
		return;
	}
	if (es_manual) {
		*modo = MODO_MANUAL;
		return;
	}

	bool es_armar  = (strcmp(cmd, "A") == 0 || strcmp(cmd, "a") == 0 || strstr(cmd, "ARMAR") != NULL);
	bool es_desarm = (strcmp(cmd, "D") == 0 || strcmp(cmd, "d") == 0 || strstr(cmd, "DESARMAR") != NULL);
	bool es_silenc = (strcmp(cmd, "S") == 0 || strcmp(cmd, "s") == 0 || strstr(cmd, "SILENCIAR") != NULL);
	bool es_forzar = (strcmp(cmd, "F") == 0 || strcmp(cmd, "f") == 0 || strstr(cmd, "FORZAR") != NULL);

	if (es_desarm || es_armar || es_silenc || es_forzar) {
		if (*modo != MODO_MANUAL) {
			UART_EnviarTexto("IGNORADO: modo AUTO. Envie M para modo MANUAL\r\n");
			return;
		}
		if (es_desarm) {
			*estado = ESTADO_DESARMADO;
			*zona = ZONA_NINGUNA;
		} else if (es_armar) {
			*estado = ESTADO_ARMADO;
			*zona = ZONA_NINGUNA;
		} else if (es_silenc) {
			if (*estado == ESTADO_ALARMA) {
				*estado = ESTADO_SILENCIADO;
			}
		} else if (es_forzar) {
			if (*estado != ESTADO_DESARMADO) {
				*estado = ESTADO_ALARMA;
				if (*zona == ZONA_NINGUNA) *zona = ZONA_FORZADA;
			} else {
				UART_EnviarTexto("IGNORADO: sistema DESARMADO. Envie A para armar\r\n");
			}
		}
	}
}

void reportarEstadoBluetooth(uint8_t estado, uint8_t modo, uint8_t zona) {
	const char* str_alarma = (estado == ESTADO_ALARMA) ? "ON" :
	                         (estado == ESTADO_SILENCIADO) ? "SILENCIADA" : "OFF";
	const char* str_sis    = (estado == ESTADO_DESARMADO) ? "DESARMADO" : "ARMADO";
	const char* str_modo   = (modo == MODO_AUTO) ? "AUTO" : "MANUAL";
	const char* str_zona   = (zona == ZONA_VENTANA) ? "VENTANA" :
	                         (zona == ZONA_PRESENCIA) ? "PRESENCIA" :
	                         (zona == ZONA_AMBAS) ? "VENTANA+PIR" :
	                         (zona == ZONA_FORZADA) ? "FORZADA" : "NINGUNA";

	char trama[80];
	snprintf(trama, sizeof(trama), "ALARMA:%s;ZONA:%s;SISTEMA:%s;MODO:%s\r\n",
	         str_alarma, str_zona, str_sis, str_modo);
	UART_EnviarTexto(trama);
}

// Funciones del ADC (Lectura de LDR)

void ADC_Iniciar(void) {
	ADMUX = (1 << REFS0) | (1 << ADLAR);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint8_t ADC_Leer(uint8_t canal) {
	ADMUX = (ADMUX & 0xF0) | (canal & 0x0F);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADCH;
}

// Funciones UART para Módulo Bluetooth HC-05 (9600 baudios)

void UART_Init(void) {
	uint16_t ubrr = 103; // 9600 bps a 16 MHz
	UBRR0H = (uint8_t)(ubrr >> 8);
	UBRR0L = (uint8_t)ubrr;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void UART_EnviarChar(char c) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;
}

void UART_EnviarTexto(const char* texto) {
	while (*texto) {
		UART_EnviarChar(*texto++);
	}
}

// Funciones I2C y Pantalla LCD 16x2

void I2C_Init(void) {
	PORTC |= (1 << PC4) | (1 << PC5); // Pull-ups internos por si no está conectado el LCD I2C
	TWSR = 0x00;
	TWBR = 72; // 100 kHz
	TWCR = (1 << TWEN);
}

void I2C_Start(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	uint16_t timeout = 500;
	while (!(TWCR & (1 << TWINT)) && --timeout);
}

void I2C_Stop(void) {
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void I2C_Escribir(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	uint16_t timeout = 500;
	while (!(TWCR & (1 << TWINT)) && --timeout);
}

static void PCF8574_Escribir(uint8_t val) {
	I2C_Start();
	I2C_Escribir(LCD_ADDR << 1);
	I2C_Escribir(val);
	I2C_Stop();
}

static void LCD_EnviarNibble(uint8_t nibble, uint8_t rs) {
	uint8_t out = ((nibble & 0x0F) << 4) | rs | BL_BM;
	PCF8574_Escribir(out);
	_delay_us(1);
	PCF8574_Escribir(out | E_BM);
	_delay_us(1);
	PCF8574_Escribir(out);
	_delay_us(40);
}

void LCD_Comando(uint8_t cmd) {
	LCD_EnviarNibble(cmd >> 4, 0);
	LCD_EnviarNibble(cmd & 0x0F, 0);
	if (cmd == 0x01 || cmd == 0x02) _delay_ms(2);
	else _delay_us(50);
}

void LCD_Dato(uint8_t data) {
	LCD_EnviarNibble(data >> 4, RS_BM);
	LCD_EnviarNibble(data & 0x0F, RS_BM);
	_delay_us(50);
}

void LCD_Init(void) {
	_delay_ms(20);
	LCD_EnviarNibble(0x03, 0); _delay_ms(5);
	LCD_EnviarNibble(0x03, 0); _delay_ms(1);
	LCD_EnviarNibble(0x03, 0); _delay_us(150);
	LCD_EnviarNibble(0x02, 0); _delay_us(150);
	LCD_Comando(0x28);
	LCD_Comando(0x08);
	LCD_Comando(0x01);
	LCD_Comando(0x06);
	LCD_Comando(0x0C);
}

void LCD_Limpiar(void) {
	LCD_Comando(0x01);
}

void LCD_Cursor(uint8_t fila, uint8_t col) {
	uint8_t base = (fila == 0) ? 0x00 : 0x40;
	LCD_Comando(0x80 | (base + col));
}

void LCD_Texto(const char* s) {
	while (*s) {
		LCD_Dato((uint8_t)*s++);
	}
}

void LCD_ActualizarPantalla(uint8_t estado, uint8_t modo, uint8_t zona) {
	char l1[17];
	char l2[17];

	const char* s_est  = (estado == ESTADO_DESARMADO) ? "DESARM" :
	                     (estado == ESTADO_ARMADO) ? "ARMADO" :
	                     (estado == ESTADO_ALARMA) ? "ALARMA" : "SILENC";
	const char* s_modo = (modo == MODO_AUTO) ? "AUTO" : "MAN";
	const char* s_zona = (zona == ZONA_VENTANA) ? "VENTANA" :
	                     (zona == ZONA_PRESENCIA) ? "PIR/MOV" :
	                     (zona == ZONA_AMBAS) ? "VEN+PIR" :
	                     (zona == ZONA_FORZADA) ? "FORZADA" : "OK";

	snprintf(l1, sizeof(l1), "EST:%-6s M:%-4s", s_est, s_modo);
	snprintf(l2, sizeof(l2), "ZONA:%-11s", s_zona);

	LCD_Limpiar();
	LCD_Cursor(0, 0);
	LCD_Texto(l1);
	LCD_Cursor(1, 0);
	LCD_Texto(l2);
}
