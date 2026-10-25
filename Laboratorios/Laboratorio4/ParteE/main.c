
#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <util/twi.h>

// Definiciones de la Matriz LED
#define PIN_MATRIZ 6    // Pin de datos PD6
#define ANCHO 8
#define ALTO 8
#define TOTAL_LEDS (ANCHO * ALTO)

// Definiciones de Botones y Buzzer
#define PIN_BOTON_HOLD PB0
#define PIN_BOTON_CAL  PB1
#define PUERTO_BOTON   PORTB
#define DDR_BOTON      DDRB
#define PIN_REG_BOTON  PINB
#define PIN_BUZZER     PD5

// Definiciones del MPU-6050
#define MPU6050_ADDR     0x68
#define MPU_SCL_PIN      PC5  // Pin SCL
#define MPU_SDA_PIN      PC4  // Pin SDA

#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_XOUT_H 0x3B

// Parámetros de inclinación y EEPROM
#define ANGULO_LIMITE    30.0f
#define ZONA_MUERTA      3.0f
#define RAD_A_GRADOS     57.2957795f
#define CLAVE_EEPROM     0xB6

// Inversión del sentido de la burbuja en el eje Y (1 = invertido, 0 = normal)
#define INVERTIR_EJE_Y   1

// Estados del Sistema
#define ESTADO_MEDICION   0
#define ESTADO_HOLD       1
#define ESTADO_CALIBRANDO 2

// Índices de color en la LUT
#define COLOR_NIVELADO  0
#define COLOR_EJE_X     1
#define COLOR_EJE_Y     2
#define COLOR_COMBINADO 3

// LUT 1: Umbrales de ángulo en grados para mapear [-30°..+30°] a las posiciones [0..7] de la matriz
const int8_t LUT_UMBRALES_ANGULO[7] = { -22, -15, -7, 0, 7, 15, 22 };

// LUT 2: Colores RGB según el estado de inclinación {R, G, B}
const uint8_t LUT_COLORES_EJE[4][3] = {
	{   0,   0, 120 }, // 0: Azul (Nivelado dentro de zona muerta)
	{ 120,   0,   0 }, // 1: Rojo (Predomina inclinación en X)
	{   0, 120,   0 }, // 2: Verde (Predomina inclinación en Y)
	{ 110, 110,   0 }  // 3: Amarillo (Inclinación combinada XY)
};

// LUT 3: Desplazamiento de fila para la matriz 8x8 (evita multiplicar y * 8)
const uint8_t LUT_FILA_OFFSET[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };

// Buffer global para los colores de la matriz
uint8_t matriz[TOTAL_LEDS][3];

// Variables globales modificadas en interrupciones (volatile)
volatile uint32_t sistema_ms = 0;
volatile uint16_t buzzer_ms_restantes = 0;

// Prototipos de Funciones
void Timer1_Millis_Init(void);
void Timer0_Buzzer_Init(void);
uint32_t get_millis(void);
void esperar_ms(uint16_t ms);

void sendBit(uint8_t bitVal);
void sendByte(uint8_t byte);
void show(uint8_t (*colors)[3]);
void setLedRGB(uint8_t (*leds)[3], int ledIndex, uint8_t r, uint8_t g, uint8_t b);
void fillAllLedsRGB(uint8_t r, uint8_t g, uint8_t b);

int coord_a_indice(uint8_t x, uint8_t y);
uint8_t mapearAngulo(float angulo);
void cargarOffsetEEPROM(float* off_x, float* off_y);
void guardarOffsetEEPROM(float off_x, float off_y);
void pitarBuzzer(uint8_t duracion_ms);

void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Escribir(uint8_t data);
uint8_t I2C_LeerACK(void);
uint8_t I2C_LeerNACK(void);
void MPU6050_EscribirByte(uint8_t reg, uint8_t data);
void MPU6050_LeerBytes(uint8_t reg, uint8_t* buffer, uint8_t len);
void MPU6050_Init(void);
void MPU6050_LeerDatos(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);

void UART_Init(void);
void UART_EnviarTexto(const char* texto);

// Interrupción del Timer1: Base de tiempo de 1 ms y apagado automático del buzzer
ISR(TIMER1_COMPA_vect) {
	sistema_ms++;
	if (buzzer_ms_restantes > 0) {
		buzzer_ms_restantes--;
		if (buzzer_ms_restantes == 0) {
			TCCR0B = 0; // Apagar Timer0
			PORTD &= ~(1 << PIN_BUZZER);
		}
	}
}

// Interrupción del Timer0: Conmuta el pin del buzzer (~1 kHz) sin bloquear el CPU
ISR(TIMER0_COMPA_vect) {
	PORTD ^= (1 << PIN_BUZZER);
}

int main(void){
	// Configuración de pines
	DDRD |= (1 << PIN_MATRIZ) | (1 << PIN_BUZZER);
	
	DDR_BOTON &= ~((1 << PIN_BOTON_HOLD) | (1 << PIN_BOTON_CAL));
	PUERTO_BOTON |= (1 << PIN_BOTON_HOLD) | (1 << PIN_BOTON_CAL);
	
	// Iniciar periféricos y Timers
	I2C_Init();
	MPU6050_Init();
	UART_Init();
	Timer1_Millis_Init();
	Timer0_Buzzer_Init();
	sei(); // Habilitar interrupciones globales
	esperar_ms(20);

	// Guardamos el estado inicial de los botones para evitar bloqueos por pull-up/pull-down
	uint8_t ant_hold = (PIN_REG_BOTON & (1 << PIN_BOTON_HOLD)) ? 1 : 0;
	uint8_t ant_cal  = (PIN_REG_BOTON & (1 << PIN_BOTON_CAL)) ? 1 : 0;

	int led_central = coord_a_indice(4, 4);
	setLedRGB(matriz, led_central, LUT_COLORES_EJE[COLOR_NIVELADO][0], LUT_COLORES_EJE[COLOR_NIVELADO][1], LUT_COLORES_EJE[COLOR_NIVELADO][2]);
	show(matriz);
	
	int16_t accl_x, accl_y, accl_z;
	int16_t gyro_x, gyro_y, gyro_z;
	uint8_t act_hold, act_cal;
	uint8_t boton_hold_presionado;
	uint8_t boton_cal_presionado;
	uint8_t estado = ESTADO_MEDICION;
	uint8_t parpadeo_borde = 0;

	uint8_t idx_color = COLOR_NIVELADO;
	int indiceLed;
	uint8_t pos_x = 4;
	uint8_t pos_y = 4;

	float angulo_x = 0.0f, angulo_y = 0.0f;
	float offset_x = 0.0f, offset_y = 0.0f;
	float incl_x = 0.0f, incl_y = 0.0f;

	// Leer offset guardado en la EEPROM
	cargarOffsetEEPROM(&offset_x, &offset_y);

	uint32_t t_anterior_imu = get_millis();
	uint32_t t_ultimo_ciclo = get_millis();
	uint32_t t_ultimo_serial = get_millis();

	while (1){
		// Leer los botones de forma continua
		act_hold = (PIN_REG_BOTON & (1 << PIN_BOTON_HOLD)) ? 1 : 0;
		act_cal  = (PIN_REG_BOTON & (1 << PIN_BOTON_CAL)) ? 1 : 0;

		boton_hold_presionado = (act_hold != ant_hold);
		boton_cal_presionado  = (act_cal != ant_cal);

		// Lógica de Botones medida con Timer1 (Toque corto en PB0: HOLD / Toque largo en PB0 o botón PB1: Calibrar)
		if (boton_cal_presionado) {
			esperar_ms(50); // Anti-rebote
			uint32_t t_inicio = get_millis();
			while((((PIN_REG_BOTON & (1 << PIN_BOTON_CAL)) ? 1 : 0) == act_cal) && ((get_millis() - t_inicio) < 600));
			ant_cal = (PIN_REG_BOTON & (1 << PIN_BOTON_CAL)) ? 1 : 0;
			estado = ESTADO_CALIBRANDO;
		} else if (boton_hold_presionado) {
			esperar_ms(50); // Anti-rebote
			uint32_t t_inicio = get_millis();
			while((((PIN_REG_BOTON & (1 << PIN_BOTON_HOLD)) ? 1 : 0) == act_hold) && ((get_millis() - t_inicio) < 850));
			uint32_t duracion = get_millis() - t_inicio;
			ant_hold = (PIN_REG_BOTON & (1 << PIN_BOTON_HOLD)) ? 1 : 0;

			if (duracion >= 800) {
				// Si se mantiene presionado pasa al estado de calibración
				estado = ESTADO_CALIBRANDO;
			} else {
				// Toque corto alterna entre MEDICION y HOLD
				if (estado == ESTADO_HOLD) {
					estado = ESTADO_MEDICION;
				} else {
					estado = ESTADO_HOLD;
				}
				pitarBuzzer(30);
			}
		}

		// Muestreo del sensor y actualización de la matriz cada 80 ms usando Timer1
		uint32_t t_actual = get_millis();
		if ((t_actual - t_ultimo_ciclo) >= 80) {
			t_ultimo_ciclo = t_actual;

			// Calcular el diferencial de tiempo real (dt en segundos) con Timer1 para el filtro complementario
			float dt = (float)(t_actual - t_anterior_imu) / 1000.0f;
			t_anterior_imu = t_actual;
			if (dt <= 0.0f || dt > 0.5f) {
				dt = 0.08f;
			}

			// Leer el MPU-6050 (Acelerómetro y Giroscopio)
			MPU6050_LeerDatos(&accl_x, &accl_y, &accl_z, &gyro_x, &gyro_y, &gyro_z);

			// Si en el simulador el eje Z está en 0, usamos 8000 (0.5g) de referencia para que atan2 sea suave
			float az_ref = (accl_z == 0) ? 8000.0f : (float)accl_z;

			// Calcular ángulos de inclinación (pitch y roll) con atan2
			float acc_ang_x = atan2f((float)accl_x, sqrtf((float)accl_y * accl_y + az_ref * az_ref)) * RAD_A_GRADOS;
			float acc_ang_y = atan2f((float)accl_y, sqrtf((float)accl_x * accl_x + az_ref * az_ref)) * RAD_A_GRADOS;

			// Aplicar filtro complementario usando el dt real medido por Timer1
			float vel_x = (float)gyro_y / 131.0f;
			float vel_y = (float)gyro_x / 131.0f;
			angulo_x = 0.85f * (angulo_x + vel_x * dt) + 0.15f * acc_ang_x;
			angulo_y = 0.85f * (angulo_y + vel_y * dt) + 0.15f * acc_ang_y;

			// Máquina de estados del sistema
			switch (estado) {

				case ESTADO_MEDICION:
					// Restar el offset de calibración
					incl_x = angulo_x - offset_x;
					incl_y = angulo_y - offset_y;

					// Zona muerta de ±3 grados
					if (fabsf(incl_x) < ZONA_MUERTA && fabsf(incl_y) < ZONA_MUERTA) {
						pos_x = 4;
						pos_y = 4;
						idx_color = COLOR_NIVELADO; // Azul en LUT
					} else {
						// Mapear ángulos de ±30° al rango 0-7 usando la LUT de umbrales
						pos_x = mapearAngulo(incl_x);
						pos_y = mapearAngulo(INVERTIR_EJE_Y ? -incl_y : incl_y);

						// Seleccionar el índice de color en la LUT según el eje predominante
						if (fabsf(incl_x) > fabsf(incl_y) * 1.4f) {
							idx_color = COLOR_EJE_X;     // Rojo
						} else if (fabsf(incl_y) > fabsf(incl_x) * 1.4f) {
							idx_color = COLOR_EJE_Y;     // Verde
						} else {
							idx_color = COLOR_COMBINADO; // Amarillo
						}
					}

					// Actualizar la Pantalla usando los colores de la LUT
					indiceLed = coord_a_indice(pos_x, pos_y);
					fillAllLedsRGB(0, 0, 0);

					// Indicador de límite si la burbuja toca un borde de la matriz
					if (pos_x == 0 || pos_x == (ANCHO - 1) || pos_y == 0 || pos_y == (ALTO - 1)) {
						parpadeo_borde = !parpadeo_borde;
						if (parpadeo_borde) {
							setLedRGB(matriz, indiceLed,
							          LUT_COLORES_EJE[idx_color][0],
							          LUT_COLORES_EJE[idx_color][1],
							          LUT_COLORES_EJE[idx_color][2]);
							pitarBuzzer(20);
						}
					} else {
						setLedRGB(matriz, indiceLed,
						          LUT_COLORES_EJE[idx_color][0],
						          LUT_COLORES_EJE[idx_color][1],
						          LUT_COLORES_EJE[idx_color][2]);
					}

					show(matriz);
					break;

				case ESTADO_HOLD:
					// Congela la posición y el color actuales en la matriz
					indiceLed = coord_a_indice(pos_x, pos_y);
					fillAllLedsRGB(0, 0, 0);
					setLedRGB(matriz, indiceLed,
					          LUT_COLORES_EJE[idx_color][0],
					          LUT_COLORES_EJE[idx_color][1],
					          LUT_COLORES_EJE[idx_color][2]);
					show(matriz);
					break;

				case ESTADO_CALIBRANDO:
					// Toma la posición actual como nuevo cero y lo guarda en EEPROM
					offset_x = angulo_x;
					offset_y = angulo_y;
					guardarOffsetEEPROM(offset_x, offset_y);
					pitarBuzzer(80);
					estado = ESTADO_MEDICION;
					break;
			}
		}

		// Reportar el ángulo en grados y el estado HOLD por monitor serial cada 400 ms usando Timer1
		if ((get_millis() - t_ultimo_serial) >= 400) {
			t_ultimo_serial = get_millis();
			char msg[60];
			snprintf(msg, sizeof(msg), "X: %d deg | Y: %d deg | HOLD: %d\r\n",
			         (int)incl_x, (int)incl_y, (estado == ESTADO_HOLD) ? 1 : 0);
			UART_EnviarTexto(msg);
		}
	}
}

// Funciones de Timers y Tiempo

// Configura Timer1 en modo CTC a 1 kHz (interrupción cada 1 ms)
void Timer1_Millis_Init(void) {
	TCCR1A = 0;
	TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // Modo CTC, Prescaler 64
	OCR1A  = 249;                                      // 16MHz / (64 * 1000Hz) - 1 = 249
	TIMSK1 = (1 << OCIE1A);                            // Habilitar interrupción Compare Match A
}

// Configura Timer0 en modo CTC para generar la frecuencia del buzzer por hardware/ISR
void Timer0_Buzzer_Init(void) {
	TCCR0A = (1 << WGM01);  // Modo CTC
	TCCR0B = 0;             // Apagado inicialmente
	OCR0A  = 30;            // ~1 kHz con Prescaler 256
	TIMSK0 = (1 << OCIE0A); // Habilitar interrupción Compare Match A
}

uint32_t get_millis(void) {
	uint32_t m;
	cli();
	m = sistema_ms;
	sei();
	return m;
}

void esperar_ms(uint16_t ms) {
	uint32_t inicio = get_millis();
	while ((get_millis() - inicio) < ms);
}

// Funciones de la Matriz

void sendBit(uint8_t bitVal){
	if(bitVal){ // Enviar un '1'
		PORTD |= (1 << PIN_MATRIZ);
		asm volatile (
		"nop\n\t""nop\n\t""nop\n\t""nop\n\t"
		"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
		); // ~0.6 us
		PORTD &= ~ (1 << PIN_MATRIZ);
		asm volatile (
		"nop\n\t""nop\n\t""nop\n\t""nop\n\t"
		"nop\n\t""nop\n\t"
		); // ~0.4 us
		}else { // Enviar un '0'
		PORTD |= (1 << PIN_MATRIZ);
		asm volatile (
		"nop\n\t""nop\n\t""nop\n\t"
		); // ~0.2 us
		PORTD &= ~(1 << PIN_MATRIZ);
		asm volatile (
		"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
		"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
		); // ~0.6 us
	}
}

void sendByte(uint8_t byte){
	for(uint8_t mask = 0x80; mask; mask >>= 1){
		sendBit(byte & mask);
	}
}

void show(uint8_t (*colors)[3]){
	cli(); // Deshabilita interrupciones
	for (int i = 0; i < TOTAL_LEDS; i++){
		sendByte(colors[i][1]); // Verde
		sendByte(colors[i][0]); // Rojo
		sendByte(colors[i][2]); // Azul
	}
	sei(); // Rehabilita interrupciones
	_delay_us(60);
}

void setLedRGB(uint8_t (*leds)[3], int ledIndex, uint8_t r, uint8_t g, uint8_t b){
	if (ledIndex < 0 || ledIndex >= TOTAL_LEDS) return;
	leds[ledIndex][0]=r;
	leds[ledIndex][1]=g;
	leds[ledIndex][2]=b;
}

void fillAllLedsRGB(uint8_t r, uint8_t g, uint8_t b){
	for(int i = 0; i < TOTAL_LEDS; i++){
		setLedRGB(matriz, i, r, g, b);
	}
}

int coord_a_indice(uint8_t x, uint8_t y) {
	if (y >= ALTO || x >= ANCHO) return 0;
	// Usamos la LUT de filas para evitar la multiplicación
	return LUT_FILA_OFFSET[y] + x;
}

// Funciones de Mapeo por LUT, EEPROM y Buzzer no bloqueante

uint8_t mapearAngulo(float angulo) {
	// Busca en la LUT de umbrales en qué columna/fila [0..7] cae el ángulo
	for (uint8_t i = 0; i < 7; i++) {
		if (angulo < (float)LUT_UMBRALES_ANGULO[i]) {
			return i;
		}
	}
	return 7;
}

void cargarOffsetEEPROM(float* off_x, float* off_y) {
	if (eeprom_read_byte((uint8_t*)0x10) == CLAVE_EEPROM) {
		*off_x = eeprom_read_float((float*)0x12);
		*off_y = eeprom_read_float((float*)0x16);
		if (isnan(*off_x) || fabsf(*off_x) > 45.0f) *off_x = 0.0f;
		if (isnan(*off_y) || fabsf(*off_y) > 45.0f) *off_y = 0.0f;
	} else {
		*off_x = 0.0f;
		*off_y = 0.0f;
		guardarOffsetEEPROM(0.0f, 0.0f);
	}
}

void guardarOffsetEEPROM(float off_x, float off_y) {
	eeprom_update_byte((uint8_t*)0x10, CLAVE_EEPROM);
	eeprom_update_float((float*)0x12, off_x);
	eeprom_update_float((float*)0x16, off_y);
}

// Activa el pitido usando Timer0 y Timer1 sin detener el programa principal
void pitarBuzzer(uint8_t duracion_ms) {
	cli();
	buzzer_ms_restantes = duracion_ms;
	TCNT0 = 0;
	TCCR0B = (1 << CS02); // Encender Timer0 con Prescaler 256
	sei();
}

// Inicializa el TWI (I2C) con una velocidad de 100kHz
void I2C_Init(void) {
	TWSR = 0; // Sin pre-escalador
	TWBR = ((F_CPU / 100000UL) - 16) / 2;
	TWCR = (1 << TWEN); // Habilitar TWI
}

// Envía una condición de START
void I2C_Start(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT))); // Esperar a que se complete
}

// Envía una condición de STOP
void I2C_Stop(void) {
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

// Escribe un byte en el bus I2C
void I2C_Escribir(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT))); // Esperar a que se complete
}

// Lee un byte del bus y envía ACK (para seguir leyendo)
uint8_t I2C_LeerACK(void) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	while (!(TWCR & (1 << TWINT))); // Esperar a que se complete
	return TWDR;
}

// Lee un byte del bus y envía NACK (para terminar la lectura)
uint8_t I2C_LeerNACK(void) {
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT))); // Esperar a que se complete
	return TWDR;
}

// Escribe un solo byte en un registro del MPU-6050
void MPU6050_EscribirByte(uint8_t reg, uint8_t data) {
	I2C_Start();
	I2C_Escribir((MPU6050_ADDR << 1) | 0); // Dirección + Bit de Escritura (0)
	I2C_Escribir(reg);                     // Dirección del registro
	I2C_Escribir(data);                    // Dato a escribir
	I2C_Stop();
}

// Lee múltiples bytes de registros del MPU-6050
void MPU6050_LeerBytes(uint8_t reg, uint8_t* buffer, uint8_t len) {
	I2C_Start();
	I2C_Escribir((MPU6050_ADDR << 1) | 0); // Dirección + Bit de Escritura (0)
	I2C_Escribir(reg);                     // Apuntar al registro inicial
	
	I2C_Start(); // Repeated Start
	I2C_Escribir((MPU6050_ADDR << 1) | 1); // Dirección + Bit de Lectura (1)
	
	for (uint8_t i = 0; i < len; i++) {
		if (i == len - 1) {
			buffer[i] = I2C_LeerNACK(); // Último byte se lee con NACK
			} else {
			buffer[i] = I2C_LeerACK();  // Bytes intermedios se leen con ACK
		}
	}
	I2C_Stop();
}

void MPU6050_Init(void) {
	// Escribe 0x00 en el registro PWR_MGMT_1 (0x6B) para despertar el sensor
	MPU6050_EscribirByte(REG_PWR_MGMT_1, 0x00);
	_delay_ms(100); // Esperar a que el sensor se estabilice
}

// Lee acelerómetro y giroscopio del MPU-6050
void MPU6050_LeerDatos(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
	uint8_t buffer[14];
	
	MPU6050_LeerBytes(REG_ACCEL_XOUT_H, buffer, 14);
	
	*ax = (int16_t)(buffer[0] << 8) | buffer[1];
	*ay = (int16_t)(buffer[2] << 8) | buffer[3];
	*az = (int16_t)(buffer[4] << 8) | buffer[5];
	*gx = (int16_t)(buffer[8] << 8) | buffer[9];
	*gy = (int16_t)(buffer[10] << 8) | buffer[11];
	*gz = (int16_t)(buffer[12] << 8) | buffer[13];
}

// Funciones para el Monitor Serial (UART a 9600 baudios)
void UART_Init(void) {
	uint16_t ubrr = 103;
	UBRR0H = (uint8_t)(ubrr >> 8);
	UBRR0L = (uint8_t)ubrr;
	UCSR0B = (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void UART_EnviarTexto(const char* texto) {
	while (*texto) {
		while (!(UCSR0A & (1 << UDRE0)));
		UDR0 = *texto++;
	}
}
