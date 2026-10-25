
#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <stdint.h>

// Definiciones de la Matriz LED
#define PIN_MATRIZ 6    // Pin de datos (PD6)
#define ANCHO 8
#define ALTO 8
#define TOTAL_LEDS (ANCHO * ALTO)

// Definiciones del Joystick (ADC) y Botones
#define PIN_JOY_X 0     // Joystick X en ADC0 (PC0)
#define PIN_JOY_Y 1     // Joystick Y en ADC1 (PC1)
#define PIN_BOTON PC3   // Botón del Joystick en PC3
#define PIN_SEMILLA_ADC 2
#define PIN_BOTON_RESET PD2 // Botón extra para iniciar/reiniciar el juego en PD2

// Definición del Buzzer
#define PIN_BUZZER PD5  // Buzzer en PD5

// Parámetros del Juego y EEPROM
#define MAX_NIVEL 16
#define CLAVE_EEPROM 0xA5

// Estados del Juego
#define ESTADO_LIBRE      0
#define ESTADO_GENERAR    1
#define ESTADO_REPRODUCIR 2
#define ESTADO_JUGADOR    3
#define ESTADO_EXITO      4
#define ESTADO_ERROR      5

// Buffer global para los colores de la matriz
uint8_t matriz[TOTAL_LEDS][3];

// Arreglos para guardar la secuencia generada
uint8_t sec_x[MAX_NIVEL];
uint8_t sec_y[MAX_NIVEL];
uint8_t sec_color[MAX_NIVEL];

// LUT 1: Tabla de búsqueda de colores RGB para los 4 tonos del juego
const uint8_t LUT_COLORES[4][3] = {
	{150, 0, 0},   // 0: Rojo
	{0, 150, 0},   // 1: Verde
	{0, 0, 150},   // 2: Azul
	{130, 130, 0}  // 3: Amarillo
};

// LUT 2: Tabla de búsqueda de valores OCR0A para el Timer0 (Prescaler 256)
// f = 16MHz / (2 * 256 * (1 + OCR0A)) -> 440Hz=70, 554Hz=55, 659Hz=46, 880Hz=35
const uint8_t LUT_OCR0A_NOTAS[4] = {70, 55, 46, 35};

// Valores OCR0A adicionales para melodías de éxito, error y modo libre
#define OCR0A_DO5   59  // 523 Hz
#define OCR0A_SOL5  39  // 784 Hz
#define OCR0A_ERROR 207 // 150 Hz
#define OCR0A_BIP   62  // 500 Hz

// LUT 3: Tabla de búsqueda de desplazamiento por fila (evita multiplicar y * 8)
const uint8_t LUT_FILA_OFFSET[8] = {0, 8, 16, 24, 32, 40, 48, 56};

// Variables volátiles para Timers e Interrupciones
volatile uint32_t milisegundos_contador = 0;
volatile uint16_t buzzer_ms_restantes = 0;

// Prototipos de Funciones
void Timer1_Millis_Init(void);
uint32_t get_millis(void);
void esperar_ms(uint16_t ms);
void Timer0_Buzzer_Init(void);
void sonarBuzzerOCR(uint8_t valor_ocr0a, uint16_t duracion_ms);

void sendBit(uint8_t bitVal);
void sendByte(uint8_t byte);
void show(uint8_t (*colors)[3]);
void setLedRGB(uint8_t (*leds)[3], int ledIndex, uint8_t r, uint8_t g, uint8_t b);
void fillAllLedsRGB(uint8_t r, uint8_t g, uint8_t b);

int coord_a_indice(uint8_t x, uint8_t y);
void ADC_Iniciar(void);
uint8_t ADC_Leer(uint8_t canal);
void colorAleatorio(uint8_t* r, uint8_t* g, uint8_t* b);
unsigned int obtenerSemillaADC(void);

void animacionExito(void);
void animacionError(void);
void mostrarRecord(uint8_t record, uint8_t nivel);
void cargarEEPROM(uint8_t* nivel, uint8_t* record);
void guardarEEPROM(uint8_t nivel, uint8_t record, uint8_t estado);

int main(void){
	// Configuración de pines
	DDRD |= (1 << PIN_MATRIZ) | (1 << PIN_BUZZER);
	DDRC &= ~(1 << PIN_BOTON);
	PORTC |= (1 << PIN_BOTON);
	
	DDRD &= ~(1 << PIN_BOTON_RESET);
	PORTD |= (1 << PIN_BOTON_RESET);
	
	DDRC &= ~(1 << PIN_SEMILLA_ADC);
	PORTC &= ~(1 << PIN_SEMILLA_ADC);
	
	// Iniciar periféricos y Timers
	ADC_Iniciar();
	Timer1_Millis_Init();
	Timer0_Buzzer_Init();
	sei();
	esperar_ms(20);

	// Guardamos el estado inicial de los botones para detectar flancos sin bloquear
	uint8_t ant_boton = (PINC & (1 << PIN_BOTON)) ? 1 : 0;
	uint8_t ant_reset = (PIND & (1 << PIN_BOTON_RESET)) ? 1 : 0;

	// SECCIÓN DE SEMILLA ALEATORIA
	// Obtenemos la semilla leyendo el ruido del pin ADC2
	unsigned int semilla = obtenerSemillaADC();
	srand(semilla);
	
	uint8_t valor_x, valor_y;
	uint8_t act_boton, act_reset;
	uint8_t boton_presionado, reset_presionado;
	uint8_t r, g, b;
	int indiceLed;
	uint8_t pos_x = 4;
	uint8_t pos_y = 4;

	uint8_t estado = ESTADO_LIBRE;
	uint8_t nivel_actual = 1;
	uint8_t record_maximo = 0;
	uint8_t paso_actual = 0;
	uint32_t t_ultimo_ciclo = 0;

	// Leer el nivel y récord guardados en la EEPROM y mostrarlos al iniciar
	cargarEEPROM(&nivel_actual, &record_maximo);
	mostrarRecord(record_maximo, nivel_actual);

	// Obtener el primer color y encender el LED central
	colorAleatorio(&r, &g, &b);
	indiceLed = coord_a_indice(pos_x, pos_y);
	fillAllLedsRGB(0, 0, 0);
	setLedRGB(matriz, indiceLed, r, g, b);
	show(matriz);

	while (1){
		// Control de velocidad no bloqueante cada 100 ms con Timer1
		uint32_t t_actual = get_millis();
		if (t_actual - t_ultimo_ciclo < 100) {
			continue;
		}
		t_ultimo_ciclo = t_actual;

		// 1. Leer el Joystick y detectar pulsación de los botones
		valor_x = ADC_Leer(PIN_JOY_X);
		valor_y = ADC_Leer(PIN_JOY_Y);

		act_boton = (PINC & (1 << PIN_BOTON)) ? 1 : 0;
		act_reset = (PIND & (1 << PIN_BOTON_RESET)) ? 1 : 0;

		boton_presionado = (act_boton != ant_boton);
		reset_presionado = (act_reset != ant_reset);

		// Si se presiona el botón extra en PD2, iniciamos el juego Simón Dice
		if (reset_presionado) {
			esperar_ms(50);
			uint32_t t_ini = get_millis();
			while((((PIND & (1 << PIN_BOTON_RESET)) ? 1 : 0) == act_reset) && (get_millis() - t_ini < 600));
			ant_reset = (PIND & (1 << PIN_BOTON_RESET)) ? 1 : 0;
			nivel_actual = 1;
			estado = ESTADO_GENERAR;
		}

		// Máquina de estados del juego
		switch (estado) {
			
			case ESTADO_LIBRE:
				// Lógica de Movimiento
				if (valor_x > 200) {
					if (pos_x < (ANCHO - 1)) pos_x++;
				} else if (valor_x < 50) {
					if (pos_x > 0) pos_x--;
				}

				if (valor_y > 200) { // Mover Arriba
					if (pos_y > 0) pos_y--;
				} else if (valor_y < 50) { // Mover Abajo
					if (pos_y < (ALTO - 1)) pos_y++;
				}
				
				// Lógica del Botón medida con Timer1 (Toque corto: color / Toque largo: juego)
				if (boton_presionado) {
					esperar_ms(50); // Anti-rebote
					uint32_t t_ini = get_millis();
					while((((PINC & (1 << PIN_BOTON)) ? 1 : 0) == act_boton) && (get_millis() - t_ini < 800));
					uint32_t duracion = get_millis() - t_ini;
					ant_boton = (PINC & (1 << PIN_BOTON)) ? 1 : 0;

					if (duracion >= 650) {
						// Toque largo inicia el juego en el nivel 1
						sonarBuzzerOCR(LUT_OCR0A_NOTAS[2], 150);
						nivel_actual = 1;
						estado = ESTADO_GENERAR;
					} else {
						// Toque corto cambia el color aleatorio en modo libre
						colorAleatorio(&r, &g, &b);
						sonarBuzzerOCR(OCR0A_BIP, 50);
					}
				}

				// Actualizar la Pantalla
				indiceLed = coord_a_indice(pos_x, pos_y);
				fillAllLedsRGB(0, 0, 0);
				setLedRGB(matriz, indiceLed, r, g, b);
				show(matriz);
				break;

			case ESTADO_GENERAR:
				// Generamos la secuencia hasta el nivel actual
				if (nivel_actual == 1) {
					sec_x[0] = rand() % ANCHO;
					sec_y[0] = rand() % ALTO;
					sec_color[0] = rand() % 4;
				} else if (nivel_actual <= MAX_NIVEL) {
					uint8_t i = nivel_actual - 1;
					sec_x[i] = rand() % ANCHO;
					sec_y[i] = rand() % ALTO;
					sec_color[i] = rand() % 4;
				}
				guardarEEPROM(nivel_actual, record_maximo, estado);
				estado = ESTADO_REPRODUCIR;
				break;

			case ESTADO_REPRODUCIR:
				fillAllLedsRGB(0, 0, 0);
				show(matriz);
				esperar_ms(300);

				// Mostrar cada posición usando las LUTs de color y de nota (500 ms ON / 200 ms OFF)
				for (uint8_t i = 0; i < nivel_actual; i++) {
					uint8_t c = sec_color[i];
					indiceLed = coord_a_indice(sec_x[i], sec_y[i]);
					
					fillAllLedsRGB(0, 0, 0);
					setLedRGB(matriz, indiceLed, LUT_COLORES[c][0], LUT_COLORES[c][1], LUT_COLORES[c][2]);
					show(matriz);
					
					sonarBuzzerOCR(LUT_OCR0A_NOTAS[c], 500);
					
					fillAllLedsRGB(0, 0, 0);
					show(matriz);
					esperar_ms(200);
				}

				// Centramos el cursor para el turno del jugador
				pos_x = 4;
				pos_y = 4;
				paso_actual = 0;
				ant_boton = (PINC & (1 << PIN_BOTON)) ? 1 : 0;
				t_ultimo_ciclo = get_millis();
				estado = ESTADO_JUGADOR;
				break;

			case ESTADO_JUGADOR:
				// Lógica de Movimiento del cursor
				if (valor_x > 200) {
					if (pos_x < (ANCHO - 1)) pos_x++;
				} else if (valor_x < 50) {
					if (pos_x > 0) pos_x--;
				}

				if (valor_y > 200) { // Mover Arriba
					if (pos_y > 0) pos_y--;
				} else if (valor_y < 50) { // Mover Abajo
					if (pos_y < (ALTO - 1)) pos_y++;
				}

				// Actualizar el cursor en la pantalla
				indiceLed = coord_a_indice(pos_x, pos_y);
				fillAllLedsRGB(0, 0, 0);
				setLedRGB(matriz, indiceLed, 100, 100, 100);
				show(matriz);

				// Confirmación de posición con el botón del joystick
				if (boton_presionado) {
					esperar_ms(50); // Anti-rebote
					uint32_t t_ini = get_millis();
					while((((PINC & (1 << PIN_BOTON)) ? 1 : 0) == act_boton) && (get_millis() - t_ini < 600));
					ant_boton = (PINC & (1 << PIN_BOTON)) ? 1 : 0;

					uint8_t c = sec_color[paso_actual];
					fillAllLedsRGB(0, 0, 0);
					setLedRGB(matriz, indiceLed, LUT_COLORES[c][0], LUT_COLORES[c][1], LUT_COLORES[c][2]);
					show(matriz);

					// Verificar si la posición coincide con la secuencia
					if (pos_x == sec_x[paso_actual] && pos_y == sec_y[paso_actual]) {
						sonarBuzzerOCR(LUT_OCR0A_NOTAS[c], 250);
						paso_actual++;

						if (paso_actual >= nivel_actual) {
							estado = ESTADO_EXITO;
						}
					} else {
						estado = ESTADO_ERROR;
					}
				}
				break;

			case ESTADO_EXITO:
				// Si supera el récord histórico lo actualizamos
				if (nivel_actual > record_maximo) {
					record_maximo = nivel_actual;
				}
				animacionExito();

				if (nivel_actual < MAX_NIVEL) {
					nivel_actual++;
					guardarEEPROM(nivel_actual, record_maximo, ESTADO_GENERAR);
					estado = ESTADO_GENERAR;
				} else {
					mostrarRecord(record_maximo, nivel_actual);
					nivel_actual = 1;
					guardarEEPROM(nivel_actual, record_maximo, ESTADO_LIBRE);
					estado = ESTADO_LIBRE;
				}
				break;

			case ESTADO_ERROR:
				animacionError();
				mostrarRecord(record_maximo, nivel_actual);

				// Reiniciamos desde el nivel 1
				nivel_actual = 1;
				guardarEEPROM(nivel_actual, record_maximo, ESTADO_GENERAR);
				estado = ESTADO_GENERAR;
				break;
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
	// Control automático de duración del sonido del buzzer en segundo plano
	if (buzzer_ms_restantes > 0) {
		buzzer_ms_restantes--;
		if (buzzer_ms_restantes == 0) {
			TCCR0B = 0x00; // Apagar Timer0 al cumplirse el tiempo
			PORTD &= ~(1 << PIN_BUZZER);
		}
	}
}

uint32_t get_millis(void) {
	uint32_t ms;
	cli();
	ms = milisegundos_contador;
	sei();
	return ms;
}

void esperar_ms(uint16_t ms) {
	uint32_t inicio = get_millis();
	while (get_millis() - inicio < ms);
}

// Timer0 en modo CTC para generar tonos del Buzzer por interrupción
void Timer0_Buzzer_Init(void) {
	TCCR0A = (1 << WGM01); // Modo CTC
	TCCR0B = 0x00;         // Apagado inicialmente
	OCR0A  = LUT_OCR0A_NOTAS[0];
	TIMSK0 = (1 << OCIE0A);
}

ISR(TIMER0_COMPA_vect) {
	PORTD ^= (1 << PIN_BUZZER);
}

void sonarBuzzerOCR(uint8_t valor_ocr0a, uint16_t duracion_ms) {
	cli();
	OCR0A = valor_ocr0a;
	TCNT0 = 0;
	buzzer_ms_restantes = duracion_ms;
	TCCR0B = (1 << CS02); // Encender Timer0 con Prescaler 256
	sei();
	esperar_ms(duracion_ms);
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

// Funciones del Joystick y conversión por LUT
int coord_a_indice(uint8_t x, uint8_t y) {
	if (y >= ALTO || x >= ANCHO) return 0;
	// Usamos la LUT de filas en lugar de multiplicar (y * ANCHO)
	return LUT_FILA_OFFSET[y] + x;
}

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

void colorAleatorio(uint8_t* r, uint8_t* g, uint8_t* b) {
	*r = 50 + (rand() % 205);
	*g = 50 + (rand() % 205);
	*b = 50 + (rand() % 205);
}

unsigned int obtenerSemillaADC(void) {
	unsigned int semilla = 0;
	for (uint8_t i = 0; i < 32; i++) {
		semilla += ADC_Leer(PIN_SEMILLA_ADC);
		_delay_us(10);
	}
	return semilla;
}

// Funciones de Animaciones y EEPROM

void animacionExito(void) {
	for (uint8_t i = 0; i < 2; i++) {
		fillAllLedsRGB(0, 120, 0);
		show(matriz);
		sonarBuzzerOCR(OCR0A_DO5, 120);
		fillAllLedsRGB(0, 0, 0);
		show(matriz);
		esperar_ms(50);
	}
	fillAllLedsRGB(0, 150, 0);
	show(matriz);
	sonarBuzzerOCR(OCR0A_SOL5, 250);
	fillAllLedsRGB(0, 0, 0);
	show(matriz);
	esperar_ms(200);
}

void animacionError(void) {
	// Parpadeo rojo y tono grave generado con Timer0
	for (uint8_t i = 0; i < 3; i++) {
		fillAllLedsRGB(150, 0, 0);
		show(matriz);
		sonarBuzzerOCR(OCR0A_ERROR, 220);
		fillAllLedsRGB(0, 0, 0);
		show(matriz);
		esperar_ms(100);
	}
}

void mostrarRecord(uint8_t record, uint8_t nivel) {
	fillAllLedsRGB(0, 0, 0);
	for (uint8_t i = 0; i < record && i < 32; i++) {
		setLedRGB(matriz, i, 120, 120, 0);
	}
	for (uint8_t i = 0; i < nivel && i < 32; i++) {
		setLedRGB(matriz, 32 + i, 0, 0, 120);
	}
	show(matriz);
	esperar_ms(800);
}

void cargarEEPROM(uint8_t* nivel, uint8_t* record) {
	if (eeprom_read_byte((uint8_t*)0x00) != CLAVE_EEPROM) {
		*nivel = 1;
		*record = 0;
		guardarEEPROM(1, 0, ESTADO_LIBRE);
	} else {
		*nivel = eeprom_read_byte((uint8_t*)0x01);
		*record = eeprom_read_byte((uint8_t*)0x02);
		if (*nivel == 0 || *nivel > MAX_NIVEL) *nivel = 1;
		if (*record > MAX_NIVEL) *record = 0;
	}
}

void guardarEEPROM(uint8_t nivel, uint8_t record, uint8_t estado) {
	eeprom_update_byte((uint8_t*)0x00, CLAVE_EEPROM);
	eeprom_update_byte((uint8_t*)0x01, nivel);
	eeprom_update_byte((uint8_t*)0x02, record);
	eeprom_update_byte((uint8_t*)0x03, estado);
}
