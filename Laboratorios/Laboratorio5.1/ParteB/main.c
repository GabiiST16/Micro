#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdint.h>

/* ============================================================================
 * DEFINICIÓN DE PINES Y CONSTANTES (ATmega328P / Arduino Uno)
 * ============================================================================ */

#define PIN_PULSADOR    PD2     /* INT0    - Pin Digital 2 (Pulsador normal / despertar) */
#define PIN_DHT11       PD3     /* INT1    - Pin Digital 3 (Sensor DHT11) */
#define PIN_EMERGENCIA  PD4     /* PCINT20 - Pin Digital 4 (Botón de emergencia) */

#define PIN_LED         PB0     /* Pin Digital 8 (cambia cada 200 ms) */
#define PIN_BUZZER      PB1     /* Pin Digital 9 (activado por INT0 / Alarma) */

#define PIN_I2C_SDA     PC4     /* Pin Analógico A4 (SDA) */
#define PIN_I2C_SCL     PC5     /* Pin Analógico A5 (SCL) */

/* Bits del expansor I2C PCF8574 hacia el controlador HD44780 de la LCD */
#define PCF_RS          (1 << 0)
#define PCF_RW          (1 << 1)
#define PCF_EN          (1 << 2)
#define PCF_BACKLIGHT   (1 << 3)

/* Períodos objetivo de las tareas (en milisegundos, base Timer0 = 1 ms) */
#define PERIODO_LED_MS              200U
#define PERIODO_LCD_MS              500U
#define PERIODO_DHT_MS              2000U
#define DURACION_PULSO_DHT_MS       20U
#define TIMEOUT_TRAMA_DHT_MS        10U
#define DURACION_BUZZER_MS          150U

/* Tiempos de transición por inactividad de la FSM (Actividad 2) */
#define TIEMPO_ACTIVO_A_STANDBY_MS  10000UL /* 10 s sin actividad -> STANDBY */
#define TIEMPO_A_APAGADO_MS         30000UL /* 30 s desde último evento -> APAGADO */

/* Configuración UART Bluetooth (HC-05 a 9600 bps @ 16 MHz -> UBRR0 = 103) */
#define BAUD_RATE                   9600UL
#define UBRR_VAL                    ((F_CPU / (16UL * BAUD_RATE)) - 1UL)
#define UART_RX_BUF_SIZE            16U
#define UART_TX_BUF_SIZE            32U
#define TRAMA_START                 ':'
#define TRAMA_STOP                  ';'

/* ============================================================================
 * TIPOS DE DATOS DE LA MÁQUINA DE ESTADOS FINITOS (SIN SWITCH-CASE)
 * ============================================================================ */

typedef enum {
    ESTADO_ACTIVO  = 0,
    ESTADO_STANDBY = 1,
    ESTADO_APAGADO = 2,
    ESTADO_SEGURO  = 3
} EstadoSistema_t;

typedef enum {
    EVENTO_NINGUNO              = 0,
    EVENTO_INACTIVIDAD_10S      = 1,
    EVENTO_INACTIVIDAD_30S      = 2,
    EVENTO_ACTIVIDAD_USUARIO    = 3,
    EVENTO_EMERGENCIA           = 4,
    EVENTO_SOLICITAR_STANDBY    = 5,
    EVENTO_SOLICITAR_APAGADO    = 6,
    EVENTO_RECUPERACION_VALIDA  = 7
} EventoFSM_t;

typedef void (*AccionEstado_t)(void);

typedef struct {
    EstadoSistema_t estado_origen;
    EventoFSM_t     evento;
    EstadoSistema_t estado_destino;
    AccionEstado_t  accion_entrada;
} TransicionFSM_t;

/* ============================================================================
 * VARIABLES GLOBALES Y VOLÁTILES
 * ============================================================================ */

/* Base de tiempos de 1 ms (Timer0) */
volatile uint8_t  flag_tick_1ms = 0;

/* Adquisición del ADC (Timer1 100 ms + ADC_vect) */
volatile uint16_t adc_valor_actual = 0;
volatile uint8_t  flag_adc_listo   = 0;

/* Eventos externos asíncronos (INT0, Emergencia y Bluetooth) */
volatile uint8_t  evento_pulsador_buzzer = 0;
volatile uint8_t  evento_actividad_fsm   = 0;
volatile uint8_t  solicitud_emergencia   = 0;
uint8_t           solicitud_standby_bt   = 0;
uint8_t           solicitud_apagado_bt   = 0;
uint8_t           solicitud_recuperar_bt = 0;

/* Variables del Watchdog Timer (Problema B - Actividad 3: Pruebas A y B) */
volatile uint8_t  flag_wdt_despertar      = 0;
uint16_t          contador_despertares_wdt = 0;
uint8_t           causa_reset_wdt         = 0;

/* Buffers circulares no bloqueantes para Bluetooth UART (Problema B - Actividad 4) */
volatile uint8_t  uart_rx_buf[UART_RX_BUF_SIZE];
volatile uint8_t  uart_rx_head = 0;
volatile uint8_t  uart_rx_tail = 0;

volatile uint8_t  uart_tx_buf[UART_TX_BUF_SIZE];
volatile uint8_t  uart_tx_head = 0;
volatile uint8_t  uart_tx_tail = 0;

/* Estado actual de la FSM y contador de inactividad */
EstadoSistema_t estado_actual         = ESTADO_ACTIVO;
uint32_t        tiempo_inactividad_ms = 0;

/* Control temporal no bloqueante del Buzzer */
uint8_t  buzzer_activo   = 0;
uint16_t buzzer_timer_ms = 0;

/* Máquina de estados y captura por interrupción (INT1 + TCNT0) del DHT11 */
volatile uint8_t dht_datos[5]   = {0, 0, 0, 0, 0};
volatile uint8_t dht_pulso_idx  = 0;
volatile uint8_t dht_tcnt_prev  = 0;
volatile uint8_t flag_dht_listo = 0;

uint8_t dht_temperatura = 0;
uint8_t dht_etapa       = 0; /* 0: espera 2s, 1: pulso bajo 20ms, 2: recepción INT1 */
uint8_t dht_timer_ms    = 0;

/* Transmisión I2C (TWI) dirigida por interrupción (ISR TWI_vect) sin esperas */
static const uint8_t direcciones_pcf[4] = { 0x27, 0x20, 0x3F, 0x38 };
volatile uint8_t twi_addr_idx  = 0;
volatile uint8_t twi_buf[4];
volatile uint8_t twi_len       = 0;
volatile uint8_t twi_idx       = 0;
volatile uint8_t twi_ocupado   = 0;

/* Motor cooperativo de la LCD 16x2 (inicialización y refresco paso a paso) */
uint8_t lcd_backlight_bit = PCF_BACKLIGHT;
uint8_t lcd_init_ticks    = 0;
uint8_t lcd_inicializado  = 0;
uint8_t lcd_paso_actual   = 0;
char    lcd_linea1[16];
char    lcd_linea2[16];

/* ============================================================================
 * GESTIÓN DEL WATCHDOG TIMER (PROBLEMA B - PRUEBAS A Y B)
 * ============================================================================ */

/* Desactiva el WDT de forma segura en el arranque tras leer MCUSR */
static void wdt_desactivar_seguro(void)
{
    cli();
    wdt_reset();
    MCUSR &= ~(1 << WDRF);
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    WDTCSR = 0x00;
    sei();
}

/* Prueba A: WDT como despertador por interrupción cada 8.0 segundos en Power-Down */
static void wdt_config_interrupcion_8s(void)
{
    cli();
    wdt_reset();
    MCUSR &= ~(1 << WDRF);
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0); /* WDIE=1, Prescaler 8.0 s */
    sei();
}

/* Prueba B: WDT en modo Reset del sistema a 2.0 segundos ante bloqueos */
static void wdt_config_reset_2s(void)
{
    cli();
    wdt_reset();
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    WDTCSR = (1 << WDE) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0); /* WDE=1, 2.0 s */
    sei();
}

ISR(WDT_vect)
{
    flag_wdt_despertar = 1; /* ISR O(1): la lectura rápida se ejecuta al despertar en main */
}

/* ============================================================================
 * DRIVER UART BLUETOOTH NO BLOQUEANTE (RX/TX POR INTERRUPCIÓN + TRAMAS XOR)
 * ============================================================================ */

static void uart_init(void)
{
    UBRR0H = (uint8_t)(UBRR_VAL >> 8);
    UBRR0L = (uint8_t)(UBRR_VAL & 0xFFU);
    UCSR0A = 0x00;
    UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0); /* RX ISR + RX/TX activos */
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);               /* Formato 8N1 */
}

/* Encola un byte en el buffer circular de transmisión y habilita ISR UDRE0 */
static void uart_encolar_byte(uint8_t dato)
{
    uint8_t sig_head = (uint8_t)((uart_tx_head + 1U) & (UART_TX_BUF_SIZE - 1U));
    if (sig_head != uart_tx_tail) {
        uart_tx_buf[uart_tx_head] = dato;
        uart_tx_head = sig_head;
        UCSR0B |= (1 << UDRIE0);
    }
}

/* Envía trama de 5 bytes: :[CMD][DATO][CHK]; con CHK = CMD ^ DATO */
static void uart_enviar_trama(uint8_t cmd, uint8_t dato)
{
    uint8_t chk = (uint8_t)(cmd ^ dato);
    uart_encolar_byte((uint8_t)TRAMA_START);
    uart_encolar_byte(cmd);
    uart_encolar_byte(dato);
    uart_encolar_byte(chk);
    uart_encolar_byte((uint8_t)TRAMA_STOP);
}

ISR(USART_RX_vect)
{
    uint8_t dato = UDR0;
    uint8_t sig_head = (uint8_t)((uart_rx_head + 1U) & (UART_RX_BUF_SIZE - 1U));
    if (sig_head != uart_rx_tail) {
        uart_rx_buf[uart_rx_head] = dato;
        uart_rx_head = sig_head;
    }
}

ISR(USART_UDRE_vect)
{
    if (uart_tx_head != uart_tx_tail) {
        UDR0 = uart_tx_buf[uart_tx_tail];
        uart_tx_tail = (uint8_t)((uart_tx_tail + 1U) & (UART_TX_BUF_SIZE - 1U));
    } else {
        UCSR0B &= ~(1 << UDRIE0); /* Buffer vacío: apagar interrupción UDRE0 */
    }
}

static void uart_procesar_tramas(uint16_t adc_actual)
{
    while (uart_rx_tail != uart_rx_head) {
        if (uart_rx_buf[uart_rx_tail] == (uint8_t)TRAMA_START) {
            uint8_t disp = (uart_rx_head >= uart_rx_tail)
                           ? (uint8_t)(uart_rx_head - uart_rx_tail)
                           : (uint8_t)(UART_RX_BUF_SIZE - uart_rx_tail + uart_rx_head);

            if (disp >= 5U) {
                uint8_t i_cmd  = (uint8_t)((uart_rx_tail + 1U) & (UART_RX_BUF_SIZE - 1U));
                uint8_t i_dato = (uint8_t)((uart_rx_tail + 2U) & (UART_RX_BUF_SIZE - 1U));
                uint8_t i_chk  = (uint8_t)((uart_rx_tail + 3U) & (UART_RX_BUF_SIZE - 1U));
                uint8_t i_stop = (uint8_t)((uart_rx_tail + 4U) & (UART_RX_BUF_SIZE - 1U));

                if (uart_rx_buf[i_stop] == (uint8_t)TRAMA_STOP) {
                    uint8_t cmd  = uart_rx_buf[i_cmd];
                    uint8_t dato = uart_rx_buf[i_dato];
                    uint8_t chk  = uart_rx_buf[i_chk];

                    if (chk == (uint8_t)(cmd ^ dato)) {
                        /* Trama válida: cualquier comando en STANDBY despierta a ACTIVO */
                        if ((estado_actual == ESTADO_STANDBY) && (cmd != (uint8_t)'E')) {
                            evento_actividad_fsm = 1;
                        }
                        tiempo_inactividad_ms = 0;

                        if (cmd == (uint8_t)'E') {
                            solicitud_emergencia = 1;
                        } else if (cmd == (uint8_t)'R') {
                            solicitud_recuperar_bt = 1;
                        } else if (cmd == (uint8_t)'S') {
                            static const uint8_t cod_estado[4] = { 'A', 'B', 'P', 'E' };
                            if ((estado_actual == ESTADO_SEGURO) && causa_reset_wdt) {
                                uart_enviar_trama('E', 'W'); /* Informa reset previo por WDT */
                            } else {
                                uart_enviar_trama('S', cod_estado[estado_actual]);
                            }
                        } else if (cmd == (uint8_t)'L') {
                            uart_enviar_trama('L', (uint8_t)(adc_actual >> 2));
                            uart_enviar_trama('T', dht_temperatura);
                        } else if (cmd == (uint8_t)'A') {
                            if (dato == (uint8_t)'1') {
                                PORTB |= (1 << PIN_BUZZER);
                            } else {
                                PORTB &= ~(1 << PIN_BUZZER);
                                buzzer_activo = 0;
                            }
                        } else if (cmd == (uint8_t)'M') {
                            if (dato == (uint8_t)'A') {
                                evento_actividad_fsm = 1;
                            } else if (dato == (uint8_t)'B') {
                                solicitud_standby_bt = 1;
                            } else if (dato == (uint8_t)'P') {
                                solicitud_apagado_bt = 1;
                            }
                        } else if (cmd == (uint8_t)'X') {
                            /* Prueba B del WDT: bloqueo intencional por bucle infinito */
                            while (1) {
                                ; /* El WDT desbordará en 2 s y reiniciará el MCU */
                            }
                        }
                    } else {
                        uart_enviar_trama('N', 'C'); /* Error de Checksum XOR */
                    }
                } else {
                    uart_enviar_trama('N', 'F'); /* Error de delimitador de trama */
                }

                uart_rx_tail = (uint8_t)((uart_rx_tail + 5U) & (UART_RX_BUF_SIZE - 1U));
                continue;
            } else {
                break; /* Esperar a recibir los 5 bytes completos */
            }
        }
        uart_rx_tail = (uint8_t)((uart_rx_tail + 1U) & (UART_RX_BUF_SIZE - 1U));
    }
}

/* ============================================================================
 * DRIVER TWI (I2C) NO BLOQUEANTE MEDIANTE INTERRUPCIÓN (ISR TWI_vect)
 * ============================================================================ */

static void twi_init(void)
{
    PORTC |= (1 << PIN_I2C_SDA) | (1 << PIN_I2C_SCL);
    TWSR = 0x00;        /* Prescaler = 1 */
    TWBR = 72;          /* 100 kHz @ 16 MHz */
    TWCR = (1 << TWEN); /* Habilitar TWI */
}

/* Inicia el envío asíncrono de N bytes al PCF8574 y retorna de inmediato */
static void twi_iniciar_paquete(uint8_t cantidad)
{
    if (twi_ocupado) {
        return;
    }
    twi_len     = cantidad;
    twi_idx     = 0;
    twi_ocupado = 1;

    /* Generar condición START y habilitar interrupción TWI */
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE);
}

ISR(TWI_vect)
{
    uint8_t estado_twi = TWSR & 0xF8;

    /* 0x08 = START enviado, 0x10 = REPEATED START enviado */
    if ((estado_twi == 0x08) || (estado_twi == 0x10)) {
        TWDR = (uint8_t)(direcciones_pcf[twi_addr_idx] << 1); /* SLA+W */
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
    }
    /* 0x18 = SLA+W con ACK, 0x28 = Dato enviado con ACK */
    else if ((estado_twi == 0x18) || (estado_twi == 0x28)) {
        if (twi_idx < twi_len) {
            TWDR = twi_buf[twi_idx];
            twi_idx++;
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
        } else {
            /* Todos los bytes fueron enviados: transmitir STOP y liberar bus */
            TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
            twi_ocupado = 0;
        }
    }
    /* 0x20 = NACK en dirección: probar siguiente dirección de PCF8574/PCF8574A */
    else if (estado_twi == 0x20) {
        twi_addr_idx = (uint8_t)((twi_addr_idx + 1U) & 0x03U);
        TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
        twi_ocupado = 0;
    }
    /* Cualquier otro código de error de bus: abortar con STOP */
    else {
        TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
        twi_ocupado = 0;
    }
}

/* ============================================================================
 * CONTROL COOPERATIVO DE LA LCD 16x2 I2C (SIN DELAYS EN INICIALIZACIÓN NI ENVÍO)
 * ============================================================================ */

static void lcd_enviar_nibble_async(uint8_t nibble_alto, uint8_t bit_rs)
{
    uint8_t base = (nibble_alto & 0xF0) | lcd_backlight_bit;
    if (bit_rs) {
        base |= PCF_RS;
    }
    twi_buf[0] = base | PCF_EN;
    twi_buf[1] = base & ~PCF_EN;
    twi_iniciar_paquete(2);
}

static void lcd_enviar_byte_async(uint8_t valor, uint8_t es_dato)
{
    uint8_t alto = (valor & 0xF0) | lcd_backlight_bit;
    uint8_t bajo = ((valor << 4) & 0xF0) | lcd_backlight_bit;

    if (es_dato) {
        alto |= PCF_RS;
        bajo |= PCF_RS;
    }

    twi_buf[0] = alto | PCF_EN;
    twi_buf[1] = alto & ~PCF_EN;
    twi_buf[2] = bajo | PCF_EN;
    twi_buf[3] = bajo & ~PCF_EN;
    twi_iniciar_paquete(4);
}

void lcd_set_backlight(uint8_t encender)
{
    if (encender) {
        lcd_backlight_bit = PCF_BACKLIGHT;
    } else {
        lcd_backlight_bit = 0x00;
    }
}

static void uint_a_texto(uint16_t valor, char *destino, uint8_t digitos)
{
    while (digitos > 0) {
        digitos--;
        destino[digitos] = (char)('0' + (valor % 10U));
        valor /= 10U;
    }
}

static void copiar_cadena_16(char *destino, const char *origen)
{
    uint8_t i = 0;
    while ((i < 16U) && (origen[i] != '\0')) {
        destino[i] = origen[i];
        i++;
    }
    while (i < 16U) {
        destino[i] = ' ';
        i++;
    }
}

static void lcd_preparar_vista_activo(uint16_t adc_val, uint8_t temp)
{
    /* Línea 1: "ADC:0000  T:00C " */
    lcd_linea1[0]  = 'A'; lcd_linea1[1]  = 'D'; lcd_linea1[2]  = 'C'; lcd_linea1[3]  = ':';
    uint_a_texto(adc_val, &lcd_linea1[4], 4);
    lcd_linea1[8]  = ' '; lcd_linea1[9]  = ' ';
    lcd_linea1[10] = 'T'; lcd_linea1[11] = ':';
    uint_a_texto(temp, &lcd_linea1[12], 2);
    lcd_linea1[14] = 'C'; lcd_linea1[15] = ' ';

    /* Línea 2: "ESTADO: ACTIVO  " */
    copiar_cadena_16(lcd_linea2, "ESTADO: ACTIVO  ");

    lcd_paso_actual = 1;
}

static void lcd_preparar_mensaje_estado(const char *texto_l1, const char *texto_l2)
{
    copiar_cadena_16(lcd_linea1, texto_l1);
    copiar_cadena_16(lcd_linea2, texto_l2);
    lcd_paso_actual = 1;
}

static void lcd_tarea_cooperativa_1ms(void)
{
    if (twi_ocupado) {
        return;
    }

    /* Secuencia de inicialización no bloqueante basada en el tick de 1 ms */
    if (!lcd_inicializado) {
        lcd_init_ticks++;

        if (lcd_init_ticks == 20U) {
            lcd_enviar_nibble_async(0x30, 0);
        } else if (lcd_init_ticks == 26U) {
            lcd_enviar_nibble_async(0x30, 0);
        } else if (lcd_init_ticks == 28U) {
            lcd_enviar_nibble_async(0x30, 0);
        } else if (lcd_init_ticks == 30U) {
            lcd_enviar_nibble_async(0x20, 0);
        } else if (lcd_init_ticks == 32U) {
            lcd_enviar_byte_async(0x28, 0);
        } else if (lcd_init_ticks == 34U) {
            lcd_enviar_byte_async(0x0C, 0);
        } else if (lcd_init_ticks == 36U) {
            lcd_enviar_byte_async(0x06, 0);
        } else if (lcd_init_ticks == 38U) {
            lcd_enviar_byte_async(0x01, 0);
        } else if (lcd_init_ticks >= 42U) {
            lcd_inicializado = 1;
            lcd_paso_actual  = 1;
        }
        return;
    }

    /* Envío no bloqueante de los 32 caracteres + 2 comandos de dirección */
    if (lcd_paso_actual == 0U) {
        return;
    } else if (lcd_paso_actual == 1U) {
        lcd_enviar_byte_async(0x80, 0);
        lcd_paso_actual++;
    } else if ((lcd_paso_actual >= 2U) && (lcd_paso_actual <= 17U)) {
        lcd_enviar_byte_async((uint8_t)lcd_linea1[lcd_paso_actual - 2U], 1);
        lcd_paso_actual++;
    } else if (lcd_paso_actual == 18U) {
        lcd_enviar_byte_async(0xC0, 0);
        lcd_paso_actual++;
    } else if ((lcd_paso_actual >= 19U) && (lcd_paso_actual <= 34U)) {
        lcd_enviar_byte_async((uint8_t)lcd_linea2[lcd_paso_actual - 19U], 1);
        if (lcd_paso_actual == 34U) {
            lcd_paso_actual = 0U;
        } else {
            lcd_paso_actual++;
        }
    }
}

/* ============================================================================
 * LECTURA NO BLOQUEANTE DEL SENSOR DHT11 (MÁQUINA DE ESTADOS + INT1 + TCNT0)
 * ============================================================================ */

ISR(INT1_vect)
{
    uint8_t t_actual = TCNT0;
    uint8_t delta;
    uint8_t byte_idx;

    if (dht_pulso_idx == 0U) {
        dht_pulso_idx = 1U;
    } else if (dht_pulso_idx == 1U) {
        dht_tcnt_prev = t_actual;
        dht_pulso_idx = 2U;
    } else if ((dht_pulso_idx >= 2U) && (dht_pulso_idx <= 41U)) {
        if (t_actual >= dht_tcnt_prev) {
            delta = (uint8_t)(t_actual - dht_tcnt_prev);
        } else {
            delta = (uint8_t)(250U - dht_tcnt_prev + t_actual);
        }
        dht_tcnt_prev = t_actual;

        byte_idx = (uint8_t)((dht_pulso_idx - 2U) >> 3);
        dht_datos[byte_idx] <<= 1;
        if (delta > 24U) {
            dht_datos[byte_idx] |= 1U;
        }

        if (dht_pulso_idx == 41U) {
            EIMSK &= ~(1 << INT1);
            flag_dht_listo = 1U;
        } else {
            dht_pulso_idx++;
        }
    }
}

static void dht11_tarea_cooperativa_1ms(uint16_t *acum_dht_ms)
{
    if (dht_etapa == 0U) {
        if (*acum_dht_ms >= PERIODO_DHT_MS) {
            *acum_dht_ms = 0U;
            DDRD  |=  (1 << PIN_DHT11);
            PORTD &= ~(1 << PIN_DHT11);
            dht_timer_ms = 0U;
            dht_etapa    = 1U;
        }
    } else if (dht_etapa == 1U) {
        dht_timer_ms++;
        if (dht_timer_ms >= DURACION_PULSO_DHT_MS) {
            dht_datos[0]   = 0U;
            dht_datos[1]   = 0U;
            dht_datos[2]   = 0U;
            dht_datos[3]   = 0U;
            dht_datos[4]   = 0U;
            dht_pulso_idx  = 0U;
            flag_dht_listo = 0U;

            PORTD |=  (1 << PIN_DHT11);
            DDRD  &= ~(1 << PIN_DHT11);

            EICRA = (EICRA & ~(1 << ISC10)) | (1 << ISC11);
            EIFR  = (1 << INTF1);
            EIMSK |= (1 << INT1);

            dht_timer_ms = 0U;
            dht_etapa    = 2U;
        }
    } else if (dht_etapa == 2U) {
        dht_timer_ms++;
        if (flag_dht_listo) {
            flag_dht_listo = 0U;
            if ((uint8_t)(dht_datos[0] + dht_datos[1] + dht_datos[2] + dht_datos[3]) == dht_datos[4]) {
                dht_temperatura = dht_datos[2];
            }
            dht_etapa = 0U;
        } else if (dht_timer_ms >= TIMEOUT_TRAMA_DHT_MS) {
            EIMSK &= ~(1 << INT1);
            dht_etapa = 0U;
        }
    }
}

/* ============================================================================
 * CONFIGURACIÓN DE REGISTROS DEL MICROCONTROLADOR
 * ============================================================================ */

static void config_reloj_16mhz(void)
{
    CLKPR = (1 << CLKPCE);
    CLKPR = 0x00; /* Prescaler de sistema = 1 (16 MHz) */
}

static void config_timer0_1ms(void)
{
    TCCR0A = (1 << WGM01);              /* Modo CTC */
    TCCR0B = (1 << CS01) | (1 << CS00); /* Prescaler 64 */
    OCR0A  = 249;                       /* 1 ms exacto @ 16 MHz */
    TIMSK0 = (1 << OCIE0A);             /* Habilitar Compare Match A */
}

static void config_timer1_100ms(void)
{
    TCCR1A = 0x00;
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); /* CTC y Prescaler 64 */
    OCR1A  = 24999;                                    /* 100 ms exactos @ 16 MHz */
    TIMSK1 = (1 << OCIE1A);                            /* Habilitar Compare Match A */
}

static void config_adc(void)
{
    ADMUX  = (1 << REFS0); /* Canal ADC0 (PC0), Referencia AVCC */
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    DIDR0  = (1 << ADC0D);
    ADCSRA |= (1 << ADSC); /* Iniciar primera conversión */
}

static void config_int0_pulsador(void)
{
    /* Pulsador normal de actividad en PD2 (INT0) con pull-up interno */
    DDRD  &= ~(1 << PIN_PULSADOR);
    PORTD |=  (1 << PIN_PULSADOR);

    EICRA = (EICRA & ~(1 << ISC00)) | (1 << ISC01); /* Flanco de bajada en INT0 */
    EIFR  = (1 << INTF0);
    EIMSK |= (1 << INT0);

    /* Pulsador físico de EMERGENCIA en PD4 (PCINT20) con pull-up interno */
    DDRD   &= ~(1 << PIN_EMERGENCIA);
    PORTD  |=  (1 << PIN_EMERGENCIA);
    PCICR  |=  (1 << PCIE2);   /* Habilitar grupo PCINT[23:16] (Puerto D) */
    PCMSK2 |=  (1 << PCINT20); /* Habilitar interrupción en PD4 */
}

static void config_gpio(void)
{
    DDRB  |=  (1 << PIN_LED) | (1 << PIN_BUZZER);
    PORTB &= ~((1 << PIN_LED) | (1 << PIN_BUZZER));

    DDRD  &= ~(1 << PIN_DHT11);
    PORTD |=  (1 << PIN_DHT11);
}

/* ============================================================================
 * SUBRUTINA DE SUEÑO PROFUNDO (SLEEP_MODE_PWR_DOWN + PRR + WDT 8S)
 * ============================================================================ */

/*
 * Ejecutada en ESTADO_APAGADO una vez que la LCD terminó de apagar su backlight
 * y el buffer TX de UART vació la trama de aviso ':SP...;'.
 * Cumple:
 *  - Apagado de periféricos mediante máscaras explícitas en PRR.
 *  - Despertar por INT0 (nivel bajo), PCINT20 (emergencia) o WDT (cada 8 s).
 *  - Prueba A del WDT: despierta cada 8 s, habilita el ADC, toma una lectura
 *    rápida, registra el evento y vuelve a dormir en SLEEP_MODE_PWR_DOWN.
 */
static void ejecutar_ciclo_power_down(void)
{
    /* 1. Apagar periféricos analógicos y cortar relojes en PRR */
    ADCSRA &= ~(1 << ADEN);
    TWCR   &= ~(1 << TWEN);
    PRR |= (1 << PRADC) | (1 << PRUSART0) | (1 << PRSPI) |
           (1 << PRTIM1) | (1 << PRTIM0) | (1 << PRTIM2) | (1 << PRTWI);

    /* 2. Configurar INT0 por nivel bajo para despertar desde SLEEP_MODE_PWR_DOWN */
    EICRA &= ~((1 << ISC01) | (1 << ISC00));
    EIFR   = (1 << INTF0);
    EIMSK |= (1 << INT0);

    /* 3. Configurar WDT en modo Interrupción cada 8.0 s (Prueba A) */
    wdt_config_interrupcion_8s();

    while ((estado_actual == ESTADO_APAGADO) &&
           (!evento_actividad_fsm) &&
           (!solicitud_emergencia)) {

        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_enable();
        sei();
        sleep_cpu(); /* El reloj de 16 MHz se detiene aquí */
        sleep_disable();

        /* Si el despertar fue causado por el WDT (cada 8 segundos - Prueba A) */
        if (flag_wdt_despertar) {
            flag_wdt_despertar = 0;
            contador_despertares_wdt++;

            /* Rehabilitar únicamente el ADC, realizar lectura rápida y apagarlo */
            PRR &= ~(1 << PRADC);
            ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
            ADCSRA |= (1 << ADSC);
            while (ADCSRA & (1 << ADSC)) {
                ; /* Espera puntual de ~104 us previa a reingresar a Power-Down */
            }
            adc_valor_actual = ADC;
            flag_adc_listo   = 1;

            ADCSRA &= ~(1 << ADEN);
            PRR |= (1 << PRADC);

            /* Rearmar la interrupción de 8 s del WDT antes de volver a dormir */
            wdt_config_interrupcion_8s();
        }
    }

    /* 4. Al despertar por usuario (INT0) o emergencia (PCINT20): restaurar hardware */
    wdt_config_reset_2s();

    /* Restaurar INT0 a detección por flanco de bajada */
    EICRA = (EICRA & ~(1 << ISC00)) | (1 << ISC01);
    EIFR  = (1 << INTF0);
    EIMSK |= (1 << INT0);

    /* Rehabilitar relojes de periféricos activos en PRR */
    PRR &= ~((1 << PRADC) | (1 << PRUSART0) | (1 << PRTIM1) | (1 << PRTIM0) | (1 << PRTWI));
    twi_init();
    uart_init();
    config_adc();
}

/* ============================================================================
 * MÁQUINA DE ESTADOS FINITOS (TABLA DE TRANSICIONES SIN SWITCH-CASE)
 * ============================================================================ */

static void accion_entrar_activo(void)
{
    /* Rehabilitar todos los periféricos operativos en PRR */
    PRR &= ~((1 << PRADC) | (1 << PRUSART0) | (1 << PRTIM1) | (1 << PRTIM0) | (1 << PRTWI));

    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADSC);
    TIMSK1 |= (1 << OCIE1A);
    PORTB  &= ~(1 << PIN_BUZZER);

    lcd_set_backlight(1);
    lcd_preparar_vista_activo(adc_valor_actual, dht_temperatura);
    uart_enviar_trama('S', 'A');
}

static void accion_entrar_standby(void)
{
    PORTB &= ~(1 << PIN_LED);
    EIMSK &= ~(1 << INT1);
    DDRD  &= ~(1 << PIN_DHT11);
    PORTD |=  (1 << PIN_DHT11);
    dht_etapa = 0U;

    /* Apagar ADC y Timer1 en STANDBY para ahorrar energía en SLEEP_MODE_IDLE */
    TIMSK1 &= ~(1 << OCIE1A);
    ADCSRA &= ~(1 << ADEN);
    PRR |= (1 << PRADC) | (1 << PRTIM1) | (1 << PRSPI) | (1 << PRTIM2);

    lcd_set_backlight(1);
    lcd_preparar_mensaje_estado("Esperando...    ", "ESTADO: STANDBY ");
    uart_enviar_trama('S', 'B');
}

static void accion_entrar_apagado(void)
{
    TIMSK1 &= ~(1 << OCIE1A);
    ADCSRA &= ~(1 << ADEN);
    EIMSK  &= ~(1 << INT1);

    PORTB &= ~((1 << PIN_LED) | (1 << PIN_BUZZER));
    DDRD  &= ~(1 << PIN_DHT11);
    PORTD |=  (1 << PIN_DHT11);
    dht_etapa = 0U;

    lcd_set_backlight(0);
    lcd_preparar_mensaje_estado("MODO BAJO CONSUM", "ESTADO: APAGADO ");
    uart_enviar_trama('S', 'P');
}

static void accion_entrar_seguro(void)
{
    /* Asegurar que TWI y UART estén encendidos por si viene desde APAGADO */
    PRR &= ~((1 << PRUSART0) | (1 << PRTIM0) | (1 << PRTWI));
    twi_init();

    TIMSK1 &= ~(1 << OCIE1A);
    ADCSRA &= ~(1 << ADEN);
    PRR    |= (1 << PRADC) | (1 << PRTIM1);
    EIMSK  &= ~(1 << INT1);

    PORTB &= ~(1 << PIN_LED);
    PORTB |=  (1 << PIN_BUZZER); /* Alarma activa en Estado Seguro */
    buzzer_activo = 0;

    lcd_set_backlight(1);
    if (causa_reset_wdt) {
        lcd_preparar_mensaje_estado("!RESET WATCHDOG!", "ESTADO: SEGURO  ");
        uart_enviar_trama('E', 'W');
    } else {
        lcd_preparar_mensaje_estado("!! EMERGENCIA !!", "ESTADO: SEGURO  ");
        uart_enviar_trama('E', 'E');
    }
}

static const TransicionFSM_t tabla_transiciones[] = {
    { ESTADO_ACTIVO,  EVENTO_INACTIVIDAD_10S,     ESTADO_STANDBY, accion_entrar_standby },
    { ESTADO_ACTIVO,  EVENTO_SOLICITAR_STANDBY,   ESTADO_STANDBY, accion_entrar_standby },
    { ESTADO_ACTIVO,  EVENTO_SOLICITAR_APAGADO,   ESTADO_APAGADO, accion_entrar_apagado },
    { ESTADO_STANDBY, EVENTO_INACTIVIDAD_30S,     ESTADO_APAGADO, accion_entrar_apagado },
    { ESTADO_STANDBY, EVENTO_SOLICITAR_APAGADO,   ESTADO_APAGADO, accion_entrar_apagado },
    { ESTADO_STANDBY, EVENTO_ACTIVIDAD_USUARIO,   ESTADO_ACTIVO,  accion_entrar_activo  },
    { ESTADO_APAGADO, EVENTO_ACTIVIDAD_USUARIO,   ESTADO_ACTIVO,  accion_entrar_activo  },
    { ESTADO_SEGURO,  EVENTO_RECUPERACION_VALIDA, ESTADO_ACTIVO,  accion_entrar_activo  },
    { ESTADO_ACTIVO,  EVENTO_EMERGENCIA,          ESTADO_SEGURO,  accion_entrar_seguro  },
    { ESTADO_STANDBY, EVENTO_EMERGENCIA,          ESTADO_SEGURO,  accion_entrar_seguro  },
    { ESTADO_APAGADO, EVENTO_EMERGENCIA,          ESTADO_SEGURO,  accion_entrar_seguro  }
};

#define NUM_TRANSICIONES (sizeof(tabla_transiciones) / sizeof(tabla_transiciones[0]))

static void fsm_procesar_evento(EventoFSM_t evento)
{
    uint8_t i;

    if (evento == EVENTO_NINGUNO) {
        return;
    }

    for (i = 0; i < NUM_TRANSICIONES; i++) {
        if ((tabla_transiciones[i].estado_origen == estado_actual) &&
            (tabla_transiciones[i].evento == evento)) {

            estado_actual = tabla_transiciones[i].estado_destino;

            if (tabla_transiciones[i].accion_entrada != 0) {
                tabla_transiciones[i].accion_entrada();
            }
            return;
        }
    }
}

static EventoFSM_t fsm_obtener_evento(void)
{
    if (solicitud_emergencia) {
        solicitud_emergencia = 0;
        return EVENTO_EMERGENCIA;
    }

    if (solicitud_recuperar_bt) {
        solicitud_recuperar_bt = 0;
        causa_reset_wdt        = 0;
        tiempo_inactividad_ms  = 0;
        return EVENTO_RECUPERACION_VALIDA;
    }

    if (evento_actividad_fsm) {
        evento_actividad_fsm  = 0;
        tiempo_inactividad_ms = 0;
        /* En ESTADO_SEGURO el pulsador físico también permite recuperar si no hay BT */
        if (estado_actual == ESTADO_SEGURO) {
            causa_reset_wdt = 0;
            return EVENTO_RECUPERACION_VALIDA;
        }
        return EVENTO_ACTIVIDAD_USUARIO;
    }

    if (solicitud_apagado_bt) {
        solicitud_apagado_bt = 0;
        return EVENTO_SOLICITAR_APAGADO;
    }

    if (solicitud_standby_bt) {
        solicitud_standby_bt = 0;
        return EVENTO_SOLICITAR_STANDBY;
    }

    if ((estado_actual == ESTADO_STANDBY) && (tiempo_inactividad_ms >= TIEMPO_A_APAGADO_MS)) {
        return EVENTO_INACTIVIDAD_30S;
    }

    if ((estado_actual == ESTADO_ACTIVO) && (tiempo_inactividad_ms >= TIEMPO_ACTIVO_A_STANDBY_MS)) {
        return EVENTO_INACTIVIDAD_10S;
    }

    return EVENTO_NINGUNO;
}

/* ============================================================================
 * RUTINAS DE SERVICIO DE INTERRUPCIÓN (TIMER0, TIMER1, ADC, INT0, PCINT2)
 * ============================================================================ */

ISR(TIMER0_COMPA_vect)
{
    flag_tick_1ms = 1;
}

ISR(TIMER1_COMPA_vect)
{
    ADCSRA |= (1 << ADSC);
}

ISR(ADC_vect)
{
    adc_valor_actual = ADC;
    flag_adc_listo   = 1;
}

ISR(INT0_vect)
{
    if (estado_actual == ESTADO_APAGADO) {
        /* En Power-Down INT0 está por nivel bajo: se desactiva tras despertar para evitar rebote */
        EIMSK &= ~(1 << INT0);
    } else if (estado_actual != ESTADO_SEGURO) {
        PORTB |= (1 << PIN_BUZZER);
        evento_pulsador_buzzer = 1;
    }
    evento_actividad_fsm = 1;
}

/* Interrupción del Botón de Emergencia en PD4 (PCINT20) */
ISR(PCINT2_vect)
{
    if (!(PIND & (1 << PIN_EMERGENCIA))) {
        solicitud_emergencia = 1;
    }
}

/* ============================================================================
 * PROGRAMA PRINCIPAL
 * ============================================================================ */

int main(void)
{
    uint16_t acum_led_ms = 0;
    uint16_t acum_lcd_ms = 0;
    uint16_t acum_dht_ms = PERIODO_DHT_MS;
    uint16_t adc_copia   = 0;

    /* 1. Inspeccionar MCUSR inmediatamente para detectar reinicio por Watchdog (Prueba B) */
    uint8_t mcusr_copia = MCUSR;
    MCUSR = 0;
    wdt_desactivar_seguro();

    config_reloj_16mhz();
    config_gpio();
    twi_init();
    uart_init();
    config_timer0_1ms();
    config_timer1_100ms();
    config_adc();
    config_int0_pulsador();

    /* Deshabilitar desde el inicio los periféricos que nunca se usan (SPI y Timer2) */
    PRR |= (1 << PRSPI) | (1 << PRTIM2);

    /* 2. Si el reinicio provino de un bloqueo detectado por el WDT, iniciar en ESTADO_SEGURO */
    if (mcusr_copia & (1 << WDRF)) {
        causa_reset_wdt = 1;
        estado_actual   = ESTADO_SEGURO;
        accion_entrar_seguro();
    } else {
        estado_actual   = ESTADO_ACTIVO;
        lcd_preparar_vista_activo(0, 0);
        uart_enviar_trama('S', 'A');
    }

    /* 3. Habilitar WDT en modo Reset a 2 segundos para supervisión continua */
    wdt_config_reset_2s();

    sei();

    while (1) {
        /* 0. Alimentar al Watchdog Timer en cada ciclo normal del super-loop */
        wdt_reset();

        /* 1. Procesar tramas Bluetooth entrantes sin bloqueo */
        uart_procesar_tramas(adc_copia);

        /* 2. Evento inmediato del pulsador (INT0) */
        if (evento_pulsador_buzzer) {
            evento_pulsador_buzzer = 0;
            buzzer_activo          = 1;
            buzzer_timer_ms        = 0;
        }

        /* 3. Evaluación de la Máquina de Estados Finitos */
        fsm_procesar_evento(fsm_obtener_evento());

        /* 4. Captura atómica del resultado del ADC */
        if (flag_adc_listo) {
            cli();
            adc_copia      = adc_valor_actual;
            flag_adc_listo = 0;
            sei();
        }

        /* 5. Planificador cooperativo dirigido por el tick de 1 ms de Timer0 */
        if (flag_tick_1ms) {
            flag_tick_1ms = 0;

            if (tiempo_inactividad_ms < TIEMPO_A_APAGADO_MS) {
                tiempo_inactividad_ms++;
            }

            fsm_procesar_evento(fsm_obtener_evento());

            if (buzzer_activo) {
                buzzer_timer_ms++;
                if (buzzer_timer_ms >= DURACION_BUZZER_MS) {
                    PORTB &= ~(1 << PIN_BUZZER);
                    buzzer_activo = 0;
                }
            }

            if (estado_actual == ESTADO_ACTIVO) {
                acum_led_ms++;
                acum_lcd_ms++;
                acum_dht_ms++;

                if (acum_led_ms >= PERIODO_LED_MS) {
                    acum_led_ms = 0;
                    PINB = (1 << PIN_LED);
                }

                if (acum_lcd_ms >= PERIODO_LCD_MS) {
                    acum_lcd_ms = 0;
                    lcd_preparar_vista_activo(adc_copia, dht_temperatura);
                }

                dht11_tarea_cooperativa_1ms(&acum_dht_ms);
            }

            lcd_tarea_cooperativa_1ms();
        }

        /* 6. Gestión de Modos de Sueño (Problema B - Actividad 1 y 2) */
        if (estado_actual == ESTADO_STANDBY) {
            /* En STANDBY la CPU duerme en modo IDLE entre ticks de 1 ms o tramas UART */
            set_sleep_mode(SLEEP_MODE_IDLE);
            sleep_enable();
            sleep_cpu();
            sleep_disable();
        } else if (estado_actual == ESTADO_APAGADO) {
            /*
             * Espera de forma no bloqueante (~34 ms) a que la LCD termine de apagar su
             * backlight por I2C y la UART termine de enviar ':SP...;' antes de apagar
             * los relojes en PRR y entrar en SLEEP_MODE_PWR_DOWN.
             */
            if ((lcd_paso_actual == 0U) && (!twi_ocupado) && (uart_tx_head == uart_tx_tail)) {
                ejecutar_ciclo_power_down();
            }
        }
    }

    return 0;
}
