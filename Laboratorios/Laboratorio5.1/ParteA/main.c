#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

/* ============================================================================
 * DEFINICIÓN DE PINES (ATmega328P / Arduino Uno)
 * ============================================================================ */

#define PIN_PULSADOR    PD2     /* INT0 - Pin Digital 2 */
#define PIN_DHT11       PD3     /* INT1 - Pin Digital 3 */

#define PIN_LED         PB0     /* Pin Digital 8 (cambia cada 200 ms) */
#define PIN_BUZZER      PB1     /* Pin Digital 9 (activado por INT0) */

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

/* ============================================================================
 * TIPOS DE DATOS DE LA MÁQUINA DE ESTADOS FINITOS (ACTIVIDAD 2)
 * ============================================================================ */

typedef enum {
    ESTADO_ACTIVO  = 0,
    ESTADO_STANDBY = 1,
    ESTADO_APAGADO = 2,
    ESTADO_SEGURO  = 3
} EstadoSistema_t;

typedef enum {
    EVENTO_NINGUNO           = 0,
    EVENTO_INACTIVIDAD_10S   = 1,
    EVENTO_INACTIVIDAD_30S   = 2,
    EVENTO_ACTIVIDAD_USUARIO = 3,
    EVENTO_EMERGENCIA        = 4
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

/* Eventos externos asíncronos (INT0 y Emergencia) */
volatile uint8_t  evento_pulsador_buzzer = 0;
volatile uint8_t  evento_actividad_fsm   = 0;
volatile uint8_t  solicitud_emergencia   = 0;

/* Estado actual de la FSM y contador de inactividad */
EstadoSistema_t estado_actual         = ESTADO_ACTIVO;
uint32_t        tiempo_inactividad_ms = 0;

/* Control temporal no bloqueante del Buzzer */
uint8_t  buzzer_activo   = 0;
uint16_t buzzer_timer_ms = 0;

/* Máquina de estados y captura por interrupción (INT1 + TCNT0) del DHT11 */
volatile uint8_t dht_datos[5]  = {0, 0, 0, 0, 0};
volatile uint8_t dht_pulso_idx = 0;
volatile uint8_t dht_tcnt_prev = 0;
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

/*
 * Se ejecuta una vez por cada tick de 1 ms (Timer0).
 * - Si la LCD aún no se ha inicializado, cumple los tiempos del datasheet HD44780
 *   contando ticks de 1 ms (sin usar _delay_ms ni _delay_us).
 * - Una vez inicializada, envía 1 comando o carácter por cada tick de 1 ms.
 */
static void lcd_tarea_cooperativa_1ms(void)
{
    if (twi_ocupado) {
        return;
    }

    /* Secuencia de inicialización no bloqueante basada en el tick de 1 ms */
    if (!lcd_inicializado) {
        lcd_init_ticks++;

        if (lcd_init_ticks == 20U) {
            lcd_enviar_nibble_async(0x30, 0); /* 1er intento 8 bits tras 20 ms */
        } else if (lcd_init_ticks == 26U) {
            lcd_enviar_nibble_async(0x30, 0); /* 2do intento tras 6 ms */
        } else if (lcd_init_ticks == 28U) {
            lcd_enviar_nibble_async(0x30, 0); /* 3er intento tras 2 ms */
        } else if (lcd_init_ticks == 30U) {
            lcd_enviar_nibble_async(0x20, 0); /* Cambio a modo 4 bits */
        } else if (lcd_init_ticks == 32U) {
            lcd_enviar_byte_async(0x28, 0);   /* 4 bits, 2 líneas, fuente 5x8 */
        } else if (lcd_init_ticks == 34U) {
            lcd_enviar_byte_async(0x0C, 0);   /* Display ON, cursor OFF */
        } else if (lcd_init_ticks == 36U) {
            lcd_enviar_byte_async(0x06, 0);   /* Incremento automático */
        } else if (lcd_init_ticks == 38U) {
            lcd_enviar_byte_async(0x01, 0);   /* Clear display */
        } else if (lcd_init_ticks >= 42U) {
            lcd_inicializado = 1;             /* Fin de los >2 ms del Clear */
            lcd_paso_actual  = 1;
        }
        return;
    }

    /* Envío no bloqueante de los 32 caracteres + 2 comandos de dirección */
    if (lcd_paso_actual == 0U) {
        return;
    } else if (lcd_paso_actual == 1U) {
        lcd_enviar_byte_async(0x80, 0); /* Cursor al inicio de línea 1 */
        lcd_paso_actual++;
    } else if ((lcd_paso_actual >= 2U) && (lcd_paso_actual <= 17U)) {
        lcd_enviar_byte_async((uint8_t)lcd_linea1[lcd_paso_actual - 2U], 1);
        lcd_paso_actual++;
    } else if (lcd_paso_actual == 18U) {
        lcd_enviar_byte_async(0xC0, 0); /* Cursor al inicio de línea 2 */
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

    /* Flanco 0: Inicio de respuesta del DHT11 (80 us en bajo) */
    if (dht_pulso_idx == 0U) {
        dht_pulso_idx = 1U;
    }
    /* Flanco 1: Fin de respuesta del DHT11 e inicio del primer bit */
    else if (dht_pulso_idx == 1U) {
        dht_tcnt_prev = t_actual;
        dht_pulso_idx = 2U;
    }
    /* Flancos 2 al 41: Los 40 bits de datos del DHT11 */
    else if ((dht_pulso_idx >= 2U) && (dht_pulso_idx <= 41U)) {
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
            EIMSK &= ~(1 << INT1); /* Deshabilitar INT1 al completar 40 bits */
            flag_dht_listo = 1U;
        } else {
            dht_pulso_idx++;
        }
    }
}

static void dht11_tarea_cooperativa_1ms(uint16_t *acum_dht_ms)
{
    /* Etapa 0: Esperar el período objetivo de 2 segundos (2000 ms) */
    if (dht_etapa == 0U) {
        if (*acum_dht_ms >= PERIODO_DHT_MS) {
            *acum_dht_ms = 0U;
            /* Iniciar pulso de arranque poniendo PD3 en BAJO durante 20 ms */
            DDRD  |=  (1 << PIN_DHT11);
            PORTD &= ~(1 << PIN_DHT11);
            dht_timer_ms = 0U;
            dht_etapa    = 1U;
        }
    }
    /* Etapa 1: Contar los 20 ms del pulso de arranque sin bloquear */
    else if (dht_etapa == 1U) {
        dht_timer_ms++;
        if (dht_timer_ms >= DURACION_PULSO_DHT_MS) {
            dht_datos[0]   = 0U;
            dht_datos[1]   = 0U;
            dht_datos[2]   = 0U;
            dht_datos[3]   = 0U;
            dht_datos[4]   = 0U;
            dht_pulso_idx  = 0U;
            flag_dht_listo = 0U;

            /* Liberar línea DATA (entrada con pull-up) */
            PORTD |=  (1 << PIN_DHT11);
            DDRD  &= ~(1 << PIN_DHT11);

            /* Configurar INT1 (PD3) por flanco de bajada (ISC11=1, ISC10=0) */
            EICRA = (EICRA & ~(1 << ISC10)) | (1 << ISC11);
            EIFR  = (1 << INTF1);
            EIMSK |= (1 << INT1);

            dht_timer_ms = 0U;
            dht_etapa    = 2U;
        }
    }
    /* Etapa 2: Esperar fin de recepción por INT1 o timeout de seguridad (10 ms) */
    else if (dht_etapa == 2U) {
        dht_timer_ms++;
        if (flag_dht_listo) {
            flag_dht_listo = 0U;
            if ((uint8_t)(dht_datos[0] + dht_datos[1] + dht_datos[2] + dht_datos[3]) == dht_datos[4]) {
                dht_temperatura = dht_datos[2];
            }
            dht_etapa = 0U;
        } else if (dht_timer_ms >= TIMEOUT_TRAMA_DHT_MS) {
            EIMSK &= ~(1 << INT1); /* Abortar si el sensor no respondió */
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
    DDRD  &= ~(1 << PIN_PULSADOR);
    PORTD |=  (1 << PIN_PULSADOR); /* Pull-up interno en PD2 */

    EICRA = (EICRA & ~(1 << ISC00)) | (1 << ISC01); /* Flanco de bajada en INT0 */
    EIFR  = (1 << INTF0);
    EIMSK |= (1 << INT0);
}

static void config_gpio(void)
{
    DDRB  |=  (1 << PIN_LED) | (1 << PIN_BUZZER);
    PORTB &= ~((1 << PIN_LED) | (1 << PIN_BUZZER));

    DDRD  &= ~(1 << PIN_DHT11);
    PORTD |=  (1 << PIN_DHT11);
}

/* ============================================================================
 * MÁQUINA DE ESTADOS FINITOS (ACTIVIDAD 2 - TABLA DE TRANSICIONES)
 * ============================================================================ */

static void accion_entrar_activo(void)
{
    ADCSRA |= (1 << ADEN) | (1 << ADSC);
    TIMSK1 |= (1 << OCIE1A);

    lcd_set_backlight(1);
    lcd_preparar_vista_activo(adc_valor_actual, dht_temperatura);
}

static void accion_entrar_standby(void)
{
    PORTB &= ~(1 << PIN_LED);
    EIMSK &= ~(1 << INT1);
    DDRD  &= ~(1 << PIN_DHT11);
    PORTD |=  (1 << PIN_DHT11);
    dht_etapa = 0U;

    lcd_set_backlight(1);
    lcd_preparar_mensaje_estado("Esperando...    ", "ESTADO: STANDBY ");
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
}

static void accion_entrar_seguro(void)
{
    TIMSK1 &= ~(1 << OCIE1A);
    ADCSRA &= ~(1 << ADEN);
    EIMSK  &= ~(1 << INT1);
    PORTB  &= ~((1 << PIN_LED) | (1 << PIN_BUZZER));

    lcd_set_backlight(1);
    lcd_preparar_mensaje_estado("!! EMERGENCIA !!", "ESTADO: SEGURO  ");
}

static const TransicionFSM_t tabla_transiciones[] = {
    { ESTADO_ACTIVO,  EVENTO_INACTIVIDAD_10S,   ESTADO_STANDBY, accion_entrar_standby },
    { ESTADO_STANDBY, EVENTO_INACTIVIDAD_30S,   ESTADO_APAGADO, accion_entrar_apagado },
    { ESTADO_STANDBY, EVENTO_ACTIVIDAD_USUARIO, ESTADO_ACTIVO,  accion_entrar_activo  },
    { ESTADO_APAGADO, EVENTO_ACTIVIDAD_USUARIO, ESTADO_ACTIVO,  accion_entrar_activo  },
    { ESTADO_ACTIVO,  EVENTO_EMERGENCIA,        ESTADO_SEGURO,  accion_entrar_seguro  },
    { ESTADO_STANDBY, EVENTO_EMERGENCIA,        ESTADO_SEGURO,  accion_entrar_seguro  },
    { ESTADO_APAGADO, EVENTO_EMERGENCIA,        ESTADO_SEGURO,  accion_entrar_seguro  }
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

    if (evento_actividad_fsm) {
        evento_actividad_fsm  = 0;
        tiempo_inactividad_ms = 0;
        return EVENTO_ACTIVIDAD_USUARIO;
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
 * RUTINAS DE SERVICIO DE INTERRUPCIÓN (TIMER0, TIMER1, ADC, INT0)
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
    PORTB |= (1 << PIN_BUZZER);
    evento_pulsador_buzzer = 1;
    evento_actividad_fsm   = 1;
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

    config_reloj_16mhz();
    config_gpio();
    twi_init();
    config_timer0_1ms();
    config_timer1_100ms();
    config_adc();
    config_int0_pulsador();

    lcd_preparar_vista_activo(0, 0);

    sei();

    while (1) {
        /* 1. Evento inmediato del pulsador (INT0) */
        if (evento_pulsador_buzzer) {
            evento_pulsador_buzzer = 0;
            buzzer_activo          = 1;
            buzzer_timer_ms        = 0;
        }

        /* 2. Evaluación de la Máquina de Estados Finitos */
        fsm_procesar_evento(fsm_obtener_evento());

        /* 3. Captura del resultado del ADC (solicitado cada 100 ms por Timer1) */
        if (flag_adc_listo) {
            cli();
            adc_copia      = adc_valor_actual;
            flag_adc_listo = 0;
            sei();
        }

        /* 4. Planificador cooperativo dirigido por el tick de 1 ms de Timer0 */
        if (flag_tick_1ms) {
            flag_tick_1ms = 0;

            /* Avance del contador de inactividad hacia 10 s (STANDBY) y 30 s (APAGADO) */
            if (tiempo_inactividad_ms < TIEMPO_A_APAGADO_MS) {
                tiempo_inactividad_ms++;
            }

            /* Evaluar transiciones por tiempo antes de ejecutar las tareas del estado */
            fsm_procesar_evento(fsm_obtener_evento());

            /* Apagado cooperativo del buzzer tras 150 ms */
            if (buzzer_activo) {
                buzzer_timer_ms++;
                if (buzzer_timer_ms >= DURACION_BUZZER_MS) {
                    PORTB &= ~(1 << PIN_BUZZER);
                    buzzer_activo = 0;
                }
            }

            /* Tareas periódicas activas únicamente en ESTADO_ACTIVO */
            if (estado_actual == ESTADO_ACTIVO) {
                acum_led_ms++;
                acum_lcd_ms++;
                acum_dht_ms++;

                /* TAREA A: Parpadeo del LED cada 200 ms */
                if (acum_led_ms >= PERIODO_LED_MS) {
                    acum_led_ms = 0;
                    PINB = (1 << PIN_LED);
                }

                /* TAREA B: Preparar nuevos datos para la LCD cada 500 ms */
                if (acum_lcd_ms >= PERIODO_LCD_MS) {
                    acum_lcd_ms = 0;
                    lcd_preparar_vista_activo(adc_copia, dht_temperatura);
                }

                /* TAREA C: Máquina de estados del DHT11 cada 2000 ms */
                dht11_tarea_cooperativa_1ms(&acum_dht_ms);
            }

            /* Avanza 1 paso de la LCD por cada ms (tanto en ACTIVO como en transiciones) */
            lcd_tarea_cooperativa_1ms();
        }
    }

    return 0;
}#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

/* ============================================================================
 * DEFINICIÓN DE PINES (ATmega328P / Arduino Uno)
 * ============================================================================ */

#define PIN_PULSADOR    PD2     /* INT0 - Pin Digital 2 */
#define PIN_DHT11       PD3     /* INT1 - Pin Digital 3 */

#define PIN_LED         PB0     /* Pin Digital 8 (cambia cada 200 ms) */
#define PIN_BUZZER      PB1     /* Pin Digital 9 (activado por INT0) */

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

/* ============================================================================
 * TIPOS DE DATOS DE LA MÁQUINA DE ESTADOS FINITOS (ACTIVIDAD 2)
 * ============================================================================ */

typedef enum {
    ESTADO_ACTIVO  = 0,
    ESTADO_STANDBY = 1,
    ESTADO_APAGADO = 2,
    ESTADO_SEGURO  = 3
} EstadoSistema_t;

typedef enum {
    EVENTO_NINGUNO           = 0,
    EVENTO_INACTIVIDAD_10S   = 1,
    EVENTO_INACTIVIDAD_30S   = 2,
    EVENTO_ACTIVIDAD_USUARIO = 3,
    EVENTO_EMERGENCIA        = 4
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

/* Eventos externos asíncronos (INT0 y Emergencia) */
volatile uint8_t  evento_pulsador_buzzer = 0;
volatile uint8_t  evento_actividad_fsm   = 0;
volatile uint8_t  solicitud_emergencia   = 0;

/* Estado actual de la FSM y contador de inactividad */
EstadoSistema_t estado_actual         = ESTADO_ACTIVO;
uint32_t        tiempo_inactividad_ms = 0;

/* Control temporal no bloqueante del Buzzer */
uint8_t  buzzer_activo   = 0;
uint16_t buzzer_timer_ms = 0;

/* Máquina de estados y captura por interrupción (INT1 + TCNT0) del DHT11 */
volatile uint8_t dht_datos[5]  = {0, 0, 0, 0, 0};
volatile uint8_t dht_pulso_idx = 0;
volatile uint8_t dht_tcnt_prev = 0;
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

/*
 * Se ejecuta una vez por cada tick de 1 ms (Timer0).
 * - Si la LCD aún no se ha inicializado, cumple los tiempos del datasheet HD44780
 *   contando ticks de 1 ms (sin usar _delay_ms ni _delay_us).
 * - Una vez inicializada, envía 1 comando o carácter por cada tick de 1 ms.
 */
static void lcd_tarea_cooperativa_1ms(void)
{
    if (twi_ocupado) {
        return;
    }

    /* Secuencia de inicialización no bloqueante basada en el tick de 1 ms */
    if (!lcd_inicializado) {
        lcd_init_ticks++;

        if (lcd_init_ticks == 20U) {
            lcd_enviar_nibble_async(0x30, 0); /* 1er intento 8 bits tras 20 ms */
        } else if (lcd_init_ticks == 26U) {
            lcd_enviar_nibble_async(0x30, 0); /* 2do intento tras 6 ms */
        } else if (lcd_init_ticks == 28U) {
            lcd_enviar_nibble_async(0x30, 0); /* 3er intento tras 2 ms */
        } else if (lcd_init_ticks == 30U) {
            lcd_enviar_nibble_async(0x20, 0); /* Cambio a modo 4 bits */
        } else if (lcd_init_ticks == 32U) {
            lcd_enviar_byte_async(0x28, 0);   /* 4 bits, 2 líneas, fuente 5x8 */
        } else if (lcd_init_ticks == 34U) {
            lcd_enviar_byte_async(0x0C, 0);   /* Display ON, cursor OFF */
        } else if (lcd_init_ticks == 36U) {
            lcd_enviar_byte_async(0x06, 0);   /* Incremento automático */
        } else if (lcd_init_ticks == 38U) {
            lcd_enviar_byte_async(0x01, 0);   /* Clear display */
        } else if (lcd_init_ticks >= 42U) {
            lcd_inicializado = 1;             /* Fin de los >2 ms del Clear */
            lcd_paso_actual  = 1;
        }
        return;
    }

    /* Envío no bloqueante de los 32 caracteres + 2 comandos de dirección */
    if (lcd_paso_actual == 0U) {
        return;
    } else if (lcd_paso_actual == 1U) {
        lcd_enviar_byte_async(0x80, 0); /* Cursor al inicio de línea 1 */
        lcd_paso_actual++;
    } else if ((lcd_paso_actual >= 2U) && (lcd_paso_actual <= 17U)) {
        lcd_enviar_byte_async((uint8_t)lcd_linea1[lcd_paso_actual - 2U], 1);
        lcd_paso_actual++;
    } else if (lcd_paso_actual == 18U) {
        lcd_enviar_byte_async(0xC0, 0); /* Cursor al inicio de línea 2 */
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

    /* Flanco 0: Inicio de respuesta del DHT11 (80 us en bajo) */
    if (dht_pulso_idx == 0U) {
        dht_pulso_idx = 1U;
    }
    /* Flanco 1: Fin de respuesta del DHT11 e inicio del primer bit */
    else if (dht_pulso_idx == 1U) {
        dht_tcnt_prev = t_actual;
        dht_pulso_idx = 2U;
    }
    /* Flancos 2 al 41: Los 40 bits de datos del DHT11 */
    else if ((dht_pulso_idx >= 2U) && (dht_pulso_idx <= 41U)) {
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
            EIMSK &= ~(1 << INT1); /* Deshabilitar INT1 al completar 40 bits */
            flag_dht_listo = 1U;
        } else {
            dht_pulso_idx++;
        }
    }
}

static void dht11_tarea_cooperativa_1ms(uint16_t *acum_dht_ms)
{
    /* Etapa 0: Esperar el período objetivo de 2 segundos (2000 ms) */
    if (dht_etapa == 0U) {
        if (*acum_dht_ms >= PERIODO_DHT_MS) {
            *acum_dht_ms = 0U;
            /* Iniciar pulso de arranque poniendo PD3 en BAJO durante 20 ms */
            DDRD  |=  (1 << PIN_DHT11);
            PORTD &= ~(1 << PIN_DHT11);
            dht_timer_ms = 0U;
            dht_etapa    = 1U;
        }
    }
    /* Etapa 1: Contar los 20 ms del pulso de arranque sin bloquear */
    else if (dht_etapa == 1U) {
        dht_timer_ms++;
        if (dht_timer_ms >= DURACION_PULSO_DHT_MS) {
            dht_datos[0]   = 0U;
            dht_datos[1]   = 0U;
            dht_datos[2]   = 0U;
            dht_datos[3]   = 0U;
            dht_datos[4]   = 0U;
            dht_pulso_idx  = 0U;
            flag_dht_listo = 0U;

            /* Liberar línea DATA (entrada con pull-up) */
            PORTD |=  (1 << PIN_DHT11);
            DDRD  &= ~(1 << PIN_DHT11);

            /* Configurar INT1 (PD3) por flanco de bajada (ISC11=1, ISC10=0) */
            EICRA = (EICRA & ~(1 << ISC10)) | (1 << ISC11);
            EIFR  = (1 << INTF1);
            EIMSK |= (1 << INT1);

            dht_timer_ms = 0U;
            dht_etapa    = 2U;
        }
    }
    /* Etapa 2: Esperar fin de recepción por INT1 o timeout de seguridad (10 ms) */
    else if (dht_etapa == 2U) {
        dht_timer_ms++;
        if (flag_dht_listo) {
            flag_dht_listo = 0U;
            if ((uint8_t)(dht_datos[0] + dht_datos[1] + dht_datos[2] + dht_datos[3]) == dht_datos[4]) {
                dht_temperatura = dht_datos[2];
            }
            dht_etapa = 0U;
        } else if (dht_timer_ms >= TIMEOUT_TRAMA_DHT_MS) {
            EIMSK &= ~(1 << INT1); /* Abortar si el sensor no respondió */
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
    DDRD  &= ~(1 << PIN_PULSADOR);
    PORTD |=  (1 << PIN_PULSADOR); /* Pull-up interno en PD2 */

    EICRA = (EICRA & ~(1 << ISC00)) | (1 << ISC01); /* Flanco de bajada en INT0 */
    EIFR  = (1 << INTF0);
    EIMSK |= (1 << INT0);
}

static void config_gpio(void)
{
    DDRB  |=  (1 << PIN_LED) | (1 << PIN_BUZZER);
    PORTB &= ~((1 << PIN_LED) | (1 << PIN_BUZZER));

    DDRD  &= ~(1 << PIN_DHT11);
    PORTD |=  (1 << PIN_DHT11);
}

/* ============================================================================
 * MÁQUINA DE ESTADOS FINITOS (ACTIVIDAD 2 - TABLA DE TRANSICIONES)
 * ============================================================================ */

static void accion_entrar_activo(void)
{
    ADCSRA |= (1 << ADEN) | (1 << ADSC);
    TIMSK1 |= (1 << OCIE1A);

    lcd_set_backlight(1);
    lcd_preparar_vista_activo(adc_valor_actual, dht_temperatura);
}

static void accion_entrar_standby(void)
{
    PORTB &= ~(1 << PIN_LED);
    EIMSK &= ~(1 << INT1);
    DDRD  &= ~(1 << PIN_DHT11);
    PORTD |=  (1 << PIN_DHT11);
    dht_etapa = 0U;

    lcd_set_backlight(1);
    lcd_preparar_mensaje_estado("Esperando...    ", "ESTADO: STANDBY ");
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
}

static void accion_entrar_seguro(void)
{
    TIMSK1 &= ~(1 << OCIE1A);
    ADCSRA &= ~(1 << ADEN);
    EIMSK  &= ~(1 << INT1);
    PORTB  &= ~((1 << PIN_LED) | (1 << PIN_BUZZER));

    lcd_set_backlight(1);
    lcd_preparar_mensaje_estado("!! EMERGENCIA !!", "ESTADO: SEGURO  ");
}

static const TransicionFSM_t tabla_transiciones[] = {
    { ESTADO_ACTIVO,  EVENTO_INACTIVIDAD_10S,   ESTADO_STANDBY, accion_entrar_standby },
    { ESTADO_STANDBY, EVENTO_INACTIVIDAD_30S,   ESTADO_APAGADO, accion_entrar_apagado },
    { ESTADO_STANDBY, EVENTO_ACTIVIDAD_USUARIO, ESTADO_ACTIVO,  accion_entrar_activo  },
    { ESTADO_APAGADO, EVENTO_ACTIVIDAD_USUARIO, ESTADO_ACTIVO,  accion_entrar_activo  },
    { ESTADO_ACTIVO,  EVENTO_EMERGENCIA,        ESTADO_SEGURO,  accion_entrar_seguro  },
    { ESTADO_STANDBY, EVENTO_EMERGENCIA,        ESTADO_SEGURO,  accion_entrar_seguro  },
    { ESTADO_APAGADO, EVENTO_EMERGENCIA,        ESTADO_SEGURO,  accion_entrar_seguro  }
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

    if (evento_actividad_fsm) {
        evento_actividad_fsm  = 0;
        tiempo_inactividad_ms = 0;
        return EVENTO_ACTIVIDAD_USUARIO;
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
 * RUTINAS DE SERVICIO DE INTERRUPCIÓN (TIMER0, TIMER1, ADC, INT0)
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
    PORTB |= (1 << PIN_BUZZER);
    evento_pulsador_buzzer = 1;
    evento_actividad_fsm   = 1;
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

    config_reloj_16mhz();
    config_gpio();
    twi_init();
    config_timer0_1ms();
    config_timer1_100ms();
    config_adc();
    config_int0_pulsador();

    lcd_preparar_vista_activo(0, 0);

    sei();

    while (1) {
        /* 1. Evento inmediato del pulsador (INT0) */
        if (evento_pulsador_buzzer) {
            evento_pulsador_buzzer = 0;
            buzzer_activo          = 1;
            buzzer_timer_ms        = 0;
        }

        /* 2. Evaluación de la Máquina de Estados Finitos */
        fsm_procesar_evento(fsm_obtener_evento());

        /* 3. Captura del resultado del ADC (solicitado cada 100 ms por Timer1) */
        if (flag_adc_listo) {
            cli();
            adc_copia      = adc_valor_actual;
            flag_adc_listo = 0;
            sei();
        }

        /* 4. Planificador cooperativo dirigido por el tick de 1 ms de Timer0 */
        if (flag_tick_1ms) {
            flag_tick_1ms = 0;

            /* Avance del contador de inactividad hacia 10 s (STANDBY) y 30 s (APAGADO) */
            if (tiempo_inactividad_ms < TIEMPO_A_APAGADO_MS) {
                tiempo_inactividad_ms++;
            }

            /* Evaluar transiciones por tiempo antes de ejecutar las tareas del estado */
            fsm_procesar_evento(fsm_obtener_evento());

            /* Apagado cooperativo del buzzer tras 150 ms */
            if (buzzer_activo) {
                buzzer_timer_ms++;
                if (buzzer_timer_ms >= DURACION_BUZZER_MS) {
                    PORTB &= ~(1 << PIN_BUZZER);
                    buzzer_activo = 0;
                }
            }

            /* Tareas periódicas activas únicamente en ESTADO_ACTIVO */
            if (estado_actual == ESTADO_ACTIVO) {
                acum_led_ms++;
                acum_lcd_ms++;
                acum_dht_ms++;

                /* TAREA A: Parpadeo del LED cada 200 ms */
                if (acum_led_ms >= PERIODO_LED_MS) {
                    acum_led_ms = 0;
                    PINB = (1 << PIN_LED);
                }

                /* TAREA B: Preparar nuevos datos para la LCD cada 500 ms */
                if (acum_lcd_ms >= PERIODO_LCD_MS) {
                    acum_lcd_ms = 0;
                    lcd_preparar_vista_activo(adc_copia, dht_temperatura);
                }

                /* TAREA C: Máquina de estados del DHT11 cada 2000 ms */
                dht11_tarea_cooperativa_1ms(&acum_dht_ms);
            }

            /* Avanza 1 paso de la LCD por cada ms (tanto en ACTIVO como en transiciones) */
            lcd_tarea_cooperativa_1ms();
        }
    }

    return 0;
}
