#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));
void wdt_init(void) {
    MCUSR = 0;
    wdt_disable();
}

ISR(BADISR_vect) {}

#define DHT_PIN                 PD7
#define DHT_PORT                PORTD
#define DHT_DDR                 DDRD
#define DHT_IN                  PIND

#define CALEFACTOR_DDR          DDRB
#define CALEFACTOR_PORT         PORTB
#define CALEFACTOR_PIN          PB0

#define VENTILADOR_DDR          DDRD
#define VENTILADOR_PORT         PORTD
#define VENTILADOR_PIN          PD6

#define LCD_CTRL_DDR            DDRB
#define LCD_CTRL_PORT           PORTB
#define LCD_RS_PIN              PB1
#define LCD_E_PIN               PB2

#define LCD_DATA_DDR            DDRD
#define LCD_DATA_PORT           PORTD
#define LCD_D4_PIN              PD2
#define LCD_D5_PIN              PD3
#define LCD_D6_PIN              PD4
#define LCD_D7_PIN              PD5

#define PWM_OFF                 0
#define PWM_BAJA                90
#define PWM_MEDIA               180
#define PWM_ALTA                255

#define PUNTO_MEDIO_DEFAULT     20.5f
#define INTERVALO_MEDICION_SEG  5

#define TEMP_SEGURA_MIN         10.0f
#define TEMP_SEGURA_MAX         38.0f

typedef enum {
    ESTADO_CALOR = 0,
    ESTADO_CONFORT,
    ESTADO_VENT_BAJA,
    ESTADO_VENT_MEDIA,
    ESTADO_VENT_ALTA
} EstadoSistema_t;

typedef struct {
    float punto_medio;
    float umbral_cal;
    float umbral_conf_min;
    float umbral_conf_max;
    float umbral_baja_max;
    float umbral_med_max;
} Umbrales_t;

static volatile uint8_t g_segundos = 0;
static volatile uint8_t g_flag_medir = 1;

static Umbrales_t g_umbrales;
static EstadoSistema_t g_estado = ESTADO_CONFORT;
static float g_temp_actual = 20.5f;

void uart_init(uint32_t baud) {
    uint16_t ubrr = (uint16_t)((F_CPU / (16UL * baud)) - 1UL);
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr & 0xFF);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_transmit(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void uart_print(const char *s) {
    while (*s) {
        if (*s == '\n') uart_transmit('\r');
        uart_transmit(*s++);
    }
}

void uart_println(const char *s) {
    uart_print(s);
    uart_transmit('\r');
    uart_transmit('\n');
}

void uart_print_int(int32_t v) {
    if (v < 0) {
        uart_transmit('-');
        v = -v;
    }
    if (v == 0) {
        uart_transmit('0');
        return;
    }
    char buf[12];
    uint8_t i = 0;
    while (v > 0) {
        buf[i++] = (char)('0' + (v % 10));
        v /= 10;
    }
    while (i > 0) {
        uart_transmit(buf[--i]);
    }
}

void uart_print_float(float v, uint8_t dec) {
    if (v < 0.0f) {
        uart_transmit('-');
        v = -v;
    }
    int32_t entera = (int32_t)v;
    uart_print_int(entera);
    if (dec > 0) {
        uart_transmit('.');
        float res = v - (float)entera;
        for (uint8_t i = 0; i < dec; i++) {
            res *= 10.0f;
            uint8_t d = (uint8_t)res;
            uart_transmit((char)('0' + d));
            res -= (float)d;
        }
    }
}

uint8_t uart_available(void) {
    return (UCSR0A & (1 << RXC0)) ? 1 : 0;
}

char uart_receive(void) {
    while (!(UCSR0A & (1 << RXC0)));
    return UDR0;
}

bool uart_receive_timeout(char *c_out, uint16_t timeout_ms) {
    while (timeout_ms--) {
        if (uart_available()) {
            *c_out = UDR0;
            return true;
        }
        _delay_ms(1);
    }
    return false;
}

int8_t dht11_read(float *temp_out) {
    uint8_t data[5] = {0, 0, 0, 0, 0};
    uint8_t i, j;
    uint16_t timeout;

    // Pulso inicial LOW de 20 ms
    DHT_DDR |= (1 << DHT_PIN);
    DHT_PORT &= ~(1 << DHT_PIN);
    _delay_ms(20);

    // Pulso HIGH de 30 µs y pasar a entrada
    DHT_PORT |= (1 << DHT_PIN);
    _delay_us(30);
    DHT_DDR &= ~(1 << DHT_PIN);
    DHT_PORT |= (1 << DHT_PIN);

    timeout = 10000;
    while ((DHT_IN & (1 << DHT_PIN)) && --timeout);
    if (timeout == 0) return -1;

    timeout = 10000;
    while (!(DHT_IN & (1 << DHT_PIN)) && --timeout);
    if (timeout == 0) return -2;

    timeout = 10000;
    while ((DHT_IN & (1 << DHT_PIN)) && --timeout);
    if (timeout == 0) return -3;

    // Lectura de 40 bits con interrupciones deshabilitadas
    cli();
    for (j = 0; j < 5; j++) {
        uint8_t byte_val = 0;
        for (i = 0; i < 8; i++) {
            timeout = 10000;
            while (!(DHT_IN & (1 << DHT_PIN)) && --timeout);
            if (timeout == 0) { sei(); return -4; }

            _delay_us(38);
            if (DHT_IN & (1 << DHT_PIN)) {
                byte_val |= (1 << (7 - i));
                timeout = 10000;
                while ((DHT_IN & (1 << DHT_PIN)) && --timeout);
                if (timeout == 0) { sei(); return -5; }
            }
        }
        data[j] = byte_val;
    }
    sei();

    uint8_t checksum = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
    if (data[4] != checksum) return -6;

    float t = (float)data[2];
    if (data[3] > 0 && data[3] < 10) t += (float)data[3] * 0.1f;
    else if (data[3] >= 10) t += (float)data[3] * 0.01f;

    *temp_out = t;
    return 0;
}

float leer_temperatura(void) {
    float temp_dht = 0.0f;
    int8_t err = dht11_read(&temp_dht);

    if (err == 0 && temp_dht >= 0.0f && temp_dht <= 55.0f) {
        return temp_dht;
    }
    return g_temp_actual;
}

void actuadores_init(void) {
    CALEFACTOR_DDR |= (1 << CALEFACTOR_PIN);
    CALEFACTOR_PORT &= ~(1 << CALEFACTOR_PIN);

    VENTILADOR_DDR |= (1 << VENTILADOR_PIN);
    VENTILADOR_PORT &= ~(1 << VENTILADOR_PIN);

    TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A = 0;
}

void calefactor_set(bool encender) {
    if (encender) CALEFACTOR_PORT |= (1 << CALEFACTOR_PIN);
    else          CALEFACTOR_PORT &= ~(1 << CALEFACTOR_PIN);
}

void ventilador_set_pwm(uint8_t duty) {
    if (duty == 0) {
        TCCR0A &= ~(1 << COM0A1);
        VENTILADOR_PORT &= ~(1 << VENTILADOR_PIN);
    } else {
        TCCR0A |= (1 << COM0A1);
        OCR0A = duty;
    }
}

static void lcd_nibble(uint8_t n) {
    if (n & 0x01) LCD_DATA_PORT |= (1 << LCD_D4_PIN);
    else          LCD_DATA_PORT &= ~(1 << LCD_D4_PIN);

    if (n & 0x02) LCD_DATA_PORT |= (1 << LCD_D5_PIN);
    else          LCD_DATA_PORT &= ~(1 << LCD_D5_PIN);

    if (n & 0x04) LCD_DATA_PORT |= (1 << LCD_D6_PIN);
    else          LCD_DATA_PORT &= ~(1 << LCD_D6_PIN);

    if (n & 0x08) LCD_DATA_PORT |= (1 << LCD_D7_PIN);
    else          LCD_DATA_PORT &= ~(1 << LCD_D7_PIN);

    LCD_CTRL_PORT |= (1 << LCD_E_PIN);
    _delay_us(2);
    LCD_CTRL_PORT &= ~(1 << LCD_E_PIN);
    _delay_us(50);
}

static void lcd_byte(uint8_t b, uint8_t es_dato) {
    if (es_dato) LCD_CTRL_PORT |= (1 << LCD_RS_PIN);
    else         LCD_CTRL_PORT &= ~(1 << LCD_RS_PIN);
    _delay_us(1);

    lcd_nibble((b >> 4) & 0x0F);
    lcd_nibble(b & 0x0F);
    _delay_us(50);
}

void lcd_cmd(uint8_t c) {
    lcd_byte(c, 0);
    if (c == 0x01 || c == 0x02) _delay_ms(2);
}

void lcd_data(uint8_t d) {
    lcd_byte(d, 1);
}

void lcd_clear(void) {
    lcd_cmd(0x01);
    _delay_ms(2);
}

void lcd_gotoxy(uint8_t col, uint8_t row) {
    uint8_t addr = (row == 0) ? (0x80 + col) : (0xC0 + col);
    lcd_cmd(addr);
}

void lcd_print(const char *s) {
    while (*s) lcd_data((uint8_t)(*s++));
}

void lcd_print_int(int16_t v) {
    if (v < 0) {
        lcd_data('-');
        v = -v;
    }
    if (v == 0) {
        lcd_data('0');
        return;
    }
    char buf[8];
    uint8_t i = 0;
    while (v > 0) {
        buf[i++] = (char)('0' + (v % 10));
        v /= 10;
    }
    while (i > 0) {
        lcd_data((uint8_t)buf[--i]);
    }
}

void lcd_print_float(float v, uint8_t dec) {
    if (v < 0.0f) {
        lcd_data('-');
        v = -v;
    }
    int16_t ent = (int16_t)v;
    lcd_print_int(ent);
    if (dec > 0) {
        lcd_data('.');
        float res = v - (float)ent;
        for (uint8_t i = 0; i < dec; i++) {
            res *= 10.0f;
            uint8_t d = (uint8_t)res;
            lcd_data((uint8_t)('0' + d));
            res -= (float)d;
        }
    }
}

void lcd_crear_grado(void) {
    uint8_t g[8] = {
        0b00110, 0b01001, 0b01001, 0b00110,
        0b00000, 0b00000, 0b00000, 0b00000
    };
    lcd_cmd(0x40);
    for (uint8_t i = 0; i < 8; i++) lcd_data(g[i]);
    lcd_cmd(0x80);
}

void lcd_init(void) {
    LCD_CTRL_DDR |= (1 << LCD_RS_PIN) | (1 << LCD_E_PIN);
    LCD_DATA_DDR |= (1 << LCD_D4_PIN) | (1 << LCD_D5_PIN) | (1 << LCD_D6_PIN) | (1 << LCD_D7_PIN);
    LCD_CTRL_PORT &= ~((1 << LCD_RS_PIN) | (1 << LCD_E_PIN));

    _delay_ms(50);
    lcd_nibble(0x03);
    _delay_ms(5);
    lcd_nibble(0x03);
    _delay_us(150);
    lcd_nibble(0x03);
    _delay_us(150);
    lcd_nibble(0x02);
    _delay_us(150);

    lcd_cmd(0x28);
    lcd_cmd(0x08);
    lcd_cmd(0x01);
    _delay_ms(2);
    lcd_cmd(0x06);
    lcd_cmd(0x0C);
    lcd_crear_grado();
}

void timer1_init_1hz(void) {
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS12);
    OCR1A = 62499;
    TIMSK1 = (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect) {
    g_segundos++;
    if (g_segundos >= INTERVALO_MEDICION_SEG) {
        g_segundos = 0;
        g_flag_medir = 1;
    }
}

void recalcular_umbrales(float sp) {
    g_umbrales.punto_medio = sp;
    float delta = sp - PUNTO_MEDIO_DEFAULT;

    g_umbrales.umbral_cal      = 15.0f + delta;
    g_umbrales.umbral_conf_min = 16.0f + delta;
    g_umbrales.umbral_conf_max = 25.0f + delta;
    g_umbrales.umbral_baja_max = 35.0f + delta;
    g_umbrales.umbral_med_max  = 45.0f + delta;
}

void ejecutar_control(float temp) {
    if (g_estado == ESTADO_CALOR) {
        if (temp >= g_umbrales.umbral_conf_min) {
            g_estado = ESTADO_CONFORT;
            calefactor_set(false);
            ventilador_set_pwm(PWM_OFF);
        }
    } else {
        if (temp <= g_umbrales.umbral_cal) {
            g_estado = ESTADO_CALOR;
            calefactor_set(true);
            ventilador_set_pwm(PWM_OFF);
        } else if (temp <= g_umbrales.umbral_conf_max) {
            g_estado = ESTADO_CONFORT;
            calefactor_set(false);
            ventilador_set_pwm(PWM_OFF);
        } else if (temp <= g_umbrales.umbral_baja_max) {
            g_estado = ESTADO_VENT_BAJA;
            calefactor_set(false);
            ventilador_set_pwm(PWM_BAJA);
        } else if (temp <= g_umbrales.umbral_med_max) {
            g_estado = ESTADO_VENT_MEDIA;
            calefactor_set(false);
            ventilador_set_pwm(PWM_MEDIA);
        } else {
            g_estado = ESTADO_VENT_ALTA;
            calefactor_set(false);
            ventilador_set_pwm(PWM_ALTA);
        }
    }
}

void actualizar_lcd(float temp, EstadoSistema_t st) {
    lcd_gotoxy(0, 0);
    lcd_print("T:");
    lcd_print_float(temp, 1);
    lcd_data(0);
    lcd_print("C ");

    switch (st) {
        case ESTADO_CALOR:      lcd_print("CAL:ON "); break;
        case ESTADO_CONFORT:    lcd_print("CONFORT"); break;
        case ESTADO_VENT_BAJA:  lcd_print("VEN:35%"); break;
        case ESTADO_VENT_MEDIA: lcd_print("VEN:70%"); break;
        case ESTADO_VENT_ALTA:  lcd_print("VEN:MAX"); break;
    }

    lcd_gotoxy(0, 1);
    lcd_print("[");
    lcd_print_int((int16_t)g_umbrales.umbral_conf_min);
    lcd_print("-");
    lcd_print_int((int16_t)g_umbrales.umbral_conf_max);
    lcd_print("]C SP:");
    lcd_print_float(g_umbrales.punto_medio, 1);
}

void enviar_telemetria(float temp, EstadoSistema_t st) {
    const char *str_cal = (st == ESTADO_CALOR) ? "ON" : "OFF";
    const char *str_accion = "CONFORT";
    uint8_t pwm_val = 0;

    switch (st) {
        case ESTADO_CALOR:      str_accion = "CALEFACTOR ACTIVO"; break;
        case ESTADO_CONFORT:    str_accion = "ZONA CONFORT (OFF)"; break;
        case ESTADO_VENT_BAJA:  str_accion = "VENTILADOR BAJA (35%)"; pwm_val = PWM_BAJA; break;
        case ESTADO_VENT_MEDIA: str_accion = "VENTILADOR MEDIA (70%)"; pwm_val = PWM_MEDIA; break;
        case ESTADO_VENT_ALTA:  str_accion = "VENTILADOR ALTA (100%)"; pwm_val = PWM_ALTA; break;
    }

    uart_print("[5s] T: ");
    uart_print_float(temp, 1);
    uart_print(" C | SP: ");
    uart_print_float(g_umbrales.punto_medio, 1);
    uart_print(" C | Cal: ");
    uart_print(str_cal);
    uart_print(" | Fan PWM: ");
    uart_print_int(pwm_val);
    uart_print(" | ");
    uart_println(str_accion);

    uart_print("DATA:");
    uart_print_float(temp, 1);
    uart_transmit(',');
    uart_print_float(g_umbrales.punto_medio, 1);
    uart_transmit(',');
    uart_print_int((st == ESTADO_CALOR) ? 1 : 0);
    uart_transmit(',');
    uart_print_int(pwm_val);
    uart_println("");
}

bool validar_y_aplicar_sp(float nuevo_sp) {
    if (nuevo_sp < TEMP_SEGURA_MIN || nuevo_sp > TEMP_SEGURA_MAX) {
        uart_println("\r\n[ALERTA DE SEGURIDAD]");
        uart_print("Punto medio ");
        uart_print_float(nuevo_sp, 1);
        uart_println(" C RECHAZADO.");
        uart_println("Queda muy cercano al limite maximo del sensor DHT11 (50 C) o bajo 10 C.\r\n");
        return false;
    }

    recalcular_umbrales(nuevo_sp);
    uart_print("\r\n[OK] Punto medio actualizado a: ");
    uart_print_float(nuevo_sp, 1);
    uart_println(" C. Umbrales recalibrados con exito.\r\n");
    g_flag_medir = 1;
    return true;
}

void leer_nuevo_punto_medio_seguro(void) {
    char buf[12];
    uint8_t idx = 0;
    _delay_ms(30);
    while (uart_available()) {
        char dummy = UDR0;
        (void)dummy;
    }

    uart_print("\r\nIngrese nuevo valor (Ej: 22.5) y Enter (Timeout 8s): ");

    while (idx < 10) {
        char c;
        if (!uart_receive_timeout(&c, 8000)) {
            uart_println("\r\n[TIMEOUT] Tiempo agotado. Operacion cancelada.");
            return;
        }

        if (c == '\r' || c == '\n') {
            if (idx == 0) {
                continue;
            }
            uart_println("");
            break;
        } else if (c == '\b' || c == 127) {
            if (idx > 0) {
                idx--;
                uart_print("\b \b");
            }
        } else if ((c >= '0' && c <= '9') || c == '.') {
            buf[idx++] = c;
            uart_transmit(c);
        }
    }
    buf[idx] = '\0';

    if (idx > 0) {
        float val = (float)atof(buf);
        validar_y_aplicar_sp(val);
    } else {
        uart_println("[Cancelado] Entrada vacia.");
    }
}

void mostrar_menu(void) {
    uart_println("\r\n================ MENU INTERACTIVO ================");
    uart_println(" 1 - Consultar estado y umbrales");
    uart_println(" + - Aumentar Punto Medio (+1.0 C)");
    uart_println(" - - Disminuir Punto Medio (-1.0 C)");
    uart_println(" 2 - Ingresar nuevo Punto Medio");
    uart_println(" 3 - Restaurar Punto Medio base (20.5 C)");
    uart_println(" 4 - Medir temperatura ahora");
    uart_println("==================================================");
    uart_print("Opcion: ");
}

void mostrar_estado(void) {
    uart_println("\r\n------------- ESTADO DEL SISTEMA -------------");
    uart_print("Temperatura Actual:  ");
    uart_print_float(g_temp_actual, 1);
    uart_println(" C");

    uart_print("Punto Medio (SP):    ");
    uart_print_float(g_umbrales.punto_medio, 1);
    uart_println(" C");

    uart_println("--- Umbrales Recalibrados ---");
    uart_print("  Calefactor ON:     <= ");
    uart_print_float(g_umbrales.umbral_cal, 1);
    uart_println(" C");

    uart_print("  Zona Confort:      [");
    uart_print_int((int16_t)g_umbrales.umbral_conf_min);
    uart_print(" a ");
    uart_print_int((int16_t)g_umbrales.umbral_conf_max);
    uart_println("] C");

    uart_print("  Vent. Baja (35%):  [");
    uart_print_int((int16_t)g_umbrales.umbral_conf_max);
    uart_print(" a ");
    uart_print_int((int16_t)g_umbrales.umbral_baja_max);
    uart_println("] C");

    uart_print("  Vent. Media (70%): [");
    uart_print_int((int16_t)g_umbrales.umbral_baja_max);
    uart_print(" a ");
    uart_print_int((int16_t)g_umbrales.umbral_med_max);
    uart_println("] C");

    uart_print("  Vent. Alta (100%): > ");
    uart_print_int((int16_t)g_umbrales.umbral_med_max);
    uart_println(" C");
    uart_println("----------------------------------------------\r\n");
}

/* =========================================================================
 * MAIN
 * ========================================================================= */
int main(void) {
    MCUSR = 0;
    wdt_disable();
    cli();

    TIMSK0 = 0;
    TIMSK2 = 0;

    actuadores_init();
    uart_init(9600);
    lcd_init();
    timer1_init_1hz();

    recalcular_umbrales(PUNTO_MEDIO_DEFAULT);
    sei();

    uart_println("\r\n========================================");
    uart_println("   CONTROL DE TEMPERATURA INTELIGENTE   ");
    uart_println("========================================");
    uart_println("Presione 'm' para ver el menu interactivo.");
    uart_println("Medicion cada 5 segundos iniciada...\r\n");

    lcd_gotoxy(0, 0);
    lcd_print("SISTEMA LISTO   ");
    lcd_gotoxy(0, 1);
    lcd_print("DHT11 Activo    ");
    _delay_ms(1000);
    lcd_clear();

    while (1) {
        if (g_flag_medir) {
            g_flag_medir = 0;
            g_temp_actual = leer_temperatura();
            ejecutar_control(g_temp_actual);
            actualizar_lcd(g_temp_actual, g_estado);
            enviar_telemetria(g_temp_actual, g_estado);
        }

        if (uart_available()) {
            char cmd = uart_receive();

            switch (cmd) {
                case 'm':
                case 'M':
                case '?':
                    mostrar_menu();
                    break;

                case '1':
                    mostrar_estado();
                    break;

                case '+':
                    validar_y_aplicar_sp(g_umbrales.punto_medio + 1.0f);
                    break;

                case '-':
                    validar_y_aplicar_sp(g_umbrales.punto_medio - 1.0f);
                    break;

                case '2':
                    leer_nuevo_punto_medio_seguro();
                    break;

                case '3':
                    recalcular_umbrales(PUNTO_MEDIO_DEFAULT);
                    uart_println("\r\n[OK] Punto medio restablecido a 20.5 C.\r\n");
                    g_flag_medir = 1;
                    break;

                case '4':
                    uart_println("\r\n[INFO] Forzando medicion inmediata...");
                    g_segundos = 0;
                    g_flag_medir = 1;
                    break;

                case '\r':
                case '\n':
                    break;

                default:
                    uart_print("Comando '");
                    uart_transmit(cmd);
                    uart_println("' desconocido. Presione 'm' para ver el menu.");
                    break;
            }
        }
    }

    return 0;
}
