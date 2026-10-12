#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

/* ========================================================================== */
/*                         CONFIGURACION Y CONSTANTES                         */
/* ========================================================================== */
#define BAUD_RATE           9600UL
#define UBRR_VAL            ((F_CPU / (16UL * BAUD_RATE)) - 1)

// Definicion de pines del puente H L298
#define MOTOR_DDR           DDRD
#define MOTOR_PORT          PORTD
#define MOTOR_IN1_PIN       PD3    
#define MOTOR_IN2_PIN       PD4    
#define MOTOR_ENA_PIN       PD5    

// Canales del ADC
#define ADC_CH_LDR          0     
#define ADC_CH_POT          1      

// Parametros del algoritmo de control
#define DEAD_ZONE           20      
#define PWM_MIN             70 
#define PWM_MAX             255  

// Limites mecanicos seguros de la persiana
#define POS_MIN_LIMIT       50     
#define POS_MAX_LIMIT       970  

#define RX_BUFFER_SIZE      16

/* Estados de sentido de giro del motor */
typedef enum {
    MOTOR_STOP = 0,
    MOTOR_CW,       // Horario: Abrir persiana (incrementar luz)
    MOTOR_CCW       // Antihorario: Cerrar persiana (reducir luz)
} motor_state_t;

/* ========================================================================== */
/*                           VARIABLES GLOBALES                               */
/* ========================================================================== */
volatile uint16_t setpoint_light = 512;
volatile bool     setpoint_updated = false;

static volatile char    rx_buffer[RX_BUFFER_SIZE];
static volatile uint8_t rx_index = 0;

static uint16_t      current_ldr = 0;
static uint16_t      current_pos = 0; 
static uint8_t       applied_pwm = 0; 
static motor_state_t motor_dir   = MOTOR_STOP;

/* ========================================================================== */
/*                             MODULO UART (SERIE)                            */
/* ========================================================================== */

void UART_Init(void) {
    UBRR0H = (uint8_t)(UBRR_VAL >> 8);
    UBRR0L = (uint8_t)(UBRR_VAL & 0xFF);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void UART_TxChar(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}
void UART_Print(const char *str) {
    while (*str) {
        UART_TxChar(*str++);
    }
}
ISR(USART_RX_vect) {
    char c = UDR0;
    UART_TxChar(c);
    if (c == '+') {
        if (setpoint_light <= 973) setpoint_light += 50;
        else setpoint_light = 1023;
        setpoint_updated = true;
        return;
    }
    if (c == '-') {
        if (setpoint_light >= 50) setpoint_light -= 50;
        else setpoint_light = 0;
        setpoint_updated = true;
        return;
    }

    if (c == '\r' || c == '\n') {
        if (rx_index > 0) {
            rx_buffer[rx_index] = '\0';
            int val = atoi((const char *)rx_buffer);
            if (val >= 0 && val <= 1023) {
                setpoint_light = (uint16_t)val;
                setpoint_updated = true;
            }
            rx_index = 0;
        }
    } else if (rx_index < (RX_BUFFER_SIZE - 1)) {
        if (c >= '0' && c <= '9') {
            rx_buffer[rx_index++] = c;
        }
    } else {
        rx_index = 0;
    }
}

/* ========================================================================== */
/*                             DRIVER ADC (10 BITS)                           */
/* ========================================================================== */

void ADC_Init(void) {
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    DIDR0 = (1 << ADC0D) | (1 << ADC1D);
}

uint16_t ADC_Read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x07);
    _delay_us(15);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

/* ========================================================================== */
/*                DRIVER TIMER0 (FAST PWM EN OC0B / PD5) Y MOTOR              */
/* ========================================================================== */

void PWM_Init(void) {
    MOTOR_DDR |= (1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN) | (1 << MOTOR_ENA_PIN);
    MOTOR_PORT &= ~((1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN) | (1 << MOTOR_ENA_PIN));
    TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0B = 0;
}

void Motor_Set(motor_state_t dir, uint8_t pwm) {
    switch (dir) {
        case MOTOR_CW:
            MOTOR_PORT |= (1 << MOTOR_IN1_PIN);
            MOTOR_PORT &= ~(1 << MOTOR_IN2_PIN);
            OCR0B = pwm;
            break;

        case MOTOR_CCW:
            MOTOR_PORT &= ~(1 << MOTOR_IN1_PIN);
            MOTOR_PORT |= (1 << MOTOR_IN2_PIN);
            OCR0B = pwm;
            break;

        case MOTOR_STOP:
        default:
            MOTOR_PORT &= ~((1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN));
            OCR0B = 0;
            pwm = 0;
            break;
    }
    motor_dir = dir;
    applied_pwm = pwm;
}

/* ========================================================================== */
/*                 ALGORITMO DE CONTROL (FLUJOGRAMA GUIA)                     */
/* ========================================================================== */

void Control_Loop(void) {
    current_ldr = ADC_Read(ADC_CH_LDR);
    current_pos = ADC_Read(ADC_CH_POT);
    int16_t error = (int16_t)setpoint_light - (int16_t)current_ldr;
    int16_t abs_err = abs(error);
    if (abs_err <= DEAD_ZONE) {
        Motor_Set(MOTOR_STOP, 0);
        return;
    }
    uint32_t calc_pwm = PWM_MIN + ((uint32_t)(abs_err - DEAD_ZONE) * (PWM_MAX - PWM_MIN)) / (1023 - DEAD_ZONE);
    if (calc_pwm > PWM_MAX) {
        calc_pwm = PWM_MAX;
    }
    if (error > 0) {

        if (current_pos < POS_MAX_LIMIT) {
            Motor_Set(MOTOR_CW, (uint8_t)calc_pwm);
        } else {
            Motor_Set(MOTOR_STOP, 0);
        }
    } else {
        if (current_pos > POS_MIN_LIMIT) {
            Motor_Set(MOTOR_CCW, (uint8_t)calc_pwm);
        } else {
            Motor_Set(MOTOR_STOP, 0);
        }
    }
}

void Send_Telemetry(void) {
    char buffer[100];
    const char *dir_text = "DETENIDO";
    if (motor_dir == MOTOR_CW)  dir_text = "HORARIO";
    if (motor_dir == MOTOR_CCW) dir_text = "ANTIHORARIO";

    snprintf(buffer, sizeof(buffer),
             "SET:%4u | LDR:%4u | POS:%4u | PWM:%3u | DIR:%-11s\r\n",
             setpoint_light, current_ldr, current_pos, applied_pwm, dir_text);

    UART_Print(buffer);
}

/* ========================================================================== */
/*                             PROGRAMA PRINCIPAL                             */
/* ========================================================================== */

int main(void) {
    UART_Init();
    ADC_Init();
    PWM_Init();
    sei();

    // Mensaje inicial en Virtual Terminal
    UART_Print("\r\n========================================================\r\n");
    UART_Print(" Comandos:\r\n");
    UART_Print("  - Escriba un numero (0 a 1023) + ENTER para cambiar Setpoint\r\n");
    UART_Print("  - Presione '+' para sumar 50 al Setpoint\r\n");
    UART_Print("  - Presione '-' para restar 50 al Setpoint\r\n");
    UART_Print("========================================================\r\n");
    uint8_t telemetry_counter = 0;

    while (1) {
        Control_Loop();
        if (++telemetry_counter >= 5) {
            telemetry_counter = 0;
            Send_Telemetry();
        }
        if (setpoint_updated) {
            setpoint_updated = false;
            char msg[50];
            snprintf(msg, sizeof(msg), ">> [INFO] Setpoint actualizado a: %u\r\n", setpoint_light);
            UART_Print(msg);
        }

        _delay_ms(40);
    }

    return 0;
}
