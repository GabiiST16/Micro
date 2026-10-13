#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>

#define BAUD 9600
#define UBR 103

#define SERVO_MIN 125
#define SERVO_MAX 600

#define LED_PIN PD6
#define NUM_LEDS_TIRA 64 // Matriz 8x8 (64 LEDs para simulacion)

uint8_t g_lcd_addr = 0x27;
#define LCD_RS 0x01
#define LCD_EN 0x04
#define LCD_BACKLIGHT 0x08

typedef enum
{
    ROJO = 0,
    AZUL,
    VERDE,
    VIOLETA,
    ROSADO,
    NINGUNO,
    TOTAL_COLORES
} color_detectado;

typedef struct
{
    const char *nombre;
    uint8_t angulo;
    uint8_t r_ref, g_ref, b_ref;
    uint8_t led_r, led_g, led_b;
} PerfilColor;

const PerfilColor perfiles[TOTAL_COLORES] = {
    [ROJO]    = {"ROJO",      0,  97,  87,  67, 255,   0,   0},
    [AZUL]    = {"AZUL",     45,  60,  70, 185,   0,   0, 255},
    [VERDE]   = {"VERDE",   135,  65, 180,  65,   0, 255,   0},
    [VIOLETA] = {"VIOLETA",  90, 140,  60, 140, 128,   0, 255},
    [ROSADO]  = {"ROSADO",  180, 170, 100, 120, 255, 105, 180},
    [NINGUNO] = {"NINGUNO",  90,   0,   0,   0,   0,   0,   0}
};

void USART_Init(unsigned int ubrr);
void USART_Transmit(unsigned char data);
void USART_SendString(const char *str);

void ADC_Init(void);
uint8_t ADC_Read(uint8_t channel);
uint16_t ADC_ReadAvg(uint8_t channel, uint8_t samples);

color_detectado clasificarColorReal(uint16_t r, uint16_t g, uint16_t b);

void servo_timer1_init(void);
void servo_set_angle(uint8_t angle);

void ws_send_byte(uint8_t b);
void encender_tira_color(uint8_t r, uint8_t g, uint8_t b);

void I2C_Init(void);
uint8_t I2C_Ping(uint8_t addr);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Write(uint8_t data);

void LCD_I2C_WriteExpander(uint8_t data);
void LCD_PulseEnable(uint8_t data);
void LCD_Send4Bits(uint8_t value);
void LCD_Command(uint8_t cmd);
void LCD_Char(uint8_t data);
void LCD_Init(void);
void LCD_String(const char *str);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Clear(void);

int main(void)
{
    DDRD |= (1 << LED_PIN);

    USART_Init(UBR);
    ADC_Init();
    servo_timer1_init();
    LCD_Init();

    USART_SendString("\r\n========================================\r\n");
    USART_SendString(" Sistema Clasificador (Modo Simulacion)\r\n");
    USART_SendString("========================================\r\n");
    
    char dbg_buf[50];
    sprintf(dbg_buf, "Estado LCD I2C: Dir 0x%02X\r\n\r\n", g_lcd_addr);
    USART_SendString(dbg_buf);

    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_String("Clasificador");
    LCD_SetCursor(1, 0);
    LCD_String("Simulacion");
    _delay_ms(1500);

    uint16_t valor_R, valor_G, valor_B;
    int16_t dif_R = 0, dif_G = 0, dif_B = 0;
    color_detectado color;
    char serial_buf[70];
    char lcd_line[17];

    while (1)
    {
        // =========================================================================
        // ADAPTACION PARA SIMULADOR (PICSimLab / Proteus):
        // En simulacion leemos tres potenciometros independientes (A0, A1, A2)
        // porque no existe acoplamiento optico entre el LED virtual y el LDR.
        // =========================================================================
        encender_tira_color(255, 0, 0);
        _delay_ms(120);
        valor_R = ADC_ReadAvg(0, 16); // Potenciometro Rojo en canal A0

        encender_tira_color(0, 255, 0);
        _delay_ms(120);
        valor_G = ADC_ReadAvg(1, 16); // Potenciometro Verde en canal A1

        encender_tira_color(0, 0, 255);
        _delay_ms(120);
        valor_B = ADC_ReadAvg(2, 16); // Potenciometro Azul en canal A2

        // Clasificar color segun el algoritmo de dominancia
        color = clasificarColorReal(valor_R, valor_G, valor_B);
        const PerfilColor *p = &perfiles[color];

        // Mover servo al angulo de la bandeja y encender tira con color clasificado
        servo_set_angle(p->angulo);
        encender_tira_color(p->led_r, p->led_g, p->led_b);

        if (color != NINGUNO)
        {
            dif_R = (int16_t)valor_R - (int16_t)p->r_ref;
            dif_G = (int16_t)valor_G - (int16_t)p->g_ref;
            dif_B = (int16_t)valor_B - (int16_t)p->b_ref;
        }
        else
        {
            dif_R = dif_G = dif_B = 0;
        }

        // Reporte por UART
        USART_SendString("---------- MEDICION RGB ----------\r\n");
        sprintf(serial_buf, "Lectura ADC : R:%3u  G:%3u  B:%3u\r\n", valor_R, valor_G, valor_B);
        USART_SendString(serial_buf);

        USART_SendString("Color Detect: ");
        USART_SendString(p->nombre);
        USART_SendString("\r\n");

        sprintf(serial_buf, "Angulo Servo: %u deg\r\n", p->angulo);
        USART_SendString(serial_buf);

        if (color != NINGUNO)
        {
            sprintf(serial_buf, "Patron Ref  : R:%3u  G:%3u  B:%3u\r\n", p->r_ref, p->g_ref, p->b_ref);
            USART_SendString(serial_buf);
            sprintf(serial_buf, "Diferencias : R:%+3d  G:%+3d  B:%+3d\r\n", dif_R, dif_G, dif_B);
            USART_SendString(serial_buf);
        }
        else
        {
            USART_SendString("Patron Ref  : N/A\r\n");
            USART_SendString("Diferencias : N/A\r\n");
        }
        USART_SendString("----------------------------------\r\n\r\n");

        // Pantalla 1: Color detectado, Angulo y Lecturas
        LCD_Clear();
        LCD_SetCursor(0, 0);
        sprintf(lcd_line, "%-8s A:%3u", p->nombre, p->angulo);
        LCD_String(lcd_line);

        LCD_SetCursor(1, 0);
        sprintf(lcd_line, "L:R%03u G%03u B%03u", valor_R, valor_G, valor_B);
        LCD_String(lcd_line);
        _delay_ms(2000);

        // Pantalla 2: Patron de referencia y Diferencias
        LCD_Clear();
        if (color != NINGUNO)
        {
            LCD_SetCursor(0, 0);
            sprintf(lcd_line, "P:R%03u G%03u B%03u", p->r_ref, p->g_ref, p->b_ref);
            LCD_String(lcd_line);

            LCD_SetCursor(1, 0);
            sprintf(lcd_line, "D:%+3d %+3d %+3d", dif_R, dif_G, dif_B);
            LCD_String(lcd_line);
        }
        else
        {
            LCD_SetCursor(0, 0);
            LCD_String("Sin Referencia");
            LCD_SetCursor(1, 0);
            LCD_String("Fuera de Rango");
        }
        _delay_ms(2000);
    }
}

color_detectado clasificarColorReal(uint16_t r, uint16_t g, uint16_t b)
{
    if (r < 18 && g < 18 && b < 18) return NINGUNO;
    if (r > 235 && g > 235 && b > 235) return NINGUNO;
		
    if ((r >= g + 4) && (r >= b + 15))
    {
        return ROJO;
    }

    if ((g >= r + 6) && (g >= b + 6))
    {
        return VERDE;
    }

    if ((b >= r + 6) && (b >= g + 4))
    {
        return AZUL;
    }

    if ((r >= g + 6) && (b >= g + 6))
    {
        return VIOLETA;
    }

    if ((r >= g + 10) && (r > b) && (b > g))
    {
        return ROSADO;
    }

    return NINGUNO;
}

void servo_timer1_init(void)
{
    DDRB |= (1 << PB1);
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10);
    ICR1 = 4999;
}

void servo_set_angle(uint8_t angle)
{
    if (angle > 180) angle = 180;
    uint16_t pulse = SERVO_MIN + ((uint32_t)angle * (SERVO_MAX - SERVO_MIN)) / 180;
    OCR1A = pulse;
}

void ws_send_byte(uint8_t b)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (b & 0x80)
        {
            PORTD |= (1 << LED_PIN);
            asm volatile("nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t");
            PORTD &= ~(1 << LED_PIN);
            asm volatile("nop\n\t nop\n\t nop\n\t nop\n\t");
        }
        else
        {
            PORTD |= (1 << LED_PIN);
            asm volatile("nop\n\t nop\n\t");
            PORTD &= ~(1 << LED_PIN);
            asm volatile("nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t");
        }
        b <<= 1;
    }
}

void encender_tira_color(uint8_t r, uint8_t g, uint8_t b)
{
    cli();
    for (uint8_t i = 0; i < NUM_LEDS_TIRA; i++)
    {
        ws_send_byte(g); // Verde primero (estándar GRB de WS2812)
        ws_send_byte(r); // Rojo
        ws_send_byte(b); // Azul
    }
    sei();
    _delay_us(60);
}

void ADC_Init(void)
{
    ADMUX = (1 << REFS0) | (1 << ADLAR);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint8_t ADC_Read(uint8_t channel)
{
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC))
        ;
    return ADCH;
}

uint16_t ADC_ReadAvg(uint8_t channel, uint8_t samples)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; i++)
    {
        sum += ADC_Read(channel);
        _delay_us(50);
    }
    return (uint16_t)(sum / samples);
}

void USART_Init(unsigned int ubrr)
{
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void USART_Transmit(unsigned char data)
{
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = data;
}

void USART_SendString(const char *str)
{
    while (*str)
        USART_Transmit(*str++);
}

void I2C_Init(void)
{
    TWSR = 0x00;
    TWBR = 72;
}

void I2C_Start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)))
        ;
}

void I2C_Stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    while (TWCR & (1 << TWSTO))
        ;
    _delay_us(100);
}

void I2C_Write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)))
        ;
}

uint8_t I2C_Ping(uint8_t addr)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    uint16_t to = 10000;
    while (!(TWCR & (1 << TWINT)) && --to);
    if (!to) return 0;

    TWDR = (addr << 1);
    TWCR = (1 << TWINT) | (1 << TWEN);
    to = 10000;
    while (!(TWCR & (1 << TWINT)) && --to);
    if (!to) return 0;

    uint8_t ack = ((TWSR & 0xF8) == 0x18);
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    while (TWCR & (1 << TWSTO))
        ;
    _delay_us(100);
    return ack;
}

void LCD_I2C_WriteExpander(uint8_t data)
{
    I2C_Start();
    I2C_Write((g_lcd_addr << 1));
    I2C_Write(data | LCD_BACKLIGHT);
    I2C_Stop();
}

void LCD_PulseEnable(uint8_t data)
{
    LCD_I2C_WriteExpander(data | LCD_EN);
    _delay_ms(1);
    LCD_I2C_WriteExpander(data & ~LCD_EN);
    _delay_ms(1);
}

void LCD_Send4Bits(uint8_t value)
{
    LCD_I2C_WriteExpander(value);
    LCD_PulseEnable(value);
}

void LCD_Command(uint8_t cmd)
{
    uint8_t high = cmd & 0xF0;
    uint8_t low  = (cmd << 4) & 0xF0;
    LCD_Send4Bits(high);
    LCD_Send4Bits(low);
    _delay_ms(4);
}

void LCD_Char(uint8_t data)
{
    uint8_t high = data & 0xF0;
    uint8_t low  = (data << 4) & 0xF0;
    LCD_Send4Bits(high | LCD_RS);
    LCD_Send4Bits(low | LCD_RS);
    _delay_ms(2);
}

void LCD_Init(void)
{
    I2C_Init();
    _delay_ms(100);

    // Auto-deteccion de direccion I2C 
    for (uint8_t a = 0x20; a <= 0x3F; a++)
    {
        if (I2C_Ping(a))
        {
            g_lcd_addr = a;
            break;
        }
    }

    LCD_Send4Bits(0x30);
    _delay_ms(10);
    LCD_Send4Bits(0x30);
    _delay_ms(5);
    LCD_Send4Bits(0x30);
    _delay_ms(5);
    LCD_Send4Bits(0x20);
    _delay_ms(5);

    LCD_Command(0x28);
    _delay_ms(2);
    LCD_Command(0x08);
    _delay_ms(2);
    LCD_Command(0x01);
    _delay_ms(10);
    LCD_Command(0x06);
    _delay_ms(2);
    LCD_Command(0x0C);
    _delay_ms(5);
}

void LCD_String(const char *str)
{
    while (*str)
        LCD_Char(*str++);
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t address = col;
    if (row == 1) address += 0x40;
    LCD_Command(0x80 | address);
}

void LCD_Clear(void)
{
    LCD_Command(0x01);
    _delay_ms(5);
}
