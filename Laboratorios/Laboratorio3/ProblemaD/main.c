#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#define PCF8574_ADDR 0x27
#define SIL  0
#define C3   131
#define D3   147
#define DS3  156
#define EB3  156
#define E3   165
#define F3   175
#define FS3  185
#define G3   196
#define GS3  208
#define AB3  208
#define A3   220
#define AS3  233
#define BB3  233
#define B3   247
#define C4   262
#define CS4  277
#define D4   294
#define DS4  311
#define EB4  311
#define E4   330
#define F4   349
#define FS4  370
#define G4   392
#define GS4  415
#define AB4  415
#define A4   440
#define AS4  466
#define BB4  466 
#define B4   494
#define C5   523
#define CS5  554
#define D5   587
#define DS5  622
#define EB5  622
#define E5   659
#define F5   698
#define FS5  740
#define G5   784
#define A5   880

#define FREQ_TO_ICR1(f) ((f > 0) ? ((F_CPU / (8UL * (f))) - 1) : 0)

typedef enum {
    ESTADO_PIANO,
    ESTADO_CANCION
} EstadoSistema;

volatile EstadoSistema estadoActual = ESTADO_PIANO;
volatile char comandoUART = 0;
volatile uint8_t nuevoComando = 0;

void UART_Init(unsigned int baud) {
    unsigned int ubrr = F_CPU / 16 / baud - 1;
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void UART_TxChar(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void UART_SendString(const char *str) {
    while (*str) {
        UART_TxChar(*str++);
    }
}

void UART_MostrarMenu(void) {
    UART_SendString("\r\n==================================================\r\n");
    UART_SendString("    PIANO ELECTRONICO POLIFONICO - ATmega328P     \r\n");
    UART_SendString("        Tecnologias de Microprocesamiento         \r\n");
    UART_SendString("==================================================\r\n");
    UART_SendString(" Comandos Disponibles por UART:\r\n");
    UART_SendString("  [1] Cancion 1: Despacito (Luis Fonsi)\r\n");
    UART_SendString("  [2] Cancion 2: The Imperial March (Star Wars)\r\n");
    UART_SendString("  [3] Cancion 3: Married Life\r\n");
    UART_SendString("  [S] o [0] Detener cancion y volver a Piano\r\n");
    UART_SendString("  [M] Mostrar este menu interactivo\r\n");
    UART_SendString("==================================================\r\n\r\n");
}

ISR(USART_RX_vect) {
    comandoUART = UDR0;
    nuevoComando = 1;
}

void I2C_Init(void) {
    TWSR = 0x00;
    TWBR = 0x48; // SCL = 100 kHz a 16 MHz
    TWCR = (1 << TWEN);
}

void I2C_Start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void I2C_Stop(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    _delay_us(100);
}

void I2C_Write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

#define LCD_BACKLIGHT 0x08

void LCD_I2C_WriteNibble(uint8_t nibble, uint8_t mode) {
    uint8_t data = (nibble & 0xF0) | mode | LCD_BACKLIGHT;
    I2C_Start();
    I2C_Write(PCF8574_ADDR << 1);
    I2C_Write(data | 0x04);
    _delay_us(1);
    I2C_Write(data & ~0x04);
    _delay_us(50);
    I2C_Stop();
}

void LCD_I2C_Cmd(uint8_t cmd) {
    LCD_I2C_WriteNibble(cmd & 0xF0, 0);
    LCD_I2C_WriteNibble((cmd << 4) & 0xF0, 0);
    if (cmd == 0x01 || cmd == 0x02) _delay_ms(2);
}

void LCD_I2C_Char(uint8_t data) {
    LCD_I2C_WriteNibble(data & 0xF0, 1);
    LCD_I2C_WriteNibble((data << 4) & 0xF0, 1);
}

void LCD_I2C_Init(void) {
    _delay_ms(50);
    LCD_I2C_WriteNibble(0x30, 0);
    _delay_ms(5);
    LCD_I2C_WriteNibble(0x30, 0);
    _delay_us(200);
    LCD_I2C_WriteNibble(0x30, 0);
    LCD_I2C_WriteNibble(0x20, 0);

    LCD_I2C_Cmd(0x28);
    LCD_I2C_Cmd(0x0C);
    LCD_I2C_Cmd(0x06);
    LCD_I2C_Cmd(0x01);
    _delay_ms(2);
}

void LCD_I2C_SetCursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? (0x80 + col) : (0xC0 + col);
    LCD_I2C_Cmd(addr);
}

void LCD_I2C_String(const char *str) {
    while (*str) LCD_I2C_Char((uint8_t)*str++);
}

void PWM_Init(void) {
 
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    DDRB |= (1 << PB1);
    PORTB &= ~(1 << PB1);

    TCCR2A = (1 << WGM21);
    TCCR2B = (1 << CS22) | (1 << CS21);
    DDRB |= (1 << PB3);
    PORTB &= ~(1 << PB3);
}

void Set_Tone_Buzzer1(uint16_t freq) {
    if (freq == 0) {
        TCCR1A &= ~(1 << COM1A1);
        PORTB &= ~(1 << PB1);
    } else {
        TCCR1A |= (1 << COM1A1);
        ICR1 = FREQ_TO_ICR1(freq);
        OCR1A = ICR1 / 2;
    }
}

void Set_Tone_Buzzer2(uint16_t freq) {
    if (freq == 0) {
        TCCR2A &= ~(1 << COM2A0);
        PORTB &= ~(1 << PB3);
    } else {
        TCCR2A |= (1 << COM2A0);
        uint32_t val = (31250UL / freq) - 1;
        if (val > 255) val = 255;
        OCR2A = (uint8_t)val;
    }
}

uint8_t Leer_Pulsadores(uint16_t *freq_out, char *nota_out) {
    if (!(PINB & (1 << PB0))) { *freq_out = C4; sprintf(nota_out, "DO (C4)"); return 1; }
    if (!(PINB & (1 << PB2))) { *freq_out = D4; sprintf(nota_out, "RE (D4)"); return 1; }
    if (!(PINB & (1 << PB4))) { *freq_out = E4; sprintf(nota_out, "MI (E4)"); return 1; }
    if (!(PINB & (1 << PB5))) { *freq_out = F4; sprintf(nota_out, "FA (F4)"); return 1; }
    if (!(PINC & (1 << PC0))) { *freq_out = G4; sprintf(nota_out, "SOL(G4)"); return 1; }
    if (!(PINC & (1 << PC1))) { *freq_out = A4; sprintf(nota_out, "LA (A4)"); return 1; }
    if (!(PINC & (1 << PC2))) { *freq_out = B4; sprintf(nota_out, "SI (B4)"); return 1; }
    if (!(PINC & (1 << PC3))) { freq_out = C5; sprintf(nota_out, "DO(C5)"); return 1; }
    
    *freq_out = 0;
    return 0;
}

const uint16_t mel_c1[] = {
    D5, CS5, B4, FS4, SIL,
    FS4, FS4, FS4, FS4, B4, B4, B4, B4, B4, A4, B4, G4, SIL,
    G4, G4, G4, G4, B4, B4, B4, B4, B4, CS5, D5, A4, SIL,
    A4, A4, A4, A4, D5, D5, D5, D5, D5, E5, E5, CS5, SIL,
    D5, CS5, B4, SIL
};

const uint16_t bajo_c1[] = {
    B3, B3, B3, B3, SIL,
    G3, G3, G3, G3, G3, G3, G3, G3, G3, G3, G3, G3, SIL,
    D3, D3, D3, D3, D3, D3, D3, D3, D3, D3, D3, D3, SIL,
    A3, A3, A3, A3, A3, A3, A3, A3, A3, A3, A3, A3, SIL,
    B3, B3, B3, SIL
};

const uint16_t dur_c1[] = {
    500, 300, 300, 700, 150,
    200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 250, 600, 150,
    200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 250, 600, 150,
    200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 250, 600, 150,
    450, 300, 800, 300
};
const uint8_t len_c1 = sizeof(mel_c1) / sizeof(mel_c1[0]);

const uint16_t mel_c2[] = {
    G4, G4, G4, DS4, AS4, G4, DS4, AS4, G4, SIL,
    D5, D5, D5, DS5, AS4, FS4, DS4, AS4, G4, SIL
};

const uint16_t bajo_c2[] = {
    G3, G3, G3, G3,  D3,  G3, G3,  D3,  G3, SIL,
    D3, D3, D3, DS3, AS3, D3, DS3, AS3, G3, SIL
};

const uint16_t dur_c2[] = {
    500, 500, 500, 350, 150, 500, 350, 150, 800, 150,
    500, 500, 500, 350, 150, 500, 350, 150, 800, 300
};
const uint8_t len_c2 = sizeof(mel_c2) / sizeof(mel_c2[0]);

const uint16_t mel_c3[] = {
    
    SIL, SIL, SIL,
    SIL, SIL, SIL,
    SIL, SIL, SIL,
    SIL, SIL, C5, D5,
    F5, F5, F5,
    F5, E5, D5,
    C5, A4, F4,
    G4, G4, G4,
    G4, A4, BB4,
    D5, C5, C5,
    A4, A4, A4,
    A4, G4, F4,
    G4, G4, G4,
    G4, A4, BB4,
    C5, BB4, A4,
    G4, F4, F4,
    F4, SIL
};

const uint16_t bajo_c3[] = {
    F3, A3, C4,
    C3, A3, C4,
    F3, A3, C4,
    C3, A3, C4, C4,
    F3, A3, C4,
    C3, A3, C4,
    F3, A3, C4,
    C3, G3, BB3,
    C3, G3, BB3,
    F3, A3, C4,
    D3, F3, A3,
    D3, F3, A3,
    G3, BB3, D4,
    C3, E3, G3,
    C3, G3, BB3,
    C3, F3, A3,
    F3, SIL
};

const uint16_t dur_c3[] = {
    380, 380, 380,
    380, 380, 380,
    380, 380, 380,
    380, 380, 190, 190,

    380, 380, 380,
    380, 380, 380,
    380, 380, 380,
    380, 380, 380,
    380, 380, 380,
    380, 380, 380,
    380, 380, 380,
    380, 380, 380,
    380, 380, 380,
    380, 380, 380,
    380, 380, 380,
    380, 380, 380,
    1000, 300
};
const uint8_t len_c3 = sizeof(mel_c3) / sizeof(mel_c3[0]);

void Reproducir_Cancion(const uint16_t *mel, const uint16_t *bajo, const uint16_t *dur, uint8_t len, const char *nombre) {
    LCD_I2C_Cmd(0x01);
    LCD_I2C_SetCursor(0, 0);
    LCD_I2C_String("Reproduciendo:");
    LCD_I2C_SetCursor(1, 0);
    LCD_I2C_String(nombre);

    UART_SendString("\r\n[REPRODUCCIÓN] ");
    UART_SendString(nombre);
    UART_SendString("\r\n");

    for (uint8_t i = 0; i < len; i++) {
        if (nuevoComando) {
            if (comandoUART == 'S' || comandoUART == 's' || 
                comandoUART == '0' || comandoUART == 'P' || comandoUART == 'p') {
                nuevoComando = 0;
                UART_SendString("[UART] Detencion manual solicitada.\r\n");
                break;
            }
        }

        Set_Tone_Buzzer1(mel[i]);
        Set_Tone_Buzzer2(bajo[i]);

        for (uint16_t t = 0; t < dur[i]; t += 10) {
            _delay_ms(10);
            if (nuevoComando && (comandoUART == 'S' || comandoUART == 's' || 
                                 comandoUART == '0' || comandoUART == 'P' || comandoUART == 'p')) {
                break;
            }
        }

        Set_Tone_Buzzer1(0);
        Set_Tone_Buzzer2(0);
        _delay_ms(25);
    }

    Set_Tone_Buzzer1(0);
    Set_Tone_Buzzer2(0);
    estadoActual = ESTADO_PIANO;

    LCD_I2C_Cmd(0x01);
    LCD_I2C_SetCursor(0, 0);
    LCD_I2C_String("Modo: Piano");

    UART_SendString("[FSM] Cancion finalizada. Retorno a MODO PIANO.\r\n");
}

int main(void) {
    DDRB &= ~((1 << PB0) | (1 << PB2) | (1 << PB4) | (1 << PB5));
    PORTB |= (1 << PB0) | (1 << PB2) | (1 << PB4) | (1 << PB5);

    DDRC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3));
    PORTC |= (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3);

    I2C_Init();
    LCD_I2C_Init();
    PWM_Init();
    UART_Init(9600);
    sei();

    LCD_I2C_SetCursor(0, 0);
    LCD_I2C_String("Modo: Piano");

    UART_MostrarMenu();

    uint16_t frecuencia = 0;
    char nombreNota[12];
    uint8_t notaPrevia = 0;

    while (1) {
        if (nuevoComando) {
            char cmd = comandoUART;
            nuevoComando = 0;

            if (cmd == '1') {
                estadoActual = ESTADO_CANCION;
                Reproducir_Cancion(mel_c1, bajo_c1, dur_c1, len_c1, "1: Despacito");
            } else if (cmd == '2') {
                estadoActual = ESTADO_CANCION;
                Reproducir_Cancion(mel_c2, bajo_c2, dur_c2, len_c2, "2: Imperial M.");
            } else if (cmd == '3') {
                estadoActual = ESTADO_CANCION;
                Reproducir_Cancion(mel_c3, bajo_c3, dur_c3, len_c3, "3: Married Life");
            } else if (cmd == 'S' || cmd == 's' || cmd == '0' || cmd == 'P' || cmd == 'p') {
                Set_Tone_Buzzer1(0);
                Set_Tone_Buzzer2(0);
                estadoActual = ESTADO_PIANO;
                LCD_I2C_Cmd(0x01);
                LCD_I2C_SetCursor(0, 0);
                LCD_I2C_String("Modo: Piano");
                UART_SendString("[UART] En MODO PIANO MANUAL.\r\n");
            } else if (cmd == 'M' || cmd == 'm' || cmd == '?') {
                UART_MostrarMenu();
            }
        }

        if (estadoActual == ESTADO_PIANO) {
            if (Leer_Pulsadores(&frecuencia, nombreNota)) {
                Set_Tone_Buzzer1(frecuencia);
                Set_Tone_Buzzer2(0);
                if (!notaPrevia) {
                    LCD_I2C_SetCursor(1, 0);
                    LCD_I2C_String("Nota: ");
                    LCD_I2C_String(nombreNota);

                    UART_SendString("Nota pulsada: ");
                    UART_SendString(nombreNota);
                    UART_SendString("\r\n");
                    notaPrevia = 1;
                }
            } else {
                Set_Tone_Buzzer1(0);
                Set_Tone_Buzzer2(0);

                if (notaPrevia) {
                    LCD_I2C_SetCursor(1, 0);
                    LCD_I2C_String("                ");
                    notaPrevia = 0;
                }
            }
        }
    }
}