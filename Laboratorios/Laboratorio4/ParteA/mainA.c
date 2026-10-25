#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#define LCD_ADDR            0x4E 
#define EEPROM_FIRMA_VALIDA 0xA6
#define EEPROM_FIRMA_DIR    ((uint8_t*)0x00)
#define NUM_CUENTAS         3

/* Descomentar para mostrar el UID de cada tarjeta leida (util para registrar tarjetas nuevas) */
/* #define DEPURAR_UID */

#define RC522_COMMAND_REG    0x01
#define RC522_COMIEN_REG     0x02
#define RC522_COMIRQ_REG     0x04
#define RC522_ERROR_REG      0x06
#define RC522_FIFODATA_REG   0x09
#define RC522_FIFOLEVEL_REG  0x0A
#define RC522_CONTROL_REG    0x0C
#define RC522_BITFRAMING_REG 0x0D
#define RC522_MODE_REG       0x11
#define RC522_TXCONTROL_REG  0x14
#define RC522_TXASK_REG      0x15
#define RC522_TMODE_REG      0x2A
#define RC522_TPRESCALER_REG 0x2B
#define RC522_TRELOAD_REGL   0x2D
#define RC522_TRELOAD_REGH   0x2C

struct Cuenta {
    uint8_t uid[4];
    char pin[5];
    uint32_t saldo;
};

const struct Cuenta cuentas_iniciales[NUM_CUENTAS] = {
    { {0xF5, 0xF0, 0x2E, 0x07}, "1234", 1000 },
    { {0x03, 0x6A, 0xC9, 0x26}, "4321", 2500 },
    { {0xFA, 0xB0, 0x2D, 0x07}, "9999", 500  }
};

typedef enum {
    ESTADO_ESPERA_TARJETA,
    ESTADO_INGRESO_PIN,
    ESTADO_MENU_PRINCIPAL,
    ESTADO_CONSULTA_SALDO,
    ESTADO_RETIRO,
    ESTADO_DEPOSITO,
    ESTADO_BLOQUEO
} EstadoFSM;

EstadoFSM estado_actual = ESTADO_ESPERA_TARJETA;
struct Cuenta cuenta_activa;
int indice_cuenta = -1;
uint8_t intentos_pin = 0;

/* =========================================================
   COMUNICACION SPI Y CONTROLADOR RC522
   ========================================================= */

void spi_iniciar(void) {
    DDRB |= (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB5);
    DDRB &= ~(1 << PB4);
    PORTB |= (1 << PB4);

    PORTB |= (1 << PB2);
    PORTB |= (1 << PB1);

    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

uint8_t spi_transferir(uint8_t dato) {
    SPDR = dato;
    while (!(SPSR & (1 << SPIF)));
    return SPDR;
}

void rc522_escribir(uint8_t reg, uint8_t valor) {
    PORTB &= ~(1 << PB2);
    spi_transferir((reg << 1) & 0x7E);
    spi_transferir(valor);
    PORTB |= (1 << PB2);
}

uint8_t rc522_leer(uint8_t reg) {
    uint8_t valor;
    PORTB &= ~(1 << PB2);
    spi_transferir(((reg << 1) & 0x7E) | 0x80);
    valor = spi_transferir(0x00);
    PORTB |= (1 << PB2);
    return valor;
}

void rc522_iniciar(void) {
    PORTB &= ~(1 << PB1);
    _delay_ms(10);
    PORTB |= (1 << PB1);
    _delay_ms(10);

    rc522_escribir(RC522_COMMAND_REG, 0x0F);
    _delay_ms(5);

    rc522_escribir(RC522_TMODE_REG, 0x8D);
    rc522_escribir(RC522_TPRESCALER_REG, 0x3E);
    rc522_escribir(RC522_TRELOAD_REGL, 30);
    rc522_escribir(RC522_TRELOAD_REGH, 0);
    rc522_escribir(RC522_TXASK_REG, 0x40);
    rc522_escribir(RC522_MODE_REG, 0x3D);

    uint8_t tx = rc522_leer(RC522_TXCONTROL_REG);
    if ((tx & 0x03) != 0x03) {
        rc522_escribir(RC522_TXCONTROL_REG, tx | 0x03);
    }
}

bool rc522_comunicar(uint8_t cmd, uint8_t *tx_data, uint8_t tx_len, uint8_t *rx_data, uint8_t *rx_len) {
    rc522_escribir(RC522_COMIEN_REG, 0xF7);
    rc522_escribir(RC522_COMIRQ_REG, 0x7F);
    rc522_escribir(RC522_FIFOLEVEL_REG, 0x80);
    rc522_escribir(RC522_COMMAND_REG, 0x00);

    for (uint8_t i = 0; i < tx_len; i++) {
        rc522_escribir(RC522_FIFODATA_REG, tx_data[i]);
    }

    rc522_escribir(RC522_COMMAND_REG, cmd);
    if (cmd == 0x0C) {
        rc522_escribir(RC522_BITFRAMING_REG, rc522_leer(RC522_BITFRAMING_REG) | 0x80);
    }

    uint16_t timeout = 2000;
    uint8_t irq;
    do {
        irq = rc522_leer(RC522_COMIRQ_REG);
        timeout--;
    } while ((timeout != 0) && !(irq & 0x30));

    rc522_escribir(RC522_BITFRAMING_REG, rc522_leer(RC522_BITFRAMING_REG) & (~0x80));

    if (timeout == 0 || (rc522_leer(RC522_ERROR_REG) & 0x1B)) {
        return false;
    }

    if (rx_data && rx_len) {
        uint8_t cant = rc522_leer(RC522_FIFOLEVEL_REG);
        *rx_len = cant;
        for (uint8_t i = 0; i < cant; i++) {
            rx_data[i] = rc522_leer(RC522_FIFODATA_REG);
        }
    }
    return true;
}

bool rc522_detectar_tarjeta(uint8_t *uid_leido) {
    uint8_t cmd_req = 0x26;
    uint8_t buffer[16];
    uint8_t len = 0;
 
    rc522_escribir(RC522_BITFRAMING_REG, 0x07);
    if (!rc522_comunicar(0x0C, &cmd_req, 1, buffer, &len)) {
        return false;
    }

    rc522_escribir(RC522_BITFRAMING_REG, 0x00);
    uint8_t cmd_anti[2] = {0x93, 0x20};
    if (rc522_comunicar(0x0C, cmd_anti, 2, buffer, &len)) {
        for (uint8_t i = 0; i < 4; i++) {
            uid_leido[i] = buffer[i];
        }
        return true;
    }
    return false;
}

/* =========================================================
   COMUNICACION TWI (I2C) Y DISPLAY LCD 16x2
   ========================================================= */

void twi_iniciar(void) {
    TWSR = 0x00;
    TWBR = 72;
    TWCR = (1 << TWEN);
}

void twi_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void twi_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    _delay_us(50);
}

void twi_escribir(uint8_t dato) {
    TWDR = dato;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void lcd_enviar_nibble(uint8_t nibble, uint8_t rs) {
    uint8_t dato = (nibble & 0xF0) | (rs ? 0x01 : 0x00) | 0x08;
    twi_start();
    twi_escribir(LCD_ADDR);
    twi_escribir(dato | 0x04);
    _delay_us(2);
    twi_escribir(dato & ~0x04);
    _delay_us(50);
    twi_stop();
}

void lcd_enviar_byte(uint8_t valor, uint8_t rs) {
    lcd_enviar_nibble(valor & 0xF0, rs);
    lcd_enviar_nibble((valor << 4) & 0xF0, rs);
}

void lcd_comando(uint8_t cmd) {
    lcd_enviar_byte(cmd, 0);
    if (cmd == 0x01 || cmd == 0x02) _delay_ms(2);
}

void lcd_caracter(char c) {
    lcd_enviar_byte((uint8_t)c, 1);
}

void lcd_iniciar(void) {
    twi_iniciar();
    _delay_ms(50);
    lcd_enviar_nibble(0x30, 0);
    _delay_ms(5);
    lcd_enviar_nibble(0x30, 0);
    _delay_us(150);
    lcd_enviar_nibble(0x30, 0);
    _delay_ms(1);
    lcd_enviar_nibble(0x20, 0);

    lcd_comando(0x28);
    lcd_comando(0x0C);
    lcd_comando(0x06);
    lcd_comando(0x01);
    _delay_ms(2);
}

void lcd_limpiar(void) {
    lcd_comando(0x01);
    _delay_ms(2);
}

void lcd_posicion(uint8_t col, uint8_t fila) {
    uint8_t offsets[] = {0x00, 0x40};
    lcd_comando(0x80 | (col + offsets[fila]));
}

void lcd_texto(const char *texto) {
    while (*texto) {
        lcd_caracter(*texto++);
    }
}

/* =========================================================
   TECLADO MATRICIAL 4x4
   ========================================================= */

void teclado_iniciar(void) {
    DDRD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);
    PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);

    DDRD &= ~((1 << PD6) | (1 << PD7));
    DDRB &= ~(1 << PB0);
    DDRC &= ~(1 << PC0);

    PORTD |= (1 << PD6) | (1 << PD7);
    PORTB |= (1 << PB0);
    PORTC |= (1 << PC0);
}

char teclado_leer(void) {
    const char mapa[4][4] = {
        {'1', '2', '3', 'A'},
        {'4', '5', '6', 'B'},
        {'7', '8', '9', 'C'},
        {'*', '0', '#', 'D'}
    };
    const uint8_t pines_fila[4] = {PD2, PD3, PD4, PD5};

    for (uint8_t f = 0; f < 4; f++) {
        PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);
        PORTD &= ~(1 << pines_fila[f]);
        _delay_us(10);

        uint8_t col = 0xFF;
        if (!(PIND & (1 << PD6))) col = 0;
        else if (!(PIND & (1 << PD7))) col = 1;
        else if (!(PINB & (1 << PB0))) col = 2;
        else if (!(PINC & (1 << PC0))) col = 3;

        if (col != 0xFF) {
            _delay_ms(20);
            while (!(PIND & (1 << PD6)) || !(PIND & (1 << PD7)) ||
                   !(PINB & (1 << PB0)) || !(PINC & (1 << PC0)));
            _delay_ms(20);
            return mapa[f][col];
        }
    }
    return '\0';
}

/* =========================================================
   INDICADORES Y GESTION DE EEPROM
   ========================================================= */

void indicadores_iniciar(void) {
    DDRC |= (1 << PC1) | (1 << PC2) | (1 << PC3);
    PORTC &= ~((1 << PC1) | (1 << PC2) | (1 << PC3));
}

void aviso_exito(void) {
    PORTC |= (1 << PC1);
    _delay_ms(1200);
    PORTC &= ~(1 << PC1);
}

void aviso_error(void) {
    PORTC |= (1 << PC2) | (1 << PC3);
    _delay_ms(400);
    PORTC &= ~(1 << PC3);
    _delay_ms(600);
    PORTC &= ~(1 << PC2);
}

void aviso_alarma_bloqueo(void) {
    for (uint8_t i = 0; i < 5; i++) {
        PORTC |= (1 << PC2) | (1 << PC3);
        _delay_ms(250);
        PORTC &= ~((1 << PC2) | (1 << PC3));
        _delay_ms(250);
    }
}

void eeprom_iniciar(void) {
    uint8_t firma = eeprom_read_byte(EEPROM_FIRMA_DIR);
    if (firma != EEPROM_FIRMA_VALIDA) {
        for (uint8_t i = 0; i < NUM_CUENTAS; i++) {
            uint8_t *dir = (uint8_t*)(1 + (i * sizeof(struct Cuenta)));
            eeprom_write_block((const void*)&cuentas_iniciales[i], (void*)dir, sizeof(struct Cuenta));
        }
        eeprom_write_byte(EEPROM_FIRMA_DIR, EEPROM_FIRMA_VALIDA);
    }
}

void eeprom_guardar_activa(void) {
    if (indice_cuenta >= 0 && indice_cuenta < NUM_CUENTAS) {
        uint8_t *dir = (uint8_t*)(1 + (indice_cuenta * sizeof(struct Cuenta)));
        eeprom_write_block((const void*)&cuenta_activa, (void*)dir, sizeof(struct Cuenta));
    }
}

int buscar_cuenta(uint8_t *uid) {
    for (uint8_t i = 0; i < NUM_CUENTAS; i++) {
        struct Cuenta c;
        uint8_t *dir = (uint8_t*)(1 + (i * sizeof(struct Cuenta)));
        eeprom_read_block((void*)&c, (const void*)dir, sizeof(struct Cuenta));

        if (memcmp(c.uid, uid, 4) == 0) {
            cuenta_activa = c;
            return i;
        }
    }
    return -1;
}

uint32_t leer_monto(void) {
    char buf[8];
    uint8_t pos = 0;
    memset(buf, 0, sizeof(buf));

    while (pos < 6) {
        char k = teclado_leer();
        if (k >= '0' && k <= '9') {
            buf[pos] = k;
            lcd_posicion(pos + 7, 1);
            lcd_caracter(k);
            pos++;
        } else if (k == '#') {
            break;
        } else if (k == '*') {
            return 0;
        }
    }
    if (pos == 0) return 0;
    buf[pos] = '\0';
    uint32_t val = 0;
    for (uint8_t i = 0; i < pos; i++) {
        val = val * 10 + (buf[i] - '0');
    }
    return val;
}

/* =========================================================
   PROGRAMA PRINCIPAL Y MAQUINA DE ESTADOS (FSM)
   ========================================================= */

int main(void) {
    indicadores_iniciar();
    teclado_iniciar();
    spi_iniciar();
    rc522_iniciar();
    lcd_iniciar();
    eeprom_iniciar();
    DDRD &= ~(1 << PD0);
    PORTD |= (1 << PD0);

    lcd_posicion(2, 0);
    lcd_texto("CAJERO ATM");
    lcd_posicion(1, 1);
    lcd_texto("ATmega328P AVR");
    _delay_ms(1500);

    while (1) {
        switch (estado_actual) {

            case ESTADO_ESPERA_TARJETA: {
                lcd_limpiar();
                lcd_posicion(0, 0);
                lcd_texto("Acerque Tarjeta");
                lcd_posicion(0, 1);
                lcd_texto("O pulse Boton...");

                uint8_t uid_leido[4];
                bool detectada = false;

                while (!detectada) {
                    if (!(PIND & (1 << PD0))) {
                        _delay_ms(30);
                        uid_leido[0] = 0xF5;
                        uid_leido[1] = 0xF0;
                        uid_leido[2] = 0x2E;
                        uid_leido[3] = 0x07;
                        detectada = true;

                        while (!(PIND & (1 << PD0)));
                        _delay_ms(30);
                    }
                    else if (rc522_detectar_tarjeta(uid_leido)) {
                        detectada = true;
                    }
                    _delay_ms(50);
                }

#ifdef DEPURAR_UID
                {
                    char uidtxt[17];
                    sprintf(uidtxt, "%02X %02X %02X %02X", uid_leido[0], uid_leido[1], uid_leido[2], uid_leido[3]);
                    lcd_limpiar();
                    lcd_posicion(0, 0);
                    lcd_texto("UID:");
                    lcd_posicion(0, 1);
                    lcd_texto(uidtxt);
                    _delay_ms(3000);
                }
#endif

                indice_cuenta = buscar_cuenta(uid_leido);
                if (indice_cuenta >= 0) {
                    intentos_pin = 0;
                    estado_actual = ESTADO_INGRESO_PIN;
                } else {
                    lcd_limpiar();
                    lcd_posicion(0, 0);
                    lcd_texto("Tarjeta Denegada");
                    aviso_error();
                }
                break;
            }

            case ESTADO_INGRESO_PIN: {
                lcd_limpiar();
                lcd_posicion(0, 0);
                lcd_texto("Ingrese PIN (4d):");
                lcd_posicion(0, 1);

                char pin_ingresado[5];
                uint8_t pos = 0;

                while (pos < 4) {
                    char tecla = teclado_leer();
                    if (tecla >= '0' && tecla <= '9') {
                        pin_ingresado[pos] = tecla;
                        lcd_caracter('*');
                        pos++;
                    } else if (tecla == '*') {
                        estado_actual = ESTADO_ESPERA_TARJETA;
                        break;
                    }
                }

                if (pos == 4) {
                    pin_ingresado[4] = '\0';
                    if (strcmp(pin_ingresado, cuenta_activa.pin) == 0) {
                        lcd_limpiar();
                        lcd_posicion(0, 0);
                        lcd_texto("PIN Aceptado");
                        aviso_exito();
                        estado_actual = ESTADO_MENU_PRINCIPAL;
                    } else {
                        intentos_pin++;
                        lcd_limpiar();
                        lcd_posicion(0, 0);
                        lcd_texto("PIN Incorrecto");
                        lcd_posicion(0, 1);
                        char msg[16];
                        sprintf(msg, "Falla %d de 3", intentos_pin);
                        lcd_texto(msg);
                        aviso_error();

                        if (intentos_pin >= 3) {
                            estado_actual = ESTADO_BLOQUEO;
                        }
                    }
                }
                break;
            }

            case ESTADO_MENU_PRINCIPAL: {
                lcd_limpiar();
                lcd_posicion(0, 0);
                lcd_texto("A:Saldo B:Retiro");
                lcd_posicion(0, 1);
                lcd_texto("C:Dep.  D:Salir");

                char opt = '\0';
                while (opt == '\0') {
                    opt = teclado_leer();
                }

                if (opt == 'A') estado_actual = ESTADO_CONSULTA_SALDO;
                else if (opt == 'B') estado_actual = ESTADO_RETIRO;
                else if (opt == 'C') estado_actual = ESTADO_DEPOSITO;
                else if (opt == 'D') {
                    lcd_limpiar();
                    lcd_posicion(0, 0);
                    lcd_texto("Sesion Cerrada");
                    _delay_ms(1000);
                    estado_actual = ESTADO_ESPERA_TARJETA;
                }
                break;
            }

            case ESTADO_CONSULTA_SALDO: {
                lcd_limpiar();
                lcd_posicion(0, 0);
                lcd_texto("Saldo Actual:");
                lcd_posicion(0, 1);
                char str_saldo[16];
                sprintf(str_saldo, "$ %lu", (unsigned long)cuenta_activa.saldo);
                lcd_texto(str_saldo);

                while (teclado_leer() == '\0') {
                    _delay_ms(50);
                }
                estado_actual = ESTADO_MENU_PRINCIPAL;
                break;
            }

            case ESTADO_RETIRO: {
                lcd_limpiar();
                lcd_posicion(0, 0);
                lcd_texto("Monto a Retirar:");
                lcd_posicion(0, 1);
                lcd_texto("Monto: ");

                uint32_t monto = leer_monto();
                if (monto == 0) {
                    estado_actual = ESTADO_MENU_PRINCIPAL;
                    break;
                }

                if (monto <= cuenta_activa.saldo) {
                    cuenta_activa.saldo -= monto;
                    eeprom_guardar_activa();

                    lcd_limpiar();
                    lcd_posicion(0, 0);
                    lcd_texto("Retiro Exitoso");
                    lcd_posicion(0, 1);
                    lcd_texto("Tome su dinero");
                    aviso_exito();
                } else {
                    lcd_limpiar();
                    lcd_posicion(0, 0);
                    lcd_texto("Saldo");
                    lcd_posicion(0, 1);
                    lcd_texto("Insuficiente");
                    aviso_error();
                }
                estado_actual = ESTADO_MENU_PRINCIPAL;
                break;
            }

            case ESTADO_DEPOSITO: {
                lcd_limpiar();
                lcd_posicion(0, 0);
                lcd_texto("Monto Depositar:");
                lcd_posicion(0, 1);
                lcd_texto("Monto: ");

                uint32_t monto = leer_monto();
                if (monto > 0) {
                    cuenta_activa.saldo += monto;
                    eeprom_guardar_activa();

                    lcd_limpiar();
                    lcd_posicion(0, 0);
                    lcd_texto("Deposito Exitoso");
                    aviso_exito();
                }
                estado_actual = ESTADO_MENU_PRINCIPAL;
                break;
            }

            case ESTADO_BLOQUEO: {
                lcd_limpiar();
                lcd_posicion(0, 0);
                lcd_texto("CUENTA BLOQUEADA");
                lcd_posicion(0, 1);
                lcd_texto("3 Intentos Fall.");

                aviso_alarma_bloqueo();
                _delay_ms(2000);

                intentos_pin = 0;
                estado_actual = ESTADO_ESPERA_TARJETA;
                break;
            }
        }
    }

    return 0;
}
