.include "m328pdef.inc"

; Configuración USART
.equ baud = 9600
.equ F_CPU = 16000000
.equ bps = (F_CPU/(16*baud))-1

; Pulsadores (Puerto B)
.equ BTN0 = PB0       ; Pin 8  (Bit 0)
.equ BTN1 = PB1       ; Pin 9  (Bit 1)
.equ BTN2 = PB2       ; Pin 10 (Bit 2)
.equ BTN_ENV = PB3    ; Pin 11 (Botón de enviar)

    .def temp = r16
    .def dato = r17

.org 0x0000
    rjmp inicio

inicio:
    ldi temp, HIGH(RAMEND)
    out SPH, temp
    ldi temp, LOW(RAMEND)
    out SPL, temp

; Puerto D: PD1 (TX) como salida
    ldi temp, 0b00000010
    out DDRD, temp

; Puerto B como entradas (botones)
    ldi temp, 0x00
    out DDRB, temp
    ldi temp, 0x0F          ; Pull-ups en PB0, PB1, PB2, PB3
    out PORTB, temp

    rcall initUART

    rjmp main_loop

main_loop:
    rjmp main_loop

; Inicialización de USART (solo TX)
initUART:
    ldi temp, HIGH(bps)
    sts UBRR0H, temp
    ldi temp, LOW(bps)
    sts UBRR0L, temp
    ldi temp, (1<<TXEN0)      ; Solo TX habilitado (MCU1 es transmisor)
    sts UCSR0B, temp
    ldi temp, (1<<UCSZ01)|(1<<UCSZ00)  ; 8 bits datos, 1 stop, sin paridad
    sts UCSR0C, temp
    ret
