.include "m328pdef.inc"

; Configuración USART
.equ baud = 9600
.equ F_CPU = 16000000
.equ bps = (F_CPU/(16*baud))-1

; Pulsadores (Puerto D)
.equ BTN0 = PD2       ; Bit 0
.equ BTN1 = PD3       ; Bit 1
.equ BTN2 = PD4       ; Bit 2
.equ BTN_ENV = PD5    ; Botón de enviar

.def temp = r16

.org 0x0000
    rjmp inicio

inicio:
    ldi temp, HIGH(RAMEND)
    out SPH, temp
    ldi temp, LOW(RAMEND)
    out SPL, temp

; Puerto D: PD1 (TX) como salida, PD2-PD5 como entradas
    ldi temp, 0b00000010    ; Solo PD1 (TX) como salida
    out DDRD, temp
    ldi temp, 0b00111100    ; Pull-ups en PD2, PD3, PD4, PD5
    out PORTD, temp

    rcall initUART

    rjmp main_loop

main_loop:

    rjmp main_loop

; Inicialización de USART (solo TX)
initUART:
    ldi temp, HIGH(bps)
    sts UBRR0H, temp
    ldi temp, LOW(bps)
    sts UBRR0L, temp          ; Corregido: era URR0L
    ldi temp, (1<<TXEN0)      ; Solo TX habilitado (MCU1 es transmisor)
    sts UCSR0B, temp
    ldi temp, (1<<UCSZ01)|(1<<UCSZ00)  ; 8 bits datos, 1 stop, sin paridad
    sts UCSR0C, temp
    ret

; Antirrebote (~20ms a 16MHz)
delay_debounce:
    ldi r18, 2
deb_l1:
    ldi r19, 210
deb_l2:
    ldi r20, 255
deb_l3:
    dec r20
    brne deb_l3
    dec r19
    brne deb_l2
    dec r18
    brne deb_l1
    ret
