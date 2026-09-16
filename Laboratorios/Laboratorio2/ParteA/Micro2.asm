.include "m328pdef.inc"

; Configuración USART
.equ baud = 9600
.equ F_CPU = 16000000
.equ bps = (F_CPU/(16*baud))-1

    .def temp = r16
    .def dato = r17

; Tabla de vectores de interrupción
.org 0x0000
    rjmp inicio             ; Reset
.org 0x0024
    rjmp USART_RX_ISR       ; Interrupción USART RX Complete

inicio:
    ldi temp, HIGH(RAMEND)
    out SPH, temp
    ldi temp, LOW(RAMEND)
    out SPL, temp

; Puerto B como salidas (LED0-LED5, pines 8-13)
    ldi temp, 0x3F
    out DDRB, temp
    ldi temp, 0x00
    out PORTB, temp

; Puerto C: PC0, PC1 como salidas (LED6-LED7, pines A0-A1)
    ldi temp, 0x03
    out DDRC, temp
    ldi temp, 0x00
    out PORTC, temp

    rcall initUART

    sei                     ; Habilitar interrupciones globales

main_loop:
    rjmp main_loop          ; No hace nada, todo lo maneja la interrupción

; Inicialización de USART (solo RX con interrupción)
initUART:
    ldi temp, HIGH(bps)
    sts UBRR0H, temp
    ldi temp, LOW(bps)
    sts UBRR0L, temp
    ldi temp, (1<<RXEN0)|(1<<RXCIE0)  ; RX habilitado + interrupción RX
    sts UCSR0B, temp
    ldi temp, (1<<UCSZ01)|(1<<UCSZ00) ; 8 bits datos, 1 stop, sin paridad
    sts UCSR0C, temp
    ret

; Rutina de interrupción: USART RX Complete
USART_RX_ISR:
    lds dato, UDR0          ; leer el byte recibido

    ; Limpiar ambos puertos
    ldi temp, 0x00
    out PORTB, temp
    out PORTC, temp

    cpi dato, 6
    breq led6
    cpi dato, 7
    breq led7

    ; Valores 0-5: LED en PORTB
    ldi temp, 1
    cpi dato, 0
    breq escribir_b
desplazar:
    lsl temp
    dec dato
    brne desplazar
escribir_b:
    out PORTB, temp
    reti

led6:
    sbi PORTC, PC0          ; LED6 en pin A0
    reti
led7:
    sbi PORTC, PC1          ; LED7 en pin A1
    reti
