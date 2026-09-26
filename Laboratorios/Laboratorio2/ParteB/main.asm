; ==============================================================================
; ARCHIVO: dac_2rr_grupo16.asm
; DISPOSITIVO: ATmega328P
; FRECUENCIA DE RELOJ: 16 MHz
; DESCRIPCIÓN: Generador de señales con DAC R-2R mediante LUT.
;              Control de SEÑAL y FRECUENCIA vía UART (9600 bps).
;              DAC en: PB0..PB1 (bits 0-1) y PD2..PD7 (bits 2-7).
;              UART en: PD0 (RX) y PD1 (TX).
; ==============================================================================

.include "m328pdef.inc"

; ------------------------------------------------------------------------------
; DEFINICIÓN DE REGISTROS
; ------------------------------------------------------------------------------
.def temp        = r16    ; Registro de trabajo general
.def uart_data   = r17    ; Dato recibido/enviado por UART
.def muestra     = r18    ; Muestra actual leída de la Flash
.def indice      = r19    ; Índice de la muestra (0 a 255)
.def aux         = r20    ; Auxiliar para separación de bits en la ISR
.def valor_ocr   = r21    ; Almacena y ajusta el período de Timer0
.def ptr_base_L  = r24    ; Puntero base de la tabla activa (Byte bajo)
.def ptr_base_H  = r25    ; Puntero base de la tabla activa (Byte alto)

; ------------------------------------------------------------------------------
; VECTOR DE INTERRUPCIONES
; ------------------------------------------------------------------------------
.org 0x0000
    rjmp RESET
.org OC0Aaddr             ; Vector oficial Timer0 Compare Match A (0x001C)
    rjmp TIMER0_COMPA_ISR

; ------------------------------------------------------------------------------
; CONFIGURACIÓN DE INICIO
; ------------------------------------------------------------------------------
RESET:
    ; 1. Inicialización del Stack Pointer
    ldi temp, LOW(RAMEND)
    out SPL, temp
    ldi temp, HIGH(RAMEND)
    out SPH, temp

    ; 2. Configurar salidas para el DAC R-2R
    ; PORTD: PD2..PD7 como SALIDAS. PD0 (RX) entrada, PD1 (TX) salida UART.
    ldi temp, 0b11111110
    out DDRD, temp
    clr temp
    out PORTD, temp

    ; PORTB: PB0 y PB1 como SALIDAS (Bits 0 y 1 del DAC)
    in temp, DDRB
    ori temp, (1<<PB0) | (1<<PB1)
    out DDRB, temp
    cbi PORTB, PB0
    cbi PORTB, PB1

    ; 3. Inicializar UART a 9600 bps @ 16 MHz (UBRR = 103)
    ldi temp, 103
    sts UBRR0L, temp
    clr temp
    sts UBRR0H, temp
    ldi temp, (1<<RXEN0) | (1<<TXEN0)
    sts UCSR0B, temp
    ldi temp, (1<<UCSZ01) | (1<<UCSZ00)
    sts UCSR0C, temp

    ; 4. Configurar Timer0 en modo CTC
    ; Prescaler = 64, OCR0A inicial = 249 (1 kHz de tasa de muestreo)
    ldi temp, (1<<WGM01)
    out TCCR0A, temp
    ldi temp, 249
    out OCR0A, temp
    ldi temp, (1<<CS01) | (1<<CS00)
    out TCCR0B, temp
    ldi temp, (1<<OCIE0A)
    sts TIMSK0, temp

    ; 5. Estado inicial: Señal 1 (Triangular)
    rcall CARGAR_SENAL_18
    clr indice

    ; Habilitar interrupciones globales
    sei

; ------------------------------------------------------------------------------
; BUCLE PRINCIPAL (Polling UART)
; ------------------------------------------------------------------------------
MAIN_LOOP:
    lds temp, UCSR0A
    sbrs temp, RXC0
    rjmp MAIN_LOOP          ; Esperar dato por UART

    lds uart_data, UDR0

    ; --- Selector de forma de onda ---
    cpi uart_data, '1'
    breq CMD_SENAL_1
    cpi uart_data, '2'
    breq CMD_SENAL_2

    ; --- Ajuste dinámico de frecuencia ---
    cpi uart_data, '+'
    breq CMD_FREQ_INC
    cpi uart_data, '-'
    breq CMD_FREQ_DEC

    ; --- Presets de frecuencia directos ---
    cpi uart_data, '3'
    breq CMD_FREQ_BAJA
    cpi uart_data, '4'
    breq CMD_FREQ_MEDIA
    cpi uart_data, '5'
    breq CMD_FREQ_ALTA

    rjmp MAIN_LOOP

; ------------------------------------------------------------------------------
; MANEJADORES DE COMANDOS
; ------------------------------------------------------------------------------
CMD_SENAL_1:
    cli
    rcall CARGAR_SENAL_18
    clr indice
    sei
    rcall CONFIRMAR_OK1
    rjmp MAIN_LOOP

CMD_SENAL_2:
    cli
    rcall CARGAR_SENAL_3
    clr indice
    sei
    rcall CONFIRMAR_OK2
    rjmp MAIN_LOOP

CMD_FREQ_INC:
    ; Aumentar frecuencia = achicar
    in valor_ocr, OCR0A
    cpi valor_ocr, 35      ; Límite mínimo de seguridad
    brlo fin_f_inc
    subi valor_ocr, 20    ; Restar 20 al período
    out OCR0A, valor_ocr
    rcall CONFIRMAR_F_INC
fin_f_inc:
    rjmp MAIN_LOOP

CMD_FREQ_DEC:
    ; Disminuir frecuencia = agrandar OCR0A
    in valor_ocr, OCR0A
    cpi valor_ocr, 230       ; Límite máximo
    brsh fin_f_dec
    subi valor_ocr, -20      ; Sumar 20 al período
    out OCR0A, valor_ocr
    rcall CONFIRMAR_F_DEC
fin_f_dec:
    rjmp MAIN_LOOP

CMD_FREQ_BAJA:
    ldi temp, 249            ; OCR0A = 249 (~3.9 Hz en sierra)
    out OCR0A, temp
    rcall CONFIRMAR_F_BAJA
    rjmp MAIN_LOOP

CMD_FREQ_MEDIA:
    ldi temp, 124            ; OCR0A = 124 (~7.8 Hz en sierra)
    out OCR0A, temp
    rcall CONFIRMAR_F_MEDIA
    rjmp MAIN_LOOP

CMD_FREQ_ALTA:
    ldi temp, 49             ; OCR0A = 49 (~19.5 Hz en sierra)
    out OCR0A, temp
    rcall CONFIRMAR_F_ALTA
    rjmp MAIN_LOOP

; ------------------------------------------------------------------------------
; RUTINAS DE SELECCIÓN Y RESPUESTAS UART
; ------------------------------------------------------------------------------
CARGAR_SENAL_18:
    ldi ptr_base_H, HIGH(tabla_triangular * 2)
    ldi ptr_base_L, LOW(tabla_triangular * 2)
    ret

CARGAR_SENAL_3:
    ldi ptr_base_H, HIGH(tabla_sierra * 2)
    ldi ptr_base_L, LOW(tabla_sierra * 2)
    ret

CONFIRMAR_OK1:
    ldi uart_data, 'O'
    rcall ENVIAR_CHAR
    ldi uart_data, 'K'
    rcall ENVIAR_CHAR
    ldi uart_data, '1'
    rcall ENVIAR_CHAR
    rjmp ENVIAR_EOL

CONFIRMAR_OK2:
    ldi uart_data, 'O'
    rcall ENVIAR_CHAR
    ldi uart_data, 'K'
    rcall ENVIAR_CHAR
    ldi uart_data, '2'
    rcall ENVIAR_CHAR
    rjmp ENVIAR_EOL

CONFIRMAR_F_INC:
    ldi uart_data, 'F'
    rcall ENVIAR_CHAR
    ldi uart_data, '+'
    rcall ENVIAR_CHAR
    rjmp ENVIAR_EOL

CONFIRMAR_F_DEC:
    ldi uart_data, 'F'
    rcall ENVIAR_CHAR
    ldi uart_data, '-'
    rcall ENVIAR_CHAR
    rjmp ENVIAR_EOL

CONFIRMAR_F_BAJA:
    ldi uart_data, 'F'
    rcall ENVIAR_CHAR
    ldi uart_data, '1'
    rcall ENVIAR_CHAR
    rjmp ENVIAR_EOL

CONFIRMAR_F_MEDIA:
    ldi uart_data, 'F'
    rcall ENVIAR_CHAR
    ldi uart_data, '2'
    rcall ENVIAR_CHAR
    rjmp ENVIAR_EOL

CONFIRMAR_F_ALTA:
    ldi uart_data, 'F'
    rcall ENVIAR_CHAR
    ldi uart_data, '3'
    rcall ENVIAR_CHAR
    rjmp ENVIAR_EOL

ENVIAR_EOL:
    ldi uart_data, 13
    rcall ENVIAR_CHAR
    ldi uart_data, 10
    rcall ENVIAR_CHAR
    ret

ENVIAR_CHAR:
    lds temp, UCSR0A
    sbrs temp, UDRE0
    rjmp ENVIAR_CHAR
    sts UDR0, uart_data
    ret

; ------------------------------------------------------------------------------
; ISR TIMER0 (Generación de muestras DAC)
; ------------------------------------------------------------------------------
TIMER0_COMPA_ISR:
    in temp, SREG
    push temp
    push aux
    push ZL
    push ZH

    ; Cargar dirección base seleccionada
    mov ZL, ptr_base_L
    mov ZH, ptr_base_H

    ; Sumar índice actual a la base con acarreo
    add ZL, indice
    clr temp
    adc ZH, temp

    ; Leer muestra desde la memoria Flash
    lpm muestra, Z

    ; 1. Escribir bits 2..7 en PD2..PD7 (preservando PD0 y PD1)
    in temp, PORTD
    andi temp, 0b00000011
    mov aux, muestra
    andi aux, 0b11111100
    or temp, aux
    out PORTD, temp

    ; 2. Escribir bits 0..1 en PB0..PB1 (preservando PB2..PB7)
    in temp, PORTB
    andi temp, 0b11111100
    mov aux, muestra
    andi aux, 0b00000011
    or temp, aux
    out PORTB, temp

    ; Incrementar índice (0 a 255 -> 0)
    inc indice

    pop ZH
    pop ZL
    pop aux
    pop temp
    out SREG, temp
    reti

; ------------------------------------------------------------------------------
; TABLAS DE DATOS (LUT en Flash - 256 muestras cada una)
; ------------------------------------------------------------------------------
tabla_triangular:
    .db 0x00, 0x03, 0x07, 0x0b, 0x0f, 0x13, 0x17, 0x1b, 0x1f, 0x23, 0x27, 0x2b, 0x2f, 0x33, 0x37, 0x3b
    .db 0x3f, 0x43, 0x47, 0x4b, 0x4f, 0x53, 0x57, 0x5b, 0x5f, 0x63, 0x67, 0x6b, 0x6f, 0x73, 0x77, 0x7b
    .db 0x7f, 0x83, 0x87, 0x8b, 0x8f, 0x93, 0x97, 0x9b, 0x9f, 0xa3, 0xa7, 0xab, 0xaf, 0xb3, 0xb7, 0xbb
    .db 0xbf, 0xc3, 0xc7, 0xcb, 0xcf, 0xd3, 0xd7, 0xdb, 0xdf, 0xe3, 0xe7, 0xeb, 0xef, 0xf3, 0xf7, 0xfb
    .db 0xff, 0xfb, 0xf7, 0xf3, 0xef, 0xeb, 0xe7, 0xe3, 0xdf, 0xdb, 0xd7, 0xd3, 0xcf, 0xcb, 0xc7, 0xc3
    .db 0xbf, 0xbb, 0xb7, 0xb3, 0xaf, 0xab, 0xa7, 0xa3, 0x9f, 0x9b, 0x97, 0x93, 0x8f, 0x8b, 0x87, 0x83
    .db 0x7f, 0x7b, 0x77, 0x73, 0x6f, 0x6b, 0x67, 0x63, 0x5f, 0x5b, 0x57, 0x53, 0x4f, 0x4b, 0x47, 0x43
    .db 0x3f, 0x3b, 0x37, 0x33, 0x2f, 0x2b, 0x27, 0x23, 0x1f, 0x1b, 0x17, 0x13, 0x0f, 0x0b, 0x07, 0x03
    .db 0x00, 0x03, 0x07, 0x0b, 0x0f, 0x13, 0x17, 0x1b, 0x1f, 0x23, 0x27, 0x2b, 0x2f, 0x33, 0x37, 0x3b
    .db 0x3f, 0x43, 0x47, 0x4b, 0x4f, 0x53, 0x57, 0x5b, 0x5f, 0x63, 0x67, 0x6b, 0x6f, 0x73, 0x77, 0x7b
    .db 0x7f, 0x83, 0x87, 0x8b, 0x8f, 0x93, 0x97, 0x9b, 0x9f, 0xa3, 0xa7, 0xab, 0xaf, 0xb3, 0xb7, 0xbb
    .db 0xbf, 0xc3, 0xc7, 0xcb, 0xcf, 0xd3, 0xd7, 0xdb, 0xdf, 0xe3, 0xe7, 0xeb, 0xef, 0xf3, 0xf7, 0xfb
    .db 0xff, 0xfb, 0xf7, 0xf3, 0xef, 0xeb, 0xe7, 0xe3, 0xdf, 0xdb, 0xd7, 0xd3, 0xcf, 0xcb, 0xc7, 0xc3
    .db 0xbf, 0xbb, 0xb7, 0xb3, 0xaf, 0xab, 0xa7, 0xa3, 0x9f, 0x9b, 0x97, 0x93, 0x8f, 0x8b, 0x87, 0x83
    .db 0x7f, 0x7b, 0x77, 0x73, 0x6f, 0x6b, 0x67, 0x63, 0x5f, 0x5b, 0x57, 0x53, 0x4f, 0x4b, 0x47, 0x43
    .db 0x3f, 0x3b, 0x37, 0x33, 0x2f, 0x2b, 0x27, 0x23, 0x1f, 0x1b, 0x17, 0x13, 0x0f, 0x0b, 0x07, 0x03

tabla_sierra:
    .db 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
    .db 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f
    .db 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f
    .db 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f
    .db 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f
    .db 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f
    .db 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f
    .db 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f
    .db 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f
    .db 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f
    .db 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf
    .db 0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf
    .db 0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf
    .db 0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf
    .db 0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef
    .db 0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff
