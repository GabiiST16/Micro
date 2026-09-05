.include "m328pdef.inc"

.equ F_CPU      = 16000000
.equ baud       = 9600
.equ bps        = (F_CPU/16/baud) - 1

.def temp       = r16
.def aux        = r17
.def dato_rec   = r18
.def num_val    = r19
.def segmentos  = r20

.cseg
.org 0x0000
    rjmp Inicio

LUT_7SEG:
    .db  0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07
    .db  0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71

Inicio:
    ldi r16, HIGH(RAMEND)
    out SPH, r16
    ldi r16, LOW(RAMEND)
    out SPL, r16

    in temp, DDRD
    ori temp, 0b11111100
    out DDRD, temp

    sbi DDRB, PB0
    sbi DDRB, PB1

    cbi PORTB, PB1

    ldi r16, LOW(bps)
    ldi r17, HIGH(bps)
    rcall initUART

    ldi r16, 0
    rcall buscar_en_lut
    rcall mostrar_en_display

wait:
    rcall getc
    mov dato_rec, r16

    mov num_val, dato_rec
    rcall ascii_a_hex
    brcs wait

    mov r16, dato_rec
    rcall putc

    mov r16, num_val
    rcall buscar_en_lut

    rcall mostrar_en_display

    rjmp wait

mostrar_en_display:
    mov temp, segmentos
    lsl temp
    lsl temp
    andi temp, 0b11111100

    in aux, PORTD
    andi aux, 0b00000011
    or aux, temp
    out PORTD, aux

    sbrc segmentos, 6
    sbi PORTB, PB0
    sbrs segmentos, 6
    cbi PORTB, PB0

    cbi PORTB, PB1

    ret

initUART:
    sts UBRR0L, r16
    sts UBRR0H, r17
    ldi r16, (1<<RXEN0) | (1<<TXEN0)
    sts UCSR0B, r16
    ldi r16, (1<<UCSZ01) | (1<<UCSZ00)
    sts UCSR0C, r16
    ret

putc:
    lds r17, UCSR0A
    sbrs r17, UDRE0
    rjmp putc
    sts UDR0, r16
    ret

getc:
    lds r17, UCSR0A
    sbrs r17, RXC0
    rjmp getc
    lds r16, UDR0
    ret

ascii_a_hex:
    cpi num_val, '0'
    brlo valor_invalido
    cpi num_val, '9' + 1
    brsh check_mayusculas
    subi num_val, '0'
    clc
    ret

check_mayusculas:
    cpi num_val, 'A'
    brlo valor_invalido
    cpi num_val, 'F' + 1
    brsh check_minusculas
    subi num_val, ('A' - 10)
    clc
    ret

check_minusculas:
    cpi num_val, 'a'
    brlo valor_invalido
    cpi num_val, 'f' + 1
    brsh valor_invalido
    subi num_val, ('a' - 10)
    clc
    ret

valor_invalido:
    sec
    ret

buscar_en_lut:
    ldi ZL, LOW(LUT_7SEG * 2)
    ldi ZH, HIGH(LUT_7SEG * 2)
    add ZL, r16
    ldi aux, 0
    adc ZH, aux
    lpm segmentos, Z
    ret
