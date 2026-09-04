.equ MATRIZ_1088AS  = 1

.include "m328pdef.inc"

.def temp           = r16
.def temp2          = r17
.def Pareja         = r18
.def SubFigura      = r19
.def FiguraActual   = r20
.def row_mask       = r21
.def row_counter    = r22
.def btn_history    = r23
.def row_data       = r24

.equ BTN1_PIN       = PC4
.equ BTN2_PIN       = PC5

.cseg
.org 0x0000
    rjmp inicio

inicio:
    ldi temp, HIGH(RAMEND)
    out SPH, temp
    ldi temp, LOW(RAMEND)
    out SPL, temp

    ldi temp, 0xFF
    out DDRD, temp

.if MATRIZ_1088AS == 1
    ldi temp, 0xFF
    out PORTD, temp

    ldi temp, 0x0F
    out DDRB, temp
    in temp, PORTB
    andi temp, 0xF0
    out PORTB, temp

    ldi temp, 0b00001111
    out DDRC, temp
    ldi temp, 0b00110000
    out PORTC, temp
.else
    ldi temp, 0x00
    out PORTD, temp

    ldi temp, 0x0F
    out DDRB, temp
    in temp, PORTB
    ori temp, 0x0F
    out PORTB, temp

    ldi temp, 0b00001111
    out DDRC, temp
    ldi temp, 0b00111111
    out PORTC, temp
.endif

    ldi Pareja, 0
    ldi SubFigura, 0
    ldi btn_history, 0

main_loop:
    rcall leer_botones

    mov FiguraActual, Pareja
    lsl FiguraActual
    add FiguraActual, SubFigura

    rcall mostrar_cuadro

    rjmp main_loop

leer_botones:
    in temp, PINC

    sbrc temp, BTN1_PIN
    rjmp b1_no_presionado

    sbrc btn_history, 0
    rjmp chequear_boton2

    sbr btn_history, (1<<0)
    inc Pareja
    cpi Pareja, 3
    brne fin_b1
    ldi Pareja, 0
fin_b1:
    rjmp chequear_boton2

b1_no_presionado:
    cbr btn_history, (1<<0)

chequear_boton2:
    sbrc temp, BTN2_PIN
    rjmp b2_no_presionado

    sbrc btn_history, 1
    rjmp fin_botones

    sbr btn_history, (1<<1)
    ldi temp2, 1
    eor SubFigura, temp2
    rjmp fin_botones

b2_no_presionado:
    cbr btn_history, (1<<1)

fin_botones:
    ret

mostrar_cuadro:
    ldi ZL, low(tabla_figuras * 2)
    ldi ZH, high(tabla_figuras * 2)

    mov temp, FiguraActual
    lsl temp
    lsl temp
    lsl temp
    add ZL, temp
    clr temp
    adc ZH, temp

    ldi row_mask, 0b00000001
    ldi row_counter, 8

scan_loop:
    lpm row_data, Z+

.if MATRIZ_1088AS == 1
.else
    com row_data
.endif

    in temp, PORTB
    andi temp, 0xF0
    mov temp2, row_data
    andi temp2, 0x0F
    or temp, temp2
    out PORTB, temp

    in temp, PORTC
    andi temp, 0xF0
    mov temp2, row_data
    swap temp2
    andi temp2, 0x0F
    or temp, temp2
    out PORTC, temp

.if MATRIZ_1088AS == 1
    mov temp, row_mask
    com temp
    out PORTD, temp
.else
    out PORTD, row_mask
.endif

    rcall delay_fila

.if MATRIZ_1088AS == 1
    ldi temp, 0xFF
    out PORTD, temp
.else
    ldi temp, 0x00
    out PORTD, temp
.endif

    lsl row_mask
    dec row_counter
    brne scan_loop

    ret

delay_fila:
    push r25
    push r26
    ldi r25, 25
delay_outer:
    ldi r26, 255
delay_inner:
    dec r26
    brne delay_inner
    dec r25
    brne delay_outer
    pop r26
    pop r25
    ret

.align 2
tabla_figuras:
    ; FIGURA 0: Carita sonriendo
    .db 0b00111100, 0b01000010
    .db 0b10100101, 0b10000001
    .db 0b10100101, 0b10011001
    .db 0b01000010, 0b00111100

    ; FIGURA 1: Carita guiñando
    .db 0b00111100, 0b01000010
    .db 0b10000101, 0b10110001
    .db 0b10100101, 0b10011001
    .db 0b01000010, 0b00111100

    ; FIGURA 2: Corazón
    .db 0b00000000, 0b01100110
    .db 0b11111111, 0b11111111
    .db 0b01111110, 0b00111100
    .db 0b00011000, 0b00000000

    ; FIGURA 3: :3
    .db 0b00000000, 0b00001110
    .db 0b01100010, 0b01100110
    .db 0b00000010, 0b01100010
    .db 0b01101110, 0b00000000

    ; FIGURA 4: Asterisco
    .db 0b00011000, 0b10011001
    .db 0b01011010, 0b11111111
    .db 0b11111111, 0b01011010
    .db 0b10011001, 0b00011000

    ; FIGURA 5: XD
    .db 0b00000000, 0b10101110
    .db 0b10101001, 0b01001001
    .db 0b01001001, 0b10101001
    .db 0b10101110, 0b00000000
