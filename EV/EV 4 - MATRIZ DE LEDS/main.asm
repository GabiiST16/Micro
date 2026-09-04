.include "m328pdef.inc"

.def temp           = r16
.def temp2          = r17
.def Pareja         = r18
.def SubFigura      = r19
.def FiguraActual   = r20
.def spi_data       = r21
.def bit_count      = r22
.def btn_history    = r23
.def row_data       = r24


.equ CS_PIN         = PB2
.equ DIN_PIN        = PB3
.equ CLK_PIN        = PB5

.equ BTN1_PIN       = PC4
.equ BTN2_PIN       = PC5

.equ MAX_DIGIT0     = 0x01
.equ MAX_DIGIT7     = 0x08
.equ MAX_DECODE     = 0x09
.equ MAX_INTENSITY  = 0x0A
.equ MAX_SCANLIMIT  = 0x0B
.equ MAX_SHUTDOWN   = 0x0C
.equ MAX_DISPTEST   = 0x0F

.cseg
.org 0x0000
    rjmp inicio

inicio:
    ldi temp, HIGH(RAMEND)
    out SPH, temp
    ldi temp, LOW(RAMEND)
    out SPL, temp

    ldi temp, (1<<CS_PIN) | (1<<DIN_PIN) | (1<<CLK_PIN)
    out DDRB, temp

    ldi temp, (1<<CS_PIN)
    out PORTB, temp

    ldi temp, 0x00
    out DDRC, temp
    ldi temp, (1<<BTN1_PIN) | (1<<BTN2_PIN)
    out PORTC, temp

    rcall max7219_init

    ldi Pareja, 0
    ldi SubFigura, 0
    ldi FiguraActual, 0
    ldi btn_history, 0

    rcall enviar_figura

main_loop:
    rcall leer_botones

    mov temp, Pareja
    lsl temp
    add temp, SubFigura

    cp temp, FiguraActual
    breq no_cambio

    mov FiguraActual, temp
    rcall enviar_figura

no_cambio:
    rcall delay_debounce
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
    brne chequear_boton2
    ldi Pareja, 0
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

enviar_figura:
    ldi ZL, low(tabla_figuras * 2)
    ldi ZH, high(tabla_figuras * 2)

    mov temp, FiguraActual
    lsl temp
    lsl temp
    lsl temp
    add ZL, temp
    clr temp
    adc ZH, temp

    ldi temp2, MAX_DIGIT0

enviar_fila_loop:
    lpm row_data, Z+
    mov spi_data, temp2
    rcall max7219_send
    inc temp2
    cpi temp2, MAX_DIGIT7 + 1
    brne enviar_fila_loop

    ret

max7219_init:
    ldi spi_data, MAX_DISPTEST
    ldi row_data, 0x00
    rcall max7219_send

    ldi spi_data, MAX_DECODE
    ldi row_data, 0x00
    rcall max7219_send

    ldi spi_data, MAX_SCANLIMIT
    ldi row_data, 0x07
    rcall max7219_send

    ldi spi_data, MAX_INTENSITY
    ldi row_data, 0x07
    rcall max7219_send

    ldi spi_data, MAX_SHUTDOWN
    ldi row_data, 0x01
    rcall max7219_send

    ldi spi_data, MAX_DIGIT0
    ldi row_data, 0x00
clear_loop:
    rcall max7219_send
    inc spi_data
    cpi spi_data, MAX_DIGIT7 + 1
    brne clear_loop

    ret

max7219_send:
    cbi PORTB, CS_PIN

    push spi_data
    ldi bit_count, 8
send_addr_bit:
    cbi PORTB, DIN_PIN
    sbrc spi_data, 7
    sbi PORTB, DIN_PIN
    sbi PORTB, CLK_PIN
    cbi PORTB, CLK_PIN
    lsl spi_data
    dec bit_count
    brne send_addr_bit
    pop spi_data

    push row_data
    ldi bit_count, 8
send_data_bit:
    cbi PORTB, DIN_PIN
    sbrc row_data, 7
    sbi PORTB, DIN_PIN
    sbi PORTB, CLK_PIN
    cbi PORTB, CLK_PIN
    lsl row_data
    dec bit_count
    brne send_data_bit
    pop row_data

    sbi PORTB, CS_PIN
    ret

delay_debounce:
    push r25
    push r26
    push r27
    ldi r27, 4
deb_outer2:
    ldi r25, 100
deb_outer:
    ldi r26, 200
deb_inner:
    dec r26
    brne deb_inner
    dec r25
    brne deb_outer
    dec r27
    brne deb_outer2
    pop r27
    pop r26
    pop r25
    ret

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
