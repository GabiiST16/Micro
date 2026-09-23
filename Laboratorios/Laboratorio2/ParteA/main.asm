.include "m328pdef.inc"

.def temp          = r16
.def uart_data     = r17
.def max_reg       = r18
.def max_val       = r19
.def mode_flag     = r20
.def scroll_offset = r21  
.def col_idx       = r22
.def bit_cnt       = r23

.equ F_CPU         = 16000000
.equ BAUD          = 9600
.equ UBRR_VAL      = (F_CPU / (16 * BAUD)) - 1

.equ PIN_SS        = PB2
.equ PIN_MOSI      = PB3
.equ PIN_SCK       = PB5

.equ REG_DECODE_MODE  = 0x09
.equ REG_INTENSITY    = 0x0A
.equ REG_SCAN_LIMIT   = 0x0B
.equ REG_SHUTDOWN     = 0x0C
.equ REG_DISPLAY_TEST = 0x0F

.cseg
.org 0x0000
    rjmp RESET
.org 0x0024
    rjmp UART_RX_ISR
RESET:
    ldi temp, HIGH(RAMEND)
    out SPH, temp
    ldi temp, LOW(RAMEND)
    out SPL, temp

    ldi temp, (1<<PIN_MOSI) | (1<<PIN_SCK) | (1<<PIN_SS)
    out DDRB, temp
    sbi PORTB, PIN_SS
    cbi PORTB, PIN_SCK

    rcall INIT_MAX7219
    rcall INIT_UART

    clr mode_flag
    clr scroll_offset
    sei

    rcall SEND_MENU_UART

MAIN_LOOP:
    cpi mode_flag, 0
    brne CHECK_FIG1
    rcall SHOW_SCROLLING_TEXT
    rjmp MAIN_LOOP

CHECK_FIG1:
    cpi mode_flag, 1
    brne CHECK_FIG2
    rcall SHOW_FIGURE_1
    rcall DELAY_FRAME
    rjmp MAIN_LOOP

CHECK_FIG2:
    rcall SHOW_FIGURE_2
    rcall DELAY_FRAME
    rjmp MAIN_LOOP

MAX7219_SEND:
    cbi PORTB, PIN_SS

    mov temp, max_reg
    rcall SHIFT_OUT_BYTE

    mov temp, max_val
    rcall SHIFT_OUT_BYTE

    sbi PORTB, PIN_SS
    ret

SHIFT_OUT_BYTE:
    ldi bit_cnt, 8
BIT_LOOP:
    cbi PORTB, PIN_SCK
    sbrc temp, 7
    sbi PORTB, PIN_MOSI
    sbrs temp, 7
    cbi PORTB, PIN_MOSI

    sbi PORTB, PIN_SCK
    lsl temp
    dec bit_cnt
    brne BIT_LOOP
    cbi PORTB, PIN_SCK
    ret

INIT_MAX7219:
    ldi max_reg, REG_DISPLAY_TEST
    ldi max_val, 0x00
    rcall MAX7219_SEND

    ldi max_reg, REG_DECODE_MODE
    ldi max_val, 0x00
    rcall MAX7219_SEND

    ldi max_reg, REG_SCAN_LIMIT
    ldi max_val, 0x07
    rcall MAX7219_SEND

    ldi max_reg, REG_INTENSITY
    ldi max_val, 0x08
    rcall MAX7219_SEND

    ldi max_reg, REG_SHUTDOWN
    ldi max_val, 0x01
    rcall MAX7219_SEND

    rcall CLEAR_MATRIX
    ret

CLEAR_MATRIX:
    ldi col_idx, 1
CLR_L:
    mov max_reg, col_idx
    ldi max_val, 0x00
    rcall MAX7219_SEND
    inc col_idx
    cpi col_idx, 9
    brne CLR_L
    ret

SHOW_SCROLLING_TEXT:
    ldi col_idx, 1

    ldi ZH, HIGH(TEXT_LUT * 2)
    ldi ZL, LOW(TEXT_LUT * 2)

    add ZL, scroll_offset
    brcc NO_INC_H
    inc ZH
NO_INC_H:

DRAW_8_COLS:
    lpm max_val, Z+
    mov max_reg, col_idx
    rcall MAX7219_SEND

    inc col_idx
    cpi col_idx, 9
    brne DRAW_8_COLS

    rcall DELAY_FRAME

    inc scroll_offset
    cpi scroll_offset, 85
    brne END_SCROLL_STEP
    clr scroll_offset

END_SCROLL_STEP:
    ret

SHOW_FIGURE_1:
    ldi ZH, HIGH(FIG1_LUT * 2)
    ldi ZL, LOW(FIG1_LUT * 2)
    rcall DRAW_STATIC_LUT
    ret

SHOW_FIGURE_2:
    ldi ZH, HIGH(FIG2_LUT * 2)
    ldi ZL, LOW(FIG2_LUT * 2)
    rcall DRAW_STATIC_LUT
    ret

DRAW_STATIC_LUT:
    ldi col_idx, 1
DRAW_ST_LOOP:
    lpm max_val, Z+
    mov max_reg, col_idx
    rcall MAX7219_SEND
    inc col_idx
    cpi col_idx, 9
    brne DRAW_ST_LOOP
    ret

DELAY_FRAME:
    ldi r23, 15
DL0: ldi r24, 200
DL1: ldi r25, 200
DL2: dec r25
    brne DL2
    dec r24
    brne DL1
    dec r23
    brne DL0
    ret

INIT_UART:
    ldi temp, HIGH(UBRR_VAL)
    sts UBRR0H, temp
    ldi temp, LOW(UBRR_VAL)
    sts UBRR0L, temp

    ldi temp, (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0)
    sts UCSR0B, temp

    ldi temp, (1<<UCSZ01) | (1<<UCSZ00)
    sts UCSR0C, temp
    ret

UART_RX_ISR:
    push temp
    in temp, SREG
    push temp
    push uart_data

    lds uart_data, UDR0
    cpi uart_data, '0'
    breq SET_SCROLL
    cpi uart_data, '1'
    breq SET_FIG1
    cpi uart_data, '2'
    breq SET_FIG2
    rjmp ISR_EXIT

SET_SCROLL:
    clr mode_flag
    clr scroll_offset
    rjmp ISR_EXIT

SET_FIG1:
    ldi mode_flag, 1
    rjmp ISR_EXIT

SET_FIG2:
    ldi mode_flag, 2

ISR_EXIT:
    pop uart_data
    pop temp
    out SREG, temp
    pop temp
    reti

SEND_MENU_UART:
    ldi ZH, HIGH(WELCOME_MENU_TEXT * 2)
    ldi ZL, LOW(WELCOME_MENU_TEXT * 2)
SEND_M_LOOP:
    lpm temp, Z+
    tst temp
    breq END_MENU
    rcall UART_TX
    rjmp SEND_M_LOOP
END_MENU:
    ret

UART_TX:
    lds r24, UCSR0A
    sbrs r24, UDRE0
    rjmp UART_TX
    sts UDR0, temp
    ret

WELCOME_MENU_TEXT:
    .db 0x0D, 0x0A
    .db "=== MENU MATRIZ LEDs ===", 0x0D, 0x0A
    .db "0: Mensaje Desplazable", 0x0D, 0x0A
    .db "1: Carita Feliz.", 0x0D, 0x0A
    .db "2: Corazon", 0x0D, 0x0A
    .db ">", 0x00

TEXT_LUT:
    .db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    .db 0x01, 0x01, 0x7F, 0x01, 0x01, 0x00
    .db 0x7F, 0x49, 0x49, 0x49, 0x41, 0x00
    .db 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00
    .db 0x7F, 0x49, 0x49, 0x49, 0x41, 0x00
    .db 0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x00
    .db 0x7F, 0x49, 0x49, 0x49, 0x41, 0x00
    .db 0x00, 0x00, 0x00, 0x00
    .db 0x7F, 0x09, 0x09, 0x09, 0x06, 0x00
    .db 0x41, 0x41, 0x7F, 0x41, 0x41, 0x00
    .db 0x7F, 0x49, 0x49, 0x49, 0x41, 0x00
    .db 0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00
    .db 0x7E, 0x09, 0x09, 0x09, 0x7E, 0x00
    .db 0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00
    .db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

FIG1_LUT:
    .db 0x3C, 0x42, 0xA5, 0x81, 0xA5, 0x99, 0x42, 0x3C

FIG2_LUT:
    .db 0x0C, 0x1E, 0x3E, 0x7C, 0x7C, 0x3E, 0x1E, 0x0C