.include "m328pdef.inc"

.def TEMP        = R16
.def SEQ_INDEX   = R17
.def LED_PATTERN = R18
.def DELAY_R1    = R19
.def DELAY_R2    = R20
.def DELAY_R3    = R21

.org 0x0000
    rjmp RESET

RESET:
    ; Configuración del Stack Pointer
    ldi TEMP, LOW(RAMEND)
    out SPL, TEMP
    ldi TEMP, HIGH(RAMEND)
    out SPH, TEMP

    ; PORTD como salida (8 LEDs PD0 a PD7)
    ldi TEMP, 0xFF
    out DDRD, TEMP
    clr TEMP
    out PORTD, TEMP

    ; PORTB como entrada (Botones PB0, PB1, PB2)
    clr TEMP
    out DDRB, TEMP
    out PORTB, TEMP

    clr SEQ_INDEX
    ldi LED_PATTERN, 0x01

MAIN_LOOP:
    rcall LEER_BOTONES
    rcall MOSTRAR_PASO
    rcall DELAY_200MS
    rjmp MAIN_LOOP


LEER_BOTONES:
    in TEMP, PINB
    andi TEMP, 0x07       ; Máscara PB0, PB1, PB2
    breq FIN_LEER_BOTONES

    rcall DELAY_20MS      ; Anti-rebote
    in TEMP, PINB
    andi TEMP, 0x07
    breq FIN_LEER_BOTONES

    ; Botón 1 (PB0): Avanzar
    sbrc TEMP, 0
    rjmp BTN_NEXT

    ; Botón 2 (PB1): Retroceder
    sbrc TEMP, 1
    rjmp BTN_PREV

    ; Botón 3 (PB2): Reiniciar
    sbrc TEMP, 2
    rjmp BTN_RESET

    rjmp FIN_LEER_BOTONES

BTN_NEXT:
    inc SEQ_INDEX
    cpi SEQ_INDEX, 8      ; Ahora son 8 secuencias (0 a 7)
    brne REINICIAR_PATRON
    clr SEQ_INDEX
    rjmp REINICIAR_PATRON

BTN_PREV:
    tst SEQ_INDEX
    brne DEC_SEQ
    ldi SEQ_INDEX, 7      ; Regresa a la secuencia 7
    rjmp REINICIAR_PATRON
DEC_SEQ:
    dec SEQ_INDEX
    rjmp REINICIAR_PATRON

BTN_RESET:
    clr SEQ_INDEX

REINICIAR_PATRON:
    ldi LED_PATTERN, 0x01

FIN_LEER_BOTONES:
    ret


MOSTRAR_PASO:
    cpi SEQ_INDEX, 0
    breq EXEC_SEQ1
    cpi SEQ_INDEX, 1
    breq EXEC_SEQ2
    cpi SEQ_INDEX, 2
    breq EXEC_SEQ3
    cpi SEQ_INDEX, 3
    breq EXEC_SEQ4
    cpi SEQ_INDEX, 4
    breq EXEC_SEQ5
    cpi SEQ_INDEX, 5
    breq EXEC_SEQ6
    cpi SEQ_INDEX, 6
    breq EXEC_SEQ7
    cpi SEQ_INDEX, 7
    breq EXEC_SEQ8
    ret

; --- Secuencia 1: Auto Fantástico (Izq) ---
EXEC_SEQ1:
    out PORTD, LED_PATTERN
    lsl LED_PATTERN
    brne FIN_PASO
    ldi LED_PATTERN, 0x01
FIN_PASO:
    ret

; --- Secuencia 2: Alternado Par / Impar ---
EXEC_SEQ2:
    cpi LED_PATTERN, 0xAA
    breq SET_55
    ldi LED_PATTERN, 0xAA
    rjmp WRITE_SEQ
SET_55:
    ldi LED_PATTERN, 0x55
WRITE_SEQ:
    out PORTD, LED_PATTERN
    ret

; --- Secuencia 3: Extremos al Centro ---
EXEC_SEQ3:
    cpi LED_PATTERN, 0x81
    breq S3_STEP2
    cpi LED_PATTERN, 0xC3
    breq S3_STEP3
    cpi LED_PATTERN, 0xE7
    breq S3_STEP4
    ldi LED_PATTERN, 0x81
    rjmp WRITE_SEQ
S3_STEP2:
    ldi LED_PATTERN, 0xC3
    rjmp WRITE_SEQ
S3_STEP3:
    ldi LED_PATTERN, 0xE7
    rjmp WRITE_SEQ
S3_STEP4:
    ldi LED_PATTERN, 0xFF
    rjmp WRITE_SEQ

; --- Secuencia 4: Parpadeo General ---
EXEC_SEQ4:
    com LED_PATTERN
    out PORTD, LED_PATTERN
    ret

; --- Secuencia 5: Auto Fantástico Inverso (Der) ---
EXEC_SEQ5:
    cpi LED_PATTERN, 0x01
    breq S5_INIT
    lsr LED_PATTERN
    out PORTD, LED_PATTERN
    ret
S5_INIT:
    ldi LED_PATTERN, 0x80
    out PORTD, LED_PATTERN
    ret

; --- Secuencia 6: Llenado Progresivo (Barra) ---
EXEC_SEQ6:
    out PORTD, LED_PATTERN
    sec                   ; Set Carry
    rol LED_PATTERN
    brne FIN_S6
    ldi LED_PATTERN, 0x01
FIN_S6:
    ret

; --- Secuencia 7: Centro hacia los Extremos ---
EXEC_SEQ7:
    cpi LED_PATTERN, 0x18
    breq S7_STEP2
    cpi LED_PATTERN, 0x3C
    breq S7_STEP3
    cpi LED_PATTERN, 0x7E
    breq S7_STEP4
    ldi LED_PATTERN, 0x18
    rjmp WRITE_SEQ
S7_STEP2:
    ldi LED_PATTERN, 0x3C
    rjmp WRITE_SEQ
S7_STEP3:
    ldi LED_PATTERN, 0x7E
    rjmp WRITE_SEQ
S7_STEP4:
    ldi LED_PATTERN, 0xFF
    rjmp WRITE_SEQ

; --- Secuencia 8: Bloques de 4 LEDs (Nibbles) ---
EXEC_SEQ8:
    cpi LED_PATTERN, 0xF0
    breq SET_0F
    ldi LED_PATTERN, 0xF0
    rjmp WRITE_SEQ
SET_0F:
    ldi LED_PATTERN, 0x0F
    rjmp WRITE_SEQ


DELAY_20MS:
    ldi DELAY_R1, 80
D20_1:
    ldi DELAY_R2, 200
D20_2:
    dec DELAY_R2
    brne D20_2
    dec DELAY_R1
    brne D20_1
    ret

DELAY_200MS:
    ldi DELAY_R3, 75
D200_3:
    rcall DELAY_20MS
    dec DELAY_R3
    brne D200_3
    ret