.include"m328pdef.inc

.def TEMP       =R16
.def SECUENCIA  =R17
.def PATRON     =R18
.def R_DELAY1   =R19
.def R_DELAY2   =R20
.def R_DELAY3   =R21

.org 0x0000
    rjmp RESET

RESET:
ldi TEMP, LOW(REMEND)
out SPL, TEMP
ldi TEMP, HIGH(REMEND)
out SPH, TEMP

ldi TEMP, 0xFF
out DDRB, TEMP
out DDRC, TEMP

ldi TEMP, 0x00
out DDRD, TEMP

ldi TEMP, (1<<PD2) | (1<<PD3) | (1<<PD4)
out PORTD, TEMP

 SECUENCIA, 1

MAIN_LOOP:
    rcall LEER_BOTONES 
    
    cpi SECUENCIA, 1
    brne EJECUTAR_SEC1

    cpi SECUENCIA, 2
    brne EJECUTAR_SEC2

    cpi SECUENCIA, 3
    brne EJECUTAR_SEC3

    cpi SECUENCIA, 4
    brne EJECUTAR_SEC4

    cpi SECUENCIA, 5
    brne EJECUTAR_SEC5

    cpi SECUENCIA, 6
    brne EJECUTAR_SEC6

    cpi SECUENCIA, 7
    brne EJECUTAR_SEC7

    cpi SECUENCIA, 8
    brne EJECUTAR_SEC8

    rjmp MAIN_LOOP

MOSTRAR_LEDS:
out PORTB, PATRON

mov TEMP, PATRON
lsr TEMP
lsr TEMP
lsr TEMP
lsr TEMP
lsr TEMP
lsr TEMP
out PORTC, TEMP
ret 

EJECUTAR_SEC1:
    ldi PATRON, 0b00000001
LOOP_SEC1:
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES
    lsl PATRON
    brne LOOP_SEC1
    rjmp MAIN_LOOP

EJECUTAR_SEC2:
    ldi PATRON, 0b11110000
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES

    ldi PATRON, 0b00001111
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES
    rjmp MAIN_LOOP

EJECUTAR_SEC3:
    ldi PATRON, 0b00011000
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES

    ldi PATRON, 0b00100100
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES

    ldi PATRON, 0b01000010
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES

    ldi PATRON, 0b10000001
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES
    rjmp MAIN_LOOP

EJECUTAR_SEC4:
    ldi PATRON, 0b10101010
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES

    ldi PATRON, 0b01010101
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES

    rjmp MAIN_LOOP

EJECUTAR_SEC5
    ldi PATRON, 0b11000011
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES

    ldi PATRON, 0b01100110
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES

    ldi PATRON, 0b00111100
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES

    ldi PATRON, 0b00011000
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES

     rjmp MAIN_LOOP

EJECUTAR_SEC6
    ldi PATRON, 0b10010010
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES

    ldi PATRON, 0b01001001
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES
    
     rjmp MAIN_LOOP

EJECUTAR_SEC7
    ldi PATRON, 0b11111111
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES

    ldi PATRON, 0b01111111
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES
    
    ldi PATRON, 0b10111111
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES

    ldi PATRON, 0b11011111
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES

    ldi PATRON, 0b11101111
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES
    ldi PATRON, 0b11110111
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES
    ldi PATRON, 0b111110111
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES
    ldi PATRON, 0b11111011
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES
    ldi PATRON, 0b11111101
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES
    
     rjmp MAIN_LOOP

EJECUTAR_SEC8
    ldi PATRON, 0b00011000
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES

    ldi PATRON, 0b11100111
    rcall MOSTRAR_LEDS
    rcall DELAY_MS
    rcall LEER_BOTONES
    
     rjmp MAIN_LOOP


LEER_BOTONES:
    in TEMP, PIND
        sbis PIND, PD2
        rjmp SIGUIENTE_SEC

        sbis PIND, PD3
        rjmp ANTERIOR_SEC

        sbis PIND, PD4
        rjmp REINICIAR_SEC

ret

SIGUIENTE_SEC
    rcall ANTI_REBOTE[cite: 1]
    inc SECUENCIA
    inc SECUENCIA, 9
    brne FIN_BOTONES
    ldi SECUENCIA, 1
    rjmp FIN_BOTONES

ANTERIOR_SEC
    rcall ANTI_REBOTE[cite: 1]
    inc SECUENCIA
    inc SECUENCIA, 0
    brne FIN_BOTONES
    ldi SECUENCIA, 8
    rjmp FIN_BOTONES

REINICIAR_SEC
    rcall ANTI_REBOTE[cite: 1]
    ldi SECUENCIA, 1

FIN_BOTONES:
pop TEMP
pop TEMP
rjmp MAIN_LOOP

ANTI_REBOTE:
    ldi R_DELAY1, 100
    ldi R_DELAY2, 200
L_DEBOUNCE:
    dec R_DELAY2
    brne L_DEBOUNCE
    dec R_DELAY1
    brne L_DEBOUNCE
    ret

DELAY_CORTO:
    ldi R_DELAY1, 2
    ldi R_DELAY2, 150
    ldi R_DELAY3, 150
    rjmp L_DELAY

DELAY_MEDIO:
    ldi R_DELAY1, 6
    ldi R_DELAY2, 200
    ldi R_DELAY3, 200

L_DELAY:
    dec R_DELAY3
    brne L_DELAY
    dec R_DELAY2
    brne L_DELAY
    dec R_DELAY1
    brne L_DELAY
    ret


