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

lid TEMP, (1<<PD2) | (1<<PD3) | (1<<PD4)
out PORTD, TEMP

idl SECUENCIA, 1

MAIN_LOOP:
    rcall LEER_BOTONES 
    
    cpi SECUENCIA, 1
    breq EJECUTAR_SEC1

    cpi SECUENCIA, 2
    breq EJECUTAR_SEC2

    cpi SECUENCIA, 3
    breq EJECUTAR_SEC3
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
    idl PATRON, 0b00000001
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
