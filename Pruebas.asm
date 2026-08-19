.include "m328pdef.inc"

; Leds de Estado
.equ L1 = PB0
.equ L2 = PB1
.equ L3 = PB2
.equ L4 = PB3
.equ L5 = PB4

; Leds de Carga
.equ Ll = PB5
.equ Lm = PC0
.equ Lp = PC1

; Motor
.equ motor = PC2

; Inputs

.equ Pi = PC3
.equ Psc = PC4
.equ Ss = PC5
.equ Sf = PD0

.def Inicio = R17
.def Selector = R16

.cseg
.org 0x0000

; Salidas

sbi DDRB, L1
sbi DDRB, L2
sbi DDRB, L3
sbi DDRB, L4
sbi DDRB, L5

sbi DDRB, Ll
sbi DDRC, Lm
sbi DDRC, Lp
sbi DDRC, motor

; Entradas

cbi DDRC, Pi
cbi DDRC, Psc
cbi DDRC, Ss
cbi DDRD, Sf

sbi PORTC, Psc
ldi Selector, 1
ldi Inicio, 1

main_loop:
    cpi Inicio, 1
    breq Lavado_listo
    rjmp Continuar_inicio
Lavado_listo:
    sbi PORTB, L1
    rjmp Continuar_inicio

Continuar_inicio:
    cbi PORTB, Ll
    cbi PORTC, Lm
    cbi PORTC, Lp

    cpi Selector, 1
    breq encender_Ll

    cpi Selector, 2
    breq encender_Lm

    cpi Selector, 3
    breq encender_Lp
    
    rjmp leer_selector

encender_Ll:
    sbi PORTB, Ll
    rjmp leer_selector

encender_Lm:
    sbi PORTC, Lm
    rjmp leer_selector

encender_Lp:
    sbi PORTC, Lp
    rjmp leer_selector

leer_selector:
    sbic PINC, Psc
    rjmp main_loop

    inc Selector
    cpi Selector, 4
    brne esperar_soltar
    ldi Selector, 1

esperar_soltar:
    sbis PINC, Psc
    rjmp esperar_soltar
    rjmp main_loop