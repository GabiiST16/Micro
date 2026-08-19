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

; 'Motores'
.equ motorR = PC2
.equ motorL = PD1

; Inputs

.equ Pi = PC3
.equ Psc = PC4
.equ Ss = PC5
.equ Sf = PD0

.def Inicio = R17
.def Selector = R16
.def CuentaLavado = R21

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
sbi DDRC, motorR
sbi DDRD, motorL

; Entradas

cbi DDRC, Pi
cbi DDRC, Psc
cbi DDRC, Ss
cbi DDRD, Sf

sbi PORTC, Psc
ldi Selector, 1
ldi Inicio, 1
ldi CuentaLavado, 5

; Variables para el delay de 1s
ldi  r18, 82
ldi  r19, 43
ldi  r20, 255

main_loop:
    cpi Inicio, 1
    breq Lavado_listo
    rjmp Continuar_inicio
Lavado_listo:
    sbi PORTB, L1
    rjmp Continuar_inicio

Continuar_inicio:
    cbi PORTC, motorR
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
    rjmp Lavado
    rjmp leer_selector

encender_Lm:
    sbi PORTC, Lm
    rjmp Lavado
    rjmp leer_selector

encender_Lp:
    sbi PORTC, Lp
    rjmp Lavado
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

Lavado:
    cbi PORTB, Ll
    cbi PORTC, Lm
    cbi PORTC, Lp

    cpi Selector, 1
    breq LavadoL

    cpi Selector, 2
    breq LavadoM

    cpi Selector, 3
    breq LavadoP

LavadoL:
    sbi PORTB, L2
    sbi PORTC, motorR
    rjmp delay_1s
    rjmp delay_1s
    cbi PORTC, motorR
    rjmp delay_1s
    dec CuentaLavado
    brne Centrifugado
    rjmp Lavado
LavadoM:
    sbi PORTB, L2
    sbi PORTC, motorR
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    cbi PORTC, motorR
    rjmp delay_1s
    rjmp delay_1s
    dec CuentaLavado
    brne Centrifugado
    rjmp Lavado
LavadoP:
    sbi PORTB, L2
    sbi PORTC, motorR
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    cbi PORTC, motorR
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    dec CuentaLavado
    brne Centrifugado
    rjmp Lavado

Centrifugado:
    cbi PORTB, L2
    sbi PORTB, L3
    sbi PORTC, motorR

    cpi Selector, 1
    breq CenL

    cpi Selector, 2
    breq CenM

    cpi Selector, 3
    breq CenP

CenL:
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    cbi PORTC, motorR
    rjmp Secador
CenM:
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    cbi PORTC, motorR
    rjmp Secador
CenP:
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    cbi PORTC, motorR
    rjmp Secador

Secador:
    cbi PORTB, L3
    sbi PORTB, L4

    cpi Selector, 1
    breq SecL

    cpi Selector, 2
    breq SecM

    cpi Selector, 3
    breq SecP
SecL:
    sbi PORTC, motorR
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    cbi PORTC, motorR
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    sbi PORTD, motorL
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp Finalizar
SecM:
    sbi PORTC, motorR
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    cbi PORTC, motorR
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    sbi PORTD, motorL
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp Finalizar
SecP:
    sbi PORTC, motorR
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    cbi PORTC, motorR
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    sbi PORTD, motorL
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp Finalizar

Finalizar:
    cbi PORTD, motorL
    cbi PORTB, L4
    sbi PORTB, L5
    rjmp delay_1s
    rjmp delay_1s
    rjmp delay_1s
    rjmp main_loop

delay_1s:
    dec  r20
    brne delay_1s
    dec  r19
    brne delay_1s
    dec  r18
    brne delay_1s
    ret