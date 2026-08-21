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
sbi PORTC, Pi
sbi PORTC, Ss
ldi Selector, 1
ldi Inicio, 0
ldi CuentaLavado, 5

; Variables para el delay de 1s
ldi  r18, 82
ldi  r19, 43
ldi  r20, 255

main_loop:
    cpi Inicio, 1
    sbi PORTB, L1
    breq Lavado_listo
    rjmp Continuar_inicio
Lavado_listo:
    rjmp Lavado

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
    rjmp leer_selector

encender_Lm:
    sbi PORTC, Lm
    rjmp leer_selector

encender_Lp:
    sbi PORTC, Lp
    rjmp leer_selector

leer_selector:
    sbic PINC, Pi
    rjmp chequear_Psc

    sbic PINC, Ss
    rjmp main_loop
    ldi Inicio, 1

chequear_Psc:
    sbic PINC, Psc
    rjmp main_loop
    inc Selector
    cpi Selector, 4
    brne esperar_soltar
    ldi Selector, 1

esperar_soltar_Pi:
    sbis PINC, Pi
    rjmp esperar_soltar_Pi
    rjmp main_loop
esperar_soltar:
    sbis PINC, Psc
    rjmp esperar_soltar
    rjmp main_loop

espera_lleno:
    sbic PIND, Sf
    rjmp espera_lleno
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
    cbi PORTB, L1
    sbi PORTB, L2
    sbi PORTC, motorR
    rcall delay_1s
    rcall delay_1s
    cbi PORTC, motorR
    rcall delay_1s
    dec CuentaLavado
    breq Centrifugado
    rjmp Lavado
LavadoM:
    cbi PORTB, L1
    sbi PORTB, L2
    sbi PORTC, motorR
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    cbi PORTC, motorR
    rcall delay_1s
    rcall delay_1s
    dec CuentaLavado
    breq Centrifugado
    rjmp Lavado
LavadoP:
    cbi PORTB, L1
    sbi PORTB, L2
    sbi PORTC, motorR
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    cbi PORTC, motorR
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    dec CuentaLavado
    breq Centrifugado
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
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    cbi PORTC, motorR
    rjmp Secador
CenM:
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    cbi PORTC, motorR
    rjmp Secador
CenP:
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
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
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    cbi PORTC, motorR
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    sbi PORTD, motorL
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rjmp Finalizar
SecM:
    sbi PORTC, motorR
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    cbi PORTC, motorR
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    sbi PORTD, motorL
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rjmp Finalizar
SecP:
    sbi PORTC, motorR
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    cbi PORTC, motorR
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    sbi PORTD, motorL
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    rjmp Finalizar

Finalizar:
    cbi PORTD, motorL
    cbi PORTB, L4
    sbi PORTB, L5
    rcall delay_1s
    rcall delay_1s
    rcall delay_1s
    ldi Inicio, 0
    ldi CuentaLavado, 5
    rjmp main_loop

delay_1s:
    dec  r20
    brne delay_1s
    dec  r19
    brne delay_1s
    dec  r18
    brne delay_1s
    ret