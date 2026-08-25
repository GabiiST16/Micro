.include "m328pdef.inc"

    
    .def temp = r16

;Operador A
    .equ A0 = PD0
    .equ A1 = PD1
    .equ A2 = PD2
    .equ A3 = PD3

;Operador B
    .equ B0 = PD4
    .equ B1 = PD5
    .equ B2 = PD6
    .equ B3 = PD7 

;Selector
    .equ S0 = PC0
    .equ S1 = PC1
    .equ S2 = PC2

;Salidas
    .equ F0 = PB0
    .equ F1 = PB1
    .equ F2 = PB2
    .equ F3 = PB3
    .equ C = PB4
    .equ Z = PB5
    .equ N = PC3

.org 0x0000
    rjmp inicio 

inicio:

    ldi temp, HIGH(RAMEND)
    out  SPH, temp
    ldi  temp, LOW(RAMEND)
    out  SPL, temp

;Puerto D como entradas
    ldi temp, 0x00
    out DDRD, temp
    out PORTD, temp

;Puerto B como salidas
    ldi temp, 0b00111111
    out DDRB, temp
    ldi temp, 0x00
    out PORTB, temp

;Puerto C como salidas y entradas
    ldi temp 0b00111000
    out DDRC, temp
    ldi temp, 0x00
    out PORTC, temp

FUNCIONA?