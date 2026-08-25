.include "m328pdef.inc"

    
    .def temp = r16
    .def A = r17
    .def B = r18
    .def S = r19
    .def F = r20
    .def C_flag = r21
    .def Z_flag = r22
    .def N_flag = r23
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
    ldi temp, 0b00001000
    out DDRC, temp
    ldi temp, 0x00
    out PORTC, temp


main_loop: 
;Lectura constante
    in temp, PIND ;Lee los 8 pines de D (A y B juntos)
    mov A, temp ;Guarda el valor en A
    andi A, 0x0F ;Toma solo los 4 pines de A
    mov B, temp ;Guarda el valor en B
    andi B, 0xF0 ;Toma solo los 4 pines de B
    swap B ;Invierte B para tenerlo en orden
    in temp, PINC ;Lee los pines de C
    mov S, temp ;Guarda el valor en S
    andi S, 0x07 ;Toma solo los 3 pines de S
    rcall selector


selector:
    cpi S, 0
    breq op_clear
    cpi S, 1 
    breq op_resta
    cpi S, 2
    breq op_suma
    cpi S, 3 
    breq op_xor
    cpi S, 4 
    breq op_and
    cpi S, 5
    breq op_or
    cpi S, 6
    breq op_shl
    cpi S, 7
    breq op_inc
    ret 

op_clear: 

