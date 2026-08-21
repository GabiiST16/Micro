.include "m328pdef.inc"
.org 0x0000
    rjmp inicio 
    ldi r16, HIGH(RAMEND)
    out  SPH, r16
    ldi  r16, LOW(RAMEND)
    out  SPL, r16
    
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
 