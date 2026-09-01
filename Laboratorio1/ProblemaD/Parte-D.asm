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
    .equ A0 = PC4 ; Pin A4 (Bit 0)
    .equ A1 = PC5 ; Pin A5 (Bit 1)
    .equ A2 = PD2 ; Pin 2  (Bit 2)
    .equ A3 = PD3 ; Pin 3  (Bit 3)

;Operador B
    .equ B0 = PD4 ; Pin 4  (Bit 0)
    .equ B1 = PD5 ; Pin 5  (Bit 1)
    .equ B2 = PD6 ; Pin 6  (Bit 2)
    .equ B3 = PD7 ; Pin 7  (Bit 3)

;Selector
    .equ S0 = PC0 ; Pin A0
    .equ S1 = PC1 ; Pin A1
    .equ S2 = PC2 ; Pin A2

;Salidas
    .equ F0 = PB0    ; Pin 8
    .equ F1 = PB1    ; Pin 9
    .equ F2 = PB2    ; Pin 10
    .equ F3 = PB3    ; Pin 11
    .equ OUT_C = PB4 ; Pin 12
    .equ OUT_Z = PB5 ; Pin 13
    .equ OUT_N = PC3 ; Pin A3

.org 0x0000
    rjmp inicio 

inicio:
    ldi temp, HIGH(RAMEND)
    out SPH, temp
    ldi temp, LOW(RAMEND)
    out SPL, temp

;Puerto D como entradas
    ldi temp, 0x00
    out DDRD, temp
    out PORTD, temp

;Puerto B como salidas
    ldi temp, 0b00111111
    out DDRB, temp
    ldi temp, 0x00
    out PORTB, temp

;Puerto C: PC3 salida (N), demás entradas
    ldi temp, 0b00001000
    out DDRC, temp
    ldi temp, 0x00
    out PORTC, temp

main_loop: 
    ; 1. Leer B (PD4..PD7) y A2, A3 (PD2, PD3)
    in temp, PIND
    mov B, temp
    andi B, 0xF0
    swap B              ; B listo (bits 0..3)

    mov A, temp
    andi A, 0x0C        ; A tiene listos los bits 2 y 3

    ; 2. Leer Selector S (PC0..PC2) y A0, A1 (PC4, PC5)
    in temp, PINC
    mov S, temp
    andi S, 0x07        ; S listo (bits 0..2)

    sbrc temp, 4        ; Si A4 (PC4) está en 1 -> bit 0 de A = 1
    ori A, 0x01
    sbrc temp, 5        ; Si A5 (PC5) está en 1 -> bit 1 de A = 1
    ori A, 0x02

    rcall selector
    
    ; 3. Salida a puertos
    out PORTB, F
    sbrc C_flag, 0
    sbi PORTB, OUT_C
    sbrc Z_flag, 0
    sbi PORTB, OUT_Z
    cbi PORTC, OUT_N
    sbrc N_flag, 0
    sbi PORTC, OUT_N

    rjmp main_loop

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
    clr F
    clr C_flag
    rjmp end_operacion

op_resta: ; A - B
    mov F, A
    sub F, B
    ldi C_flag, 0
    brcc resta_sin_carry
    ldi C_flag, 1
resta_sin_carry:
    rjmp end_operacion

op_suma: ; A + B
    mov F, A 
    add F, B
    ldi C_flag, 0
    sbrc F, 4
    ldi C_flag, 1
    rjmp end_operacion

op_xor: ; A xor B
    mov F, A
    eor F, B
    clr C_flag
    rjmp end_operacion

op_and: ; A and B
    mov F, A
    and F, B
    clr C_flag
    rjmp end_operacion

op_or: ; A or B
    mov F, A 
    or F, B
    clr C_flag
    rjmp end_operacion

op_shl: ; A << 1
    mov F, A
    lsl F 
    ldi C_flag, 0 
    sbrc F, 4 
    ldi C_flag, 1 
    rjmp end_operacion

op_inc: ; A + 1
    mov F, A 
    inc F
    ldi C_flag, 0
    sbrc F, 4
    ldi C_flag, 1
    rjmp end_operacion

end_operacion:
    andi F, 0x0F

    ldi Z_flag, 0
    brne sin_zero
    ldi Z_flag, 1
sin_zero:

    ldi N_flag, 0
    sbrc F, 3
    ldi N_flag, 1
    
    ret
