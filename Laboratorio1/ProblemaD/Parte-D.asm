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
    
    ;Salida a puertos
    out PORTB, F ; F en PB0-PB3 (limpia PB4-PB5)
    sbrc C_flag, 0
    sbi PORTB, 4 ; C en PB4
    sbrc Z_flag, 0
    sbi PORTB, 5 ; Z en PB5
    cbi PORTC, 3 ; Limpiar N primero
    sbrc N_flag, 0
    sbi PORTC, 3 ; N en PC3

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
    clr F ; Deja F en 0
    clr C_flag ; No hay carry
    rjmp end_operacion

op_resta: ; A - B
    mov F, A
    sub F, B ; F = A - B (8 bits), SREG.C = borrow

    ; Leo el carry del SREG (LDI y BRCC no modifican SREG)
    ldi C_flag, 0 ; Dejo el C_flag en 0
    brcc resta_sin_carry ; Si C=0 (no hay borrow), salta de linea
    ldi C_flag, 1       ; No hubo salto de linea entonces pongo C en 1
resta_sin_carry:
    rjmp end_operacion

op_suma: ; A + B
    mov F, A 
    add F, B
    ; En suma de 4 bits, el carry queda en el bit 4 de F
    ldi C_flag, 0 ; Dejo el C_flag en 0
    sbrc F, 4 ; Si el bit 4 de F es 0, saltea la siguiente linea
    ldi C_flag, 1 ; No hubo salto de linea entonces pongo C en 1
    rjmp end_operacion

op_xor: ; A xor B
    mov F, A
    eor F, B
    clr C_flag ; No hay carry en xor
    rjmp end_operacion

op_and: ; A and B
    mov F, A
    and F, B
    clr C_flag ; No hay carry en and
    rjmp end_operacion

op_or: ; A or B
    mov F, A 
    or F, B
    clr C_flag ; No hay carry en or
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


; Calculo de Z, N y retorno
end_operacion:
    ; Dejo solo el resultado de 4 bits
    andi F, 0x0F        ; ANDI pone Z=1 en SREG si F queda en 0

    ; Z flag: basada en resultado de 4 bits
    ldi Z_flag, 0 ; Dejo el Z_flag en 0
    brne sin_zero  ; Si el SREG.Z es 1 (F != 0), salta de linea
    ldi Z_flag, 1 ; No hubo salto de linea entonces pongo Z en 1
sin_zero:

    ; N flag: signo en bit 3
    ldi N_flag, 0 ; Dejo el N_flag en 0
    sbrc F, 3 ; Si bit 3 de F es 0, saltea
    ldi N_flag, 1 ; No hubo salto de linea entonces pongo N en 1
    
    ret ; Vuelvo a main_loop
