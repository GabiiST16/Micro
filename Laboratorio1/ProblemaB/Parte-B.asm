.include "m328pdef.inc"

    .def temp = r16
    .def cont = r17

.org 0x0000
    rjmp inicio 

inicio:

    ldi temp, HIGH(RAMEND)
    out  SPH, temp
    ldi  temp, LOW(RAMEND)
    out  SPL, temp

;Puerto D como salidas (display)
    ldi temp, 0xFF
    out DDRD, temp

;Puerto B como entradas (botones)
    ldi temp, 0x00
    out DDRB, temp
    ldi temp, 0x07
    out PORTB, temp     ;pull-ups en PB0, PB1, PB2

    ldi cont, 0
    rcall mostrar_display ;mostrar 0 al inicio

main_loop:
    sbic PINB, 0 ;si PB0 esta en 0 (presionado) salta
    rjmp boton_dec
    rjmp boton_inc

boton_inc:
    ;Antirrebote
    rcall delay_debounce
    sbic PINB, 0 ;confirma que sigue presionado
    rjmp main_loop

    ;Incrementar contador
    inc cont
    cpi cont, 10
    brne no_wrap_inc
    ldi cont, 0         ;si llego a 10 vuelve a 0
no_wrap_inc:
    rcall mostrar_display

    ;Esperar a que suelte el boton
esperar_soltar_inc:
    sbis PINB, 0 ;si PB0 esta en 1 (suelto) salta
    rjmp esperar_soltar_inc
    rcall delay_debounce
    rjmp main_loop

boton_dec:
    sbic PINB, 1 ;si PB1 esta en 0 (presionado) salta
    rjmp boton_reset

    ;Antirrebote
    rcall delay_debounce
    sbic PINB, 1
    rjmp boton_reset

    ;Decrementar contador
    cpi cont, 0
    brne no_wrap_dec
    ldi cont, 10        ;si esta en 0, cargar 10 para que al decrementar quede en 9
no_wrap_dec:
    dec cont
    rcall mostrar_display

    ;Esperar a que suelte el boton
esperar_soltar_dec:
    sbis PINB, 1
    rjmp esperar_soltar_dec
    rcall delay_debounce
    rjmp main_loop

boton_reset:
    sbic PINB, 2 ;si PB2 esta en 0 (presionado) salta
    rjmp main_loop

    ;Antirrebote
    rcall delay_debounce
    sbic PINB, 2
    rjmp main_loop

    ;Reiniciar contador a 0
    ldi cont, 0
    rcall mostrar_display

    ;Esperar a que suelte el boton
esperar_soltar_rst:
    sbis PINB, 2
    rjmp esperar_soltar_rst
    rcall delay_debounce
    rjmp main_loop

;Subrutina: muestra el valor de cont en el display 7 segmentos
;Compara cont con cada digito y carga el patron correspondiente
; Segmentos: gfedcba (bit6 a bit0)
mostrar_display:
    cpi cont, 0
    breq mostrar_0
    cpi cont, 1
    breq mostrar_1
    cpi cont, 2
    breq mostrar_2
    cpi cont, 3
    breq mostrar_3
    cpi cont, 4
    breq mostrar_4
    cpi cont, 5
    breq mostrar_5
    cpi cont, 6
    breq mostrar_6
    cpi cont, 7
    breq mostrar_7
    cpi cont, 8
    breq mostrar_8
    cpi cont, 9
    breq mostrar_9
    ret

mostrar_0:
    ldi temp, 0x3F       ; a,b,c,d,e,f prendidos
    out PORTD, temp
    ret
mostrar_1:
    ldi temp, 0x06       ; b,c prendidos
    out PORTD, temp
    ret
mostrar_2:
    ldi temp, 0x5B       ; a,b,d,e,g prendidos
    out PORTD, temp
    ret
mostrar_3:
    ldi temp, 0x4F       ; a,b,c,d,g prendidos
    out PORTD, temp
    ret
mostrar_4:
    ldi temp, 0x66       ; b,c,f,g prendidos
    out PORTD, temp
    ret
mostrar_5:
    ldi temp, 0x6D       ; a,c,d,f,g prendidos
    out PORTD, temp
    ret
mostrar_6:
    ldi temp, 0x7D       ; a,c,d,e,f,g prendidos
    out PORTD, temp
    ret
mostrar_7:
    ldi temp, 0x07       ; a,b,c prendidos
    out PORTD, temp
    ret
mostrar_8:
    ldi temp, 0x7F       ; todos prendidos
    out PORTD, temp
    ret
mostrar_9:
    ldi temp, 0x6F       ; a,b,c,d,f,g prendidos
    out PORTD, temp
    ret

;Subrutina: delay para antirrebote (~20ms a 16MHz)
delay_debounce:
    ldi r18, 2
deb_l1:
    ldi r19, 210
deb_l2:
    ldi r20, 255
deb_l3:
    dec r20
    brne deb_l3
    dec r19
    brne deb_l2
    dec r18
    brne deb_l1
    ret
