.include "m328pdef.inc"

; Configuración USART
.equ baud = 9600
.equ F_CPU = 16000000
.equ bps = 103        ; (F_CPU/(16*baud))-1

; Pulsadores (Puerto B)
.equ BTN0 = PB0       ; Pin 8  (Bit 0)
.equ BTN1 = PB1       ; Pin 9  (Bit 1)
.equ BTN2 = PB2       ; Pin 10 (Bit 2)
.equ BTN_ENV = PB3    ; Pin 11 (Botón de enviar)

.def temp = r16
.def dato = r17

.org 0x0000
    rjmp inicio

inicio:
    ldi temp, HIGH(RAMEND)
    out SPH, temp
    ldi temp, LOW(RAMEND)
    out SPL, temp

; Puerto D: PD1 (TX) como salida
    ldi temp, 0b00000010
    out DDRD, temp

; Puerto B como entradas (botones) y PB5 (LED) como salida
    ldi temp, 0b00100000    ; PB5 como salida, resto como entrada
    out DDRB, temp
    ldi temp, 0b00001111    ; Pull-ups en PB0-PB3, PB5 arranca en 0 (apagado)
    out PORTB, temp

    rcall initUART

    rjmp main_loop

main_loop:
    sbic PINB, BTN_ENV      ; ¿PB3 = 0 (presionado)? salta
    rjmp main_loop
    rcall delay_debounce
    sbic PINB, BTN_ENV      ; confirmar que sigue presionado
    rjmp main_loop
    
    ; Leer los 3 botones y armar valor
    in dato, PINB
    
    ; DEBUG: Encender LED 13 (PB5) para confirmar que se detectó el botón
    sbi PORTB, 5

    com dato                ; invertir (presionado = 1)
    andi dato, 0x07         ; solo bits 0, 1, 2
    rcall enviarUART
    
    ; Esperar que suelte BTN_ENV
esperar_soltar:
    sbis PINB, BTN_ENV      ; si está en 1 (suelto), salta el rjmp
    rjmp esperar_soltar
    
    ; DEBUG: Apagar LED 13
    cbi PORTB, 5
    
    rcall delay_debounce
    rjmp main_loop

; Inicialización de USART (solo TX)
initUART:
    ldi temp, HIGH(bps)
    sts UBRR0H, temp
    ldi temp, LOW(bps)
    sts UBRR0L, temp
    ldi temp, (1<<TXEN0)      ; Solo TX habilitado (MCU1 es transmisor)
    sts UCSR0B, temp
    ldi temp, (1<<UCSZ01)|(1<<UCSZ00)  ; 8 bits datos, 1 stop, sin paridad
    sts UCSR0C, temp
    ret
    
enviarUART:
    lds temp, UCSR0A
    sbrs temp, UDRE0        ; ¿buffer vacío? salta
    rjmp enviarUART
    sts UDR0, dato
    ret

; Antirrebote (~20ms a 16MHz)
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
