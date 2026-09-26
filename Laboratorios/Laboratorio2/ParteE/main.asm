;====================================================================
; PROYECTO: Automatizacion de Puerta de Garaje (Version Blindada)
; ASIGNACION DE PINES:
;   - PORTC (Entradas con Pull-Up interno):
;       PC0 -> BT_ABRIR (Pulsador, activo en 0)
;       PC1 -> BT_CERRAR (Pulsador, activo en 0)
;       PC2 -> S1 Final de Carrera Superior / Abierta (activo en 0)
;       PC3 -> S2 Final de Carrera Inferior / Cerrada (activo en 0)
;   - PORTB (Interrupcion de Seguridad):
;       PB0 -> S3 Sensor de Obstaculos (PCINT0, activo en 0)
;   - PORTD (Salidas):
;       PD4 -> Motor Abrir (o LED)
;       PD5 -> Motor Cerrar (o LED)
;       PD6 -> Alarma Sonora
;====================================================================

.include "m328pdef.inc"

.equ F_CPU = 16000000
.equ BAUD  = 115200
.equ BPS   = 8    ; UBRR = 8 para 115200 baudios @ 16MHz

; Estados de la maquina
.equ ESTADO_CERRADA  = 0
.equ ESTADO_ABRIENDO = 1
.equ ESTADO_ABIERTA  = 2
.equ ESTADO_CERRANDO = 3
.equ ESTADO_DETENIDO = 4

; Registros
.def aux1             = r16
.def aux2             = r17
.def estado_actual    = r18   ; Mantiene el estado 0..4
.def flag_obstaculo   = r19   ; 1 = Requiere reporte USART de obstaculo
.def in_pinc          = r21   ; Lectura limpia de PORTC

; Mascara de seguridad para salidas (PORTD)
.equ MASK_SALIDAS     = (1<<PD4) | (1<<PD5) | (1<<PD6)

;====================================================================
; TABLA DE VECTORES DE INTERRUPCION
;====================================================================
.cseg
.org 0x0000
    rjmp RESET
.org PCI0addr                 ; Vector PCINT0_vect (PB0)
    rjmp ISR_PCINT0

;====================================================================
; INICIALIZACION
;====================================================================
RESET:
    ; 1. Stack Pointer
    ldi aux1, HIGH(RAMEND)
    out SPH, aux1
    ldi aux1, LOW(RAMEND)
    out SPL, aux1

    ; 2. Configurar Entradas PORTC: PC0..PC3 con Pull-ups
    clr aux1
    out DDRC, aux1
    ldi aux1, 0b00001111
    out PORTC, aux1

    ; 3. Configurar Entrada de Interrupcion PORTB: PB0 con Pull-up
    cbi DDRB, PB0
    sbi PORTB, PB0

    ; 4. Configurar Salidas PORTD: PD4, PD5, PD6
    in aux1, DDRD
    ori aux1, MASK_SALIDAS
    out DDRD, aux1
    
    ; Salidas apagadas al iniciar
    in aux1, PORTD
    andi aux1, ~MASK_SALIDAS
    out PORTD, aux1

    ; 5. Inicializar USART
    rcall INICIALIZAR_USART

    ; 6. Configurar Interrupcion Pin Change (PCINT0) en PB0
    lds aux1, PCICR
    ori aux1, (1<<PCIE0)
    sts PCICR, aux1

    lds aux1, PCMSK0
    ori aux1, (1<<PCINT0)
    sts PCMSK0, aux1

    ; 7. Estado inicial
    clr flag_obstaculo
    ldi estado_actual, ESTADO_CERRADA

    sei                       ; Habilitar interrupciones globales

    ; Mensaje inicial
    ldi zl, LOW(msg_cerrada * 2)
    ldi zh, HIGH(msg_cerrada * 2)
    rcall ENVIAR_TEXTO_USART

;====================================================================
; BUCLE PRINCIPAL
;====================================================================
MAIN_LOOP:
    tst flag_obstaculo
    breq verificar_maquina

    clr flag_obstaculo
    ldi zl, LOW(msg_obstaculo * 2)
    ldi zh, HIGH(msg_obstaculo * 2)
    rcall ENVIAR_TEXTO_USART

    ldi zl, LOW(msg_detenido * 2)
    ldi zh, HIGH(msg_detenido * 2)
    rcall ENVIAR_TEXTO_USART

verificar_maquina:
    rcall EVALUAR_MAQUINA_ESTADOS
    rcall RETARDO_DEBOUNCE
    rjmp MAIN_LOOP

;====================================================================
; RUTINA DE INTERRUPCION (Sensor S3 en PB0) - 100% PROTEGIDA
;====================================================================
ISR_PCINT0:
    push aux1
    in aux1, SREG
    push aux1

    ; Comprobar si PB0 esta en nivel bajo (0 = obstaculo activo)
    sbic PINB, PB0
    rjmp FIN_ISR

    ; Solo actuar si la puerta esta en movimiento
    cpi estado_actual, ESTADO_ABRIENDO
    breq PARADA_SEGURIDAD
    cpi estado_actual, ESTADO_CERRANDO
    breq PARADA_SEGURIDAD
    rjmp FIN_ISR

PARADA_SEGURIDAD:
    ; Detener inmediatamente todos los motores y la alarma
    in aux1, PORTD
    andi aux1, ~MASK_SALIDAS
    out PORTD, aux1

    ldi estado_actual, ESTADO_DETENIDO
    ldi flag_obstaculo, 1

FIN_ISR:
    pop aux1
    out SREG, aux1
    pop aux1
    reti

;====================================================================
; EVALUACION DE LA MAQUINA DE ESTADOS (Bifurcaciones con rjmp seguro)
;====================================================================
EVALUAR_MAQUINA_ESTADOS:
    in in_pinc, PINC          ; Captura instantanea de las entradas

    ; BLINDAJE 1: ¿Ambos finales de carrera activos al mismo tiempo?
    sbrc in_pinc, PC2
    rjmp TEST_ESTADOS
    sbrc in_pinc, PC3
    rjmp TEST_ESTADOS
    ; Si ambos estan en 0 -> Error de sensores, forzar apagado
    in aux1, PORTD
    andi aux1, ~MASK_SALIDAS
    out PORTD, aux1
    ret

TEST_ESTADOS:
    cpi estado_actual, ESTADO_CERRADA
    brne COMPR_1
    rjmp M_ESTADO_CERRADA

COMPR_1:
    cpi estado_actual, ESTADO_ABRIENDO
    brne COMPR_2
    rjmp M_ESTADO_ABRIENDO

COMPR_2:
    cpi estado_actual, ESTADO_ABIERTA
    brne COMPR_3
    rjmp M_ESTADO_ABIERTA

COMPR_3:
    cpi estado_actual, ESTADO_CERRANDO
    brne COMPR_4
    rjmp M_ESTADO_CERRANDO

COMPR_4:
    cpi estado_actual, ESTADO_DETENIDO
    brne FIN_EVAL
    rjmp M_ESTADO_DETENIDO

FIN_EVAL:
    ret

; --- ESTADO 0: PUERTA CERRADA ---
M_ESTADO_CERRADA:
    ; Si ambos botones estan apretados a la vez, no hacer nada
    rcall CHEQUEAR_BOTONES_SIMULTANEOS
    brcs RET_CERRADA

    ; Si presiona ABRIR (PC0=0)
    sbrs in_pinc, PC0
    rjmp INICIAR_APERTURA
RET_CERRADA:
    ret

INICIAR_APERTURA:
    ; Si ya esta en final de carrera S1 (PC2=0), no abrir
    sbis PINC, PC2
    ret

    ; Interbloqueo seguro: apagar MotorCerrar y activar MotorAbrir + Alarma
    in aux1, PORTD
    andi aux1, ~(1<<PD5)
    ori aux1, (1<<PD4) | (1<<PD6)
    out PORTD, aux1

    ldi estado_actual, ESTADO_ABRIENDO

    ldi zl, LOW(msg_abriendo * 2)
    ldi zh, HIGH(msg_abriendo * 2)
    rcall ENVIAR_TEXTO_USART
    ret

; --- ESTADO 1: PUERTA ABRIENDO ---
M_ESTADO_ABRIENDO:
    ; 1. Comprobar si llego a final de carrera superior S1 (PC2=0)
    sbis PINC, PC2
    rjmp FIN_APERTURA

    ; Si ambos botones estan apretados a la vez, ignorar
    rcall CHEQUEAR_BOTONES_SIMULTANEOS
    brcs RET_ABRIENDO

    ; 2. Comprobar si presiona CERRAR (PC1=0) para invertir marcha
    sbis PINC, PC1
    rjmp INICIAR_CIERRE
RET_ABRIENDO:
    ret

FIN_APERTURA:
    in aux1, PORTD
    andi aux1, ~MASK_SALIDAS
    out PORTD, aux1

    ldi estado_actual, ESTADO_ABIERTA

    ldi zl, LOW(msg_abierta * 2)
    ldi zh, HIGH(msg_abierta * 2)
    rcall ENVIAR_TEXTO_USART
    ret

; --- ESTADO 2: PUERTA ABIERTA ---
M_ESTADO_ABIERTA:
    ; Si ambos botones estan apretados a la vez, ignorar
    rcall CHEQUEAR_BOTONES_SIMULTANEOS
    brcs RET_ABIERTA

    ; Si presiona CERRAR (PC1=0)
    sbrs in_pinc, PC1
    rjmp INICIAR_CIERRE
RET_ABIERTA:
    ret

INICIAR_CIERRE:
    ; Si ya esta en final de carrera S2 (PC3=0), no cerrar
    sbis PINC, PC3
    ret

    ; Interbloqueo seguro: apagar MotorAbrir y activar MotorCerrar + Alarma
    in aux1, PORTD
    andi aux1, ~(1<<PD4)
    ori aux1, (1<<PD5) | (1<<PD6)
    out PORTD, aux1

    ldi estado_actual, ESTADO_CERRANDO

    ldi zl, LOW(msg_cerrando * 2)
    ldi zh, HIGH(msg_cerrando * 2)
    rcall ENVIAR_TEXTO_USART
    ret

; --- ESTADO 3: PUERTA CERRANDO ---
M_ESTADO_CERRANDO:
    ; 1. Comprobar si llego a final de carrera inferior S2 (PC3=0)
    sbis PINC, PC3
    rjmp FIN_CIERRE

    ; Si ambos botones estan apretados a la vez, ignorar
    rcall CHEQUEAR_BOTONES_SIMULTANEOS
    brcs RET_CERRANDO

    ; 2. Comprobar si presiona ABRIR (PC0=0) para invertir marcha
    sbis PINC, PC0
    rjmp INICIAR_APERTURA
RET_CERRANDO:
    ret

FIN_CIERRE:
    in aux1, PORTD
    andi aux1, ~MASK_SALIDAS
    out PORTD, aux1

    ldi estado_actual, ESTADO_CERRADA

    ldi zl, LOW(msg_cerrada * 2)
    ldi zh, HIGH(msg_cerrada * 2)
    rcall ENVIAR_TEXTO_USART
    ret

; --- ESTADO 4: DETENIDO POR SEGURIDAD ---
M_ESTADO_DETENIDO:
    ; Si el sensor de obstaculo sigue retenido (PB0=0), no permitir arrancar
    sbis PINB, PB0
    ret

    ; Si ambos botones estan apretados a la vez, ignorar
    rcall CHEQUEAR_BOTONES_SIMULTANEOS
    brcs RET_DETENIDO

    ; Salir con ABRIR o CERRAR
    sbis PINC, PC0
    rjmp INICIAR_APERTURA

    sbis PINC, PC1
    rjmp INICIAR_CIERRE
RET_DETENIDO:
    ret

;====================================================================
; SUBRUTINA: DETECCION DE BOTONES SIMULTANEOS
; Devuelve Carry = 1 si ambos botones (PC0 y PC1) estan en 0
;====================================================================
CHEQUEAR_BOTONES_SIMULTANEOS:
    sbrc in_pinc, PC0
    rjmp BOTONES_OK           ; PC0 no esta presionado
    sbrc in_pinc, PC1
    rjmp BOTONES_OK           ; PC1 no esta presionado
    sec                       ; Ambos en 0 -> conflicto
    ret
BOTONES_OK:
    clc                       ; Sin conflicto
    ret

;====================================================================
; COMUNICACION USART
;====================================================================
INICIALIZAR_USART:
    ldi aux1, HIGH(BPS)
    sts UBRR0H, aux1
    ldi aux1, LOW(BPS)
    sts UBRR0L, aux1

    ldi aux1, (1<<RXEN0) | (1<<TXEN0)
    sts UCSR0B, aux1

    ldi aux1, (1<<UCSZ01) | (1<<UCSZ00)
    sts UCSR0C, aux1
    ret

ENVIAR_CARACTER:
    lds aux2, UCSR0A
    sbrs aux2, UDRE0
    rjmp ENVIAR_CARACTER
    sts UDR0, aux1
    ret

ENVIAR_TEXTO_USART:
    lpm aux1, z+
    tst aux1
    breq FIN_TEXTO
    rcall ENVIAR_CARACTER
    rjmp ENVIAR_TEXTO_USART
FIN_TEXTO:
    ret

;====================================================================
; RETARDO ANTIRREBOTE
;====================================================================
RETARDO_DEBOUNCE:
    push r24
    push r25
    ldi r25, 120
d_loop1:
    ldi r24, 250
d_loop2:
    dec r24
    brne d_loop2
    dec r25
    brne d_loop1
    pop r25
    pop r24
    ret

;====================================================================
; MENSAJES EN MEMORIA FLASH (Alineados a 16 bits)
;====================================================================
msg_abriendo:  .db "Puerta abriendo.", 13, 10, 0, 0     
msg_abierta:   .db "Puerta abierta.", 13, 10, 0        
msg_cerrando:  .db "Puerta cerrando.", 13, 10, 0, 0     
msg_cerrada:   .db "Puerta cerrada.", 13, 10, 0         
msg_obstaculo: .db "Obstaculo detectado.", 13, 10, 0, 0    
msg_detenido:  .db "Movimiento detenido por seguridad.", 13, 10, 0, 0
