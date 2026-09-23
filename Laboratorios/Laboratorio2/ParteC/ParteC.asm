.include "m328pdef.inc"

; ------------------------------------------------------------------------------
; CONSTANTES DE CONFIGURACIÓN DE RELOJ Y USART
; ------------------------------------------------------------------------------
.equ F_CPU          = 16000000              ; Frecuencia de reloj: 16 MHz
.equ BAUD           = 9600                  ; Velocidad en baudios
.equ UBRR_VAL       = (F_CPU / (16 * BAUD)) - 1  ; UBRR = 103 (0x0067)

; ------------------------------------------------------------------------------
; MÁSCARAS DE HARDWARE EN PORTD (Pines D2 a D7)
; ------------------------------------------------------------------------------
.equ PIN_SOL_BAJAR  = PD2                   ; Pin D2 (X0)
.equ PIN_SOL_SUBIR  = PD3                   ; Pin D3 (X1)
.equ PIN_MOT_ABAJO  = PD4                   ; Pin D4 (X5: Abajo)
.equ PIN_MOT_ARRIBA = PD5                   ; Pin D5 (X6: Arriba)
.equ PIN_MOT_DER    = PD6                   ; Pin D6 (X7: Derecha)
.equ PIN_MOT_IZQ    = PD7                   ; Pin D7 (X10: Izquierda)

; Movimientos ortogonales directos
.equ DIR_STOP       = 0b00000000
.equ DIR_ABAJO      = (1 << PIN_MOT_ABAJO)
.equ DIR_ARRIBA     = (1 << PIN_MOT_ARRIBA)
.equ DIR_DER        = (1 << PIN_MOT_DER)
.equ DIR_IZQ        = (1 << PIN_MOT_IZQ)

; Movimientos diagonales simultáneos calibrados con el pinout real
.equ DIR_ARR_DER    = (1 << PIN_MOT_ARRIBA) | (1 << PIN_MOT_DER) ; PD5 + PD6
.equ DIR_ARR_IZQ    = (1 << PIN_MOT_ARRIBA) | (1 << PIN_MOT_IZQ) ; PD5 + PD7
.equ DIR_ABJ_DER    = (1 << PIN_MOT_ABAJO)  | (1 << PIN_MOT_DER) ; PD4 + PD6
.equ DIR_ABJ_IZQ    = (1 << PIN_MOT_ABAJO)  | (1 << PIN_MOT_IZQ) ; PD4 + PD7

; Máscaras para aislar registros de motores y solenoides
.equ MASK_MOTORES   = 0b11110000
.equ MASK_SOLENOIDE = 0b00001100
.equ MASK_USART     = 0b00000011

; ------------------------------------------------------------------------------
; CALIBRACIÓN DE VIAJE HACIA EL CENTRO DE LA HOJA A4
; Cada paso = 150 ms
; ------------------------------------------------------------------------------
.equ PASOS_CENTRO_X = 70
.equ PASOS_CENTRO_Y = 50

; ------------------------------------------------------------------------------
; DEFINICIÓN DE REGISTROS DE TRABAJO
; ------------------------------------------------------------------------------
.def temp           = r16                   ; Registro general de trabajo
.def aux            = r17                   ; Registro secundario de trabajo
.def dir_mask       = r18                   ; Máscara de dirección activa
.def dur_pasos      = r19                   ; Duración del trazo (unidades de paso)
.def del_cnt1       = r20                   ; Contador 1 para bucles de retardo
.def del_cnt2       = r21                   ; Contador 2 para bucles de retardo
.def del_cnt3       = r22                   ; Contador 3 para bucles de retardo
.def rx_char        = r23                   ; Carácter recibido por USART

; ==============================================================================
; VECTOR DE RESET
; ==============================================================================
.cseg
.org 0x0000
    rjmp RESET_HANDLER

; ==============================================================================
; INICIALIZACIÓN DEL SISTEMA
; ==============================================================================
RESET_HANDLER:
    ldi temp, HIGH(RAMEND)
    out SPH, temp
    ldi temp, LOW(RAMEND)
    out SPL, temp

    ldi temp, 0b11111110
    out DDRD, temp

    in temp, PORTD
    andi temp, MASK_USART
    out PORTD, temp

    rcall USART_INIT

    rcall SUBIR_LAPIZ
    rcall DELAY_500MS

    rcall MOSTRAR_MENU

; ==============================================================================
; BUCLE PRINCIPAL
; ==============================================================================
MAIN_LOOP:
    rcall USART_RX      ; Esperar comando del usuario
    mov rx_char, temp

    cpi rx_char, '1'
    breq CMD_TRIANGULO

    cpi rx_char, '2'
    breq CMD_CIRCULO

    cpi rx_char, '3'
    breq CMD_PENTAGRAMA

    cpi rx_char, '4'
    breq CMD_LIBRE

    cpi rx_char, 'P'
    breq CMD_POKEMON
    cpi rx_char, 'p'
    breq CMD_POKEMON

    cpi rx_char, 'T'
    breq CMD_TODAS
    cpi rx_char, 't'
    breq CMD_TODAS

    cpi rx_char, 'M'
    breq CMD_PERSONALIZADO
    cpi rx_char, 'm'
    breq CMD_PERSONALIZADO

    rjmp MAIN_LOOP

CMD_TRIANGULO:
    rcall VIAJAR_AL_CENTRO
    rcall DIBUJAR_TRIANGULO
    rcall MOSTRAR_MENU
    rjmp MAIN_LOOP

CMD_CIRCULO:
    rcall VIAJAR_AL_CENTRO
    rcall DIBUJAR_CIRCULO
    rcall MOSTRAR_MENU
    rjmp MAIN_LOOP

CMD_PENTAGRAMA:
    rcall VIAJAR_AL_CENTRO
    rcall DIBUJAR_PENTAGRAMA
    rcall MOSTRAR_MENU
    rjmp MAIN_LOOP

CMD_LIBRE:
    rcall VIAJAR_AL_CENTRO
    rcall DIBUJAR_LIBRE_CASA
    rcall MOSTRAR_MENU
    rjmp MAIN_LOOP

CMD_POKEMON:
    rcall VIAJAR_AL_CENTRO
    rcall DIBUJAR_CUBONE
    rcall MOSTRAR_MENU
    rjmp MAIN_LOOP

CMD_TODAS:
    rcall VIAJAR_AL_CENTRO
    rcall DIBUJAR_TODAS
    rcall MOSTRAR_MENU
    rjmp MAIN_LOOP

CMD_PERSONALIZADO:
    rcall MODO_PERSONALIZADO
    rcall MOSTRAR_MENU
    rjmp MAIN_LOOP

; ==============================================================================
; SUBRUTINAS DE CONTROL
; ==============================================================================

; ------------------------------------------------------------------------------
; SUBIR_LAPIZ
; ------------------------------------------------------------------------------
SUBIR_LAPIZ:
    in temp, PORTD
    cbr temp, (1 << PIN_SOL_BAJAR)          ; Apagar D2
    sbr temp, (1 << PIN_SOL_SUBIR)          ; Encender D3
    out PORTD, temp

    ldi del_cnt1, 4
SUBIR_P_LOOP:
    rcall DELAY_10MS
    dec del_cnt1
    brne SUBIR_P_LOOP

    in temp, PORTD
    cbr temp, (1 << PIN_SOL_SUBIR)          ; Apagar pulso
    out PORTD, temp

    rcall DELAY_CONMUTACION
    ret

; ------------------------------------------------------------------------------
; BAJAR_LAPIZ
; ------------------------------------------------------------------------------
BAJAR_LAPIZ:
    in temp, PORTD
    cbr temp, (1 << PIN_SOL_SUBIR)          ; Apagar D3
    sbr temp, (1 << PIN_SOL_BAJAR)          ; Encender D2
    out PORTD, temp

    ldi del_cnt1, 4
BAJAR_P_LOOP:
    rcall DELAY_10MS
    dec del_cnt1
    brne BAJAR_P_LOOP

    in temp, PORTD
    cbr temp, (1 << PIN_SOL_BAJAR)       ; Apagar pulso
    out PORTD, temp

    rcall DELAY_CONMUTACION
    ret

; ------------------------------------------------------------------------------
; PARAR_MOTORES
; ------------------------------------------------------------------------------
PARAR_MOTORES:
    in temp, PORTD
    andi temp, ~MASK_MOTORES
    out PORTD, temp
    rcall DELAY_CONMUTACION
    ret

; ------------------------------------------------------------------------------
; MOVER_TRAZO
; ------------------------------------------------------------------------------
MOVER_TRAZO:
    in temp, PORTD
    andi temp, ~MASK_MOTORES
    or temp, dir_mask
    out PORTD, temp

BUCLE_PASOS:
    tst dur_pasos
    breq FIN_TRAZO
    rcall DELAY_PASO_BASE
    dec dur_pasos
    rjmp BUCLE_PASOS

FIN_TRAZO:
    rcall PARAR_MOTORES
    ret

; ------------------------------------------------------------------------------
; MOVER_TRAZO_HD:
; ------------------------------------------------------------------------------
MOVER_TRAZO_HD:
    in temp, PORTD
    andi temp, ~MASK_MOTORES
    or temp, dir_mask
    out PORTD, temp

BUCLE_HD:
    tst dur_pasos
    breq FIN_HD
    rcall DELAY_PASO_HD
    dec dur_pasos
    rjmp BUCLE_HD

FIN_HD:
    ret

; ------------------------------------------------------------------------------
; EJECUTAR_LUT:
; ------------------------------------------------------------------------------
EJECUTAR_LUT:
    lpm temp, Z+
    lpm dur_pasos, Z+

    cpi temp, 0x00
    breq FIN_LUT

    cpi temp, 0xFF
    breq EJECUTAR_CTRL_LUT

    mov dir_mask, temp
    rcall MOVER_TRAZO_HD
    rjmp EJECUTAR_LUT

EJECUTAR_CTRL_LUT:
    cpi dur_pasos, 0x01
    breq CTRL_BAJAR
    cpi dur_pasos, 0x02
    breq CTRL_SUBIR
    cpi dur_pasos, 0x03
    breq CTRL_DELAY
    cpi dur_pasos, 0x04
    breq CTRL_PARAR
    rjmp EJECUTAR_LUT

CTRL_BAJAR:
    rcall BAJAR_LAPIZ
    rjmp EJECUTAR_LUT

CTRL_SUBIR:
    rcall SUBIR_LAPIZ
    rjmp EJECUTAR_LUT

CTRL_DELAY:
    rcall DELAY_250MS
    rjmp EJECUTAR_LUT

CTRL_PARAR:
    rcall PARAR_MOTORES
    rjmp EJECUTAR_LUT

FIN_LUT:
    rcall PARAR_MOTORES
    ret

; ------------------------------------------------------------------------------
; VIAJAR al Centro
; ------------------------------------------------------------------------------
VIAJAR_AL_CENTRO:
    rcall SUBIR_LAPIZ
    rcall DELAY_500MS         ; Espera reposo total del pistón

    ldi ZL, LOW(STR_VIAJE_CENTRO * 2)
    ldi ZH, HIGH(STR_VIAJE_CENTRO * 2)
    rcall USART_PRINT_FLASH

    ; 1. Desplazamiento horizontal hacia la IZQUIERDA (PD7)
    ldi dir_mask, DIR_IZQ
    ldi dur_pasos, PASOS_CENTRO_X
    rcall MOVER_TRAZO

    rcall DELAY_250MS

    ; 2. Desplazamiento vertical hacia ABAJO (PD4)
    ldi dir_mask, DIR_ABAJO
    ldi dur_pasos, PASOS_CENTRO_Y
    rcall MOVER_TRAZO

    rcall DELAY_500MS             ; Estabilización mecánica en el centro
    ret

; ==============================================================================
; RUTINAS DE DIBUJO DE FIGURAS
; ==============================================================================

; ------------------------------------------------------------------------------
; 1. TRIÁNGULO EQUILÁTER
; ------------------------------------------------------------------------------
DIBUJAR_TRIANGULO:
    ldi ZL, LOW(STR_DIB_TRIANGULO * 2)
    ldi ZH, HIGH(STR_DIB_TRIANGULO * 2)
    rcall USART_PRINT_FLASH

    rcall BAJAR_LAPIZ
    rcall DELAY_500MS

    ; Lado 1: Base hacia la IZQUIERDA
    ldi dir_mask, DIR_IZQ
    ldi dur_pasos, 16
    rcall MOVER_TRAZO
    rcall DELAY_250MS

    ; Lado 2: Diagonal arriba-derecha
    ldi dir_mask, DIR_ARR_DER
    ldi dur_pasos, 8
    rcall MOVER_TRAZO
    rcall DELAY_250MS

    ; Lado 3: Diagonal abajo-derecha
    ldi dir_mask, DIR_ABJ_DER
    ldi dur_pasos, 8
    rcall MOVER_TRAZO
    rcall DELAY_250MS

    rcall SUBIR_LAPIZ
    rcall DELAY_500MS
    rcall IMPRIMIR_LISTO
    ret

; ------------------------------------------------------------------------------
; 2. CÍRCULO 64 cortes
; ------------------------------------------------------------------------------
DIBUJAR_CIRCULO:
    ldi ZL, LOW(STR_DIB_CIRCULO * 2)
    ldi ZH, HIGH(STR_DIB_CIRCULO * 2)
    rcall USART_PRINT_FLASH

    ldi ZL, LOW(LUT_CIRCULO * 2)
    ldi ZH, HIGH(LUT_CIRCULO * 2)
    rcall EJECUTAR_LUT

    rcall DELAY_250MS ; Pausa de reposo
    rcall SUBIR_LAPIZ
    rcall DELAY_500MS
    rcall IMPRIMIR_LISTO
    ret

; ------------------------------------------------------------------------------
; 3. ESTRELLA DE 5 PUNTAS
; ------------------------------------------------------------------------------
DIBUJAR_PENTAGRAMA:
    ldi ZL, LOW(STR_DIB_PENTAGRAMA * 2)
    ldi ZH, HIGH(STR_DIB_PENTAGRAMA * 2)
    rcall USART_PRINT_FLASH

    ldi ZL, LOW(LUT_ESTRELLA * 2)
    ldi ZH, HIGH(LUT_ESTRELLA * 2)
    rcall EJECUTAR_LUT

    rcall DELAY_250MS                       ; Pausa de reposo tras micro-pasos HD antes de accionar el piston
    rcall SUBIR_LAPIZ
    rcall DELAY_500MS
    rcall IMPRIMIR_LISTO
    ret


; ------------------------------------------------------------------------------
; 4. FIGURA LIBRE (CASITA CLÁSICA)
; Paredes y base ortogonales + techo triangular a dos aguas con diagonales
; ------------------------------------------------------------------------------
DIBUJAR_LIBRE_CASA:
    ldi ZL, LOW(STR_DIB_LIBRE * 2)
    ldi ZH, HIGH(STR_DIB_LIBRE * 2)
    rcall USART_PRINT_FLASH

    rcall BAJAR_LAPIZ
    rcall DELAY_500MS

    ; Cuadrado de la casa:
    ; 1. Pared hacia abajo
    ldi dir_mask, DIR_ABAJO
    ldi dur_pasos, 8
    rcall MOVER_TRAZO

    ; 2. Suelo hacia la izquierda
    ldi dir_mask, DIR_IZQ
    ldi dur_pasos, 8
    rcall MOVER_TRAZO

    ; 3. Pared hacia arriba
    ldi dir_mask, DIR_ARRIBA
    ldi dur_pasos, 8
    rcall MOVER_TRAZO

    ; 4. Viga horizontal hacia la derecha
    ldi dir_mask, DIR_DER
    ldi dur_pasos, 8
    rcall MOVER_TRAZO

    ; Techo triangular a dos aguas:
    ; 5. Diagonal hacia el pico del techo (arriba-izquierda)
    ldi dir_mask, DIR_ARR_IZQ
    ldi dur_pasos, 4
    rcall MOVER_TRAZO

    ; 6. Diagonal bajada hacia la esquina izquierda (abajo-izquierda)
    ldi dir_mask, DIR_ABJ_IZQ
    ldi dur_pasos, 4
    rcall MOVER_TRAZO

    rcall SUBIR_LAPIZ
    rcall DELAY_500MS
    rcall IMPRIMIR_LISTO
    ret

; ------------------------------------------------------------------------------

; ==============================================================================
; MACROS DE MOVIMIENTO PARA DIBUJO DIRECTO
; ==============================================================================
.macro MOVER_IZ
    ldi dir_mask, DIR_IZQ
    ldi dur_pasos, @0
    rcall MOVER_TRAZO
.endmacro

.macro MOVER_DER
    ldi dir_mask, DIR_DER
    ldi dur_pasos, @0
    rcall MOVER_TRAZO
.endmacro

.macro MOVER_AR
    ldi dir_mask, DIR_ARRIBA
    ldi dur_pasos, @0
    rcall MOVER_TRAZO
.endmacro

.macro MOVER_AB
    ldi dir_mask, DIR_ABAJO
    ldi dur_pasos, @0
    rcall MOVER_TRAZO
.endmacro

.macro MOVER_AI
    ldi dir_mask, DIR_ARR_IZQ
    ldi dur_pasos, @0
    rcall MOVER_TRAZO
.endmacro

.macro MOVER_AD
    ldi dir_mask, DIR_ARR_DER
    ldi dur_pasos, @0
    rcall MOVER_TRAZO
.endmacro

.macro MOVER_ZI
    ldi dir_mask, DIR_ABJ_IZQ
    ldi dur_pasos, @0
    rcall MOVER_TRAZO
.endmacro

.macro MOVER_ZD
    ldi dir_mask, DIR_ABJ_DER
    ldi dur_pasos, @0
    rcall MOVER_TRAZO
.endmacro

; ==============================================================================
; CONSTANTE DE LARGO DE LÍNEA UNITARIA (Ajusta el tamaño del trazo de cada llamada)
;   PASO_UNITARIO = 1  -> Trazo corto (la mitad de largo que antes)
;   PASO_UNITARIO = 2  -> Trazo mediano (el tamaño anterior)
; ==============================================================================
.equ PASO_UNITARIO  = 1

; ==============================================================================
; SUBRUTINAS DE MOVIMIENTO RÁPIDO (CADA LLAMADA AVANZA 1 PASO)
; ==============================================================================
IZ:
IZQ:
    ldi dir_mask, DIR_IZQ
    ldi dur_pasos, PASO_UNITARIO
    rcall MOVER_TRAZO
    ret

DER:
    ldi dir_mask, DIR_DER
    ldi dur_pasos, PASO_UNITARIO
    rcall MOVER_TRAZO
    ret

AR:
ARR:
    ldi dir_mask, DIR_ARRIBA
    ldi dur_pasos, PASO_UNITARIO
    rcall MOVER_TRAZO
    ret

AB:
ABJ:
    ldi dir_mask, DIR_ABAJO
    ldi dur_pasos, PASO_UNITARIO
    rcall MOVER_TRAZO
    ret

AI:
ARR_IZQ_FN:
    ldi dir_mask, DIR_ARR_IZQ
    ldi dur_pasos, PASO_UNITARIO
    rcall MOVER_TRAZO
    ret

AD:
ARR_DER_FN:
    ldi dir_mask, DIR_ARR_DER
    ldi dur_pasos, PASO_UNITARIO
    rcall MOVER_TRAZO
    ret

ZI:
ABJ_IZQ_FN:
    ldi dir_mask, DIR_ABJ_IZQ
    ldi dur_pasos, PASO_UNITARIO
    rcall MOVER_TRAZO
    ret

ZD:
ABJ_DER_FN:
    ldi dir_mask, DIR_ABJ_DER
    ldi dur_pasos, PASO_UNITARIO
    rcall MOVER_TRAZO
    ret

BAJAR:
    rcall BAJAR_LAPIZ
    rcall DELAY_500MS
    ret

SUBIR:
    rcall SUBIR_LAPIZ
    rcall DELAY_500MS
    ret

PAUSA:
    rcall DELAY_250MS
    ret

; ------------------------------------------------------------------------------
; P. POKÉMON ASIGNADO: CUBONE 
; ------------------------------------------------------------------------------
DIBUJAR_CUBONE:
    ldi ZL, LOW(STR_DIB_CUBONE * 2)
    ldi ZH, HIGH(STR_DIB_CUBONE * 2)
    rcall USART_PRINT_FLASH

    ; Bajar lápiz al inicio
    rcall BAJAR_LAPIZ
    rcall DELAY_500MS

	rcall IZ
	rcall IZ
	rcall IZ
	rcall IZ

	rcall AB

	rcall IZ
	rcall IZ
	rcall IZ

	rcall AB
	rcall AB

	rcall IZ

	rcall AB

	rcall IZ
	; segunda oreja
	rcall IZ
	rcall AR
	rcall AR
	rcall IZ
	rcall AR
	rcall AR
	rcall DER
	rcall AR
	rcall DER
	rcall DER
	rcall AB
	rcall DER
	rcall AB
	rcall AB
	rcall AB
	rcall IZ
	rcall AB
	rcall IZ
	;Termina segunda oreja

	rcall AB
	rcall AB
	
	rcall IZ

	rcall AB
	rcall AB

	rcall IZ

	rcall AB
	rcall AB
	rcall AB
	rcall AB

	rcall IZ

	rcall AB

	rcall IZ

	rcall AB

	rcall IZ

	rcall AB
	rcall AB
	rcall AB

	rcall DER

	rcall AB
	rcall AB

	rcall DER

	rcall AB

	rcall DER
	rcall DER

	rcall AR

	rcall DER
	rcall DER

	rcall AR
	; ACA EMPIEZA NARIZ
	rcall IZ
	rcall IZ
	rcall AR
	rcall AR
	rcall DER
	rcall AR
	rcall DER
	rcall DER
	rcall AB
	rcall AB
	rcall IZ
	rcall AB
	; Termina Nariz

	rcall DER
	rcall DER

	rcall AB

	rcall DER
	rcall DER

	rcall AR

	rcall DER

	rcall AR

	rcall DER
	rcall DER
	rcall DER
	rcall DER

	rcall AR

	rcall DER

	rcall AB

	rcall DER
	rcall DER

	rcall AR
	rcall AR

	rcall DER

	rcall AR

	rcall DER

	rcall AR

	rcall DER

	rcall AR

	rcall DER

	rcall AR
	rcall AR

	rcall IZ
	rcall IZ

	rcall AR
	rcall AR
	
	rcall IZ

	rcall AR
	rcall AR

	rcall IZ

	rcall AR

	rcall DER

	rcall AR
	rcall AR

	rcall DER

	rcall AR
	rcall AR

	rcall IZ
	rcall IZ

	rcall AB

	rcall IZ
	rcall IZ

	rcall AB
	rcall IZ
	rcall AB
	rcall IZ
	rcall DER
	rcall AR
	rcall DER
	rcall AR

	rcall IZ
	rcall IZ

	rcall AR

	; Empezamos ojo
	rcall SUBIR
	rcall AB
	rcall AB
	rcall AB
	rcall AB
	rcall AB
	rcall AB

	rcall BAJAR
	rcall IZ
	rcall IZ
	rcall AB
	rcall IZ
	rcall AB
	rcall IZ
	rcall AB
	rcall AB
	rcall IZ
	rcall AB
	rcall AB
	rcall AB
	rcall AB
	rcall DER
	rcall DER
	rcall DER
	rcall DER
	rcall AR
	rcall DER
	rcall DER
	rcall AR
	rcall DER
	rcall AR
	rcall AR
	rcall AR
	rcall AR
	rcall AR
	rcall IZ
	rcall AR
	rcall IZ
	rcall SUBIR
	;Craneo Terminado
	rcall IZ
	rcall IZ
	rcall IZ
	rcall IZ
	rcall IZ
	rcall IZ
	rcall IZ
	rcall IZ
	rcall IZ
	rcall IZ
	rcall IZ
	rcall IZ
	rcall IZ
	rcall IZ
	rcall AB
	rcall AB
	rcall AB
	rcall AB
	rcall AB
	rcall AB
	rcall AB
	rcall AB
	rcall AB
	rcall AB
	rcall AB
	rcall AB

	rcall BAJAR

	rcall IZ
	rcall IZ
	rcall IZ
	rcall AR
	rcall IZ
	rcall AR
	rcall AR
	rcall IZ
	rcall AR
	rcall AR
	rcall IZ
	rcall AR
	rcall AR
	rcall AR
	rcall AR
	rcall DER
	rcall AR
	rcall AR
	rcall AR
	rcall IZ
	rcall AR
	rcall IZ
	rcall IZ
	rcall IZ
	rcall AB
	rcall IZ
	rcall AB
	rcall IZ
	rcall IZ
	rcall IZ
	rcall IZ
	rcall AB
	rcall IZ
	rcall AB
	rcall AB
	rcall AB
	rcall DER
	rcall AB
	rcall DER
	rcall DER
	rcall AB
	rcall DER
	rcall DER
	rcall AB
	rcall AB
	rcall DER
	rcall AB
	rcall DER
	rcall AB
	rcall AB
	rcall DER
	rcall AB
	rcall AB
	rcall IZ
	rcall AB
	rcall AB
	rcall DER
	rcall AB
	rcall DER
	rcall AB
	rcall DER
	rcall DER ; Empieza mano
	;Mano
	rcall AR
	rcall AR
	rcall IZ
	rcall AR
	rcall IZ
	rcall IZ
	; Temina mano, empieza loop de nuevo
	rcall IZ
	rcall AB
	rcall AB
	rcall DER
	rcall AB
	rcall DER
	rcall AB
	rcall DER
	rcall DER
	rcall AB ;continua el dibujo
	rcall DER
	rcall DER
	rcall DER
	rcall AR 
	rcall AR ; Finalizacion de hueso
	rcall IZ
	rcall IZ
	rcall AR
	rcall AR
	rcall AR
	rcall AB
	rcall AB
	rcall AB
	rcall DER
	rcall DER
	rcall AB
	; Continua el dibujo por el brazo
	rcall DER
	rcall DER
	rcall AR
	rcall DER
	rcall DER
	rcall DER
	rcall DER
	rcall DER
	rcall IZ
	rcall IZ
	rcall IZ
	rcall AB
	rcall AB
	rcall IZ
	rcall AB
	rcall IZ
	rcall AB
	rcall IZ ;  Inicio del pixel Pierna
	rcall IZ
	rcall AB
	rcall IZ
	rcall IZ
	rcall IZ
	rcall AB
	rcall IZ
	rcall AB
	rcall AB
	rcall DER
	rcall AB
	rcall DER
	rcall DER
	rcall AB
	rcall DER
	rcall DER ; Pixel Final PIERNA
	rcall AR
	rcall AR
	rcall AR
	rcall AR
	rcall AB
	rcall AB
	rcall AB
	rcall AB
	rcall DER
	rcall AB
	rcall DER
	rcall AB
	rcall AB
	rcall DER
	rcall AB
	rcall DER
	rcall DER
	rcall DER
	rcall AR
	rcall DER
	rcall AR
	rcall DER
	rcall AR
	rcall DER
	rcall AR
	rcall DER ; Comienza a finalizar pierna IZQ
	rcall AR
	rcall AR
	rcall AR
	rcall AR
	rcall AR
	rcall AR
	rcall IZ
	rcall AR
	rcall IZ
	rcall AR
	rcall IZ
	rcall DER
	rcall AB
	rcall DER
	rcall AB
	rcall DER
	rcall AB
	rcall AB
	rcall AB
	rcall AB
	rcall AB
	rcall AB
	rcall DER
	rcall DER
	rcall DER
	rcall DER ; Empieza pierna DERECHA
	; Panza
	rcall AR
	rcall DER
	rcall DER
	rcall AR
	rcall DER
	rcall AR
	rcall DER
	rcall AR
	rcall AR
	rcall DER
	rcall AR
	rcall AR
	rcall AR ; Termina panza comienza a volver
	rcall AB
	rcall AB
	rcall AB
	rcall IZ
	rcall AB
	rcall AB
	rcall IZ
	rcall AB
	rcall IZ
	rcall AB
	rcall IZ
	rcall IZ
	rcall AB ;Termina panza

	rcall AB
	rcall DER
	rcall DER
	rcall AB
	rcall DER
	rcall DER
	rcall DER
	rcall DER
	rcall DER
	rcall DER
	rcall AR
	rcall AR
	rcall IZ
	rcall AR
	rcall DER
	rcall AR
	rcall AR
	rcall AR
	rcall AR
	rcall IZ
	rcall AR
	rcall AR
	rcall IZ
	rcall AR
	rcall IZ
	rcall AR
	rcall AR
	rcall IZ
	rcall AR ; TERMINACION PANZA

	rcall AR
	rcall IZ
	rcall AR
	rcall IZ
	rcall AR
	rcall IZ

	rcall DER
	rcall AB
	rcall DER
	rcall AB
	rcall DER
	rcall AB
	; Continuar con brazo DERECHO
	rcall DER
	rcall AB
	rcall DER
	rcall DER
	rcall AB
	rcall DER
	rcall AB
	rcall DER
	rcall DER
	rcall AR
	rcall DER
	rcall AR
	rcall AR
	rcall AR
	rcall IZ
	rcall AR
	rcall IZ
	rcall AR
	rcall IZ
	rcall IZ
	rcall AR
	rcall IZ
	rcall IZ
	rcall AR
	rcall IZ
	rcall IZ
	rcall AR

    ; Subir lápiz al finalizar
    rcall SUBIR_LAPIZ
    rcall DELAY_500MS
    rcall IMPRIMIR_LISTO
    ret

; ==============================================================================
;	DIBUJAR TODAS LAS FIGURAS EN SECUENCIA
; ==============================================================================
DIBUJAR_TODAS:
    ldi ZL, LOW(STR_DIB_TODAS * 2)
    ldi ZH, HIGH(STR_DIB_TODAS * 2)
    rcall USART_PRINT_FLASH

    ; --- 1. TRIANGULO (Centro-Superior) ---
    rcall DIBUJAR_TRIANGULO
    rcall DELAY_500MS

    MOVER_IZ 22
    rcall DELAY_500MS

    ; --- 2. CIRCULO (Superior-Izquierda) ---
    rcall DIBUJAR_CIRCULO
    rcall DELAY_500MS

    MOVER_AB 25
    rcall DELAY_500MS

    ; --- 3. PENTAGRAMA (Inferior-Izquierda) ---
    rcall DIBUJAR_PENTAGRAMA
    rcall DELAY_500MS

    MOVER_DER 25
    rcall DELAY_500MS

    ; --- 4. CASITA (Inferior-Centro) ---
    rcall DIBUJAR_LIBRE_CASA
    rcall DELAY_500MS

    ; Mover a la DERECHA y más hacia ARRIBA para Cubone
 
    MOVER_DER 33
    MOVER_AR 22
    rcall DELAY_500MS

    ; --- 5. CUBONE (Lateral Derecho) ---
    rcall DIBUJAR_CUBONE
    rcall DELAY_500MS

    ; Retorno al centro
    MOVER_IZ 28
    MOVER_AR 22
    rcall DELAY_500MS

    ldi ZL, LOW(STR_TODAS_OK * 2)
    ldi ZH, HIGH(STR_TODAS_OK * 2)
    rcall USART_PRINT_FLASH
    ret

MODO_PERSONALIZADO:
    ldi ZL, LOW(STR_MENU_MANUAL * 2)
    ldi ZH, HIGH(STR_MENU_MANUAL * 2)
    rcall USART_PRINT_FLASH

BUCLE_MANUAL:
    rcall USART_RX
    mov rx_char, temp

    cpi rx_char, 'X'
    breq SALIR_MANUAL
    cpi rx_char, 'x'
    breq SALIR_MANUAL
    rjmp PROCESAR_COMANDOS_MANUAL

SALIR_MANUAL:
    rcall SUBIR_LAPIZ
    ret

PROCESAR_COMANDOS_MANUAL:
    ; Solenoide:
    cpi rx_char, 'B'
    breq MAN_LAPIZ_BAJAR
    cpi rx_char, 'b'
    breq MAN_LAPIZ_BAJAR

    cpi rx_char, 'U'
    breq MAN_LAPIZ_SUBIR
    cpi rx_char, 'u'
    breq MAN_LAPIZ_SUBIR

    ; Movimientos ortogonales:
    cpi rx_char, 'W'
    breq MAN_ARR
    cpi rx_char, 'w'
    breq MAN_ARR

    cpi rx_char, 'S'
    breq MAN_ABJ
    cpi rx_char, 's'
    breq MAN_ABJ

    cpi rx_char, 'A'
    breq MAN_IZQ
    cpi rx_char, 'a'
    breq MAN_IZQ

    cpi rx_char, 'D'
    breq MAN_DER
    cpi rx_char, 'd'
    breq MAN_DER

    ; Movimientos diagonales:
    cpi rx_char, 'Q'
    breq MAN_ARR_IZQ
    cpi rx_char, 'q'
    breq MAN_ARR_IZQ

    cpi rx_char, 'E'
    breq MAN_ARR_DER
    cpi rx_char, 'e'
    breq MAN_ARR_DER

    cpi rx_char, 'Z'
    breq MAN_ABJ_IZQ
    cpi rx_char, 'z'
    breq MAN_ABJ_IZQ

    cpi rx_char, 'C'
    breq MAN_ABJ_DER
    cpi rx_char, 'c'
    breq MAN_ABJ_DER

    rjmp BUCLE_MANUAL

MAN_LAPIZ_BAJAR:
    rcall BAJAR_LAPIZ
    ldi ZL, LOW(STR_MAN_BAJADO * 2)
    ldi ZH, HIGH(STR_MAN_BAJADO * 2)
    rcall USART_PRINT_FLASH
    rjmp BUCLE_MANUAL

MAN_LAPIZ_SUBIR:
    rcall SUBIR_LAPIZ
    ldi ZL, LOW(STR_MAN_SUBIDO * 2)
    ldi ZH, HIGH(STR_MAN_SUBIDO * 2)
    rcall USART_PRINT_FLASH
    rjmp BUCLE_MANUAL

MAN_ARR:
    ldi dir_mask, DIR_ARRIBA
    rjmp EJECUTAR_PASO_MANUAL

MAN_ABJ:
    ldi dir_mask, DIR_ABAJO
    rjmp EJECUTAR_PASO_MANUAL

MAN_IZQ:
    ldi dir_mask, DIR_IZQ
    rjmp EJECUTAR_PASO_MANUAL

MAN_DER:
    ldi dir_mask, DIR_DER
    rjmp EJECUTAR_PASO_MANUAL

MAN_ARR_IZQ:
    ldi dir_mask, DIR_ARR_IZQ
    rjmp EJECUTAR_PASO_MANUAL

MAN_ARR_DER:
    ldi dir_mask, DIR_ARR_DER
    rjmp EJECUTAR_PASO_MANUAL

MAN_ABJ_IZQ:
    ldi dir_mask, DIR_ABJ_IZQ
    rjmp EJECUTAR_PASO_MANUAL

MAN_ABJ_DER:
    ldi dir_mask, DIR_ABJ_DER
    rjmp EJECUTAR_PASO_MANUAL

EJECUTAR_PASO_MANUAL:
    ldi dur_pasos, 2
    rcall MOVER_TRAZO
    rjmp BUCLE_MANUAL

; ==============================================================================
; SUBRUTINAS DE COMUNICACIÓN USART
; ==============================================================================
USART_INIT:
    ldi temp, HIGH(UBRR_VAL)
    sts UBRR0H, temp
    ldi temp, LOW(UBRR_VAL)
    sts UBRR0L, temp

    ldi temp, (1 << RXEN0) | (1 << TXEN0)
    sts UCSR0B, temp

    ldi temp, (1 << UCSZ01) | (1 << UCSZ00)
    sts UCSR0C, temp
    ret

USART_TX:
    lds aux, UCSR0A
    sbrs aux, UDRE0
    rjmp USART_TX
    sts UDR0, temp
    ret

USART_RX:
    lds aux, UCSR0A
    sbrs aux, RXC0
    rjmp USART_RX
    lds temp, UDR0
    ret

USART_PRINT_FLASH:
    lpm temp, Z+
    tst temp
    breq FIN_PRINT
    rcall USART_TX
    rjmp USART_PRINT_FLASH
FIN_PRINT:
    ret

MOSTRAR_MENU:
    ldi ZL, LOW(STR_MENU_PRINCIPAL * 2)
    ldi ZH, HIGH(STR_MENU_PRINCIPAL * 2)
    rcall USART_PRINT_FLASH
    ret

IMPRIMIR_LISTO:
    ldi ZL, LOW(STR_LISTO * 2)
    ldi ZH, HIGH(STR_LISTO * 2)
    rcall USART_PRINT_FLASH
    ret

; ==============================================================================
; SUBRUTINAS DE TEMPORIZACIÓN
; ==============================================================================
DELAY_CONMUTACION:
    ldi del_cnt1, 5
D_CONM_LOOP:
    rcall DELAY_10MS
    dec del_cnt1
    brne D_CONM_LOOP
    ret

DELAY_250MS:
    ldi del_cnt1, 25
D_250_LOOP:
    rcall DELAY_10MS
    dec del_cnt1
    brne D_250_LOOP
    ret

DELAY_PASO_BASE:
    ldi del_cnt1, 15
D_PASO_LOOP:
    rcall DELAY_10MS
    dec del_cnt1
    brne D_PASO_LOOP
    ret

DELAY_PASO_HD:
    ldi del_cnt1, 4                         ; 4 * 10 ms = 40 ms por micro-paso
D_HD_LOOP:
    rcall DELAY_10MS
    dec del_cnt1
    brne D_HD_LOOP
    ret

DELAY_500MS:
    ldi del_cnt1, 50
D_500_LOOP:
    rcall DELAY_10MS
    dec del_cnt1
    brne D_500_LOOP
    ret

DELAY_1S:
    rcall DELAY_500MS
    rcall DELAY_500MS
    ret

DELAY_10MS:
    ldi del_cnt2, 208
D_10MS_L1:
    ldi del_cnt3, 255
D_10MS_L2:
    dec del_cnt3
    brne D_10MS_L2
    dec del_cnt2
    brne D_10MS_L1
    ret

STR_MENU_PRINCIPAL:
    .db 13, 10, "========================================================", 13, 10
    .db "  UTEC - TECNOLOGIAS DE MICROPROCESAMIENTO", 13, 10
    .db "  2DO LABORATORIO: CONTROL DE PLOTTER (PROBLEMA C)", 13, 10
    .db "  GRUPO 16 | POKEMON ASIGNADO: CUBONE (#104)", 13, 10
    .db "  Integrantes: G. Gonzalez, F. Rodriguez, J. Carbone", 13, 10
    .db "========================================================", 13, 10
    .db "  [1] Dibujar Triangulo ", 13, 10
    .db "  [2] Dibujar Circulo (Poligono regular)", 13, 10
    .db "  [3] Dibujar Pentagrama (Estrella 5 puntas)", 13, 10
    .db "  [4] Dibujar Figura Libre (Casita) ", 13, 10
    .db "  [P] Dibujar Pokemon Asignado: CUBONE (#104) ", 13, 10
    .db "  [T] Dibujar TODAS las figuras en secuencia", 13, 10
    .db "  [M] Modo de Edicion Personalizada (Control Manual)", 13, 10
    .db "========================================================", 13, 10
    .db "  Ingrese su opcion: ", 0

STR_MENU_MANUAL:
    .db 13, 10, "--------------------------------------------------------", 13, 10
    .db "  >>> MODO DE EDICION Y CONTROL PERSONALIZADO <<< ", 13, 10
    .db "--------------------------------------------------------", 13, 10
    .db "  Movimiento: [W] Arr  [S] Abj  [A] Izq  [D] Der  ", 13, 10
    .db "  Diagonal:   [Q] Arr-Izq   [E] Arr-Der ", 13, 10
    .db "              [Z] Abj-Izq   [C] Abj-Der ", 13, 10
    .db "  Solenoide:  [B] Bajar lapiz (Trazar)  ", 13, 10
    .db "              [U] Subir lapiz (Mover sin trazo) ", 13, 10
    .db "  Salir:      [X] Volver al menu principal      ", 13, 10
    .db "--------------------------------------------------------", 13, 10
    .db "  Ingrese comando: ", 0

STR_MAN_BAJADO:
    .db "  [OK] Lapiz ABAJO (Trazando)", 13, 10, 0

STR_MAN_SUBIDO:
    .db "  [OK] Lapiz ARRIBA (Sin trazo)", 13, 10, 0

STR_DIB_TRIANGULO:
    .db 13, 10, "-> [INICIO] Dibujando Triangulo... ", 13, 10, 0

STR_DIB_CIRCULO:
    .db 13, 10, "-> [INICIO] Dibujando Circulo (Poligono)...", 13, 10, 0

STR_DIB_PENTAGRAMA:
    .db 13, 10, "-> [INICIO] Dibujando Pentagrama (5 puntas)", 13, 10, 0

STR_DIB_LIBRE:
    .db 13, 10, "-> [INICIO] Dibujando Figura Libre (Casita)", 13, 10, 0

STR_DIB_CUBONE:
    .db 13, 10, "-> [INICIO] Dibujando Pokemon: CUBONE (#104) ", 13, 10, 0

STR_DIB_TODAS:
    .db 13, 10, "-> [INICIO] Dibujando TODAS las figuras... ", 13, 10, 0

STR_TODAS_OK:
    .db 13, 10, "-> [FIN] Todas las figuras completadas con exito.", 13, 10, 0

STR_LISTO:
    .db "-> [OK] Trazo finalizado.", 13, 10, 0

STR_VIAJE_CENTRO:
    .db "-> [MOV] Viajando al centro...", 13, 10, 0, 0

LUT_CIRCULO:
    .db 0xFF, 0x01, 0xFF, 0x03
    ; Cuadrante 1: Arriba e Izquierda (dX = -60, dY = +60)
    .db DIR_ARRIBA, 7, DIR_ARR_IZQ, 1
    .db DIR_ARRIBA, 5, DIR_ARR_IZQ, 2
    .db DIR_ARRIBA, 3, DIR_ARR_IZQ, 2
    .db DIR_ARRIBA, 3, DIR_ARR_IZQ, 2
    .db DIR_ARRIBA, 2, DIR_ARR_IZQ, 3
    .db DIR_ARRIBA, 2, DIR_ARR_IZQ, 3
    .db DIR_ARR_IZQ, 4, DIR_ARR_IZQ, 4
    .db DIR_ARR_IZQ, 4, DIR_ARR_IZQ, 3
    .db DIR_IZQ, 1, DIR_ARR_IZQ, 3
    .db DIR_IZQ, 2, DIR_ARR_IZQ, 2
    .db DIR_IZQ, 3, DIR_ARR_IZQ, 2
    .db DIR_IZQ, 3, DIR_ARR_IZQ, 2
    .db DIR_IZQ, 4, DIR_ARR_IZQ, 1
    .db DIR_IZQ, 6, DIR_IZQ, 0

    ; Cuadrante 2: Abajo e Izquierda (dX = -60, dY = -60)
    .db DIR_IZQ, 6, DIR_IZQ, 0
    .db DIR_IZQ, 4, DIR_ABJ_IZQ, 1
    .db DIR_IZQ, 3, DIR_ABJ_IZQ, 2
    .db DIR_IZQ, 3, DIR_ABJ_IZQ, 2
    .db DIR_IZQ, 2, DIR_ABJ_IZQ, 2
    .db DIR_IZQ, 1, DIR_ABJ_IZQ, 3
    .db DIR_ABJ_IZQ, 3, DIR_ABJ_IZQ, 4
    .db DIR_ABJ_IZQ, 4, DIR_ABJ_IZQ, 4
    .db DIR_ABJ_IZQ, 3, DIR_ABAJO, 2
    .db DIR_ABJ_IZQ, 3, DIR_ABAJO, 2
    .db DIR_ABJ_IZQ, 2, DIR_ABAJO, 3
    .db DIR_ABJ_IZQ, 2, DIR_ABAJO, 3
    .db DIR_ABJ_IZQ, 2, DIR_ABAJO, 5
    .db DIR_ABJ_IZQ, 1, DIR_ABAJO, 7

    ; Cuadrante 3: Abajo y Derecha (dX = +60, dY = -60)
    .db DIR_ABAJO, 7, DIR_ABJ_DER, 1
    .db DIR_ABAJO, 5, DIR_ABJ_DER, 2
    .db DIR_ABAJO, 3, DIR_ABJ_DER, 2
    .db DIR_ABAJO, 3, DIR_ABJ_DER, 2
    .db DIR_ABAJO, 2, DIR_ABJ_DER, 3
    .db DIR_ABAJO, 2, DIR_ABJ_DER, 3
    .db DIR_ABJ_DER, 4, DIR_ABJ_DER, 4
    .db DIR_ABJ_DER, 4, DIR_ABJ_DER, 3
    .db DIR_DER, 1, DIR_ABJ_DER, 3
    .db DIR_DER, 2, DIR_ABJ_DER, 2
    .db DIR_DER, 3, DIR_ABJ_DER, 2
    .db DIR_DER, 3, DIR_ABJ_DER, 2
    .db DIR_DER, 4, DIR_ABJ_DER, 1
    .db DIR_DER, 6, DIR_DER, 0

    ; Cuadrante 4: Arriba y Derecha (dX = +60, dY = +60)
    .db DIR_DER, 6, DIR_DER, 0
    .db DIR_DER, 4, DIR_ARR_DER, 1
    .db DIR_DER, 3, DIR_ARR_DER, 2
    .db DIR_DER, 3, DIR_ARR_DER, 2
    .db DIR_DER, 2, DIR_ARR_DER, 2
    .db DIR_DER, 1, DIR_ARR_DER, 3
    .db DIR_ARR_DER, 3, DIR_ARR_DER, 4
    .db DIR_ARR_DER, 4, DIR_ARR_DER, 4
    .db DIR_ARR_DER, 3, DIR_ARRIBA, 2
    .db DIR_ARR_DER, 3, DIR_ARRIBA, 2
    .db DIR_ARR_DER, 2, DIR_ARRIBA, 3
    .db DIR_ARR_DER, 2, DIR_ARRIBA, 3
    .db DIR_ARR_DER, 2, DIR_ARRIBA, 5
    .db DIR_ARR_DER, 1, DIR_ARRIBA, 7
    .db 0xFF, 0x04, 0x00, 0x00

LUT_ESTRELLA:
    .db 0xFF, 0x01, 0xFF, 0x03
    ; 1. Punta Superior -> Valle Sup. Izq. (dX = -10, dY = -28)
    .db DIR_ABJ_IZQ, 1, DIR_ABAJO, 2
    .db DIR_ABJ_IZQ, 1, DIR_ABAJO, 2
    .db DIR_ABJ_IZQ, 1, DIR_ABAJO, 2
    .db DIR_ABJ_IZQ, 1, DIR_ABAJO, 2
    .db DIR_ABJ_IZQ, 1, DIR_ABAJO, 2
    .db DIR_ABJ_IZQ, 1, DIR_ABAJO, 2
    .db DIR_ABJ_IZQ, 1, DIR_ABAJO, 2
    .db DIR_ABJ_IZQ, 1, DIR_ABAJO, 2
    .db DIR_ABJ_IZQ, 1, DIR_ABAJO, 2
    .db DIR_ABJ_IZQ, 1, DIR_ABAJO, 1

    ; 2. Brazo Horizontal Izquierdo (dX = -25, dY = 0)
    .db DIR_IZQ, 25, 0xFF, 0x04

    ; 3. Punta Izq. -> Valle Inf. Izq. (dX = +22, dY = -16)
    .db DIR_ABJ_DER, 2, DIR_DER, 1
    .db DIR_ABJ_DER, 2, DIR_DER, 1
    .db DIR_ABJ_DER, 2, DIR_DER, 1
    .db DIR_ABJ_DER, 2, DIR_DER, 1
    .db DIR_ABJ_DER, 2, DIR_DER, 1
    .db DIR_ABJ_DER, 2, DIR_DER, 1
    .db DIR_ABJ_DER, 4, DIR_DER, 0

    ; 4. Valle Inf. Izq. -> Pata Inf. Izq. (dX = -12, dY = -26)
    .db DIR_ABJ_IZQ, 1, DIR_ABAJO, 2
    .db DIR_ABJ_IZQ, 1, DIR_ABAJO, 2
    .db DIR_ABJ_IZQ, 1, DIR_ABAJO, 2
    .db DIR_ABJ_IZQ, 1, DIR_ABAJO, 2
    .db DIR_ABJ_IZQ, 1, DIR_ABAJO, 2
    .db DIR_ABJ_IZQ, 1, DIR_ABAJO, 2
    .db DIR_ABJ_IZQ, 1, DIR_ABAJO, 2
    .db DIR_ABJ_IZQ, 1, DIR_ABAJO, 2
    .db DIR_ABJ_IZQ, 2, DIR_ABAJO, 1
    .db DIR_ABJ_IZQ, 2, 0xFF, 0x04

    ; 5. Pata Inf. Izq. -> Valle Central (dX = +25, dY = +18)
    .db DIR_ARR_DER, 2, DIR_DER, 1
    .db DIR_ARR_DER, 2, DIR_DER, 1
    .db DIR_ARR_DER, 2, DIR_DER, 1
    .db DIR_ARR_DER, 2, DIR_DER, 1
    .db DIR_ARR_DER, 2, DIR_DER, 1
    .db DIR_ARR_DER, 2, DIR_DER, 1
    .db DIR_ARR_DER, 2, DIR_DER, 1
    .db DIR_ARR_DER, 4, DIR_DER, 0

    ; 6. Valle Central -> Pata Inf. Der. (dX = +25, dY = -18)
    .db DIR_ABJ_DER, 4, DIR_DER, 0
    .db DIR_ABJ_DER, 2, DIR_DER, 1
    .db DIR_ABJ_DER, 2, DIR_DER, 1
    .db DIR_ABJ_DER, 2, DIR_DER, 1
    .db DIR_ABJ_DER, 2, DIR_DER, 1
    .db DIR_ABJ_DER, 2, DIR_DER, 1
    .db DIR_ABJ_DER, 2, DIR_DER, 1
    .db DIR_ABJ_DER, 2, 0xFF, 0x04

    ; 7. Pata Inf. Der. -> Valle Inf. Der. (dX = -12, dY = +26)
    .db DIR_ARR_IZQ, 2, DIR_ARRIBA, 1
    .db DIR_ARR_IZQ, 2, DIR_ARRIBA, 1
    .db DIR_ARR_IZQ, 1, DIR_ARRIBA, 2
    .db DIR_ARR_IZQ, 1, DIR_ARRIBA, 2
    .db DIR_ARR_IZQ, 1, DIR_ARRIBA, 2
    .db DIR_ARR_IZQ, 1, DIR_ARRIBA, 2
    .db DIR_ARR_IZQ, 1, DIR_ARRIBA, 2
    .db DIR_ARR_IZQ, 1, DIR_ARRIBA, 2
    .db DIR_ARR_IZQ, 1, DIR_ARRIBA, 2
    .db DIR_ARR_IZQ, 1, DIR_ARRIBA, 2

    ; 8. Valle Inf. Der. -> Punta Lateral Der. (dX = +22, dY = +16)
    .db DIR_ARR_DER, 4, DIR_DER, 0
    .db DIR_ARR_DER, 2, DIR_DER, 1
    .db DIR_ARR_DER, 2, DIR_DER, 1
    .db DIR_ARR_DER, 2, DIR_DER, 1
    .db DIR_ARR_DER, 2, DIR_DER, 1
    .db DIR_ARR_DER, 2, DIR_DER, 1
    .db DIR_ARR_DER, 2, 0xFF, 0x04

    ; 9. Punta Lateral Der. -> Valle Sup. Der. (dX = -25, dY = 0)
    .db DIR_IZQ, 25, DIR_IZQ, 0

    ; 10. Valle Sup. Der. -> Punta Superior (dX = -10, dY = +28)
    .db DIR_ARR_IZQ, 1, DIR_ARRIBA, 1
    .db DIR_ARR_IZQ, 1, DIR_ARRIBA, 2
    .db DIR_ARR_IZQ, 1, DIR_ARRIBA, 2
    .db DIR_ARR_IZQ, 1, DIR_ARRIBA, 2
    .db DIR_ARR_IZQ, 1, DIR_ARRIBA, 2
    .db DIR_ARR_IZQ, 1, DIR_ARRIBA, 2
    .db DIR_ARR_IZQ, 1, DIR_ARRIBA, 2
    .db DIR_ARR_IZQ, 1, DIR_ARRIBA, 2
    .db DIR_ARR_IZQ, 1, DIR_ARRIBA, 2
    .db DIR_ARR_IZQ, 1, DIR_ARRIBA, 2
    .db 0xFF, 0x04, 0x00, 0x00
