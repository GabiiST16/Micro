; ==============================================================================
; UNIVERSIDAD TECNOLÃ“GICA (UTEC) - INGENIERÃA MECATRÃ“NICA
; UNIDAD CURRICULAR: TECNOLOGÃAS DE MICROPROCESAMIENTO - 2026
; 2DO LABORATORIO - PROBLEMA C: CONTROL DE PLOTTER CON ATmega328P Y PLC
; 
; GRUPO 16:
;   - Gabriel GonzÃ¡lez
;   - Facundo RodrÃ­guez
;   - Jacinto Carbone
;
; POKÃ‰MON ASIGNADO: Cubone (#104) - Tipo Tierra
; "Cabeza redondeada + hueso/hacha. Silueta reconocible con pocos trazos."
;
; MICROCONTROLADOR: ATmega328P (F_CPU = 16 MHz)
; COMUNICACIÃ“N: USART (9600 Baudios, 8 bits, 1 stop, sin paridad)
; ==============================================================================
; MAPEO REAL DE PINES HACIA EL PLC (A travÃ©s de MÃ³dulos de RelÃ©s / Optos):
;   PD0 (D0) -> USART RX (Entrada serie desde PC)
;   PD1 (D1) -> USART TX (Salida serie hacia PC)
;   PD2 (D2) -> Bajar solenoide neumÃ¡tico (Entrada X0 del PLC)
;   PD3 (D3) -> Subir solenoide neumÃ¡tico (Entrada X1 del PLC)
;   PD4 (D4) -> Movimiento hacia ABAJO    (Entrada X5 del PLC)
;   PD5 (D5) -> Movimiento hacia ARRIBA   (Entrada X6 del PLC)
;   PD6 (D6) -> Movimiento hacia DERECHA  (Entrada X7 del PLC)
;   PD7 (D7) -> Movimiento hacia IZQUIERDA(Entrada X10 del PLC)
; ==============================================================================

.include "m328pdef.inc"

; ------------------------------------------------------------------------------
; CONSTANTES DE CONFIGURACIÃ“N DE RELOJ Y USART
; ------------------------------------------------------------------------------
.equ F_CPU          = 16000000              ; Frecuencia de reloj: 16 MHz
.equ BAUD           = 9600                  ; Velocidad en baudios
.equ UBRR_VAL       = (F_CPU / (16 * BAUD)) - 1  ; UBRR = 103 (0x0067)

; ------------------------------------------------------------------------------
; MÃSCARAS DE HARDWARE EN PORTD (Pines D2 a D7)
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

; Movimientos diagonales simultÃ¡neos calibrados con el pinout real
.equ DIR_ARR_DER    = (1 << PIN_MOT_ARRIBA) | (1 << PIN_MOT_DER) ; PD5 + PD6
.equ DIR_ARR_IZQ    = (1 << PIN_MOT_ARRIBA) | (1 << PIN_MOT_IZQ) ; PD5 + PD7
.equ DIR_ABJ_DER    = (1 << PIN_MOT_ABAJO)  | (1 << PIN_MOT_DER) ; PD4 + PD6
.equ DIR_ABJ_IZQ    = (1 << PIN_MOT_ABAJO)  | (1 << PIN_MOT_IZQ) ; PD4 + PD7

; MÃ¡scaras para aislar registros de motores y solenoides
.equ MASK_MOTORES   = 0b11110000
.equ MASK_SOLENOIDE = 0b00001100
.equ MASK_USART     = 0b00000011

; ------------------------------------------------------------------------------
; CALIBRACIÃ“N DE VIAJE HACIA EL CENTRO DE LA HOJA A4 (Desde Home en esquina sup. der.)
; Cada paso = ~150 ms
; ------------------------------------------------------------------------------
.equ PASOS_CENTRO_X = 70                     ; Desplazamiento a la IZQUIERDA (PD7)
.equ PASOS_CENTRO_Y = 50                     ; Desplazamiento hacia ABAJO (PD4)

; ------------------------------------------------------------------------------
; DEFINICIÃ“N DE REGISTROS DE TRABAJO
; ------------------------------------------------------------------------------
.def temp           = r16                   ; Registro general de trabajo
.def aux            = r17                   ; Registro secundario de trabajo
.def dir_mask       = r18                   ; MÃ¡scara de direcciÃ³n activa
.def dur_pasos      = r19                   ; DuraciÃ³n del trazo (unidades de paso)
.def del_cnt1       = r20                   ; Contador 1 para bucles de retardo
.def del_cnt2       = r21                   ; Contador 2 para bucles de retardo
.def del_cnt3       = r22                   ; Contador 3 para bucles de retardo
.def rx_char        = r23                   ; CarÃ¡cter recibido por USART

; ==============================================================================
; VECTOR DE RESET
; ==============================================================================
.cseg
.org 0x0000
    rjmp RESET_HANDLER

; ==============================================================================
; INICIALIZACIÃ“N DEL SISTEMA
; ==============================================================================
RESET_HANDLER:
    ; 1. ConfiguraciÃ³n del Stack Pointer
    ldi temp, HIGH(RAMEND)
    out SPH, temp
    ldi temp, LOW(RAMEND)
    out SPL, temp

    ; 2. Configurar pines de salida en PORTD (D2 a D7 como salidas)
    ldi temp, 0b11111110
    out DDRD, temp

    ; 3. Estado inicial seguro: motores apagados y bobinas inactivas
    in temp, PORTD
    andi temp, MASK_USART
    out PORTD, temp

    ; 4. Inicializar mÃ³dulo USART
    rcall USART_INIT

    ; 5. Levantar el lÃ¡piz por seguridad al arrancar
    rcall SUBIR_LAPIZ
    rcall DELAY_500MS

    ; 6. Mostrar menÃº principal
    rcall MOSTRAR_MENU

; ==============================================================================
; BUCLE PRINCIPAL (DESPACHADOR DE COMANDOS DEL MENÃš)
; ==============================================================================
MAIN_LOOP:
    rcall USART_RX                          ; Esperar comando del usuario
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

; --- Saltos intermedios para opciones ---
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
; SUBRUTINAS DE CONTROL DE HARDWARE DEL PLOTTER
; ==============================================================================

; ------------------------------------------------------------------------------
; SUBIR_LAPIZ: Desactiva bajada (X0) y activa subida (X1, D3) con pulso de 250ms
; ------------------------------------------------------------------------------
SUBIR_LAPIZ:
    in temp, PORTD
    cbr temp, (1 << PIN_SOL_BAJAR)          ; Apagar D2 (X0)
    sbr temp, (1 << PIN_SOL_SUBIR)          ; Encender D3 (X1)
    out PORTD, temp

    rcall DELAY_250MS

    in temp, PORTD
    cbr temp, (1 << PIN_SOL_SUBIR)          ; Apagar pulso
    out PORTD, temp

    rcall DELAY_CONMUTACION
    ret

; ------------------------------------------------------------------------------
; BAJAR_LAPIZ: Desactiva subida (X1) y activa bajada (X0, D2) con pulso de 250ms
; ------------------------------------------------------------------------------
BAJAR_LAPIZ:
    in temp, PORTD
    cbr temp, (1 << PIN_SOL_SUBIR)          ; Apagar D3 (X1)
    sbr temp, (1 << PIN_SOL_BAJAR)          ; Encender D2 (X0)
    out PORTD, temp

    rcall DELAY_250MS

    in temp, PORTD
    cbr temp, (1 << PIN_SOL_BAJAR)
    out PORTD, temp

    rcall DELAY_CONMUTACION
    ret

; ------------------------------------------------------------------------------
; PARAR_MOTORES: Apaga seÃ±ales de movimiento (D4..D7) preservando USART y solenoides
; ------------------------------------------------------------------------------
PARAR_MOTORES:
    in temp, PORTD
    andi temp, ~MASK_MOTORES
    out PORTD, temp
    rcall DELAY_CONMUTACION
    ret

; ------------------------------------------------------------------------------
; MOVER_TRAZO: Aplica dir_mask (r18) durante dur_pasos (r19)
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
; Conmuta direcciÃ³n en caliente utilizando micro-pasos de alta definiciÃ³n (40 ms)
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
; IntÃ©rprete de tablas Look-Up Table (LUT) en memoria Flash (.cseg).
; Lee pares (comando, duraciÃ³n). Si comando es 0xFF ejecuta acciÃ³n de control
; (subir/bajar lÃ¡piz, parar motores, retardos). Si comando es 0x00 termina.
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
; VIAJAR_AL_CENTRO:
; Levanta el lÃ¡piz, viaja en secuencia ortogonal (Izquierda PD7 -> Abajo PD4)
; hasta el centro del papel A4 sin rayar la hoja.
; ------------------------------------------------------------------------------
VIAJAR_AL_CENTRO:
    rcall SUBIR_LAPIZ
    rcall DELAY_500MS                       ; Espera reposo total del pistÃ³n

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

    rcall DELAY_500MS                       ; EstabilizaciÃ³n mecÃ¡nica en el centro
    ret

; ==============================================================================
; RUTINAS DE DIBUJO DE FIGURAS
; ==============================================================================

; ------------------------------------------------------------------------------
; 1. TRIÃNGULO EQUILÃTERO (GRANDE)
; Base simÃ©trica completa y diagonales de 60Â°
; ------------------------------------------------------------------------------
DIBUJAR_TRIANGULO:
    ldi ZL, LOW(STR_DIB_TRIANGULO * 2)
    ldi ZH, HIGH(STR_DIB_TRIANGULO * 2)
    rcall USART_PRINT_FLASH

    rcall BAJAR_LAPIZ
    rcall DELAY_500MS

    ; Lado 1: Base hacia la IZQUIERDA (16 pasos)
    ldi dir_mask, DIR_IZQ
    ldi dur_pasos, 16
    rcall MOVER_TRAZO
    rcall DELAY_250MS

    ; Lado 2: Diagonal arriba-derecha (8 pasos)
    ldi dir_mask, DIR_ARR_DER
    ldi dur_pasos, 8
    rcall MOVER_TRAZO
    rcall DELAY_250MS

    ; Lado 3: Diagonal abajo-derecha (8 pasos, cierra exactamente la base)
    ldi dir_mask, DIR_ABJ_DER
    ldi dur_pasos, 8
    rcall MOVER_TRAZO
    rcall DELAY_250MS

    rcall SUBIR_LAPIZ
    rcall DELAY_500MS
    rcall IMPRIMIR_LISTO
    ret

; ------------------------------------------------------------------------------
; 2. CÃRCULO (ALTA DEFINICIÃ“N - 64 CORTES - DIÃMETRO ~100 MM)
; Ejecutado mediante Look-Up Table (LUT) en Flash con micro-pasos de 40 ms
; ------------------------------------------------------------------------------
DIBUJAR_CIRCULO:
    ldi ZL, LOW(STR_DIB_CIRCULO * 2)
    ldi ZH, HIGH(STR_DIB_CIRCULO * 2)
    rcall USART_PRINT_FLASH

    ldi ZL, LOW(LUT_CIRCULO * 2)
    ldi ZH, HIGH(LUT_CIRCULO * 2)
    rcall EJECUTAR_LUT

    rcall SUBIR_LAPIZ
    rcall DELAY_500MS
    rcall IMPRIMIR_LISTO
    ret

; ------------------------------------------------------------------------------
; 3. ESTRELLA DE 5 PUNTAS (ALTA DEFINICIÃ“N BRESENHAM)
; Silueta continua de 10 aristas rectas con mÃ¡s de 100 micro-cortes entrelazados
; y simetrÃ­a bilateral geomÃ©trica exacta
; ------------------------------------------------------------------------------
DIBUJAR_PENTAGRAMA:
    ldi ZL, LOW(STR_DIB_PENTAGRAMA * 2)
    ldi ZH, HIGH(STR_DIB_PENTAGRAMA * 2)
    rcall USART_PRINT_FLASH

    ldi ZL, LOW(LUT_ESTRELLA * 2)
    ldi ZH, HIGH(LUT_ESTRELLA * 2)
    rcall EJECUTAR_LUT

    rcall SUBIR_LAPIZ
    rcall DELAY_500MS
    rcall IMPRIMIR_LISTO
    ret


; ------------------------------------------------------------------------------
; 4. FIGURA LIBRE (CASITA CLÃSICA)
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
; P. POKÃ‰MON ASIGNADO: CUBONE (#104 - GRUPO 16)
; Trazo continuo unificado: Calavera original + Cuerpo de Estrella + Hueso
; El lÃ¡piz se baja una sola vez al inicio y sube al terminar (sin interrupciones)
; ------------------------------------------------------------------------------
DIBUJAR_CUBONE:
    ldi ZL, LOW(STR_DIB_CUBONE * 2)
    ldi ZH, HIGH(STR_DIB_CUBONE * 2)
    rcall USART_PRINT_FLASH

    ; Bajar lÃ¡piz al inicio y asegurar contacto firme
    rcall BAJAR_LAPIZ
    rcall DELAY_500MS

    ; ==========================================================================
    ; PARTE 1: CABEZA Y CRÃNEO DE CUBONE (TRAZO ORIGINAL CERRADO)
    ; ==========================================================================
    ; 1. MandÃ­bula inferior hacia la izquierda
    ldi dir_mask, DIR_IZQ
    ldi dur_pasos, 8
    rcall MOVER_TRAZO

    ; 2. Punta del hocico
    ldi dir_mask, DIR_ARR_IZQ
    ldi dur_pasos, 3
    rcall MOVER_TRAZO

    ; 3. Tabique nasal hacia arriba
    ldi dir_mask, DIR_ARRIBA
    ldi dur_pasos, 4
    rcall MOVER_TRAZO

    ; 4. Cuerno frontal (sale en diagonal arriba-izquierda y regresa)
    ldi dir_mask, DIR_ARR_IZQ
    ldi dur_pasos, 4
    rcall MOVER_TRAZO
    ldi dir_mask, DIR_ABJ_DER
    ldi dur_pasos, 4
    rcall MOVER_TRAZO

    ; 5. Frente y bÃ³veda craneal hacia la derecha
    ldi dir_mask, DIR_ARR_DER
    ldi dur_pasos, 3
    rcall MOVER_TRAZO
    ldi dir_mask, DIR_DER
    ldi dur_pasos, 5
    rcall MOVER_TRAZO

    ; 6. Cuerno dorsal posterior (sale en diagonal arriba-derecha y regresa)
    ldi dir_mask, DIR_ARR_DER
    ldi dur_pasos, 5
    rcall MOVER_TRAZO
    ldi dir_mask, DIR_ABJ_IZQ
    ldi dur_pasos, 5
    rcall MOVER_TRAZO

    ; 7. Nuca / parte trasera del crÃ¡neo
    ldi dir_mask, DIR_ABJ_DER
    ldi dur_pasos, 3
    rcall MOVER_TRAZO

    ; 8. Cuello hacia abajo cerrando exactamente en la base (0, 0)
    ldi dir_mask, DIR_ABAJO
    ldi dur_pasos, 7
    rcall MOVER_TRAZO

    ; ==========================================================================
    ; PARTE 2: CUERPO DE ESTRELLA (4 EXTREMIDADES) CONTINUO SIN LEVANTAR EL LÃPIZ
    ; ==========================================================================
    ; --- 1. BRAZO IZQUIERDO ---
    ; Hombro hacia la punta de la mano izquierda
    ldi dir_mask, DIR_IZQ
    ldi dur_pasos, 14
    rcall MOVER_TRAZO
    ; Retorno hacia la cintura izquierda
    ldi dir_mask, DIR_ABJ_DER
    ldi dur_pasos, 6
    rcall MOVER_TRAZO
    ldi dir_mask, DIR_DER
    ldi dur_pasos, 4
    rcall MOVER_TRAZO

    ; --- 2. PATA INFERIOR IZQUIERDA ---
    ; Cintura hacia la punta del pie izquierdo
    ldi dir_mask, DIR_ABJ_IZQ
    ldi dur_pasos, 6
    rcall MOVER_TRAZO
    ldi dir_mask, DIR_ABAJO
    ldi dur_pasos, 6
    rcall MOVER_TRAZO
    ; Pie izquierdo hacia la entrepierna central (X = 0)
    ldi dir_mask, DIR_ARR_DER
    ldi dur_pasos, 8
    rcall MOVER_TRAZO
    ldi dir_mask, DIR_DER
    ldi dur_pasos, 2
    rcall MOVER_TRAZO

    ; --- 3. PATA INFERIOR DERECHA ---
    ; Entrepierna hacia la punta del pie derecho
    ldi dir_mask, DIR_ABJ_DER
    ldi dur_pasos, 8
    rcall MOVER_TRAZO
    ldi dir_mask, DIR_DER
    ldi dur_pasos, 2
    rcall MOVER_TRAZO
    ; Pie derecho hacia la cintura derecha
    ldi dir_mask, DIR_ARR_IZQ
    ldi dur_pasos, 6
    rcall MOVER_TRAZO
    ldi dir_mask, DIR_ARRIBA
    ldi dur_pasos, 6
    rcall MOVER_TRAZO

    ; --- 4. BRAZO DERECHO HACIA LA MANO ---
    ; Cintura derecha hacia la punta de la mano derecha
    ldi dir_mask, DIR_ARR_DER
    ldi dur_pasos, 6
    rcall MOVER_TRAZO
    ldi dir_mask, DIR_DER
    ldi dur_pasos, 4
    rcall MOVER_TRAZO

    ; ==========================================================================
    ; PARTE 3: EL HUESO DE CUBONE SOSTENIDO EN LA MANO DERECHA
    ; ==========================================================================
    ; Cabeza superior del hueso
    ldi dir_mask, DIR_ARR_DER
    ldi dur_pasos, 2
    rcall MOVER_TRAZO
    ldi dir_mask, DIR_DER
    ldi dur_pasos, 2
    rcall MOVER_TRAZO
    ldi dir_mask, DIR_ABJ_DER
    ldi dur_pasos, 2
    rcall MOVER_TRAZO

    ; CaÃ±a del hueso hacia abajo
    ldi dir_mask, DIR_ABAJO
    ldi dur_pasos, 12
    rcall MOVER_TRAZO

    ; Cabeza inferior del hueso
    ldi dir_mask, DIR_ABJ_DER
    ldi dur_pasos, 2
    rcall MOVER_TRAZO
    ldi dir_mask, DIR_IZQ
    ldi dur_pasos, 4
    rcall MOVER_TRAZO
    ldi dir_mask, DIR_ARR_IZQ
    ldi dur_pasos, 2
    rcall MOVER_TRAZO

    ; CaÃ±a del hueso hacia arriba
    ldi dir_mask, DIR_ARRIBA
    ldi dur_pasos, 12
    rcall MOVER_TRAZO

    ; Cierre cabeza superior en la mano
    ldi dir_mask, DIR_ARR_DER
    ldi dur_pasos, 2
    rcall MOVER_TRAZO
    ldi dir_mask, DIR_ABJ_IZQ
    ldi dur_pasos, 2
    rcall MOVER_TRAZO
    ldi dir_mask, DIR_IZQ
    ldi dur_pasos, 2
    rcall MOVER_TRAZO

    ; --- RETORNO FINAL DE LA MANO HACIA EL CUELLO ---
    ldi dir_mask, DIR_IZQ
    ldi dur_pasos, 14
    rcall MOVER_TRAZO

    ; Subir lÃ¡piz Ãºnicamente al finalizar toda la figura
    rcall SUBIR_LAPIZ
    rcall DELAY_500MS
    rcall IMPRIMIR_LISTO
    ret

; ------------------------------------------------------------------------------
; T. DIBUJAR TODAS LAS FIGURAS EN SECUENCIA
; ------------------------------------------------------------------------------
DIBUJAR_TODAS:
    ldi ZL, LOW(STR_DIB_TODAS * 2)
    ldi ZH, HIGH(STR_DIB_TODAS * 2)
    rcall USART_PRINT_FLASH

    rcall DIBUJAR_TRIANGULO
    rcall DELAY_1S

    rcall DIBUJAR_CIRCULO
    rcall DELAY_1S

    rcall DIBUJAR_PENTAGRAMA
    rcall DELAY_1S

    rcall DIBUJAR_LIBRE_CASA
    rcall DELAY_1S

    rcall DIBUJAR_CUBONE

    ldi ZL, LOW(STR_TODAS_OK * 2)
    ldi ZH, HIGH(STR_TODAS_OK * 2)
    rcall USART_PRINT_FLASH
    ret

; ==============================================================================
; MODO DE CONTROL Y EDICIÃ“N PERSONALIZADO (REQUISITO 3)
; Control interactivo por terminal serie en tiempo real
; ==============================================================================
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
; SUBRUTINAS DE COMUNICACIÃ“N USART
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
; SUBRUTINAS DE TEMPORIZACIÃ“N (CALIBRADAS A 16 MHz)
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

; ==============================================================================
; CADENAS DE TEXTO EN MEMORIA FLASH (.cseg)
; Conteo estrictamente par de bytes por lÃ­nea (0 Warnings en avrasm2)
; ==============================================================================
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

; ==============================================================================
; TABLAS LOOK-UP TABLE (LUT) DE FIGURAS EN ALTA DEFINICIÃ“N (.cseg)
; Formato por entrada: (Comando/MÃ¡scara, DuraciÃ³n de pasos)
; Comandos especiales: 0xFF, 0x01 (Bajar lÃ¡piz)
;                      0xFF, 0x02 (Subir lÃ¡piz)
;                      0xFF, 0x03 (Delay 250ms)
;                      0xFF, 0x04 (Parar motores)
;                      0x00, 0x00 (Fin de tabla)
; ==============================================================================

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

