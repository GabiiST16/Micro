.include "m328pdef.inc"
.org 0x0000
    rjmp inicio 
    ldi r16, HIGH(RAMEND)
    out  SPH, r16
    ldi  r16, LOW(RAMEND)
    out  SPL, r16
    
