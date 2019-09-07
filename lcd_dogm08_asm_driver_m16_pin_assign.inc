;**************************************************************** v2a
;   ATMega16 Version (3.3V or 5V)
;
;   This AVR-asm code module is usable as an include file for assembly
;   language and or mixed asm/C application programs.
;
;   This code is designed to be executed on an AVR ATMega microcomputer.
;   And may be readily adapted for compatibility with IAR/AVR compilers.
;   See the IAR assembler reference guide for more information by
;   clicking 'Help > AVR Assembly Reference Guide" on the above menus.
;
;
;*********************************************************************
;
;   This module contains procedures to initialize and update
;   Dthe EA DOGM081L LCD modules configured with one 8 charactor line.
;
;   The display module hardware interface uses a 1-direction, write only
;   SPI interface. (See below for more information.)
;
;   The display module software interface uses an 8-byte
;   data (RAM) based display buffer.
;   (See below for more information.)
;*********************************************************************
;
;   *** Port B Interface Definitions:
;
;  Port B             PB7   PB6   PB5   PB4   PB3   PB2   PB1   PB0
;  Port B alt names   SCK   MISO  MOSI  /SS   /RS    -     -     -
;  LCD Mod Signal     D6     -     D7   /CSB   -     -     -     -
;  LCD Mod Pin #      29     -     28    38    -     -     -     -
;
;      Notes:  RS ==>  0 = command regs, 1 = data regs
;             /SS = active low SPI select signal

;*********************************************************************

;***  DATA Segment  **************************************************
.DSEG
dsp_buff:   .byte 8

;***  CODE Segment Subroutines  ************************************
.CSEG

;Equates added 11/30/16 so pin used for /RS can be changed
.equ	PORT_RSn	= PORTA   ;equates for the reg. select pin
.equ	PORT_bit_RSn	= PORTA0

;************************
;NAME:        delay_30uS
;ASSUMES:     nothing
;RETURNS:     nothing
;MODIFIES:    R24, SREG
;CALLED BY:   init_dsp
;DESCRIPTION: This procedure will generate a fixed delay of just over
;             30 uS (assuming a 1 MHz clock).
;********************************************************************
delay_30uS:   nop     ; fine tune delay
              nop
              push  r24
              ldi   r24, 0x0f  ; load delay count.
d30_loop:     dec   r24        ; count down to
              brne  d30_loop   ; zero.
              pop   r24
              ret

;*********************
;NAME:        v_delay
;ASSUMES:     R22, R23 = initial count values defining how many
;             30uS delays will be called. This procedure can generate
;             short delays (r23 = small #) or much longer delays (where
;             R23 value is large).
;RETURNS:     nothing
;MODIFIES:    R22, R23, SREG
;CALLED BY:   init_dsp, plus...
;DESCRIPTION: This procedure will generate a variable delay for a fixed
;             period of time based the values pasted in R24 and R25.
;
;Sample Delays:
;
;             R22  R23  DelayTime
;             ---  ---  ---------
;              1    1   ~65.5 uS
;              0    1   ~14.2 mS
;              0    9   ~130 mS

;**********************************************************************
v_delay:      rcall delay_30uS ; delay for ~30uS
              dec   r22      ; decrement inner loop value, and
              brne  v_delay  ; loop until zero.
              dec   r23      ; decr outer loop count, and loop back
              brne  v_delay  ; to inner loop delay until r23 zero.
              ret

;***********************
;NAME:        delay_40mS
;ASSUMES:     nothing
;RETURNS:     nothing
;MODIFIES:    R22,R23, SREG
;CALLED BY:   init_dsp, ????
;DESCRIPTION: This procedure will generate a fixed delay of just over
;             40 mS.
;********************************************************************
delay_40mS:   ldi   r22,0        ; load inner loop var
              ldi   r23,4        ; load outer loop var
              rcall  v_delay     ; delay
              ret

;************************
;NAME:       init_spi_lcd
;ASSUMES:    IMPORTANT: PortB set as output (during program init)
;RETURNS:    nothing
;MODIFIES:   DDRB, SPCR
;CALLED BY:  init_dsp, update
;DESCRITION: init SPI port for command and data writes to LCD via SPI
;********************************************************************
init_spi_lcd:
      push r16
      ldi r16,(1<<SPE) | (1<<MSTR) | (1<<CPOL) | (1<<CPHA)
      out SPCR,r16   ; Enable SPI, Master, fck/4, 

      ;kill any spurious data...
      in r16, SPSR   ; clear SPIF bit in SPSR
      in r16, SPDR   ;
      pop r16        ; restore r16 value...
      ret

;********************************
;NAME:       lcd_spi_transmit_CMD
;ASSUMES:    r16 = byte for LCD.
;            SPI port is configured.
;RETURNS:    nothing
;MODIFIES:   R16, PortB, SPCR
;CALLED BY:  init_dsp, update
;DESCRITION: outputs a byte passed in r16 via SPI port. Waits for data
;            to be written by spi port before continuing.
;*********************************************************************
lcd_spi_transmit_CMD:
     push r16          ; save command, need r16.
;     cbi   portB, 3    ; clr PB1 = RS = 0 = command.
     cbi   PORT_RSn, PORT_bit_RSn    ; RS = 0 = command.
     cbi   portB, 4    ; clr PB2 = /SS = selected.
     pop r16           ; restore command
     out SPDR,r16      ; write data to SPI port.

     ;Wait for transmission complete
wait_transmit:
     in r16, SPSR     ; read status reg
     sbrs r16, SPIF   ; if bit 7  = 0 wait
     rjmp wait_transmit
     sbi   portB, 4   ; set PB2 = /SS = deselected
     ret

;*********************************
;NAME:       lcd_spi_transmit_DATA
;ASSUMES:    r16 = byte to transmit to LCD.
;            SPI port is configured.
;RETURNS:    nothing
;MODIFIES:   R16, SPCR
;CALLED BY:  init_dsp, update
;DESCRITION: outputs a byte passed in r16 via SPI port. Waits for
;            data to be written by spi port before continuing.
;*****************************************************************
lcd_spi_transmit_DATA:
     push r16          ; save command, need r16.
;     sbi   portB, 3    ; clr PB1 = RS = 1 = data.
     sbi   PORT_RSn, PORT_bit_RSn    ; RS = 1 = data.
     cbi   portB, 4    ; clr PB2 = /SS = selected.
        in r16, SPSR   ; clear SPIF bit in SPSR.
        in r16, SPDR   ;
     pop r16           ; restore command.
     out SPDR,r16      ; write data to SPI port.

     ;Wait for transmission complete
wait_transmit1:
     in r16, SPSR     ; read status reg
     in r17, SPDR     ; and clear SPIF
     sbrs r16, SPIF   ; if bit 7  = 0 wait
     rjmp wait_transmit1
     sbi   portB, 4   ; set PB2 = /SS = deselected
     ret

;************************
;NAME:       init_lcd_dog
;ASSUMES:    nothing
;RETURNS:    nothing
;MODIFIES:   R16, R17
;CALLED BY:  main application
;DESCRITION: inits DOG module LCD display for SPI (serial) operation.
;NOTE:  Can be used as is with MCU clock speeds of 4MHz or less.
;********************************************************************
; public __version_1 void init_dsp(void)
init_lcd_dog:
       rcall init_spi_lcd   ; init SPI port for DOG LCD.

start_dly_40ms:
       rcall  delay_40mS    ; startup delay.

func_set1:
       ldi    r16,0x39      ; send fuction set #1
       rcall  lcd_spi_transmit_CMD   ;
       rcall  delay_30uS    ; delay for command to be processed

func_set2:
       ldi    r16,0x39      ; send fuction set #2
       rcall  lcd_spi_transmit_CMD
       rcall  delay_30uS    ; delay for command to be processed

bias_set:
       ldi    r16,0x1E      ; set bias value.
       rcall  lcd_spi_transmit_CMD
       rcall  delay_30uS

power_ctrl:
       ldi    r16,0x50      ; ~0x50 for 5VDC, ~55 for 3.3VDC
	                        ; (delicate adjustment).
       rcall  lcd_spi_transmit_CMD
       rcall  delay_30uS

follower_ctrl:
       ldi    r16,0x6C      ; follower mode on...
       rcall  lcd_spi_transmit_CMD
       rcall  delay_40mS

contrast_set:
       ldi    r16,0x77      ; ~77@5VDC, ~7f@3.3VDC
       rcall  lcd_spi_transmit_CMD
       rcall  delay_30uS

display_on:
       ldi    r16,0x0c      ; display on, cursor off, blink off
       rcall  lcd_spi_transmit_CMD
       rcall  delay_30uS

clr_display:
       ldi    r16,0x01      ; clear display, cursor home
       rcall  lcd_spi_transmit_CMD

       rcall  delay_30uS

entry_mode:
       ldi    r16,0x06      ; clear display, cursor home
       rcall  lcd_spi_transmit_CMD;
       rcall  delay_30uS
       ret

;**************************
;NAME:       update_lcd_dog
;ASSUMES:    display buffers loaded with display data
;RETURNS:    nothing
;MODIFIES:   R16,R20,R30,R31,SREG
;
;DESCRITION: Updates the LCD display using the
;  contents of dsp_buff.
;*******************************************************************
; public __version_1 void update_dsp_dog (void)
update_lcd_dog:
       rcall init_spi_lcd    ; init SPI port for LCD.
	   rcall delay_30uS			;delay added 102911 by kls
       ldi   r20,8          ; init 'chars per line' counter.

       ;send line to the LCD module.
wr_line1:
       ldi   ZH, HIGH(dsp_buff)  ; init ptr to line 1 display buffer.
       ldi   ZL, LOW(dsp_buff)   ;
snd_ddram_addr:
       ldi   r16,0x80             ; init DDRAM addr-ctr
       rcall lcd_spi_transmit_CMD ;
       rcall delay_30uS
snd_buff:
       ld    r16, Z+
       rcall lcd_spi_transmit_DATA
       rcall delay_30uS
       dec   r20
       brne  snd_buff

       ret
