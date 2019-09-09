;==============================================================================
; Project Title: Beat to Beat Heart Rate Monitor with Autocalibration
; Author: Wilmer Suarez
; Version: 3.0
; Last updated: 09/07/19
; Target: ATmega16 @ 1MHz
; Total number of words:
; Total number of cycles:
;
; DESCRIPTION: Using Timer/Counter1's Input Capture, this program measures the
; period of the signal coming from the analog front end circuit. The period is
; measured and, from it, the frequency is calculated and displayed on the LCD
; Module. The calculated frequency == heart reate.
;
; VERSION HISTORY
; 1.0 Base Version
; 2.0 Modified for Auto-Calibration
; 3.0 Optimized Code / Cleanup
;==============================================================================
; Do not show device register/bit definitions include file in output listfile
.nolist
.include "m16def.inc"
.list
.include "lcd_dogm08_asm_driver_m16_pin_assign.inc" ; LCD DOG driver
.include "div32u.inc" ; 32-bit division subroutine
.include "bin2BCD16.inc" ; C

.dseg ; In SRAM
counting:	   .byte 1 ; flag == 1 when counter is counting
meas_complete: .byte 1 ; flag == 1 when measurement is complete
count_low:	   .byte 1 ; low byte of count read from counter1 (16-bit counter)
count_high:	   .byte 1 ; high byte of count read from counter1 (16-bit counter)

;===============INTERRUPT VECTOR TABLE===============;
.cseg ; In Program Memory

; This constant is used in the calculation to convert the measured period of the PPG signal
; to cycles per minute (cpm)(the heart rate), with a precision of 0.1.
; Constant derivation:
;	F(0.1cpm) = 60/(period_count)*256us --> F = 600/(period_count)*256us -->
;	F = 600000000/(period_count)*256 --> F = 2343750/(period_count)
.equ CPM_CONSTANT 0x23C346

reset:
.org RESET		   ; "Resest" interrupt vector (Program starts here at reset)
	rjmp start
.org ICP1addr      ; Timer/Counter1 Input Capture interrupt vector
	rjmp count_ISR

start:
	;===============PORT CONFIGURATION===============;
	sbi DDRA, 0 ; Port A, Pin0 = LCD module, register select pin

	sbi DDRC, 0 ; Port C, Pin0 as output for chip select
	sbi DDRC, 1 ; Port C, Pin1 as output to LED

	ldi r16, $3F   ; PORT D, Pin6 as input for "Vout" signal from analog front end circuit
				   ; (photoplethysmogram - PPG)
	out DDRD, r16  ; Pin7 = input for Calibration-start pushbutton
	ldi r16, $80   ; Enabling pull-up resistor for pushbutton
	out PORTD, r16

	ldi r16, $B3  ; Configure PORTB SPI pins (for LCD module connections)
				  ; and both inputs of the analog comparator
	out DDRB, r16

	;===============STACK INITIALIZATION===============;
	ldi r16, LOW(RAMEND)	; load low byte of stack pointer
	out SPL, r16
	ldi r16, HIGH(RAMEND)	; load high byte of stack pointer
	out SPH, r16

	;===============VARIABLE INITIALIZATION===============;
	ldi r16, $00
	sts counting, r16	   ; intitialzing counter flag (0 == not counting)
	sts meas_complete, r16 ; intializing meas_complete flag (0 == measurement not complete)

	;===============INITIALIZE LCD DOG MODULE===============;
	call init_lcd_dog

	;===============CONFIGURE INPUTER CAPTURE INTERRUPT & TIMER/COUNTER1 ===============;
	; Enable Input Capture noise canceler | Input capture triggers at rising edge |
	; Counter clock = clkio/256 (from prescaler)
	ldi r16, (1 << ICNC1) | (1 << ICES1) | (1 << CS12)
	out TCCR1B, r16
	ldi r16, 1 << TICIE1 ; enable input capture interrupt for Timer/Counter 1
	out TIMSK, r16

	sei ; enable gloabl interrupt

main_loop:
	sbi PORTC, 0 ; deselect MAX5402 digital pot

	ldi r16, 0 << TICIE1 ; Disable input capture interrupt (while system is autocalibrating)
	out TIMSK, r16

	;===============AUTOCALIBRATION===============;
	; in r16, PIND ; calibrate voltage at Vce if push button at PD7 is pressed
	; sbrs r16, 7
	; call auto_calibrate

	ldi r16, 1 << TICIE1 ; Re-enable input capture interrupt
	out TIMSK, r16

	cbi portB, 4 ; deselect DOG LCD module

	;===============CHECK IF PERIOD MEASUREMENT IS COMPLETE===============;
	lds r16, meas_complete
	sbrs r16, 0 	; continue with calculation if meas_complete flag is set
	rjmp main_loop

	;===============CONVERT PERIOD TO ===============;
	;r21:r20:r19:r18 = Dividend = 0x23C346
	ldi r18, LOW(CPM_CONSTANT)
	ldi r19, BYTE2(CPM_CONSTANT)
	ldi r20, BYTE3(CPM_CONSTANT)
	ldi r21, BYTE4(CPM_CONSTANT)

	;r25:r24:r23:r22 = Divisor = count captured (period of PPG)
	lds r22, LOW(count_low)
	lds r23, LOW(count_high)
	ldi r24, $00
	ldi r25, $00

	call div32u ; devide 2343750/count

	;===============CONVERT QUOTIENT TO BCD===============;
	mov r16, r18
	mov r17, r19

	call bin2BCD16

	; TODO: Modify hex2ASCII to convert from BCD to ASCII instead
	; So technically decimal to ascii
	;===============CONVERT BCD TO ASCII===============;
	mov r25, r14
	lsr r14
	lsr r14
	lsr r14
	lsr r14
	mov r16, r14
	call hex2ASCII
	mov r23, r16

	andi r25, $0F
	mov r16, r25
	call hex2ASCII
	mov r24, r16

	mov r25, r13
	lsr r13
	lsr r13
	lsr r13
	lsr r13
	mov r16, r13
	call hex2ASCII
	mov r20, r16

	andi r25, $0F
	mov r16, r25
	call hex2ASCII
	mov r21, r16

	mov r16, r12
	ldi r17, $00

	call bin2BCD16

	mov r25, r13

	lsr r13
	lsr r13
	lsr r13
	lsr r13
	mov r16, r13
	call hex2ASCII
	mov r15, r16

	andi r25, $0F
	mov r16, r25
	call hex2ASCII
	mov r14, r16

	ldi r16, $00
	call hex2ASCII
	mov r13, r16

	;===============UPDATE LCD MODULE===============;
	call load_msg		; load ASCII characters into buffer
	call update_lcd_dog ; update the display with buffer contents

	;===============READY FLAGS AND INPUT CAPTURE FOR NEXT MEASUREMENT===============;
	ldi r16, $00		   ; reset measurement complete flag
	sts meas_complete, r16
	ldi r16, 1 << TICIE1   ; re-enable input capture interrupt
	out TIMSK, r16

	rjmp main_loop

;==============================================================================
; Subroutine name: "count_ISR"
;
; Description: Interrupt service routine, triggered when
; rising edge is going into Timer/Counter1's Input Capture pin (ICPT1)
;
; Author:	    Wilmer Suarez
; Version:      1.0
; Last updated: 11/29/2016
; Target:       ATmega16
; Number of words:
; Number of cycles:
; Low registers modified:
; High registers modified: r16
;==============================================================================
count_ISR:
	; Reserve used register values in stack
	push r16
	in r16, SREG
	push r16

	lds r16, counting
	sbrs r16, 0 	; skip if counting flag is set
	rjmp counter_not_running
	rjmp counter_running

	;===============COUNTING == 0===============;
counter_not_running:
	;===============CLEAR OLD COUNTER VALUE===============;
	clr r16
	out TCNT1H, r16
	out TCNT1L, r16

	;===============START COUNTER===============;
	ldi r16, (1 << ICNC1) | (1 << ICES1) | (1 << CS12)
	out TCCR1B, r16
	ldi r16, $01		; Set counting flag
	sts counting, r16

	rjmp count_ISR_done

	;===============COUNTING == 1===============;
counter_running:
	;===============STOP COUNTER===============;
	ldi r16, (1 << ICNC1) | (1 << ICES1)
	out TCCR1B, r16
	ldi r16, $00		; clear counting flag
	sts counting, r16

	;===============READ COUTNER VALUE===============;
	in r16, ICR1L		; Low byte must be read first
	sts count_low, r16
	in r16, ICR1H
	sts count_high, r16

	ldi r16, $01			; set measurement complete flag
	sts meas_complete, r16

	;===============DISABLE INPUT CAPTURE INTERRUPT===============;
	ldi r16, 0 << TICIE1; enabling input capture interrupt
	out TIMSK, r16

count_ISR_done:
	; Restore old register values from stack
	pop r16
	out SREG, r16
	pop r16

	reti

;==============================================================================
; Subroutine name: "hex2ASCII"
;
; Description:
; Convert DIP switch value to ASCII to
; place in the buffer
;
; Author: Wilmer Suarez
; Version: 1.0
; Last updated: 11/06/2016
; Target: ATmega16 @ 1 MHz
; Number of words:
; Number of cycles:
; Low registers used:
; High registers used:
;
; Notes:
;==============================================================================
hex2ASCII:
	cpi r16, 10 ; compare r16 with 10
	brlo zero_9 ; branch to 0_to_9 if r16 < 10 (0 - 9)
	subi r16, 10 ; subtract 10 from r16
	subi r16, -$41 ; add $41 to r16
	ret

	zero_9:
	subi r16, -$30 ; add $30 to r16
	ret

;==============================================================================
; Subroutine name: "load_msg"
;
; Description: Loads data into the diplay buffer.
;
; Author: Wilmer Suarez
; Version: 1.0
; Last updated: 11/09/2016
; Target: ATmega16 @ 1 MHz
; Number of words:
; Number of cycles:
; Low registers used:
; High registers used: r16
;
; Notes:
;==============================================================================
load_msg:
	sts dsp_buff + 0, r23 ; 1st digit
	sts dsp_buff + 1, r24 ; 2nd digit
	sts dsp_buff + 2, r20 ; 3rd digit
	ldi r16, $2E
	sts dsp_buff + 3, r16 ; decimal period
	sts dsp_buff + 4, r21 ; 1st decimal digit
	sts dsp_buff + 5, r15 ; 2nd decimal digit
	ldi r16, $20
	sts dsp_buff + 6, r16 ; space
	sts dsp_buff + 7, r16 ; space

	ret

;==============================================================================
; Subroutine Name: "auto_calibrate"
;
; Description: This subroutine compares the output of a MAX5402 Digital
; Potentiometer and the output of a voltage divider.
;
; Author: Wilmer Suarez
; Version: 1.0
; Last Updated: 12/06/2016
; Target: ATmega16A
; Number of words:
; Number of cycles:
; Low registers used:
; High registers used:
;
; Notes:
;==============================================================================
auto_calibrate:
	sbi PORTB, 4	; deselect LCD module
	cbi PORTC, 1	; turn OFF LED

	;===============CONFIGURE SPI CONTROL REGISTER===============;
	; Enable SPI | Data Order: MSB first | Set MCU as Master
    ldi r16, (1 << SPE) | (1 << MSTR)
	out SPCR,r16
	ldi r16, (1 << SPI2X)	; SCK = fosc/2
	out SPSR, r16

	;===============CLEAR SPI INTERRUPT FLAG===============;
    in r16, SPSR
	in r16, SPDR

	;===============CONFIGURE ANALOG COMPARATOR===============;
	ldi r16, $00
	out ACSR, r16
	ldi r16, $00	; Initial Value for MAX5402 wiper position

	;===============SEND DATA TO DIGITAL POT (CURRENT SLAVE)===============;
send_data:
	cbi PORTC, 0	; select MAX5402
	out SPDR, r16	; send wiper data to the SPI Data Register

wait:
	sbis SPSR, SPIF	; Wait for data transfer to complete
	rjmp wait

	sbi PORTC, 0   ; deselect MAX5402

	inc r16		   ; compare to max value
	SBIC ACSR, ACO ; If ACO = 0 skip next instruction
	rjmp send_data ; Digital pot wiper = set point voltage = Vce of the
				   ; LTR-3208E

	sbi PORTC, 1

	inc r16		   ; compare to max value
	SBIC ACSR, ACO ; If ACO = 0 skip next
	rjmp send_data ; instruction = Digital
				   ; pot wiper = set point
				   ; voltage = Vce of the
				   ; LTR-3208E

	sbi PORTC, 1

	ret
