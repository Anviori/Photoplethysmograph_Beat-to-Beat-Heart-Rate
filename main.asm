;==============================================================================
; Project Title: Beat to Beat Heart Rate Monitor with Autocalibration
; Author: Wilmer Suarez
; Version: 3.0
; Last updated: --/--/19
; Target: ATmega16 @ 1MHz
; Total number of words:
; Total number of cycles:
;
; DESCRIPTION: Using Timer/Counter1, this program measures the period of the
; signal coming from the analog front end circuit. This is done using Input
; Capture. The period is measured and, from it, the frequency is calculated
; and displayed on the LCD Module. This calculated frequency ~= heart reate.
;
; VERSION HISTORY
; 1.0 Base Version
; 2.0 Modified for Auto-Calibration
; 3.0 Optimized Code / Cleanup
;==============================================================================

; Do not show device denition include file(s) in output listfile
.nolist
.include "m16def.inc"
.list
; LCD DOG driver
.include "lcd_dogm08_asm_driver_m16_pin_assign.inc"
; 32-bit division subroutine
.include "div32u.inc"

*** TODO: FIX ALL TITLES v ***
//---------------------------Variables (what type of variables (where?)---------------------------//
.dseg
counting:	   .byte 1 ; flag = 1 when counter is counting
meas_complete: .byte 1 ; flag = 1 when measurement is complete
count_low:	   .byte 1 ; low byte of count read from counter1
count_high:	   .byte 1 ; high byte of count read from counter1

//---------------Vector Table---------------//
.cseg
reset:
.org RESET		   ; resest interrupt vector (Program starts here at reset)
	rjmp start
.org ICP1addr      ; ICP1 interrupt vector
	rjmp count_ISR

start:
//-------------Port Configuration-------------//
	sbi DDRA, 0

	sbi DDRC, 0 ; Port C, Pin0 as output for chip select

	sbi DDRC, 1 ; Port C, Pin1 as output to LED

	ldi r16, $3F  ; PORT D, Pin6 as input of Vout for analog front end circuit
	out DDRD, r16 ; Pin7 = input for pushbutton

	ldi r16, $80   ; Enabling pull-up resistor
	out PORTD, r16

	ldi r16, $B3  ; PORTB as output MISO and both inputs of analog comparator as Input
	out DDRB, r16

//------------Stack Initialization------------//
	ldi r16, LOW(RAMEND)	; load low byte of stack pointer
	out SPL, r16
	ldi r16, HIGH(RAMEND)	; load high byte of stack pointer
	out SPH, r16

//---------Initialize Variable Values---------//
	ldi r16, $00      ; intitialzing counter flag (not counting)
	sts counting, r16
	sts meas_complete, r16 ; intializing meas_complete flag (measurement not complete)
	call init_lcd_dog

//------Configure Input Capture Interrupt------//
//----------Configure Timer/Counter1----------//
	ldi r16, $C4	  ; set to trigger at a rising edge & clkio/256
	out TCCR1B, r16
	ldi r16, 1 << TICIE1; enabling input capture interrupt
	out TIMSK, r16

	sei		; enable gloabl interrupt

main_loop:
//-----------Polling Pushbutton Input-----------//
	sbi PORTC, 0 ; deselect MAX5402

	ldi r16, 0 << TICIE1 ; enabling input capture interrupt
	out TIMSK, r16

	in r16, PIND ; calibrate voltage at Vce if push button at PD7 is pressed
	sbrs r16, 7
	call auto_calibrate

	ldi r16, 1 << TICIE1 ; enabling input capture interrupt
	out TIMSK, r16

	cbi portB, 4 ; deselect DOG LCD

//----------Polling meas_complete----------//
	lds r16, meas_complete
	sbrs r16, 0 	; skip if meas_complete flag is set

	rjmp main_loop

//-----Dividend & Divisor Configuration-----//
	ldi r18, LOW(2343750)
	ldi r19, BYTE2(2343750)
	ldi r20, BYTE3(2343750)
	ldi r21, BYTE4(2343750)

	lds r22, LOW(count_low)
	lds r23, LOW(count_high)
	ldi r24, $00
	ldi r25, $00

	call div32u ; devide 2343750/count

//--------Convert Division Result to BCD--------//

	mov r16, r18
	mov r17, r19

	call bin2BCD16  ; convert current timer count to BCD

//-------------Convert BCD to ASCII-------------//
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

//*---------Load & Update LCD Module---------//
	call load_msg	; load ASCII into Buffer
	call update_lcd_dog ; update the display
						; with whats in the
						; buffer

//------------Clear meas_complete------------//
	ldi r16, $00
	sts meas_complete, r16

//------Enable Input Caputer Interrupt------//
	ldi r16, 1 << TICIE1; enabling input capture interrupt
	out TIMSK, r16

	rjmp main_loop

;==============================================================================
//* "count_ISR"
//*
//* Description: Interrupt service routine, triggered when
//* rising edge is going into ICP1 pin
//*
//*
//* Author:		  Wilmer Suarez
//* Version:      1.0
//* Last updated: 11/29/2016
//* Target:       ATmega16
//* Number of words:
//* Number of cycles:
//* Low registers modified:
//* High registers modified: r16
;==============================================================================
count_ISR:
	push r16
	in r16, SREG
	push r16

	lds r16, counting
	sbrs r16, 0 	; skip if counting flag is set
	rjmp counter_not_running
	rjmp counter_running

//--------------Counting = 0--------------//
counter_not_running:
// stopping counter
	ldi r16, $C0
	out TCCR1B, r16

// clearing counter value
	clr r16
	out TCNT1H, r16
	out TCNT1L, r16

// starting counter
	ldi r16, $C4
	out TCCR1B, r16

// counting flag set
	ldi r16, $01
	sts counting, r16

	pop r16
	out SREG, r16
	pop r16

	reti ; return from ISR

//--------------Counting = 1--------------//
counter_running:
// stopping counter
	ldi r16, $C0
	out TCCR1B, r16

// reading counter value
	in r16, ICR1L
	sts count_low, r16
	in r16, ICR1H
	sts count_high, r16

// counting flag cleared
	ldi r16, $00
	sts counting, r16

// measurement complete variable is set
	ldi r16, $01
	sts meas_complete, r16

// disable Input Capture interrupt
	ldi r16, 0 << TICIE1; enabling input capture interrupt
	out TIMSK, r16

	pop r16
	out SREG, r16
	pop r16

	reti ; return from ISR

//--------------------------------------------
//* "bin2BCD16" - 16-bit Binary to BCD conversion
//*
//* This subroutine converts a 16-bit number
//* (fbinH:fbinL) to a 5-digit
//* packed BCD number represented by 3 bytes
//* (tBCD2:tBCD1:tBCD0).
//* MSD of the 5-digit number is placed in the
//* lowermost nibble of tBCD2.
//*
//* Number of words	:25
//* Number of cycles	:751/768 (Min/Max)
//* Low registers used	:3 (tBCD0,tBCD1,tBCD2)
//* High registers used  :4
//* (fbinL,fbinH,cnt16a,tmp16a)
//* Pointers used	: Z
//--------------------------------------------
//***** Subroutine Register Variables
.equ	AtBCD0	=13		;address of tBCD0
.equ	AtBCD2	=15		;address of tBCD1

.def	tBCD0	=r13		;BCD value digits 1
;and 0
.def	tBCD1	=r14		;BCD value digits
; 3 and 2
.def	tBCD2	=r15		;BCD value digit 4
.def	fbinL	=r16		;binary value Low byte
.def	fbinH	=r17		;binary value High byte
.def	cnt16a	=r18		;loop counter
.def	tmp16a	=r19		;temporary value

//***** Code
bin2BCD16:
	ldi	cnt16a,16	;Init loop counter
	clr	tBCD2		;clear result (3 bytes)
	clr	tBCD1
	clr	tBCD0
	clr	ZH		;clear ZH
	;(not needed for AT90Sxx0x)
bBCDx_1:lsl	fbinL		;shift input value
	rol	fbinH		;through all bytes
	rol	tBCD0		;
	rol	tBCD1
	rol	tBCD2
	dec	cnt16a		;decrement loop counter
	brne	bBCDx_2		;if counter not zero
	ret			;   return

bBCDx_2:ldi	r30,AtBCD2+1	;Z points to result
; MSB + 1
bBCDx_3:
	ld	tmp16a,-Z	;get (Z) with pre-decrement
;---------------------------------------------
;For AT90Sxx0x, substitute the above line with:
;
;	dec	ZL
;	ld	tmp16a,Z
;
;---------------------------------------------
	subi	tmp16a,-$03	;add 0x03
	sbrc	tmp16a,3	;if bit 3 not clear
	st	Z,tmp16a	;	store back
	ld	tmp16a,Z	;get (Z)
	subi	tmp16a,-$30	;add 0x30
	sbrc	tmp16a,7	;if bit 7 not clear
	st	Z,tmp16a	;	store back
	cpi	ZL,AtBCD0	;done all three?
	brne	bBCDx_3		;loop again if not
	rjmp	bBCDx_1

//-------------------------------------
//* Name - "hex2ASCII"
//*
//* Description:
//* Convert DIP switch value to ASCII to
//* place in the buffer
//*
//* Author: Wilmer Suarez
//* Version: 1.0
//* Last updated: 11/06/2016
//* Target: ATmega16 @ 1 MHz
//* Number of words:
//* Number of cycles: 20
//* Low registers used:
//* High registers used: r16,r18
//*
//* Parameters:
//* returns r16 loaded with ASCII
//* form of DIP Switch input
//*
//* Notes:
//-------------------------------------
hex2ASCII:
	cpi r16, 10 ; compare r16 with 10
	brlo zero_9 ; branch to 0_to_9
				; if r16 = $00-$09
	subi r16, 10 ; subtract 10 from r16
	subi r16, -$41 ; add $41 to r16
	ret

	zero_9:
	subi r16, -$30 ; add $30 to r16
	ret

//*******************************************
//* "Subroutine_name" - load_msg
//*
//* Description:
//* Loads a predefined string msg into a
//* specified diplay buffer.
//*
//* Author: Wilmer Suarez
//* Version:
//* Last updated: 11/09/2016
//* Target: ATmega16 @ 1 MHz
//* Number of words:
//* Number of cycles: 10
//* Low registers used:
//* High registers used: r16
//*
//* Parameters:
//*  assumes: Z = offset of message to be loaded.
//*  returns: buffer loaded with message
//*  calls:  none
//*  called by: main program and diagnostics
//*****************************************
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

//*------------------------------------
//* Subroutine Name: "auto_calibrate"
//*
//* Author: Wilmer Suarez
//*
//* Version: 1.0
//*
//* Last Updated:
//* 12/06/2016
//*
//* Target: ATmega16A
//*
//* DESCRIPTION:
//* This subroutine compares the output of a MAX5402 Digital Pot and
//* the output of a voltage divider.
//*------------------------------------
auto_calibrate:
	sbi portB, 4 ; set /SS of DOG LCD = 1 (Deselected)
	cbi PORTC, 1 ; turn OFF LED

//---Configuring SPI Control Register---//
    ldi r16, $50  ; Enable SPI
	out SPCR,r16  ; Data Order = MSB first
	ldi r16, (1 << SPI2X) ; Set as Master
	out SPSR, r16 ; SCK = fosc/2

//-------Clear SPI Interrupt Flag-------//
    in r16, SPSR
	in r16, SPDR

//--------Configure Analog Comparator--------//
	ldi r16, $00
	out ACSR, r16
	ldi r16, $00 ; Initial Value for MAX5402 wiper position

//------------Send Data to Slave------------//
send_data:
	cbi PORTC, 0  ; select MAX5402
	out SPDR, r16 ; send data to the SPI Data Register

//------Wait for Data to be Transfered------//
wait:
	sbis SPSR, SPIF
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
