;==============================================================================
; "c" - 16-bit Binary to BCD conversion
;
; This subroutine converts a 16-bit number
; (fbinH:fbinL) to a 5-digit
; packed BCD number represented by 3 bytes
; (tBCD2:tBCD1:tBCD0).
; MSD of the 5-digit number is placed in the
; lowermost nibble of tBCD2.
;
; Number of words	:25
; Number of cycles	:751/768 (Min/Max)
; Low registers used	:3 (tBCD0,tBCD1,tBCD2)
; High registers used  :4
; (fbinL,fbinH,cnt16a,tmp16a)
; Pointers used	: Z
;==============================================================================
;**** Subroutine Register Variables
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

;**** Code
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