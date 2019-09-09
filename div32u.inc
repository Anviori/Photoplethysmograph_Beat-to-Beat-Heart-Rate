;*******************************************
;* "div32u" - 32/32 Bit Unsigned Division
;*
;* Ken Short
;*
;* This subroutine divides the two 32-bit numbers
;* "dd32u3:dd32u2:dd32u1:dd32u0" (dividend)
;* and
;* "dv32u3:dv32u2:dv32u1:dv32u0" (divisor).
;* The result is placed in
;* "dres32u3:dres32u2:dres32u1:dres32u0"
;* and the remainder in
;* "drem32u3:drem32u2:drem32u1:drem32u0"
;*
;* Number of words:
;* Number of cycles: 655/751 (Min/Max) ATmega16
;* #Low registers used: 2 (drem16uL,drem16uH)
;* #High registers used: 5 (dres16uL/dd16uL
;* ,dres16uH/dd16uH,dv16uL,dv16uH, dcnt16u)
;* A $0000 divisor returns $FFFF
;******************************************

;***** Subroutine Register Variables
.def	drem32u0 =r12    ;remainder
.def	drem32u1 =r13
.def	drem32u2 =r14
.def	drem32u3 =r15

.def	dres32u0 =r18    ;result (quotient)
.def	dres32u1 =r19
.def	dres32u2 =r20
.def	dres32u3 =r21

.def	dd32u0	=r18    ;dividend
.def	dd32u1	=r19
.def	dd32u2	=r20
.def	dd32u3	=r21

.def	dv32u0	=r22    ;divisor
.def	dv32u1	=r23
.def	dv32u2	=r24
.def	dv32u3	=r25

.def	dcnt32u	=r17

;***** Code

div32u:
	clr	drem32u0	;clear remainder Low byte
    clr drem32u1
    clr drem32u2
	sub	drem32u3,drem32u3; clear remainder
	                     ; High byte and
						 ; carry
	ldi	dcnt32u,33	;init loop counter
d32u_1:
	rol	dd32u0		;shift left dividend
	rol	dd32u1
	rol	dd32u2
	rol	dd32u3
	dec	dcnt32u		;decrement counter
	brne	d32u_2		;if done
	ret			;    return
d32u_2:
	rol	drem32u0	;shift dividend into remainder
    rol	drem32u1
    rol	drem32u2
	rol	drem32u3

	sub	drem32u0,dv32u0	;remainder = remainder - divisor
    sbc	drem32u1,dv32u1
    sbc	drem32u2,dv32u2
	sbc	drem32u3,dv32u3	;
	brcc	d32u_3		; branch if reult is
	                    ; pos or zero

	add	drem32u0,dv32u0	; if result negative
	                    ; restore remainder
	adc	drem32u1,dv32u1
	adc	drem32u2,dv32u2
	adc	drem32u3,dv32u3
	clc			; clear carry to be shifted
	            ; into result
	rjmp	d32u_1		; else
d32u_3:	sec			    ; set carry to be
                        ; shifted into
						; result
	rjmp	d32u_1
