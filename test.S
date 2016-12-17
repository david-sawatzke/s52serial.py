	;;  A simple test file to let an LED blink
	.org 0000h
start:  CLR P0.0
	ACALL delay
	SETB P0.0
	ACALL delay
	AJMP start
delay:	mov r0,	#0x6
d1:	mov r1, #0xff
d2:	mov r2, #0xff
d3:	djnz r2, d3
	djnz r1, d2
	djnz r0, d1
	RET
