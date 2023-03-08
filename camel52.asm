	.include	"macros.asm"
; ===============================================
; CamelForth for the Intel 8052 (internal RAM)
; (c) 1994,1997,2000 Bradford J. Rodriguez
; Permission is granted to freely copy, modify,
; and distribute this program for personal or
; educational use.  Commercial inquiries should
; be directed to the author at 115 First St.,
; #105, Collingwood, Ontario L9Y 4W3 Canada
;
; CAMEL52.ASM: Code Primitives
;   Source code is for the Metalink ASM51 assembler.
;   Forth words are documented as follows:
;x   NAME     stack -- stack    description
;   where x=C for ANS Forth Core words, X for ANS
;   Extensions, Z for internal or private words.
;
; Subroutine-Threaded Forth model for Intel 8052
; 16 bit cell, 8 bit char, 8 bit (byte) adrs unit
; split Code & Data spaces; Data in internal RAM
;    8051 PC = Forth IP  Interpreter Pointer
;         SP =       RSP Return Stack Pointer low
;                    RSP high byte = 0
;         R0 =       PSP Parameter Stack Ptr low
;                    PSP high = 0
;       DPTR =       TOS (top Param. Stack item)
;  A,B,R1-R5 =       temporaries
;                    (no W register is defined)
;      R6,R7 =       loop index
; ===============================================
; REVISION HISTORY
; $Log: Camel52.asm,v $
; 14 Mar 2001 - added INCLUDES for experimental code:
;       Tovalue.asm, Code.asm, Sfraccess.asm.
;       Added 'CYGNAL' configuration equate.
;       Modified Cygnal initialization code for generic 8052;
;       added dummy Flash ROM routines for generic 8052.
;       Modified UINIT to use table at end of ROM.
;
; Revision 1.3  2000/10/24 19:03:18  brad
; First release of CamelForth for Cygnal C8051F.
; 256 bytes internal RAM with 32K Flash ROM.
;
; 19 Oct 2000 - revised HIDE, REVEAL to work with Flash ROM.

; 13 Oct 2000 - added Cygnal C8051F000 initialization.
;       Converted to Metalink ASM51 assembler.
;       Added IALIGN; modified MARKER to align & erase flash ROM.
;       Changed >IN, BASE, STATE, DP, HP, LP, 'SOURCE to Byte size.
;       Added CPLUSSTORE, TWOCFETCH, TWOCSTORE, CVARIABLE.
;       Added SAVE-RAM, RESTORE-RAM; removed UINIT, #INIT.
;       Modified COLD to autoexec latest word; added MAIN.
;
; Revision 1.2  2000/10/12 13:57:38  brad
; Modified for flash ROM program storage;
; added support for Cygnal C8051F000 flash ROM.
;
; 11 Oct 2000 - added Flash ROM support.
;       Changed IMMED flag to FEh, nonimmmediate default to FFh;
;       modified CREATE, IMMEDIATE, IMMED? accordingly.
;       Added ,NODEST to compile empty forward branch;
;       modified IF, ELSE, LEAVE, ?DO to use ,NODEST.
;	Added ,NOCF to compile empty code field, and
;	added <BUILDS to create word with empty code field
;       and empty parameter field (no RAM address);
;       modified CONSTANT, USER, :, MARKER to use <BUILDS.
;       Added +WRITE, -WRITE, +ERASE, -ERASE, WIPE for Cygnal flash ROM.
;
; Revision 1.1  2000/10/11 15:47:16  brad
; CamelForth/8052 using internal RAM.
; Based on revision 1.6 of CamelForth/8051.
;
; v1.1  First version of internal-RAM CamelForth, 7 Oct 2000
; 8052  based on Revision 1.6 of CAMEL51.ASM.
;       Converted primitives to use internal RAM.
;       Removed multitasker.
;       Added IDOES>, MARKER.
;
; Revision 1.6  1999/10/17 20:50:20  brad
; Fixed name length in IC@.  17 oct 99
; Fixed FM/MOD (again).
;
; Revision 1.5  1997/05/28 23:03:52  brad
; v1.5  Added multitasker words.       28 Mar 97
;       Corrected memory map comments.
;       Fixed UM* bug.
;       Fixed FM/MOD bug, per Ed Smeda (thanks!)
;       Fixed >BODY to return Data adrs, per ANS.
;       Moved SWAP from REPEAT to WHILE, per ANS.
;	ABORT" ?ABORT now use IS" and ITYPE.
;       Fixed WORDS to ignore smudge bit.
;       Fixed IWORD to return Code address.
;       Renamed ROMDICT,RAMDICT to CODERAM,DATARAM
; v1.4  changed QUIT to type CR even    9 Nov 96
;       if STATE<>0, to support downloading.
;       Added example I/O.
; v1.3  changed PLUSLOOPSENSE to       17 Mar 96
;       PLUSLPSENSE (for some A51 assemblers)
; v1.2  fixed names of KEY? and S>D
; v1.1  bug fixes for split I & D mem  14 Mar 95
;       in COLD: changed CMOVE to I->D
;       in XISQUOTE: changed COUNT to ICOUNT
;       in LIT: uses MOVC instead of MOVX
;       in FIND:  changed NFA>LFA @ to NFA>LFA I@
;       in QUIT:  changed TYPE to ITYPE
; v1.0  alpha test version, 12 Dec 94
; ===============================================
; CONFIGURATION EQUATES
	.equ	CYGNAL,0	; 1=Cygnal C8051F000; 0=generic 8052

; Forth linkage
	.equ	link,0
	.equ	IMMED,0xFE	; flag for Immediate word
	.equ	NONIMMED,0xFF

; 8051 EQUATES
;$MOD8F000			; Include regsiter definition file.
	.equ	dr1,0x01
	.equ	dr2,0x02	; r2-r5 accessed as
	.equ	dr3,0x03	; direct registers;
	.equ	dr4,0x04	; required for PUSH and
	.equ	dr5,0x05	; POP instructions.
	.equ	dr6,0x06	; Assumes register bank 0
	.equ	dr7,0x07	; is selected.

; FORTH MEMORY MAP EQUATES

; Internal RAM allocation:
;  08-13 = User (system) variables, 12 bytes
;  14-7F = Application variables (available to user)
;  80-89 = Leave stack, 10 bytes, grows up  shared with HOLD
;  80-89 = HOLD area, grows down from PAD, 10 bytes
;  8A    = PAD buffer, for end user, ZERO bytes
;  8A-B1 = Terminal Input Buffer, 40 bytes
;  B2-D9 = return stack, 40 bytes, grows up
;  DA-FF = parameter stack, 38 bytes, grows down
;
; See also the definitions of U0, S0, and R0
; in the "system variables & constants" area.

	.equ	userarea,0x08	; base of user area
	.equ	dataram,0x14	; where data goes (internal RAM)

	.equ	lstack,0x80
	.equ	padbufr,0x8A	; NO PAD AREA IS DEFINED FOR THE USER
	.equ	termbufr,0x8A	; base of terminal input buffer
	.equ	rstack,0xB2	; initial return stack pointer
	.equ	pstack,0xff	; initial data stack pointer    *** FIX .S,DEPTH ***
		; first decrement will wrap to 0FFh

; External Memory map:
;   0000h-1FFFh Forth kernel
;   E000h       Forth dictionary (writable Code Space)

; Initial RAM & ROM pointers for CamelForth.
	.if CYGNAL
	.equ	coderam,0x2000	; where new code goes  (Cygnal)
	.else
	.equ	coderam,0xE000	; where new code goes  (generic 8052)
	.endif

	.area	CODE(ABS)
; RESET AND INTERRUPT VECTORS ===================
	ljmp reset
	ljmp ie0svc
ie0svc: reti
	.db	0xff,0xff,0xff,0xff
	ljmp tf0svc
tf0svc: reti
	.db	0xff,0xff,0xff,0xff
	ljmp ie1svc
ie1svc: reti
	.db	0xff,0xff,0xff,0xff
	ljmp tf1svc
tf1svc: reti
	.db	0xff,0xff,0xff,0xff
	ljmp ritisvc
ritisvc:	reti
	.db	0xff,0xff,0xff,0xff
	ljmp tf2svc
tf2svc: reti

;-----------------------------------------------------------------------------
; RESET INITIALIZATION CODE
;-----------------------------------------------------------------------------

	.if CYGNAL
	.equ	GREEN_LED,P1.6	; Port I/O pin connected to Green LED.

	.org	0xB3	; End of interrupt vector space. (Cygnal)
reset:	; Cygnal C8051F initialization
	; Disable the WDT. (IRQs not enabled at this point.)
	; If interrupts were enabled, we would need to explicitly disable
	; them so that the 2nd move to WDTCN occurs no more than four clock
	; cycles after the first move to WDTCN.
	mov WDTCN,#0xDE
	mov WDTCN,#0xAD
	; Start up the XTAL oscillator (but don't use it yet!)
	mov OSCXCN,#0b01100101

	; Enable I/O functions: I2C, SPI, UART, 4 captures
	mov XBR0,#0b00100111
	; Enable I/O functions: T2, SYSCLK
	mov XBR1,#0b10100000
	; Enable the Port I/O Crossbar & weak pull ups
	mov XBR2,#0x40
	; P0.0 SDA ( I2C )
	; P0.1 SCL
	; P0.2 SCK ( SPI )
	; P0.3 MISO
	; P0.4 MOSI
	; P0.5 NSS
	; P0.6 TX  ( UART)
	; P0.7 RX
	; P1.0 CEX0 ( PCA captures)
	; P1.1 CEX1
	; P1.2 CEX2
	; P1.3 CEX3
	; P1.4 T2
	; P1.5 /SYSCLK

	mov PRT0CF,#0b01010110	; TX, MOSI, SCK & SC all outputs
	mov PRT1CF,#0b01110001	; CEX0 & green LED outputs

	; Initialize LED to OFF
	clr GREEN_LED
	; UART setup
	mov RCAP2L,#0xFA	; Autoload value for 57.6Kbaud
	mov RCAP2H,#0xFF	; with 11.0592 MHz XTAL
	orl T2CON,#0b00110100	; T2 is Baudrate gen. RCLK & TCLK =1
	orl SCON,#0b01110000		; Serial Mode 1
	; Wait for XTAL stable and valid

osc_wait:
	mov a,OSCXCN
	jnb acc.7, osc_wait
	orl OSCICN,#0b00001000	;select extn XTAL osc as sys clock
	anl OSCICN,#~0b00000100	; disable internal oscillator

	; Flash Memory Timing
	mov FLSCL,#0x88	; 6.4 MHz <= System Clock < 12.8 MHz

	.else

reset:	; Generic 8051 initialization
	mov ie,#0	; disable all irpts
	mov pcon,#0	; T1 baudrate not doubled
	mov tmod,#0x20	; T1 mode 2, T0 mode 0
	mov th1,#0xfd	; 9600 baud @ 11.0592 MHz
	setb tcon.6	; enable timer 1
	mov scon,#0x52	; UART mode 1 (8-bit)

	.endif		; CYGNAL

; CamelForth entry

	setb scon.1	; set TX flag to allow 1st EMIT

	mov r0,#pstack	; start addr of param stack
	mov sp,#rstack	; start addr of return stack
	ljmp COLD	; enter Forth interpreter

; SERIAL I/O ====================================
;C EMIT     c --      output character to console
	.drw link
	.link
	.fw	EMIT
EMIT:	jnb scon.1,EMIT ; await Tx interrupt flag
	clr scon.1	; clear flag
	mov sbuf,dpl	; output TOS char to UART
	ajmp poptos	; pop new TOS

;C KEY      -- c      get character from keyboard
	.drw link
	.link
	.fw	KEY
KEY:	jnb scon.0,KEY	; await Rx interrupt flag
	clr scon.0
	dec r0		; push old TOS
	mov @r0,dph
	dec r0
	mov @r0,dpl
	mov dpl,sbuf	; get new char in TOS
	mov dph,#0
	ret

;X KEY?      -- f      return true if char waiting
	.drw link
	.link
	.fw	KEY?
QUERYKEY:	dec r0	; push old TOS
	mov @r0,dph
	dec r0
	mov @r0,dpl
	mov a,scon	; get rx flag in carry
	rrc a
	ajmp cyprop	; propagate that thru TOS

; INTERPRETER LOGIC =============================

; NEXT and ENTER are not needed for Subroutine
; Threading. EXIT may be used in high level code.

;C EXIT     --            exit a colon definition
	.drw link
	.link
	.fw	EXIT
EXIT:	dec sp		; discard ret adrs in caller
	dec sp
	ret		; return to caller's caller

;Z LIT      -- x    fetch inline literal to stack
	.drw link
	.link
	.fw	LIT
LIT:	dec r0		; push old TOS
	mov @r0,dph
	dec r0
	mov @r0,dpl
	pop dph		; get return address
	pop dpl
	clr a
	movc a,@a+dptr	; get literal low byte
	inc dptr
	mov r2,a
	clr a
	movc a,@a+dptr	; get literal high byte
	inc dptr
	push dpl	; restore updated ret adr
	push dph
	mov dph,a	; put literal in TOS
	mov dpl,r2
	ret

;C EXECUTE  i*x xt -- j*x      execute Forth word
;C                             at 'xt'
	.drw link
	.link
	.fw	EXECUTE
EXECUTE:	push dpl	; push addr onto r.stack,
	push dph	; then pop new TOS->DPTR
	; 'ret' in poptos will then execute
	; desired word; its 'ret' will return to
	; EXECUTE's caller.
	ajmp poptos

; DEFINING WORDS ================================

;C VARIABLE --            define a Forth VARIABLE
;   CREATE CELL ALLOT ;
; Action of ROMable variable is that of CONSTANT;
; the constant holds the RAM address.
	.drw link
	.link
	.fw	VARIABLE
VARIABLE:	lcall CREATE
	lcall CELL
	ljmp ALLOT

;Z CVARIABLE --            define a Forth VARIABLE
;   CREATE 1 ALLOT ;
; Action of ROMable variable is that of CONSTANT;
; the constant holds the RAM address.
	.drw link
	.link
	.fw	CVARIABLE
CVARIABLE:	lcall CREATE
	acall LIT
	.drw  1
	ljmp ALLOT

;C CONSTANT --            define a Forth constant
;   <BUILDS  I,	Harvard model, 8052
;   DOES> (machine code fragment)
; Note that the constant is stored in Code space.
	.drw link
	.link
	.fw	CONSTANT
CONSTANT:	lcall BUILDS
	lcall ICOMMA
	lcall XDOES
; DOCON, code action of CONSTANT,
; entered by CALL DOCON
docon:			; -- x         exec action of constant
dovar:			; -- a-addr    exec action of ROMable var
docreate:		; -- a-addr  exec action of Harv.CREATE
	dec r0		; push old TOS
	mov @r0,dph
	dec r0
	mov @r0,dpl
	pop dph		; get addr of param field
	pop dpl		;     (in Code memory!)
	ajmp IFETCH	; go fetch its contents

;Z USER     n --         define user variable 'n'
;   <BUILDS I,   Harvard model, 8052
;   DOES> (machine code fragment)
; Note that this version allows only an 8-bit
; offset from a fixed user area.
	.drw link
	.link
	.fw	USER
USER:	lcall BUILDS
	lcall ICOMMA
	lcall XDOES
; DOUSER, code action of USER,
; entered by CALL DOUSER
douser:	acall pushtos	; push old TOS
	pop dph		; get addr of param field
	pop dpl		;     (in Code memory!)
	acall IFETCH	; go fetch its contents
	mov a,dpl	; add to starting addr of user area
	add a,#userarea
	mov dpl,a
	mov a,dph
	addc a,#0
	mov dph,a
	ret

; DOCREATE's action is for a table in RAM.
; DOROM is the code action for a table in ROM;
; it returns the address of the parameter field.
; Entered by CALL DOROM
dorom:	acall pushtos	; push old TOS
	pop dph		; param field adrs -> TOS
	pop dpl
	ret

; DODOES, code action of DOES> clause
; (internal code fragment, not a Forth word)
; entered by       LCALL fragment
;                  address of data
;                       ...
;        fragment: LCALL DODOES
;                  high-level thread
; Enters high-level thread with address of
; data on top of stack.  HARVARD MODEL: the data
; (in Data space) does NOT follow LCALL fragment
; (in Code space); instead, the address of the
; data is appended after LCALL fragment.
dodoes:	; -- a-addr     support routine for DOES>
	dec r0		; push old TOS
	mov @r0,dph
	dec r0
	mov @r0,dpl
	pop dr5		; addr of DOES> clause
	pop dr4		;   Forth code
	pop dph		; addr of defined word's
	pop dpl		;   Param. field
	push dr4	; restore Forth code addr
	push dr5
	ajmp IFETCH	; fetch adrs from P.field
			;  & go do the DOES> code

; STACK OPERATIONS ==============================

;C DUP      x -- x x       duplicate top of stack
	.drw link
	.link
	.fw	DUP
DUP:
pushtos:	dec r0	; push hi byte of TOS
	mov @r0,dph
	dec r0
	mov @r0,dpl
	ret

;C ?DUP     x -- 0 | x x           DUP if nonzero
	.drw link
	.link
	.fw	?DUP
QDUP:	mov a,dph
	orl a,dpl
	jnz pushtos
	ret

;C DROP     x --                drop top of stack
	.drw link
	.link
	.fw	DROP
DROP:
poptos:	mov dpl,@r0	; pop lo byte -> TOS
	inc r0
	mov dph,@r0	; pop hi byte -> TOS
	inc r0
	ret

;C SWAP     x1 x2 -- x2 x1     swap top two items
	.drw link
	.link
	.fw	SWAP
SWOP:	mov dr2,@r0	; pop lo byte -> X
	inc r0
	mov dr3,@r0	; pop hi byte -> X
	; inc r0
	; dec r0	; push hi byte of TOS
halfover:	mov @r0,dph
	dec r0		; push lo byte of TOS
	mov @r0,dpl
	mov dph,r3	; old 2nd item -> TOS
	mov dpl,r2
	ret

;C OVER     x1 x2 -- x1 x2 x1   per stack diagram
	.drw link
	.link
	.fw	OVER
OVER:	mov dr2,@r0	; pop lo byte -> X
	inc r0
	mov dr3,@r0	; pop hi byte -> X
	dec r0		; restore stack pointer
	dec r0		; predecrement for 'halfover'
	sjmp halfover	; push TOS, then copy X to TOS

;C ROT     x1 x2 x3 -- x2 x3 x1 per stack diagram
	.drw link
	.link
	.fw	ROT
ROT:	; x3 is in TOS
	mov dr4,@r0	; pop x2 -> r5:r4
	inc r0
	mov dr5,@r0
	inc r0
	mov dr2,@r0	; pop x1 -> r3:r2
	inc r0
	mov dr3,@r0
	; inc r0
	; dec r0	; push x2
	mov @r0,dr5
	dec r0
	mov @r0,dr4
	dec r0		; predecr. for 'halfover'
	sjmp halfover	; push x3 (TOS), then
			;    copy x1 to TOS

;C >R       x --   R: -- x   push to return stack
	.drw link
	.link
	.fw	>R
TOR:	pop dr3		; save ret addr in r3:r2
	pop dr2
	push dpl	; push lo byte*
	push dph	; push hi byte*
	push dr2	; restore ret addr
	push dr3
	sjmp poptos	; pop new TOS
;* NB. stored lo:hi in regs because SP increments

;C R>       -- x   R: x --  pop from return stack
	.drw link
	.link
	.fw	R>
RFROM:	dec r0	; push old TOS
	mov @r0,dph
	dec r0
	mov @r0,dpl
	pop dr3		; save ret addr in r3:r2
	pop dr2
	pop dph		; pop hi byte
	pop dpl		; pop lo byte
	push dr2	; restore return address
	push dr3
	ret

;C R@       -- x  R: x -- x  fetch from rtn stack
	.drw link
	.link
	.fw	R@
RFETCH:	dec r0		; push old TOS
	mov @r0,dph
	dec r0
	mov @r0,dpl
	mov r1,sp	; get copy of SP
	dec r1		; skip return address
	dec r1
	mov dph,@r1	; fetch 2nd return stack item
	dec r1
	mov dpl,@r1
	ret

;Z SP@      -- a-addr      get data stack pointer
	.drw link
	.link
	.fw	SP@
SPFETCH:	dec r0	; push old TOS
	mov @r0,dph
	dec r0
	mov @r0,dpl
	mov dph,#0	; 16-bit pointer 00:R0
	mov dpl,r0
	ret

;Z SP!      a-addr --      set data stack pointer
; Note: only the low 8 bits are significant!
	.drw link
	.link
	.fw	SP!
SPSTORE: mov r0,dpl	; set stack pointer
	ajmp poptos	; get new TOS

;Z RP@      -- a-addr    get return stack pointer
	.drw link
	.link
	.fw	RP@
RPFETCH:	dec r0	; push old TOS
	mov @r0,dph
	dec r0
	mov @r0,dpl
	mov dph,#0	; 16-bit pointer 00:SP
	mov dpl,sp
	ret

;Z RP!      a-addr --    set return stack pointer
; Note: only the low 8 bits are significant!
	.drw link
	.link
	.fw	RP!
RPSTORE:	pop dr3	; save ret addr in r3:r2
	pop dr2
	mov sp,dpl	; set new stack pointer
	push dr2	; restore ret addr
	push dr3
	ajmp poptos	; get new TOS

;X NIP    x1 x2 -- x2           per stack diagram
	.drw link
	.link
	.fw	NIP
NIP:	acall SWOP
	ajmp DROP

;X TUCK   x1 x2 -- x2 x1 x2     per stack diagram
	.drw link
	.link
	.fw	TUCK
TUCK:	acall SWOP
	ajmp OVER

; MEMORY OPERATIONS =============================

;C !        x a-addr --    store cell in Data mem
; Byte order is lo,hi.
	.drw link
	.link
	.fw	!
STORE:	mov r1,dpl	; put data address in R1
	mov a,@r0	; low byte of X
	inc r0
	mov @r1,a
	inc r1
	mov a,@r0	; high byte of X
	inc r0
	mov @r1,a
	ajmp poptos	; pop new TOS

;C C!       c c-addr --    store char in Data mem
	.drw link
	.link
	.fw	C!
CSTORE:	mov r1,dpl	; put data address in R1
	mov a,@r0	; low byte is char
	inc r0
	mov @r1,a
	inc r0	; skip high byte
	ajmp poptos	; pop new TOS

;C @        a-addr -- x  fetch cell from Data mem
; Byte order is lo,hi.
	.drw link
	.link
	.fw	@
FETCH:	mov r1,dpl	; put data address in R1
	mov dpl,@r1	; low byte
	inc r1
	mov dph,@r1	; high byte
	ret

;C C@       c-addr -- c  fetch char from Data mem
	.drw link
	.link
	.fw	C@
CFETCH:	mov r1,dpl	; put data address in R1
	mov dpl,@r1
	mov dph,#0
	ret

;C I!       x a-addr --    store cell in Code mem
; On 8051, the only way to store to Code memory
; is to have it also appear in Data space.
; So, I! and IC! write to external RAM with MOVX.
	.drw link
	.link
	.fw	I!
ISTORE:	mov a,@r0	; low byte of X
	inc r0
	movx @dptr,a
	inc dptr
	mov a,@r0	; high byte of X
	inc r0
	movx @dptr,a
	ajmp poptos	; pop new TOS

;C IC!      c c-addr --    store char in Code mem
	.drw link
	.link
	.fw	IC!
ICSTORE: mov a,@r0	; low byte is char
	inc r0
	movx @dptr,a
	inc r0	; skip high byte
	ajmp poptos	; pop new TOS

;Z I@       a-addr -- x  fetch cell from Code mem
; Byte order is lo,hi.
	.drw link
	.link
	.fw	I@
IFETCH:	clr a
	movc a,@a+dptr	; low byte
	mov r2,a	; ..temporary stash
	mov a,#1
	movc a,@a+dptr	; high byte
	mov dpl,r2	; copy to TOS (DPTR)
	mov dph,a
	ret

;Z IC@      a-addr -- x  fetch char from Code mem
	.drw link
	.link
	.fw	IC@
ICFETCH:	clr a
	movc a,@a+dptr	; low byte
	mov dpl,a
	mov dph,#0
	ret

; ARITHMETIC AND LOGICAL OPERATIONS =============

;C +        n1/u1 n2/u2 -- n3/u3        add n1+n2
	.drw link
	.link
	.fw	+
PLUS:	mov a,@r0	; low byte
	inc r0
	add a,dpl
	mov dpl,a
	mov a,@r0	; high byte
	inc r0
	addc a,dph
	mov dph,a
	ret

;Z M+       d n -- d         add single to double
	.drw link
	.link
	.fw	M+
MPLUS:	mov dr2,@r0	; pop d.high -> r3:r2
	inc r0
	mov dr3,@r0
	inc r0
	mov a,@r0	; d.low, low byte
	add a,dpl
	mov @r0,a
	inc r0
	mov a,@r0	; d.low, high byte
	addc a,dph
	mov @r0,a
	dec r0
	clr a
	addc a,r2	; d.high, low byte
	mov dpl,a
	clr a
	addc a,r3	; d.high, high byte
	mov dph,a
	ret

;C -        n1/u1 n2/u2 -- n3/u3   subtract n1-n2
	.drw link
	.link
	.fw	-
MINUS:	mov a,@r0	; low byte
	inc r0
	clr c
	subb a,dpl
	mov dpl,a
	mov a,@r0	; high byte
	inc r0
	subb a,dph	; NOTE: LESS expects valid flags,
	mov dph,a	;  A=DPH, when MINUS returns
	ret

;C AND      x1 x2 -- x3               logical AND
	.drw link
	.link
	.fw	AND
ANDD:	mov a,@r0	; low byte
	inc r0
	anl dpl,a
	mov a,@r0	; high byte
	inc r0
	anl dph,a
	ret

;C OR       x1 x2 -- x3                logical OR
	.drw link
	.link
	.fw	OR
ORR:	mov a,@r0	; low byte
	inc r0
	orl dpl,a
	mov a,@r0	; high byte
	inc r0
	orl dph,a
	ret

;C XOR      x1 x2 -- x3               logical XOR
	.drw link
	.link
	.fw	XOR
XORR:	mov a,@r0	; low byte
	inc r0
	xrl dpl,a
	mov a,@r0	; high byte
	inc r0
	xrl dph,a
	ret

;C INVERT   x1 -- x2            bitwise inversion
	.drw link
	.link
	.fw	INVERT
INVERT:	xrl dpl,#0xff
	xrl dph,#0xff
	ret

;C NEGATE   x1 -- x2             two's complement
	.drw link
	.link
	.fw	NEGATE
NEGATE:	xrl dpl,#0xff
	xrl dph,#0xff
	inc dptr
	ret

;C 1+       n1/u1 -- n2/u2           add 1 to TOS
	.drw link
	.link
	.fw	1+
ONEPLUS:	inc dptr
	ret

;C 1-       n1/u1 -- n2/u2    subtract 1 from TOS
	.drw link
	.link
	.fw	1-
ONEMINUS:	mov a,dpl
	jnz dphok
	dec dph		;	if dpl=0, decr. affects dph
dphok:	dec dpl
	ret

;Z ><       x1 -- x2        swap bytes (not ANSI)
	.drw link
	.link
	.fw	><
SWAPBYTES:	mov a,dpl
	mov dpl,dph
	mov dph,a
	ret

;C 2*       x1 -- x2        arithmetic left shift
	.drw link
	.link
	.fw	2*
TWOSTAR: mov a,dpl	; lo byte, left shift
	add a,dpl
	mov dpl,a
	mov a,dph	; hi byte, left rot w/cy
	rlc a
	mov dph,a
	ret

;C 2/       x1 -- x2       arithmetic right shift
	.drw link
	.link
	.fw	2/
TWOSLASH: mov a,dph	; get msb of TOS into cy
	rlc a
	mov a,dph	; high byte, right rotate
	rrc a
	mov dph,a
	mov a,dpl	; low byte, right rotate
	rrc a
	mov dpl,a
	ret
	
;C LSHIFT   x1 u -- x2         logical left shift
	.drw link
	.link
	.fw	LSHIFT
LSHIFT:	mov r4,dpl	; r4 = loop counter
	mov dpl,@r0	; pop x1 -> DPTR
	inc r0
	mov dph,@r0
	inc r0
	inc r4	; test for r4=0 case
	sjmp lshtest
lshloop: mov a,dpl	; shift left
	add a,dpl
	mov dpl,a
	mov a,dph
	rlc a
	mov dph,a
lshtest: djnz r4,lshloop
	ret

;C RSHIFT   x1 u -- x2        logical right shift
	.drw link
	.link
	.fw	RSHIFT
RSHIFT:	mov r4,dpl	; r4 = loop counter
	mov dpl,@r0	; pop x1 -> DPTR
	inc r0
	mov dph,@r0
	inc r0
	inc r4	; test for r4=0 case
	sjmp rshtest
rshloop:	clr c	; clear carry
	mov a,dph	; shift right
	rrc a
	mov dph,a
	mov a,dpl
	rrc a
	mov dpl,a
rshtest:	djnz r4,rshloop
	ret

;C +!       n/u a-addr --    add cell to Data mem
	.drw link
	.link
	.fw	+!
PLUSSTORE:	mov r1,dpl	; put data address in R1
	mov a,@r0	; low byte of n
	inc r0
	add a,@r1	; low byte of memory
	mov @r1,a
	inc r1
	mov a,@r0	; high byte of n
	inc r0
	addc a,@r1	; high byte of memory
	mov @r1,a
	ajmp poptos	; pop new TOS

;Z C+!       n/u a-addr --    add byte to Data mem
	.drw link
	.link
	.fw	C+!
CPLUSSTORE:	mov r1,dpl	; put data address in R1
	mov a,@r0	; low byte of n
	inc r0
	add a,@r1	; low byte of memory
	mov @r1,a
	inc r0	; ignore high byte of n
	ajmp poptos	; pop new TOS

; COMPARISON OPERATIONS =========================

;X <>     x1 x2 -- flag            test not equal
	.drw link
	.link
	.fw	<>
NOTEQUAL:	acall EQUAL
	sjmp ZEROEQUAL

;C 0=       n/u -- flag      return true if TOS=0
	.drw link
	.link
	.fw	0=
ZEROEQUAL:	mov a,dph
zequ1:  orl a,dpl	; A = z or nz, per DPTR
	clr c
	subb a,#1	; cy set if A was 0
cyprop:	subb a,acc	; -1 if A was 0, else 0
	mov dph,a
	mov dpl,a
	ret	; NB! A=0 iff TOS=0

;C 0<       n -- flag        true if TOS negative
	.drw link
	.link
	.fw	0<
ZEROLESS:	mov a,dph
	rlc a	; cy set if A negative
	sjmp cyprop	; propagate cy thru TOS

;C =        x1 x2 -- flag              test x1=x2
	.drw link
	.link
	.db	NONIMMED,1	; hand code this one due to =
	.ascii	"="
EQUAL:	acall MINUS	; x1-x2 in TOS, A=DPH
	sjmp zequ1

;C <        n1 n2 -- flag      test n1<n2, signed
	.drw link
	.link
	.fw	<
LESS:	acall MINUS	; n1-n2 in TOS, A=DPH,
		; CY and OV valid
 ; if result negative (MSB=1) & not OV, n1<n2
 ; neg. & OV => n1 +ve, n2 -ve, result -ve, n1>n2
 ; if result positive (MSB=0) & not OV, n1>=n2
 ; pos. & OV => n1 -ve, n2 +ve, result +ve, n1<n2
 ; thus OV reverses the sense of the sign bit
	jnb psw.2,msbok ; jump if overflow clear
	cpl a	; OV set: invert msb
msbok:	rlc a	; put msb (sign) in cy
	sjmp cyprop	; & propagate thru TOS

;C >        n1 n2 -- flag      test n1>n2, signed
	.drw link
	.link
	.fw	>
GREATER:	acall SWOP
	sjmp LESS


;C U<       u1 u2 -- flag    test n1<n2, unsigned
	.drw link
	.link
	.fw	U<
ULESS:	acall MINUS	; TOS=u1-u2, cy set if u1<u2
	sjmp cyprop	; propagate cy thru TOS

;X U>    u1 u2 -- flag       test u1>u2, unsigned
	.drw link
	.link
	.fw	U>
UGREATER:	acall SWOP
	sjmp ULESS

; LOOP AND BRANCH OPERATIONS ====================

; branch and ?branch are done with sjmp and jz,
; respectively, using the following routines
; which leave a value in A.  Typical use:
;   lcall zerosense, jz destadr
;   lcall loopsense, jz destadr, lcall unloop
; LEAVE may exit loop by branching ^--here

	.drw link
	.link
	.fw	?BRANCH
qbranch:
zerosense:	; n --     leave zero in A if TOS=0
	mov dr2,@r0	; new TOS in a:r2
	inc r0
	mov a,@r0
	inc r0
	xch a,dph	; DPH=new TOS hi, A=old DPH
	orl a,dpl	; A=0 if old TOS was zero
	mov dpl,r2	; new TOS lo in DPL
	ret

; LOOP and +LOOP are done with jz, using the
; following routines which leave a value in A.
; If the loop terminates, (index crosses 8000h),
; a nonzero value is left in A.  A=0 to loop.
; Typical use:
;   lcall loopsense, jz destadr, lcall unloop
; LEAVE may exit loop by branching ^-here
; The topmost loop index is in regs r7:r6.

	.drw link
	.link
	.fw	(LOOP)
xloop:
loopsense:	; --        leave 0 in A if 'loop'
	mov a,r6	; add 1 to loop index
	add a,#1	;  ...leaves OV flag set if
	mov r6,a	;  loop terminates
	mov a,r7
	addc a,#0
	mov r7,a
	jb psw.2,termloop ; jump if OV set
takeloop:	clr a	; OV clear, make A zero
	ret	;   to take loop branch

	.drw link
	.link
	.fw	(+LOOP)
xplusloop:
pluslpsense:	; n --    leave 0 in A if '+loop'
	mov a,r6	; add TOS to loop index
	add a,dpl	;  ...leaves OV flag set if
	mov r6,a	;  loop terminates
	mov a,r7
	addc a,dph
	mov r7,a
	mov dpl,@r0	; pop new TOS, OV unaffected
	inc r0
	mov dph,@r0
	inc r0
	jnb psw.2,takeloop ; jump if OV clear
termloop:	clr a		; OV set, make A nonzero
	cpl a		;  to force loop termination
	ret

;Z (do)    n1|u1 n2|u2 --  R: -- sys1 sys2
;Z                           run-time code for DO
; '83 and ANSI standard loops terminate when the
; boundary of limit-1 and limit is crossed, in
; either direction.  This can be conveniently
; implemented by making the limit 8000h, so that
; arithmetic overflow logic can detect crossing.
; I learned this trick from Laxen & Perry F83.
; fudge factor = 8000h-limit, to be added to
; the start value.
	.drw link
	.link
	.fw	(DO)
XDO:	; limit index --
	pop dr3		; get return adrs in r3:r2
	pop dr2
	push dr6	; push previous index
	push dr7
	mov a,@r0	; get (-limit) + 8000h
	inc r0		;   = (~limit) + 8001h
	cpl a		;   in r5:r4
	add a,#01
	mov r4,a
	mov a,@r0
	inc r0
	cpl a
	addc a,#0x80
	mov r5,a
	push dr4	; push this fudge factor
	push dr5
	mov a,r4	; fudge+index -> r7:r6
	add a,dpl
	mov r6,a
	mov a,r5
	addc a,dph
	mov r7,a
	push dr2	; restore return addr
	push dr3
	ajmp poptos ; go pop new TOS

;C I        -- n   R: sys1 sys2 -- sys1 sys2
;C                   get the innermost loop index
	.drw link
	.link
	.fw	I
II:	dec r0		; push old TOS
	mov @r0,dph
	dec r0
	mov @r0,dpl
	mov r1,sp	; get copy of SP
	dec r1		; skip return address
	dec r1
	dec r1		; skip hi byte of fudge
	clr c
	mov a,r6	; index-fudge = true index
	subb a,@r1
	mov dpl,a
	inc r1
	mov a,r7
	subb a,@r1
	mov dph,a	; leaves true index on TOS
	ret

;C J        -- n   R: 4*sys -- 4*sys
;C                      get the second loop index
	.drw link
	.link
	.fw	J
JJ:	dec r0	; push old TOS
	mov @r0,dph
	dec r0
	mov @r0,dpl
	mov r1,sp	; get copy of SP
	dec r1		; skip return address
	dec r1
	dec r1		; skip inner fudge factor
	dec r1
	mov dr3,@r1	; outer index hi
	dec r1
	mov dr2,@r1	; outer index lo
	dec r1
	mov b,@r1	; outer fudge hi
	dec r1
	clr c
	mov a,r2	; index-fudge = true index
	subb a,@r1
	mov dpl,a
	mov a,r3
	subb a,b
	mov dph,a	; leaves true index on TOS
	ret

;C UNLOOP   --   R: sys1 sys2 --  drop loop parms
	.drw link
	.link
	.fw	UNLOOP
UNLOOP:	pop dr3	; get return adrs in r3:r2
	pop dr2
	dec sp		; discard fudge factor
	dec sp
	pop dr7		; restore previous loop index
	pop dr6
	push dr2	; restore return addr
	push dr3
	ret

; MULTIPLY AND DIVIDE ===========================

;C UM*     u1 u2 -- ud   unsigned 16x16->32 mult.
	.drw link
	.link
	.fw	UM*
UMSTAR:	mov dr1,@r0	; u1 Lo in r1
	inc r0	; u1 Hi in mem!

	mov a,r1	; u1L*u2L -> B:A -> r3:r2
	mov b,dpl
	mul ab
	mov r2,a
	mov r3,b

	mov a,r1	; u1L*u2H -> B:A
	mov b,dph	;       add into r4:r3
	mul ab
	add a,r3
	mov r3,a
	clr a
	addc a,b
	mov r4,a

	mov a,@r0	; u1H*u2L -> B:A
	mov b,dpl	;       add into r4:r3
	mul ab
	add a,r3
	mov r3,a
	mov a,r4
	addc a,b
	mov r4,a
	clr a	;       w/possible cy->r5
	addc a,#0
	mov r5,a

	mov a,@r0	; u1H*u2H -> B:A
	mov b,dph	;       add into r5:r4
	mul ab
	add a,r4
	mov r4,a
	mov a,r5
	addc a,b
	mov dph,a	; result in dph:r4:r3:r2
	mov dpl,r4
	mov @r0,dr3
	dec r0
	mov @r0,dr2
	ret

;C UM/MOD   ud u1 -- u2 u3     unsigned 32/16->16
	.drw link
	.link
	.fw	UM/MOD
UMSLASHMOD:	; DPH:DPL = divisor
	mov dr3,@r0	; r2:r3:r4:r5 = dividend
		; note stack order:
	inc r0	;  ^      xxxx
	mov dr2,@r0	;  |      xxxx
		; high   hi byte \ low
	inc r0	; adrs   lo byte /  cell
	mov dr5,@r0	;        hi byte \ high
		;  R0--> lo byte /  cell
	inc r0	;  on    -------
	mov dr4,@r0	;  entry
		;
	mov r1,#17	; loop counter
	clr c
	sjmp div2
div1:	; division loop
	mov a,r3
	rlc a
	mov r3,a
	mov a,r2
	rlc a
	mov r2,a
	jnc div3
	; here cy=1, cy:r2:r3 is a 17 bit value,
	; we know we can subtract divisor
	clr c
	mov a,r3
	subb a,dpl
	mov r3,a
	mov a,r2
	subb a,dph
	mov r2,a
	clr c
	sjmp div4
div3:	; here cy=0, r2:r3 is a 16 bit value
	clr c
	mov a,r3
	subb a,dpl
	mov r3,a
	mov a,r2
	subb a,dph
	mov r2,a
	jnc div4
	; borrow occurred -- undo the subtract
	mov a,r3
	add a,dpl
	mov r3,a
	mov a,r2
	addc a,dph
	mov r2,a
	setb c
div4:	; here cy=0 if subtracted, cy=1 if not
	cpl c
div2:   mov a,r5
	rlc a
	mov r5,a
	mov a,r4
	rlc a
	mov r4,a
	djnz r1,div1
	mov dpl,r5	; put quotient in TOS
	mov dph,r4
	mov @r0,dr2	; push remainder on stack
	dec r0
	mov @r0,dr3
	ret

; BLOCK AND STRING OPERATIONS ===================

;C FILL    c-addr u char --  fill Data mem w/char
; This is a quick translation of the 8051 version.  Further optimization
; is possible, given that lengths > 255 are meaningless.
	.drw link
	.link
	.fw	FILL
FILL:	mov r4,dpl	; stash char temporarily
	mov dr2,@r0	; get count in r3:r2
	inc r0
	mov dr3,@r0
	inc r0
	mov dr1,@r0	; get addr in r1
	inc r0
	; mov dph,@r0	; hi addr is ignored
	inc r0
	mov a,r4	; get char in A
	inc r3	; adjust r3,r2 for djnz loop
	inc r2
	sjmp filltest
fillloop:	mov @r1,a
	inc r1
filltest:	djnz r2,fillloop
	djnz r3,fillloop
	ajmp poptos	; pop new TOS

;Z I->D     c-addr1 c-addr2 u --  move Code->Data
; Block move from Code space to Data space.
;   ?DUP IF
;     OVER + SWAP DO
;       DUP IC@ I C! 1+
;     LOOP DUP
;   THEN 2DROP ;
	.drw link
	.link
	.fw	I->D
ITOD:	acall QDUP
	acall zerosense
	jz itod2
	acall OVER
	acall PLUS
	acall SWOP
	acall XDO
itod1:  acall DUP
	acall ICFETCH
	acall II
	acall CSTORE
	acall ONEPLUS
	acall loopsense
	jz itod1
	acall UNLOOP
	acall DUP
itod2:  acall DROP
	ajmp DROP

;Z D->I     c-addr1 c-addr2 u --  move Data->Code
; Block move from Data space to Code space.
; On the 8052 this is NOT identical to CMOVE!
;   ?DUP IF
;     OVER + SWAP DO
;       DUP C@ I IC! 1+
;     LOOP DUP
;   THEN 2DROP ;
	.drw link
	.link
	.fw	D->I
DTOI:	acall QDUP
	acall zerosense
	jz dtoi2
	acall OVER
	acall PLUS
	acall SWOP
	acall XDO
dtoi1:  acall DUP
	acall CFETCH
	acall II
	acall ICSTORE
	acall ONEPLUS
	acall loopsense
	jz dtoi1
	acall UNLOOP
	acall DUP
dtoi2:  acall DROP
	ajmp DROP

;X CMOVE   c-addr1 c-addr2 u --  move from bottom
; as defined in the ANSI optional String word set
; On byte machines, CMOVE and CMOVE> are logical
; factors of MOVE.  They are easy to implement on
; CPUs which have a block-move instruction.
;   ?DUP IF
;     OVER + SWAP DO
;       DUP C@ I C! 1+
;     LOOP DUP
;   THEN 2DROP ;
	.drw link
	.link
	.fw	CMOVE
CMOVE:	acall QDUP
	acall zerosense
	jz cmove2
	acall OVER
	acall PLUS
	acall SWOP
	acall XDO
cmove1: acall DUP
	acall CFETCH
	acall II
	acall CSTORE
	acall ONEPLUS
	acall loopsense
	jz cmove1
	acall UNLOOP
	acall DUP
cmove2: acall DROP
	ajmp DROP

;X CMOVE>  c-addr1 c-addr2 u --  move from top
; as defined in the ANSI optional String word set
;   ?DUP IF
;     1- ROT OVER +   \ addr2 u-1 addr1+u-1
;     ROT ROT OVER +  \ addr1+u-1 addr2 addr2+u-1
;     DO
;       DUP C@ I C! 1-
;     -1 +LOOP DUP
;   THEN 2DROP ;
	.drw link
	.link
	.fw	CMOVE>
CMOVEUP:	acall QDUP
	acall zerosense
	jz cmovu2
	acall ONEMINUS
	acall ROT
	acall OVER
	acall PLUS
	acall ROT
	acall ROT
	acall OVER
	acall PLUS
	acall XDO
cmovu1: acall DUP
	acall CFETCH
	acall II
	acall CSTORE
	acall ONEMINUS
	acall LIT
	.drw -1
	acall pluslpsense
	jz cmovu1
	acall UNLOOP
	acall DUP
cmovu2: acall DROP
	ajmp DROP

;Z SKIP   c-addr u c -- c-addr' u'
;Z                          skip matching chars
; Although SKIP, SCAN, and S= are perhaps not the
; ideal factors of WORD and FIND, they closely
; follow the string operations available on many
; CPUs, and so are easy to implement and fast.
; This is a quick translation of the 8051 version.  Further optimization
; is possible, given that lengths > 255 are meaningless.
	.drw link
	.link
	.fw	SKIP
SKIP:	mov r4,dpl	; stash char temporarily
	mov dr2,@r0	; get count in r3:r2
	inc r0
	mov dr3,@r0
	inc r0
	mov dr1,@r0	; get addr in dph:r1
	; inc r0
	; mov dph,@r0	; hi addr is ignored
	inc r3	; adj r3,r2 for djnz loop
	inc r2
	sjmp skiptest
skiploop:	mov a,@r1	; get char
	xrl a,r4	; compare with desired
	jnz skipmis	; exit if mismatch
	inc r1
skiptest:	djnz r2,skiploop
	djnz r3,skiploop
	; count exhausted; r3:r2=0000,
	;    adrs points past last char
skipmis:	; either mismatch, or count exhausted
	; mov @r0,dph	; push updated addr - dph is unchanged
	; dec r0
	mov @r0,dr1
	dec r2		; adjust r3,r2 back
	dec r3		;   to a normal count,
	mov dph,r3	; put in TOS,
	mov dpl,r2
	inc dptr	; adjust for extra decr
	ret

;Z SCAN    c-addr u c -- c-addr' u'
;Z                      find matching char
; This is a quick translation of the 8051 version.  Further optimization
; is possible, given that lengths > 255 are meaningless.
	.drw link
	.link
	.fw	SCAN
SCAN:	mov r4,dpl	; stash char temporarily
	mov dr2,@r0	; get count in r3:r2
	inc r0
	mov dr3,@r0
	inc r0
	mov dr1,@r0	; get addr in dph:r1
	; inc r0
	; mov dph,@r0	; hi addr is ignored
	inc r3	; adj r3,r2 for djnz loop
	inc r2
	sjmp scantest
scanloop:	mov a,@r1	; get char
	xrl a,r4	; compare with desired
	jz scanmis	; exit if match
	inc r1
scantest:	djnz r2,scanloop
	djnz r3,scanloop
	; count exhausted; r3:r2=0000
	;   adrs points past last char
scanmis:	; either match, or count exhausted
	; mov @r0,dph	; push updated addr - dph is unchanged
	; dec r0
	mov @r0,dr1
	dec r2		; adjust r3,r2 back
	dec r3		;   to a normal count,
	mov dph,r3	; put in TOS,
	mov dpl,r2
	inc dptr	; adjust for extra decr
	ret

;Z S=    c-addr1 c-addr2 u -- n   string compare
;Z             n<0: s1<s2, n=0: s1=s2, n>0: s1>s2
; Omitted in 8051 version.

;Z N=    c-addr1 c-addr2 u -- n   string:name cmp
;Z             n<0: s1<s2, n=0: s1=s2, n>0: s1>s2
;   ?DUP IF
;       OVER + SWAP DO
;           DUP C@ I IC@ -
;           ?DUP IF NIP UNLOOP EXIT THEN
;       1+ LOOP DUP
;   THEN 2DROP 0 ;
; Harvard model: c-addr1=>Data, c-addr2=>Code.
; This is a quick translation of the 8051 version.  Further optimization
; is possible, given that lengths > 255 are meaningless.
	.drw link
	.link
	.fw	N=
NEQUAL:	mov r2,dpl	; count in r3:r2
	mov r3,dph
	mov dpl,@r0	; get Code addr in dptr
	inc r0
	mov dph,@r0
	inc r0
	mov dr1,@r0	; get Data addr in --:r1
	inc r0
	; mov a,@r0	; hi addr is ignored
	inc r0
	inc r3	; adjust for djnz loop
	inc r2
	sjmp Nequtest
Nequloop:	clr a	; get Code char
	movc a,@a+dptr
	inc dptr
	clr c	; Code-Data  (was Data-Code in 8051 version)
	subb a,@r1
	inc r1
	jnz Nequfail
Nequtest:	djnz r2,Nequloop
	djnz r3,Nequloop
	mov dph,r3	; strings match, r3=0,
	mov dpl,r3	;  so make TOS=0
	sjmp Nequdone
Nequfail:	cpl c	; reverse sense of carry (Data-Code)
	subb a,acc	; -1 if cy set, 0 if clr
	mov dph,a	; (Data<Code) (Data>Code)
	orl a,#1	; TOS = FFFF or 0001
	mov dpl,a
Nequdone:	ret

;         acall QDUP
;         acall zerosense
;         jz sequ3
;         acall OVER
;         acall PLUS
;         acall SWOP
;         acall XDO
; sequ1:  acall DUP
;         acall CFETCH
;         acall II
;         acall ICFETCH
;         acall MINUS
;         acall QDUP
;         acall zerosense
;         jz sequ2
;         acall NIP
;         acall UNLOOP
;         ret
; sequ2:  acall ONEPLUS
;         acall loopsense
;         jz sequ1
;         acall UNLOOP
;         acall DUP
; sequ3:  acall DROP
;         acall DROP
;         acall LIT
;         .drw 0
;         ret

;	$EJECT
; ===============================================
; CamelForth for the Intel 8051
; (c) 1994 Bradford J. Rodriguez
; Permission is granted to freely copy, modify,
; and distribute this program for personal or
; educational use.  Commercial inquiries should
; be directed to the author at 221 King St. E.,
; #32, Hamilton, Ontario L8N 1B5 Canada
;
; CAMEL51D.AZM: CPU and Model Dependencies
;   Source code is for the A51 assembler.
;   Forth words are documented as follows:
;*   NAME     stack -- stack    description
;   Word names in upper case are from the ANS
;   Forth Core word set.  Names in lower case are
;   "internal" implementation words & extensions.
;
; Subroutine-Threaded Forth model for Intel 8051
;   cell size is   16 bits (2 bytes)
;   char size is    8 bits (1 byte)
;   address unit is 8 bits (1 byte), i.e.,
;       addresses are byte-aligned.
; ===============================================

; ALIGNMENT AND PORTABILITY OPERATORS ===========
; Many of these are synonyms for other words,
; and so are defined as CODE words.

;C ALIGN    --                         align HERE
	.drw link
	.link
	.fw	ALIGN
ALIGN:	ret	; noop!

;C ALIGNED  addr -- a-addr       align given addr
	.drw link
	.link
	.fw	ALIGNED
ALIGNED:	ret	; noop!

;Z CELL     -- n                 size of one cell
	.drw link
	.link
	.fw	CELL
CELL:	lcall docon
	.drw 2

;C CELL+    a-addr1 -- a-addr2      add cell size
;   2 + ;
	.drw link
	.link
	.fw	CELL+
CELLPLUS:	inc dptr
	inc dptr
	ret

;C CELLS    n1 -- n2            cells->adrs units
	.drw link
	.link
	.fw	CELLS
CELLS:	ljmp TWOSTAR

;C CHAR+    c-addr1 -- c-addr2   add char size
	.drw link
	.link
	.fw	CHAR+
CHARPLUS:	inc dptr
	ret

;C CHARS    n1 -- n2            chars->adrs units
	.drw link
	.link
	.fw	CHARS
CHARS:	ret

;C >BODY    xt -- a-addr      adrs of CREATE data
;   3 + I@ ;                   8051 (3 byte CALL)
	.drw link
	.link
	.fw	>BODY
TOBODY:	inc dptr
	inc dptr
	inc dptr
	ljmp IFETCH

; Note that I@ and I! use lo,hi byte order (same
; as 8086 and Z80), but the 8051 LCALL and LJMP
; addresses are stored hi,lo.  This difference
; is encapsulated within ,XT !CF and ,CF .

;X COMPILE,  xt --         append execution token
; I called this word ,XT before I discovered that
; it is defined in the ANSI standard as COMPILE,.
; On a DTC Forth this simply appends xt (like , )
; but on an STC Forth this must append 'CALL xt'.
;   012 IC, >< I, ;  12h = 8051 Lcall instruction
	.drw link
	.link
	.db	NONIMMED,8
	.ascii	"COMPILE,"	;hand code this one due to ,
COMMAXT:	lcall LIT
	.drw 0x12
	lcall ICCOMMA
	lcall SWAPBYTES
	ljmp ICOMMA

;Z !CF    adrs cfa --   set code action of a word
;   012 OVER IC!         store 'LCALL adrs' instr
;   1+ SWAP >< SWAP I! ;     8051 Harvard VERSION
; Depending on the implementation this could
; append CALL adrs or JUMP adrs.
	.drw link
	.link
	.fw	!CF
STORECF:	lcall LIT
	.drw 0x12
	lcall OVER
	lcall ICSTORE
	lcall ONEPLUS
	lcall SWOP
	lcall SWAPBYTES
	lcall SWOP
	ljmp ISTORE

;Z ,CF    adrs --             append a code field
;   012 IC, >< I, ;          8051 Harvard VERSION
	.drw link
	.link
	.db	NONIMMED,3
	.ascii	",CF"	; hand code this one due to ,
COMMACF:	sjmp COMMAXT

;Z ,NOCF    --      append a dummy code field  (8052)
; This appends the a dummy code field to the Forth word header.
; This will be later resolved by !CF.
; To speed Flash programming, this just skips the code field.
;   3 IALLOT ;
	.drw link
	.link
	.db	NONIMMED,5
	.ascii	",NOCF"		; hand code this one due to ,
COMMANOCF:	lcall LIT
	.drw  3
	ljmp IALLOT

;Z !COLON   --       change code field to docolon
;   -3 IALLOT ;           8052 Harvard version; undo ,NOCF
; This should be used immediately after <BUILDS.
; This is made a distinct word, because on an STC
; Forth, colon definitions have no code field.
	.drw link
	.link
	.fw	!COLON
STORCOLON:	lcall LIT
	.drw -3
	ljmp IALLOT

; ***
; This is approximately the end of the first 2K
; block.  CALLs and JMPs crossing this boundary
; must use the Long form.
; ***

;Z ,EXIT    --      append hi-level EXIT action
;   022 IC, ;               8051 VERSION
; This is made a distinct word, because on an STC
; Forth, it appends a RET instruction, not an xt.
	.drw link
	.link
	.db	NONIMMED,5
	.ascii	",EXIT"		; hand code this one due to ,
CEXIT:	lcall LIT
	.drw 0x22
	ljmp ICCOMMA

; CONTROL STRUCTURES ============================
; These words allow Forth control structure words
; to be defined portably.

;Z ,BRANCH   xt --    append a branch instruction
; xt is the branch operator to use, e.g. qbranch
; or (loop).  It does NOT append the destination
; address.  On the 8051 this compiles
;         LCALL xt   jz-opcode
; unless xt=0 in which case the LCALL is omitted
; and an 'sjmp' instruction is compiled.
;   ?DUP IF ,XT 060 ELSE 080 THEN IC, ;
	.drw link
	.link
	.db	NONIMMED,7
	.ascii	",BRANCH"		; hand code this one due to ,
COMMABRANCH:	lcall QDUP
	lcall zerosense
	jz combr1
	lcall COMMAXT	; LCALL sense-routine
	lcall LIT
	.drw 0x60	; jz opcode
	ljmp ICCOMMA
combr1: lcall LIT
	.drw 0x80	; sjmp opcode
	ljmp ICCOMMA

; high level code may use 'branch' as an argument
; to ,BRANCH:
	.equ	branch,0

;Z ,DEST   dest --        append a branch address
; This appends the given destination address to
; the branch instruction.  On the 8051 this is a
; one-byte relative address.
;   IHERE 1+ - IC, ;
	.drw link
	.link
	.db	NONIMMED,5
	.ascii	",DEST"		; hand code this one due to ,
COMMADEST:	lcall IHERE
	lcall ONEPLUS
	lcall MINUS
	ljmp ICCOMMA

;Z ,NODEST   --        append a dummy branch address (8052)
; This appends the a dummy destination address to the branch
; instruction.  This will be later resolved by !DEST.
; To speed Flash programming, this just skips the address byte.
;   1 IALLOT ;
	.drw link
	.link
	.db	NONIMMED,7
	.ascii	",NODEST"	; hand code this one due to ,
COMMANODEST:	lcall LIT
	.drw  1
	ljmp IALLOT

;Z !DEST   dest adrs --    change a branch dest'n
; Changes the destination address found at 'adrs'
; to the given 'dest'.  On the 8051 this is a
; one-byte relative address.
;   TUCK 1+ - SWAP IC! ;
	.drw link
	.link
	.fw	!DEST
STOREDEST:	lcall TUCK
	lcall ONEPLUS
	lcall MINUS
	lcall SWOP
	ljmp ICSTORE

;Z ,UNLOOP  --       append an UNloop instruction
; Used after a LOOP or +LOOP is compiled.
; Required on the 8051 because the loop branch
; must be followed by UNLOOP.  No-op on Z80.
;   ['] UNLOOP ,XT ;
	.drw link
	.link
	.db	NONIMMED,7
	.ascii	",UNLOOP"	; hand code this one due to ,
COMMAUNLOOP:	lcall LIT
	.drw UNLOOP
	ljmp COMMAXT

; HEADER STRUCTURE ==============================
; The structure of the Forth dictionary headers
; (name, link, immediate flag, and "smudge" bit)
; does not necessarily differ across CPUs.  This
; structure is not easily factored into distinct
; "portable" words; instead, it is implicit in
; the definitions of FIND and CREATE, and also in
; NFA>LFA, NFA>CFA, IMMED?, IMMEDIATE, HIDE, and
; REVEAL.  These words must be (substantially)
; rewritten if either the header structure or its
; inherent assumptions are changed.

;	$EJECT
; ===============================================
; CamelForth for the Intel 8051
; (c) 1994 Bradford J. Rodriguez
; Permission is granted to freely copy, modify,
; and distribute this program for personal or
; educational use.  Commercial inquiries should
; be directed to the author at 221 King St. E.,
; #32, Hamilton, Ontario L8N 1B5 Canada
;
; CAMEL51H.AZM: High Level Words
;   Source code is for the A51 assembler.
;   Forth words are documented as follows:
;x   NAME     stack -- stack    description
; ===============================================

; SYSTEM VARIABLES & CONSTANTS ==================

;C BL      -- char                 an ASCII space
	.drw link
	.link
	.fw	BL
BL:	lcall docon
	.drw 0x20

;Z TIBSIZE  -- n                      size of TIB
	.drw link
	.link
	.fw	TIBSIZE
TIBSIZE:	lcall docon
	.drw 39	; 1 char safety zone - 8052

;X TIB     -- a-addr        Terminal Input Buffer
;  termbufr CONSTANT TIB    fixed address in internal RAM
	.drw link
	.link
	.fw	TIB
TIB:	lcall docon
	.drw termbufr

;Z u0      -- a-addr       current user area adrs
;  0 USER U0
	.drw link
	.link
	.fw	U0
U0:     lcall douser
	.drw 0

;C >IN     -- a-addr        holds offset into TIB
;  2 USER >IN
	.drw link
	.link
	.fw	>IN
TOIN:	lcall douser
	.drw 0

;C BASE    -- a-addr       holds conversion radix
;  4 USER BASE
	.drw link
	.link
	.fw	BASE
BASE:	lcall douser
	.drw 1

;C STATE   -- a-addr         holds compiler state
;  6 USER STATE
	.drw link
	.link
	.fw	STATE
STATE:	lcall douser
	.drw 2

;Z DP      -- a-addr         holds dictionary ptr
;  8 USER DP
	.drw link
	.link
	.fw	DP
DP:	lcall douser
	.drw 3

;Z 'SOURCE  -- a-addr        two cells: len, adrs
; 10 USER 'SOURCE
	.drw link
	.link
	.db	NONIMMED,7
	.ascii	"'SOURCE"
TICKSOURCE:	lcall douser
	.drw 4

;Z LATEST    -- a-addr         last word in dict.
;   14 USER LATEST
	.drw link
	.link
	.fw	LATEST
LATEST:	lcall douser
	.drw 6

;Z HP       -- a-addr                HOLD pointer
;   16 USER HP
	.drw link
	.link
	.fw	HP
HP:	lcall douser
	.drw 8

;Z LP       -- a-addr         Leave-stack pointer
;   18 USER LP
	.drw link
	.link
	.fw	LP
LP:	lcall douser
	.drw 9

;Z IDP    -- a-addr        ROM dictionary pointer
;   20 USER IDP
	.drw link
	.link
	.fw	IDP
IDP:	lcall douser
	.drw 10

;Z S0       -- a-addr      end of parameter stack
;                          fixed address in internal RAM
	.drw link
	.link
	.fw	S0
S0:     lcall docon
	.drw pstack

;X PAD       -- a-addr            user PAD buffer
;                          fixed address in internal RAM
	.drw link
	.link
	.fw	PAD
PAD:	lcall docon
	.drw padbufr

;Z L0       -- a-addr       bottom of Leave stack
;                          fixed address in internal RAM
	.drw link
	.link
	.fw	L0
L0:     lcall docon
	.drw lstack

;Z R0       -- a-addr         end of return stack
; on the 8051 this is the init value of RSP (SP).
	.drw link
	.link
	.fw	R0
RP0:    lcall docon	; note R0 is an 8051 reg!
	.drw rstack

;Z uinit    -- addr  initial values for user area
	.drw link
	.link
	.fw	UINIT
UINIT:	lcall docon
	.drw raminit	; table located at end of file

;Z #init    -- n    #bytes of user area init data
	.drw link
	.link
	.fw	#INIT
NINIT:	lcall docon
	.drw raminitsize	; table located at end of file


; ARITHMETIC OPERATORS ==========================

;C S>D    n -- d           single -> double prec.
;   DUP 0< ;
	.drw link
	.link
	.fw	S>D
STOD:	lcall DUP
	ljmp ZEROLESS

;Z ?NEGATE  n1 n2 -- n3  negate n1 if n2 negative
;   0< IF NEGATE THEN ;        ...a common factor
	.drw link
	.link
	.fw	?NEGATE
QNEGATE:	lcall ZEROLESS
	lcall zerosense
	jz qneg1
	lcall NEGATE
qneg1:  ret

;C ABS     n1 -- +n2     absolute value
;   DUP ?NEGATE ;
	.drw link
	.link
	.fw	ABS
ABS:	lcall DUP
	sjmp QNEGATE

;X DNEGATE   d1 -- d2     negate double precision
;   SWAP INVERT SWAP INVERT 1 M+ ;
	.drw link
	.link
	.fw	DNEGATE
DNEGATE:	lcall SWOP
	lcall INVERT
	lcall SWOP
	lcall INVERT
	lcall LIT
	.drw 1
	ljmp MPLUS

;Z ?DNEGATE  d1 n -- d2   negate d1 if n negative
;   0< IF DNEGATE THEN ;       ...a common factor
	.drw link
	.link
	.fw	?DNEGATE
QDNEGATE:	lcall ZEROLESS
	lcall zerosense
	jz dneg1
	acall DNEGATE
dneg1:  ret

;X DABS     d1 -- +d2    absolute value .dbl.prec.
;   DUP ?DNEGATE ;
	.drw link
	.link
	.fw	DABS
DABS:	lcall DUP
	sjmp QDNEGATE

;C M*     n1 n2 -- d    signed 16*16->32 multiply
;   2DUP XOR >R        carries sign of the result
;   SWAP ABS SWAP ABS UM*
;   R> ?DNEGATE ;
	.drw link
	.link
	.fw	M*
MSTAR:	acall TWODUP
	lcall XORR
	lcall TOR
	lcall SWOP
	acall ABS
	lcall SWOP
	acall ABS
	lcall UMSTAR
	lcall RFROM
	sjmp QDNEGATE

;C SM/REM   d1 n1 -- n2 n3   symmetric signed div
;   2DUP XOR >R              sign of quotient
;   OVER >R                  sign of remainder
;   ABS >R DABS R> UM/MOD
;   SWAP R> ?NEGATE
;   SWAP R> ?NEGATE ;
; Ref. dpANS-6 section 3.2.2.1.
	.drw link
	.link
	.fw	SM/REM
SMSLASHREM:	acall TWODUP
	lcall XORR
	lcall TOR
	lcall OVER
	lcall TOR
	acall ABS
	lcall TOR
	acall DABS
	lcall RFROM
	lcall UMSLASHMOD
	lcall SWOP
	lcall RFROM
	acall QNEGATE
	lcall SWOP
	lcall RFROM
	ajmp  QNEGATE

;C FM/MOD   d1 n1 -- n2 n3   floored signed div'n
;   DUP >R              divisor
;   2DUP XOR >R         sign of quotient
;   >R                  divisor
;   DABS R@ ABS UM/MOD
;   SWAP R> ?NEGATE SWAP  apply sign to remainder
;   R> 0< IF              if quotient negative,
;       NEGATE
;       OVER IF             if remainder nonzero,
;         R@ ROT -  SWAP 1-     adjust rem,quot
;       THEN
;   THEN  R> DROP ;
; Ref. dpANS-6 section 3.2.2.1.
	.drw link
	.link
	.fw	FM/MOD
FMSLASHMOD:	lcall DUP
	lcall TOR
	acall TWODUP
	lcall XORR
	lcall TOR
	lcall TOR
	acall DABS
	lcall RFETCH
	acall ABS
	lcall UMSLASHMOD
	lcall SWOP
	lcall RFROM
	acall QNEGATE
	lcall SWOP
	lcall RFROM
	lcall ZEROLESS
	lcall zerosense
	jz fmmod1
	lcall NEGATE
	lcall OVER
	lcall zerosense
	jz fmmod1
	lcall RFETCH
	lcall ROT
	lcall MINUS
	lcall SWOP
	lcall ONEMINUS
fmmod1: lcall RFROM
	lcall DROP
	ret

;C *      n1 n2 -- n3             signed multiply
;   M* DROP ;
	.drw link
	.link
	.fw	*
STAR:	acall MSTAR
	ljmp DROP

;C /MOD   n1 n2 -- n3 n4     signed divide/rem'dr
;   >R S>D R> FM/MOD ;
	.drw link
	.link
	.fw	/MOD
SLASHMOD:	lcall TOR
	acall STOD
	lcall RFROM
	ajmp FMSLASHMOD

;C /      n1 n2 -- n3               signed divide
;   /MOD nip ;
	.drw link
	.link
	.fw	/
SLASH:	acall SLASHMOD
	ljmp NIP

;C MOD    n1 n2 -- n3            signed remainder
;   /MOD DROP ;
	.drw link
	.link
	.fw	MOD
MODD:	acall SLASHMOD
	ljmp DROP

;C */MOD  n1 n2 n3 -- n4 n5    n1*n2/n3, rem&quot
;   >R M* R> FM/MOD ;
	.drw link
	.link
	.fw	*/MOD
SSMOD:	lcall TOR
	acall MSTAR
	lcall RFROM
	ajmp FMSLASHMOD

;C */     n1 n2 n3 -- n4                 n1*n2/n3
;   */MOD nip ;
	.drw link
	.link
	.fw	*/
STARSLASH:	acall SSMOD
	ljmp NIP

;C MAX    n1 n2 -- n3              signed maximum
;   2DUP < IF SWAP THEN DROP ;
	.drw link
	.link
	.fw	MAX
MAX:	acall TWODUP
	lcall LESS
	lcall zerosense
	jz max1
	lcall SWOP
max1:   ljmp DROP

;C MIN    n1 n2 -- n3              signed minimum
;   2DUP > IF SWAP THEN DROP ;
	.drw link
	.link
	.fw	MIN
MIN:	acall TWODUP
	lcall GREATER
	lcall zerosense
	jz min1
	lcall SWOP
min1:   ljmp DROP

; DOUBLE OPERATORS ==============================

;C 2@    a-addr -- x1 x2            fetch 2 cells
;   DUP CELL+ @ SWAP @ ;
;   the lower address will appear on top of stack
	.drw link
	.link
	.fw	2@
TWOFETCH:	lcall DUP
	lcall CELLPLUS
	lcall FETCH
	lcall SWOP
	ljmp FETCH

;C 2!    x1 x2 a-addr --            store 2 cells
;   SWAP OVER ! CELL+ ! ;
;   the top of stack is stored at the lower adrs
	.drw link
	.link
	.fw	2!
TWOSTORE:	lcall SWOP
	lcall OVER
	lcall STORE
	lcall CELLPLUS
	ljmp STORE

;Z 2C@    a-addr -- c1 c2            fetch 2 bytes
;   DUP CHAR+ C@ SWAP C@ ;
;   the lower address will appear on top of stack
	.drw link
	.link
	.fw	2C@
TWOCFETCH:	lcall DUP
	lcall CHARPLUS
	lcall CFETCH
	lcall SWOP
	ljmp CFETCH

;Z 2C!    c1 c2 a-addr --            store 2 bytes
;   SWAP OVER C! CHAR+ C! ;
;   the top of stack is stored at the lower adrs
	.drw link
	.link
	.fw	2C!
TWOCSTORE:	lcall SWOP
	lcall OVER
	lcall CSTORE
	lcall CHARPLUS
	ljmp CSTORE

;C 2DROP  x1 x2 --                   drop 2 cells
;   DROP DROP ;
	.drw link
	.link
	.fw	2DROP
TWODROP:	lcall DROP
	ljmp DROP

;C 2DUP   x1 x2 -- x1 x2 x1 x2    dup top 2 cells
;   OVER OVER ;
	.drw link
	.link
	.fw	2DUP
TWODUP:	lcall OVER
	ljmp OVER

;C 2SWAP  x1 x2 x3 x4 -- x3 x4 x1 x2  per diagram
;   ROT >R ROT R> ;
	.drw link
	.link
	.fw	2SWAP
TWOSWAP:	lcall ROT
	lcall TOR
	lcall ROT
	lcall RFROM	; can't ljmp RFROM!
	ret

;C 2OVER  x1 x2 x3 x4 -- x1 x2 x3 x4 x1 x2
;   >R >R 2DUP R> R> 2SWAP ;
	.drw link
	.link
	.fw	2OVER
TWOOVER:	lcall TOR
	lcall TOR
	acall TWODUP
	lcall RFROM
	lcall RFROM
	ajmp TWOSWAP

; INPUT/OUTPUT ==================================

;C COUNT   c-addr1 -- c-addr2 u  counted->adr/len
;   DUP CHAR+ SWAP C@ ;
	.drw link
	.link
	.fw	COUNT
COUNT:	lcall DUP
	lcall CHARPLUS
	lcall SWOP
	ljmp CFETCH

;C CR      --                      output newline
;   0D EMIT 0A EMIT ;
	.drw link
	.link
	.fw	CR
CRR:	lcall LIT
	.drw 0x0d
	lcall EMIT
	lcall LIT
	.drw 0x0a
	ljmp EMIT

;C SPACE   --                      output a space
;   BL EMIT ;
	.drw link
	.link
	.fw	SPACE
SPACE:	lcall BL
	ljmp EMIT

;C SPACES   n --                  output n spaces
;   BEGIN DUP WHILE SPACE 1- REPEAT DROP ;
	.drw link
	.link
	.fw	SPACES
SPACES:
SPCS1:  lcall DUP
	lcall zerosense
	jz SPCS2
	acall SPACE
	lcall ONEMINUS
	sjmp SPCS1
SPCS2:  ljmp DROP

;Z umin     u1 u2 -- u           unsigned minimum
;   2DUP U> IF SWAP THEN DROP ;
	.drw link
	.link
	.fw	UMIN
UMIN:	acall TWODUP
	lcall UGREATER
	lcall zerosense
	jz UMIN1
	lcall SWOP
UMIN1:  ljmp DROP

;Z umax    u1 u2 -- u            unsigned maximum
;   2DUP U< IF SWAP THEN DROP ;
	.drw link
	.link
	.fw	UMAX
UMAX:	acall TWODUP
	lcall ULESS
	lcall zerosense
	jz UMAX1
	lcall SWOP
UMAX1:  ljmp DROP

;C ACCEPT  c-addr +n -- +n'  get line from term'l
;   OVER + 1- OVER      -- sa ea a
;   BEGIN KEY           -- sa ea a c
;   DUP 0D <> WHILE
;       DUP EMIT        -- sa ea a c
;       DUP 8 = IF  DROP 1-    >R OVER R> UMAX
;             ELSE  OVER C! 1+ OVER UMIN
;       THEN            -- sa ea a
;   REPEAT              -- sa ea a c
;   DROP NIP SWAP - ;
	.drw link
	.link
	.fw	ACCEPT
ACCEPT:	lcall OVER
	lcall PLUS
	lcall ONEMINUS
	lcall OVER
ACC1:   lcall KEY
	lcall DUP
	lcall LIT
	.drw 0x0D
	lcall NOTEQUAL
	lcall zerosense
	jz ACC5
	lcall DUP
	lcall EMIT
	lcall DUP
	lcall LIT
	.drw 8
	lcall EQUAL
	lcall zerosense
	jz ACC3
	lcall DROP
	lcall ONEMINUS
	lcall TOR
	lcall OVER
	lcall RFROM
	acall UMAX
	sjmp ACC4
ACC3:   lcall OVER
	lcall CSTORE
	lcall ONEPLUS
	lcall OVER
	acall UMIN
ACC4:   sjmp ACC1
ACC5:   lcall DROP
	lcall NIP
	lcall SWOP
	ljmp MINUS

;C TYPE    c-addr +n --       type line to term'l
;   ?DUP IF
;     OVER + SWAP DO I C@ EMIT LOOP
;   ELSE DROP THEN ;
	.drw link
	.link
	.fw	TYPE
TYPE:	lcall QDUP
	lcall zerosense
	jz TYP4
	lcall OVER
	lcall PLUS
	lcall SWOP
	lcall XDO
TYP3:   lcall II
	lcall CFETCH
	lcall EMIT
	lcall loopsense
	jz TYP3
	lcall UNLOOP
	sjmp TYP5
TYP4:   lcall DROP
TYP5:   ret

;Z (IS")    -- c-addr u      run-time code for IS"
;   R> ICOUNT 2DUP + ALIGNED >R	;
; Harvard model, for string stored in Code space
; e.g. as used by ."
	.drw link
	.link
	.fwq	(IS")
XISQUOTE:	lcall RFROM
	acall ICOUNT
	acall TWODUP
	lcall PLUS
	; lcall ALIGNED
	lcall TOR	; do NOT ljmp TOR!
	ret

;Z (S")    -- c-addr u       run-time code for S"
;   R@ I@                     get Data address
;   R> CELL+ DUP IC@ CHAR+    -- Dadr Radr+2 n+1
;   2DUP + ALIGNED >R         -- Dadr Iadr n+1
;   >R OVER R> I>D            -- Dadr
;   COUNT ;
; Harvard model, for string stored in Code space
; which is copied to Data space.
	.drw link
	.link
	.fwq	(S")
XSQUOTE:	lcall RFETCH
	lcall IFETCH
	lcall RFROM
	lcall CELLPLUS
	lcall DUP
	lcall ICFETCH
	lcall CHARPLUS
	acall TWODUP
	lcall PLUS
	; lcall ALIGNED
	lcall TOR
	lcall TOR
	lcall OVER
	lcall RFROM
	lcall ITOD
	ajmp COUNT

;C IS"      --            compile in-line string
;   COMPILE (IS")  [ HEX ]
;   22 IWORD
;   IC@ 1+ ALIGNED IALLOT ; IMMEDIATE
; Harvard model: string is stored in Code space
	.drw link
	.link
	.fwiq	IS"
ISQUOTE:	lcall LIT
	.drw XISQUOTE
	lcall COMMAXT
	lcall LIT
	.drw 0x22
	lcall IWORD
	lcall ICFETCH
	lcall ONEPLUS
	; lcall ALIGNED
	ljmp IALLOT

;C S"       --             compile in-line string
;   COMPILE (S")  [ HEX ]
;   HERE I,                     data address
;   22 IWORD
;   IC@ 1+ ALIGNED
;   DUP ALLOT IALLOT ; IMMEDIATE
; Harvard model: string is stored in Code space
	.drw link
	.link
	.fwiq	S"
SQUOTE:	lcall LIT
	.drw XSQUOTE
	lcall COMMAXT
	acall HERE
	lcall ICOMMA
	lcall LIT
	.drw 0x22
	lcall IWORD
	lcall ICFETCH
	lcall ONEPLUS
	; lcall ALIGNED
	lcall DUP
	acall ALLOT
	ljmp IALLOT

;C ."       --            compile string to print
;   POSTPONE IS"  POSTPONE ITYPE ; IMMEDIATE
	.drw link
	.link
	.fwiq	."
DOTQUOTE:	acall ISQUOTE
	lcall LIT
	.drw ITYPE
	ljmp COMMAXT

;Z ICOUNT  c-addr1 -- c-addr2 u  counted->adr/len
;   DUP CHAR+ SWAP IC@ ;          from Code space
	.drw link
	.link
	.fw	ICOUNT
ICOUNT:	lcall DUP
	lcall CHARPLUS
	lcall SWOP
	ljmp ICFETCH

;Z ITYPE   c-addr +n --       type line to term'l
;   ?DUP IF                       from Code space
;     OVER + SWAP DO I IC@ EMIT LOOP
;   ELSE DROP THEN ;
	.drw link
	.link
	.fw	ITYPE
ITYPE:	lcall QDUP
	lcall zerosense
	jz ITYP4
	lcall OVER
	lcall PLUS
	lcall SWOP
	lcall XDO
ITYP3:  lcall II
	lcall ICFETCH
	lcall EMIT
	lcall loopsense
	jz ITYP3
	lcall UNLOOP
	sjmp ITYP5
ITYP4:  lcall DROP
ITYP5:  ret


;Z IWORD     c -- c-addr       WORD to Code space
;   WORD
;   IHERE TUCK OVER C@ CHAR+ D->I ;
	.drw link
	.link
	.fw	IWORD
IWORD:	lcall WORD
	lcall IHERE
	lcall TUCK
	lcall OVER
	lcall CFETCH
	lcall CHARPLUS
	ljmp DTOI

; NUMERIC OUTPUT ================================
; Numeric conversion is done l.s.digit first, so
; the output buffer is built backwards in memory.

; Some double-precision arithmetic operators are
; needed to implement ANSI numeric conversion.

;Z UD/MOD   ud1 u2 -- u3 ud4     32/16->32 divide
;   >R 0 R@ UM/MOD  ROT ROT R> UM/MOD ROT ;
	.drw link
	.link
	.fw	UD/MOD
UDSLASHMOD:	lcall TOR
	lcall LIT
	.drw 0
	lcall RFETCH
	lcall UMSLASHMOD
	lcall ROT
	lcall ROT
	lcall RFROM
	lcall UMSLASHMOD
	ljmp ROT

;Z UD*      ud1 d2 -- ud3      32*16->32 multiply
;   DUP >R UM* DROP  SWAP R> UM* ROT + ;
	.drw link
	.link
	.fw	UD*
UDSTAR:	lcall DUP
	lcall TOR
	lcall UMSTAR
	lcall DROP
	lcall SWOP
	lcall RFROM
	lcall UMSTAR
	lcall ROT
	ljmp PLUS

;C HOLD  char --        add char to output string
;   -1 HP +!  HP @ C! ;
	.drw link
	.link
	.fw	HOLD
HOLD:	lcall LIT
	.drw -1
	acall HP
	lcall CPLUSSTORE
	acall HP
	lcall CFETCH
	ljmp CSTORE

;C <#    --              begin numeric conversion
;   PAD HP ! ;          (initialize Hold Pointer)
	.drw link
	.link
	.fw	<#
LESSNUM:	acall PAD
	acall HP
	ljmp CSTORE

;Z >digit   n -- c            convert to 0..9A..Z
;   [ HEX ] DUP 9 > 7 AND + 30 + ;
	.drw link
	.link
	.fw	>DIGIT
TODIGIT:	lcall DUP
	lcall LIT
	.drw 9
	lcall GREATER
	lcall LIT
	.drw 7
	lcall ANDD
	lcall PLUS
	lcall LIT
	.drw 0x30
	ljmp PLUS

;C #     ud1 -- ud2     convert 1 digit of output
;   BASE @ UD/MOD ROT >digit HOLD ;
	.drw link
	.link
	.fw	#
NUM:	acall BASE
	lcall CFETCH
	acall UDSLASHMOD
	lcall ROT
	acall TODIGIT
	sjmp HOLD

;C #S    ud1 -- ud2      convert remaining digits
;   BEGIN # 2DUP OR 0= UNTIL ;
	.drw link
	.link
	.fw	#S
NUMS:
NUMS1:
	acall NUM
	acall TWODUP
	lcall ORR
	lcall ZEROEQUAL
	lcall zerosense
	jz NUMS1
	ret

;C #>    ud1 -- c-addr u    end conv., get string
;   2DROP HP @ PAD OVER - ;
	.drw link
	.link
	.fw	#>
NUMGREATER:	acall TWODROP
	acall HP
	lcall CFETCH
	acall PAD
	lcall OVER
	ljmp MINUS

;C SIGN  n --               add minus sign if n<0
;   0< IF 2D HOLD THEN ;
	.drw link
	.link
	.fw	SIGN
SIGN:	lcall ZEROLESS
	lcall zerosense
	jz SIGN1
	lcall LIT
	.drw 0x2D
	acall HOLD
SIGN1:  ret

;C U.    u --                  display u unsigned
;   <# 0 #S #> TYPE SPACE ;
	.drw link
	.link
	.fw	U.
UDOT:	acall LESSNUM
	lcall LIT
	.drw 0
	acall NUMS
	acall NUMGREATER
	acall TYPE
	ajmp SPACE

;C .     n --                    display n signed
;   <# DUP ABS 0 #S ROT SIGN #> TYPE SPACE ;
	.drw link
	.link
	.fw	.
DOT:	acall LESSNUM
	lcall DUP
	lcall ABS
	lcall LIT
	.drw 0
	acall NUMS
	lcall ROT
	acall SIGN
	acall NUMGREATER
	acall TYPE
	ajmp SPACE

;C DECIMAL  --         set number base to decimal
;   10 BASE ! ;
	.drw link
	.link
	.fw	DECIMAL
DECIMAL:	lcall LIT
	.drw 10
	acall BASE
	ljmp CSTORE

;X HEX     --              set number base to hex
;   16 BASE ! ;
	.drw link
	.link
	.fw	HEX
HEX:	lcall LIT
	.drw 16
	acall BASE
	ljmp CSTORE

; DICTIONARY MANAGEMENT =========================

;C HERE    -- addr         returns dictionary ptr
;   DP @ ;
	.drw link
	.link
	.fw	HERE
HERE:	acall DP
	ljmp CFETCH

;C ALLOT   n --          allocate n bytes in dict
;   DP +! ;
	.drw link
	.link
	.fw	ALLOT
ALLOT:	acall DP
	ljmp CPLUSSTORE

;C ,    x --                  append cell to dict
;   HERE ! 1 CELLS ALLOT ;
	.drw link
	.link
	.db	NONIMMED,1
	.ascii	","		; hand code this one due to ,
COMMA:	lcall HERE
	lcall STORE
	lcall LIT
	.drw 2
	sjmp ALLOT

;C C,   char --               append char to dict
;   HERE C! 1 CHARS ALLOT ;
	.drw link
	.link
	.db	NONIMMED,2
	.ascii	"C,"		; hand code this one due to ,
CCOMMA:	lcall HERE
	lcall CSTORE
	lcall LIT
	.drw 1
	sjmp ALLOT

; The following additional words support the
; "Harvard" model, with separate address spaces
; for Instructions (Code) and Data.  ANSI
; requires DP to manage the Data space, so a
; separate Instruction Dictionary Pointer, IDP,
; is added to manage the Code space.  Also added:
;   I@ IC@ I! IC!        (in the primitives)
;   IHERE IALLOT I, IC,
;   ITYPE ICOUNT WORD>I
; It should be possible to convert the Harvard
; implementation to a combined-code-and-data
; system, by equating these words to their
; Data-space counterparts.

;Z IHERE    -- addr    return Code dictionary ptr
;   IDP @ ;
	.drw link
	.link
	.fw	IHERE
IHERE:	lcall IDP
	ljmp FETCH

;Z IALLOT   n --    allocate n bytes in Code dict
;   IDP +! ;
	.drw link
	.link
	.fw	IALLOT
IALLOT:	lcall IDP
	ljmp PLUSSTORE

; ***
; This is approximately the end of the second 2K
; block.  CALLs and JMPs crossing this boundary
; must use the Long form.
; ***

;Z I,   x --             append cell to Code dict
;   IHERE I! 1 CELLS IALLOT ;
	.drw link
	.link
	.db	NONIMMED,2
	.ascii	"I,"		; hand code this one due to ,
ICOMMA:	lcall IHERE
	lcall ISTORE
	lcall LIT
	.drw 2
	sjmp IALLOT

;Z IC,  x --             append char to Code dict
;   IHERE IC! 1 CHARS IALLOT ;
	.drw link
	.link
	.db	NONIMMED,3
	.ascii	"IC,"		; hand code this one due to ,
ICCOMMA:	lcall IHERE
	lcall ICSTORE
	lcall LIT
	.drw 1
	sjmp IALLOT

; INTERPRETER ===================================
; Note that NFA>LFA, NFA>CFA, IMMED?, and FIND
; are dependent on the structure of the Forth
; header.  This may be common across many CPUs,
; or it may be different.

;C SOURCE   -- adr n         current input buffer
;   'SOURCE 2@ ;        length is at lower adrs
	.drw link
	.link
	.fw	SOURCE
SOURCE:	lcall TICKSOURCE
	ljmp TWOCFETCH

;X /STRING  a u n -- a+n u-n          trim string
;   ROT OVER + ROT ROT - ;
	.drw link
	.link
	.fw	/STRING
SLASHSTRING:	lcall ROT
	lcall OVER
	lcall PLUS
	lcall ROT
	lcall ROT
	ljmp MINUS

;Z >counted  src n dst --     copy to counted str
;   2DUP C! CHAR+ SWAP CMOVE ;
	.drw link
	.link
	.fw	>COUNTED
TOCOUNTED:	lcall TWODUP
	lcall CSTORE
	lcall CHARPLUS
	lcall SWOP
	ljmp CMOVE

;C WORD   char -- c-addr     word delim'd by char
;   DUP  SOURCE >IN @ /STRING   -- c c adr n
;   DUP >R   ROT SKIP           -- c adr' n'
;   OVER >R  ROT SCAN           -- adr" n"
;   DUP IF CHAR- THEN        skip trailing delim.
;   R> R> ROT -   >IN +!        update >IN offset
;   TUCK -                      -- adr' N
;   HERE >counted               --
;   HERE                        -- a
;   BL OVER COUNT + C! ;    append trailing blank
	.drw link
	.link
	.fw	WORD
WORD:	lcall DUP
	lcall SOURCE
	lcall TOIN
	lcall CFETCH
	lcall SLASHSTRING
	lcall DUP
	lcall TOR
	lcall ROT
	lcall SKIP
	lcall OVER
	lcall TOR
	lcall ROT
	lcall SCAN
	lcall DUP
	lcall zerosense
	jz WORD1
	lcall ONEMINUS	; char-
WORD1:  lcall RFROM
	lcall RFROM
	lcall ROT
	lcall MINUS
	lcall TOIN
	lcall CPLUSSTORE
	lcall TUCK
	lcall MINUS
	lcall HERE
	lcall TOCOUNTED
	lcall HERE
	lcall BL
	lcall OVER
	lcall COUNT
	lcall PLUS
	ljmp CSTORE

;Z NFA>LFA   nfa -- lfa    name adr -> link field
;   3 - ;
	.drw link
	.link
	.fw	NFA>LFA
NFATOLFA:	lcall LIT
	.drw 3
	ljmp MINUS

;Z NFA>CFA   nfa -- cfa    name adr -> code field
;   ICOUNT 7F AND + ;       mask off 'smudge' bit
; Harvard model.
	.drw link
	.link
	.fw	NFA>CFA
NFATOCFA:	lcall ICOUNT
	lcall LIT
	.drw 0x7F
	lcall ANDD
	ljmp PLUS

;Z IMMED?    nfa -- f        fetch immediate flag
;   1- IC@  1 AND 0=             nonzero if immed  (8052 mod)
; Harvard model.
	.drw link
	.link
	.fw	IMMED?
IMMEDQ:	lcall ONEMINUS
	lcall ICFETCH
	lcall LIT
	.drw  1
	lcall ANDD
	ljmp  ZEROEQUAL

;C FIND   c-addr -- c-addr 0   if not found
;C                  xt  1      if immediate
;C                  xt -1      if "normal"
;   LATEST @ BEGIN             -- a nfa
;       2DUP OVER C@ CHAR+     -- a nfa a nfa n+1
;       N=                     -- a nfa f
;       DUP IF
;           DROP
;           NFA>LFA I@ DUP     -- a link link
;       THEN
;   0= UNTIL                   -- a nfa  OR  a 0
;   DUP IF
;       NIP DUP NFA>CFA        -- nfa xt
;       SWAP IMMED?            -- xt iflag
;       0= 1 OR                -- xt 1/-1
;   THEN ;
	.drw link
	.link
	.fw	FIND
FIND:	lcall LATEST
	lcall FETCH
FIND1:	lcall TWODUP
	lcall OVER
	lcall CFETCH
	lcall CHARPLUS
	lcall NEQUAL
	lcall DUP
	lcall zerosense
	jz FIND2
	lcall DROP
	lcall NFATOLFA
	lcall IFETCH
	lcall DUP
FIND2:  lcall ZEROEQUAL
	lcall zerosense
	jz FIND1
	lcall DUP
	lcall zerosense
	jz FIND3
	lcall NIP
	lcall DUP
	lcall NFATOCFA
	lcall SWOP
	acall IMMEDQ
	lcall ZEROEQUAL
	lcall LIT
	.drw 1
	lcall ORR
FIND3:  ret

;C LITERAL  x --           append numeric literal
;   STATE @ IF ['] LIT ,XT I, THEN ; IMMEDIATE
; This tests STATE so that it can also be used
; interpretively.  (ANSI doesn't require this.)
	.drw link
	.link
	.fwi	LITERAL
LITERAL:	lcall STATE
	lcall CFETCH
	lcall zerosense
	jz LITER1
	lcall LIT
	.drw LIT
	lcall COMMAXT
	lcall ICOMMA
LITER1: ret

;Z DIGIT?   c -- n -1   if c is a valid digit
;Z            -- x  0   otherwise
;   [ HEX ] DUP 39 > 100 AND +     silly looking
;   DUP 140 > 107 AND -   30 -     but it works!
;   DUP BASE @ U< ;
	.drw link
	.link
	.fw	DIGIT?
DIGITQ:	lcall DUP
	lcall LIT
	.drw 0x39
	lcall GREATER
	lcall LIT
	.drw 0x100
	lcall ANDD
	lcall PLUS
	lcall DUP
	lcall LIT
	.drw 0x140
	lcall GREATER
	lcall LIT
	.drw 0x107
	lcall ANDD
	lcall MINUS
	lcall LIT
	.drw 0x30
	lcall MINUS
	lcall DUP
	lcall BASE
	lcall CFETCH
	ljmp ULESS

;Z ?SIGN   adr n -- adr' n' f   get optional sign
;Z  advance adr/n if sign; return NZ if negative
;   OVER C@                 -- adr n c
;   2C - DUP ABS 1 = AND    -- +=-1, -=+1, else 0
;   DUP IF 1+               -- +=0, -=+2
;       >R 1 /STRING R>     -- adr' n' f
;   THEN ;
	.drw link
	.link
	.fw	?SIGN
QSIGN:	lcall OVER
	lcall CFETCH
	lcall LIT
	.drw 0x2C
	lcall MINUS
	lcall DUP
	lcall ABS
	lcall LIT
	.drw 0x1
	lcall EQUAL
	lcall ANDD
	lcall DUP
	lcall zerosense
	jz QSIGN1
	lcall ONEPLUS
	lcall TOR
	lcall LIT
	.drw 0x1
	lcall SLASHSTRING
	lcall RFROM
QSIGN1: ret

;C >NUMBER  ud adr u -- ud' adr' u'
;C                       convert string to number
;   BEGIN
;   DUP WHILE
;       OVER C@ DIGIT?
;       0= IF DROP EXIT THEN
;       >R 2SWAP BASE @ UD*
;       R> M+ 2SWAP
;       1 /STRING
;   REPEAT ;
	.drw link
	.link
	.fw	>NUMBER
TONUMBER:
TONUM1: lcall DUP
	lcall zerosense
	jz TONUM3
	lcall OVER
	lcall CFETCH
	lcall DIGITQ
	lcall ZEROEQUAL
	lcall zerosense
	jz TONUM2
	ljmp DROP
TONUM2: lcall TOR
	lcall TWOSWAP
	lcall BASE
	lcall CFETCH
	lcall UDSTAR
	lcall RFROM
	lcall MPLUS
	lcall TWOSWAP
	lcall LIT
	.drw 0x1
	lcall SLASHSTRING
	sjmp TONUM1
TONUM3: ret

;Z ?NUMBER  c-addr -- n -1      string->number
;Z                 -- c-addr 0  if convert error
;   DUP  0 0 ROT COUNT      -- ca ud adr n
;   ?SIGN >R  >NUMBER       -- ca ud adr' n'
;   IF   R> 2DROP 2DROP 0   -- ca 0   (error)
;   ELSE 2DROP NIP R>
;       IF NEGATE THEN  -1  -- n -1   (ok)
;   THEN ;
	.drw link
	.link
	.fw	?NUMBER
QNUMBER:
	lcall DUP
	lcall LIT
	.drw 0x0
	lcall DUP
	lcall ROT
	lcall COUNT
	lcall QSIGN
	lcall TOR
	lcall TONUMBER
	lcall zerosense
	jz QNUM1
	lcall RFROM
	lcall TWODROP
	lcall TWODROP
	lcall LIT
	.drw 0x0
	sjmp QNUM3
QNUM1:  lcall TWODROP
	lcall NIP
	lcall RFROM
	lcall zerosense
	jz QNUM2
	lcall NEGATE
QNUM2:  lcall LIT
	.drw -1
QNUM3:  ret

;Z INTERPRET    i*x c-addr u -- j*x
;Z                         interpret given buffer
; This is a common factor of EVALUATE and QUIT.
; ref. dpANS-6, 3.4 The Forth Text Interpreter
;   'SOURCE 2!  0 >IN !
;   BEGIN
;   BL WORD DUP C@ WHILE        -- textadr
;       FIND                    -- a 0/1/-1
;       ?DUP IF                 -- xt 1/-1
;           1+ STATE @ 0= OR    immed or interp?
;           IF EXECUTE ELSE ,XT THEN
;       ELSE                    -- textadr
;           ?NUMBER
;           IF POSTPONE LITERAL     converted ok
;           ELSE COUNT TYPE 3F EMIT CR ABORT  err
;           THEN
;       THEN
;   REPEAT DROP ;
	.drw link
	.link
	.fw	INTERPRET
INTERPRET:	lcall TICKSOURCE
	lcall TWOCSTORE
	lcall LIT
	.drw 0x0
	lcall TOIN
	lcall CSTORE
INTER1: lcall BL
	lcall WORD
	lcall DUP
	lcall CFETCH
	lcall zerosense
	jz INTER9
	lcall FIND
	lcall QDUP
	lcall zerosense
	jz INTER4
	lcall ONEPLUS
	lcall STATE
	lcall CFETCH
	lcall ZEROEQUAL
	lcall ORR
	lcall zerosense
	jz INTER2
	lcall EXECUTE
	sjmp INTER3
INTER2: lcall COMMAXT
INTER3: sjmp INTER8
INTER4: acall QNUMBER
	lcall zerosense
	jz INTER5
	lcall LITERAL
	sjmp INTER6
INTER5: lcall COUNT
	lcall TYPE
	lcall LIT
	.drw 0x3F
	lcall EMIT
	lcall CRR
	lcall ABORT
INTER6:
INTER8: sjmp INTER1
INTER9: ljmp DROP

;C EVALUATE  i*x c-addr u -- j*x  interprt string
;   'SOURCE 2@ >R >R  >IN @ >R
;   INTERPRET
;   R> >IN !  R> R> 'SOURCE 2! ;
	.drw link
	.link
	.fw	EVALUATE
EVALUATE:	lcall TICKSOURCE
	lcall TWOCFETCH
	lcall TOR
	lcall TOR
	lcall TOIN
	lcall CFETCH
	lcall TOR
	acall INTERPRET
	lcall RFROM
	lcall TOIN
	lcall CSTORE
	lcall RFROM
	lcall RFROM
	lcall TICKSOURCE
	ljmp TWOCSTORE

;C QUIT     --    R: i*x --    interpret from kbd
;   L0 LP !  R0 RP!   0 STATE !
;   BEGIN
;       TIB DUP TIBSIZE ACCEPT  SPACE
;       INTERPRET
;       STATE @ 0= IF ." OK" THEN CR
;   AGAIN ;
	.drw link
	.link
	.fw	QUIT
QUIT:	lcall L0
	lcall LP
	lcall CSTORE
	lcall RP0
	lcall RPSTORE
	lcall LIT
	.drw 0x0
	lcall STATE
	lcall CSTORE
QUIT1:  lcall TIB
	lcall DUP
	lcall TIBSIZE
	lcall ACCEPT
	lcall SPACE
	acall INTERPRET
	lcall STATE
	lcall CFETCH
	lcall ZEROEQUAL
	lcall zerosense
	jz QUIT2
	lcall XISQUOTE
	.db 2
	.ascii	"ok"
	lcall ITYPE
QUIT2:  lcall CRR
	sjmp QUIT1

;C ABORT    i*x --   R: j*x --   clear stk & QUIT
;   S0 SP!  QUIT ;
	.drw link
	.link
	.fw	ABORT
ABORT:	lcall S0
	lcall SPSTORE
	sjmp QUIT	; QUIT never returns

;Z ?ABORT   f c-addr u --       abort & print msg
;   ROT IF ITYPE ABORT THEN 2DROP ;
	.drw link
	.link
	.fw	?ABORT
QABORT:	lcall ROT
	lcall zerosense
	jz QABO1
	lcall ITYPE
	sjmp ABORT	; ABORT never returns
QABO1:  ljmp TWODROP

;C ABORT"  i*x 0  -- i*x   R: j*x -- j*x  x1=0
;C         i*x x1 --       R: j*x --      x1<>0
;   POSTPONE IS" POSTPONE ?ABORT ; IMMEDIATE
	.drw link
	.link
	.fwiq	ABORT"
ABORTQUOTE:	lcall ISQUOTE
	lcall LIT
	.drw QABORT
	ljmp COMMAXT

;C '    -- xt             find word in dictionary
;   BL WORD FIND
;   0= ABORT" ?" ;
	.drw link
	.link
	.db  NONIMMED,1,0x27	; 27h = '
TICK:	lcall BL
	lcall WORD
	lcall FIND
	lcall ZEROEQUAL
	lcall XISQUOTE
	.db 1
	.ascii "?"
	sjmp QABORT

;C CHAR   -- char           parse ASCII character
;   BL WORD 1+ C@ ;
	.drw link
	.link
	.fw	CHAR
CHAR:	lcall BL
	lcall WORD
	lcall ONEPLUS
	ljmp CFETCH

;C [CHAR]   --          compile character literal
;   CHAR  ['] LIT ,XT  I, ; IMMEDIATE
	.drw link
	.link
	.fwi	[CHAR]
BRACCHAR:	acall CHAR
	lcall LIT
	.drw LIT
	lcall COMMAXT
	ljmp ICOMMA

;C (    --                     skip input until )
;   [ HEX ] 29 WORD DROP ; IMMEDIATE
	.drw link
	.link
	.fwi	(
PAREN:	lcall LIT
	.drw 0x29
	lcall WORD
	ljmp DROP

; COMPILER ======================================

;C CREATE   --         create an empty definition
;   LATEST @ I, NONIMMED IC,          link & immed field
;   IHERE LATEST !             new "latest" link
;   BL IWORD IC@ 1+ IALLOT            name field
;   docreate ,CF                      code field
;   HERE I, ;             <-- Harvard model only!
; Harvard model, separate Code and Data spaces.
	.drw link
	.link
	.fw	CREATE
CREATE:	lcall LATEST
	lcall FETCH
	lcall ICOMMA
	lcall LIT
	.drw  NONIMMED
	lcall ICCOMMA
	lcall IHERE
	lcall LATEST
	lcall STORE
	lcall BL
	lcall IWORD
	lcall ICFETCH
	lcall ONEPLUS
	lcall IALLOT
	lcall LIT
	.drw docreate
	lcall COMMACF
	lcall HERE
	ljmp ICOMMA

;Z <BUILDS   --      create an empty definition with no defined action (8052)
;   LATEST @ I, NONIMMED IC,          link & immed field
;   IHERE LATEST !             new "latest" link
;   BL IWORD IC@ 1+ IALLOT            name field
;   ,NOCF ; code field
; A pointer to HERE is *not* appended to the Code space!
; The defined word may append whatever it wishes.
; Harvard model, separate Code and Data spaces.
	.drw link
	.link
	.fw	<BUILDS
BUILDS:	lcall LATEST
	lcall FETCH
	lcall ICOMMA
	lcall LIT
	.drw  NONIMMED
	lcall ICCOMMA
	lcall IHERE
	lcall LATEST
	lcall STORE
	lcall BL
	lcall IWORD
	lcall ICFETCH
	lcall ONEPLUS
	lcall IALLOT
	ljmp COMMANOCF

;Z (DOES>)  --      run-time action of DOES>
;   R>              adrs of headless DOES> def'n
;   LATEST @ NFA>CFA    code field to fix up
;   !CF ;
	.drw link
	.link
	.fw	(DOES>)
XDOES:	lcall RFROM
	lcall LATEST
	lcall FETCH
	lcall NFATOCFA
	ljmp STORECF

;C DOES>    --      change action of latest def'n
;   ['] (DOES>) ,XT
;   dodoes ,CF ; IMMEDIATE
	.drw link
	.link
	.fwi	DOES>
DOES:	lcall LIT
	.drw XDOES
	lcall COMMAXT
	lcall LIT
	.drw dodoes
	ljmp COMMACF

;C RECURSE  --      recurse current definition
;   LATEST @ NFA>CFA ,XT ; IMMEDIATE
	.drw link
	.link
	.fwi	RECURSE
RECURSE:	lcall LATEST
	lcall FETCH
	lcall NFATOCFA
	ljmp COMMAXT

;C [        --      enter interpretive state
;   0 STATE ! ; IMMEDIATE
	.drw link
	.link
	.fwi	[
LEFTBRACKET:	lcall LIT
	.drw 0x0
	lcall STATE
	ljmp CSTORE

;C ]        --      enter compiling state
;   -1 STATE ! ;
	.drw link
	.link
	.fw	]
RIGHTBRACKET:	lcall LIT
	.drw -1
	lcall STATE
	ljmp CSTORE

;Z HIDE     -- nfa        "hide" latest definition
;   LATEST @                       get adr of latest def.
;   DUP NFA>LFA I@  LATEST ! ;     link to previous def.
; Flash-ROM Harvard model. (Cygnal)
	.drw link
	.link
	.fw	HIDE
HIDE:	lcall LATEST
	lcall FETCH
	lcall DUP
	lcall NFATOLFA
	lcall IFETCH
	lcall LATEST
	ljmp STORE

;Z REVEAL   nfa --      "reveal" latest definition
;   DUP NFA>LFA I@  LATEST @ =     validity check
;   IF LATEST ! ELSE DROP THEN ;
; Flash-ROM Harvard model. (Cygnal)
	.drw link
	.link
	.fw	REVEAL
REVEAL:	lcall DUP
	lcall NFATOLFA
	lcall IFETCH
	lcall LATEST
	lcall FETCH
	lcall EQUAL
	lcall zerosense
	jz REVEAL1
	lcall LATEST
	lcall STORE
	sjmp REVEAL2
REVEAL1: lcall DROP
REVEAL2: ret

;C IMMEDIATE   --   make last def'n immediate
;   IMMED LATEST @ 1- IC! ;   set immediate flag
; Harvard model.
	.drw link
	.link
	.fw	IMMEDIATE
IMMEDIATE:	lcall LIT
	.drw  IMMED
	lcall LATEST
	lcall FETCH
	lcall ONEMINUS
	ljmp ICSTORE

;C :        -- colon-sys           begin a colon definition
;   <BUILDS HIDE ] !COLON ;
	.drw link
	.link
	.db	NONIMMED,1
	.ascii	":"		; hand code this one due to :
COLON:	acall BUILDS
	acall HIDE
	acall RIGHTBRACKET
	ljmp STORCOLON

;C ;        colon-sys --             end a colon definition
;   REVEAL  ,EXIT
;   POSTPONE [	; IMMEDIATE
	.drw link
	.link
	.db	IMMED,1
	.ascii	";"		; hand code this one due to ;
SEMICOLON:	acall REVEAL
	lcall CEXIT
	ajmp LEFTBRACKET

;C [']  --         find word & compile as literal
;   '  ['] LIT ,XT  I, ; IMMEDIATE
; When encountered in a colon definition, the
; phrase  ['] xxx  will cause   LIT,xxt  to be
; compiled into the colon definition (where
; xxt is the execution token of word xxx).
; When the colon definition executes, xxt will
; be put on the stack.  (All xt's are one cell.)
	.drw link
	.link
	.fwi	[']
BRACTICK:	acall TICK	; get xt of 'xxx'
	lcall LIT
	.drw LIT
	lcall COMMAXT	; append LIT action
	ljmp ICOMMA	; append xt literal

;C POSTPONE  --   postpone compile action of word
;   BL WORD FIND
;   DUP 0= ABORT" ?"
;   0< IF   -- xt  non immed: add code to current
;                  def'n to compile xt later.
;       ['] LIT ,XT  I,     add "LIT,xt,COMMAXT"
;       ['] ,XT ,XT         to current definition
;   ELSE  ,XT      immed: compile into cur. def'n
;   THEN ; IMMEDIATE
	.drw link
	.link
	.fwi	POSTPONE
POSTPONE:	lcall BL
	lcall WORD
	lcall FIND
	lcall DUP
	lcall ZEROEQUAL
	lcall XISQUOTE
	.db 1,'?'
	lcall QABORT
	lcall ZEROLESS
	lcall zerosense
	jz POST1
	lcall LIT
	.drw LIT
	lcall COMMAXT
	lcall ICOMMA
	lcall LIT
	.drw COMMAXT
	lcall COMMAXT
	sjmp POST2
POST1:  lcall COMMAXT
POST2:  ret

;Z COMPILE   --   append inline execution token
;   R> DUP CELL+ >R @ ,XT ;
; The phrase ['] xxx ,XT appears so often that
; this word was created to combine the actions
; of LIT and ,XT.  It takes an inline literal
; execution token and appends it to the dict.
;    head COMPILE,7,COMPILE,docolon
;        .drw RFROM,DUP,CELLPLUS,TOR
;        .drw FETCH,COMMAXT,EXIT
; N.B.: not used in the current implementation

; CONTROL STRUCTURES ============================

;C IF       -- adrs    conditional forward branch
;   ['] qbranch ,BRANCH  IHERE ,NODEST ;
;   IMMEDIATE
	.drw link
	.link
	.fwi	IF
IFF:	lcall LIT
	.drw qbranch
	lcall COMMABRANCH
	lcall IHERE
	ljmp COMMANODEST

;C THEN     adrs --        resolve forward branch
;   IHERE SWAP !DEST ; IMMEDIATE
	.drw link
	.link
	.fwi	THEN
THEN:	lcall IHERE
	lcall SWOP
	ljmp STOREDEST

;C ELSE     adrs1 -- adrs2    branch for IF..ELSE
;   ['] branch ,BRANCH  IHERE ,NODEST
;   SWAP  POSTPONE THEN ; IMMEDIATE
	.drw link
	.link
	.fwi	ELSE
ELS:	lcall LIT
	.drw branch
	lcall COMMABRANCH
	lcall IHERE
	lcall COMMANODEST
	lcall SWOP
	sjmp THEN

;C BEGIN    -- adrs        target for bwd. branch
;   IHERE ; IMMEDIATE
	.drw link
	.link
	.fwi	BEGIN
BEGIN:	ljmp IHERE

;C UNTIL    adrs --   conditional backward branch
;   ['] qbranch ,BRANCH  ,DEST ; IMMEDIATE
;   conditional backward branch
	.drw link
	.link
	.fwi	UNTIL
UNTIL:	lcall LIT
	.drw qbranch
	lcall COMMABRANCH
	ljmp COMMADEST

;X AGAIN    adrs --      uncond'l backward branch
;   ['] branch ,BRANCH  ,DEST ; IMMEDIATE
;   unconditional backward branch
	.drw link
	.link
	.fwi	AGAIN
AGAIN:	lcall LIT
	.drw branch
	lcall COMMABRANCH
	ljmp COMMADEST

;C WHILE    -- adrs         branch for WHILE loop
;   POSTPONE IF SWAP ; IMMEDIATE
	.drw link
	.link
	.fwi	WHILE
WHILE:	acall IFF
	ljmp SWOP

;C REPEAT   adrs1 adrs2 --     resolve WHILE loop
;   POSTPONE AGAIN POSTPONE THEN ; IMMEDIATE
	.drw link
	.link
	.fwi	REPEAT
REPEAT:	acall AGAIN
	sjmp THEN

;Z >L   x --   L: -- x        move to leave stack
;   CELL LP +!  LP @ ! ;      (L stack grows up)
	.drw link
	.link
	.fw	>L
TOL:	lcall CELL
	lcall LP
	lcall CPLUSSTORE
	lcall LP
	lcall CFETCH
	ljmp STORE

;Z L>   -- x   L: x --      move from leave stack
;   LP @ @  CELL NEGATE LP +! ;
	.drw link
	.link
	.fw	L>
LFROM:	lcall LP
	lcall CFETCH
	lcall FETCH
	lcall CELL
	lcall NEGATE
	lcall LP
	ljmp CPLUSSTORE

;C DO       -- adrs   L: -- 0
;   ['] xdo ,XT  IHERE     target for bwd branch
;   0 >L ; IMMEDIATE           marker for LEAVEs
	.drw link
	.link
	.fwi	DO
DO:	lcall LIT
	.drw XDO
	lcall COMMAXT
	lcall IHERE
	lcall LIT
	.drw 0x0
	ajmp TOL

;Z ENDLOOP   adrs xt --   L: 0 a1 a2 .. aN --
;   ,BRANCH  ,DEST  ,UNLOOP       backward loop
;   BEGIN L> ?DUP WHILE POSTPONE THEN REPEAT ;
;                                 resolve LEAVEs
; This is a common factor of LOOP and +LOOP.
	.drw link
	.link
	.fw	ENDLOOP
ENDLOOP:	lcall COMMABRANCH
	lcall COMMADEST
	lcall COMMAUNLOOP	; new on 8051!
LOOP1:  acall LFROM
	lcall QDUP
	lcall zerosense
	jz LOOP2
	acall THEN
	sjmp LOOP1
LOOP2:  ret

;C LOOP    adrs --   L: 0 a1 a2 .. aN --
;   ['] xloop ENDLOOP ;  IMMEDIATE
	.drw link
	.link
	.fwi	LOOP
LOOP:	lcall LIT
	.drw xloop
	sjmp ENDLOOP

;C +LOOP   adrs --   L: 0 a1 a2 .. aN --
;   ['] xplusloop ENDLOOP ;  IMMEDIATE
	.drw link
	.link
	.fwi	+LOOP
PLUSLOOP:	lcall LIT
	.drw xplusloop
	sjmp ENDLOOP

;C LEAVE    --    L: -- adrs
;   ['] UNLOOP ,XT
;   ['] branch ,BRANCH   IHERE ,NODEST  >L
;	; IMMEDIATE      unconditional forward branch
	.drw link
	.link
	.fwi	LEAVE
LEAVE:	lcall LIT
	.drw UNLOOP
	lcall COMMAXT
	lcall LIT
	.drw branch
	lcall COMMABRANCH
	lcall IHERE
	lcall COMMANODEST
	ljmp TOL

; OTHER OPERATIONS ==============================

;X WITHIN   n1|u1 n2|u2 n3|u3 -- f   n2<=n1<n3?
;  OVER - >R - R> U< ;          per ANS document
	.drw link
	.link
	.fw	WITHIN
WITHIN:	lcall OVER
	lcall MINUS
	lcall TOR
	lcall MINUS
	lcall RFROM
	ljmp ULESS

;C MOVE    addr1 addr2 u --     smart move
;             VERSION FOR 1 ADDRESS UNIT = 1 CHAR
;  >R 2DUP SWAP DUP R@ +     -- ... dst src src+n
;  WITHIN IF  R> CMOVE>        src <= dst < src+n
;       ELSE  R> CMOVE  THEN ;          otherwise
	.drw link
	.link
	.fw	MOVE
MOVE:	lcall TOR
	lcall TWODUP
	lcall SWOP
	lcall DUP
	lcall RFETCH
	lcall PLUS
	lcall WITHIN
	lcall zerosense
	jz MOVE1
	lcall RFROM
	lcall CMOVEUP
	sjmp MOVE2
MOVE1:  lcall RFROM
	lcall CMOVE
MOVE2:  ret

;C DEPTH    -- +n        number of items on stack
;   SP@ S0 SWAP - 2/ ;   16-BIT VERSION!
	.drw link
	.link
	.fw	DEPTH
DEPTH:	lcall SPFETCH
	lcall S0
	lcall SWOP
	lcall MINUS
	ljmp TWOSLASH

;C ENVIRONMENT?  c-addr u -- false   system query
;                         -- i*x true
;   2DROP 0 ;       the minimal definition!
	.drw link
	.link
	.fw	ENVIRONMENT?
ENVIRONMENTQ:	lcall TWODROP
	lcall LIT
	.drw 0x0
	ret

; UTILITY WORDS AND STARTUP =====================

;X WORDS    --          list all words in dict.
;   LATEST @ BEGIN
;       DUP ICOUNT 7F AND ITYPE SPACE
;       NFA>LFA I@
;   DUP 0= UNTIL
;   DROP ;
	.drw link
	.link
	.fw	WORDS
WORDS:	lcall LATEST
	lcall FETCH
WDS1:   lcall DUP
	lcall ICOUNT
	lcall LIT
	.drw 0x7f
	lcall ANDD
	lcall ITYPE
	lcall SPACE
	lcall NFATOLFA
	lcall IFETCH
	lcall DUP
	lcall ZEROEQUAL
	lcall zerosense
	jz WDS1
	ljmp DROP

;X .S      --           print stack contents
;   SP@ S0 - IF
;       SP@ S0 2 - DO I @ U. -2 +LOOP
;   THEN ;
	.drw link
	.link
	.fw	.S
DOTS:	lcall SPFETCH
	lcall S0
	lcall MINUS
	lcall zerosense
	jz DOTS2
	lcall SPFETCH
	lcall S0
	lcall LIT
	.drw 0x2
	lcall MINUS
	lcall XDO
DOTS1:  lcall II
	lcall FETCH
	lcall UDOT
	lcall LIT
	.drw -2
	lcall pluslpsense
	jz DOTS1
	lcall UNLOOP
DOTS2:  ret

;Z COLD     --      cold start Forth system
;   RESTORE-RAM
;   LATEST @ NFA>CFA EXECUTE ;
	.drw link
	.link
	.fw	COLD
COLD:	lcall RESTORERAM
	lcall LATEST
	lcall FETCH
	lcall NFATOCFA
	ljmp EXECUTE	; must never return!

;	$EJECT
; ===============================================
; CamelForth for the Intel 8051
; Primitive testing code
;
; This is the "minimal" test of the CamelForth
; kernel.  It verifies the threading and nesting
; mechanisms, the stacks, and the primitives
;   DUP EMIT EXIT lit branch ONEPLUS.
; It is particularly useful because it does not
; use the DO..LOOP, multiply, or divide words,
; and because it can be used on embedded CPUs.
; The numeric display word .A is also useful
; for testing the rest of the Core wordset.
;
; Much of this code has been retained because it
; supports DUMP.  It can be deleted without
; affecting the CamelForth kernel.  Be careful
; not to delete the equates at the end of file.
; ===============================================
;
;       Extra primitives for the testing code.
;
SWAB:	mov a,dph
	mov dph,dpl
	mov dpl,a
	ret

LO:	anl dpl,#0x0f
	mov dph,#0
	ret

HI:	anl dpl,#0xf0
	mov a,dpl
	rr a
	rr a
	rr a
	rr a
	mov dpl,a
	mov dph,#0
	ret

DOTHEX:	mov a,dpl
	clr c
	subb a,#0x0a
	jc numeric
	add a,#7
numeric:	add a,#0x3a
	mov dpl,a
	lcall EMIT
	ret

DOTHH:	lcall DUP
	lcall HI
	lcall DOTHEX
	lcall LO
	lcall DOTHEX
	ret
;
;       : .B ( a - a+1)  DUP C@ .HH 20 EMIT 1+ ;
;
DOTB:	lcall DUP
	lcall ICFETCH
	lcall DOTHH
	lcall LIT
	.drw 0x20
	lcall EMIT
	lcall ONEPLUS
	ret
;
;       : .A ( a)  DUP SWAB .HH .HH 20 EMIT ;
;
DOTA:	lcall DUP
	lcall SWAB
	lcall DOTHH
	lcall DOTHH
	lcall LIT
	.drw 0x20
	lcall EMIT
	ret

;
;       : DUMP  ( a n -- )
;               0 DO
;                  CR DUP .A SPACE
;                  .B .B .B .B  .B .B .B .B
;                  .B .B .B .B  .B .B .B .B
;               16 +LOOP DROP ;
;
	.drw link
	.link
	.fw	IDUMP
IDUMP:	lcall LIT
	.drw 0
	lcall XDO
dump1:  lcall CRR
	lcall DUP
	lcall DOTA
	lcall SPACE
	lcall DOTB
	lcall DOTB
	lcall DOTB
	lcall DOTB
	lcall DOTB
	lcall DOTB
	lcall DOTB
	lcall DOTB
	lcall DOTB
	lcall DOTB
	lcall DOTB
	lcall DOTB
	lcall DOTB
	lcall DOTB
	lcall DOTB
	lcall DOTB
	lcall LIT
	.drw 16
	lcall xplusloop
	jz dump1
	lcall UNLOOP
	lcall DROP
	ret

; EXAMPLE I/O ===================================
; !P1     c --      output byte to port 1
	.drw link
	.link
	.fw	!P1
STOREP1: mov p1,dpl	; output TOS char to P1
	ljmp poptos	; pop new TOS

; @P1      -- c      get byte from port 1
	.drw link
	.link
	.fw	@P1
FETCHP1: lcall pushtos	; push old TOS
	mov dpl,p1	; get P1 byte in TOS
	mov dph,#0
	ret

; !TCON     c --      output byte to TCON reg
	.drw link
	.link
	.fw	!TCON
STORETCON:	mov tcon,dpl ; output TOS char to TCON
	ljmp poptos	; pop new TOS

; @TCON      -- c      get byte from TCON reg
	.drw link
	.link
	.fw	@TCON
FETCHTCON:	lcall pushtos ; push old TOS
	mov dpl,tcon	; get TCON byte in TOS
	mov dph,#0
	ret

; Other I/O ports are left as an exercise for the
; student.

; ===============================================
; NEW FORTH WORDS

; (?do)    n1|u1 n2|u2 --  R: -- sys1 sys2
;                          run-time code for ?DO
; returns Z flag if n1=n2, so following JZ
; instruction can skip loop.
; ?DO..LOOP is compiled as:
;   lcall xqdo  jz $1
;   $2: ...body of loop...
;   lcall xloop  jz $2  lcall unloop
;   $1:
	.drw link
	.link
	.fw	(?DO)
XQDO:	; limit index --
	pop dr3		; get return adrs in r3:r2
	pop dr2
	push dr6	; push previous index
	push dr7
	mov a,@r0	; get (-limit) + 8000h
	inc r0	;   = (~limit) + 8001h
	cpl a	;   in r5:r4
	add a,#01
	mov r4,a
	mov a,@r0
	inc r0
	cpl a
	addc a,#0x80
	mov r5,a
	push dr4	; push this fudge factor
	push dr5
	mov a,r4	; fudge+index -> r7:r6
	add a,dpl
	mov r6,a
	mov a,r5
	addc a,dph
	mov r7,a
	; if index=limit, fudge+index = 8000h
	; and we should return Z to skip loop
	xrl a,#0x80
	orl a,r6	; A=zero if index=limit
	jnz xqdo1	; A=NZ if ok to enter loop
	; if loop is skipped, need to unloop stk
	dec sp		; discard fudge factor
	dec sp
	pop dr7		; restore previous loop index
	pop dr6
	; A=Z to skip loop, NZ otherwise
xqdo1:  mov dpl,@r0	; pop new TOS...
	inc r0
	mov dph,@r0
	inc r0	; ...preserving A
	push dr2	; restore return addr
	push dr3
	ret

; ?DO       -- adrs   L: -- 0
;   0 >L                       marker for LEAVEs
;   ['] xqdo ,BRANCH  IHERE ,NODEST >L
;   IHERE ; IMMEDIATE      target for bwd branch
; Note that this uses the LEAVE-resolve mechanism
; to complete the branch after  'lcall xqdo'
	.drw link
	.link
	.fwi	?DO
QDO:	lcall LIT
	.drw 0x0
	lcall TOL
	lcall LIT
	.drw XQDO
	lcall COMMABRANCH
	lcall IHERE
	lcall COMMANODEST
	lcall TOL
	ljmp IHERE

; DOIDOES, code action of IDOES> clause
; (internal code fragment, not a Forth word)
; entered by       LCALL fragment
;                  data
;                       ...
;        fragment: LCALL DOIDOES
;                  high-level thread
; Enters high-level thread with address of
; CODE SPACE data on top of stack.  HARVARD MODEL.
doidoes:	; -- a-addr     support routine for IDOES>
	dec r0		; push old TOS
	mov @r0,dph
	dec r0
	mov @r0,dpl
	pop dr5		; addr of DOES> clause
	pop dr4		;   Forth code
	pop dph		; addr of defined word's
	pop dpl		;   Param. field
	push dr4	; restore Forth code addr
	push dr5
	ret	; go do the DOES> code

; IDOES>    --      change action of latest def'n
;   ['] (DOES>) ,XT
;   doidoes ,CF ; IMMEDIATE
; IDOES> is used when the DOES clause needs to access
; the data appended in Code space (not Data space).
; This definition is the same as DOES> except that
; it uses 'doidoes' instead of 'dodoes'.
	.drw link
	.link
	.fwi	IDOES>
IDOES:	lcall LIT
	.drw XDOES
	lcall COMMAXT
	lcall LIT
	.drw doidoes
	ljmp COMMACF

;X MARKER    --
;    IALIGN	; align to flash ROM block	(Cygnal)
;    LATEST @  IHERE	; get these BEFORE create
;    <BUILDS  HERE I,  I, I,
;    IDOES>
;       IHERE SWAP	; save highest ROM addr used	(Cygnal)
;	DUP I@ DP !	; first cell = saved HERE (from CREATE)
;       CELL+ DUP I@ IDP !	; second cell = saved IHERE
;       CELL+ I@ LATEST !	; third cell = saved LATEST
;       IHERE SWAP +ERASE WIPE ;	; erase the freed ROM	(Cygnal)
; 8051 Harvard model: note that CREATE appends the current
; value of HERE as the first cell of the Code space parameter
; field.
	.drw link
	.link
	.fw	MARKER
MARKER:	acall IALIGN	; Cygnal
	lcall LATEST
	lcall FETCH
	lcall IHERE
	lcall BUILDS
	lcall HERE
	lcall ICOMMA
	lcall ICOMMA
	lcall ICOMMA
	lcall XDOES
	; IDOES> definition
	lcall doidoes
	lcall IHERE	; Cygnal
	lcall SWOP	; Cygnal
	lcall DUP
	lcall IFETCH
	lcall DP
	lcall CSTORE
	lcall CELLPLUS
	lcall DUP
	lcall IFETCH
	lcall IDP
	lcall STORE
	lcall CELLPLUS
	lcall IFETCH
	lcall LATEST
	lcall STORE
	lcall IHERE	; Cygnal
	lcall SWOP	; Cygnal
	lcall PLUSERASE	; Cygnal
	ajmp WIPE	; Cygnal


; ===============================================
; CYGNAL C8051F SUPPORT WORDS
	.if CYGNAL

;Z +WRITE    --
; enable Flash memory writes
	.drw link
	.link
	.fw	+WRITE
PLUSWRITE:	mov PSCTL,#1	; enable Write
	ret

;Z -WRITE    --
; disable Flash memory writes
	.drw link
	.link
	.fw	-WRITE
MINUSWRITE:	mov PSCTL,#0	; disable Erase & Write
	ret

;Z +ERASE    --
; enable Flash memory erase
	.drw link
	.link
	.fw	+ERASE
PLUSERASE:	mov PSCTL,#3	; enable Erase & Write
	ret

;Z -ERASE    --
; disable Flash memory erase; RE-ENABLES WRITE.
	.drw link
	.link
	.fw	-ERASE
MINUSERASE:	mov PSCTL,#1	; disable Erase, enable Write
	ret

;Z WIPE     loaddr hiaddr --
; Erases all flash sectors covering the range loaddr..(hiaddr-1).
; +ERASE must be used before calling WIPE.
;   8080 UMIN  SWAP  2000 UMAX
;   FE00 AND      ( round loaddr down to 512 byte sector)
;   ?DO  0 I IC!  200 +LOOP  -ERASE ;
	.drw link
	.link
	.fw	WIPE
WIPE:	lcall LIT
	.drw  0x8080
	lcall UMIN
	lcall SWOP
	lcall LIT
	.drw  0x2000
	lcall UMAX
	lcall LIT
	.drw  0xFE00
	lcall ANDD
	lcall XQDO
	jz wipe2
wipe1:  lcall LIT
	.drw  0
	lcall II
	lcall ICSTORE
	lcall LIT
	.drw  0x200
	lcall xplusloop
	jz wipe1
	lcall UNLOOP
wipe2:  sjmp MINUSERASE

;Z IALIGN    --
; Adjust the ROM dictionary pointer, IDP, up to the next
; 512-byte boundary.  If already on a 512-byte boundary,
; no action.
;   IHERE  1FF + FE00 AND  IDP ! ;
	.drw link
	.link
	.fw	IALIGN
IALIGN:	lcall IHERE
	lcall LIT
	.drw  0x1FF
	lcall PLUS
	lcall LIT
	.drw  0xFE00
	lcall ANDD
	lcall IDP
	ljmp STORE

;Z SAVE-RAM   --
; Save internal RAM locations 08-7F to flash ROM.
;   +ERASE 8000 DUP IC!  +WRITE	; erase flash ROM 8000-807F
;   08 8000 78 D->I ;
	.drw link
	.link
	.fw	SAVE-RAM
SAVERAM:	acall PLUSERASE
	lcall LIT
	.drw  0x8000
	lcall DUP
	lcall ICSTORE
	lcall PLUSWRITE
	lcall LIT
	.drw  0x08
	lcall LIT
	.drw  0x8000
	lcall LIT
	.drw  0x78
	ljmp DTOI

	.else

	; For a generic 8052, define "dummy" equivalents for the
	; Cygnal Flash ROM routines...so we don't have to change
	; MARKER and MAIN.  Note that these won't appear in the dictionary.
PLUSWRITE:
MINUSWRITE:
PLUSERASE:
MINUSERASE:
IALIGN:	ret

WIPE:	ljmp TWODROP

	.endif ; CYGNAL

;Z RESTORE-RAM    --
; Restore internal RAM locations 08-7F from flash ROM (Cygnal)
; or from ordinary ROM (generic 8052).
; Note that this will also restore the user variables.
;   raminit 08 raminitsize I->D ;
	.drw link
	.link
	.fw	RESTORE-RAM
RESTORERAM:	lcall LIT
	.drw  raminit
	lcall LIT
	.drw  0x08
	lcall LIT
	.drw  raminitsize
	ljmp ITOD

; ===============================================
; EXPERIMENTAL CODE

	.include "tovalue.asm"	; VALUE, TO
	.include "code.asm"	; CODE, END-CODE
	.include "sfaccess.asm"	; SF@:, SF!:

; ===============================================
; Last word in Forth dictionary will be executed by COLD.
; Do not put other words after this word unless you want
; them to autoboot!
; : MAIN
;   +WRITE	; Cygnal
;   ." 8051 CamelForth etc."
;   ABORT ;
	.drw link
	.link
	.fw	MAIN
MAIN:	lcall PLUSWRITE
	lcall XISQUOTE
       .db 35
	.ascii	'8052 CamelForth v1.4  19 Mar 2001'
       .db 0xd,0x0a
	lcall ITYPE
	ljmp ABORT	; ABORT never returns

; Initial dictionary pointer for CamelForth.
; DO NOT delete!
	.equ	lastword,link	; NFA of final word

; Initial values for user area (for RESTORE-RAM)
; For the Cygnal C8051F, initial user values are stored in the
;    128-byte flash ROM area at 8000h.  This will be included when
;    the kernel is downloaded to flash ROM.  The user can then store
;    new initialization values interactively with SAVE-RAM.
; For the generic 8052, initial user values are just stored at
;    the end of the program image.  They are fixed as assembly time.

	.if CYGNAL
	.org     0x8000
	.endif
raminit:
	; RAM 08-13 = User (system) variables, 12 bytes
	.db 0		; >IN
	.db 10		; BASE
	.db 0		; STATE
	.db dataram	; DP
	.db 0,0		; SOURCE init'd elsewhere
	.drw lastword	; LATEST
	.db 0	; HP init'd elsewhere
	.db 0		; LP init'd elsewhere
	.drw coderam	; IDP

	; RAM 14-7F = Application variables (available to user)
	; Additional RAM init values can be added here.

	; This must follow RAM init values.
	.if CYGNAL
	.equ	raminitsize,0x78	; for Cygnal, always load RAM 08-7F
	.else
	.equ	raminitsize,.-raminit
	.endif

;        END
