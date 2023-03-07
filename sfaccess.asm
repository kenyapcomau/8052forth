; ====================================================================
; Experimental SFR access words for CamelForth/8052 (internal RAM)
; (c) 2001  Bradford J. Rodriguez
; Permission is granted to freely copy, modify, and distribute this
; program for personal or educational use.  Commercial inquiries should
; be directed to the author at 115 First St., #105, Collingwood, Ontario
; L9Y 4W3 Canada
; ====================================================================
; REVISION HISTORY
; $Log: Sfraccess.asm,v $
;
; 3 Jan 2001 - initial (alpha test) version.

; ====================================================================
; DESCRIPTION
;

; SF@:  builds a Forth word which fetches a given 8051 SFR.
;   Usage:   n SF@: name     where 'n' is the SFR address, 0-FF.
;   Then     name            will fetch a byte from the SFR. ( -- c )
;
; SF@:   ( n -- )       Builds...
;   CODE                  header of CODE word
;   ['] DUP COMPILE,      CALL PUSHTOS  (push TOS onto memory stack)
;   75 IC, 83 IC, 0 IC,   MOV DPH,#0    (clear TOS hi)
;   85 IC,    IC, 82 IC,  MOV DPL,n     (move SFR to TOS lo)
;   ,EXIT ;               RET
;
; This takes advantage of the fact that COMPILE, builds an 8051 CALL,
; and ,EXIT builds an 8051 RET.  This won't work on other CamelForths!

        .drw link
	.link
	.fw	SF@:
SFFETCH: acall  KODE
        lcall   LIT
        .drw     pushtos
        lcall   COMMAXT
        lcall   LIT
        .drw    0x75
        lcall   ICCOMMA
        lcall   LIT
        .drw    0x83
        lcall   ICCOMMA
        lcall   LIT
        .drw     0
        lcall   ICCOMMA
        lcall   LIT
        .drw    0x85
        lcall   ICCOMMA
        lcall   ICCOMMA
        lcall   LIT
        .drw    0x82
        lcall   ICCOMMA
        ljmp    CEXIT

; SF!:  builds a Forth word which stores a given 8051 SFR.
;   Usage:   n SF!: name     where 'n' is the SFR address, 0-FF.
;   Then     name            will store a byte in the SFR.  ( c -- )
;
; SF!:   ( n -- )       Builds...
;   CODE                  header of CODE word
;   85 IC, 82 IC,  IC,    MOV n,DPL     (move TOS lo to SFR)
;   ['] DROP COMPILE,     CALL POPTOS   (pop new TOS from memory stack)
;   ,EXIT ;               RET

        .drw link
	.link
	.fw	SF!:
SFSTORE: acall  KODE
        lcall   LIT
        .drw    0x85
        lcall   ICCOMMA
        lcall   LIT
        .drw    0x82
        lcall   ICCOMMA
        lcall   ICCOMMA
        lcall   LIT
        .drw     poptos
        lcall   COMMAXT
        ljmp    CEXIT

; SFSETB:  builds a Forth word which sets a given bit in a given SFR.
;   Usage:   bit# n SFSETB: name     where 'n' is the SFR address, 0-FF,
;				     and bit# is 0-7.
;   Then     name            will set that bit in the SFR.  ( -- )
;   NOTE: this will only work with the bit-addressible SFRs, 80,88,90,88....F0,F8.
;
; SFSETB:   ( n -- )
;   +                     combine bit# with register address
;   CODE                  Build header of CODE word
;   0D2 IC,  IC,          Build  SETB bit
;   ,EXIT ;               Build  RET

        .drw link
	.link
	.fw	SFSETB:
SFSETB: lcall   PLUS
	acall   KODE
        lcall   LIT
        .drw    0xD2
        lcall   ICCOMMA
        lcall   ICCOMMA
        ljmp    CEXIT

; SFCLRB:  builds a Forth word which clears a given bit in a given SFR.
;   Usage:   bit# n SFCLRB: name     where 'n' is the SFR address, 0-FF,
;				     and bit# is 0-7.
;   Then     name            will clear that bit in the SFR.  ( -- )
;   NOTE: this will only work with the bit-addressible SFRs, 80,88,90,88....F0,F8.
;
; SFCLRB:   ( n -- )
;   +                     combine bit# with register address
;   CODE                  Build header of CODE word
;   0C2 IC,  IC,          Build  CLR bit
;   ,EXIT ;               Build  RET

        .drw link
	.link
	.fw	SFCLRB:
SFCLRB: lcall   PLUS
	acall   KODE
        lcall   LIT
        .drw    0xC2
        lcall   ICCOMMA
        lcall   ICCOMMA
        ljmp    CEXIT
