; ====================================================================
; Experimental CODE defining word for CamelForth/8052 (internal RAM)
; (c) 2000  Bradford J. Rodriguez
; Permission is granted to freely copy, modify, and distribute this
; program for personal or educational use.  Commercial inquiries should
; be directed to the author at 115 First St., #105, Collingwood, Ontario
; L9Y 4W3 Canada
; ====================================================================
; REVISION HISTORY
; $Log: Code.asm,v $
;
; 20 Nov 2000 - initial (alpha test) version.

; ====================================================================
; DESCRIPTION
;
; CODE and END-CODE are used to define 8051 assembly language words
; in the target dictionary.  In the absence of a resident 8051
; assembler, the user must hand-assemble the desired code and
; use I, and IC, to compile opcodes and operands into the dictionary.

;Z CODE   --      create a header for a CODE definition
;   LATEST @ I, NONIMMED IC,          link & immed field
;   IHERE LATEST !                    new "latest" link
;   BL IWORD IC@ 1+ IALLOT            name field
;   ;	                              there is no code field!
; Machine code should be appended with I, and IC,
        .drw link
	.link
	.fw	CODE
KODE:   lcall LATEST
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
        ljmp IALLOT

;Z END-CODE   --   end a CODE definition
;  No special action is required to end a CODE definition.
;  This is a dummy definition for cosmetic use.
        .drw link
	.link
	.fw	END-CODE
ENDCODE: ret
