; ====================================================================
; Experimental VALUE/TO code for CamelForth/8052 (internal RAM)
; (c) 2000  Bradford J. Rodriguez
; Permission is granted to freely copy, modify, and distribute this
; program for personal or educational use.  Commercial inquiries should
; be directed to the author at 115 First St., #105, Collingwood, Ontario
; L9Y 4W3 Canada
; ====================================================================
; REVISION HISTORY
; $Log: Tovalue.asm,v $
;
; 7 Nov 2000 - initial (alpha test) version.

; ====================================================================
; DESCRIPTION
;
; This code does "run time" binding using the 8051 user flags
; PSW.5 and PSW.1.  These flags are otherwised unused by CamelForth.
; They must be left undisturbed by the application!
;
;     PSW.5   PSW.1     VALUE action
;       0       0       fetch
;       0       1       RAM address
;       1       0       store
;       1       1       plusstore
;
; This is slightly slower than a "compile time" binding approach,
; but much simpler.

;X TO   --      store data in VALUE which follows
        drw link
link	set	$+1
	db	NONIMMED,2,'TO'
TO:     setb    psw.5
        ret

;Z +TO  --      add data to VALUE which follows
        drw link
link	set	$+1
	db	NONIMMED,3,'+TO'
PLUSTO: setb    psw.5
        setb    psw.1
        ret

;Z ADR  --      get address of VALUE which follows
        drw link
link	set	$+1
	db	NONIMMED,3,'ADR'
ADR:    setb    psw.1
        ret


; Usage:
;       VALUE FOO               defines a RAM value named FOO
;       FOO                     returns contents of FOO
;       n TO FOO                stores n in FOO
;       n +TO FOO               adds n to FOO
;       ADR FOO                 returns address of FOO

; --------------------------------------------------------------------
; CELL values

;X VALUE     --         define cell VALUE variable
;   <BUILDS HERE I,     Harvard model, 8052
;   CELL ALLOT
;   DOES> (machine code fragment)
        drw link
link	set	$+1
	db	NONIMMED,5,'VALUE'
VALUE:  lcall BUILDS
        lcall HERE
        lcall ICOMMA
        lcall CELL
        lcall ALLOT
        lcall XDOES
; DOVALUE, code action of VALUE,
; entered by CALL DOVALUE
dovalue: ; -- x     exec action of VALUE
        dec r0          ; push old TOS
        mov @r0,dph
        dec r0
        mov @r0,dpl
        pop dph         ; get addr of param field
        pop dpl         ;     (in Code memory!)
        lcall IFETCH    ; fetch ROM contents (RAM address)
        ; Now we have ( oldTOS RAMadr ) on stack.
        ; Do the desired action.
        ; Note that this leaves psw.1 and psw.5 cleared.
        jbc psw.1,adrvalue
        jbc psw.5,tovalue
        ljmp FETCH      	; fetch RAM contents
adrvalue: jbc psw.5,plusvalue
        ret                     ; return RAM address
tovalue: ljmp STORE             ; store to RAM
plusvalue: ljmp PLUSSTORE       ; add to RAM

; defined VALUE words are compiled as:
;       <forth header>
;       LCALL dovalue
;       DRW ram-address

; --------------------------------------------------------------------
; BYTE values

;Z CVALUE     --         define cell VALUE variable
;   <BUILDS HERE I,     Harvard model, 8052
;   1 CHARS ALLOT
;   DOES> (machine code fragment)
        drw link
link	set	$+1
	db	NONIMMED,6,'CVALUE'
CVALUE:  lcall BUILDS
        lcall HERE
        lcall ICOMMA
        lcall LIT
        drw 1
        lcall ALLOT
        lcall XDOES
; DOCVALUE, code action of CVALUE,
; entered by CALL DOCVALUE
docvalue: ; -- x     exec action of CVALUE
        dec r0          ; push old TOS
        mov @r0,dph
        dec r0
        mov @r0,dpl
        pop dph         ; get addr of param field
        pop dpl         ;     (in Code memory!)
        lcall IFETCH    ; fetch ROM contents (RAM address)
        ; Now we have ( oldTOS RAMadr ) on stack.
        ; Do the desired action.
        ; Note that this leaves psw.1 and psw.5 cleared.
        jbc psw.1,adrcvalue
        jbc psw.5,tocvalue
        ljmp CFETCH      	; fetch RAM contents
adrcvalue: jbc psw.5,pluscvalue
        ret                     ; return RAM address
tocvalue: ljmp CSTORE             ; store to RAM
pluscvalue: ljmp CPLUSSTORE       ; add to RAM


