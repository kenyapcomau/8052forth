	.list	(me)

	.macro	.drw a
	.db	<a
	.db	>a
	.endm

	.macro	.fw s,?l1,?l2
	.db	NONIMMED,l2-l1
l1:
	.ascii	"s"
l2:
	.endm

	.macro	.fwq s,?l1,?l2
	.db	NONIMMED,l2-l1
l1:
	.ascii	/s/
l2:
	.endm

	.macro	.fwi s,?l1,?l2
	.db	IMMED,l2-l1
l1:
	.ascii	"s"
l2:
	.endm

	.macro	.fwiq s,?l1,?l2
	.db	IMMED,l2-l1
l1:
	.ascii	/s/
l2:
	.endm

	.macro 	.link
	.equ	link,.+1
	.endm
