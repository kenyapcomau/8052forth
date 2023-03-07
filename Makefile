AS=sdas8051
LD=sdld

all: camel52.ihx

%.ihx: %.asm
	$(AS) -o -l $<
	$(LD) -i $(<:.asm=.rel)
