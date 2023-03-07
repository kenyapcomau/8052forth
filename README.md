# CamelForth for the 8052

**WARNING This is work in progress**

This is a revival of Brad Rodriguez's CamelForth for the 8052 downloadable as cam52-14.zip. It uses the 256 byte internal RAM. It was originally assembled with the Metalink ASM51 assembler. I have edited it to assemble with the asxxxx 8051 open source assembler. The intention is to try to make it work on the WCH CH552 and above 8051 derivatives. The original distribution is in orig/.

The easiest way to get the required tools is to get the SDCC C compiler toolkit where they have renamed the assembler and linker sdas8051 and sdld respectively.

At the moment it assembles to exactly the same binary file as the one in the archive, proving that I have not broken anything. Incidentally the hex file that's in the archive was built from a slightly older version of the provided source, only different i the greeting message.

The license of this code is GPL3, see copying.
