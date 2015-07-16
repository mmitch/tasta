# only set overrides when needed
# defaults are taken from Makefile.orig

MCU = attiny85

# F_OSC should be unused!
F_OSC = 16500000

# set PLL clock (lfuse), everything else default
## AVRDUDE_FUSES = -U lfuse:w:0xC1:m -U hfuse:w:0xDF:m -U efuse:w:0xFF:m -U lock:w:0xFF:m 

# from http://codeandlife.com/2012/02/22/v-usb-with-attiny45-attiny85-without-a-crystal/
# - use SUT=10 (slow rising power, 64ms)
# - use brownout detection at 2.7V
AVRDUDE_FUSES = -U lfuse:w:0xE1:m -U hfuse:w:0xDD:m

# extra source file:
SRC = $(TARGET).c usbdrv/usbdrv.c

# extra assembler file:
ASRC = usbdrv/usbdrvasm.S

##	-U lfuse:w:0xC1:m 	-U hfuse:w:0xDF:m 

# and delegate to the default Makefile:
include Makefile.orig
