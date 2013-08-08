tnc1
====

Mobilinkd TNC1 Firmware

====
To build the software, just type "make".  The EEPROM section needs to be
removed from the ihex file.

avr-objcopy -O ihex -R.eeprom images/mobilinkd-tnc1.elf images/mobilinkd-tnc1.hex

The hex file can then be uploaded using "avrdude -c avr109" or using the
Mobilinkd Android configuration app.

To build an image which includes the bootloader, build the xboot++ bootloader
using the supplied xboot configuration file.  Then merge the two files with
srec_cat.

srec_cat xboot/xboot.hex -intel mobilinkd-tnc1/images/mobilinkd-tnc1.hex \
    -intel -o mobilinkd-tnc1/images/mobilinkd-boot.hex -intel

The output file, mobilinkd-boot.hex, can then be uploaded via the ISP port on
the board using an ISP programmer.
