TNC2 Firmware
====

# Building the Firmware

To build the software, just type "make".  This will create a file called ``images/mobilinkd-tnc1.elf``

The EEPROM section needs to be removed from the ihex file.

    $ avr-objcopy -O ihex -R.eeprom images/mobilinkd-tnc1.elf images/mobilinkd-tnc1.hex

The hex file can then be uploaded using ``avrdude -c avr109`` or using the
Mobilinkd Android configuration app.  When using ``avrdude`` enter the bootloader by holding down the power button and pressing the reset button.  The TNC will enter the bootloader.  You have 8 seconds to start the upload.

    $ avrdude -c avr109 -p m328p -P /dev/rfcomm0 -U images/mobilinkd-tnc1.hex

If the bootloader has not been activated within 8 seconds, the TNC resets.    

See below for setting up the Bluetooth serial port on Linux.

## Bootloader

The Mobilinkd TNC2 uses the XBoot++ bootloader: https://github.com/alexforencich/xboot

The configuration file for the bootloader is in the xboot subdirectory.
An existing ``xboot.hex`` image is in the ``images/`` directory.

To build an image which includes the bootloader, either use the supplied
bootloader image or build the xboot++ bootloader using the supplied xboot
configuration file.  Then merge the two files with srec_cat.

    $ cd images
    $ srec_cat xboot.hex -intel mobilinkd-tnc1.hex -intel -o mobilinkd-boot.hex -intel

The output file, ``mobilinkd-boot.hex``, can then be uploaded via the
ISP port on the board using an ISP programmer such as the USBASP v2.0.  This
is the programmer that I use, along with a custom cable with pogo pins.

    $ avrdude -p m328pb -c usbasp -v -U mobilinkd-boot.hex
    
If you are creating your own version of the board, you will need to set the
fuses on the TNC before flashing the image.

## Fuses

The MCU fuses must be set as follows:

    lfuse: 0xFF
    hfuse: 0xD2
    efuse: 0x05
    
    $ avrdude -p m328pb -c usbasp -v -B 40 -U lfuse:w:0xFF:m -U hfuse:w:0xD2:m -U efuse:w:0x05:m

The lock bit (set after the firmware with bootloader is written) should be set to:

    lock: 0x2F

    $ avrdude -p m328pb -c usbasp -v -B 40 -U lock:w:0x2F:m 

## Configuring a Bluetooth Serial Port on Linux

This section assumes some familiarity with working with Bluetooth devices on Linux.  At a minimum, you need to know how to pair a device with your system.

The first thing to do is to pair the Bluetooth device.  This is best done
via the ``Bluetooth Setting`` applet.  Once paired, you will need the MAC
address of the TNC.  Otherwise you can use the ``bluetoothctl`` program
interactively from the command line.

    $ bluetoothctl
    Agent registered
    [NEW] Device 20:16:11:15:20:34 Mobilinkd TNC2
    # pair 20:16:11:15:20:34
    Attempting to pair with 20:16:11:15:20:34
    [CHG] Device 20:16:11:15:20:34 Connected: yes
    Request PIN code
    [agent] Enter PIN code: 1234
    [CHG] Device 20:16:11:15:20:34 UUIDs: 00001101-0000-1000-8000-00805f9b34fb
    [CHG] Device 20:16:11:15:20:34 ServicesResolved: yes
    [CHG] Device 20:16:11:15:20:34 Paired: yes
    Pairing successful
    [CHG] Device 20:16:11:15:20:34 ServicesResolved: no
    [CHG] Device 20:16:11:15:20:34 Connected: no
    [CHG] Device 20:16:11:15:20:34 RSSI: -58
    # quit
    ...
    $
    

Set up the serial port with:

    $ sudo rfcomm bind 0 <MAC ADDRESS> 1

For example, using the MAC address above:

    $ sudo rfcomm bind 0 20:16:11:15:20:34 1

This will create the device ``/dev/rfcomm0``.

When done using this device, before turning off then TNC, unbind the port.

    $ sudo rfcomm unbind 0
