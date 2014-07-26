megafighter
===========

dual Midifighter clone using an arduino Mega2560 with the dual mocolufa firmware and adafruit neopixel library

The code is setup for 32 buttons (2x 4x4 grids), 16faders and addressable RGB LEDS that are user programmable and respond to midi, There is support for rotary encoders and there is also support for saving settings in the onboard EEPROM.  As the mega2560 has 4 hardware serial ports, I also added preliminary support for midi-thru so that we can use these boxes as a simple usb to midi adapter.  I have commented on the different data byte types for rotary encoders for various DJ software, if they don't work for you, try a different pair of data bytes.

I have commented it as much as possible so anything that's not explained here, you should be able to figure out.  

Download the dual moco-lufa firmware from here:
http://morecatlab.akiba.coocan.jp/lab/index.php/aruino/midi-firmware-for-arduino-uno-moco/?lang=en

You will need to get the mocolufa firmware onto the mega2560's 8u2 or 16u2 chip, follow the instructions on the above link to do so, you'll also need to look at the instructions on how to boot between the 2 modes of the firmware (usb-serial for programming/usb-midi to actually use the firmware) using a simple jumper.

Then open the megafighter.ino file in your arduino ide, compile, flash it to your unit, remove the programming jumper, you'll need to remove the usb cable and plug it back in to reboot the mega2560 bootloader, then you can use something like midi-ox to test and see that all your buttons and faders/knobs/endcoders are working and outputting midi as expected.

The leds should respond to the midi messages that are sent by the appropriate button, so for something like Traktor, just send midi note/led messages back to the megafighter to get the leds to light up.
