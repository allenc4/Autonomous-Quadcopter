all:
	avrdude -C /usr/share/arduino/hardware/tools/avrdude.conf -p ATmega2560 -P /dev/ttyACM0 -cstk500v2 -b 115200 -V -v -v -D -U flash:w:AP_OpticalFlow_test.hex:i 
