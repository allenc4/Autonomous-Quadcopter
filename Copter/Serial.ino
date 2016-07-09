#include "Serial.h"

/**
 * Begin serial communication and start by sending initialization data to connected
 * controller (RC min, max values, etc)
 */
void Serial::init() {

}

/**
 * Reads the data input from the Serial line (expecting RC channel values)
 * Expects key:value pair, separated by comma
 */
long *Serial::getRC(char *str) {
	// Reset rc_channels array
	memset(this->rc_channels, EMPTY, arraySize(this->rc_channels));

	char buf[255] = {0};
	bool cmdWaiting = false;
	int buf_offset = 0;

	while (hal.console->available()) {
		char c = (char) hal.console->read();
		if (c != ',') {
			// Add character to buffer
			buf[buf_offset++] = c;
			cmdWaiting = true;
		} else {
			// comma reached, meaning new value sent. So process previous value
			process(buf);
			cmdWaiting = false;

			// Reset buffer to empty
			buf_offset = 0;
			memset(buf, 0, arraySize(buf));
		}
	}

	// If there is another command waiting to be processed, process it now
	if (cmdWaiting) {
		process(buf);
	}

	// Return the rc_channels array
	return this->rc_channels;
}

void Serial::write(const char *str, int16_t len) {

}

bool checksum(const char *str, int16_t chksum) {

}

/**
 * Processes the command received from the serial request.
 * Expects commands in form of param:value to set parameters.
 */
void process(char *cmd) {

}
