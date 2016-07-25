#include "Serial.h"

Serial::Serial(AP_HAL::UARTDriver *driver) {
	_lastPktTime = 0;
	_driver = driver;
}

/**
 * Begin serial communication and start by sending initialization data to connected
 * controller (RC min, max values, etc)
 */
void Serial::init(uint32_t baudRate) {
	_driver->begin(baudRate);
	write("APM connected\n");
}

/**
 * Reads the data input from the Serial line (expecting RC channel values)
 * Expects key:value, separated by comma ending with a (checksum)
 */
void Serial::read() {
	int bufLen = 0;

	// Read in any serial commands
	int16_t numBytes = _driver->available();
	if (numBytes == 0 || numBytes >= 512) {
		// Either nothing to read or it is over the character limit, so exit immediately
		return;
	}

	while (numBytes > 0) {
		_readBuf[bufLen] = (char)_driver->read();
		bufLen++;
		numBytes--;
	}
	_readBuf[bufLen] = '\0';

	// Get the checksum (should be the only value in parenthesis)
	char *chkStart = strchr(_readBuf, '(');
	if (chkStart == NULL) {
		return;  // Checksum should be in parenthesis but it wasnt found, so exit
	}
	int chkStartPos = chkStart - _readBuf + 1;
	char *chkEnd = strchr(chkStart, ')');
	if (chkEnd == NULL) {
		return;
	}
	int chkEndPos = chkEnd - _readBuf;
	if (chkEndPos <= chkStartPos) {
		return;
	}

	char *chksum = (char *)malloc(chkEndPos - chkStartPos + 1);
	strncpy(chksum, _readBuf + chkStartPos, chkEndPos - chkStartPos);
	chksum[chkEndPos - chkStartPos] = '\0';
	_readBuf[chkStartPos-1] = '\0';
#if DEBUG == ENABLED
	char t[200];
	sprintf(t, "Checksum: %s\n", chksum);
	_driver->write(t);

#endif

	// Get the actual checksum and compare to the expected
	if (!checksumValid(_readBuf, chksum)) {
		delete chksum;
		return;
	}

	// _readBuf should now consist of param:value,param:value,... so split by ','
	char *cmd = strtok(_readBuf, ",");
	while (cmd != NULL) {
		// Process all param:value pairs
		process(cmd);
		cmd = strtok(NULL, ",");
	}

	delete chksum;
}

void Serial::write(const char *str) {
	_driver->write(str);
}

/**
 * Processes the command received from the serial request.
 * Expects commands in form of param:value to set parameters.
 */
void Serial::process(char *cmd) {
	// Break up the string into param:value
	char *split = strtok(cmd, ":");
	if (split == NULL) {
		return;
	}

	unsigned int i = 0;
	for (i = 0; i < strlen(split); i++) {
		_param[i] = split[i];
	}
	_param[i] = '\0';  // Null terminate the string

	split = strtok(NULL, ":");
	if (split == NULL) {
		return;
	}

	for (i = 0; i < strlen(split); i++) {
		_value[i] = split[i];
	}
	_value[i] = '\0';


//	// At this point, we have the parameter to be set (_param) and value to set it (_value)

	// Process the command
#if DEBUG == ENABLED
	char t[200];
	sprintf(t, "Attempting to set %s to %s\n", _param, _value);
	write(t);
#endif

	// Convert the value to a number (if applicable) and handle any error checking
	char *endptr;
	int errno = 0;
	long val = strtol(_value, &endptr, 10);
	bool valIsNum = false;
	if (endptr == _value)
	{
	    // nothing parsed from the string so do nothing
	}
	if ((val == LONG_MAX || val == LONG_MIN) && errno == ERANGE)
	{
	    // out of range
	}

	if (strcmp(_param, "thr")) {
		if (valIsNum) {
			rc_channels[RC_CHANNEL_THROTTLE].set_pwm(val);
		}
	} else if (strcmp(_param, "roll")) {
		if (valIsNum) {
			rc_channels[RC_CHANNEL_ROLL].set_pwm(val);
		}
	} else if (strcmp(_param, "pitch")) {
		if (valIsNum) {
			rc_channels[RC_CHANNEL_PITCH].set_pwm(val);
		}
	} else if (strcmp(_param, "yaw")) {
		if (valIsNum) {
			rc_channels[RC_CHANNEL_YAW].set_pwm(val);
		}
	} else {
#if DEBUG == ENABLED
		char t[200];
		sprintf(t, "Unknown command entered: %s\n", _param);
		write(t);
#endif
		return;
	}


	// Only set the last packet time if we got a valid message (key:value:checksum) and checksums match
	_lastPktTime = hal.scheduler->millis();
}

bool Serial::checksumValid(char *str, char *chk) {

#if DEBUG == ENABLED
	char t[200];
	sprintf(t, "Getting checksum for \"%s\"\nLength: %d\n", str, strlen(str));
	write(t);
#endif

	long sum = 0;
	for (unsigned int i = 0; i < strlen(str); i++) {
		sum += str[i];
	}

#if DEBUG == ENABLED
	if (sum != strtol(chk, NULL, 10)) {
		char t[200];
		sprintf(t, "Invalid checksum. Expected %ld \t Got %ld\n", strtol(chk, NULL, 10), sum);
		write(t);
	}
#endif

	return (sum == strtol(chk, NULL, 10));
}

uint32_t Serial::lastReceiveTime() {
	return _lastPktTime;
}
