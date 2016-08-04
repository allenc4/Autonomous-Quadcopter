#include "Serial.h"

Serial::Serial(AP_HAL::UARTDriver *driver) {
	_lastPktTime = 0;
	_driver = driver;
	_readBufLen = 0;
}

/**
 * Begin serial communication and start by sending initialization data to connected
 * controller (RC min, max values, etc)
 */
void Serial::init(uint32_t baudRate) {
	_driver->begin(baudRate);
}

//throttle:1200,pitch:0,roll:0,yaw:8(19999)\n
/**
 * Reads the data input from the Serial line (expecting RC channel values)
 * Expects key:value, separated by comma ending with a (checksum)
 */
void Serial::read() {
	char temp[500];

	// Read in any serial commands
	int16_t numBytes = _driver->available();
	if (numBytes == 0) {
		// Nothing to read, so exit immediately
		return;
	}

	while (_driver->available()) {
		char t = _driver->read();
		_readBuf[_readBufLen] = t;
		if (t == '\n') {
			break;
		}

		if (_readBufLen >= 511) {
			// too big, so reset the buffer
			_driver->flush();
			_driver->write("Received more than 512 bytes for a command. Disregarding command.");
			_readBufLen = 0;
			return;
		}

		_readBufLen++;
	}

	// If the last character is not a '\n', there is more data to be received
	// so don't process the command yet
	if (_readBuf[_readBufLen] != '\n') {
		return;
	} else {
		// Null terminate the buffer and reset the counter
		_readBuf[_readBufLen+1] = '\0';
		_readBufLen = 0;
	}

#if DEBUG == ENABLED
	snprintf(temp, sizeof(temp), "APM Received: %s\n", _readBuf);
	_driver->write(temp);
#endif

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
	snprintf(temp, sizeof(temp), "Checksum: %s\n", chksum);
	_driver->write(temp);

#endif

	// Get the actual checksum and compare to the expected
	if (!checksumValid(_readBuf, chksum)) {
		delete chksum;
		return;
	}

	// _readBuf should now consist of param:value,param:value,... so split by ','
	char *cmdSavePtr;
	char *cmd = strtok_r(_readBuf, ",", &cmdSavePtr);
	while (cmd != NULL) {
		// Process all param:value pairs
		process(cmd);
 		cmd = strtok_r(NULL, ",", &cmdSavePtr);
	}

	delete chksum;
}

void Serial::write(const char *str, bool appendChecksum) {
	if (appendChecksum) {
		long checksum = calculateChecksum(str);

		char t[512];
		sprintf(t, "%s(%ld)\n", str, checksum);
		_driver->write(t);
	} else {
		_driver->write(str);
	}
}

/**
 * Processes the command received from the serial request.
 * Expects commands in form of param:value to set parameters.
 */
void Serial::process(char *cmd) {

#if DEBUG == ENABLED
	char t[200];
	snprintf(t, sizeof(t), "Attempting to process |%s|\n", cmd);
	write(t);
	hal.scheduler->delay(200);
#endif

	// Check for initial start first because that wont have a :value
	if (strcmp(cmd, "initial_start") == 0) {
		// The controller board started and is initially requesting initial/parameter information

		// Send each radio min, max, and trim values
		char t[300];
		sprintf(t, "throttle_min:%hd,throttle_max:%hd,throttle_trim:%hd,"
				"roll_min:%hd,roll_max:%hd,roll_trim:%hd,"
				"pitch_min:%hd,pitch_max:%hd,pitch_trim:%hd,"
				"yaw_min:%hd,yaw_max:%hd,yaw_trim:%hd",
				rc_channels[RC_CHANNEL_THROTTLE].radio_min,
				rc_channels[RC_CHANNEL_THROTTLE].radio_max,
				rc_channels[RC_CHANNEL_THROTTLE].radio_trim,
				rc_channels[RC_CHANNEL_ROLL].radio_min,
				rc_channels[RC_CHANNEL_ROLL].radio_max,
				rc_channels[RC_CHANNEL_ROLL].radio_trim,
				rc_channels[RC_CHANNEL_PITCH].radio_min,
				rc_channels[RC_CHANNEL_PITCH].radio_max,
				rc_channels[RC_CHANNEL_PITCH].radio_trim,
				rc_channels[RC_CHANNEL_YAW].radio_min,
				rc_channels[RC_CHANNEL_YAW].radio_max,
				rc_channels[RC_CHANNEL_YAW].radio_trim);

		write(t);
		return;
	} else if (strcmp(cmd, "trim_radio") == 0) {
		for (int i = RC_CHANNEL_MIN; i <= RC_CHANNEL_MAX; i++) {
			rc_channels[i].trim();
		}

		// Save trim values to eeprom
		g.roll_trim.set_and_save(rc_channels[RC_CHANNEL_ROLL].radio_in);
		g.pitch_trim.set_and_save(rc_channels[RC_CHANNEL_PITCH].radio_in);
		g.yaw_trim.set_and_save(rc_channels[RC_CHANNEL_YAW].radio_in);
		g.throttle_trim.set_and_save(rc_channels[RC_CHANNEL_THROTTLE].radio_in);
		write("Trimming RC channels", false);
		return;
	}

	// Break up the string into param:value
	char *split = strtok(cmd, ":");
	if (split == NULL && strcmp(cmd,"initial_start") != 0) {
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


	// At this point, we have the parameter to be set (_param) and value to set it (_value)

	// Process the command

	// Convert the value to a number (if applicable) and handle any error checking
	char *endptr;
	int errno = 0;
	long val = strtol(_value, &endptr, 10);
	bool valIsNum = false;
	if (endptr == _value)
	{
	    // nothing parsed from the string so do nothing
	} else if ((val == LONG_MAX || val == LONG_MIN) && errno == ERANGE)
	{
	    // out of range
	} else {
		valIsNum = true;
	}

	if (strcmp(_param, "throttle") == 0) {
		if (valIsNum) {
			rc_channels[RC_CHANNEL_THROTTLE].set_pwm(val);
		}
	} else if (strcmp(_param, "roll") == 0) {
		if (valIsNum) {
			rc_channels[RC_CHANNEL_ROLL].set_pwm(val);
		}
	} else if (strcmp(_param, "pitch") == 0) {
		if (valIsNum) {
			rc_channels[RC_CHANNEL_PITCH].set_pwm(val);
		}
	} else if (strcmp(_param, "yaw") == 0) {
		if (valIsNum) {
			rc_channels[RC_CHANNEL_YAW].set_pwm(val);
		}
	} else {
#if DEBUG == ENABLED
		char t[200];
		snprintf(t, sizeof(t), "Unknown command entered: %s\n", _param);
		write(t);
#endif
		return;
	}

#if DEBUG == ENABLED
	char t[200];
	snprintf(t, sizeof(t), "Set %s to %s\n", _param, _value);
	write(t);
#endif


	// Only set the last packet time if we got a valid message (key:value:checksum) and checksums match
	_lastPktTime = hal.scheduler->millis();
}

bool Serial::checksumValid(char *str, char *chk) {

//#if DEBUG == ENABLED
//	char t[200];
//	snprintf(t, sizeof(t), "Getting checksum for \"%s\"\nLength: %d\n", str, strlen(str));
//	write(t);
//#endif

	long sum = calculateChecksum(str);

#if DEBUG == ENABLED
	if (sum != strtol(chk, NULL, 10)) {
		char t[200];
		snprintf(t, sizeof(t), "Invalid checksum. Expected %ld \t Got %ld\n", strtol(chk, NULL, 10), sum);
		write(t);
	}
#endif

	return (sum == strtol(chk, NULL, 10));
}

uint32_t Serial::lastReceiveTime() {
	return _lastPktTime;
}

long Serial::calculateChecksum(const char *str) {
	long sum = 0;
	for (unsigned int i = 0; i < strlen(str); i++) {
		sum += str[i];
	}

	return sum;
}
