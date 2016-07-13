/*
 * Serial.h
 *
 * The telemetry port on the APM uses UART0 (aka Serial connection). This port uses the
 * hal.console object, which is the same as the standard USB serial connection used.
 * All code for reading and writing data with the USB serial port will be the same as the telemetry port
 * by default (unless the telemetry port was manually changed to be used with UART2)
 *
 *  Created on: Jun 23, 2016
 *      Author: chris
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <AP_HAL.h>

#include "Config.h"

class Serial {
public:
	void init();
	long *getRC(char *str);
	void write(const char *str, int16_t len);

private:
	bool checksum(const char *str, int16_t chksum);
	void process(char *cmd);

	long rc_channels[4];

};


#endif /* SERIAL_H_ */
