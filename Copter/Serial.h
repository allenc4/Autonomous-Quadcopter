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
#include <UARTDriver.h>
#include "limits.h"
#include "errno.h"
#include "Config.h"
#include "stdio.h"

class Serial {
public:
	Serial(AP_HAL::UARTDriver *driver);
	void init(uint32_t baudRate);
	void read();
	void write(const char *str, bool appendChecksum=true);
	uint32_t lastReceiveTime();
	long calculateChecksum(const char *str);

private:
	void process(char *cmd);
	bool checksumValid(char *str, char *chk);

	AP_HAL::UARTDriver *_driver;

	uint32_t _lastPktTime;

	char _readBuf[512];
	int16_t _readBufLen;
	char _cmdBuf[512];
	char _param[255];
	char _value[255];
//	char _chksum[255];

};


#endif /* SERIAL_H_ */
