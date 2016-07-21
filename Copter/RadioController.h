/*
 * Radio.h
 *
 *  Created on: Jul 20, 2016
 *      Author: chris
 */

#ifndef RADIOCONTROLLER_H_
#define RADIOCONTROLLER_H_

#include <RC_Channel.h>         // RC Channel Library

#include "Config.h"


class RadioController {
public:
	RadioController(int8_t chRoll, int8_t chPitch, int8_t chThrottle, int8_t chYaw);
	~RadioController()
	{
		for (int i = 0; i < 4; i++) {
			delete _rc_channels[i];
		}
	}

	void read();
	void init();

	// Get the channels of the RC receiver values
	RC_Channel **getRCChannels();
	RC_Channel *getRCRoll();
	RC_Channel *getRCPitch();
	RC_Channel *getRCThrottle();
	RC_Channel *getRCYaw();

private:
	RC_Channel *_rc_channels[4];

	// Hold channel values for roll, pitch, throttle, and yaw
	int8_t _chRoll, _chPitch, _chThrottle, _chYaw;

	// In the rc_channels array, hold the position of which RC_Channels object maps to which index
	int8_t _rc_arr_roll, _rc_arr_pitch, _rc_arr_throttle, _rc_arr_yaw;
};


#endif /* RADIOCONTROLLER_H_ */
