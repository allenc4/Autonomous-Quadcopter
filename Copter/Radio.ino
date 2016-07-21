#include "Radio.h"

Radio::Radio(int8_t chRoll, int8_t chPitch, int8_t chThrottle, int8_t chYaw) {
	_chRoll 	= chRoll;
	_chPitch 	= chPitch;
	_chThrottle = chThrottle;
	_chYaw	 	= chYaw;

	// Hold the position of each channel within the array
	_rc_arr_roll		= 0;
	_rc_arr_pitch		= 1;
	_rc_arr_throttle	= 2;
	_rc_arr_yaw			= 3;

	_rc_channels[_rc_arr_roll] 		= new RC_Channel(_chRoll);
	_rc_channels[_rc_arr_pitch] 	= new RC_Channel(_chPitch);
	_rc_channels[_rc_arr_throttle]	= new RC_Channel(_chThrottle);
	_rc_channels[_rc_arr_yaw]		= new RC_Channel(_chYaw);

}

/**
 * Setup each of the RC_Channel objects (roll, pitch, throttle, and yaw)
 * to use the radio min/max values
 */
void Radio::init() {
	_rc_channels[_chRoll]->radio_min = RC_ROLL_MIN;
	_rc_channels[_chRoll]->radio_max = RC_ROLL_MAX;
	_rc_channels[_chRoll]->set_angle(4500);
	_rc_channels[_chRoll]->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

	_rc_channels[_chPitch]->radio_min = RC_PITCH_MIN;
	_rc_channels[_chPitch]->radio_max = RC_PITCH_MAX;
	_rc_channels[_chPitch]->set_angle(4500);
	_rc_channels[_chPitch]->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

	_rc_channels[_chYaw]->radio_min = RC_YAW_MIN;
	_rc_channels[_chYaw]->radio_max = RC_YAW_MAX;
	_rc_channels[_chYaw]->set_angle(4500);
	_rc_channels[_chYaw]->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

	_rc_channels[_chThrottle]->radio_min = RC_THROTTLE_MIN;
	_rc_channels[_chThrottle]->radio_max = RC_THROTTLE_MAX;
	_rc_channels[_chThrottle]->set_range(0, 1000);
	_rc_channels[_chThrottle]->set_range_out(0, 1000);

	// Set trim values to be in the middle for roll, pitch, yaw and min for throttle
	for (int i = 0; i < 30; i++) {
		// Read RC values
		read();
		hal.scheduler->delay(20);
	}

	for (int i = 0; i < 4; i++) {
		_rc_channels[i]->trim();
	}
}

// Read RC values for throttle, roll, pitch, yaw
void Radio::read() {
	_rc_channels[_rc_arr_roll]->set_pwm(hal.rcin->read(_chRoll));
	_rc_channels[_rc_arr_pitch]->set_pwm(hal.rcin->read(_chPitch));
	_rc_channels[_rc_arr_throttle]->set_pwm(hal.rcin->read(_chThrottle));
	_rc_channels[_rc_arr_yaw]->set_pwm(hal.rcin->read(_chYaw));

    // DEBUGGING PURPOSES ONLY ///////////////////////////////
//	_rc_channels[_rc_arr_roll]->set_pwm(RC_THROTTLE_MIN + 400);
//	_rc_channels[_rc_arr_pitch]->set_pwm((RC_ROLL_MIN + RC_ROLL_MAX)/2);
//	_rc_channels[_rc_arr_throttle]->set_pwm((RC_PITCH_MIN + RC_PITCH_MAX)/2);
//	_rc_channels[_rc_arr_yaw]->set_pwm((RC_YAW_MIN + RC_YAW_MAX)/2);

    // Print out the values read in from the RC channels
#if DEBUG == ENABLED
    if (loop_count % 20 == 0) {
			hal.console->printf("Throttle: %d\t Roll: %d\t Pitch: %d\t Yaw: %d\n",
				_rc_channels[_rc_arr_roll]->control_in,
				_rc_channels[_rc_arr_pitch]->control_in,
				_rc_channels[_rc_arr_throttle]->control_in,
				_rc_channels[_rc_arr_yaw]->control_in);
		}
	}
#endif
}

RC_Channel **Radio::getRCChannels() {
	return _rc_channels;
}

RC_Channel *Radio::getRCRoll() {
	return _rc_channels[_rc_arr_roll];
}

RC_Channel *Radio::getRCPitch() {
	return _rc_channels[_rc_arr_pitch];
}

RC_Channel *Radio::getRCThrottle() {
	return _rc_channels[_rc_arr_yaw];
}

RC_Channel *Radio::getRCYaw() {
	return _rc_channels[_rc_arr_yaw];
}
