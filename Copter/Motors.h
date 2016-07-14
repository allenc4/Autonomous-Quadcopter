/*
 * Motor.h
 *
 *  Created on: May 27, 2016
 *      Author: chris
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <AP_Math.h>

#include "Config.h"
#include "OpticalFlow.h"
#if LIDAR == ENABLED
#include "AltHold.h"
#endif

class Motors {

public:
	void output(uint16_t motor_num, uint16_t pwm);
	void output();
	void output_Min();
	void output_Zero();
	void calibrate_ESCs();
	void init_yaw();
	void output_Throttle();


//	void calibrate_ESC();
//	static void Calibrate_ESCs();

private:
	float target_yaw;
	const int8_t min_throttle_offset = 75;
#if LIDAR == ENABLED
	AltHold altHold;
#endif
	float wrap_180(float x);
};



#endif /* MOTOR_H_ */
