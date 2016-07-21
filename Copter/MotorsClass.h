/*
 * Motor.h
 *
 *  Created on: May 27, 2016
 *      Author: chris
 */

#ifndef MOTORSCLASS_H_
#define MOTORSCLASS_H_

#include <AP_Math.h>
#include <RC_Channel.h>         // RC Channel Library
#include <AP_Motors.h>			// Motors Library

#include "Config.h"
#include "OpticalFlow.h"
#include "RadioController.h"
#if LIDAR == ENABLED
#include "AltHold.h"
#endif

class MotorsClass {

public:
	MotorsClass(RadioController radio);
	AP_MotorsQuad *getMotors();
	void init();
	void output(uint16_t motor_num, uint16_t pwm);
	void calibrate_ESCs();

private:
	RadioController *_radio;
	AP_MotorsQuad *_motors;
	const int8_t min_throttle_offset = 75;
#if LIDAR == ENABLED
	AltHold altHold;
#endif
};



#endif /* MOTORSCLASS_H_ */
