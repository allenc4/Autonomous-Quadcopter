/*
 * Motor.h
 *
 *  Created on: May 27, 2016
 *      Author: chris
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "Config.h"

class Motors {

public:
	void output(uint16_t motor_num, uint16_t pwm);
	void output();
	void output_Min();
	void output_Zero();


//	void calibrate_ESC();
//	static void Calibrate_ESCs();

private:
	const int8_t min_throttle_offset = 50;
};



#endif /* MOTOR_H_ */
