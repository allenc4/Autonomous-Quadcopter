/*
 * Tests.h
 *
 *  Created on: May 26, 2016
 *      Author: chris
 */

#ifndef TESTS_H_
#define TESTS_H_

#include <AP_Progmem_AVR.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

// Local includes
#include "Config.h"

class Tests {

public:
	void accel_Gyro_Test();
	void motor_Test();
	void individual_Motor_Test(int8_t motor_num);

};


#endif /* TESTS_H_ */
