/*
 * OpticalFlow.h
 *
 *  Created on: Jul 6, 2016
 *      Author: chris
 */

#ifndef OPTICALFLOW_H_
#define OPTICALFLOW_H_

#include <AP_OpticalFlow.h>
#include <AC_PID.h>
#include "Config.h"
#include "RangeFinder.h"

static AP_OpticalFlow_ADNS3080 _optflow;
AC_PID pids_optflow[2]; // Only need pitch (0) and roll (1) PIDs


class OpticalFlow {
public:
	OpticalFlow(RangeFinder *rf);
	bool init();
	void update();

	// calculate modified roll/pitch depending upon optical flow calculated position
	int32_t get_of_roll(int32_t input_roll, int32_t input_yaw);
	int32_t get_of_pitch(int32_t input_pitch, int32_t input_yaw);
private:
	RangeFinder *rangeFinder;
};

#endif /* OPTICALFLOW_H_ */
