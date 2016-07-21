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

#define OPTFLOW_ORIENTATION 	AP_OPTICALFLOW_ADNS3080_PINS_FORWARD
#define OPTFLOW_RESOLUTION		ADNS3080_RESOLUTION_1600
#define OPTFLOW_FOV				AP_OPTICALFLOW_ADNS3080_08_FOV

static AP_OpticalFlow_ADNS3080 _optflow;
AC_PID pids_optflow[2]; // Only need pitch (0) and roll (1) PIDs


class OpticalFlow {
public:
	OpticalFlow(RangeFinder *rf);
	bool init();
	void update();

	void reset_I();

	// calculate modified roll/pitch depending upon optical flow calculated position
	int32_t get_of_roll(int32_t input_roll, int32_t input_yaw);
	int32_t get_of_pitch(int32_t input_pitch, int32_t input_yaw);

	float get_change_x();
	float get_change_y();
	void debug_print();

	static void read(uint32_t now);
private:
	RangeFinder *rangeFinder;
	uint32_t last_of_update;
	uint32_t last_of_roll_update;
	uint32_t last_of_pitch_update;

	int32_t _of_roll, _of_pitch;
};

#endif /* OPTICALFLOW_H_ */
