/*
 * RangeFinder_Lidar.h
 *
 *  Created on: Jul 13, 2016
 *      Author: chris
 */

#ifndef RANGEFINDER_LIDAR_H_
#define RANGEFINDER_LIDAR_H_

#include "RangeFinder.h"

#define LIDAR_ADDRESS				0x62	// I2C address of the LIDAR
#define REGISTER_MEASURE 			0x00 	// Register to write to initiate ranging
#define MEASURE_VALUE				0x04	// Value to initiate ranging
#define REGISTER_HIGH_LOW_BYTES		0x8f	// Register to get both High and Low bytes
#define RANGEFINDER_READ_TIMEOUT_MS		1000	// Time (in milliseconds) to wait for new LIDAR data before
											// triggering a watchdog timeout
#define RANGEFINDER_READ_TIMOUT_ATTEMPTS	10		// Only allow 10 attempts of consecutive read fails before throwing error


class RangeFinder_Lidar: public RangeFinder {
public:
	bool init();
	bool update();
	bool update(uint16_t &distance);
	uint16_t getStartingDistanceOffset();

private:
	uint16_t _starting_distance_offset;
};


#endif /* RANGEFINDER_LIDAR_H_ */
