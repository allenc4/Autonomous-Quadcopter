/*
 * RangeFinder.h
 *
 *  Created on: Jul 5, 2016
 *      Author: chris
 */

#ifndef RANGEFINDER_H_
#define RANGEFINDER_H_

#include "Config.h"

class RangeFinder {
public:
	RangeFinder();
	bool init();
	bool isHealthy();
	bool getDistance(uint16_t &distance);
private:
	uint16_t distCM;
	uint32_t timeLastUpdate;
	uint8_t  numReadFails;
};


#endif /* RANGEFINDER_H_ */
