/*
 * RangeFinder.h
 *
 *  Created on: Jul 5, 2016
 *      Author: chris
 */

#ifndef RANGEFINDER_H_
#define RANGEFINDER_H_

#include "Config.h"

#define RANGEFINDER_READ_TIMEOUT_MS		1000	// Time (in milliseconds) to wait for new LIDAR data before
											// triggering a watchdog timeout
#define RANGEFINDER_READ_TIMOUT_ATTEMPTS	10		// Only allow 10 attempts of consecutive read fails before throwing error

class RangeFinder {
public:
	RangeFinder();
	virtual bool init();
	bool isHealthy();
	virtual bool update();
	virtual bool update(uint16_t &distance);
	uint16_t getLastDistance();
private:
protected:
	uint16_t distCM;
	uint32_t timeLastUpdate;
	uint8_t  numReadFails;
};


#endif /* RANGEFINDER_H_ */
