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
	virtual bool init();
	bool isHealthy();
	virtual bool update();
	virtual bool update(uint16_t &distance);
	uint16_t getLastDistance();
private:
	uint16_t distCM;
	uint32_t timeLastUpdate;
	uint8_t  numReadFails;
};


#endif /* RANGEFINDER_H_ */
