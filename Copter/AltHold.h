/*
 * AltHold.h
 *
 *  Created on: Jul 13, 2016
 *      Author: tyler
 */

#ifndef _ALTHOLD_
#define _ALTHOLD_

#include "RangeFinder.h"
#include "PID.h"


#define ALTHOLD_HOLDING 0
#define ALTHOLD_TAKEOFF 1
#define ALTHOLD_LANDING	2
#define ALTHOLD_STOPPED 3

//max and min height in cm
#define ALTHOLD_HEIGHT_MIN	10
#define ALTHOLD_HEIGHT_MAX	150
#define ALTHOLD_HEIGHT_TAKEOFF 20
#define ALTHOLD_HEIGHT_LAND 15

#define ALTHOLD_HEIGHT_THRESHOLD 2

#define ALTHOLD_ACCELERATION_MAX 2 // cm/s //really slow

class AltHold{
public:
	AltHold();
	int32_t getP();
	int32_t getD();
	int32_t getI();
	void setP(int32_t p);
	void setD(int32_t d);
	void setI(int32_t i);
	void holdAltitute();
private:
	int32_t _p;
	int32_t _i;
	int32_t _d;
	PID pid;
	int32_t _lastState;
	long _lastThrottle;
	int32_t _hoverValue;
	int32_t _acceleration; // cm/s
	int32_t _lastAccelUpdateTime;
	int32_t _lastDistance;
	long _mapThrottle(long x, long in_min, long in_max, long out_min, long out_max);
	void _updateAcceleration();
};

#endif

