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
#include "AP_Math.h"


#define ALTHOLD_HOLDING 					0
#define ALTHOLD_TAKEOFF 					1
#define ALTHOLD_LANDING						2
#define ALTHOLD_STOPPED 					3

//max and min height in cm
#define ALTHOLD_HEIGHT_MIN					10
#define ALTHOLD_HEIGHT_MAX					150
#define ALTHOLD_HEIGHT_TAKEOFF 				20
#define ALTHOLD_HEIGHT_LAND 				15

#define ALTHOLD_HEIGHT_THRESHOLD 			2

#define ALTHOLD_DISTANCE_P 					1
#define ALTHOLD_DISTANCE_I 					0
#define ALTHOLD_DISTANCE_D 					0
#define ALTHOLD_DISTANCE_IMAX 				0

#define ALTHOLD_VELOCITY_P 					0.75
#define ALTHOLD_VELOCITY_I 					1.5
#define ALTHOLD_VELOCITY_D 					0
#define ALTHOLD_VELOCITY_IMAX 				5

#define ALTHOLD_ACCEL_P 					6
#define ALTHOLD_ACCEL_I 					0
#define ALTHOLD_ACCEL_D 					0
#define ALTHOLD_ACCEL_IMAX 					0

// cm/s/s //really slow
#define ALTHOLD_ACCELERATION_MAX 			250
// cm/s max velocity
#define ALTHOLD_VELOCITY_MAX 				250
//The max change in distance we want from frame to frame
#define ALTHOLD_MAX_DISTANCE_LEASH_LENGTH 	100 //1m

#define ALTHOLD_ACCEL_UPDATE_INTERVAL 		500 //ms

#define ALTHOLD_DEBUG 						DISABLED

class AltHold{
public:
	AltHold();
	PID getDistancePid();
	PID getAccelerationPid();
	PID getVelocityPid();
	void holdAltitute();
private:
	PID * _distancePid;
	PID * _accelPid;
	PID * _velocityPid;
	int32_t _lastState;
	long _lastThrottle;
	float _acceleration; // cm/s
	int32_t _lastVelUpdateTime;
	int32_t _lastDistance;
	int32_t _hoverPoint;
	float _velocity;
	float _lastVelocityTarget;
	float _distanceLeashLength;
	long _mapThrottle(long x, long in_min, long in_max, long out_min, long out_max);
	void _updateCurrentVelocity();
	void _updateVelocity(uint32_t targetDistance);
	void _updateAcceleration();
	float sqrt_controller(float error, float p, float second_ord_lim);
	void _updateAcceleration(float targetVelocity);
	void _updateThrottleOutput(float targetAcceleration);
};

#endif

