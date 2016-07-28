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
#include "RadioController.h"


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

#define ALTHOLD_DISTANCE_P 					3 //1-3
#define ALTHOLD_DISTANCE_I 					0 //not used
#define ALTHOLD_DISTANCE_D 					0 //not used
#define ALTHOLD_DISTANCE_IMAX 				0 //not used

#define ALTHOLD_VELOCITY_P 					8 //1-8
#define ALTHOLD_VELOCITY_I 					0 //not used
#define ALTHOLD_VELOCITY_D 					0 //not used
#define ALTHOLD_VELOCITY_IMAX 				0 //not used

#define ALTHOLD_ACCEL_P 					1.5f //.5-1.5
#define ALTHOLD_ACCEL_I 					0 //0-3
#define ALTHOLD_ACCEL_D 					0 //0-0.4
#define ALTHOLD_ACCEL_IMAX 					0 //0-1000

// cm/s/s //really slow
#define ALTHOLD_ACCELERATION_MAX 			10000
// cm/s max velocity
#define ALTHOLD_VELOCITY_MAX 				10000
//The max change in distance we want from frame to frame
#define ALTHOLD_MAX_DISTANCE_LEASH_LENGTH 	10 //cm //1m

#define ALTHOLD_VELOCITY_UPDATE_INTERVAL 		0 //ms

#define ALTHOLD_DEBUG 						DISABLED

//vars for calculating the hover point
#define ALTHOLD_CALC_HOVER_DELAY			200 //ms
#define ALTHOLD_CALC_HOVER_MAX_HEIGHT		75 //cm
#define ALTHOLD_CALC_HOVER_HELD_TIME		3000 //ms
#define ALTHOLD_CALC_HOVER_THRESHOLD		3
#define ALTHOLD_CALC_HOVER_INCREMENT		10
#define ALTHOLD_CALC_HOVER_TIMEOUT			30000
#define ALTHOLD_CALC_HOVER_HEIGHT_TIMEOUT	200

class AltHold{
public:
	AltHold(RangeFinder *rf);
	PID getDistancePid();
	PID getAccelerationPid();
	PID getVelocityPid();
	void holdAltitute();
	void loadHoverPoint();
	float getZLeashLength();
	float get_velocity();
	static float getAltEstimate();
private:
	PID * _distancePid;
	PID * _accelPid;
	PID * _velocityPid;
	int32_t _lastState;
	long _lastThrottle;
	float _acceleration; // cm/s
	int32_t _lastVelUpdateTime;
	int32_t _lastDistance;

	RangeFinder *_rangefinder;

	float _velocity;
	float _lastVelocityTarget;
	float _distanceLeashLength;

	//hover calc vars
	int32_t _oldDistance;
	int32_t _hoverPoint;
	int32_t _lastHoverUp;
	int32_t _lastHoverDown;
	bool _hoverPointCalculated;
	bool _hasRisen;
	bool _gotLastDistance;
	uint32_t _heldTime;
	uint32_t _lastHoverCalcTime;
	int32_t _lastHoverDistance;
	uint32_t _calcHoverBeginTime;
	bool _slowStart;
	bool _slowStartThrottle;
	int32_t _slowStartOffset;
	bool _calcHoverToHigh;

	LowPassFilterFloat _vel_error_filter;
	LowPassFilterFloat _accel_error_filter;


	long _mapThrottle(long x, long in_min, long in_max, long out_min, long out_max);
	void _updateCurrentVelocity();
	void _updateVelocity(uint32_t targetDistance);
	void _updateAcceleration();
	float sqrt_controller(float error, float p, float second_ord_lim);
	void _updateAcceleration(float targetVelocity);
	void _updateThrottleOutput(float targetAcceleration);
	bool _caluclateHoverPoint();
};

#endif

