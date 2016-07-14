/*
 * AltHold.ino
 *
 *  Created on: Jul 13, 2016
 *      Author: tyler
 */

#include "AltHold.h"
#include "AC_PID.h"

AltHold::AltHold(){
	this->_lastState = ALTHOLD_STOPPED;
	this->_lastThrottle = 0;
	this->_lastVelUpdateTime = hal.scheduler->millis();
	//start by estimating the hover point at half throttle
	this->_hoverPoint = (RC_THROTTLE_MAX + RC_THROTTLE_MIN) / 2;

	this->_distancePid = new PID(
			ALTHOLD_DISTANCE_P,
			ALTHOLD_DISTANCE_I,
			ALTHOLD_DISTANCE_D,
			ALTHOLD_DISTANCE_IMAX
	);
	this->_accelPid = new PID(
			ALTHOLD_ACCEL_P,
			ALTHOLD_ACCEL_I,
			ALTHOLD_ACCEL_D,
			ALTHOLD_ACCEL_IMAX
	);
	this->_velocityPid = new PID(
			ALTHOLD_VELOCITY_P,
			ALTHOLD_VELOCITY_I,
			ALTHOLD_VELOCITY_D,
			ALTHOLD_VELOCITY_IMAX
	);

	float kP = ALTHOLD_DISTANCE_P;
	if(kP <= 0)
	{
		kP = ALTHOLD_MAX_DISTANCE_LEASH_LENGTH;
	}

	if(ALTHOLD_VELOCITY_MAX <= ALTHOLD_ACCELERATION_MAX / kP)
	{
		this->_distanceLeashLength = ALTHOLD_VELOCITY_MAX / kP;
	}else{
		this->_distanceLeashLength = (ALTHOLD_ACCELERATION_MAX / (2.0f * kP * kP)) +
				(ALTHOLD_VELOCITY_MAX * ALTHOLD_VELOCITY_MAX / (2.0f * ALTHOLD_ACCELERATION_MAX));
	}

	if(this->_distanceLeashLength < ALTHOLD_MAX_DISTANCE_LEASH_LENGTH)
	{
		this->_distanceLeashLength = ALTHOLD_MAX_DISTANCE_LEASH_LENGTH;
	}
}

PID AltHold::getDistancePid(){
	return *(this->_distancePid);
}
PID AltHold::getAccelerationPid(){
	return *(this->_accelPid);
}
PID AltHold::getVelocityPid(){
	return *(this->_velocityPid);
}

/**
 *  Scales the x parameter and scales it so it is between a new min and max rangethat it represent something meaningful.
 *  Takes a number between one range and places it in another – e.g., if we had a value of 50,
 *      which was between 0-100, and we wanted to scale it to be between 0 and 500,
 *      the map function would return 250.
 */
long AltHold::_mapThrottle(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void AltHold::holdAltitute(){
	long currentThrottle = rc_channels[RC_CHANNEL_THROTTLE];
//	long currentThrottle = this->_hoverPoint;

	//if we have no throttle input do nothing
	if(currentThrottle <= RC_THROTTLE_MIN+25) //this is the current min_throttle_offset variable should be changed to a var not harcoded
	{
//		hal.console->println("Need throttle to move");
		this->_lastState = ALTHOLD_STOPPED;
		return;
	}

	//lets update our acceleration
	this->_updateCurrentVelocity();

	//map the throttle to the available height range
	currentThrottle = this->_mapThrottle(currentThrottle, RC_THROTTLE_MIN, RC_THROTTLE_MAX, ALTHOLD_HEIGHT_MIN, ALTHOLD_HEIGHT_MAX);
	int32_t currentState;
	uint16_t currentDistance = lidar.getLastDistance();

	//if we were taking off and haven't gotten to the proper height yet keep taking off
	if(this->_lastState == ALTHOLD_TAKEOFF && currentDistance < ALTHOLD_HEIGHT_TAKEOFF)
	{
//		hal.console->println("Taking Off");
		currentState = ALTHOLD_TAKEOFF;
	}
	//if we were stopped and there is throttle input take off
	else if(this->_lastState == ALTHOLD_STOPPED && currentThrottle >= ALTHOLD_HEIGHT_MIN)
	{
//		hal.console->println("Starting take off");
		currentState = ALTHOLD_TAKEOFF;
	}
	//if we were holding and throttle input goes below the landing height land
	else if(this->_lastState == ALTHOLD_HOLDING && currentDistance <= ALTHOLD_HEIGHT_LAND + ALTHOLD_HEIGHT_THRESHOLD)
	{
//		hal.console->println("Starting land");
		currentState = ALTHOLD_LANDING;
	}
	else if(this->_lastState == ALTHOLD_LANDING && currentDistance > ALTHOLD_HEIGHT_MIN - ALTHOLD_HEIGHT_THRESHOLD)
	{
//		hal.console->println("Landing");
		currentState = ALTHOLD_LANDING;
	}
	//if we are taking off and we reach the take off height then we hold
	else if((this->_lastState == ALTHOLD_TAKEOFF || this->_lastState == ALTHOLD_HOLDING) && currentDistance >= ALTHOLD_HEIGHT_TAKEOFF - ALTHOLD_HEIGHT_THRESHOLD )
	{
//		hal.console->println("Holding");
		currentState = ALTHOLD_HOLDING;
	}
	else{
//		hal.console->println("Stopping");
		currentState = ALTHOLD_STOPPED;
	}

	long desiredDistance;
	float desiredAcceleration;

	switch(currentState){
		case ALTHOLD_TAKEOFF:
			//here we want to get up to desired height and then store the hover value
			desiredDistance = ALTHOLD_HEIGHT_TAKEOFF;
			break;
		case ALTHOLD_HOLDING:
			desiredDistance = currentThrottle;
			break;
		case ALTHOLD_LANDING:
			desiredDistance = ALTHOLD_HEIGHT_MIN;
			break;
		default:
			hal.console->println("Could not figure out state landing");
			desiredDistance = ALTHOLD_HEIGHT_LAND;
			break;
	}

	this->_updateVelocity(desiredDistance);


	_lastState = currentState;
	_lastThrottle = rc_channels[RC_CHANNEL_THROTTLE];

}

void AltHold::_updateCurrentVelocity(){

	uint32_t currentTime = hal.scheduler->millis();
	if(currentTime - this->_lastVelUpdateTime < ALTHOLD_ACCEL_UPDATE_INTERVAL)
	{
		return;
	}

	uint16_t currentDistance = lidar.getLastDistance();
	float timeDiff = (float)currentTime - (float)this->_lastVelUpdateTime;
	timeDiff /= 1000; //convert to seconds
	float distanceDiff = (float)currentDistance - (float)this->_lastDistance;

	this->_velocity = distanceDiff / timeDiff;

	this->_lastDistance = currentDistance;
	this->_lastVelUpdateTime = currentTime;
}

void AltHold::_updateVelocity(uint32_t targetDistance){

	hal.console->print(" Target Distance: ");
	hal.console->print(targetDistance);

	uint32_t currentDistance = lidar.getLastDistance();

	hal.console->print(" Current Distance: ");
	hal.console->print(currentDistance);

	float distanceError = (float)targetDistance - (float)currentDistance;

	hal.console->print(" Distance Error: ");
	hal.console->print(distanceError);

	//this makes it so that we aren't trying to go super far in one frame
	if(distanceError > this->_distanceLeashLength)
	{
		targetDistance = currentDistance + this->_distanceLeashLength;
		distanceError = this->_distanceLeashLength;
	}
	if(distanceError < -this->_distanceLeashLength)
	{
		targetDistance = currentDistance - this->_distanceLeashLength;
		distanceError = -this->_distanceLeashLength;
	}

	float targetVelocity = this->sqrt_controller(distanceError, this->_distancePid->kP(), ALTHOLD_ACCELERATION_MAX);

	if(targetVelocity > ALTHOLD_VELOCITY_MAX)
	{
		targetVelocity = ALTHOLD_VELOCITY_MAX;
	}
	if(targetVelocity < -ALTHOLD_VELOCITY_MAX)
	{
		targetVelocity = -ALTHOLD_VELOCITY_MAX;
	}

	this->_updateAcceleration(targetVelocity);

}

void AltHold::_updateAcceleration(float targetVelocity)
{

	hal.console->print(" Target Velocity: ");
	hal.console->print(targetVelocity);

	//removed the feedforward peice until i understand it
	//believe it is just a performance thing
	float accelFeedforward = 0;
	this->_lastVelocityTarget = targetVelocity;

	float velocityError = targetVelocity - this->_velocity;

	float p = this->_velocityPid->kP() * velocityError;

	float targetAcceleration = accelFeedforward + p;

	//contrain target acceleration
	if(targetAcceleration > ALTHOLD_ACCELERATION_MAX)
	{
		targetAcceleration = ALTHOLD_ACCELERATION_MAX;
	}
	if(targetAcceleration < -ALTHOLD_ACCELERATION_MAX)
	{
		targetAcceleration = -ALTHOLD_ACCELERATION_MAX;
	}

	this->_updateThrottleOutput(targetAcceleration);
}

void AltHold::_updateThrottleOutput(float targetAcceleration){

	hal.console->print(" Target Accel: ");
	hal.console->print(targetAcceleration);

	float acceleration = -(ahrs.get_accel_ef().z + GRAVITY_MSS) * 100.0f;

	float accelerationError = targetAcceleration - acceleration;

	float accelPidValue = this->_accelPid->get_pid(accelerationError);

	hal.console->print(" Accel PID Value: ");
	hal.console->print(accelPidValue);
	hal.console->print(" /1000: ");
	hal.console->print(accelPidValue/1000.0f);

	float throttleOut = (accelPidValue/1000.0f) + this->_hoverPoint;

	rc_channels[RC_CHANNEL_THROTTLE] = throttleOut;

	hal.console->print(" Throttle Output: ");
	hal.console->print(throttleOut);
	hal.console->println("");
}

// Proportional controller with piecewise sqrt sections to constrain second derivative
//taken from AC_AttitudeControl arudpilot(master)
float AltHold::sqrt_controller(float error, float p, float second_ord_lim)
{
    if (second_ord_lim <= 0.0f || p == 0.0f) {
        return error*p;
    }

    float linear_dist = second_ord_lim/sq(p);

    if (error > linear_dist) {
        return safe_sqrt(2.0f*second_ord_lim*(error-(linear_dist/2.0f)));
    } else if (error < -linear_dist) {
        return -safe_sqrt(2.0f*second_ord_lim*(-error-(linear_dist/2.0f)));
    } else {
        return error*p;
    }
}


