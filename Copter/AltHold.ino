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
	this->_p = 0;
	this->_i = 0;
	this->_d = 0;
	this->_lastAccelUpdateTime = hal.scheduler->millis();
}

int32_t AltHold::getP(){
	return this->pid.kP();
}
int32_t AltHold::getD(){
	return this->pid.kD();
}
int32_t AltHold::getI(){
	return this->pid.kI();
}
void AltHold::setP(int32_t p){
	this->pid.kP(p);
}
void AltHold::setD(int32_t d){
	this->pid.kD(d);
}
void AltHold::setI(int32_t i){
	this->pid.kI(i);
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

	//lets update our acceleration
	this->_updateAcceleration();

	//map the throttle to the available height range
	long currentThrottle = rc_channels[RC_CHANNEL_THROTTLE];
	currentThrottle = this->_mapThrottle(currentThrottle, RC_THROTTLE_MIN, RC_THROTTLE_MAX, ALTHOLD_HEIGHT_MIN, ALTHOLD_HEIGHT_MAX);

	int32_t currentState;
	uint16_t currentDistance = lidar.getLastDistance();

	//if we have no throttle input do nothing
	if(currentThrottle < RC_THROTTLE_MIN - 75) //this is the current min_throttle_offset variable should be changed to a var not harcoded
	{
		 return;
	}

	//if we were taking off and haven't gotten to the proper height yet keep taking off
	if(this->_lastState == ALTHOLD_TAKEOFF && currentDistance < ALTHOLD_HEIGHT_TAKEOFF)
	{
		currentState = ALTHOLD_TAKEOFF;
	}
	//if we were stopped and there is throttle input take off
	else if(this->_lastState == ALTHOLD_STOPPED && currentThrottle >= ALTHOLD_HEIGHT_MIN)
	{
		currentState = ALTHOLD_TAKEOFF;
	}
	//if we were holding and throttle input goes below the landing height land
	else if(this->_lastState == ALTHOLD_HOLDING && currentThrottle <= ALTHOLD_HEIGHT_LAND + ALTHOLD_HEIGHT_THRESHOLD)
	{
		currentState = ALTHOLD_LANDING;
	}
	//if we are taking off and we reach the take off height then we hold
	else if(this->_lastState == ALTHOLD_TAKEOFF && currentDistance >= ALTHOLD_HEIGHT_TAKEOFF - ALTHOLD_HEIGHT_THRESHOLD )
	{
		currentState = ALTHOLD_HOLDING;
	}else{
		currentState = ALTHOLD_STOPPED;
	}

	long desiredDistance;

	switch(currentState){
		case ALTHOLD_TAKEOFF:
			//here we want to get up to desired height and then store the hover value
			desiredDistance = ALTHOLD_HEIGHT_TAKEOFF;
			break;
		case ALTHOLD_HOLDING:
			desiredDistance = currentThrottle;
			break;
		case ALTHOLD_LANDING:
			desiredDistance = ALTHOLD_HEIGHT_LAND;
			break;
		default:
			hal.console->println("Could not figure out state landing");
			desiredDistance = ALTHOLD_HEIGHT_LAND;
			break;
	}

	float error = desiredDistance - currentDistance;
	long pidValue = pid.get_pid(error);

	hal.console->printf("Calculated Altitude PID Value: %ld", pidValue);

	_lastState = currentState;
	_lastThrottle = rc_channels[RC_CHANNEL_THROTTLE];

}

//this should be called in the scheduler
void AltHold::_updateAcceleration(){

	uint16_t currentDistance = lidar.getLastDistance();

	uint32_t currentTime = hal.scheduler->millis();
	uint32_t timeDiff = currentTime - this->_lastAccelUpdateTime;
	timeDiff /= 1000; //convert to seconds
	uint32_t distanceDiff = currentDistance - this->_lastDistance;

	this->_acceleration = distanceDiff / timeDiff;

	this->_lastDistance = currentDistance;
	this->_lastAccelUpdateTime = currentTime;

}
