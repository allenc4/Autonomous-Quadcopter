///*
// * AltHold.ino
// *
// *  Created on: Jul 13, 2016
// *      Author: tyler
// */
//
//#include "AltHold.h"
//#include "AC_PID.h"
//#if LIDAR == ENABLED
//AltHold::AltHold(RangeFinder *rf){
//	this->_rangefinder = rf;
//	this->_oldDistance = 0;
//	this->_lastState = ALTHOLD_STOPPED;
//	this->_lastThrottle = 0;
//	this->_lastVelUpdateTime = hal.scheduler->millis();
//	//start by estimating the hover point at half throttle
//	this->_hoverPoint = 400;
//	this->_lastHoverDown = this->_hoverPoint - ALTHOLD_CALC_HOVER_INCREMENT;
//	this->_lastHoverUp = this->_hoverPoint + ALTHOLD_CALC_HOVER_INCREMENT;
//	this->_gotLastDistance = false;
//
//	// Check if the hover point was previously calculated and saved to EEPROM
//	int32_t tHoverPoint = 0; //g.hover_point.get();
//	if (tHoverPoint == 0) {
//		// Default value of 0 saved to EEPROM, so we need to calculate the hover point
//		this->_hoverPointCalculated = false;
//	} else {
//		this->_hoverPointCalculated = true;
//		this->_hoverPoint = tHoverPoint;
//	}
//
//	this->_slowStartThrottle = this->_hoverPoint/2;
//	this->_slowStart = false;
//	this->_slowStartOffset = 0;
//
//	this->_distancePid = new PID(
//			ALTHOLD_DISTANCE_P,
//			ALTHOLD_DISTANCE_I,
//			ALTHOLD_DISTANCE_D,
//			ALTHOLD_DISTANCE_IMAX
//	);
//	this->_accelPid = new PID(
//			ALTHOLD_ACCEL_P,
//			ALTHOLD_ACCEL_I,
//			ALTHOLD_ACCEL_D,
//			ALTHOLD_ACCEL_IMAX
//	);
//	this->_velocityPid = new PID(
//			ALTHOLD_VELOCITY_P,
//			ALTHOLD_VELOCITY_I,
//			ALTHOLD_VELOCITY_D,
//			ALTHOLD_VELOCITY_IMAX
//	);
//
//	this->_calcHoverBeginTime = 0;
//
//	float kP = ALTHOLD_DISTANCE_P;
//	if(kP <= 0)
//	{
//		kP = ALTHOLD_MAX_DISTANCE_LEASH_LENGTH;
//	}
//
//	if(ALTHOLD_VELOCITY_MAX <= ALTHOLD_ACCELERATION_MAX / kP)
//	{
//		this->_distanceLeashLength = ALTHOLD_VELOCITY_MAX / kP;
//	}else{
//		this->_distanceLeashLength = (ALTHOLD_ACCELERATION_MAX / (2.0f * kP * kP)) +
//				(ALTHOLD_VELOCITY_MAX * ALTHOLD_VELOCITY_MAX / (2.0f * ALTHOLD_ACCELERATION_MAX));
//	}
//
//	if(this->_distanceLeashLength < ALTHOLD_MAX_DISTANCE_LEASH_LENGTH)
//	{
//		this->_distanceLeashLength = ALTHOLD_MAX_DISTANCE_LEASH_LENGTH;
//	}
//}
//
//PID AltHold::getDistancePid(){
//	return *(this->_distancePid);
//}
//PID AltHold::getAccelerationPid(){
//	return *(this->_accelPid);
//}
//PID AltHold::getVelocityPid(){
//	return *(this->_velocityPid);
//}
//
///**
// *  Scales the x parameter and scales it so it is between a new min and max rangethat it represent something meaningful.
// *  Takes a number between one range and places it in another – e.g., if we had a value of 50,
// *      which was between 0-100, and we wanted to scale it to be between 0 and 500,
// *      the map function would return 250.
// */
//long AltHold::_mapThrottle(long x, long in_min, long in_max, long out_min, long out_max)
//{
//  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//}
//
//void AltHold::holdAltitute(){
//	long currentThrottle = rc_channels[RC_CHANNEL_THROTTLE].radio_in;
////	long currentThrottle = this->_hoverPoint;
//
//	//if we have no throttle input do nothing
//	if(currentThrottle <= RC_THROTTLE_MIN+75) //this is the current min_throttle_offset variable should be changed to a var not harcoded
//	{
//#if DEBUG == ENABLED
//		hal.console->println("Need throttle to move");
//#endif
//		this->_lastState = ALTHOLD_STOPPED;
//		return;
//	}
//
//	if(!this->_caluclateHoverPoint())
//	{
//		return;
//	}
//
//	//lets update our acceleration
//	this->_updateCurrentVelocity();
//
//	//map the throttle to the available height range
//	currentThrottle = this->_mapThrottle(currentThrottle, RC_THROTTLE_MIN, RC_THROTTLE_MAX, ALTHOLD_HEIGHT_MIN, ALTHOLD_HEIGHT_MAX);
//	int32_t currentState;
//	uint16_t currentDistance = _rangefinder->getLastDistance();
//
//	//if we were taking off and haven't gotten to the proper height yet keep taking off
//	if(this->_lastState == ALTHOLD_TAKEOFF && currentDistance < ALTHOLD_HEIGHT_TAKEOFF)
//	{
//#if DEBUG == ENABLED
//		hal.console->println("Taking Off");
//#endif
//		currentState = ALTHOLD_TAKEOFF;
//	}
//	//if we were stopped and there is throttle input take off
//	else if(this->_lastState == ALTHOLD_STOPPED && currentThrottle >= ALTHOLD_HEIGHT_MIN)
//	{
//#if DEBUG == ENABLED
//		hal.console->println("Starting take off");
//#endif
//		currentState = ALTHOLD_TAKEOFF;
//	}
//	//if we were holding and throttle input goes below the landing height land
//	else if(this->_lastState == ALTHOLD_HOLDING && currentDistance <= ALTHOLD_HEIGHT_LAND + ALTHOLD_HEIGHT_THRESHOLD)
//	{
//#if DEBUG == ENABLED
//		hal.console->print("Starting land ");
//#endif
//		currentState = ALTHOLD_LANDING;
//	}
//	else if(this->_lastState == ALTHOLD_LANDING && currentDistance > ALTHOLD_HEIGHT_MIN - ALTHOLD_HEIGHT_THRESHOLD)
//	{
//#if DEBUG == ENABLED
//		hal.console->print("Landing ");
//#endif
//		currentState = ALTHOLD_LANDING;
//	}
//	//if we are taking off and we reach the take off height then we hold
//	else if((this->_lastState == ALTHOLD_TAKEOFF || this->_lastState == ALTHOLD_HOLDING) && currentDistance >= ALTHOLD_HEIGHT_TAKEOFF - ALTHOLD_HEIGHT_THRESHOLD )
//	{
//#if DEBUG == ENABLED
//		hal.console->print("Holding ");
//#endif
//		currentState = ALTHOLD_HOLDING;
//	}
//	else{
//#if DEBUG == ENABLED
//		hal.console->print("Stopping ");
//#endif
//		currentState = ALTHOLD_STOPPED;
//	}
//
//	long desiredDistance;
//
//	switch(currentState){
//		case ALTHOLD_TAKEOFF:
//			//here we want to get up to desired height and then store the hover value
////			desiredDistance = ALTHOLD_HEIGHT_TAKEOFF;
//			this->_slowStart = true;
//			desiredDistance = 50;
//			break;
//		case ALTHOLD_HOLDING:
//			this->_slowStart = false;
//			desiredDistance = 50;
//			break;
//		case ALTHOLD_LANDING:
//			this->_slowStart = false;
//			desiredDistance = ALTHOLD_HEIGHT_MIN;
//			break;
//		default:
//#if DEBUG == ENABLED
//			hal.console->println("Could not figure out state landing");
//#endif
//			desiredDistance = ALTHOLD_HEIGHT_LAND;
//			break;
//	}
//
//	this->_updateVelocity(desiredDistance);
//
//
//	_lastState = currentState;
//	_lastThrottle = rc_channels[RC_CHANNEL_THROTTLE].radio_in;
//
//}
//
//void AltHold::_updateCurrentVelocity(){
//
//	uint32_t currentTime = hal.scheduler->millis();
//	if(currentTime - this->_lastVelUpdateTime < ALTHOLD_VELOCITY_UPDATE_INTERVAL)
//	{
//		return;
//	}
//
//	uint16_t currentDistance = _rangefinder->getLastDistance();
//	float timeDiff = (float)currentTime - (float)this->_lastVelUpdateTime;
//	timeDiff /= 1000; //convert to seconds
//	float distanceDiff = (float)currentDistance - (float)this->_lastDistance;
//
//	this->_velocity = distanceDiff / timeDiff;
//
//	this->_lastDistance = currentDistance;
//	this->_lastVelUpdateTime = currentTime;
//}
//
//void AltHold::_updateVelocity(uint32_t targetDistance){
//
//	if(targetDistance > 50)
//	{
//		targetDistance = 50;
//	}
//#if DEBUG == ENABLED
//	hal.console->print(" Target Distance: ");
//	hal.console->print(targetDistance);
//#endif
//	uint32_t currentDistance = _rangefinder->getLastDistance();
//
//#if DEBUG == ENABLED
//	hal.console->print(" Current Distance: ");
//	hal.console->print(currentDistance);
//#endif
//
//	float distanceError = (float)targetDistance - (float)currentDistance;
//
//#if DEBUG == ENABLED
//	hal.console->print(" Distance Error: ");
//	hal.console->print(distanceError);
//#endif
//
//	//this makes it so that we aren't trying to go super far in one frame
////	if(distanceError > this->_distanceLeashLength)
////	{
////		targetDistance = currentDistance + this->_distanceLeashLength;
////		distanceError = this->_distanceLeashLength;
////	}
////	if(distanceError < -this->_distanceLeashLength)
////	{
////		targetDistance = currentDistance - this->_distanceLeashLength;
////		distanceError = -this->_distanceLeashLength;
////	}
//
//	float targetVelocity = this->sqrt_controller(distanceError, this->_distancePid->kP(), ALTHOLD_ACCELERATION_MAX);
//
//	if(targetVelocity > ALTHOLD_VELOCITY_MAX)
//	{
//		targetVelocity = ALTHOLD_VELOCITY_MAX;
//	}
//	if(targetVelocity < -ALTHOLD_VELOCITY_MAX)
//	{
//		targetVelocity = -ALTHOLD_VELOCITY_MAX;
//	}
//
//	this->_updateAcceleration(targetVelocity);
//
//}
//
//void AltHold::_updateAcceleration(float targetVelocity)
//{
//
//#if DEBUG == ENABLED
//	hal.console->print(" Target Velocity: ");
//	hal.console->print(targetVelocity);
//#endif
//
//	//removed the feedforward peice until i understand it
//	//believe it is just a performance thing
//	float accelFeedforward = 0;
//	this->_lastVelocityTarget = targetVelocity;
//
////	float velocityError = _vel_error_filter.apply(targetVelocity - this->_velocity);
//
//	float velocityError = targetVelocity - this->_velocity;
//
//	float p = this->_velocityPid->kP() * velocityError;
//
//	float targetAcceleration = accelFeedforward + p;
//
//	//contrain target acceleration
////	if(targetAcceleration > ALTHOLD_ACCELERATION_MAX)
////	{
////		targetAcceleration = ALTHOLD_ACCELERATION_MAX;
////	}
////	if(targetAcceleration < -ALTHOLD_ACCELERATION_MAX)
////	{
////		targetAcceleration = -ALTHOLD_ACCELERATION_MAX;
////	}
//
//	targetAcceleration = constrain_int32(targetAcceleration, -32000, 32000);
//
//	this->_updateThrottleOutput(targetAcceleration);
//}
//
//void AltHold::_updateThrottleOutput(float targetAcceleration){
//
//#if DEBUG == ENABLED
//	hal.console->print(" Target Accel: ");
//	hal.console->print(targetAcceleration);
//#endif
//
//	float acceleration = -(ahrs.get_accel_ef().z + GRAVITY_MSS) * 100.0f;
//
////	float accelerationError = constrain_int32(_accel_error_filter.apply(targetAcceleration - acceleration), -32000, 32000);
//	float accelerationError = targetAcceleration - acceleration;
//
//	float accelPidValue = this->_accelPid->get_pid(accelerationError);
//
//#if DEBUG == ENABLED
//	hal.console->print(" Accel PID Value: ");
//	hal.console->print(accelPidValue);
//	hal.console->print(" /1000: ");
//	hal.console->print(accelPidValue/1000.0f);
//#endif
//
//	float throttleOut = (accelPidValue/1000.0f) + this->_hoverPoint;
//
//	if(this->_slowStart)
//	{
//		float tmpThrottleOut = this->_slowStartThrottle + this->_slowStartOffset;
//
//		if(tmpThrottleOut < throttleOut)
//		{
//			throttleOut = tmpThrottleOut+5;
//			this->_slowStartOffset += 5;
//		}
//	}else{
//		this->_slowStartOffset = 0;
//	}
//
//	attitude.set_throttle_out(throttleOut, true);
//
//#if DEBUG == ENABLED
//	hal.console->print(" Throttle Output: ");
//	hal.console->print(throttleOut);
//	hal.console->println("");
//#endif
//}
//
//// Proportional controller with piecewise sqrt sections to constrain second derivative
////taken from AC_AttitudeControl arudpilot(master)
//float AltHold::sqrt_controller(float error, float p, float second_ord_lim)
//{
//    if (second_ord_lim <= 0.0f || p == 0.0f) {
//        return error*p;
//    }
//
//    float linear_dist = second_ord_lim/sq(p);
//
//    if (error > linear_dist) {
//        return safe_sqrt(2.0f*second_ord_lim*(error-(linear_dist/2.0f)));
//    } else if (error < -linear_dist) {
//        return -safe_sqrt(2.0f*second_ord_lim*(-error-(linear_dist/2.0f)));
//    } else {
//        return error*p;
//    }
//}
//
//bool AltHold::_caluclateHoverPoint(){
//	if(!this->_hoverPointCalculated)
//	{
//
//		if (!_gotLastDistance) {
//			_oldDistance = _rangefinder->getLastDistance();
//
//			if(_oldDistance < 2)
//			{
//				this->_hasRisen = false;
//			}
//			_gotLastDistance = true;
//		}
//		if(this->_calcHoverBeginTime == 0)
//		{
//			this->_calcHoverBeginTime = hal.scheduler->millis();
//		}else if(/*hal.scheduler->millis() - this->_calcHoverBeginTime > ALTHOLD_CALC_HOVER_TIMEOUT ||*/ _oldDistance >= ALTHOLD_CALC_HOVER_HEIGHT_TIMEOUT - ALTHOLD_HEIGHT_THRESHOLD)
//		{
//			a_led->write(HAL_GPIO_LED_OFF);
//			b_led->write(HAL_GPIO_LED_OFF);
//			c_led->write(HAL_GPIO_LED_OFF);
////			hal.console->print(" TIMEOUT(");
////			hal.console->print(this->_hoverPoint);
////			hal.console->print(", ");
////			hal.console->print(_oldDistance);
////			hal.console->print(", ");
////			hal.console->print(hal.scheduler->millis() - this->_calcHoverBeginTime);
////			hal.console->println(")");
//			if(this->_hoverPoint > RC_THROTTLE_MIN + ALTHOLD_CALC_HOVER_INCREMENT)
//				this->_hoverPoint -= ALTHOLD_CALC_HOVER_INCREMENT;
//
//			return false;
//		}
//
//		attitude.set_throttle_out(this->_hoverPoint, true);
//
//#if DEBUG == ENABLED
////		hal.console->print(" Old Distance: ");
////		hal.console->print(_oldDistance);
//#endif
//		if (loop_count % 20 == 0 && _gotLastDistance) {
//			_gotLastDistance = false;
//	//		hal.scheduler->delay(ALTHOLD_CALC_HOVER_DELAY);
//
//			uint16_t newDistance;
//			_rangefinder->update(newDistance);
//
//#if DEBUG == ENABLED
//			hal.console->print(" New Distance: ");
//			hal.console->print(newDistance);
//#endif
//
//			float distanceDiff = newDistance - _oldDistance;
//
//#if DEBUG == ENABLED
//			hal.console->print(" Distance Diff: ");
//			hal.console->print(distanceDiff);
//#endif
//
//			if(newDistance > ALTHOLD_CALC_HOVER_MAX_HEIGHT)
//			{
//
//#if DEBUG == ENABLED
//				hal.console->print(" Too High(");
//				hal.console->print(this->_lastHoverDown);
//				hal.console->print(")");
//#endif
//
//				this->_hoverPoint = this->_lastHoverDown;
//				this->_heldTime = 0;
//			}else if(distanceDiff > ALTHOLD_CALC_HOVER_THRESHOLD)
//			{
//				this->_lastHoverUp = this->_hoverPoint;
//				this->_hoverPoint = (this->_hoverPoint + this->_lastHoverDown)/2;
//				this->_hasRisen = true;
//
//#if DEBUG == ENABLED
//				hal.console->print(" We  Rose(");
//				hal.console->print(this->_hoverPoint);
//				hal.console->print(")");
//#endif
//
//				this->_heldTime = 0;
//			}else if(distanceDiff < -ALTHOLD_CALC_HOVER_THRESHOLD)
//			{
//				this->_lastHoverDown = this->_hoverPoint;
//				this->_hoverPoint += ALTHOLD_CALC_HOVER_INCREMENT;
//
//#if DEBUG == ENABLED
//				hal.console->print(" We  Drop(");
//				hal.console->print(this->_hoverPoint);
//				hal.console->print(")");
//#endif
//
//				this->_heldTime = 0;
//			}else if(this->_hasRisen)
//			{
//
//#if DEBUG == ENABLED
//				hal.console->print(" Hovering(");
//				hal.console->print(this->_hoverPoint);
//				hal.console->print(")");
//#endif
//
//				int32_t currentTime = hal.scheduler->millis();
//
//				this->_heldTime += currentTime - this->_lastHoverCalcTime;
//				if(this->_heldTime >= ALTHOLD_CALC_HOVER_HELD_TIME)
//				{
//
//					a_led->toggle();
//					b_led->toggle();
//					c_led->toggle();
//
//#if DEBUG == ENABLED
//					hal.console->print(" CALCULATED: ");
//					hal.console->print(this->_hoverPoint);
//#endif
//
//					this->_hoverPointCalculated = true;
//
//					// Write to eeprom
//					g.hover_point.set_and_save(_hoverPoint);
//					this->_slowStartThrottle = this->_hoverPoint/2;
//				}
//			}else if(!this->_hasRisen)
//			{
//				this->_hoverPoint += ALTHOLD_CALC_HOVER_INCREMENT;
//				this->_hasRisen = false;
//
//#if DEBUG == ENABLED
//				hal.console->print(" Getingup(");
//				hal.console->print(this->_hoverPoint);
//				hal.console->print(")");
//#endif
//
//				this->_heldTime = 0;
//			}else if(this->_hasRisen && newDistance <= 2){
//				this->_hoverPoint = 400;
//				this->_hasRisen = false;
//			}
//
//			if(this->_hoverPoint > RC_THROTTLE_MAX)
//			{
//				this->_hoverPoint = RC_THROTTLE_MAX;
//
//#if DEBUG == ENABLED
//				hal.console->print(" To  MUCH(");
//				hal.console->print(this->_hoverPoint);
//				hal.console->print(")");
//#endif
//
//			}
//
//			this->_lastHoverCalcTime = hal.scheduler->millis();
//
//		}
//
//#if DEBUG == ENABLED
//		hal.console->println("");
//#endif
//
//	}
//
//	return this->_hoverPointCalculated;
//}
//#endif
//
