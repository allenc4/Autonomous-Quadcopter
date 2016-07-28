#include "RangeFinder_Lidar.h"

/**
 * Does a quick write to the I2C bus to check if the LIDAR is connected.
 * Returns false if the write was unsuccessful (probably disconnected). If false is returned,
 * LIDAR is NOT ready to use. If everything is connected and working OK, returns true.
 */
bool RangeFinder_Lidar::init() {

	///////////////////////////////////////////////////////////////////
	// FOR TESTING OF OPTICAL FLOW SENSOR ONLY
	///////////////////////////////////////////////////////////////////
//	return true;
	_starting_distance_offset = 0;
	hal.i2c->setTimeout(50);

	// Send command to lidar to take reading
	if (!update(_starting_distance_offset)) {
		return false;
	} else {
		return true;
	}
}


/**
 * Gets the last read distance (in centimeters) if there is a successful read.
 * If the read was successful, returns true; otherwise, returns false
 */
bool RangeFinder_Lidar::update() {
	///////////////////////////////////////////////////////////////////
	// FOR TESTING OF OPTICAL FLOW SENSOR ONLY
	///////////////////////////////////////////////////////////////////
//	distCM = 49;
//	return true;

	timeLastUpdate = hal.scheduler->millis();

	// Start by getting the i2c bus semaphore
	AP_HAL::Semaphore *i2c_sem = hal.i2c->get_semaphore();

	// If we can't get the semaphore, exit immediately
	if (!i2c_sem->take(1)) {
		hal.console->printf("Couldn't get semaphore for I2C\n");
		numReadFails++;
		return false;
	}

	// Send command to lidar to take reading
	if (hal.i2c->writeRegister(LIDAR_ADDRESS, REGISTER_MEASURE, MEASURE_VALUE) != 0) {
		hal.console->printf("Error writing to LIDAR register\n");
		numReadFails++;
		i2c_sem->give();
		return false;
	}

	hal.scheduler->delay(10);


	uint8_t buf[2];

	// Read the high and low byte distance registers
	if (hal.i2c->readRegisters(LIDAR_ADDRESS, REGISTER_HIGH_LOW_BYTES, 2, &buf[0]) != 0) {
		// Problem reading
		hal.console->printf("Error reading from LIDAR\n");
		numReadFails++;
		i2c_sem->give();
		return false;
	}

	// Combine results into a cm distance (taking into account the offset that the lidar is from the ground)
	distCM = ((uint16_t)buf[0] << 8 | buf[1]);
	if (distCM < _starting_distance_offset) {
		distCM = 0;
	} else {
		distCM -= _starting_distance_offset;
	}

	//correct for quad tilt

	 //downward facing vector with no rotation
	Vector3f original(0,0,-1);

	//rotate the orignal vector using the sensors DCM matrix
	//not sure if this is the body or global dcm matrix havent checked
	//should be the body tho
	Vector3f rotated = ahrs.get_dcm_matrix() * original;

	//this gives us the cos of the angle between the two vectors
	float cosTheta = (original * rotated)/(original.length() * rotated.length());
	float angle = acos(cosTheta);

	//using sin rule
	//a/sin(A) = b/sin(B)
	//a/sin(90) = b/sin(angle)
	//sin(90) = 1
	//a = b/sin(angle)
	//b = a*sin(angle)
	distCM *= sin(angle);

	i2c_sem->give();
	numReadFails = 0;  // Reset numReadFails since we only count consecutive failures

	// Wait a bit before next reading
	hal.scheduler->delay_microseconds(100);

	return true;
}

/**
 * Gets the last read distance (in centimeters) if there is a successful read.
 * If the read was successful, returns true and updates the passed parameter
 * with the distance in centimeters; otherwise, returns false
 */
bool RangeFinder_Lidar::update(uint16_t &distance) {
	bool stat = update();
	if (stat) {
		distance = distCM;
	}
	return stat;
}

/**
 * Get the distance offset of the lidar. That is, get the distance the lidar is
 * from the ground when landed.
 */
uint16_t RangeFinder_Lidar::getStartingDistanceOffset() {
	return _starting_distance_offset;
}
