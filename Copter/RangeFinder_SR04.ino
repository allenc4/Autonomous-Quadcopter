#include "RangeFinder_SR04.h"

/**
 * Does a quick write to the I2C bus to check if the LIDAR is connected.
 * Returns false if the write was unsuccessful (probably disconnected). If false is returned,
 * LIDAR is NOT ready to use. If everything is connected and working OK, returns true.
 */
bool RangeFinder_Lidar::init() {
	// TODO - Implement
	return true;
}


/**
 * Gets the last read distance (in centimeters) if there is a successful read.
 * If the read was successful, returns true; otherwise, returns false
 */
bool RangeFinder_Lidar::update() {
	// TODO - Implement
	return false;
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
