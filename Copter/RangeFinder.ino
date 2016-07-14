#include "RangeFinder.h"

RangeFinder::RangeFinder() {
	numReadFails = 0;
	distCM = 0;
	timeLastUpdate = 0;
}

/**
 * Checks the current system time versus the last update time of the sensor read.
 * If the last update time is more than the predefined allocated timeout time (LIDAR_READ_TIMEOUT_MS),
 * function returns false, meaning we are not receiving any data.
 */
bool RangeFinder::isHealthy() {
	///////////////////////////////////////////////////////////////////
	// FOR TESTING OF OPTICAL FLOW SENSOR ONLY
	///////////////////////////////////////////////////////////////////
//	return true;

	if (hal.scheduler->millis() - timeLastUpdate >= RANGEFINDER_READ_TIMEOUT_MS ||
			numReadFails >= RANGEFINDER_READ_TIMOUT_ATTEMPTS) {
		return false;
	} else {
		return true;
	}
}

/**
 * Gets the last known value read from the LIDAR
 */
uint16_t RangeFinder::getLastDistance() {
	return distCM;
}
