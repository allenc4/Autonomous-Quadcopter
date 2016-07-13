#include "RangeFinder.h"

RangeFinder::RangeFinder() {
	numReadFails = 0;
}

/**
 * Does a quick write to the I2C bus to check if the LIDAR is connected.
 * Returns false if the write was unsuccessful (probably disconnected). If false is returned,
 * LIDAR is NOT ready to use. If everything is connected and working OK, returns true.
 */
bool RangeFinder::init() {
	hal.i2c->setTimeout(50);

	// Get pointer to i2c bus semaphore
	AP_HAL::Semaphore *i2c_sem = hal.i2c->get_semaphore();

	// If we can't get the semaphore, exit immediately
	if (!i2c_sem->take(1)) {
		hal.console->println("Couldnt take sem");
		return false;
	}

	// Send command to lidar to take reading
	if (hal.i2c->writeRegister(LIDAR_ADDRESS, REGISTER_MEASURE, MEASURE_VALUE) != 0) {
		hal.console->println("Couldnt Write Register");
		i2c_sem->give();
		return false;
	}

	// If we made it here, write was successful
	i2c_sem->give();
	timeLastUpdate = hal.scheduler->millis();
	return true;
}

/**
 * Checks the current system time versus the last update time of the sensor read.
 * If the last update time is more than the predefined allocated timeout time (LIDAR_READ_TIMEOUT_MS),
 * function returns false, meaning we are not receiving any data.
 */
bool RangeFinder::isHealthy() {
	if (hal.scheduler->millis() - timeLastUpdate <= LIDAR_READ_TIMEOUT_MS ||
			numReadFails > LIDAR_READ_TIMOUT_ATTEMPTS) {
		return true;
	} else {
		return false;
	}
}

/**
 * Gets the last read distance (in centimeters) if there is a successful read.
 * Updates the passed uint16_t variable if the read was successful and returns true.
 * Otherwise, returns false
 */
bool RangeFinder::getDistance(uint16_t &distance) {
	timeLastUpdate = hal.scheduler->millis();

	// Start by getting the i2c bus semaphore
	AP_HAL::Semaphore *i2c_sem = hal.i2c->get_semaphore();

	// If we can't get the semaphore, exit immediately
	if (!i2c_sem->take(1)) {
		numReadFails++;
		return false;
	}

	// Send command to lidar to take reading
	if (hal.i2c->writeRegister(LIDAR_ADDRESS, REGISTER_MEASURE, MEASURE_VALUE) != 0) {
		numReadFails++;
		i2c_sem->give();
		return false;
	}

	hal.scheduler->delay(1);


	uint8_t buf[2];

	// Read the high and low byte distance registers
	if (hal.i2c->readRegisters(LIDAR_ADDRESS, REGISTER_HIGH_LOW_BYTES, 2, &buf[0]) != 0) {
		// Problem reading
		numReadFails++;
		i2c_sem->give();
		return false;
	}

	// Combine results into a cm distance
	distance = ((uint16_t)buf[0] << 8 | buf[1]);

	i2c_sem->give();
	numReadFails = 0;  // Reset numReadFails since we only count consecutive failures

	// Wait a bit before next reading
	hal.scheduler->delay_microseconds(100);

	return true;
}
