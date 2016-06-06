#include "Config.h"

/**
 * Various setup fragments to initialize the board and get ready for flight.
 */
void Init_Arducopter() {

	// we don't want writes to the serial port to cause us to pause
	// mid-flight, so set the serial ports non-blocking once we are
	// ready to fly

	if (!DEBUG_ENABLED) {
		hal.uartA->set_blocking_writes(false);
		hal.uartB->set_blocking_writes(false);
		hal.uartC->set_blocking_writes(false);
	}

	/*
	 * Get the accelerometer, magnetometer, and gyroscope sensors ready
	 */

	// Disable barometer to stop it corrupting bus
	hal.gpio->pinMode(40, GPIO_OUTPUT);
	hal.gpio->write(40, 1);

	// Initialize MPU6050 sensor
	ins.init(AP_InertialSensor::COLD_START,
			 AP_InertialSensor::RATE_100HZ,
			 NULL);

	// Initialize MPU6050's internal sensor fusion (aka DigitalMotionProcessing)
	hal.scheduler->suspend_timer_procs();  // stop bus collisions
	ins.dmp_init();
	ins.push_gyro_offsets_to_dmp();
	hal.scheduler->resume_timer_procs();

}

/**
 * Enables output to the motors and sends a 490Hz pulse to negate ESC averaging filter effect
 */
void Setup_Motors() {

	if (DEBUG_ENABLED) {
		hal.console->println("Setting up motors.");
	}

	// Enable output to the motors
	hal.rcout->set_freq(0xF, RC_FAST_SPEED);  // Send 490Hz pulse to negate ESC averaging filter effect
	hal.rcout->enable_mask(0xFF);
}
