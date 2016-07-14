#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include "Config.h"
#include "INS_UserInteract.h"

/**
 * Various setup fragments to initialize the board and get ready for flight.
 */
bool Init_Arducopter() {

	// we don't want writes to the serial port to cause us to pause
	// mid-flight, so set the serial ports non-blocking once we are
	// ready to fly

	if (DEBUG != ENABLED) {
		hal.uartA->set_blocking_writes(false);
		hal.uartB->set_blocking_writes(false);
		hal.uartC->set_blocking_writes(false);
	}

	// Initialize LEDs
	hal.gpio->pinMode(13, HAL_GPIO_OUTPUT);
	hal.gpio->write(13, 0);

    a_led = hal.gpio->channel(HAL_GPIO_A_LED_PIN);
	b_led = hal.gpio->channel(HAL_GPIO_B_LED_PIN);
    c_led = hal.gpio->channel(HAL_GPIO_C_LED_PIN);

    a_led->mode(HAL_GPIO_OUTPUT);
	b_led->mode(HAL_GPIO_OUTPUT);
    c_led->mode(HAL_GPIO_OUTPUT);

    // Turn A and C lights off during setup, but leave B on.
    a_led->write(HAL_GPIO_LED_OFF);
	b_led->write(HAL_GPIO_LED_ON);
    c_led->write(HAL_GPIO_LED_OFF);

	/*
	 * Get the accelerometer, magnetometer, and gyroscope sensors ready
	 */

	// Disable barometer to stop it corrupting bus
	hal.gpio->pinMode(40, HAL_GPIO_OUTPUT);
	hal.gpio->write(40, 1);

	// Initialize MPU6050 sensor
	ins.init(AP_InertialSensor::COLD_START,
			 AP_InertialSensor::RATE_100HZ,
			 NULL);
	ahrs.init();  // Internally calls MPU6050's internal sensor fusion (DigitalMotionProcessing)


	// Initialize MPU6050's internal sensor fusion (aka DigitalMotionProcessing)
//	hal.scheduler->suspend_timer_procs();  // stop bus collisions
//	ins.dmp_init();

	// setup fast AHRS gains to get right attitude
	ahrs.set_fast_gains(true);

	// Set accelerometer offsets and scale
//	Vector3<float> accel_offsets(ACCEL_X_OFFSET, ACCEL_Y_OFFSET, ACCEL_Z_OFFSET);
//	Vector3<float> accel_scaling(ACCEL_X_SCALE, ACCEL_Y_SCALE, ACCEL_Z_SCALE);
//	ins.set_accel_offsets(accel_offsets);
//	ins.set_accel_scale(accel_scaling);

//	ins.push_gyro_offsets_to_dmp();
//	ins.push_accel_offsets_to_dmp();

	// Initialize LIDAR and ensure it is connected at startup
#if LIDAR == ENABLED
	if (DEBUG == ENABLED) {
		hal.console->println("Initializing LIDAR...");
	}
	if (!lidar->init()) {
		hal.console->println("LIDAR not initialized. Must correct issue before flight");
		return false;
	}

	if (DEBUG == ENABLED) {
		hal.console->println("LIDAR initialized...");
	}

	// Initialize the Optical Flow sensor and ensure it is connected at startup
#if OPTFLOW == ENABLED
	if (!opticalFlow.init()) {
		hal.console->println("Optical Flow is enabled but failed to initialize the sensor");
		return false;
	}
#endif // OPTFLOW == ENABLED

#endif // LIDAR == ENABLED



//	hal.scheduler->delay(50);
//	hal.scheduler->resume_timer_procs();


	return true;
}

/**
 * Enables output to the motors and sends a 490Hz pulse to negate ESC averaging filter effect
 */
void Setup_Motors() {

	if (DEBUG) {
		hal.console->println("Setting up motors.");
	}

	// Enable output to the motors
	hal.rcout->set_freq(0xF, RC_FAST_SPEED);  // Send 490Hz pulse to negate ESC averaging filter effect
	hal.rcout->enable_mask(0xFF);
}

/**
 * Calibrate the accelerometer. This MUST be done on the ground with USB serial connected.
 */
void accel_calibration() {

	bool calibrate = false;

	if (ins.calibrated()) {
		hal.console->printf("Accelerometer has been calibrated already. Calibrate again (Y/N)?\n");

		while (hal.console->available() <= 0) {
			hal.scheduler->delay(20);
		}

		if (hal.console->read() == 'Y') {
			calibrate = true;
		}
	} else {
		calibrate = true;
	}

	if (calibrate) {
		hal.console->printf("Ensure the quadcopter is on a level surface and not moving. Press any key to continue\n");

		// Wait until the user is ready to calibrate
		while (hal.console->available() <= 0) {
			hal.scheduler->delay(20);
		}

		// Call the Ardupilot library function to calibrate the accelerometer, get offsets and scaling,
		// and pitch/roll trim values
		AP_InertialSensor_UserInteract *console = new INS_UserInteract;
		ins.calibrate_accel(NULL, console, trim_pitch, trim_roll);

		hal.console->printf("Pitch trim: %4.2f\t Roll trim: %4.2f",
				trim_pitch, trim_roll);
		while (hal.console->available() <= 0) {
			hal.scheduler->delay(20);
		}
	}

}
