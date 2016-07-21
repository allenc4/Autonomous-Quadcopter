#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <RC_Channel.h>
#include <AP_Motors.h>

#include "Config.h"
#include "INS_UserInteract.h"

/**
 * Various setup fragments to initialize the board and get ready for flight.
 */
bool Init_Arducopter() {

//	hal.storage->init(NULL);

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

	ahrs.init();  // Internally calls MPU6050's internal sensor fusion (DigitalMotionProcessing)
	ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);

	// Initialize MPU6050 sensor
	ins.init(AP_InertialSensor::COLD_START,
			 AP_InertialSensor::RATE_100HZ);

	// Reset gyro
	ahrs.reset_gyro_drift();

	if(!ins.calibrated() || ACCEL_CALIBRATE == ENABLED){
		//flash the leds 3 times so we know it needs to be calibrated
		for(int i = 0; i < 4; i++)
		{
			a_led->write(HAL_GPIO_LED_OFF);
			b_led->write(HAL_GPIO_LED_OFF);
			c_led->write(HAL_GPIO_LED_OFF);
			hal.scheduler->delay(1000);
			a_led->write(HAL_GPIO_LED_ON);
			b_led->write(HAL_GPIO_LED_ON);
			c_led->write(HAL_GPIO_LED_ON);
			hal.scheduler->delay(1000);
		}
		// Initialize MPU6050 sensor
		float roll_trim, pitch_trim;
		AP_InertialSensor_UserInteractStream interact(hal.console);
		if(!ins.calibrate_accel(&interact, roll_trim, pitch_trim)){
			hal.scheduler->delay(500);
			return false;
		}
	}

	//check if the accelerometer has been calibrated
	//this will load the accel offsets from EEPORM
	//this should not be called during flight as it reads
	//from the eeprom and this is slow
	if(!ins.calibrated())
	{
		hal.console->println("Accelerometer is not calibrated.  Please Calibrate Before Continuing");
		return false;
	}

	// Push accelerometer and gyro offsets to DMP

	// setup fast AHRS gains to get right attitude
	ahrs.set_fast_gains(true);

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

	// Initialize compass
#if COMPASS == ENABLED
	if (!compass.init()) {
		hal.console->println("compass initialisation failed!");
		return false;
	} else {
//		compass.set_offsets(0,0,0); // set offsets to account for surrounding interference
//		compass.set_declination(ToRad(0.0)); // set local difference between magnetic north and true north
	}
#endif


//	hal.scheduler->delay(50);
//	hal.scheduler->resume_timer_procs();


	return true;
}

void flashLeds(bool flash){
	while(flash){
		a_led->mode(HAL_GPIO_LED_OFF);
		b_led->mode(HAL_GPIO_LED_ON);
		c_led->mode(HAL_GPIO_LED_OFF);
		hal.scheduler->delay(500);
		a_led->mode(HAL_GPIO_LED_OFF);
		b_led->mode(HAL_GPIO_LED_OFF);
		c_led->mode(HAL_GPIO_LED_ON);
		hal.scheduler->delay(500);
		a_led->mode(HAL_GPIO_LED_ON);
		b_led->mode(HAL_GPIO_LED_OFF);
		c_led->mode(HAL_GPIO_LED_OFF);
		hal.scheduler->delay(500);
	}

	if(!flash){
		a_led->mode(HAL_GPIO_LED_OFF);
		b_led->mode(HAL_GPIO_LED_OFF);
		c_led->mode(HAL_GPIO_LED_OFF);
		hal.scheduler->delay(500);
		a_led->mode(HAL_GPIO_LED_OFF);
		b_led->mode(HAL_GPIO_LED_ON);
		c_led->mode(HAL_GPIO_LED_OFF);
	}

}

/**
 * Enables output to the motors and sends a 490Hz pulse to negate ESC averaging filter effect
 */
void Setup_Motors() {

	if (DEBUG) {
		hal.console->println("Setting up motors.");
	}

	// Enable output to the motors
//	hal.rcout->set_freq(0xF, RC_FAST_SPEED);  // Send 490Hz pulse to negate ESC averaging filter effect
//	hal.rcout->enable_mask(0xFF);

	motors.set_update_rate(RC_FAST_SPEED);
	motors.set_frame_orientation(AP_MOTORS_X_FRAME);
	motors.Init();
	motors.set_min_throttle(100);
//	motors.set_max_throttle(1000);

	for(uint8_t i = RC_CHANNEL_MIN; i <= RC_CHANNEL_MAX; i++) {
		hal.scheduler->delay(20);
		rc_channels[i].set_pwm(hal.rcin->read(i));
	}

	// All setup stuff is done so we want the motors enabled, but we don't want the motors to spin until the pilot is ready
	motors.enable();
	motors.armed(false);
}

void Setup_RC_Channels() {
	rc_channels[RC_CHANNEL_ROLL].radio_min = RC_ROLL_MIN;
	rc_channels[RC_CHANNEL_ROLL].radio_max = RC_ROLL_MAX;
	rc_channels[RC_CHANNEL_ROLL].set_angle(4500);
	rc_channels[RC_CHANNEL_ROLL].set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

	rc_channels[RC_CHANNEL_PITCH].radio_min = RC_PITCH_MIN;
	rc_channels[RC_CHANNEL_PITCH].radio_max = RC_PITCH_MAX;
	rc_channels[RC_CHANNEL_PITCH].set_angle(4500);
	rc_channels[RC_CHANNEL_PITCH].set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

	rc_channels[RC_CHANNEL_YAW].radio_min = RC_YAW_MIN;
	rc_channels[RC_CHANNEL_YAW].radio_max = RC_YAW_MAX;
	rc_channels[RC_CHANNEL_YAW].set_angle(4500);
	rc_channels[RC_CHANNEL_YAW].set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

	rc_channels[RC_CHANNEL_THROTTLE].radio_min = RC_THROTTLE_MIN;
	rc_channels[RC_CHANNEL_THROTTLE].radio_max = RC_THROTTLE_MAX;
	rc_channels[RC_CHANNEL_THROTTLE].set_range(0, 1000);
	rc_channels[RC_CHANNEL_THROTTLE].set_range_out(0, 1000);

	// Set trim values to be in the middle for roll, pitch, yaw and min for throttle
	for (int i = 0; i < 30; i++) {
		// Read RC values
		uint16_t periods[8];
		hal.rcin->read(periods, RC_CHANNEL_MAX+1);

		for (int i = RC_CHANNEL_MIN; i <= RC_CHANNEL_MAX; i++) {
			rc_channels[i].set_pwm(periods[i]);
		}

		hal.scheduler->delay(20);
	}

	for (int i = 0; i < 4; i++) {
		rc_channels[i].trim();
	}

}
