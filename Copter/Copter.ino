/**
 * Autonomous quadcopter firmware for use with ArduPilot APM 2.6
 * with an indoor SLAM implementation.
 *
 * Requires ArduCopter 3.0.1-rc-2 libraries
 *    See: https://github.com/ArduPilot/ardupilot
 *
 */

#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_ADC.h>
#include <AP_Vehicle.h>
#include <AP_Notify.h>
#include <DataFlash.h>
#include <StorageManager.h>

// Below libraries needed for Inertial Sensor
#include <GCS_MAVLink.h>
#include <Filter.h>
#include <AP_InertialSensor.h>
#include <AP_Mission.h>
#include <AP_Terrain.h>
#include <AP_Buffer.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_InertialNav.h>
#include <AP_AHRS.h>

#include <AP_Baro.h>
#include <AP_GPS.h>
#include <AP_Airspeed.h>
#include <AP_Declination.h>
#include <AP_Compass.h>
#include <AP_OpticalFlow.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <GPIO.h>

#include <AP_Curve.h>
#include <AC_AttitudeControl.h>
#include <APM_PI.h>
#include <AC_PID.h>
#include <PID.h>
#include <RC_Channel.h>         // RC Channel Library

#include <AP_Motors.h>

#include <memcheck.h>           // memory limit checker
#include <AP_Scheduler.h>       // main loop scheduler

// Local includes
#include "Config.h"
#include "Parameters.h"
#include "Motors.h"
#include "OpticalFlow.h"
#include "RangeFinder_Lidar.h"

// Function definitions
static void fast_loop();
static void medium_loop();
static void slow_loop();
void rc_read();
long map(long x, long in_min, long in_max, long out_min, long out_max);

// ArduPilot Hardware Abstraction Layer (HAL)
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
AP_Vehicle::MultiCopter vechicleType;

extern const AP_Param::Info var_info[];
AP_Param param_loader(var_info);

Parameters g;

// MPU6050 accel/gyro chip
AP_InertialSensor ins;
// GPS and Barometer not formally used in this code. Only declared to create AHRS and AttitudeControl objects
AP_GPS gps;
#if CONFIG_BARO == HAL_BARO_BMP085
static AP_Baro_BMP085 barometer;
#elif CONFIG_BARO == HAL_BARO_PX4
static AP_Baro_PX4 barometer;
#elif CONFIG_BARO == HAL_BARO_VRBRAIN
static AP_Baro_VRBRAIN barometer;
#elif CONFIG_BARO == HAL_BARO_HIL
static AP_Baro_HIL barometer;
#elif CONFIG_BARO == HAL_BARO_MS5611
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
#elif CONFIG_BARO == HAL_BARO_MS5611_SPI
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
#else
 #error Unrecognized CONFIG_BARO setting
#endif

// Vector that holds gyro data
Vector3f gyroVals;

// Orientation
AP_Compass_HMC5843 compass;

////////////////////////////////////////////////////////////////////////////////
// Rate contoller targets
////////////////////////////////////////////////////////////////////////////////
//uint8_t rate_targets_frame = EARTH_FRAME;    // indicates whether rate targets provided in earth or body frame
//int32_t roll_rate_target_ef;
//int32_t pitch_rate_target_ef;
//int32_t yaw_rate_target_ef;
//int32_t roll_rate_target_bf;     // body frame roll rate target
//int32_t pitch_rate_target_bf;    // body frame pitch rate target
//int32_t yaw_rate_target_bf;      // body frame yaw rate target
////////////////////////////////////////////////////////////////////////////////
// Convenience accessors for commonly used trig functions. These values are generated
// by the DCM through a few simple equations.
// The cos values are defaulted to 1 to get a decent initial value for a level state
//float cos_roll_x         = 1.0;
//float cos_pitch_x        = 1.0;
//float cos_yaw            = 1.0;
//float sin_yaw;
//float sin_roll;
//float sin_pitch;

////////////////////////////////////////////////////////////////////////////////
// SIMPLE Mode
////////////////////////////////////////////////////////////////////////////////
int16_t control_roll;
int16_t control_pitch;

uint32_t lastModeSelectTime;
uint32_t modeSelectTimer;
int16_t lastMode;

// Roll and pitch trim values
float trim_roll, trim_pitch;

// RC receiver channel values
RC_Channel rc_channels[] = {
		RC_Channel(RC_CHANNEL_ROLL),
		RC_Channel(RC_CHANNEL_PITCH),
		RC_Channel(RC_CHANNEL_THROTTLE),
		RC_Channel(RC_CHANNEL_YAW)
};

PID pids[6];

// LIDAR Lite
#if LIDAR == ENABLED
RangeFinder * lidar = new RangeFinder_Lidar();

// Lidar must be enabled (for altitude) to enable optical flow
#if OPTFLOW == ENABLED
OpticalFlow opticalFlow(lidar);
#endif
#endif

// Set up Motors instance to control all motors
//Motors motors;
AP_MotorsQuad motors(rc_channels[RC_CHANNEL_ROLL],
		rc_channels[RC_CHANNEL_PITCH],
		rc_channels[RC_CHANNEL_THROTTLE],
		rc_channels[RC_CHANNEL_YAW]);

AP_AHRS_DCM ahrs(ins, barometer, gps);
AC_AttitudeControl attitude(ahrs, vechicleType, motors, g.p_stabilize_roll, g.p_stabilize_pitch, g.p_stabilize_yaw,
        g.pid_rate_roll, g.pid_rate_pitch, g.pid_rate_yaw);

// The Commanded ROll based on optical flow sensor.
int32_t of_roll;
// The Commanded PITCH based on optical flow sensor. Negative pitch means go forward.
int32_t of_pitch;

// LEDs
AP_HAL::DigitalSource *a_led;
AP_HAL::DigitalSource *b_led;
AP_HAL::DigitalSource *c_led;

// For debugging and printing to console
uint32_t loop_count;

// System Timers
uint32_t G_Dt;
uint32_t fastLoopTimer = 0;
//timer for how often medium loop should be executed
uint32_t mediumLoopExecute = 500;		
//last time we executed the medium loop
uint32_t mediumLoopLastExecute = 0;		
//timer for how often slow loop should be executed
uint32_t slowLoopExecute = 1000;		
//last time we executed the slow loop
uint32_t slowLoopLastExecute = 0;		

void setup()
{
	// this needs to be the first call, as it fills memory with sentinel values
	memcheck_init();
	//load parameters from eeprom
	//not sure if this is necessary but it works
	Parameters::load_parameters();

	loop_count = 0;

	// Set the maximum tilt angles
	vechicleType.angle_max = DEFAULT_ANGLE_MAX;
	// Initialize PID array
	pids[PID_PITCH_RATE].kP(PITCH_RATE_P);
//	pids[PID_PITCH_RATE].kI(PITCH_RATE_I);
//	pids[PID_PITCH_RATE].kD(PITCH_RATE_D);
	pids[PID_PITCH_RATE].imax(PITCH_RATE_I_MAX);

	pids[PID_ROLL_RATE].kP(ROLL_RATE_P);
//	pids[PID_ROLL_RATE].kI(ROLL_RATE_I);
//	pids[PID_ROLL_RATE].kD(ROLL_RATE_D);
	pids[PID_ROLL_RATE].imax(ROLL_RATE_I_MAX);

	pids[PID_YAW_RATE].kP(YAW_RATE_P);
//	pids[PID_YAW_RATE].kI(YAW_RATE_I);
//	pids[PID_YAW_RATE].kD(YAW_RATE_D);
	pids[PID_YAW_RATE].imax(YAW_RATE_I_MAX);

	pids[PID_PITCH_STAB].kP(PITCH_STAB_P);
	pids[PID_ROLL_STAB].kP(ROLL_STAB_P);
	pids[PID_YAW_STAB].kP(YAW_STAB_P);


	// If we are calibrating ESCs, do that now
	if (ESC_CALIBRATE == ENABLED) {
		hal.console->println("Entering ESC calibration now....");
//		motors.calibrate_ESCs();
	}

	if (!Init_Arducopter()) {
		// Something went wrong in the initial setup, so we dont want to continue.
		// Flash red light letting user know an error occurred
		a_led->write(HAL_GPIO_LED_ON);
		b_led->write(HAL_GPIO_LED_OFF);
		c_led->write(HAL_GPIO_LED_OFF);
		hal.scheduler->delay(1000);

		while (true) {
			a_led->write(HAL_GPIO_LED_OFF);
			hal.scheduler->delay(1000);
			a_led->write(HAL_GPIO_LED_ON);
			hal.scheduler->delay(1000);
		}
	}

	// Wait until roll, pitch, and yaw values are stable before we initialize the motors
	// (multirotor is not moving and gyro sensor data is initialized)
	hal.console->println("Waiting for roll, pitch, and yaw values to stabilize...");

	// TODO - change to false
	bool sensorsReady = false;
	float prev_sensor_roll = EMPTY, prev_sensor_pitch = EMPTY, prev_sensor_yaw = EMPTY;

	while (!sensorsReady) {
		// Wait until we have orientation data
		ins.wait_for_sample();

		hal.scheduler->delay(50);
		ahrs.update();
		float sensor_roll = ahrs.roll,
				sensor_pitch = ahrs.pitch,
				sensor_yaw = ahrs.yaw;

		// If there is no previous sensor data, set it now
		if (prev_sensor_roll == EMPTY) {
			prev_sensor_roll = sensor_roll;
			prev_sensor_pitch = sensor_pitch;
			prev_sensor_yaw = sensor_yaw;

			continue;
		}

		// Check if the difference between the current and previous sensor readings are within
		// the pre-defined stabilized range
		if (fabsf(prev_sensor_roll - sensor_roll) <= INS_SENSOR_MIN_UPDATE_RAD &&
				fabsf(prev_sensor_pitch - sensor_pitch) <= INS_SENSOR_MIN_UPDATE_RAD &&
				fabsf(prev_sensor_yaw - sensor_yaw) <= INS_SENSOR_MIN_UPDATE_RAD)
		{
			// Multirotor is stable and INS sensor is ready to go
			sensorsReady = true;

			hal.console->printf("Quad is stable and INS sensor is set... Pitch: %0.6f %0.6f   Roll: %0.6f %0.6f   Yaw: %0.6f %0.6f\n",
					prev_sensor_pitch, sensor_pitch,
					prev_sensor_roll, sensor_roll,
					prev_sensor_yaw, sensor_yaw);

			hal.console->printf("Current sensor reading vs previous....      Pitch: %0.6f   Roll: %0.6f   Yaw: %0.6f\n:",
					fabsf(prev_sensor_pitch - sensor_pitch),
					fabsf(prev_sensor_roll - sensor_roll),
					fabsf(prev_sensor_yaw - sensor_yaw));

		}

		prev_sensor_roll = sensor_roll;
		prev_sensor_pitch = sensor_pitch;
		prev_sensor_yaw = sensor_yaw;

	}

//	motors.init_yaw();

//	accel_calibration();

	// Setup RC receiver
	Setup_RC_Channels();


	// Setup motors for output
	Setup_Motors();


	// Turn on green/blue light
	a_led->write(HAL_GPIO_LED_OFF);
	b_led->write(HAL_GPIO_LED_OFF);
	c_led->write(HAL_GPIO_LED_ON);

	lastMode = NO_MODE;

}

void loop()
{
	uint32_t currentMillis = hal.scheduler->millis();
	
	ins.wait_for_sample();

	// Execute the fast loop
	// ---------------------
	fast_loop();

	if(currentMillis - mediumLoopLastExecute >= mediumLoopExecute)
	{
		mediumLoopLastExecute = currentMillis;
		medium_loop();
	}

	if(currentMillis - slowLoopLastExecute >= slowLoopExecute)
	{
		slowLoopLastExecute = currentMillis;
		slow_loop();
	}
	// Test each individual motor
//	motor_Test();

	// Test and display accelerometer/gyro values
//	accel_Gyro_Test();

	loop_count++;
}

// Main loop - 100hz
static void fast_loop() {

    G_Dt = (float)(hal.scheduler->micros() - fastLoopTimer) / 1000000.f;
    fastLoopTimer = hal.scheduler->micros();
    // IMU DCM Algorithm
    ahrs.update();


//	if (DEBUG == ENABLED) {
//		if (loop_count % 20 == 0) {
////			hal.console->printf("Channels: [1]: %u\t [2]: %u\t [3]: %u\t [4]: %u\t [5]: %u\t [6]: %u\t [7]: %u \t [8]: %u\n",
////					channels[0], channels[1], channels[2], channels[3], channels[4], channels[5], channels[6], channels[7]);
//
//			hal.console->printf("Throttle: %ld\t Roll: %u (%ld)\t Pitch: %u (%ld)\t Yaw: %u (%ld)\n",
//					rc_channels[RC_CHANNEL_THROTTLE],
//					channels[RC_CHANNEL_ROLL], rc_channels[RC_CHANNEL_ROLL],
//					channels[RC_CHANNEL_PITCH], rc_channels[RC_CHANNEL_PITCH],
//					channels[RC_CHANNEL_YAW], rc_channels[RC_CHANNEL_YAW]);
//		}
//	}

    rc_read();
	// DEBUGGING PURPOSES ONLY ///////////////////////////////
//	rc_channels[RC_CHANNEL_THROTTLE].set_pwm(RC_THROTTLE_MIN + 400);
//	rc_channels[RC_CHANNEL_ROLL].set_pwm((RC_ROLL_MIN + RC_ROLL_MAX)/2);
//	rc_channels[RC_CHANNEL_PITCH].set_pwm((RC_PITCH_MIN + RC_PITCH_MAX)/2);
//	rc_channels[RC_CHANNEL_YAW].set_pwm((RC_YAW_MIN + RC_YAW_MAX)/2);

	if(rc_channels[RC_CHANNEL_THROTTLE].radio_in <= rc_channels[RC_CHANNEL_THROTTLE].radio_min  + 75) {
		if (rc_channels[RC_CHANNEL_YAW].radio_in > (rc_channels[RC_CHANNEL_YAW].radio_max - 25)) {
			if(lastMode != MOTORS_ARMED)
			{
				lastModeSelectTime = hal.scheduler->millis();
				modeSelectTimer = 0;
				lastMode = MOTORS_ARMED;
			}

			uint32_t currentTime = hal.scheduler->millis();
			modeSelectTimer += currentTime - lastModeSelectTime;
			lastModeSelectTime = currentTime;

		} else if (rc_channels[RC_CHANNEL_YAW].radio_in < (rc_channels[RC_CHANNEL_YAW].radio_min  + 25)) {

			if(lastMode != MOTORS_DISARMED)
			{
				lastModeSelectTime = hal.scheduler->millis();
				modeSelectTimer = 0;
				lastMode = MOTORS_DISARMED;
			}

			uint32_t currentTime = hal.scheduler->millis();
			modeSelectTimer += currentTime - lastModeSelectTime;
			lastModeSelectTime = currentTime;
		}

	}

	if(modeSelectTimer > MODE_SELECT_TIME) {
		switch (lastMode) {
		case MOTORS_ARMED:
			hal.console->print("motors armed\n");
			motors.armed(true);
			break;
		case MOTORS_DISARMED:
			hal.console->print("motors disarmed\n");
			motors.armed(false);
			break;
		}
		modeSelectTimer = 0;
		lastMode = NO_MODE;
	}



	// run low level rate controllers that only require IMU data
	attitude.rate_controller_run();

	// Only output to motors if we are armed
	motors.output();

#if DEBUG == ENABLED
	if (loop_count % 20 == 0) {
		hal.console->printf("Motor FL: %d\t FR: %d\t BL: %d\t BR: %d\n",
				motors.motor_out[2],
				motors.motor_out[0],
				motors.motor_out[1],
				motors.motor_out[3]);
//		hal.console->printf("RCTCI: %d\t RCTRI: %d\t", rc_channels[RC_CHANNEL_THROTTLE].control_in, rc_channels[RC_CHANNEL_THROTTLE].radio_in);
//		hal.console->printf("RCPCI: %d\t RCPRI: %d\t", rc_channels[RC_CHANNEL_PITCH].control_in, rc_channels[RC_CHANNEL_PITCH].radio_in);
//		hal.console->printf("RCRCI: %d\t RCRRI: %d\t", rc_channels[RC_CHANNEL_ROLL].control_in, rc_channels[RC_CHANNEL_ROLL].radio_in);
//		hal.console->printf("RCYCI: %d\t RCYRI: %d\n", rc_channels[RC_CHANNEL_YAW].control_in, rc_channels[RC_CHANNEL_YAW].radio_in);
	}
#endif

	// run the attitude controllers
	stabilize_run();

	// Update readings from LIDAR and Optical Flow sensors
#if LIDAR == ENABLED
	lidar->update();

	// Test and display LIDAR values
//	lidarTest();

#if OPTFLOW == ENABLED
	opticalFlow.update();
//	opticalFlow.debug_print();

#endif
#endif
}

static void medium_loop() {
//	hal.console->printf("Medium loop time: %lu\n", hal.scheduler->millis());


}

static void slow_loop() {

	// TODO - Check failsafes
}

void rc_read() {
	// Read RC values
	rc_channels[RC_CHANNEL_ROLL].set_pwm(hal.rcin->read(RC_CHANNEL_ROLL));
	rc_channels[RC_CHANNEL_PITCH].set_pwm(hal.rcin->read(RC_CHANNEL_PITCH));
	rc_channels[RC_CHANNEL_THROTTLE].set_pwm(hal.rcin->read(RC_CHANNEL_THROTTLE));
	rc_channels[RC_CHANNEL_YAW].set_pwm(hal.rcin->read(RC_CHANNEL_YAW));
}

/**
 *  Scales the x parameter and scales it so it is between a new min and max rangethat it represent something meaningful.
 *  Takes a number between one range and places it in another – e.g., if we had a value of 50,
 *      which was between 0-100, and we wanted to scale it to be between 0 and 500,
 *      the map function would return 250.
 */
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//static void update_trig() {
//    Vector2f yawvector;
//    const Matrix3f &temp   = ahrs.get_dcm_matrix();
//
//    yawvector.x = temp.a.x; 	// sin
//    yawvector.y = temp.b.x;		// cos
//    yawvector.normalize();
//
//    cos_pitch_x = safe_sqrt(1 - (temp.c.x * temp.c.x));	 // level = 1
//    cos_roll_x  = temp.c.z / cos_pitch_x;				 // level = 1
//
//    cos_pitch_x = constrain_float(cos_pitch_x, 0, 1.0);
//    // this relies on constrain_float() of infinity doing the right thing,
//    // which it does do in avr-libc
//    cos_roll_x = constrain_float(cos_roll_x, -1.0, 1.0);
//
//    sin_yaw = constrain_float(yawvector.y, -1.0, 1.0);
//    cos_yaw = constrain_float(yawvector.x, -1.0, 1.0);
//
//    // added to convert earth frame to body frame for rate controllers
//    sin_pitch = -temp.c.x;
//    sin_roll  = temp.c.y / cos_pitch_x;
//
//    //flat:
//    // 0 ° = cos_yaw:  1.00, sin_yaw:  0.00,
//    // 90° = cos_yaw:  0.00, sin_yaw:  1.00,
//    // 180 = cos_yaw: -1.00, sin_yaw:  0.00,
//    // 270 = cos_yaw:  0.00, sin_yaw: -1.00,
//
//    gyroVals = ins.get_gyro();
//}

AP_HAL_MAIN();  // special macro that replace's one of Arduino's to setup the code (e.g. ensure loop() is called in a loop).
