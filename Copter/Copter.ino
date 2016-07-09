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

// Below libraries needed for Inertial Sensor
#include <GCS_MAVLink.h>
#include <Filter.h>
#include <AP_InertialSensor.h>
#include <AP_AHRS.h>

#include <AP_Baro.h>
#include <AP_GPS.h>
#include <AP_Airspeed.h>
#include <AP_Declination.h>
#include <AP_Compass.h>
#include <AP_OpticalFlow.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <AC_PID.h>
#include <PID.h>

#include <memcheck.h>           // memory limit checker
#include <AP_Scheduler.h>       // main loop scheduler


// Local includes
#include "Config.h"
#include "Motors.h"
#include "OpticalFlow.h"
#include "RangeFinder.h"

// Function definitions
static void fast_loop();
static void medium_loop();
static void slow_loop();
static void update_trig();
long map(long x, long in_min, long in_max, long out_min, long out_max);


// ArduPilot Hardware Abstraction Layer (HAL)
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

// MPU6050 accel/gyro chip
AP_InertialSensor_MPU6000 ins;
GPS *gps;  // Not formally used in this code. Only declared to create AHRS object
AP_AHRS_MPU6000 ahrs(&ins, gps);

// Orientation
////////////////////////////////////////////////////////////////////////////////
// Convenience accessors for commonly used trig functions. These values are generated
// by the DCM through a few simple equations.
// The cos values are defaulted to 1 to get a decent initial value for a level state
float cos_roll_x         = 1.0;
float cos_pitch_x        = 1.0;
float cos_yaw            = 1.0;
float sin_yaw;
float sin_roll;
float sin_pitch;

// Roll and pitch trim values
float trim_roll, trim_pitch;

// RC receiver channel values
long rc_channels[8];

PID pids[6];

// LIDAR Lite
#if LIDAR == ENABLED
RangeFinder lidar;

// Lidar must be enabled (for altitude) to enable optical flow
#if OPTFLOW == ENABLED
OpticalFlow opticalFlow(&lidar);
#endif
#endif

// Set up Motors instance to control all motors
Motors motors;

// The Commanded ROll based on optical flow sensor.
int32_t of_roll;
// The Commanded PITCH based on optical flow sensor. Negative pitch means go forward.
int32_t of_pitch;

// LEDs
AP_HAL::DigitalSource *a_led;
AP_HAL::DigitalSource *b_led;
AP_HAL::DigitalSource *c_led;

// For debugging and printing to console
uint16_t loop_count;

// System Timers
// Time in microseconds of main control loop
static uint32_t fast_loopTimer;
// Counters for branching from 10 hz control loop
static uint8_t medium_loopCounter;
// Counters for branching from 3.3 hz control loop
static uint8_t slow_loopCounter;
// Counter of main loop executions.  Used for performance monitoring and failsafe processing
static uint16_t mainLoop_count;
// the time when the last HEARTBEAT message arrived from a GCS - used for triggering gcs failsafe
//static uint32_t last_heartbeat_ms;


// main loop scheduler
static AP_Scheduler scheduler;

/*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 10ms units) and the maximum time they are expected to take (in
  microseconds)
 */
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
//    { update_GPS,            2,     900 },
	{ medium_loop,         2,     500 },
	{ slow_loop,			10,     500 }
//    { fifty_hz_loop,         2,     950 },
//    { slow_loop,            10,     500 },
//    { gcs_send_heartbeat,  100,     700 },
//    { compass_accumulate,    2,     700 },
//    { super_slow_loop,     100,    1100 },
//    { perf_update,        1000,     500 }
};

void setup()
{

	// this needs to be the first call, as it fills memory with sentinel values
	memcheck_init();

	loop_count = 0;

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
		motors.calibrate_ESCs();
	}

	if (!Init_Arducopter()) {
		// Something went wrong in the initial setup, so we dont want to continue.
		// Flash red light letting user know an error occurred
		a_led->write(0);
		b_led->write(1);
		c_led->write(1);
		hal.scheduler->delay(1000);

		while (true) {
			a_led->write(1);
			hal.scheduler->delay(1000);
			a_led->write(0);
			hal.scheduler->delay(1000);
		}
	}

	// Wait until roll, pitch, and yaw values are stable before we initialize the motors
	// (multirotor is not moving and gyro sensor data is initialized)
	hal.console->println("Waiting for roll, pitch, and yaw values to stabilize...");

	bool sensorsReady = false;
	float prev_sensor_roll = EMPTY, prev_sensor_pitch = EMPTY, prev_sensor_yaw = EMPTY;

	while (!sensorsReady) {
		// Wait until we have orientation data
		while (ins.num_samples_available() == 0);

		hal.scheduler->delay(20);
		ins.update();
		float sensor_roll,sensor_pitch,sensor_yaw;
		ins.quaternion.to_euler(&sensor_roll, &sensor_pitch, &sensor_yaw);

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

	motors.init_yaw();

//	accel_calibration();

	Setup_Motors();

	// Initialize the main loop scheduler
	scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
}

void loop()
{
	// Since we use the AP_Scheduler class to queue function calls,
	// we need to keep track of processing time for the loops
	uint32_t timer = hal.scheduler->micros();

	// We want this to execute fast
	// ----------------------------
	if (ins.num_samples_available() >= 2) {

		// check loop time
		fast_loopTimer = timer;

		// for mainloop failure monitoring
		mainLoop_count++;

		// Execute the fast loop
		// ---------------------
		fast_loop();

		// tell the scheduler one tick has passed
		scheduler.tick();
	} else {
		uint16_t deltaTime = timer - fast_loopTimer;
		if (deltaTime < 10000) {
			uint16_t time_to_next_loop = 10000 - deltaTime;
			scheduler.run(time_to_next_loop);
		}
	}

	// Test each individual motor
//	motor_Test();

	// Test and display accelerometer/gyro values
//	accel_Gyro_Test();

}

// Main loop - 100hz
static void fast_loop() {
    // IMU DCM Algorithm
    ahrs.update();
	ins.update();

	update_trig();

	// Get RC values
	uint16_t channels[8];
	hal.rcin->read(channels, 8);

	// Scale the yaw, roll, and pitch values in the rc_channels array
	rc_channels[RC_CHANNEL_YAW] = map(
			channels[RC_CHANNEL_YAW],
			RC_YAW_MIN,
			RC_YAW_MAX,
			RC_YAW_MIN_SCALED,
			RC_YAW_MAX_SCALED);

	rc_channels[RC_CHANNEL_ROLL] = map(
			channels[RC_CHANNEL_ROLL],
			RC_ROLL_MIN,
			RC_ROLL_MAX,
			RC_ROLL_MIN_SCALED,
			RC_ROLL_MAX_SCALED);

	rc_channels[RC_CHANNEL_PITCH] = -map(
			channels[RC_CHANNEL_PITCH],
			RC_PITCH_MIN,
			RC_PITCH_MAX,
			RC_PITCH_MIN_SCALED,
			RC_PITCH_MAX_SCALED);

	rc_channels[RC_CHANNEL_THROTTLE] = channels[RC_CHANNEL_THROTTLE];

	// DEBUGGING PURPOSES ONLY ///////////////////////////////
	//	rc_channels[RC_CHANNEL_THROTTLE] = RC_THROTTLE_MIN + 400;
	//	rc_channels[RC_CHANNEL_ROLL] = 0;
	//	rc_channels[RC_CHANNEL_PITCH] = 0;
	//	rc_channels[RC_CHANNEL_YAW] = 0;

	// Output throttle response to motors
	 motors.output();
}

static void medium_loop() {

	// Update readings from LIDAR and Optical Flow sensors
#if LIDAR == ENABLED
	lidar.update();

	// Test and display LIDAR values
//	lidarTest();

#if OPTFLOW == ENABLED
	opticalFlow.update();
#endif
#endif

}

static void slow_loop() {

	// TODO - Check failsafes
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

static void update_trig() {
    Vector2f yawvector;
    const Matrix3f &temp   = ahrs.get_dcm_matrix();

    yawvector.x = temp.a.x; 	// sin
    yawvector.y = temp.b.x;		// cos
    yawvector.normalize();

    cos_pitch_x = safe_sqrt(1 - (temp.c.x * temp.c.x));	 // level = 1
    cos_roll_x  = temp.c.z / cos_pitch_x;				 // level = 1

    cos_pitch_x = constrain_float(cos_pitch_x, 0, 1.0);
    // this relies on constrain_float() of infinity doing the right thing,
    // which it does do in avr-libc
    cos_roll_x = constrain_float(cos_roll_x, -1.0, 1.0);

    sin_yaw = constrain_float(yawvector.y, -1.0, 1.0);
    cos_yaw = constrain_float(yawvector.x, -1.0, 1.0);

    // added to convert earth frame to body frame for rate controllers
    sin_pitch = -temp.c.x;
    sin_roll  = temp.c.y / cos_pitch_x;

    //flat:
    // 0 ° = cos_yaw:  1.00, sin_yaw:  0.00,
    // 90° = cos_yaw:  0.00, sin_yaw:  1.00,
    // 180 = cos_yaw: -1.00, sin_yaw:  0.00,
    // 270 = cos_yaw:  0.00, sin_yaw: -1.00,
}

AP_HAL_MAIN();  // special macro that replace's one of Arduino's to setup the code (e.g. ensure loop() is called in a loop).
