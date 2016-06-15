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

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <PID.h>

// Local includes
#include "Tests.h"
#include "Config.h"
#include "Motors.h"

// ArduPilot Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

// MPU6050 accel/gyro chip
AP_InertialSensor_MPU6000 ins;

// RC receiver channel values
long rc_channels[8];

// Set up Motors instance to control all motors
Motors motors;

PID pids[6];

uint16_t loop_count;


void setup()
{

	loop_count = 0;

	// Initialize PID array
	pids[PID_PITCH_RATE].kP(0.7);
	//  pids[PID_PITCH_RATE].kI(1);
	pids[PID_PITCH_RATE].imax(50);

	pids[PID_ROLL_RATE].kP(0.7);
	//  pids[PID_ROLL_RATE].kI(1);
	pids[PID_ROLL_RATE].imax(50);

	pids[PID_YAW_RATE].kP(0.7);
	//  pids[PID_YAW_RATE].kI(1);
	pids[PID_YAW_RATE].imax(50);

	pids[PID_PITCH_STAB].kP(4.5);
	pids[PID_ROLL_STAB].kP(4.5);
	pids[PID_YAW_STAB].kP(10);

	Init_Arducopter();

	// If we are calibrating ESCs, do that now
	if (ESC_CALIBRATE == ENABLED) {
		hal.console->println("Entering ESC calibration now....");
		motors.calibrate_ESCs();
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

	Setup_Motors();
}

void loop()
{
//  uint16_t channels[8];  // array for raw channel values
//
//  // Read RC channels and store in channels array
//  hal.rcin->read(channels, 8);
//
//  // Copy from channels array to something human readable - array entry 0 = input 1, etc.
//  uint16_t rcthr, rcyaw, rcpit, rcroll;   // Variables to store rc input
//  rcthr = channels[2];
//  rcyaw = channels[3];
//  rcpit = channels[1];
//  rcroll = channels[0];
//
//  hal.console->printf_P(
//            PSTR("individual read THR %d YAW %d PIT %d ROLL %d\r\n"),
//            rcthr, rcyaw, rcpit, rcroll);

//	// If debugging is enabled, give a menu of options for configuring, testing, and debugging
//	if (DEBUG_ENABLED == ENABLED) {
//		hal.console->write("Testing is enabled. Choose from the available options...\n");
//		hal.console->write("  (1) Calibrate ESCs");
//	}

	// Wait until new orientation data (normally 5ms max)
	while (ins.num_samples_available() == 0);

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

	rc_channels[RC_CHANNEL_PITCH] = map(
			channels[RC_CHANNEL_PITCH],
			RC_PITCH_MIN,
			RC_PITCH_MAX,
			RC_PITCH_MIN_SCALED,
			RC_PITCH_MAX_SCALED);

	rc_channels[RC_CHANNEL_THROTTLE] = channels[RC_CHANNEL_THROTTLE];

	// Get yaw/pitch/roll data from MPU6050 sensor and convert to degrees
	ins.update();
	float sensor_roll,sensor_pitch,sensor_yaw;
	ins.quaternion.to_euler(&sensor_roll, &sensor_pitch, &sensor_yaw);
	sensor_roll = ToDeg(sensor_roll);
	// Fix roll data because APM is upside down :)
	if (sensor_roll < 0) {
		sensor_roll += 180;
	} else {
		sensor_roll -= 180;
	}
	sensor_pitch = ToDeg(sensor_pitch) ;
	sensor_yaw = ToDeg(sensor_yaw) ;

	if (DEBUG_ENABLED) {
		loop_count++;

		//We do not want the serial line to get flooded, so print out once every 20 times through or so
		if (loop_count > 20) {
			loop_count = 0;

			// Print out the sensor yaw/pitch/roll data in degrees
//			hal.console->printf("Pitch: %4.1f   Roll: %4.1f   Yaw: %4.1f\n",
//					sensor_pitch,
//					sensor_roll,
//					sensor_yaw);

//			hal.console->printf_P(PSTR("individual read THR %d YAW %d PIT %d ROLL %d\r\n"),
//					rc_channels[RC_CHANNEL_THROTTLE],
//					rc_channels[RC_CHANNEL_YAW],
//					rc_channels[RC_CHANNEL_PITCH],
//					rc_channels[RC_CHANNEL_ROLL]);
		}


	}


	// Output throttle response to motors
	motors.output();

}

AP_HAL_MAIN();    // special macro that replace's one of Arduino's to setup the code (e.g. ensure loop() is called in a loop).

/**
 *  Scales the x param (RC stick value) so that it represent something meaningful.
 *  Takes a number between one range and places it in another – e.g., if we had a value of 50,
 *      which was between 0-100, and we wanted to scale it to be between 0 and 500,
 *      the map function would return 250.
 */
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
