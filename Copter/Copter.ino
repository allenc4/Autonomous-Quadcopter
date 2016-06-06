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
uint16_t rc_channels[4];

// Set up Motors instance to control all motors
Motors motors;

PID pids[6];

uint16_t loop_count;


void setup()
{
	loop_count = 0;

	Init_Arducopter();
	Setup_Motors();

	// Initialize PID array
	pids[PID_PITCH_RATE].kP(0.7);
	//  pids[PID_PITCH_RATE].kI(1);
	pids[PID_PITCH_RATE].imax(50);

	pids[PID_ROLL_RATE].kP(0.7);
	//  pids[PID_ROLL_RATE].kI(1);
	pids[PID_ROLL_RATE].imax(50);

	pids[PID_YAW_RATE].kP(2.5);
	//  pids[PID_YAW_RATE].kI(1);
	pids[PID_YAW_RATE].imax(50);

	pids[PID_PITCH_STAB].kP(4.5);
	pids[PID_ROLL_STAB].kP(4.5);
	pids[PID_YAW_STAB].kP(10);


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
	hal.rcin->read(rc_channels, 4);

	// Scale the yaw, roll, and pitch values in the rc_channels array
	rc_channels[RC_CHANNEL_YAW] = map(
			rc_channels[RC_CHANNEL_YAW],
			RC_YAW_MIN,
			RC_YAW_MAX,
			RC_YAW_MIN_SCALED,
			RC_YAW_MAX_SCALED);

	rc_channels[RC_CHANNEL_ROLL] = map(
			rc_channels[RC_CHANNEL_ROLL],
			RC_ROLL_MIN,
			RC_ROLL_MAX,
			RC_ROLL_MIN_SCALED,
			RC_ROLL_MAX_SCALED);

	rc_channels[RC_CHANNEL_PITCH] = map(
			rc_channels[RC_CHANNEL_PITCH],
			RC_PITCH_MIN,
			RC_PITCH_MAX,
			RC_PITCH_MIN_SCALED,
			RC_PITCH_MAX_SCALED);

	// Get yaw/pitch/roll data from MPU6050 sensor and convert to degrees
	ins.update();
	float sensor_roll,sensor_pitch,sensor_yaw;
	ins.quaternion.to_euler(&sensor_roll, &sensor_pitch, &sensor_yaw);
	sensor_roll = ToDeg(sensor_roll) ;
	sensor_pitch = ToDeg(sensor_pitch) ;
	sensor_yaw = ToDeg(sensor_yaw) ;

	if (DEBUG_ENABLED) {
		loop_count++;

		//We do not want the serial line to get flooded, so print out once every 20 times through or so
		if (loop_count >= 20) {
			loop_count = 0;

			// Print out the sensor yaw/pitch/roll data in degrees
			hal.console->printf("Pitch: %4.1f   Roll: %4.1f   Yaw: %4.1f\n",
					sensor_pitch,
					sensor_roll,
					sensor_yaw);

		}
//		hal.console->printf_P(PSTR("individual read THR %d YAW %d PIT %d ROLL %d\r\n"),
//					rc_channels[RC_CHANNEL_THROTTLE],
//					rc_channels[RC_CHANNEL_YAW],
//					rc_channels[RC_CHANNEL_PITCH],
//					rc_channels[RC_CHANNEL_ROLL]);


	}




	// Output throttle response to motors
	motors.output();


//	hal.scheduler->delay(50);
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
