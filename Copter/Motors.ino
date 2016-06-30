/*
 * Motor.ino
 *
 *  Created on: May 27, 2016
 *      Author: chris
 */

#include "Motors.h"

/**
 * Output a specified PWM to a specified motor number defined in the configuration header file.
 * A pulse width of RC_THROTTLE_MIN + 50 and below will be set to zero (turn motor off).
 * Param pwm: Pulse width modulation sent to the motor
 * 			  Must be within RC_THROTTLE_MIN (or 0 for off) and RC_THROTTLE_MAX range
 */
void Motors::output(uint16_t motor_num, uint16_t pwm) {
	if (pwm < RC_THROTTLE_MIN + min_throttle_offset) {
		pwm = 0;
	} else if (pwm > RC_THROTTLE_MAX) {
		pwm = RC_THROTTLE_MAX;
	}

	if (DEBUG_ENABLED) {
//		hal.console->printf("PWM: %d\r\n", pwm);
	}

	hal.rcout->write(motor_num, pwm);
}

/**
 * Outputs exact throttle PWM to all motors. This function does NOT perform any stabilization,
 * we just directly pass the pwm response from the throttle to the motors.
 */
void Motors::output() {

	// Get yaw/pitch/roll data from MPU6050 sensor and convert to degrees
	ins.update();
	float sensor_roll,sensor_pitch,sensor_yaw;
	ins.quaternion.to_euler(&sensor_roll, &sensor_pitch, &sensor_yaw);
	sensor_roll = ToDeg(sensor_roll) + ACCEL_X_OFFSET;
//	// Fix roll data because APM is upside down :)
//	if (sensor_roll < 0) {
//		sensor_roll += INS_ROLL_OFFSET;
//	} else {
//		sensor_roll -= INS_ROLL_OFFSET;
//	}
	sensor_pitch = ToDeg(sensor_pitch) + ACCEL_Y_OFFSET;
	sensor_yaw = ToDeg(sensor_yaw);

//	Vector3f accel = ins.get_accel();
//	float sensor_pitch = atan(accel.x / sqrt(pow(accel.y,2) + pow(accel.z,2)));
//	float sensor_roll = atan(accel.y / sqrt(pow(accel.x,2) + pow(accel.z,2)));
//	float sensor_yaw = atan(sqrt(pow(accel.x, 2) + pow(accel.y, 2)) / accel.z);
//
//	sensor_roll 	*= 180.0;
//	sensor_pitch 	*= 180.0;
//	sensor_yaw 		*= 180.0;
//
//	sensor_roll 	/= PI;
//	sensor_pitch 	/= PI;
//	sensor_yaw		/= PI;


	// Get rotational velocity data for each axis from the gyro and convert from rad/sec to deg
	Vector3f gyro = ins.get_gyro();
	double gyro_pitch = ToDeg(gyro.y) + INS_PITCH_OFFSET;
	double gyro_roll = ToDeg(gyro.x) + INS_ROLL_OFFSET;
	double gyro_yaw = ToDeg(gyro.z);

	// Perform stabilization only if throttle is above minimum level
	if (rc_channels[RC_CHANNEL_THROTTLE] > RC_THROTTLE_MIN + min_throttle_offset) {

		// To get the PID, we need to get the error from where we currently are to where we want to be

		// Stability PIDS
		float stab_output_pitch = constrain_float(
				pids[PID_PITCH_STAB].get_pid((float)rc_channels[RC_CHANNEL_PITCH] - sensor_pitch, 1),
				-250,
				250);
		float stab_output_roll = constrain_float(
				pids[PID_ROLL_STAB].get_pid((float)rc_channels[RC_CHANNEL_ROLL] - sensor_roll, 1),
				-250,
				250);
		float stab_output_yaw = constrain_float(
				pids[PID_YAW_STAB].get_pid(wrap_180(target_yaw - sensor_yaw), 1),
				-360,
				360);

		// If controller asks for yaw change, overwrite stab_output for the yaw value
		// Yaw value will be between -150 and 150 so if there is a radio value greater than a 5
		// degree offset, rotate
		if (abs(rc_channels[RC_CHANNEL_YAW]) > 5) {
			stab_output_yaw = rc_channels[RC_CHANNEL_YAW];
			target_yaw = sensor_yaw; // remember for when radio stops
		}

		// Acrobatic/Rate PIDS
		long pitch_output = (long) constrain_int16(
				pids[PID_PITCH_RATE].get_pid(stab_output_pitch - gyro_pitch, 1),
				-500,
				500);
		long roll_output  = (long) constrain_int16(
				pids[PID_ROLL_RATE].get_pid(stab_output_roll - gyro_roll, 1),
				-500,
				500);
		long yaw_output   = (long) constrain_int16(
				pids[PID_YAW_RATE].get_pid(stab_output_yaw - gyro_yaw, 1),
				-500,
				500);

		// Only worry about yaw change if the pitch and roll values are semi-stable first.
		// We want the multirotor level before we make any yaw adjustments.
		if (abs(pitch_output) > 10 || abs(roll_output) > 10 || abs(yaw_output) < THRESHOLD_YAW_OUTPUT) {
			yaw_output = 0;
		}

		// Only compensate if the roll, pitch, and yaw values are above the pre-defined threshold
//		if (abs(pitch_output) < THRESHOLD_PITCH_OUTPUT) {
//			pitch_output = 0;
//		}
//		if (abs(roll_output) < THRESHOLD_ROLL_OUTPUT) {
//			roll_output = 0;
//		}

//		// PIDS for rate mode only
		// error = desired - actual
//		long pitch_output =   pids[PID_PITCH_RATE].get_pid(rc_channels[RC_CHANNEL_PITCH] - gyro_pitch, 1);
//		long roll_output  =   pids[PID_ROLL_RATE].get_pid(rc_channels[RC_CHANNEL_ROLL] - gyro_roll, 1);
//		long yaw_output   =   pids[PID_YAW_RATE].get_pid(rc_channels[RC_CHANNEL_YAW] - gyro_yaw, 1);

		output(MOTOR_FL, rc_channels[RC_CHANNEL_THROTTLE] + roll_output + pitch_output - yaw_output);
		output(MOTOR_BL, rc_channels[RC_CHANNEL_THROTTLE] + roll_output - pitch_output + yaw_output);
		output(MOTOR_FR, rc_channels[RC_CHANNEL_THROTTLE] - roll_output + pitch_output + yaw_output);
		output(MOTOR_BR, rc_channels[RC_CHANNEL_THROTTLE] - roll_output - pitch_output - yaw_output);

		// Print out the sensor yaw/pitch/roll data in degrees
		if (DEBUG_ENABLED)
		{
			if (loop_count == 20)
			{
				loop_count = 0;

//				hal.console->printf("rc_channel_pitch: %4.1f\t accel_pitch: %4.1f\t stab_out_pitch: %4.1f\t gyro_pitch: %4.1f\t pitch_output: %li\n",
//				(float)rc_channels[RC_CHANNEL_PITCH],
//				sensor_pitch,
//				stab_output_pitch,
//				gyro_pitch,
//				pitch_output);


				hal.console->printf("Motor PWMs....FL: %li\t BL: %li\t FR: %li\t BR: %li\t\t",
						rc_channels[RC_CHANNEL_THROTTLE] + roll_output + pitch_output, // - yaw_output,
						rc_channels[RC_CHANNEL_THROTTLE] + roll_output - pitch_output, // + yaw_output,
						rc_channels[RC_CHANNEL_THROTTLE] - roll_output + pitch_output, // + yaw_output,
						rc_channels[RC_CHANNEL_THROTTLE] - roll_output - pitch_output); // - yaw_output);
//
//				hal.console->printf("RC throt: %li\t RC pit: %li\t RC roll: %li\t RC yaw: %li\t\t",
//						rc_channels[RC_CHANNEL_THROTTLE],
//						rc_channels[RC_CHANNEL_PITCH],
//						rc_channels[RC_CHANNEL_ROLL],
//						rc_channels[RC_CHANNEL_YAW]);
				hal.console->printf("Accel_Pitch: %4.1f\t Accel_Roll: %4.1f\t Accel_Yaw: %4.1f\t",
						sensor_pitch, sensor_roll, sensor_yaw);
				hal.console->printf("Gyro pitch: %4.1f\t Gyro roll: %4.1f\t Gyro yaw: %4.1f\t\t",
						gyro_pitch, gyro_roll, gyro_yaw);
				hal.console->printf("pitch_output: %li\t roll_output: %li\t yaw_output: %li\n",
						pitch_output, roll_output, yaw_output);


							// Print expected motor outputs compared to real values
//							uint16_t motor_readings[8];
//							hal.rcout->read(motor_readings, 8);
//
//							hal.console->printf("Motor PWMs (expected vs (actual))...FL: %li (%li)\t BL: %li (%li)\t"
//									"FR: %li (%li)\t BR: %li (%li)\n",
//									(long) (rc_channels[RC_CHANNEL_THROTTLE] + roll_output + pitch_output), // - yaw_output,
//									(long) motor_readings[MOTOR_FL],
//									(long) (rc_channels[RC_CHANNEL_THROTTLE] + roll_output - pitch_output), // + yaw_output,
//									(long) motor_readings[MOTOR_BL],
//									(long) (rc_channels[RC_CHANNEL_THROTTLE] - roll_output + pitch_output), // + yaw_output,
//									(long) motor_readings[MOTOR_FR],
//									(long) (rc_channels[RC_CHANNEL_THROTTLE] - roll_output - pitch_output), // - yaw_output);
//									(long) motor_readings[MOTOR_BR]);



			} else
			{
				loop_count++;
			}
		} // End if - DEBUG_ENABLED

	} else {
		// Otherwise, turn the motors off
		output_Zero();

		for (int i = 0; i < 6; i++) {
			// Reset PID integrals while we are on the ground
			pids[i].reset_I();
		}

		// reset yaw target so we maintain this on takeoff
		target_yaw = sensor_yaw;
	}
}

void Motors::output_Min() {
	output(MOTOR_FR, RC_THROTTLE_MIN + min_throttle_offset);
	output(MOTOR_BR, RC_THROTTLE_MIN + min_throttle_offset);
	output(MOTOR_FL, RC_THROTTLE_MIN + min_throttle_offset);
	output(MOTOR_BL, RC_THROTTLE_MIN + min_throttle_offset);
}

void Motors::output_Zero() {
	output(MOTOR_FR, 0);
	output(MOTOR_BR, 0);
	output(MOTOR_FL, 0);
	output(MOTOR_BL, 0);
}

void Motors::output_Throttle() {
	if (DEBUG_ENABLED) {
		if (loop_count == 20) {
			hal.console->print(rc_channels[RC_CHANNEL_THROTTLE]); //printf("Throttle value: \n", rc_channels[RC_CHANNEL_THROTTLE]);
			hal.console->println();
			loop_count = 0;
		}
		loop_count++;
	}
	output(MOTOR_FR, rc_channels[RC_CHANNEL_THROTTLE]);
	output(MOTOR_BR, rc_channels[RC_CHANNEL_THROTTLE]);
	output(MOTOR_FL, rc_channels[RC_CHANNEL_THROTTLE]);
	output(MOTOR_BL, rc_channels[RC_CHANNEL_THROTTLE]);
}

/**
 * Calibrates each of the Electronic Speed Controllers (ESCs) to the minimum and maximum
 * throttle values defined. This function is blocking, as it requires user input, and should
 * ONLY be run with the propellers off, battery disconnected from the APM (but powering the ESCs),
 * and serial monitor running.
 */
void Motors::calibrate_ESCs() {

	// To calibrate ESCs, the maximum throttle is sent to the motors. The ESCs will beep
	// (acknowledging they received the max throttle), the user will enter any key to continue,
	// and the minimum throttle value will then be sent

	int8_t motors[4] = {
				MOTOR_FR,
				MOTOR_BR,
				MOTOR_FL,
				MOTOR_BL};

	// reduce update rate to motors to 50Hz
	hal.rcout->set_freq(0xF, RC_SLOW_SPEED);  // Send 490Hz pulse to negate ESC averaging filter effect
	hal.rcout->enable_mask(0xFF);

	// Send max speed to all motors
	hal.console->println("Sending max throttle response to motors now...\n");

	for (int i = 0; i < 4; i++) {
		output(motors[i], RC_THROTTLE_MAX);
	}

	// Wait for user to enter any key before we continue
	hal.console->println("Wait for the ESCs to beep (acknowledging they received the max throttle input), "
			"then press any key to continue.\n");

	while (hal.console->available() == 0)
	{
		hal.scheduler->delay(20);
	}

	hal.console->read(); // flush the input stream

	for (int i = 0; i < 4; i++) {
		output(motors[i], RC_THROTTLE_MIN);
	}

	hal.console->println("Wait for the ESCs to beep again (acknowledging they received the min throttle input), "
			"then unplug the battery and restart the APM.");

	while (1) {
		hal.scheduler->delay(20);
	}
}

/**
 * Before calling output, call this function to set the target yaw output to be the
 * current sensor yaw value. This should be called in the setup() function, not in loop()
 */
void Motors::init_yaw() {
	// Get yaw/pitch/roll data from MPU6050 sensor and convert to degrees
	ins.update();
	float sensor_roll,sensor_pitch;
	ins.quaternion.to_euler(&sensor_roll, &sensor_pitch, &target_yaw);

	target_yaw = ToDeg(target_yaw);
}

/**
 * Wraps the passed parameter to be between the -180 and +180 range. For example, if the parameter is -181,
 * we return 179. Whereas if the parameter is 181, we return -179. If the parameter is between -180 and 180,
 * we just return that value.
 */
float Motors::wrap_180(float x) {
	if (x < -180) {
		return x + 360;
	} else if (x > 180) {
		return x - 360;
	} else {
		return x;
	}
}
