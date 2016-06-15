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
	// Get rotational velocity data for each axis from the gyro and convert from rad/sec to deg
	Vector3f gyro = ins.get_gyro();
	float gyro_pitch = ToDeg(gyro.y);
	float gyro_roll = ToDeg(gyro.x);
	float gyro_yaw = ToDeg(gyro.z);

	// Perform acrobatic stabilization only if throttle is above minimum level
	if (rc_channels[RC_CHANNEL_THROTTLE] > RC_THROTTLE_MIN + min_throttle_offset) {

		// To get the PID, we need to get the error from we we currently are to where we want to be
		long pitch_output = pids[PID_PITCH_RATE].get_pid(gyro_pitch - rc_channels[RC_CHANNEL_PITCH], 1);
		long roll_output  = pids[PID_ROLL_RATE].get_pid(gyro_roll - rc_channels[RC_CHANNEL_ROLL], 1);
		long yaw_output   = pids[PID_YAW_RATE].get_pid(gyro_yaw - rc_channels[RC_CHANNEL_YAW], 1);

		output(MOTOR_FL, rc_channels[RC_CHANNEL_THROTTLE] - roll_output + pitch_output - yaw_output);
		output(MOTOR_BL, rc_channels[RC_CHANNEL_THROTTLE] - roll_output - pitch_output + yaw_output);
		output(MOTOR_FR, rc_channels[RC_CHANNEL_THROTTLE] + roll_output + pitch_output + yaw_output);
		output(MOTOR_BR, rc_channels[RC_CHANNEL_THROTTLE] + roll_output - pitch_output - yaw_output);

		// Print out the sensor yaw/pitch/roll data in degrees
		if (DEBUG_ENABLED && loop_count == 20) {
			hal.console->printf("Motor PWMs....FL: %li    BL: %li    FR: %li    BR: %li        ",
					rc_channels[RC_CHANNEL_THROTTLE] - roll_output + pitch_output - yaw_output,
					rc_channels[RC_CHANNEL_THROTTLE] - roll_output - pitch_output + yaw_output,
					rc_channels[RC_CHANNEL_THROTTLE] + roll_output + pitch_output + yaw_output,
					rc_channels[RC_CHANNEL_THROTTLE] + roll_output - pitch_output - yaw_output);

			hal.console->printf("RC throt: %li    RC pit: %li    RC roll: %li    RC yaw: %li        ",
					rc_channels[RC_CHANNEL_THROTTLE],
					rc_channels[RC_CHANNEL_PITCH],
					rc_channels[RC_CHANNEL_ROLL],
					rc_channels[RC_CHANNEL_YAW]);
			hal.console->printf("Gyro pitch: %4.1f    Gyro roll: %4.1f    Gyro yaw: %4.1f          ",
					gyro_pitch, gyro_roll, gyro_yaw);
			hal.console->printf("pitch_output: %li    roll_output: %li    yaw_output: %li\n",
					pitch_output, roll_output, yaw_output);

		}
	} else {
		// Otherwise, turn the motors off
		output_Zero();

		for (int i = 0; i < 6; i++) {
			// Reset PID integrals while we are on the ground
			pids[i].reset_I();
		}
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

/**
 * Calibrates each of the Electronic Speed Controllers (ESCs) to the minimum and maximum
 * throttle values defined. This function is blocking, as it requires user input, and should
 * ONLY be run with the propellers off, battery disconnected from the APM (but powering the ESCs),
 * and serial monitor running.
 */
void Motors::calibrate_ESCs() {

	// To calibrate ESCs, the maximum throttle is sent to the motors. The ESCs will beep
	// (acknowledging they received the max throttle), the user will enter any key to continue,
	// and the minimum throttle value will then be sent.

	// Send max speed to all motors
	int8_t motors[4] = {
			MOTOR_FR,
			MOTOR_BR,
			MOTOR_FL,
			MOTOR_BL};

	hal.console->println("Sending max throttle response to motors now...\n");

	for (int i = 0; i < 4; i++) {
		output(i, RC_THROTTLE_MAX);
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
		output(i, RC_THROTTLE_MIN);
	}

	hal.console->println("Wait for the ESCs to beep again (acknowledging they received the min throttle input), "
			"then unplug the battery and restart the APM.");

	while (1) {
		hal.scheduler->delay(20);
	}
}
