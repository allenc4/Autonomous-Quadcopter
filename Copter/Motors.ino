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
		hal.console->printf("PWM: %d\r\n", pwm);
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
	if (rc_channels[RC_CHANNEL_THROTTLE] > RC_THROTTLE_MIN + Motors::min_throttle_offset) {
		long pitch_output = pids[PID_PITCH_RATE].get_pid(gyro_pitch - rc_channels[RC_CHANNEL_PITCH], 1);
		long roll_output  = pids[PID_ROLL_RATE].get_pid(gyro_roll - rc_channels[RC_CHANNEL_ROLL], 1);
		long yaw_output   = pids[PID_YAW_RATE].get_pid(gyro_yaw - rc_channels[RC_CHANNEL_YAW], 1);

		output(MOTOR_FL, rc_channels[RC_CHANNEL_THROTTLE] - roll_output - pitch_output);
		output(MOTOR_BL, rc_channels[RC_CHANNEL_THROTTLE] - roll_output + pitch_output);
		output(MOTOR_FR, rc_channels[RC_CHANNEL_THROTTLE] + roll_output - pitch_output);
		output(MOTOR_BR, rc_channels[RC_CHANNEL_THROTTLE] + roll_output + pitch_output);
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
	for (int i = MOTOR_NUM_START; i <= MOTOR_NUM_END; i++) {
		output(i, RC_THROTTLE_MIN + min_throttle_offset);
	}
}

void Motors::output_Zero() {
	for (int i = MOTOR_NUM_START; i <= MOTOR_NUM_END; i++) {
		output(i, 0);
	}
}


