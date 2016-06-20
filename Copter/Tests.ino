/*
 * Tests.ino
 *
 *  Created on: May 26, 2016
 *      Author: chris
 */

#include <AP_Progmem_AVR.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include "Config.h"
/**
 * Test accelerometer and gyroscope readings.
 */
void accel_Gyro_Test()
{
	Vector3f accel;
	Vector3f gyro;

	hal.console->println("Enter 'a' for accelerometer values, 'g' for gyroscope values, "
			"'b' for both accelerometer and gyroscope values, "
			"'s' for roll/pitch/yaw sensor data, or 'e' to exit.\n");

	while (1)
	{
		char input;

		if (hal.console->available())
			input = hal.console->read();

		if (input == 'e')
			break;
		else
		{
			// Clear out any existing samples from ins
			ins.update();
			// Wait until we have a sample
			while (ins.num_samples_available() == 0);

			if (input == 'a')
			{
				// Read sample from ins
				accel = ins.get_accel();

				// Display results
				hal.console->printf_P(PSTR("Accelerometer X:%4.2f \t Y:%4.2f \t Z:%4.2f\n"),
									  ToDeg(accel.x),
									  ToDeg(accel.y),
									  ToDeg(accel.z));

			} else if (input == 'g')
			{
				gyro = ins.get_gyro();

				hal.console->printf_P(PSTR("Gyroscope X:%4.2f \t Y:%4.2f \t Z:%4.2f\n"),
									  ToDeg(gyro.x),
									  ToDeg(gyro.y),
									  ToDeg(gyro.z));
			} else if (input == 'b')
			{
				accel = ins.get_accel();
				gyro  = ins.get_gyro();

				hal.console->printf_P(PSTR("Accelerometer X:%4.2f \t Y:%4.2f \t Z:%4.2f \t Gyroscope X:%4.2f \t Y:%4.2f \t Z:%4.2f\n"),
									  ToDeg(accel.x), ToDeg(accel.y), ToDeg(accel.z),
									  ToDeg(gyro.x), ToDeg(gyro.y), ToDeg(gyro.z));
			} else if (input == 's')
			{
				float sensor_roll,sensor_pitch,sensor_yaw;
				ins.quaternion.to_euler(&sensor_roll, &sensor_pitch, &sensor_yaw);
				//ins.set_accel_offsets();

				hal.console->printf_P(PSTR("Gyroscope roll:%4.1f \t pitch:%4.1f \t yaw:%4.1f\n"),
									  ToDeg(sensor_roll),
									  ToDeg(sensor_pitch),
									  ToDeg(sensor_yaw));
			}

			hal.scheduler->delay(100);
		}
	}
}

/**
 * Test individual motors from serial client.
 * Enter 'm' to test each motor. User will have option to set PWM rate to motor.
 */
void motor_Test()
{
	hal.console->println("Enter 'm' for motor tests, 'e' to exit.\n");

	int8_t motors[4] = {
			MOTOR_FR,
			MOTOR_BR,
			MOTOR_FL,
			MOTOR_BL};

	while (1)
	{
		while (hal.console->available() == 0)
		{
			hal.scheduler->delay(20);
		}

		char t = hal.console->read();
		if (t == 'm')
		{
			for (int8_t i = 0; i < 4; i++)
			{
				// Test motor i; user can choose various PWM options.
				// This is a blocking call
			    individual_Motor_Test(motors[i]);

			    // After testing that particular motor, turn it off
			    hal.rcout->write(i, 0);
			}
		}
		else if (t == 'e')
		{
			// Done with motor tests, so exit
			break;
		}
	}
}

/**
 * Test Motor motor_num. All testing performed through serial CLI
 * @param motor_num Motor number interested in testing
 */
void individual_Motor_Test(int8_t motor_num)
{
	uint16_t t_throttle = 0;
	bool breakLoop = false;

	while (1)
	{
		hal.console->printf_P(PSTR("Testing motor %d\r\nEnter 0, L, H, +, or - (or e to exit)\n"), motor_num);

		// Wait for user input
		while (hal.console->available() <= 0) {
			hal.scheduler->delay(20);
		}

		switch (hal.console->read()) {
		case '0':
			// Set the motor PWM to 0 (stop spinning)
			t_throttle = 0;
			hal.console->printf_P(PSTR("Setting motor PWM to %d\n"), t_throttle);
			hal.rcout->write(motor_num, t_throttle);
			break;
		case 'L':
			// Sets the motor PWM to the lowest possible setting: RC_THROTTLE_MIN
			t_throttle = RC_THROTTLE_MIN;
			hal.console->printf_P(PSTR("Setting motor to lowest setting (%d)\n"), t_throttle);
			hal.rcout->write(motor_num, t_throttle);
			break;
		case 'H':
			// Sets the motor PWM to the highest possible setting: RC_THROTTLE_MAX
			t_throttle = RC_THROTTLE_MAX;
			hal.console->printf_P(PSTR("Setting motor to highest setting (%d)\n"),t_throttle);
			hal.rcout->write(motor_num, RC_THROTTLE_MAX);
			break;
		case '+':
			// Increases motor speed by 100 PWM. If the speed surpasses the maximum output speed,
			// the motor will be set to RC_THROTTLE_MAX
			if (t_throttle < RC_THROTTLE_MIN)
				t_throttle = RC_THROTTLE_MIN;
			else if (t_throttle + 100 > RC_THROTTLE_MAX)
				t_throttle = RC_THROTTLE_MAX;
			else
				t_throttle += 100;

			hal.console->printf_P(PSTR("Setting motor PWM to %d\n"), t_throttle);
			hal.rcout->write(motor_num, t_throttle);
			break;
		case '-':
			// Decreases motor by 100 PWM. If the speed goes below the minimum output speed,
			// the motor will be set to RC_THROTTLE_MIN
			if (t_throttle <= RC_THROTTLE_MIN)
				t_throttle = RC_THROTTLE_MIN;
			else if (t_throttle - 100 > RC_THROTTLE_MAX)
				t_throttle = RC_THROTTLE_MAX;
			else
				t_throttle -= 100;

			hal.console->printf_P(PSTR("Setting motor PWM to %d\n"), t_throttle);
			hal.rcout->write(motor_num, t_throttle);
			break;
		case 'e':
			// Sets the motor speed to 0 (turns off motor), and exists method
			hal.rcout->write(motor_num, 0);
			breakLoop = true;
			break;
		}

		if (breakLoop == true)
			break;

		hal.scheduler->delay(20);

	}
}




