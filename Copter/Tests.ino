/*
 * Tests.ino
 *
 *  Created on: May 26, 2016
 *      Author: chris
 */

#include "Tests.h"

/**
 * Test accelerometer and gyroscope readings.
 */
void Tests::accel_Gyro_Test()
{
	// TODO - Implement Accel_Gyro_Test() to work with external accel/gyro chip
}

/**
 * Test individual motors from serial client.
 * Enter 'm' to test each motor. User will have option to set PWM rate to motor.
 */
void Tests::motor_Test()
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
void Tests::individual_Motor_Test(int8_t motor_num)
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




