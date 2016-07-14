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

void lidarTest() {
#if LIDAR == ENABLED
	uint16_t lidarDistCm = lidar->getLastDistance();

	if (!lidar->isHealthy()) {
		// Failsafe here...
		a_led->write(0);
		b_led->write(1);
		c_led->write(1);

		hal.console->println("LIDAR is not healthy.");
	} else if (lidarDistCm <= 0) {
		// Reading didn't take so disregard
		return;
	}

	hal.console->printf("%d\n", lidarDistCm);

#endif


//	hal.scheduler->m
//
//
//	uint8_t ack = 100;  // Setup variable to hold ACK responses
//	uint8_t distanceArr[2];
//	AP_HAL::Semaphore *i2c_sem = hal.i2c->get_semaphore();
//
//	// Wait to get the semaphore
//	int wait_count = 0;
//	bool has_semaphore = false;
//	while ((has_semaphore = i2c_sem->take(1)) == false && wait_count < 5) {
//		wait_count ++;
//		hal.scheduler->delay(20);
//	}
//
//	if (!has_semaphore) {
//		hal.console->println("Couldn't get semaphore lock for i2c bus");
//		i2c_sem->give();
//		return;
//	} else {
//		hal.console->println("Got semaphore...");
//	}
//
//
//	// Write 0x04 to register 0x00
//	while (ack != 0) {
//		ack = hal.i2c->writeRegister((uint8_t)LIDAR_ADDRESS, REGISTER_MEASURE, MEASURE_VALUE);
//		hal.scheduler->delay(1);
//
//		hal.console->printf("ack: %d\n", ack);
//	}
//	hal.console->println("Two...");
//
//
//	ack = 100;
//	// While ACK is received
//	while (ack != 0) {
//		ack = hal.i2c->readRegisters(LIDAR_ADDRESS, REGISTER_HIGH_LOW_BYTES, 2, distanceArr);
//		hal.scheduler->delay(1);  // Prevent over-sampling
//	}
//
//	hal.console->println("Three...");
//
//	i2c_sem->give();
//
//	// Shift high byte[0] 8 to the left and add low byte[1] to create 16-bit integer
//	int distance = (distanceArr[0] << 8);
//	distance |= distanceArr[1];
//
//	if (loop_count == 20) {
//		hal.console->printf("Distance read %d\r\n", distance);
//		loop_count = 0;
//	}
//	loop_count++;
//
//	hal.scheduler->delay(50);

}



