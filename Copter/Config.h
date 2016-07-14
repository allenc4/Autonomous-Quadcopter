// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
#ifndef __ARDUCOPTER_CONFIG_H__
#define __ARDUCOPTER_CONFIG_H__

// Default and automatic configuration details.
#include "Defines.h"


#ifdef USE_CMAKE_APM_CONFIG
 #include "APM_Config_cmake.h"  // <== Prefer cmake config if it exists
#else
 #include "APM_Config.h" // <== THIS INCLUDE, DO NOT EDIT IT. EVER.
#endif

// Debug flag. If enabled, allows for CLI interaction with board.
// For debugging/testing only. Disable when used for flight
#define DEBUG			DISABLED
#define ESC_CALIBRATE	DISABLED
#define LIDAR_ENABLED	ENABLED

// Enable lIDAR Lite range finder
#define LIDAR			DISABLED

// Optical Flow sensor
#define OPTFLOW					DISABLED

//////////////////////////////////////////////////////////////////////////////
// HARDWARE CONFIGURATION AND CONNECTIONS
//////////////////////////////////////////////////////////////////////////////
#ifndef CONFIG_HAL_BOARD
#error CONFIG_HAL_BOARD must be defined to build ArduCopter
#endif


#ifdef HAL_SERIAL0_BAUD_DEFAULT
# define SERIAL0_BAUD HAL_SERIAL0_BAUD_DEFAULT
#endif

//////////////////////////////////////////////////////////////////////////////
// Receiver input pins (start from 0 NOT 1)
#define RC_CHANNEL_ROLL		0
#define RC_CHANNEL_PITCH	1
#define RC_CHANNEL_THROTTLE	2
#define RC_CHANNEL_YAW		3
#define RC_CHANNEL_AUX_1	4

// RC MIN and MAX values
#define RC_YAW_MIN       966
#define RC_YAW_MAX       1994
#define RC_PITCH_MIN     969
#define RC_PITCH_MAX     1987
#define RC_ROLL_MIN      969
#define RC_ROLL_MAX      1984
#define RC_THROTTLE_MIN  970
#define RC_THROTTLE_MAX  1993

// RC scaled values
#define RC_YAW_MIN_SCALED 	-150
#define RC_YAW_MAX_SCALED	 150
#define RC_ROLL_MIN_SCALED	-45
#define RC_ROLL_MAX_SCALED	 45
#define RC_PITCH_MIN_SCALED	-45
#define RC_PITCH_MAX_SCALED	 45

// Inertial sensor values for pitch, roll, and yaw need to be stable before flight.
// If the difference between the last read sensor value and current value is less than
// or equal to the min_update (in radians), the sensor is ready
#define INS_SENSOR_MIN_UPDATE_RAD    0.0001

/////////////////////////////////////////////////////////////////////
// Offsets for gyroscope and accelerometer
// INS Gyroscope
#define INS_ROLL_OFFSET 	-1.65 // -1.05  //0.00579265487498855
#define INS_PITCH_OFFSET	1.65// 1.082921172521
//#define INS_YAW_OFFSET		-0.9585314622399

// Accelerometer offset in degrees
#define ACCEL_X_OFFSET		-4.4// -0.9  // Roll
#define ACCEL_Y_OFFSET		7.3// 2.645 // Pitch
#define ACCEL_Z_OFFSET		0 	  // Yaw
//#define ACCEL_X_SCALE		1
//#define ACCEL_Y_SCALE		1 //0.99
//#define ACCEL_Z_SCALE		1

#define TRIM_PITCH				-0.02
#define TRIM_ROLL				-0.02

#define THRESHOLD_PITCH_OUTPUT		10
#define THRESHOLD_ROLL_OUTPUT		10
#define THRESHOLD_YAW_OUTPUT		10

//////////////////////////////////////////////////////////////////////////////
// Serial port speeds.
//
#define SERIAL0_BAUD                   115200
#define SERIAL1_BAUD                    57600
#define SERIAL2_BAUD                    57600

///////////////////////////////////////////////////////////////////////////////
// Motor mapping
//
#define MOTOR_FL    2  // Front left
#define MOTOR_FR    0  // Front Right
#define MOTOR_BL    1  // Back Left
#define MOTOR_BR    3  // Back Right


///////////////////////////////////////////////////////////////////////////////
// ESC update speed
//
#define RC_FAST_SPEED       490
#define RC_SLOW_SPEED		50


//////////////////////////////////////////////////////////////////////////////
// Stabilize Rate Control
//
#define PID_PITCH_RATE 			0
#define 	PITCH_RATE_P		0.7f
#define		PITCH_RATE_I		1
#define		PITCH_RATE_D		0
#define		PITCH_RATE_I_MAX	50

#define PID_ROLL_RATE 			1
#define		ROLL_RATE_P			0.7f
#define		ROLL_RATE_I			1
#define		ROLL_RATE_D			0
#define 	ROLL_RATE_I_MAX		100

#define PID_YAW_RATE 			2
#define		YAW_RATE_P			0.2f
#define		YAW_RATE_I			1
#define		YAW_RATE_D			0
#define		YAW_RATE_I_MAX		50

// Only P values needed for stability
#define PID_PITCH_STAB 			3
#define		PITCH_STAB_P		4.5f
#define PID_ROLL_STAB 			4
#define 	ROLL_STAB_P			4.5f
#define PID_YAW_STAB 			5
#define 	YAW_STAB_P			10

// Optical Flow PIDs
#define PID_OPTFLOW_PITCH		0
#define     OPTFLOW_PITCH_P		2.5f
#define     OPTFLOW_PITCH_I		0.5f
#define     OPTFLOW_PITCH_D		0.12f
#define PID_OPTFLOW_ROLL		1
#define		OPTFLOW_ROLL_P		2.5f
#define		OPTFLOW_ROLL_I		0.5f
#define		OPTFLOW_ROLL_D		0.12f
#define OPTFLOW_I_MAX			1

// GPIO LED pins
#ifndef HAL_GPIO_A_LED_PIN
 # define HAL_GPIO_A_LED_PIN        27
#endif
#ifndef HAL_GPIO_B_LED_PIN
 # define HAL_GPIO_B_LED_PIN        26
#endif
#ifndef HAL_GPIO_C_LED_PIN
 # define HAL_GPIO_C_LED_PIN        25
#endif
#ifndef HAL_GPIO_LED_ON
 # define HAL_GPIO_LED_ON           0
#endif
#ifndef HAL_GPIO_LED_OFF
 # define HAL_GPIO_LED_OFF          1
#endif

//////////////////////////////////////////////////////////////////////////////
// I2C / Lidar
#define LIDAR_ADDRESS				0x62	// I2C address of the LIDAR
#define REGISTER_MEASURE 			0x00 	// Register to write to initiate ranging
#define MEASURE_VALUE				0x04	// Value to initiate ranging
#define REGISTER_HIGH_LOW_BYTES		0x8f	// Register to get both High and Low bytes
#define RANGEFINDER_READ_TIMEOUT_MS		1000	// Time (in milliseconds) to wait for new LIDAR data before
											// triggering a watchdog timeout
#define RANGEFINDER_READ_TIMOUT_ATTEMPTS	10		// Only allow 10 attempts of consecutive read fails before throwing error

//#define ROLL_PITCH_INPUT_MAX      4500            // roll, pitch input range (45 degrees)
//#define DEFAULT_ANGLE_MAX         4500            // Maximum lean angle (45 degrees)
//#define ANGLE_RATE_MAX            18000           // default maximum rotation rate in roll/pitch axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes

//////////////////////////////////////////////////////////////////////////////
// Rate controller gains
// Control output to the motors based on the desired rotation rate from the upper
// Stabilize (angular) controller. These terms are generally related to the
// power-to-weight ratio of the copter with more powerful copters requiring lower rate PID values.
//

//#define RATE_ROLL_P        		0.150f     // The higher the P, the higher the motor response to achieve desired turn rate
//#define RATE_ROLL_I        		0.100f     // A high I term will ramp quickly to hold the desired rate,
//#define RATE_ROLL_D        		0.004f     // D is used to dampen response of the copter to accelerations toward the desired set point.
//#define RATE_ROLL_IMAX         	1000
//
//#define RATE_PITCH_P       		0.150f
//#define RATE_PITCH_I       		0.100f
//#define RATE_PITCH_D       		0.004f
//#define RATE_PITCH_IMAX        	1000
//#define RATE_YAW_P             	0.200f
//#define RATE_YAW_I             	0.020f
//#define RATE_YAW_D             	0.000f
//#define RATE_YAW_IMAX           	1000

//////////////////////////////////////////////////////////////////////////////
// Throttle control gains
//
//#define THROTTLE_CRUISE       450             // default estimate of throttle required for vehicle to maintain a hover
//
//#define THR_MID_DEFAULT       500             // Throttle output (0 ~ 1000) when throttle stick is in mid position
//
//#define THR_MIN_DEFAULT       130             // minimum throttle sent to the motors when armed and pilot throttle above zero
//#define THR_MAX_DEFAULT       1000            // maximum throttle sent to the motors
//
//#define THR_DZ_DEFAULT         100             // the deadzone above and below mid throttle while in althold or loiter
//
//#define ALT_HOLD_P            1.0f
//
//// RATE control
//#define THROTTLE_RATE_P       5.0f
//
//// Throttle Accel control
//#define THROTTLE_ACCEL_P      0.50f
//#define THROTTLE_ACCEL_I      1.00f
//#define THROTTLE_ACCEL_D      0.0f
//#define THROTTLE_ACCEL_IMAX   800
//
////////////////////////////////////////////////////////////////////////////////
//// Throttle Failsafe
////
//#define FS_THR_VALUE_DEFAULT             975
//
///*
//  build a firmware version string.
//  GIT_VERSION comes from Makefile builds
//*/
//#ifndef GIT_VERSION
//#define FIRMWARE_STRING THISFIRMWARE
//#else
//#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
//#endif

#endif // __ARDUCOPTER_CONFIG_H__
