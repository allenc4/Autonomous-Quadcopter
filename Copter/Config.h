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


#define HAL_STORAGE_SIZE            16384
// Debug flag. If enabled, allows for CLI interaction with board.
// For debugging/testing only. Disable when used for flight
#define DEBUG			DISABLED
#define ESC_CALIBRATE	DISABLED
#define ACCEL_CALIBRATE DISABLED

// Enable lIDAR Lite range finder
#define LIDAR			DISABLED

// Optical Flow sensor
#define OPTFLOW					DISABLED

//modes
//the time needed to hold controller in certain position to
//change the mode
#define MODE_SELECT_TIME		5000
#define NO_MODE					0
#define ACCEL_CALIBRATE_MODE 	1

//////////////////////////////////////////////////////////////////////////////
// HARDWARE CONFIGURATION AND CONNECTIONS
//////////////////////////////////////////////////////////////////////////////
#ifndef CONFIG_HAL_BOARD
#error CONFIG_HAL_BOARD //must be defined to build ArduCopter
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
#define INS_ROLL_OFFSET 	0 //-1.71 // -1.05  //0.00579265487498855
#define INS_PITCH_OFFSET	0 //1.65// 1.082921172521
//#define INS_YAW_OFFSET		-0.9585314622399

// Accelerometer offset in degrees
#define ACCEL_X_OFFSET		0 //-7.75 //-4.4// -0.9  // Roll
#define ACCEL_Y_OFFSET		0 //10.03 //7.3// 2.645 // Pitch
#define ACCEL_Z_OFFSET		0 	  // Yaw
//#define ACCEL_X_SCALE		1
//#define ACCEL_Y_SCALE		1 //0.99
//#define ACCEL_Z_SCALE		1

#define TRIM_PITCH				-0.02
#define TRIM_ROLL				-0.02

#define THRESHOLD_PITCH_OUTPUT		0
#define THRESHOLD_ROLL_OUTPUT		0
#define THRESHOLD_YAW_OUTPUT		0

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

//increases or decreases the speed of rotation on each axis
//should be a value between 1 and 10
#define  ACRO_PITCH_RATE 1
#define  ACRO_ROLL_RATE 1
#define  ACRO_YAW_RATE 1


//this is the degree of the motor + 90 and then some trig
#define MOTOR_FL_ROLL_FACTOR 	 0.707
#define MOTOR_FL_PITCH_FACTOR 	 0.707
#define MOTOR_FL_YAW_FACTOR 	 1

#define MOTOR_BL_ROLL_FACTOR 	 0.707
#define MOTOR_BL_PITCH_FACTOR 	-0.707
#define MOTOR_BL_YAW_FACTOR 	 -1

#define MOTOR_FR_ROLL_FACTOR 	-0.707
#define MOTOR_FR_PITCH_FACTOR 	 0.707
#define MOTOR_FR_YAW_FACTOR 	 -1

#define MOTOR_BR_ROLL_FACTOR 	-0.707
#define MOTOR_BR_PITCH_FACTOR 	-0.707
#define MOTOR_BR_YAW_FACTOR 	 1


//////////////////////////////////////////////////////////////////////////////
// Stabilize Rate Control
//
#define MASTER_P				2.5f //0.15f - ardu default
#define	MASTER_I				1.0f//0.1f
#define MASTER_D				0.0f//0.004f
#define MASTER_IMAX				100

#define PID_PITCH_RATE 			0
#define 	PITCH_RATE_P		MASTER_P
#define		PITCH_RATE_I		MASTER_I
#define		PITCH_RATE_D		MASTER_D
#define		PITCH_RATE_I_MAX	MASTER_IMAX

#define PID_ROLL_RATE 			1
#define		ROLL_RATE_P			MASTER_P
#define		ROLL_RATE_I			MASTER_I
#define		ROLL_RATE_D			MASTER_D
#define 	ROLL_RATE_I_MAX		MASTER_IMAX

#define PID_YAW_RATE 			2
#define		YAW_RATE_P			1.5f
#define		YAW_RATE_I			0.0f
#define		YAW_RATE_D			0.0f
#define		YAW_RATE_I_MAX		0.0f

#define MASTER_STAB_P			4.5f
// Only P values needed for stability
#define PID_PITCH_STAB 			3
#define		PITCH_STAB_P		MASTER_STAB_P
#define PID_ROLL_STAB 			4
#define 	ROLL_STAB_P			MASTER_STAB_P
#define PID_YAW_STAB 			5
#define 	YAW_STAB_P			MASTER_STAB_P

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


//THESE VALUES ARE JUST SO THAT THE Parameters.ino File will build we can sort through them later
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//
// WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING
//
//  DO NOT EDIT this file to adjust your configuration.  Create your own
//  APM_Config.h and use APM_Config.h.example as a reference.
//
// WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING
///
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//
// Default and automatic configuration details.
//
// Notes for maintainers:
//
// - Try to keep this file organised in the same order as APM_Config.h.example
//

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// HARDWARE CONFIGURATION AND CONNECTIONS
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// Battery monitoring
//
#ifndef BATTERY_EVENT
# define BATTERY_EVENT			DISABLED
#endif
#ifndef LOW_VOLTAGE
# define LOW_VOLTAGE			9.6
#endif
#ifndef VOLT_DIV_RATIO
# define VOLT_DIV_RATIO			3.56	// This is the proper value for an on-board APM1 voltage divider with a 3.9kOhm resistor
//# define VOLT_DIV_RATIO		15.70	// This is the proper value for the AttoPilot 50V/90A sensor
//# define VOLT_DIV_RATIO		4.127	// This is the proper value for the AttoPilot 13.6V/45A sensor

#endif

#ifndef CURR_AMP_PER_VOLT
# define CURR_AMP_PER_VOLT		27.32	// This is the proper value for the AttoPilot 50V/90A sensor
//# define CURR_AMP_PER_VOLT	13.66	// This is the proper value for the AttoPilot 13.6V/45A sensor
#endif

#ifndef CURR_AMPS_OFFSET
# define CURR_AMPS_OFFSET		0.0
#endif
#ifndef HIGH_DISCHARGE
# define HIGH_DISCHARGE		1760
#endif

//////////////////////////////////////////////////////////////////////////////
// INPUT_VOLTAGE
//
#ifndef INPUT_VOLTAGE
# define INPUT_VOLTAGE			4.68	//  4.68 is the average value for a sample set.  This is the value at the processor with 5.02 applied at the servo rail
#endif


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// RADIO CONFIGURATION
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// STARTUP BEHAVIOUR
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// FLIGHT AND NAVIGATION CONTROL
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Attitude control gains
//
#ifndef SERVO_ROLL_P
# define SERVO_ROLL_P         0.4
#endif
#ifndef SERVO_ROLL_I
# define SERVO_ROLL_I         0.0
#endif
#ifndef SERVO_ROLL_D
# define SERVO_ROLL_D         0.0
#endif
#ifndef SERVO_ROLL_INT_MAX
# define SERVO_ROLL_INT_MAX   5
#endif
#define SERVO_ROLL_INT_MAX_CENTIDEGREE SERVO_ROLL_INT_MAX*100
#ifndef ROLL_SLEW_LIMIT
# define ROLL_SLEW_LIMIT      0
#endif
#ifndef SERVO_PITCH_P
# define SERVO_PITCH_P        0.6
#endif
#ifndef SERVO_PITCH_I
# define SERVO_PITCH_I        0.0
#endif
#ifndef SERVO_PITCH_D
# define SERVO_PITCH_D        0.0
#endif
#ifndef SERVO_PITCH_INT_MAX
# define SERVO_PITCH_INT_MAX  5
#endif
#define SERVO_PITCH_INT_MAX_CENTIDEGREE SERVO_PITCH_INT_MAX*100
#ifndef PITCH_COMP
# define PITCH_COMP           0.2
#endif
#ifndef SERVO_YAW_P
# define SERVO_YAW_P          0.0
#endif
#ifndef SERVO_YAW_I
# define SERVO_YAW_I          0.0
#endif
#ifndef SERVO_YAW_D
# define SERVO_YAW_D          0.0
#endif
#ifndef SERVO_YAW_INT_MAX
# define SERVO_YAW_INT_MAX    0
#endif
#ifndef RUDDER_MIX
# define RUDDER_MIX           0.5
#endif


//////////////////////////////////////////////////////////////////////////////
// Navigation control gains
//
#ifndef NAV_ROLL_P
# define NAV_ROLL_P           0.7
#endif
#ifndef NAV_ROLL_I
# define NAV_ROLL_I           0.0
#endif
#ifndef NAV_ROLL_D
# define NAV_ROLL_D           0.02
#endif
#ifndef NAV_ROLL_INT_MAX
# define NAV_ROLL_INT_MAX     5
#endif
#define NAV_ROLL_INT_MAX_CENTIDEGREE NAV_ROLL_INT_MAX*100
#ifndef NAV_PITCH_ASP_P
# define NAV_PITCH_ASP_P      0.65
#endif
#ifndef NAV_PITCH_ASP_I
# define NAV_PITCH_ASP_I      0.0
#endif
#ifndef NAV_PITCH_ASP_D
# define NAV_PITCH_ASP_D      0.0
#endif
#ifndef NAV_PITCH_ASP_INT_MAX
# define NAV_PITCH_ASP_INT_MAX 5
#endif
#define NAV_PITCH_ASP_INT_MAX_CMSEC NAV_PITCH_ASP_INT_MAX*100
#ifndef NAV_PITCH_ALT_P
# define NAV_PITCH_ALT_P      0.65
#endif
#ifndef NAV_PITCH_ALT_I
# define NAV_PITCH_ALT_I      0.0
#endif
#ifndef NAV_PITCH_ALT_D
# define NAV_PITCH_ALT_D      0.0
#endif
#ifndef NAV_PITCH_ALT_INT_MAX
# define NAV_PITCH_ALT_INT_MAX 5
#endif
#define NAV_PITCH_ALT_INT_MAX_CM NAV_PITCH_ALT_INT_MAX*100


//////////////////////////////////////////////////////////////////////////////
// Energy/Altitude control gains
//
#ifndef THROTTLE_TE_P
# define THROTTLE_TE_P        0.50
#endif
#ifndef THROTTLE_TE_I
# define THROTTLE_TE_I        0.0
#endif
#ifndef THROTTLE_TE_D
# define THROTTLE_TE_D        0.0
#endif
#ifndef THROTTLE_TE_INT_MAX
# define THROTTLE_TE_INT_MAX  20
#endif
#ifndef THROTTLE_SLEW_LIMIT
# define THROTTLE_SLEW_LIMIT  0
#endif
#ifndef P_TO_T
# define P_TO_T               0
#endif
#ifndef T_TO_P
# define T_TO_P               0
#endif
#ifndef PITCH_TARGET
# define PITCH_TARGET         0
#endif

//////////////////////////////////////////////////////////////////////////////
// Crosstrack compensation
//
#ifndef XTRACK_GAIN
# define XTRACK_GAIN          1 // deg/m
#endif
#ifndef XTRACK_ENTRY_ANGLE
# define XTRACK_ENTRY_ANGLE   30 // deg
#endif
# define XTRACK_GAIN_SCALED XTRACK_GAIN*100
# define XTRACK_ENTRY_ANGLE_CENTIDEGREE XTRACK_ENTRY_ANGLE*100

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// DEBUGGING
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


#endif // __ARDUCOPTER_CONFIG_H__
