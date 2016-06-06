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
#define DEBUG_ENABLED	ENABLED
//#define ESC_CALIBRATE	DISABLED


//////////////////////////////////////////////////////////////////////////////
// HARDWARE CONFIGURATION AND CONNECTIONS
//////////////////////////////////////////////////////////////////////////////
#ifndef CONFIG_HAL_BOARD
#error CONFIG_HAL_BOARD must be defined to build ArduCopter
#endif
//////////////////////////////////////////////////////////////////////////////
// sensor types

#define CONFIG_INS_TYPE HAL_INS_DEFAULT
#define CONFIG_BARO     HAL_BARO_DEFAULT
#define CONFIG_COMPASS  HAL_COMPASS_DEFAULT

#ifdef HAL_SERIAL0_BAUD_DEFAULT
# define SERIAL0_BAUD HAL_SERIAL0_BAUD_DEFAULT
#endif


//#define MAGNETOMETER ENABLED

//#if HAL_CPU_CLASS < HAL_CPU_CLASS_75 || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
// // low power CPUs (APM1, APM2 and SITL)
// # define MAIN_LOOP_RATE    100
// # define MAIN_LOOP_SECONDS 0.01
// # define MAIN_LOOP_MICROS  10000
//#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
// // Linux boards
// # define MAIN_LOOP_RATE    200
// # define MAIN_LOOP_SECONDS 0.005
// # define MAIN_LOOP_MICROS  5000
//#else
// // high power CPUs (Flymaple, PX4, Pixhawk, VRBrain)
// # define MAIN_LOOP_RATE    400
// # define MAIN_LOOP_SECONDS 0.0025
// # define MAIN_LOOP_MICROS  2500
//#endif

//////////////////////////////////////////////////////////////////////////////
// FRAME_CONFIG
//
//#define FRAME_CONFIG   QUAD_FRAME

//////////////////////////////////////////////////////////////////////////////
// Receiver input pins (start from 0 NOT 1)
#define RC_CHANNEL_ROLL		0
#define RC_CHANNEL_PITCH	1
#define RC_CHANNEL_THROTTLE	2
#define RC_CHANNEL_YAW		3
#define RC_CHANNEL_AUX_1	4

//////////////////////////////////////////////////////////////////////////////
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

//////////////////////////////////////////////////////////////////////////////
// Serial port speeds.
//
#define SERIAL0_BAUD                   115200
#define SERIAL1_BAUD                    57600
#define SERIAL2_BAUD                    57600

///////////////////////////////////////////////////////////////////////////////
// Motor mapping
#define MOTOR_FL    2  // Front left
#define MOTOR_FR    0  // Front Right
#define MOTOR_BL    1  // Back Left
#define MOTOR_BR    3  // Back Right

#define MOTOR_NUM_START  0  // Lowest number used by motor map
#define MOTOR_NUM_END    3  // Highest number used by motor map

// ESC update speed
#define RC_FAST_SPEED       490
#define RC_SLOW_SPEED		50


//////////////////////////////////////////////////////////////////////////////
// Stabilize Rate Control
//
#define PID_PITCH_RATE 0
#define PID_ROLL_RATE 1
#define PID_PITCH_STAB 2
#define PID_ROLL_STAB 3
#define PID_YAW_RATE 4
#define PID_YAW_STAB 5

//#define ROLL_PITCH_INPUT_MAX      4500            // roll, pitch input range (45 degrees)
//#define DEFAULT_ANGLE_MAX         4500            // Maximum lean angle (45 degrees)
//#define ANGLE_RATE_MAX            18000           // default maximum rotation rate in roll/pitch axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes

//////////////////////////////////////////////////////////////////////////////
// Rate controller gains
// Control output to the motors based on the desired rotation rate from the upper
// Stabilize (angular) controller. These terms are generally related to the
// power-to-weight ratio of the copter with more powerful copters requiring lower rate PID values.
//

#define RATE_ROLL_P        		0.150f     // The higher the P, the higher the motor response to achieve desired turn rate
#define RATE_ROLL_I        		0.100f     // A high I term will ramp quickly to hold the desired rate,
#define RATE_ROLL_D        		0.004f     // D is used to dampen response of the copter to accelerations toward the desired set point.
#define RATE_ROLL_IMAX         	1000

#define RATE_PITCH_P       		0.150f
#define RATE_PITCH_I       		0.100f
#define RATE_PITCH_D       		0.004f
#define RATE_PITCH_IMAX        	1000
#define RATE_YAW_P             	0.200f
#define RATE_YAW_I             	0.020f
#define RATE_YAW_D             	0.000f
#define RATE_YAW_IMAX           	1000

//////////////////////////////////////////////////////////////////////////////
// Throttle control gains
//
#define THROTTLE_CRUISE       450             // default estimate of throttle required for vehicle to maintain a hover

#define THR_MID_DEFAULT       500             // Throttle output (0 ~ 1000) when throttle stick is in mid position

#define THR_MIN_DEFAULT       130             // minimum throttle sent to the motors when armed and pilot throttle above zero
#define THR_MAX_DEFAULT       1000            // maximum throttle sent to the motors

#define THR_DZ_DEFAULT         100             // the deadzone above and below mid throttle while in althold or loiter

#define ALT_HOLD_P            1.0f

// RATE control
#define THROTTLE_RATE_P       5.0f

// Throttle Accel control
#define THROTTLE_ACCEL_P      0.50f
#define THROTTLE_ACCEL_I      1.00f
#define THROTTLE_ACCEL_D      0.0f
#define THROTTLE_ACCEL_IMAX   800

//////////////////////////////////////////////////////////////////////////////
// Throttle Failsafe
//
#define FS_THR_VALUE_DEFAULT             975

/*
  build a firmware version string.
  GIT_VERSION comes from Makefile builds
*/
#ifndef GIT_VERSION
#define FIRMWARE_STRING THISFIRMWARE
#else
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
#endif

#endif // __ARDUCOPTER_CONFIG_H__
