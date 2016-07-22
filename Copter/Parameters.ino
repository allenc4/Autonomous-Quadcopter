/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  ArduPlane parameter definitions

*/

#define GSCALAR(v, name, def) { g.v.vtype, name, Parameters::k_param_ ## v, &g.v, { def_value:def } }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &g.v, { group_info: class::var_info } }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &v,  { group_info: class::var_info } }

const AP_Param::Info var_info[] PROGMEM = {
	GSCALAR(format_version,         "FORMAT_VERSION", 0),
	GSCALAR(software_type,          "SYSID_SW_TYPE",  Parameters::k_software_type),

	GSCALAR(battery_monitoring,     "BATT_MONITOR",   DISABLED),
	GSCALAR(volt_div_ratio,         "VOLT_DIVIDER",   VOLT_DIV_RATIO),
	GSCALAR(curr_amp_per_volt,      "AMP_PER_VOLT",   CURR_AMP_PER_VOLT),
	GSCALAR(input_voltage,          "INPUT_VOLTS",    INPUT_VOLTAGE),
	GSCALAR(pack_capacity,          "BATT_CAPACITY",  HIGH_DISCHARGE),

	// @Group: INS_
	// @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
	GOBJECT(ins,            "INS_", AP_InertialSensor),


	  // PID controller
	    //---------------
	    // @Param: RATE_RLL_P
	    // @DisplayName: Roll axis rate controller P gain
	    // @Description: Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output
	    // @Range: 0.08 0.20
	    // @User: Standard

	    // @Param: RATE_RLL_I
	    // @DisplayName: Roll axis rate controller I gain
	    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
	    // @Range: 0.01 0.5
	    // @User: Standard

	    // @Param: RATE_RLL_IMAX
	    // @DisplayName: Roll axis rate controller I gain maximum
	    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
	    // @Range: 0 500
	    // @Units: PWM
	    // @User: Standard

	    // @Param: RATE_RLL_D
	    // @DisplayName: Roll axis rate controller D gain
	    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
	    // @Range: 0.001 0.008
	    // @User: Standard
	    GGROUP(pid_rate_roll,     "RATE_RLL_", AC_PID),

	    // @Param: RATE_PIT_P
	    // @DisplayName: Pitch axis rate controller P gain
	    // @Description: Pitch axis rate controller P gain.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output
	    // @Range: 0.08 0.20
	    // @User: Standard

	    // @Param: RATE_PIT_I
	    // @DisplayName: Pitch axis rate controller I gain
	    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
	    // @Range: 0.01 0.5
	    // @User: Standard

	    // @Param: RATE_PIT_IMAX
	    // @DisplayName: Pitch axis rate controller I gain maximum
	    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
	    // @Range: 0 500
	    // @Units: PWM
	    // @User: Standard

	    // @Param: RATE_PIT_D
	    // @DisplayName: Pitch axis rate controller D gain
	    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
	    // @Range: 0.001 0.008
	    // @User: Standard
	    GGROUP(pid_rate_pitch,    "RATE_PIT_", AC_PID),

	    // @Param: RATE_YAW_P
	    // @DisplayName: Yaw axis rate controller P gain
	    // @Description: Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output
	    // @Range: 0.150 0.250
	    // @User: Standard

	    // @Param: RATE_YAW_I
	    // @DisplayName: Yaw axis rate controller I gain
	    // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
	    // @Range: 0.010 0.020
	    // @User: Standard

	    // @Param: RATE_YAW_IMAX
	    // @DisplayName: Yaw axis rate controller I gain maximum
	    // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
	    // @Range: 0 500
	    // @Units: PWM
	    // @User: Standard

	    // @Param: RATE_YAW_D
	    // @DisplayName: Yaw axis rate controller D gain
	    // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
	    // @Range: 0.000 0.001
	    // @User: Standard
	    GGROUP(pid_rate_yaw,      "RATE_YAW_", AC_PID),

	    // @Param: LOITER_LAT_P
	    // @DisplayName: Loiter latitude rate controller P gain
	    // @Description: Loiter latitude rate controller P gain.  Converts the difference between desired speed and actual speed into a lean angle in the latitude direction
	    // @Range: 2.000 6.000
	    // @User: Standard

	    // @Param: LOITER_LAT_I
	    // @DisplayName: Loiter latitude rate controller I gain
	    // @Description: Loiter latitude rate controller I gain.  Corrects long-term difference in desired speed and actual speed in the latitude direction
	    // @Range: 0.020 0.060
	    // @User: Standard

	    // @Param: LOITER_LAT_IMAX
	    // @DisplayName: Loiter rate controller I gain maximum
	    // @Description: Loiter rate controller I gain maximum.  Constrains the lean angle that the I gain will output
	    // @Range: 0 4500
	    // @Units: Centi-Degrees
	    // @User: Standard

	    // @Param: LOITER_LAT_D
	    // @DisplayName: Loiter latitude rate controller D gain
	    // @Description: Loiter latitude rate controller D gain.  Compensates for short-term change in desired speed vs actual speed
	    // @Range: 0.200 0.600
	    // @User: Standard
	    GGROUP(pid_loiter_rate_lat,      "LOITER_LAT_",  AC_PID),

	    // @Param: LOITER_LON_P
	    // @DisplayName: Loiter longitude rate controller P gain
	    // @Description: Loiter longitude rate controller P gain.  Converts the difference between desired speed and actual speed into a lean angle in the longitude direction
	    // @Range: 2.000 6.000
	    // @User: Standard

	    // @Param: LOITER_LON_I
	    // @DisplayName: Loiter longitude rate controller I gain
	    // @Description: Loiter longitude rate controller I gain.  Corrects long-term difference in desired speed and actual speed in the longitude direction
	    // @Range: 0.020 0.060
	    // @User: Standard

	    // @Param: LOITER_LON_IMAX
	    // @DisplayName: Loiter longitude rate controller I gain maximum
	    // @Description: Loiter longitude rate controller I gain maximum.  Constrains the lean angle that the I gain will output
	    // @Range: 0 4500
	    // @Units: Centi-Degrees
	    // @User: Standard

	    // @Param: LOITER_LON_D
	    // @DisplayName: Loiter longituderate controller D gain
	    // @Description: Loiter longitude rate controller D gain.  Compensates for short-term change in desired speed vs actual speed
	    // @Range: 0.200 0.600
	    // @User: Standard
	    GGROUP(pid_loiter_rate_lon,      "LOITER_LON_",  AC_PID),

	    // @Param: THR_RATE_P
	    // @DisplayName: Throttle rate controller P gain
	    // @Description: Throttle rate controller P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
	    // @Range: 1.000 8.000
	    // @User: Standard

	    // @Param: THR_RATE_I
	    // @DisplayName: Throttle rate controller I gain
	    // @Description: Throttle rate controller I gain.  Corrects long-term difference in desired vertical speed and actual speed
	    // @Range: 0.000 0.100
	    // @User: Standard

	    // @Param: THR_RATE_IMAX
	    // @DisplayName: Throttle rate controller I gain maximum
	    // @Description: Throttle rate controller I gain maximum.  Constrains the desired acceleration that the I gain will generate
	    // @Range: 0 500
	    // @Units: cm/s/s
	    // @User: Standard

	    // @Param: THR_RATE_D
	    // @DisplayName: Throttle rate controller D gain
	    // @Description: Throttle rate controller D gain.  Compensates for short-term change in desired vertical speed vs actual speed
	    // @Range: 0.000 0.400
	    // @User: Standard
	    GGROUP(pid_throttle,      "THR_RATE_", AC_PID),

	    // @Param: THR_ACCEL_P
	    // @DisplayName: Throttle acceleration controller P gain
	    // @Description: Throttle acceleration controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output
	    // @Range: 0.500 1.500
	    // @User: Standard

	    // @Param: THR_ACCEL_I
	    // @DisplayName: Throttle acceleration controller I gain
	    // @Description: Throttle acceleration controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration
	    // @Range: 0.000 3.000
	    // @User: Standard

	    // @Param: THR_ACCEL_IMAX
	    // @DisplayName: Throttle acceleration controller I gain maximum
	    // @Description: Throttle acceleration controller I gain maximum.  Constrains the maximum pwm that the I term will generate
	    // @Range: 0 500
	    // @Units: PWM
	    // @User: Standard

	    // @Param: THR_ACCEL_D
	    // @DisplayName: Throttle acceleration controller D gain
	    // @Description: Throttle acceleration controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration
	    // @Range: 0.000 0.400
	    // @User: Standard
	    GGROUP(pid_throttle_accel,"THR_ACCEL_", AC_PID),

	    // @Param: OF_RLL_P
	    // @DisplayName: Optical Flow based loiter controller roll axis P gain
	    // @Description: Optical Flow based loiter controller roll axis P gain.  Converts the position error from the target point to a roll angle
	    // @Range: 2.000 3.000
	    // @User: Standard

	    // @Param: OF_RLL_I
	    // @DisplayName: Optical Flow based loiter controller roll axis I gain
	    // @Description: Optical Flow based loiter controller roll axis I gain.  Corrects long-term position error by more persistently rolling left or right
	    // @Range: 0.250 0.750
	    // @User: Standard

	    // @Param: OF_RLL_IMAX
	    // @DisplayName: Optical Flow based loiter controller roll axis I gain maximum
	    // @Description: Optical Flow based loiter controller roll axis I gain maximum.  Constrains the maximum roll angle that the I term will generate
	    // @Range: 0 4500
	    // @Units: Centi-Degrees
	    // @User: Standard

	    // @Param: OF_RLL_D
	    // @DisplayName: Optical Flow based loiter controller roll axis D gain
	    // @Description: Optical Flow based loiter controller roll axis D gain.  Compensates for short-term change in speed in the roll direction
	    // @Range: 0.100 0.140
	    // @User: Standard
	    GGROUP(pid_optflow_roll,  "OF_RLL_",   AC_PID),

	    // @Param: OF_PIT_P
	    // @DisplayName: Optical Flow based loiter controller pitch axis P gain
	    // @Description: Optical Flow based loiter controller pitch axis P gain.  Converts the position error from the target point to a pitch angle
	    // @Range: 2.000 3.000
	    // @User: Standard

	    // @Param: OF_PIT_I
	    // @DisplayName: Optical Flow based loiter controller pitch axis I gain
	    // @Description: Optical Flow based loiter controller pitch axis I gain.  Corrects long-term position error by more persistently pitching left or right
	    // @Range: 0.250 0.750
	    // @User: Standard

	    // @Param: OF_PIT_IMAX
	    // @DisplayName: Optical Flow based loiter controller pitch axis I gain maximum
	    // @Description: Optical Flow based loiter controller pitch axis I gain maximum.  Constrains the maximum pitch angle that the I term will generate
	    // @Range: 0 4500
	    // @Units: Centi-Degrees
	    // @User: Standard

	    // @Param: OF_PIT_D
	    // @DisplayName: Optical Flow based loiter controller pitch axis D gain
	    // @Description: Optical Flow based loiter controller pitch axis D gain.  Compensates for short-term change in speed in the pitch direction
	    // @Range: 0.100 0.140
	    // @User: Standard
	    GGROUP(pid_optflow_pitch, "OF_PIT_",   AC_PID),

		 // P controllers
		//--------------
		// @Param: STB_RLL_P
		// @DisplayName: Roll axis stabilize controller P gain
		// @Description: Roll axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired roll angle and actual angle to a desired roll rate
		// @Range: 3.000 12.000
		// @User: Standard
		GGROUP(p_stabilize_roll,       "STB_RLL_", AC_P),

		// @Param: STB_PIT_P
		// @DisplayName: Pitch axis stabilize controller P gain
		// @Description: Pitch axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate
		// @Range: 3.000 12.000
		// @User: Standard
		GGROUP(p_stabilize_pitch,      "STB_PIT_", AC_P),

		// @Param: STB_YAW_P
		// @DisplayName: Yaw axis stabilize controller P gain
		// @Description: Yaw axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired yaw angle and actual angle to a desired yaw rate
		// @Range: 3.000 6.000
		// @User: Standard
		GGROUP(p_stabilize_yaw,        "STB_YAW_", AC_P),

	    // @Param: THR_ALT_P
	    // @DisplayName: Altitude controller P gain
	    // @Description: Altitude controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
	    // @Range: 1.000 3.000
	    // @User: Standard

	    // @Param: THR_ALT_I
	    // @DisplayName: Altitude controller I gain
	    // @Description: Altitude controller I gain.  Corrects for longer-term difference in desired altitude and actual altitude
	    // @Range: 0.000 0.100
	    // @User: Standard

	    // @Param: THR_ALT_IMAX
	    // @DisplayName: Altitude controller I gain maximum
	    // @Description: Altitude controller I gain maximum.  Constrains the maximum climb rate rate that the I term will generate
	    // @Range: 0 500
	    // @Units: cm/s
	    // @User: Standard
	    GGROUP(pi_alt_hold,     "THR_ALT_", APM_PI),

	    // @Param: HLD_LAT_P
	    // @DisplayName: Loiter latitude position controller P gain
	    // @Description: Loiter latitude position controller P gain.  Converts the distance (in the latitude direction) to the target location into a desired speed which is then passed to the loiter latitude rate controller
	    // @Range: 0.100 0.300
	    // @User: Standard

	    // @Param: HLD_LAT_I
	    // @DisplayName: Loiter latitude position controller I gain
	    // @Description: Loiter latitude position controller I gain.  Corrects for longer-term distance (in latitude) to the target location
	    // @Range: 0.000 0.100
	    // @User: Standard

	    // @Param: HLD_LAT_IMAX
	    // @DisplayName: Loiter latitude position controller I gain maximum
	    // @Description: Loiter latitude position controller I gain maximum.  Constrains the maximum desired speed that the I term will generate
	    // @Range: 0 3000
	    // @Units: cm/s
	    // @User: Standard
	    GGROUP(pi_loiter_lat,   "HLD_LAT_", APM_PI),

	    // @Param: HLD_LON_P
	    // @DisplayName: Loiter longitude position controller P gain
	    // @Description: Loiter longitude position controller P gain.  Converts the distance (in the longitude direction) to the target location into a desired speed which is then passed to the loiter longitude rate controller
	    // @Range: 0.100 0.300
	    // @User: Standard

	    // @Param: HLD_LON_I
	    // @DisplayName: Loiter longitude position controller I gain
	    // @Description: Loiter longitude position controller I gain.  Corrects for longer-term distance (in longitude direction) to the target location
	    // @Range: 0.000 0.100
	    // @User: Standard

	    // @Param: HLD_LON_IMAX
	    // @DisplayName: Loiter longitudeposition controller I gain maximum
	    // @Description: Loiter  longitudeposition controller I gain maximum.  Constrains the maximum desired speed that the I term will generate
	    // @Range: 0 3000
	    // @Units: cm/s
	    // @User: Standard
	    GGROUP(pi_loiter_lon,   "HLD_LON_", APM_PI),

		// @Param: THR_MID
		// @DisplayName: Throttle Mid Position
		// @Description: The throttle output (0 ~ 1000) when throttle stick is in mid position.
		// 		Used to scale the manual throttle so that the mid throttle stick position is close to
		// 		the throttle required to hover
		// @User: Standard
		// @Range: 300 700
		// @Units: Percent*10
		// @Increment: 1
		GSCALAR(throttle_mid,        "THR_MID",    THROTTLE_MID_DEFAULT),

		// @Param: ROLL_TRIM
		// @DisplayName: Roll Trim Position
		// @Description: The roll output when roll stick is in the center position (when stick is released).
		// @User: Standard
		// @Range: RC_ROLL_MIN - RC_ROLL_MAX
		// @Units: pwm
		// @Increment: 1
		GSCALAR(roll_trim,			"ROLL_TRIM",	(RC_ROLL_MAX + RC_ROLL_MIN)/2),

		// @Param: PITCH_TRIM
		// @DisplayName: Pitch Trim Position
		// @Description: The pitch output when pitch stick is in the center position (when stick is released).
		// @User: Standard
		// @Range: RC_PITCH_MIN - RC_PITCH_MAX
		// @Units: pwm
		// @Increment: 1
		GSCALAR(pitch_trim,			"PITCH_TRIM",	(RC_PITCH_MAX + RC_PITCH_MIN)/2),

		// @Param: THROTTLE_TRIM
		// @DisplayName: Throttle Trim Position
		// @Description: The throttle output when throttle stick is all the way down in the starting position.
		// @User: Standard
		// @Range: RC_THROTTLE_MIN - RC_THROTTLE_MAX
		// @Units: pwm
		// @Increment: 1
		GSCALAR(throttle_trim,		"THROTTLE_TRIM",	RC_THROTTLE_MIN),

		// @Param: YAW_TRIM
		// @DisplayName: Yaw Trim Position
		// @Description: The yaw output when yaw stick is in the center position (when stick is released).
		// @User: Standard
		// @Range: RC_YAW_MIN - RC_YAW_MAX
		// @Units: pwm
		// @Increment: 1
		GSCALAR(yaw_trim,			"YAW_TRIM",		(RC_YAW_MAX + RC_YAW_MIN)/2),


	AP_VAREND
};


void Parameters::load_parameters()
{
	if (!g.format_version.load() ||
	     g.format_version != Parameters::k_format_version) {

		// erase all parameters
		hal.console->printf_P(PSTR("Firmware change (%u -> %u): erasing EEPROM...\n"),
						g.format_version.get(), Parameters::k_format_version);
		AP_Param::erase_all();

		// save the current format version
		g.format_version.set_and_save(Parameters::k_format_version);
		hal.console->println_P(PSTR("done."));
    } else {
	    unsigned long before = hal.scheduler->micros();
	    // Load all auto-loaded EEPROM variables
	    AP_Param::load_all();

	    hal.console->printf_P(PSTR("load_all took %luus\n"), hal.scheduler->micros() - before);
	}
}
