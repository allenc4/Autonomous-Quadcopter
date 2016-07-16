// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <AP_Common.h>

// Global parameter class.
//
class Parameters {
public:
	static void load_parameters();
    // The version of the layout as described by the parameter enum.
    //
    // When changing the parameter enum in an incompatible fashion, this
    // value should be incremented by one.
    //
    // The increment will prevent old parameters from being used incorrectly
    // by newer code.
    //
    static const uint16_t k_format_version = 13;

	// The parameter software_type is set up solely for ground station use
	// and identifies the software type (eg ArduPilotMega versus ArduCopterMega)
	// GCS will interpret values 0-9 as ArduPilotMega.  Developers may use
	// values within that range to identify different branches.
	//
    static const uint16_t k_software_type = 9;		// 0 for APM trunk

    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
		k_param_software_type,

        // Misc
        //


		// 110: Telemetry control
		//

        // 120: Fly-by-wire control
        //

        //
        // 130: Sensor parameters
        //
		k_param_battery_monitoring,
		k_param_volt_div_ratio,
		k_param_curr_amp_per_volt,
		k_param_input_voltage,
		k_param_pack_capacity,
		k_param_ins,


        //
        // 150: Navigation parameters
        //

        //
        // 170: Radio settings
        //

        //
        // 200: Feed-forward gains
        //

        //
        // 210: flight modes
        //

        //
        // 220: Waypoint data
        //

        //
        // 240: PID Controllers
        //
        // Heading-to-roll PID:
        // heading error from command to roll command deviation from trim
        // (bank to turn strategy)
        //
        k_param_pidNavRoll = 240,

        // Roll-to-servo PID:
        // roll error from command to roll servo deviation from trim
        // (tracks commanded bank angle)
        //
        k_param_pidServoRoll,

        //
        // Pitch control
        //
        // Pitch-to-servo PID:
        // pitch error from command to pitch servo deviation from trim
        // (front-side strategy)
        //
        k_param_pidServoPitch,

        // Airspeed-to-pitch PID:
        // airspeed error from command to pitch servo deviation from trim
        // (back-side strategy)
        //
        k_param_pidNavPitchAirspeed,

        //
        // Yaw control
        //
        // Yaw-to-servo PID:
        // yaw rate error from command to yaw servo deviation from trim
        // (stabilizes dutch roll)
        //
        k_param_pidServoRudder,

        //
        // Throttle control
        //
        // Energy-to-throttle PID:
        // total energy error from command to throttle servo deviation from trim
        // (throttle back-side strategy alternative)
        //
        k_param_pidTeThrottle,

        // Altitude-to-pitch PID:
        // altitude error from command to pitch servo deviation from trim
        // (throttle front-side strategy alternative)
        //
        k_param_pidNavPitchAltitude,

        // 254,255: reserved
    };

    AP_Int16    format_version;
	AP_Int8		software_type;

	// Telemetry control
	//

    // Feed-forward gains
    //

    // Crosstrack navigation
    //

    // Estimation
    //

    // Waypoints
    //

    // Fly-by-wire
    //


    // Throttle
    //


	// Failsafe

    // Flight modes
    //


    // Navigational maneuvering limits
    //


    // Misc
    //
    AP_Int8		battery_monitoring;	// 0=disabled, 3=voltage only, 4=voltage and current
    AP_Float	volt_div_ratio;
    AP_Float	curr_amp_per_volt;
    AP_Float	input_voltage;
	AP_Int16	pack_capacity;		// Battery pack capacity less reserve
	AP_Vector3f	accel_offsets;
	AP_Vector3f	accel_scale;

};

extern const AP_Param::Info var_info[];

#endif // PARAMETERS_H
