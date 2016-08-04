#include <AP_Math.h>

#include "Config.h"

// get_smoothing_gain - returns smoothing gain to be passed into attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth
//      result is a number from 2 to 12 with 2 being very sluggish and 12 being very crisp
float get_smoothing_gain()
{
    return (2.0f + (float) RC_FEEL_RP_SOFT / 10.0f);
}

/**
 * Stablilize mode... Levels quadcopter based on input from pilot RC
 */
void stabilize_run()
{
    int16_t target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;

    // if not armed or throttle at zero, set throttle to zero and exit immediately
    if(!motors.armed() || rc_channels[RC_CHANNEL_THROTTLE].control_in <= 0) {
        attitude.relax_bf_rate_controller();
        attitude.set_yaw_target_to_current_heading();
        attitude.set_throttle_out(0, false);
        return;
    }

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles(rc_channels[RC_CHANNEL_ROLL].control_in,
    		rc_channels[RC_CHANNEL_PITCH].control_in,
			target_roll,
			target_pitch);

#if DEBUG == ENABLED
    if (loop_count % 20 == 0) {
		hal.console->printf("RCRollCi: %d\t RollTarg: %d\t RCPitCi: %d\t PitTarg: %d\t",
				rc_channels[RC_CHANNEL_ROLL].control_in,
				target_roll,
				rc_channels[RC_CHANNEL_PITCH].control_in,
				target_pitch);
    }
#endif

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(
    		rc_channels[RC_CHANNEL_YAW].control_in);

//#if DEBUG == ENABLED
//    if (loop_count % 20 == 0) {
//		hal.console->printf("RCYawCi: %d\t YawTarg: %d\t",
//				rc_channels[RC_CHANNEL_YAW].control_in,
//				target_yaw_rate);
//    }
//#endif

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(
    		rc_channels[RC_CHANNEL_THROTTLE].control_in);

//#if DEBUG == ENABLED
//    if (loop_count % 20 == 0) {
//		hal.console->printf("RCThrotCi: %d\t ThrotTarg: %d\n",
//				rc_channels[RC_CHANNEL_THROTTLE].control_in,
//				pilot_throttle_scaled);
//    }
//#endif

    // call attitude controller
    attitude.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude.set_throttle_out(pilot_throttle_scaled, true);
}

/**
 * Stabilizes mode based on pilots input and optical flow sensor readings to attempt to
 * keep vehicle at a stationary pitch/roll
 */
#if LIDAR == ENABLED && OPTFLOW == ENABLED
void ofLoiter_run()
{
    int16_t target_roll, target_pitch, pilot_throttle_scaled;
    float target_yaw_rate = 0;


    // if not armed or throttle at zero, set throttle to zero and exit immediately
   if(!motors.armed() || rc_channels[RC_CHANNEL_THROTTLE].control_in <= 0) {
	   attitude.relax_bf_rate_controller();
	   attitude.set_yaw_target_to_current_heading();
	   attitude.set_throttle_out(0, false);
	   return;
   }

   // convert pilot input to lean angles
   get_pilot_desired_lean_angles(rc_channels[RC_CHANNEL_ROLL].control_in,
		rc_channels[RC_CHANNEL_PITCH].control_in,
		target_roll,
		target_pitch);

   // get pilot's desired yaw rate
   target_yaw_rate = get_pilot_desired_yaw_rate(
		rc_channels[RC_CHANNEL_YAW].control_in);

   // get pilot's desired throttle
   pilot_throttle_scaled = get_pilot_desired_throttle(
		rc_channels[RC_CHANNEL_THROTTLE].control_in);

   // Check to see if we have landed (+/- 2 cm from the initial starting position)
   bool landed = (lidar->getLastDistance() <= 2) && pilot_throttle_scaled < 100;

    // when landed reset targets and output zero throttle
    if (landed) {
        attitude.relax_bf_rate_controller();
        attitude.set_yaw_target_to_current_heading();
        // move throttle to between minimum and non-takeoff-throttle to keep us on the ground
        attitude.set_throttle_out(motors.throttle_min(), false);
        opticalFlow.reset_I();
    }else{
        // mix in user control with optical flow
        target_roll  = opticalFlow.get_of_roll(target_roll, rc_channels[RC_CHANNEL_YAW].control_in);
        target_pitch = opticalFlow.get_of_pitch(target_pitch, rc_channels[RC_CHANNEL_YAW].control_in);

#if DEBUG == ENABLED
       if(loop_count % 20 == 0)
       {

    	   hal.console->print("Target Roll: ");
    	   hal.console->print(target_roll);
    	   hal.console->print(" Target Pitch: ");
    	   hal.console->print(target_pitch);
    	   hal.console->println();
       }
#endif

        // call attitude controller
        attitude.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

//        // run altitude controller
//        if (lidar->isHealthy()) {
//            // if sonar is ok, use surface tracking
//            target_climb_rate = get_throttle_surface_tracking(target_climb_rate, pos_control.get_alt_target(), G_Dt);
//        }
//
//        // update altitude target and call position controller
//        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
//        pos_control.update_z_controller();

        // output pilot's throttle
        attitude.set_throttle_out(pilot_throttle_scaled, true);
    }
}
#endif

#if LIDAR == ENABLED
void altHold_run() {
	int16_t target_roll, target_pitch;
	float target_yaw_rate;

	// if not armed or throttle at zero, set throttle to zero and exit immediately
	if(!motors.armed() || rc_channels[RC_CHANNEL_THROTTLE].control_in <= 0) {
		attitude.relax_bf_rate_controller();
		attitude.set_yaw_target_to_current_heading();
		attitude.set_throttle_out(0, false);
		pos_control.set_alt_target_to_current_alt();
		return;
	}

	// convert pilot input to lean angles
	get_pilot_desired_lean_angles(rc_channels[RC_CHANNEL_ROLL].control_in,
			rc_channels[RC_CHANNEL_PITCH].control_in,
			target_roll,
			target_pitch);


	 // get pilot's desired yaw rate
	target_yaw_rate = get_pilot_desired_yaw_rate(
			rc_channels[RC_CHANNEL_YAW].control_in);

	 float target_alt_cm = map(rc_channels[RC_CHANNEL_THROTTLE].radio_in, RC_THROTTLE_MIN, RC_THROTTLE_MAX, 10, 150);


	// Check to see if we have landed (+/- 2 cm from the initial starting position)
    bool landed = lidar->getLastDistance() <= 2;

    if(landed && target_alt_cm > 0)
    {
//    	hal.console->println("Taking Off");

    	landed = false;
    	set_throttle_takeoff();
    } //else if (landed) {
////    	hal.console->println("Landed");
//    	// when landed reset targets and output zero throttle
//    	attitude.relax_bf_rate_controller();
//		attitude.set_yaw_target_to_current_heading();
//		// move throttle to between minimum and non-takeoff-throttle to keep us on the ground
//		attitude.set_throttle_out(motors.throttle_min(), false);
//		pos_control.set_alt_target_to_current_alt();
//		inav.set_altitude(0.0f);
//	}

    if(!landed && target_alt_cm > 0) {
//    	hal.console->println("outputting");
		// call attitude controller
	    attitude.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
	    // body-frame rate controller is run directly from 100hz loop

//	    target_climb_rate = get_throttle_surface_tracking(target_climb_rate, pos_control.get_alt_target(), G_Dt);
	   // pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);

	    //sets target alt to a distance
	    pos_control.set_alt_target_with_slew(target_alt_cm, G_Dt);

	    // call position controller (which internally calls set_throttle_out)
	    pos_control.update_z_controller();
	}
}
#endif

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
void get_pilot_desired_lean_angles(int16_t roll_in, int16_t pitch_in, int16_t &roll_out, int16_t &pitch_out)
{
    static float _scaler = 1.0;
    static int16_t _angle_max = 0;

    // range check the input
    roll_in = constrain_int16(roll_in, -ROLL_PITCH_INPUT_MAX, ROLL_PITCH_INPUT_MAX);
    pitch_in = constrain_int16(pitch_in, -ROLL_PITCH_INPUT_MAX, ROLL_PITCH_INPUT_MAX);

    // return filtered roll if no scaling required
    if (vechicleType.angle_max == ROLL_PITCH_INPUT_MAX) {
        roll_out = roll_in;
        pitch_out = pitch_in;
        return;
    }

    // check if angle_max has been updated and redo scaler
    if (vechicleType.angle_max != _angle_max) {
        _angle_max = vechicleType.angle_max;
        _scaler = (float)vechicleType.angle_max/(float)ROLL_PITCH_INPUT_MAX;
    }

    // convert pilot input to lean angle
    roll_out = (int16_t)((float)roll_in * _scaler);
    pitch_out = (int16_t)((float)pitch_in * _scaler);
}

// get_pilot_desired_heading - transform pilot's yaw input into a desired heading
// returns desired angle in centi-degrees
float get_pilot_desired_yaw_rate(int16_t stick_angle)
{
    // convert pilot input to the desired yaw rate
    return stick_angle * ACRO_YAW_P;
}

// get_pilot_desired_throttle - transform pilot's throttle input to make cruise throttle mid stick
// used only for manual throttle modes
// returns throttle output 0 to 1000
#define THROTTLE_IN_MIDDLE 500          // the throttle mid point
int16_t get_pilot_desired_throttle(int16_t throttle_control)
{
    int16_t throttle_out;

    // exit immediately in the simple cases
    if( throttle_control == 0 || g.throttle_mid == 500) {
        return throttle_control;
    }

    // ensure reasonable throttle values
    throttle_control = constrain_int16(throttle_control,0,1000);
    g.throttle_mid = constrain_int16(g.throttle_mid,300,700);

    // check throttle is above, below or in the deadband
    if (throttle_control < THROTTLE_IN_MIDDLE) {
        // below the deadband
        throttle_out = THROTTLE_MIN_DEFAULT + ((float)(throttle_control-THROTTLE_MIN_DEFAULT))*((float)(g.throttle_mid - THROTTLE_MIN_DEFAULT))/((float)(500-THROTTLE_MIN_DEFAULT));
    }else if(throttle_control > THROTTLE_IN_MIDDLE) {
        // above the deadband
        throttle_out = g.throttle_mid + ((float)(throttle_control-500))*(float)(1000-g.throttle_mid)/500.0f;
    }else{
        // must be in the deadband
        throttle_out = g.throttle_mid;
    }

    return throttle_out;
}

// get_pilot_desired_climb_rate - transform pilot's throttle input to
// climb rate in cm/s.  we use radio_in instead of control_in to get the full range
// without any deadzone at the bottom
#define THROTTLE_IN_DEADBAND_TOP (THROTTLE_IN_MIDDLE+25)  // top of the deadband
#define THROTTLE_IN_DEADBAND_BOTTOM (THROTTLE_IN_MIDDLE-25)  // bottom of the deadband
int16_t get_pilot_desired_climb_rate(int16_t throttle_control)
{
    int16_t desired_rate = 0;

    // ensure a reasonable throttle value
    throttle_control = constrain_int16(throttle_control,0,1000);

    // ensure a reasonable deadzone
    int32_t throttle_deadzone = 25;
    int32_t pilot_velocity_z_max = 250;

    // check throttle is above, below or in the deadband
    if (throttle_control < THROTTLE_IN_DEADBAND_BOTTOM) {
        // below the deadband
        desired_rate = (int32_t) pilot_velocity_z_max * (throttle_control-THROTTLE_IN_DEADBAND_BOTTOM) / (THROTTLE_IN_MIDDLE - throttle_deadzone);
    }else if (throttle_control > THROTTLE_IN_DEADBAND_TOP) {
        // above the deadband
        desired_rate = (int32_t)pilot_velocity_z_max * (throttle_control-THROTTLE_IN_DEADBAND_TOP) / (THROTTLE_IN_MIDDLE - throttle_deadzone);
    }else{
        // must be in the deadband
        desired_rate = 0;
    }

    return desired_rate;
}

#if LIDAR == ENABLED
// get_throttle_surface_tracking - hold copter at the desired distance above the ground
//      returns climb rate (in cm/s) which should be passed to the position controller
float get_throttle_surface_tracking(int16_t target_rate, float current_alt_target, float dt)
{
    static uint32_t last_call_ms = 0;
    float distance_error;
    float velocity_correction;


    // adjust sonar target alt if motors have not hit their limits
    if ((target_rate<0 && !motors.limit.throttle_lower) || (target_rate>0 && !motors.limit.throttle_upper)) {
        target_lidar_alt += target_rate * dt;
    }

    // do not let target altitude get too far from current altitude above ground
    // Note: the 750cm limit is perhaps too wide but is consistent with the regular althold limits and helps ensure a smooth transition
//    target_lidar_alt = constrain_float(target_lidar_alt,(float)lidar->getLastDistance()-,(float)lidar->getLastDistance()-altHold.getZLeashLength());

    // calc desired velocity correction from target sonar alt vs actual sonar alt (remove the error already passed to Altitude controller to avoid oscillations)
    distance_error = (target_lidar_alt - lidar->getLastDistance());
    velocity_correction = distance_error * 0.8;
    velocity_correction = constrain_float(velocity_correction, -150, 150);

    // return combined pilot climb rate + rate to correct sonar alt error
    return (target_rate + velocity_correction);
}

void set_throttle_takeoff()
{
   // tell position controller to reset alt target and reset I terms
   pos_control.init_takeoff();

   // tell motors to do a slow start
   motors.slow_start(true);
}

#endif

void correct_yaw() {

	float sensor_yaw = ahrs.yaw_sensor/100.0f;
	float desiredYaw = wrap_180_cd(target_yaw - sensor_yaw);

	desiredYaw = constrain_float(desiredYaw, -4500.0f, 4500.0f);

	// Stability PIDS
	float stab_output_yaw = constrain_float(
		g.p_stabilize_yaw.get_p(desiredYaw),
		-360,
		360);

	// If controller asks for yaw change, overwrite stab_output for the yaw value
	// Yaw value will be between -150 and 150 so if there is a radio value greater than a 5
	// degree offset, rotate
	if (abs(rc_channels[RC_CHANNEL_YAW].control_in) > 100) {
		hal.console->printf("Resetting yaw. CI value: %d\n", rc_channels[RC_CHANNEL_YAW].control_in);
		stab_output_yaw = rc_channels[RC_CHANNEL_YAW].control_in;
		target_yaw = sensor_yaw; // remember for when radio stops
	}

	// Acrobatic/rate PIDs
	long yaw_output = constrain_int16(
		g.pid_rate_yaw.get_pid((stab_output_yaw * ACRO_YAW_RATE) - ahrs.get_gyro().z, 1),
			-500,
			500);

	if (loop_count % 20 == 0)
		hal.console->printf("Setting yaw to: %ld\n", yaw_output);
	motors.set_yaw(yaw_output);
}
