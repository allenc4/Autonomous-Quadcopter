#include <AP_Math.h>

#include "Config.h"


//#include <APM_PI.h>
//
//#include "Config.h"
//
//// run roll, pitch and yaw rate controllers and send output to motors
//// targets for these controllers comes from stabilize controllers
//void
//run_rate_controllers()
//{
//    // call rate controllers
//    rc_channels[RC_CHANNEL_ROLL].servo_out = get_rate_roll(roll_rate_target_bf);
//    rc_channels[RC_CHANNEL_PITCH].servo_out = get_rate_pitch(pitch_rate_target_bf);
//    rc_channels[RC_CHANNEL_YAW].servo_out = get_rate_yaw(yaw_rate_target_bf);
//    // run throttle controller if accel based throttle controller is enabled and active (active means it has been given a target)
////    if( g.throttle_accel_enabled && throttle_accel_controller_active ) {
////        set_throttle_out(get_throttle_accel(throttle_accel_target_ef), true);
////    }
//}
//
//int16_t
//get_rate_roll(int32_t target_rate)
//{
//    int32_t p,i,d;                  // used to capture pid values for logging
//    int32_t current_rate;           // this iteration's rate
//    int32_t rate_error;             // simply target_rate - current_rate
//    int32_t output;                 // output from pid controller
//
//    // get current rate
//    current_rate    = (gyroVals.x * DEGX100);
//
//    // call pid controller
//    rate_error  = target_rate - current_rate;
//    p           = g.pid_rate_roll.get_p(rate_error);
//
//    // get i term
//    i = g.pid_rate_roll.get_integrator();
//
//    // update i term as long as we haven't breached the limits or the I term will certainly reduce
////    if (!motors.reached_limit(AP_MOTOR_ROLLPITCH_LIMIT) || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
////        i = g.pid_rate_roll.get_i(rate_error, G_Dt);
////    }
//
//    d = g.pid_rate_roll.get_d(rate_error, G_Dt);
//    output = p + i + d;
//
//    // constrain output
//    output = constrain_int32(output, -5000, 5000);
//
//    // output control
//    return output;
//}
//
//int16_t
//get_rate_pitch(int32_t target_rate)
//{
//    int32_t p,i,d;                                                                      // used to capture pid values for logging
//    int32_t current_rate;                                                       // this iteration's rate
//    int32_t rate_error;                                                                 // simply target_rate - current_rate
//    int32_t output;                                                                     // output from pid controller
//
//    // get current rate
//    current_rate    = (gyroVals.y * DEGX100);
//
//    // call pid controller
//    rate_error      = target_rate - current_rate;
//    p               = g.pid_rate_pitch.get_p(rate_error);
//
//    // get i term
//    i = g.pid_rate_pitch.get_integrator();
//
//    // update i term as long as we haven't breached the limits or the I term will certainly reduce
////    if (!motors.reached_limit(AP_MOTOR_ROLLPITCH_LIMIT) || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
////        i = g.pid_rate_pitch.get_i(rate_error, G_Dt);
////    }
//
//    d = g.pid_rate_pitch.get_d(rate_error, G_Dt);
//    output = p + i + d;
//
//    // constrain output
//    output = constrain_int32(output, -5000, 5000);
//
//    // output control
//    return output;
//}
//
//int16_t
//get_rate_yaw(int32_t target_rate)
//{
//    int32_t p,i,d;                                                                      // used to capture pid values for logging
//    int32_t rate_error;
//    int32_t output;
//
//    // rate control
//    rate_error              = target_rate - (gyroVals.z * DEGX100);
//
//    // separately calculate p, i, d values for logging
//    p = g.pid_rate_yaw.get_p(rate_error);
//
//    // get i term
//    i = g.pid_rate_yaw.get_integrator();
//
//    // update i term as long as we haven't breached the limits or the I term will certainly reduce
////    if (!motors.reached_limit(AP_MOTOR_YAW_LIMIT) || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
////        i = g.pid_rate_yaw.get_i(rate_error, G_Dt);
////    }
//
//    // get d value
//    d = g.pid_rate_yaw.get_d(rate_error, G_Dt);
//
//    output  = p+i+d;
//    output = constrain_int32(output, -4500, 4500);
//
//    // output control:
//    int16_t yaw_limit = 2200 + abs(rc_channels[RC_CHANNEL_YAW].control_in);
//
//    // smoother Yaw control:
//    return constrain_int32(output, -yaw_limit, yaw_limit);
//}
//
//// set_roll_rate_target - to be called by upper controllers to set roll rate targets in the earth frame
//void set_roll_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
//    rate_targets_frame = earth_or_body_frame;
//    if( earth_or_body_frame == BODY_FRAME ) {
//        roll_rate_target_bf = desired_rate;
//    }else{
//        roll_rate_target_ef = desired_rate;
//    }
//}
//
//// set_pitch_rate_target - to be called by upper controllers to set pitch rate targets in the earth frame
//void set_pitch_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
//    rate_targets_frame = earth_or_body_frame;
//    if( earth_or_body_frame == BODY_FRAME ) {
//        pitch_rate_target_bf = desired_rate;
//    }else{
//        pitch_rate_target_ef = desired_rate;
//    }
//}
//
//// update_rate_contoller_targets - converts earth frame rates to body frame rates for rate controllers
//void
//update_rate_contoller_targets()
//{
//    if( rate_targets_frame == EARTH_FRAME ) {
//        // convert earth frame rates to body frame rates
//        roll_rate_target_bf 	= roll_rate_target_ef - sin_pitch * yaw_rate_target_ef;
//        hal.console->printf("rol_bf: %d", roll_rate_target_bf);
//        pitch_rate_target_bf 	= cos_roll_x  * pitch_rate_target_ef + sin_roll * cos_pitch_x * yaw_rate_target_ef;
//        yaw_rate_target_bf 		= cos_pitch_x * cos_roll_x * yaw_rate_target_ef - sin_roll * pitch_rate_target_ef;
//    }
//}
//
//void
//get_stabilize_roll(int32_t target_angle)
//{
//    // angle error
//    target_angle            = wrap_180_cd(target_angle - ahrs.roll_sensor);
//
//    // limit the error we're feeding to the PID
//    target_angle            = constrain_int32(target_angle, -4500, 4500);
//
//    // convert to desired rate
//    int32_t target_rate = g.pi_stabilize_roll.get_pi(target_angle, G_Dt);
//
//    // set targets for rate controller
//    set_roll_rate_target(target_rate, EARTH_FRAME);
//}
//
//void
//get_stabilize_pitch(int32_t target_angle)
//{
//    // angle error
//    target_angle            = wrap_180_cd(target_angle - ahrs.pitch_sensor);
//
//    // limit the error we're feeding to the PID
//    target_angle            = constrain_int32(target_angle, -4500, 4500);
//
//    // convert to desired rate
//    int32_t target_rate = g.pi_stabilize_pitch.get_pi(target_angle, G_Dt);
//
//    // set targets for rate controller
//    set_pitch_rate_target(target_rate, EARTH_FRAME);
//}
//
//// set_throttle_out - to be called by upper throttle controllers when they wish to provide throttle output directly to motors
//// provide 0 to cut motors
//void set_throttle_out( int16_t throttle_out)
//{
//	rc_channels[RC_CHANNEL_THROTTLE].servo_out = throttle_out;
//
//   // TODO - update compass with throttle value
////   compass.set_throttle((float)g.rc_3.servo_out/1000.0f);
//}
// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more

// get_smoothing_gain - returns smoothing gain to be passed into attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth
//      result is a number from 2 to 12 with 2 being very sluggish and 12 being very crisp
float get_smoothing_gain()
{
    return (2.0f + (float) RC_FEEL_RP_MEDIUM / 10.0f);
}

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
    hal.console->printf("RCRollCi: %d\t RollTarg: %d\t RCPitCi: %d\t PitTarg: %d\t",
    		rc_channels[RC_CHANNEL_ROLL].control_in,
			target_roll,
			rc_channels[RC_CHANNEL_PITCH].control_in,
			target_pitch);
#endif

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(
    		rc_channels[RC_CHANNEL_YAW].control_in);

#if DEBUG == ENABLED
    hal.console->printf("RCYawCi: %d\t YawTarg: %d\t",
    		rc_channels[RC_CHANNEL_YAW].control_in,
			target_yaw_rate);
#endif

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(
    		rc_channels[RC_CHANNEL_THROTTLE].control_in);

#if DEBUG == ENABLED
    hal.console->printf("RCThrotCi: %d\t ThrotTarg: %d\n",
    		rc_channels[RC_CHANNEL_THROTTLE].control_in,
			pilot_throttle_scaled);
#endif

    // call attitude controller
    attitude.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude.set_throttle_out(pilot_throttle_scaled, true);
}

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
    if (aparm.angle_max == ROLL_PITCH_INPUT_MAX) {
        roll_out = roll_in;
        pitch_out = pitch_in;
        return;
    }

    // check if angle_max has been updated and redo scaler
    if (aparm.angle_max != _angle_max) {
        _angle_max = aparm.angle_max;
        _scaler = (float)aparm.angle_max/(float)ROLL_PITCH_INPUT_MAX;
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
