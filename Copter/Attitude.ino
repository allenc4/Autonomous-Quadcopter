#include <APM_PI.h>

#include "Config.h"

// run roll, pitch and yaw rate controllers and send output to motors
// targets for these controllers comes from stabilize controllers
void
run_rate_controllers()
{
    // call rate controllers
    rc_channels[RC_CHANNEL_ROLL].servo_out = get_rate_roll(roll_rate_target_bf);
    rc_channels[RC_CHANNEL_PITCH].servo_out = get_rate_pitch(pitch_rate_target_bf);
    rc_channels[RC_CHANNEL_YAW].servo_out = get_rate_yaw(yaw_rate_target_bf);
    // run throttle controller if accel based throttle controller is enabled and active (active means it has been given a target)
//    if( g.throttle_accel_enabled && throttle_accel_controller_active ) {
//        set_throttle_out(get_throttle_accel(throttle_accel_target_ef), true);
//    }
}

int16_t
get_rate_roll(int32_t target_rate)
{
    int32_t p,i,d;                  // used to capture pid values for logging
    int32_t current_rate;           // this iteration's rate
    int32_t rate_error;             // simply target_rate - current_rate
    int32_t output;                 // output from pid controller

    // get current rate
    current_rate    = (gyroVals.x * DEGX100);

    // call pid controller
    rate_error  = target_rate - current_rate;
    p           = g.pid_rate_roll.get_p(rate_error);

    // get i term
    i = g.pid_rate_roll.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
//    if (!motors.reached_limit(AP_MOTOR_ROLLPITCH_LIMIT) || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
//        i = g.pid_rate_roll.get_i(rate_error, G_Dt);
//    }

    d = g.pid_rate_roll.get_d(rate_error, G_Dt);
    output = p + i + d;

    // constrain output
    output = constrain_int32(output, -5000, 5000);

    // output control
    return output;
}

int16_t
get_rate_pitch(int32_t target_rate)
{
    int32_t p,i,d;                                                                      // used to capture pid values for logging
    int32_t current_rate;                                                       // this iteration's rate
    int32_t rate_error;                                                                 // simply target_rate - current_rate
    int32_t output;                                                                     // output from pid controller

    // get current rate
    current_rate    = (gyroVals.y * DEGX100);

    // call pid controller
    rate_error      = target_rate - current_rate;
    p               = g.pid_rate_pitch.get_p(rate_error);

    // get i term
    i = g.pid_rate_pitch.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
//    if (!motors.reached_limit(AP_MOTOR_ROLLPITCH_LIMIT) || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
//        i = g.pid_rate_pitch.get_i(rate_error, G_Dt);
//    }

    d = g.pid_rate_pitch.get_d(rate_error, G_Dt);
    output = p + i + d;

    // constrain output
    output = constrain_int32(output, -5000, 5000);

    // output control
    return output;
}

int16_t
get_rate_yaw(int32_t target_rate)
{
    int32_t p,i,d;                                                                      // used to capture pid values for logging
    int32_t rate_error;
    int32_t output;

    // rate control
    rate_error              = target_rate - (gyroVals.z * DEGX100);

    // separately calculate p, i, d values for logging
    p = g.pid_rate_yaw.get_p(rate_error);

    // get i term
    i = g.pid_rate_yaw.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
//    if (!motors.reached_limit(AP_MOTOR_YAW_LIMIT) || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
//        i = g.pid_rate_yaw.get_i(rate_error, G_Dt);
//    }

    // get d value
    d = g.pid_rate_yaw.get_d(rate_error, G_Dt);

    output  = p+i+d;
    output = constrain_int32(output, -4500, 4500);

    // output control:
    int16_t yaw_limit = 2200 + abs(rc_channels[RC_CHANNEL_YAW].control_in);

    // smoother Yaw control:
    return constrain_int32(output, -yaw_limit, yaw_limit);
}

// set_roll_rate_target - to be called by upper controllers to set roll rate targets in the earth frame
void set_roll_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        roll_rate_target_bf = desired_rate;
    }else{
        roll_rate_target_ef = desired_rate;
    }
}

// set_pitch_rate_target - to be called by upper controllers to set pitch rate targets in the earth frame
void set_pitch_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        pitch_rate_target_bf = desired_rate;
    }else{
        pitch_rate_target_ef = desired_rate;
    }
}

// update_rate_contoller_targets - converts earth frame rates to body frame rates for rate controllers
void
update_rate_contoller_targets()
{
    if( rate_targets_frame == EARTH_FRAME ) {
        // convert earth frame rates to body frame rates
        roll_rate_target_bf 	= roll_rate_target_ef - sin_pitch * yaw_rate_target_ef;
        hal.console->printf("rol_bf: %d", roll_rate_target_bf);
        pitch_rate_target_bf 	= cos_roll_x  * pitch_rate_target_ef + sin_roll * cos_pitch_x * yaw_rate_target_ef;
        yaw_rate_target_bf 		= cos_pitch_x * cos_roll_x * yaw_rate_target_ef - sin_roll * pitch_rate_target_ef;
    }
}

void
get_stabilize_roll(int32_t target_angle, int32_t sensor_roll)
{
    // angle error
    target_angle            = wrap_180_cd(target_angle - sensor_roll);

    // limit the error we're feeding to the PID
    target_angle            = constrain_int32(target_angle, -4500, 4500);

    // convert to desired rate
    int32_t target_rate = g.pi_stabilize_roll.get_pi(target_angle, G_Dt);

    // set targets for rate controller
    set_roll_rate_target(target_rate, EARTH_FRAME);
}

void
get_stabilize_pitch(int32_t target_angle, int32_t sensor_pitch)
{
    // angle error
    target_angle            = wrap_180_cd(target_angle - sensor_pitch);

    // limit the error we're feeding to the PID
    target_angle            = constrain_int32(target_angle, -4500, 4500);

    // convert to desired rate
    int32_t target_rate = g.pi_stabilize_pitch.get_pi(target_angle, G_Dt);

    // set targets for rate controller
    set_pitch_rate_target(target_rate, EARTH_FRAME);
}

// set_throttle_out - to be called by upper throttle controllers when they wish to provide throttle output directly to motors
// provide 0 to cut motors
void set_throttle_out( int16_t throttle_out)
{
	rc_channels[RC_CHANNEL_THROTTLE].servo_out = throttle_out;

   // TODO - update compass with throttle value
//   compass.set_throttle((float)g.rc_3.servo_out/1000.0f);
}
