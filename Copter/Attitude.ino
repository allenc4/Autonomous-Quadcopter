#include <AP_Math.h>

#include "Config.h"

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
    if(!motors.getMotors()->armed() || radio.getRCThrottle()->control_in <= 0) {
        attitude.relax_bf_rate_controller();
        attitude.set_yaw_target_to_current_heading();
        attitude.set_throttle_out(0, false);
        return;
    }

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles(radio.getRCRoll()->control_in,
    		radio.getRCPitch()->control_in,
			target_roll,
			target_pitch);

//#if DEBUG == ENABLED
//    if (loop_count % 20 == 0) {
//		hal.console->printf("RCRollCi: %d\t RollTarg: %d\t RCPitCi: %d\t PitTarg: %d\t",
//				rc_channels[RC_CHANNEL_ROLL].control_in,
//				target_roll,
//				rc_channels[RC_CHANNEL_PITCH].control_in,
//				target_pitch);
//    }
//#endif

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(
    		radio.getRCYaw()->control_in);

//#if DEBUG == ENABLED
//    if (loop_count % 20 == 0) {
//		hal.console->printf("RCYawCi: %d\t YawTarg: %d\t",
//				rc_channels[RC_CHANNEL_YAW].control_in,
//				target_yaw_rate);
//    }
//#endif

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(
    		radio.getRCThrottle()->control_in);

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
