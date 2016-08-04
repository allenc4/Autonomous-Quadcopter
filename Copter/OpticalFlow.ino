#include "OpticalFlow.h"

#if OPTFLOW == ENABLED
OpticalFlow::OpticalFlow(RangeFinder *rf) {
	rangeFinder = rf;
	last_of_update = 0;
	last_of_roll_update = 0;
	last_of_pitch_update = 0;

	// Set up the PIDs for the optical flow
	pids_optflow[PID_OPTFLOW_PITCH].kP(OPTFLOW_PITCH_P);
	pids_optflow[PID_OPTFLOW_PITCH].kI(OPTFLOW_PITCH_I);
	pids_optflow[PID_OPTFLOW_PITCH].kD(OPTFLOW_PITCH_D);
	pids_optflow[PID_OPTFLOW_PITCH].imax(OPTFLOW_IMAX);

	pids_optflow[PID_OPTFLOW_ROLL].kP(OPTFLOW_ROLL_P);
	pids_optflow[PID_OPTFLOW_ROLL].kI(OPTFLOW_ROLL_I);
	pids_optflow[PID_OPTFLOW_ROLL].kD(OPTFLOW_ROLL_D);
	pids_optflow[PID_OPTFLOW_ROLL].imax(OPTFLOW_IMAX);

	reset_I();
}

bool OpticalFlow::init() {

	_optflow.init();
	if (!_optflow.healthy()){
		// Failed to initialize optical flow sensor
		hal.console->println("Failed to initialize optical flow sensor....");
		hal.scheduler->delay(20);

		return false;
	}

	// suspend timer while we set-up SPI communication
	hal.scheduler->suspend_timer_procs();

	_optflow.set_orientation(OPTFLOW_ORIENTATION);   // set optical flow sensor's orientation on aircraft
//	_optflow.set_frame_rate(2000);                   // set minimum update rate (which should lead to maximum low light performance
//	_optflow.set_resolution(OPTFLOW_RESOLUTION);     // set optical flow sensor's resolution
	_optflow.set_field_of_view(OPTFLOW_FOV);         // set optical flow sensor's field of view

	// resume timer
	hal.scheduler->resume_timer_procs();
	hal.scheduler->delay(50);

	return true;
}

void OpticalFlow::update() {
	// if new data has arrived, process it
	if( _optflow.last_update != last_of_update ) {
		last_of_update = _optflow.last_update;

		// Ensure rangeFinder distance is greater than 0
		uint16_t dist = rangeFinder->getLastDistance();

		// Update internal lon and lat with estimation based on optical flow
		_optflow.update_position(ahrs.roll, ahrs.pitch, ahrs.sin_yaw(), ahrs.cos_yaw(), dist);

//		if (DEBUG == ENABLED) {
//			hal.console->printf("OpticalFlow change x: %4.1f, y: %4.1f\n", _optflow.change_x, _optflow.change_y);
//		}
	}
}

void OpticalFlow::reset_I() {
	pids_optflow[PID_OPTFLOW_ROLL].reset_I();
	pids_optflow[PID_OPTFLOW_PITCH].reset_I();

	_of_roll  = 0;
	_of_pitch = 0;
}

// Below two methods (get_of_roll() and get_of_pitch())taken from Attitude.pde from ArduCopter firmware

/**
 * Calculate modified roll/pitch depending upon optical flow calculated position
 * input_roll: roll input of RC stick (in centi-degrees (RC_Channel.control_in))
 * input_yaw:  yaw input of RC stick (in centi-degrees (RC_Channel.control_in))
 *
 */
int32_t OpticalFlow::get_of_roll(int32_t input_roll, int32_t input_yaw)
{
    float tot_x_cm = 0;      // total distance from target
    int32_t new_roll = 0;
    int32_t p,i,d;

    // check if new optflow data available
    if( _optflow.last_update != last_of_roll_update) {
        last_of_roll_update = _optflow.last_update;

        // add new distance moved
        tot_x_cm += _optflow.x;

        // only correct if caller isn't modifying roll AND yaw (accounting for RC jitter)
        if (abs(input_roll) <= 200 && abs(input_yaw) <= 200 && rangeFinder->getLastDistance() < 1500) {
        	p = pids_optflow[PID_OPTFLOW_ROLL].get_p(-tot_x_cm);
        	i = pids_optflow[PID_OPTFLOW_ROLL].get_i(-tot_x_cm, 1.0f);
        	d = pids_optflow[PID_OPTFLOW_ROLL].get_d(-tot_x_cm, 1.0f);
            new_roll = p+i+d;
        } else {
#if DEBUG == ENABLED
        	hal.console->println("Resetting OF motion in roll func...");
#endif
        	_optflow.clear_motion();
        	pids_optflow[PID_OPTFLOW_ROLL].reset_I();
            tot_x_cm = 0;
            p = 0;              // for logging
            i = 0;
            d = 0;
        }

        // limit amount of change and maximum angle
        of_roll = constrain_int32(new_roll, (of_roll-20), (of_roll+20));

    }

    // limit max angle
    of_roll = constrain_int32(of_roll, -1000, 1000);

    return input_roll+of_roll;
}

int32_t OpticalFlow::get_of_pitch(int32_t input_pitch, int32_t input_yaw)
{
    float tot_y_cm = 0;  // total distance from target
    int32_t new_pitch = 0;
    int32_t p,i,d;

    // check if new optflow data available
    if( _optflow.last_update != last_of_pitch_update ) {
        last_of_pitch_update = _optflow.last_update;

        // add new distance moved
        tot_y_cm += _optflow.y;

        // only correct if caller isn't modifying pitch AND yaw (accounting for RC jitter)
        if (abs(input_pitch) <= 200 && abs(input_yaw) <= 200 && rangeFinder->getLastDistance() < 1500) {
        	p = pids_optflow[PID_OPTFLOW_PITCH].get_p(tot_y_cm);
        	i = pids_optflow[PID_OPTFLOW_PITCH].get_i(tot_y_cm, 1.0f);
        	d = pids_optflow[PID_OPTFLOW_PITCH].get_d(tot_y_cm, 1.0f);
            new_pitch = p + i + d;
//            hal.console->printf("pitch: p: %d\t i: %d\t, d: %d\n");
        } else {
#if DEBUG == ENABLED
        	hal.console->println("Resetting OF motion in pitch func...");
#endif
        	_optflow.clear_motion();
            tot_y_cm = 0;
            pids_optflow[PID_OPTFLOW_PITCH].reset_I();
            p = 0;              // for logging
            i = 0;
            d = 0;
        }

        // limit amount of change
        of_pitch = constrain_int32(new_pitch, (of_pitch-20), (of_pitch+20));

    }

    // limit max angle
    of_pitch = constrain_int32(of_pitch, -1000, 1000);

    return input_pitch+of_pitch;
}

float OpticalFlow::get_change_x() {
	return _optflow.x_cm;
}

float OpticalFlow::get_change_y() {
	return _optflow.y_cm;
}

void OpticalFlow::debug_print() {
	if (loop_count % 10 == 0) {

		hal.console->printf("Squal: %d\t x_cm: %4.2f\t y_cm: %4.2f\t"
				"pitch: %4.2f\t roll: %4.2f\n",
//				_optflow.x,
//				_optflow.y,
				_optflow.surface_quality,
				_optflow.x_cm,
				_optflow.y_cm,
				ahrs.roll,
				ahrs.pitch);
	}
}

// Hack to get register_timer_process() to work
void OpticalFlow::read(uint32_t now) {
	_optflow.read();
}
#endif
