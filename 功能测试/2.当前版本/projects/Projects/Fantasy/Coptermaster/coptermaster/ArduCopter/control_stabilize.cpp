/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_stabilize.pde - init and run calls for stabilize flight mode
 */

// stabilize_init - initialise stabilize controller
bool Copter::stabilize_init(bool ignore_checks)
{
    // set target altitude to zero for reporting
    // To-Do: make pos controller aware when it's active/inactive so it can always report the altitude error?
    pos_control.set_alt_target(0);

    // stabilize should never be made to fail
    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::stabilize_run()
{
	struct timeval startTime, endTime;
    float target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;

    // if not armed or throttle at zero, set throttle to zero and exit immediately
    //if(!motors.armed() || ap.throttle_zero) {
    //    attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
    //    // slow start if landed
    //    if (ap.land_complete) {
	// ------------------------  ��׮�� ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ��׮���� --------------------------------- 
	bool motor_state = motors.armed();

	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("armed", startTime, endTime);

	if (motor_state == false || ap.throttle_zero == true) {
		// ------------------------  ��׮�� ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ��׮���� --------------------------------- 
		attitude_control.set_throttle_out_unstabilized(0, true, g.throttle_filt);
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("set_throttle_out_unstabilized", startTime, endTime);
		// slow start if landed
		if (ap.land_complete == true) {
			// ------------------------  ��׮�� ---------------------------------
			gettimeofday(&startTime, NULL);
			// ------------------------  ��׮���� --------------------------------- 
            motors.slow_start(true);
			gettimeofday(&endTime, NULL);
			supt->setCurProcessResult("slow_start", startTime, endTime);
        }
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->control_in, channel_pitch->control_in, target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
	// ------------------------  ��׮�� ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ��׮���� --------------------------------- 
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);

	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("get_pilot_desired_yaw_rate", startTime, endTime);

    // get pilot's desired throttle
	// ------------------------  ��׮�� ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ��׮���� --------------------------------- 
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->control_in);
	
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("get_pilot_desired_throttle", startTime, endTime);

    // call attitude controller
	// ------------------------  ��׮�� ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ��׮���� --------------------------------- 
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw_smooth", startTime, endTime);
    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
	// ------------------------  ��׮�� ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ��׮���� --------------------------------- 
    attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt); 

	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("set_throttle_out", startTime, endTime);
}
