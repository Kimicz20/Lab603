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
	cout << "---- stabilize_run begin ----" << endl;
	long start, end;
    float target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;

    // if not armed or throttle at zero, set throttle to zero and exit immediately
    //if(!motors.armed() || ap.throttle_zero) {
    //    attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
    //    // slow start if landed
    //    if (ap.land_complete) {
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("armed", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
	bool motor_state = motors.armed();

	end = clock();
	this->supt->setCurProcessResult("armed", end, 2);
	this->supt->setCurProcessResult("armed", (end - start), 3);

	if (motor_state == false || ap.throttle_zero == true) {
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("set_throttle_out_unstabilized", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
		attitude_control.set_throttle_out_unstabilized(0, true, g.throttle_filt);
		end = clock();
		this->supt->setCurProcessResult("set_throttle_out_unstabilized", end, 2);
		this->supt->setCurProcessResult("set_throttle_out_unstabilized", (end - start), 3);
		// slow start if landed
		if (ap.land_complete == true) {
			// ------------------------  ²å×®µã ---------------------------------
			start = clock();
            motors.slow_start(true);
			this->supt->setCurProcessResult("slow_start", start, 1);
			// ------------------------  ²å×®¼¤Àø ---------------------------------
            motors.slow_start(true);
			end = clock();
			this->supt->setCurProcessResult("slow_start", end, 2);
			this->supt->setCurProcessResult("slow_start", (end - start), 3);
        }
		cout << "---- stabilize_run end ----" << endl;
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->control_in, channel_pitch->control_in, target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("get_pilot_desired_yaw_rate", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);

	end = clock();
	this->supt->setCurProcessResult("get_pilot_desired_yaw_rate", end, 2);
	this->supt->setCurProcessResult("get_pilot_desired_yaw_rate", (end - start), 3);

    // get pilot's desired throttle
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("get_pilot_desired_throttle", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->control_in);
	
	end = clock();
	this->supt->setCurProcessResult("get_pilot_desired_throttle", end, 2);
	this->supt->setCurProcessResult("get_pilot_desired_throttle", (end - start), 3);

    // call attitude controller
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw_smooth", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

	end = clock();
	this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw_smooth", end, 2);
	this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw_smooth", (end - start), 3);
    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("set_throttle_out", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt); 

	end = clock();
	this->supt->setCurProcessResult("set_throttle_out", end, 2);
	this->supt->setCurProcessResult("set_throttle_out", (end - start), 3);
	cout << "---- stabilize_run end ----" << endl;
}
