/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"


// counter to verify landings
static uint32_t land_detector_count = 0;

// run land and crash detectors
// called at MAIN_LOOP_RATE
void Copter::update_land_and_crash_detectors()
{
    // update 1hz filtered acceleration
    Vector3f accel_ef = ahrs.get_accel_ef_blended();
    accel_ef.z += GRAVITY_MSS;
    land_accel_ef_filter.apply(accel_ef, MAIN_LOOP_SECONDS);

	long start, end;
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	supt->setCurProcessResult("update_land_detector", start, 1);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
	update_land_detector();
	end = clock();
	supt->setCurProcessResult("update_land_detector", end, 2);
	supt->setCurProcessResult("update_land_detector", (end - start), 3);

#if PARACHUTE == ENABLED
    // check parachute
    parachute_check();
#endif
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	supt->setCurProcessResult("crash_check", start, 1);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
	crash_check();
	end = clock();
	supt->setCurProcessResult("crash_check", end, 2);
	supt->setCurProcessResult("crash_check", (end - start), 3);
}

// update_land_detector - checks if we have landed and updates the ap.land_complete flag
// called at MAIN_LOOP_RATE
void Copter::update_land_detector()
{
    // land detector can not use the following sensors because they are unreliable during landing
    // barometer altitude :                 ground effect can cause errors larger than 4m
    // EKF vertical velocity or altitude :  poor barometer and large acceleration from ground impact
    // earth frame angle or angle error :   landing on an uneven surface will force the airframe to match the ground angle
    // gyro output :                        on uneven surface the airframe may rock back an forth after landing
    // range finder :                       tend to be problematic at very short distances
    // input throttle :                     in slow land the input throttle may be only slightly less than hover
	long start, end;
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	supt->setCurProcessResult("armed", start, 1);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
	bool has_armed = motors.armed();
	end = clock();
	supt->setCurProcessResult("armed", end, 2);
	supt->setCurProcessResult("armed", (end - start), 3);
	if (has_armed == false) {
		//if (!motors.armed()) {
		// if disarmed, always landed. 
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		supt->setCurProcessResult("set_land_complete", start, 1);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
		set_land_complete(true);
		end = clock();
		supt->setCurProcessResult("set_land_complete", end, 2);
		supt->setCurProcessResult("set_land_complete", (end - start), 3);
}
	else if (ap.land_complete) {
#if FRAME_CONFIG == HELI_FRAME
		// if rotor speed and collective pitch are high then clear landing flag
		if (motors.get_throttle() > get_non_takeoff_throttle() && motors.rotor_runup_complete()) {
#else
		// if throttle output is high then clear landing flag
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		supt->setCurProcessResult("get_throttle", start, 1);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
		float get_the_throttle = motors.get_throttle();
		end = clock();
		supt->setCurProcessResult("get_throttle", end, 2);
		supt->setCurProcessResult("get_throttle", (end - start), 3);
		float get_non_takeoffthrotle = get_non_takeoff_throttle();
		if (get_the_throttle > get_non_takeoffthrotle) {
			//if (motors.get_throttle() > get_non_takeoff_throttle()) {
#endif
			// ------------------------  ²å×®µã ---------------------------------
			start = clock();
			supt->setCurProcessResult("set_land_complete", start, 1);
			// ------------------------  ²å×®¼¤Àø --------------------------------- 
			set_land_complete(false);
			end = clock();
			supt->setCurProcessResult("set_land_complete", end, 2);
			supt->setCurProcessResult("set_land_complete", (end - start), 3);
		}
		}
	else {

#if FRAME_CONFIG == HELI_FRAME
		// check that collective pitch is on lower limit (should be constrained by LAND_COL_MIN)
		bool motor_at_lower_limit = motors.limit.throttle_lower;
#else
		// check that the average throttle output is near minimum (less than 12.5% hover throttle)
		bool motor_at_lower_limit = motors.limit.throttle_lower && motors.is_throttle_mix_min();
#endif

		// check that the airframe is not accelerating (not falling or breaking after fast forward flight)
		bool accel_stationary = (land_accel_ef_filter.get().length() <= LAND_DETECTOR_ACCEL_MAX);

		if (motor_at_lower_limit && accel_stationary) {
			// landed criteria met - increment the counter and check if we've triggered
			if (land_detector_count < ((float)LAND_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
				land_detector_count++;
			}
			else {
				long start, end;
				// ------------------------  ²å×®µã ---------------------------------
				start = clock();
				supt->setCurProcessResult("set_land_complete", start, 1);
				// ------------------------  ²å×®¼¤Àø --------------------------------- 
				set_land_complete(true);
				end = clock();
				supt->setCurProcessResult("set_land_complete", end, 2);
				supt->setCurProcessResult("set_land_complete", (end - start), 3);
			}
		}
		else {
			// we've sensed movement up or down so reset land_detector
			land_detector_count = 0;
		}
	}
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	supt->setCurProcessResult("set_land_complete_maybe", start, 1);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
	set_land_complete_maybe(ap.land_complete || (land_detector_count >= LAND_DETECTOR_MAYBE_TRIGGER_SEC*MAIN_LOOP_RATE));
	end = clock();
	supt->setCurProcessResult("set_land_complete_maybe", end, 2);
	supt->setCurProcessResult("set_land_complete_maybe", (end - start), 3);
}

void Copter::set_land_complete(bool b)
{
    // if no change, exit immediately
    if( ap.land_complete == b )
        return;

    land_detector_count = 0;

    if(b){
        //Log_Write_Event(DATA_LAND_COMPLETE);
    } else {
        //Log_Write_Event(DATA_NOT_LANDED);
    }
    ap.land_complete = b;
}

// set land complete maybe flag
void Copter::set_land_complete_maybe(bool b)
{
    // if no change, exit immediately
    if (ap.land_complete_maybe == b)
        return;

    if (b) {
        //Log_Write_Event(DATA_LAND_COMPLETE_MAYBE);
    }
    ap.land_complete_maybe = b;
}

// update_throttle_thr_mix - sets motors throttle_low_comp value depending upon vehicle state
//  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
//  has no effect when throttle is above hover throttle
void Copter::update_throttle_thr_mix()
{
#if FRAME_CONFIG != HELI_FRAME
    // if disarmed or landed prioritise throttle
    if(!motors.armed() || ap.land_complete) {
        motors.set_throttle_mix_min();
        return;
    }

    if (mode_has_manual_throttle(control_mode)) {
        // manual throttle
        if(channel_throttle->control_in <= 0) {
            motors.set_throttle_mix_min();
        } else {
            motors.set_throttle_mix_mid();
        }
    } else {
        // autopilot controlled throttle

        // check for aggressive flight requests - requested roll or pitch angle below 15 degrees
        const Vector3f angle_target = attitude_control.angle_ef_targets();
        bool large_angle_request = (pythagorous2(angle_target.x, angle_target.y) > 1500.0f);

        // check for large external disturbance - angle error over 30 degrees
        const Vector3f angle_error = attitude_control.angle_bf_error();
        bool large_angle_error = (pythagorous2(angle_error.x, angle_error.y) > 3000.0f);

        // check for large acceleration - falling or high turbulence
        Vector3f accel_ef = ahrs.get_accel_ef_blended();
        accel_ef.z += GRAVITY_MSS;
        bool accel_moving = (accel_ef.length() > 3.0f);

        // check for requested decent
        bool descent_not_demanded = pos_control.get_desired_velocity().z >= 0.0f;

        if ( large_angle_request || large_angle_error || accel_moving || descent_not_demanded) {
            motors.set_throttle_mix_max();
        } else {
            motors.set_throttle_mix_min();
        }
    }
#endif
}
