/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_circle.pde - init and run calls for circle flight mode
 */

// circle_init - initialise circle controller flight mode
bool Copter::circle_init(bool ignore_checks)
{
    if (position_ok() || ignore_checks) {
        circle_pilot_yaw_override = false;

        // initialize speeds and accelerations
        pos_control.set_speed_xy(wp_nav.get_speed_xy());
        pos_control.set_accel_xy(wp_nav.get_wp_acceleration());
        pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        pos_control.set_accel_z(g.pilot_accel_z);

        // initialise circle controller including setting the circle center based on vehicle speed
        circle_nav.init();

        return true;
    }else{
        return false;
    }
}

// circle_run - runs the circle flight mode
// should be called at 100hz or more
void Copter::circle_run()
{
    float target_yaw_rate = 0;
    float target_climb_rate = 0;
	long start, end;
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("get_interlock", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
	bool get_interlock = motors.get_interlock();
	end = clock();
	this->supt->setCurProcessResult("get_interlock", end, 2);
	this->supt->setCurProcessResult("get_interlock", (end - start), 3);
	if (ap.auto_armed == 0 || ap.land_complete == 1 || get_interlock == false) {
	//if(!ap.auto_armed || ap.land_complete || !motors.get_interlock()) {
        // To-Do: add some initialisation of position controllers
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("set_throttle_out_unstabilized", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
		end = clock();
		this->supt->setCurProcessResult("set_throttle_out_unstabilized", end, 2);
		this->supt->setCurProcessResult("set_throttle_out_unstabilized", (end - start), 3);
#endif
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("set_alt_target_to_current_alt", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        pos_control.set_alt_target_to_current_alt();
		end = clock();
		this->supt->setCurProcessResult("set_alt_target_to_current_alt", end, 2);
		this->supt->setCurProcessResult("set_alt_target_to_current_alt", (end - start), 3);
        return;
    }

    // process pilot inputs
   // if (!failsafe.radio) {
	if (failsafe.radio==0) {
        // get pilot's desired yaw rate
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("get_pilot_desired_yaw_rate", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
		end = clock();
		this->supt->setCurProcessResult("get_pilot_desired_yaw_rate", end, 2);
		this->supt->setCurProcessResult("get_pilot_desired_yaw_rate", (end - start), 3);
        if (!is_zero(target_yaw_rate)) {
            circle_pilot_yaw_override = true;
        }

        // get pilot desired climb rate
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("get_pilot_desired_climb_rate", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->control_in);
		end = clock();
		this->supt->setCurProcessResult("get_pilot_desired_climb_rate", end, 2);
		this->supt->setCurProcessResult("get_pilot_desired_climb_rate", (end - start), 3);

        // check for pilot requested take-off
        if (ap.land_complete && target_climb_rate > 0) {
            // indicate we are taking off
			// ------------------------  ²å×®µã ---------------------------------
			start = clock();
			this->supt->setCurProcessResult("set_land_complete", start, 1);
			// ------------------------  ²å×®¼¤Àø ---------------------------------
            set_land_complete(false);
			end = clock();
			this->supt->setCurProcessResult("set_land_complete", end, 2);
			this->supt->setCurProcessResult("set_land_complete", (end - start), 3);
            // clear i term when we're taking off
			// ------------------------  ²å×®µã ---------------------------------
			start = clock();
			this->supt->setCurProcessResult("set_throttle_takeoff", start, 1);
			// ------------------------  ²å×®¼¤Àø ---------------------------------
            set_throttle_takeoff();
			end = clock();
			this->supt->setCurProcessResult("set_throttle_takeoff", end, 2);
			this->supt->setCurProcessResult("set_throttle_takeoff", (end - start), 3);
        }
    }
	
    // run circle controller
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("update", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
    circle_nav.update();
	end = clock();
	this->supt->setCurProcessResult("update", end, 2);
	this->supt->setCurProcessResult("update", (end - start), 3);

    // call attitude controller
    if (circle_pilot_yaw_override) {
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(circle_nav.get_roll(), circle_nav.get_pitch(), target_yaw_rate);
		end = clock();
		this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", end, 2);
		this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", (end - start), 3);
    }else{
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("angle_ef_roll_pitch_yaw", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        attitude_control.angle_ef_roll_pitch_yaw(circle_nav.get_roll(), circle_nav.get_pitch(), circle_nav.get_yaw(),true);
		end = clock();
		this->supt->setCurProcessResult("angle_ef_roll_pitch_yaw", end, 2);
		this->supt->setCurProcessResult("angle_ef_roll_pitch_yaw", (end - start), 3);
    }

    // run altitude controller
    if (sonar_enabled && (sonar_alt_health >= SONAR_ALT_HEALTH_MAX)) {
        // if sonar is ok, use surface tracking
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("get_surface_tracking_climb_rate", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
		end = clock();
		this->supt->setCurProcessResult("get_surface_tracking_climb_rate", end, 2);
		this->supt->setCurProcessResult("get_surface_tracking_climb_rate", (end - start), 3);
    }
    // update altitude target and call position controller
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("set_alt_target_from_climb_rate", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
    pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt, false);
	end = clock();
	this->supt->setCurProcessResult("set_alt_target_from_climb_rate", end, 2);
	this->supt->setCurProcessResult("set_alt_target_from_climb_rate", (end - start), 3);
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("update_z_controller", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
    pos_control.update_z_controller();
	end = clock();
	this->supt->setCurProcessResult("update_z_controller", end, 2);
	this->supt->setCurProcessResult("update_z_controller", (end - start), 3);
}
