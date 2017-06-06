/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_rtl.pde - init and run calls for RTL flight mode
 *
 * There are two parts to RTL, the high level decision making which controls which state we are in
 * and the lower implementation of the waypoint or landing controllers within those states
 */

// rtl_init - initialise rtl controller
bool Copter::rtl_init(bool ignore_checks)
{
    if (position_ok() || ignore_checks) {
        rtl_climb_start();
        return true;
    }else{
        return false;
    }
}

// rtl_run - runs the return-to-launch controller
// should be called at 100hz or more
void Copter::rtl_run()
{
	struct timeval startTime, endTime;
	
    // check if we need to move to next state
	//FixÐÞ¸Ä2.3
	rtl_state_complete = true;
	if (supt->getParamValueWithNameAndKey("rtl_loiterathome_run", "!rtl_state_complete") == 1)
		rtl_state_complete = false;

	
    if (rtl_state_complete) {
		int b;
		string tmp[] = { "rtl_return_start", "rtl_loiterathome_start", "rtl_descent_start","rtl_land_start"};
		b = supt->getParamValueFormNamesWithKey(tmp, "rtl_state");
		switch (b) {
			case 0:
				rtl_state = RTL_InitialClimb;
				break;
			case 1:
				rtl_state = RTL_ReturnHome;
				break;
			case 2:
			case 3:
				rtl_state = RTL_FinalDescent;
				break;
		}
        switch (rtl_state) {
        case RTL_InitialClimb:
			// ------------------------  ²å×®µã ---------------------------------
			gettimeofday(&startTime, NULL);
			// ------------------------  ²å×®¼¤Àø --------------------------------- 
            rtl_return_start();
			gettimeofday(&endTime, NULL);
			supt->setCurProcessResult("rtl_return_start", startTime, endTime);
            break;
        case RTL_ReturnHome:
			// ------------------------  ²å×®µã ---------------------------------
			gettimeofday(&startTime, NULL);
			// ------------------------  ²å×®¼¤Àø --------------------------------- 
            rtl_loiterathome_start();
			gettimeofday(&endTime, NULL);
			supt->setCurProcessResult("rtl_loiterathome_start", startTime, endTime);

            break;
        case RTL_LoiterAtHome:
            if (g.rtl_alt_final > 0 && !failsafe.radio) {
				// ------------------------  ²å×®µã ---------------------------------
				gettimeofday(&startTime, NULL);
				// ------------------------  ²å×®¼¤Àø --------------------------------- 
                rtl_descent_start();
				gettimeofday(&endTime, NULL);
				supt->setCurProcessResult("rtl_descent_start", startTime, endTime);

            }else{
				// ------------------------  ²å×®µã ---------------------------------
				gettimeofday(&startTime, NULL);
				// ------------------------  ²å×®¼¤Àø --------------------------------- 
                rtl_land_start();
				gettimeofday(&endTime, NULL);
				supt->setCurProcessResult("rtl_land_start", startTime, endTime);

            }
            break;
        case RTL_FinalDescent:
            // do nothing
            break;
        case RTL_Land:
            // do nothing - rtl_land_run will take care of disarming motors
            break;
        }
    }

	int b;
	string tmp[] = { "rtl_climb_return_run", "rtl_loiterathome_run", "rtl_descent_run","rtl_land_run" };
	b = supt->getParamValueFormNamesWithKey(tmp, "rtl_state");
	switch (b) {
		case 0:
			rtl_state = RTL_InitialClimb;
			break;
		case 1:
			rtl_state = RTL_ReturnHome;
			break;
		case 2:
			rtl_state = RTL_LoiterAtHome;
			break;
		case 3:
			rtl_state = RTL_FinalDescent;
			break;
		case 4:
			rtl_state = RTL_Land;
			break;
	}

    // call the correct run function
    switch (rtl_state) {

    case RTL_InitialClimb:
    case RTL_ReturnHome:
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        rtl_climb_return_run();
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("rtl_climb_return_run", startTime, endTime);
        break;

    case RTL_LoiterAtHome:
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        rtl_loiterathome_run();
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("rtl_loiterathome_run", startTime, endTime);
        break;

    case RTL_FinalDescent:
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        rtl_descent_run();
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("rtl_descent_run", startTime, endTime);
        break;

    case RTL_Land:
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        rtl_land_run();
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("rtl_land_run", startTime, endTime);
        break;
    }
}

// rtl_climb_start - initialise climb to RTL altitude
void Copter::rtl_climb_start()
{
    rtl_state = RTL_InitialClimb;
    rtl_state_complete = false;
    rtl_alt = get_RTL_alt();

    // initialise waypoint and spline controller
    wp_nav.wp_and_spline_init();

    // get horizontal stopping point
    Vector3f destination;
    wp_nav.get_wp_stopping_point_xy(destination);

#if AC_RALLY == ENABLED
    // rally_point.alt will be the altitude of the nearest rally point or the RTL_ALT. uses absolute altitudes
    Location rally_point = rally.calc_best_rally_or_home_location(current_loc, rtl_alt+ahrs.get_home().alt);
    rally_point.alt -= ahrs.get_home().alt; // convert to altitude above home
    rally_point.alt = maxf(rally_point.alt, current_loc.alt);    // ensure we do not descend before reaching home
    destination.z = pv_alt_above_origin(rally_point.alt);
#else
    destination.z = pv_alt_above_origin(rtl_alt);
#endif

    // set the destination
    wp_nav.set_wp_destination(destination);
    wp_nav.set_fast_waypoint(true);

    // hold current yaw during initial climb
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// rtl_return_start - initialise return to home
void Copter::rtl_return_start()
{
    rtl_state = RTL_ReturnHome;
    rtl_state_complete = false;

    // set target to above home/rally point
#if AC_RALLY == ENABLED
    // rally_point will be the nearest rally point or home.  uses absolute altitudes
    Location rally_point = rally.calc_best_rally_or_home_location(current_loc, rtl_alt+ahrs.get_home().alt);
    rally_point.alt -= ahrs.get_home().alt; // convert to altitude above home
    rally_point.alt = maxf(rally_point.alt, current_loc.alt);    // ensure we do not descend before reaching home
    Vector3f destination = pv_location_to_vector(rally_point);
#else
    Vector3f destination = pv_location_to_vector(ahrs.get_home());
    destination.z = pv_alt_above_origin(rtl_alt));
#endif

    wp_nav.set_wp_destination(destination);

    // initialise yaw to point home (maybe)
    set_auto_yaw_mode(get_default_auto_yaw_mode(true));
}

// rtl_climb_return_run - implements the initial climb, return home and descent portions of RTL which all rely on the wp controller
//      called by rtl_run at 100hz or more
void Copter::rtl_climb_return_run()
{
	struct timeval startTime, endTime;
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    if(!ap.auto_armed || !motors.get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        // reset attitude control targets
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("get_interlock", startTime, endTime);

		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("set_throttle_out_unstabilized", startTime, endTime);
#endif
        // To-Do: re-initialise wpnav targets
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("get_pilot_desired_yaw_rate", startTime, endTime);
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // run waypoint controller
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    wp_nav.update_wpnav();
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("update_wpnav", startTime, endTime);

    // call z-axis position controller (wpnav should have already updated it's alt target)
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    pos_control.update_z_controller();
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("update_z_controller", startTime, endTime);

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", startTime, endTime);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(),true);
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("angle_ef_roll_pitch_yaw", startTime, endTime);
    }

    // check if we've completed this stage of RTL
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    rtl_state_complete = wp_nav.reached_wp_destination();
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("reached_wp_destination", startTime, endTime);
}

// rtl_return_start - initialise return to home
void Copter::rtl_loiterathome_start()
{
    rtl_state = RTL_LoiterAtHome;
    rtl_state_complete = false;
    rtl_loiter_start_time = millis();

    // yaw back to initial take-off heading yaw unless pilot has already overridden yaw
    if(get_default_auto_yaw_mode(true) != AUTO_YAW_HOLD) {
        set_auto_yaw_mode(AUTO_YAW_RESETTOARMEDYAW);
    } else {
        set_auto_yaw_mode(AUTO_YAW_HOLD);
    }
}

// rtl_climb_return_descent_run - implements the initial climb, return home and descent portions of RTL which all rely on the wp controller
//      called by rtl_run at 100hz or more
void Copter::rtl_loiterathome_run()
{
	struct timeval startTime, endTime;
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    if(!ap.auto_armed || !motors.get_interlock()) {
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("get_interlock", startTime, endTime);
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        // reset attitude control targets
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("set_throttle_out_unstabilized", startTime, endTime);
#endif
        // To-Do: re-initialise wpnav targets
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("get_pilot_desired_yaw_rate", startTime, endTime);
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // run waypoint controller
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    wp_nav.update_wpnav();
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("update_wpnav", startTime, endTime);

    // call z-axis position controller (wpnav should have already updated it's alt target)
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    pos_control.update_z_controller();
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("update_z_controller", startTime, endTime);

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", startTime, endTime);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(),true);
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("angle_ef_roll_pitch_yaw", startTime, endTime);
    }

    // check if we've completed this stage of RTL
    if ((millis() - rtl_loiter_start_time) >= (uint32_t)g.rtl_loiter_time.get()) {
        if (auto_yaw_mode == AUTO_YAW_RESETTOARMEDYAW) {
            // check if heading is within 2 degrees of heading when vehicle was armed
            if (labs(wrap_180_cd(ahrs.yaw_sensor-initial_armed_bearing)) <= 200) {
                rtl_state_complete = true;
            }
        } else {
            // we have loitered long enough
            rtl_state_complete = true;
        }
    }
}

// rtl_descent_start - initialise descent to final alt
void Copter::rtl_descent_start()
{
    rtl_state = RTL_FinalDescent;
    rtl_state_complete = false;
	struct timeval startTime, endTime;
    // Set wp navigation target to above home
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
	wp_nav.init_loiter_target(wp_nav.get_wp_destination());
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("init_loiter_target", startTime, endTime);

    // initialise altitude target to stopping point
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    pos_control.set_target_to_stopping_point_z();
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("set_target_to_stopping_point_z", startTime, endTime);

    // initialise yaw
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    set_auto_yaw_mode(AUTO_YAW_HOLD);
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("set_auto_yaw_mode", startTime, endTime);
}

// rtl_descent_run - implements the final descent to the RTL_ALT
//      called by rtl_run at 100hz or more
void Copter::rtl_descent_run()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;
	struct timeval startTime, endTime;
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    if(!ap.auto_armed || !motors.get_interlock()) {
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("get_interlock", startTime, endTime);
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("set_throttle_out_unstabilized", startTime, endTime);
#endif
        // set target to current position
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        wp_nav.init_loiter_target();
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("init_loiter_target", startTime, endTime);
        return;
    }

    // process pilot's input
    if (!failsafe.radio) {
        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
			// ------------------------  ²å×®µã ---------------------------------
			gettimeofday(&startTime, NULL);
			// ------------------------  ²å×®¼¤Àø --------------------------------- 
            update_simple_mode();
			gettimeofday(&endTime, NULL);
			supt->setCurProcessResult("update_simple_mode", startTime, endTime);

            // process pilot's roll and pitch input
            roll_control = channel_roll->control_in;
            pitch_control = channel_pitch->control_in;
        }

        // get pilot's desired yaw rate
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("get_pilot_desired_yaw_rate", startTime, endTime);
    }

    // process roll, pitch inputs
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    wp_nav.set_pilot_desired_acceleration(roll_control, pitch_control);
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("set_pilot_desired_acceleration", startTime, endTime);

    // run loiter controller
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("update_loiter", startTime, endTime);

    // call z-axis position controller
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    pos_control.set_alt_target_with_slew(pv_alt_above_origin(g.rtl_alt_final), G_Dt);
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("set_alt_target_with_slew", startTime, endTime);

	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    pos_control.update_z_controller();
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("update_z_controller", startTime, endTime);

    // roll & pitch from waypoint controller, yaw rate from pilot
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", startTime, endTime);

    // check if we've reached within 20cm of final altitude
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    rtl_state_complete = fabsf(pv_alt_above_origin(g.rtl_alt_final) - inertial_nav.get_altitude()) < 20.0f;
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("get_altitude", startTime, endTime);
}

// rtl_loiterathome_start - initialise controllers to loiter over home
void Copter::rtl_land_start()
{
    rtl_state = RTL_Land;
    rtl_state_complete = false;
	struct timeval startTime, endTime;
    // Set wp navigation target to above home
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    wp_nav.init_loiter_target(wp_nav.get_wp_destination());
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("init_loiter_target", startTime, endTime);
    // initialise altitude target to stopping point
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    pos_control.set_target_to_stopping_point_z();
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("set_target_to_stopping_point_z", startTime, endTime);
    // initialise yaw
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    set_auto_yaw_mode(AUTO_YAW_HOLD);
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("set_auto_yaw_mode", startTime, endTime);
}

// rtl_returnhome_run - return home
//      called by rtl_run at 100hz or more
void Copter::rtl_land_run()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;
	struct timeval startTime, endTime;
    // if not auto armed or landing completed or motor interlock not enabled set throttle to zero and exit immediately
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    if(!ap.auto_armed || ap.land_complete || !motors.get_interlock()) {
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("get_interlock", startTime, endTime);
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("set_throttle_out_unstabilized", startTime, endTime);
#endif
        // set target to current position
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        wp_nav.init_loiter_target();
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("init_loiter_target", startTime, endTime);
#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        // disarm when the landing detector says we've landed and throttle is at minimum
        if (ap.land_complete && (ap.throttle_zero || failsafe.radio)) {
            init_disarm_motors();
        }
#else
        // disarm when the landing detector says we've landed
        if (ap.land_complete) {
			// ------------------------  ²å×®µã ---------------------------------
			gettimeofday(&startTime, NULL);
			// ------------------------  ²å×®¼¤Àø --------------------------------- 
            init_disarm_motors();
			gettimeofday(&endTime, NULL);
			supt->setCurProcessResult("init_disarm_motors", startTime, endTime);
        }
#endif

        // check if we've completed this stage of RTL
        rtl_state_complete = ap.land_complete;
        return;
    }

    // relax loiter target if we might be landed
    if (ap.land_complete_maybe) {
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        wp_nav.loiter_soften_for_landing();
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("loiter_soften_for_landing", startTime, endTime);
    }

    // process pilot's input
    if (!failsafe.radio) {
        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
			// ------------------------  ²å×®µã ---------------------------------
			gettimeofday(&startTime, NULL);
			// ------------------------  ²å×®¼¤Àø --------------------------------- 
            update_simple_mode();
			gettimeofday(&endTime, NULL);
			supt->setCurProcessResult("update_simple_mode", startTime, endTime);
            // process pilot's roll and pitch input
            roll_control = channel_roll->control_in;
            pitch_control = channel_pitch->control_in;
        }

        // get pilot's desired yaw rate
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("get_pilot_desired_yaw_rate", startTime, endTime);
    }

     // process pilot's roll and pitch input
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    wp_nav.set_pilot_desired_acceleration(roll_control, pitch_control);
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("set_pilot_desired_acceleration", startTime, endTime);

    // run loiter controller
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("update_loiter", startTime, endTime);

    // call z-axis position controller
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    float cmb_rate = get_land_descent_speed();
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("get_land_descent_speed", startTime, endTime);

	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    pos_control.set_alt_target_from_climb_rate(cmb_rate, G_Dt, true);
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("set_alt_target_from_climb_rate", startTime, endTime);

	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    pos_control.update_z_controller();
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("update_z_controller", startTime, endTime);
    // record desired climb rate for logging
    desired_climb_rate = cmb_rate;

    // roll & pitch from waypoint controller, yaw rate from pilot
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", startTime, endTime);

    // check if we've completed this stage of RTL
    rtl_state_complete = ap.land_complete;
}

// get_RTL_alt - return altitude which vehicle should return home at
//      altitude is in cm above home
float Copter::get_RTL_alt()
{
    // maximum of current altitude + climb_min and rtl altitude
    float ret = maxf(current_loc.alt + maxf(0, g.rtl_climb_min), g.rtl_altitude);
    ret = maxf(ret, RTL_ALT_MIN);

#if AC_FENCE == ENABLED
    // ensure not above fence altitude if alt fence is enabled
    if ((fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) != 0) {
        ret = minf(ret, fence.get_safe_alt()*100.0f);
    }
#endif

    return ret;
}

