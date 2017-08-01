/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

static bool land_with_gps;

static uint32_t land_start_time;
static bool land_pause;

// land_init - initialise land controller
bool Copter::land_init(bool ignore_checks)
{
    // check if we have GPS and decide which LAND we're going to do
    land_with_gps = position_ok();
    if (land_with_gps) {
        // set target to stopping point
        Vector3f stopping_point;
        wp_nav.get_loiter_stopping_point_xy(stopping_point);
        wp_nav.init_loiter_target(stopping_point);
    }

    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(wp_nav.get_speed_down(), wp_nav.get_speed_up());
    pos_control.set_accel_z(wp_nav.get_accel_z());

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    land_start_time = millis();

    land_pause = false;

    // reset flag indicating if pilot has applied roll or pitch inputs during landing
    ap.land_repo_active = false;

    return true;
}

// land_run - runs the land controller
// should be called at 100hz or more
void Copter::land_run()
{
	struct timeval startTime, endTime;
	//FixÐÞ¸ÄV1.4
	string str[] = { "2", "land_gps_run", "land_nogps_run" };
	int r = supt->getParamValueFormNamesWithKey(str, "land_with_gps");
	land_with_gps = false;
	if (r == 1)
		land_with_gps = true;
    if (land_with_gps) {
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        land_gps_run();
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("land_gps_run", startTime, endTime);
    }else{
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
		
        land_nogps_run();
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("land_nogps_run", startTime, endTime);

    }
}

// land_run - runs the land controller
//      horizontal position controlled with loiter controller
//      should be called at 100hz or more
void Copter::land_gps_run()
{
	struct timeval startTime, endTime;
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;

    // if not auto armed or landed or motor interlock not enabled set throttle to zero and exit immediately
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
	bool has_interlock = motors.get_interlock();
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("get_interlock", startTime, endTime);

	//FixÐÞ¸ÄV1.9
	string str[] = { "4","loiter_soften_for_landing", "set_throttle_out_unstabilized", "init_loiter_target" ,"init_disarm_motors"};
	
	ap.auto_armed = true;
	if (supt->getParamValueFormNamesWithKey(str, "ap.auto_armed") == 0)
		ap.auto_armed = false;
	
	ap.land_complete = false;
	if (supt->getParamValueFormNamesWithKey(str, "ap.land_complete") == 1)
		ap.land_complete = true;
	
	has_interlock = true;
	if (supt->getParamValueFormNamesWithKey(str, "has_interlock") == 0)
		ap.auto_armed = false;

	if(ap.auto_armed==0 || ap.land_complete==1 || has_interlock==false) {
	//if(!ap.auto_armed || ap.land_complete || !motors.get_interlock()) {
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
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        wp_nav.init_loiter_target();
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("init_loiter_target", startTime, endTime);

//#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
//        // disarm when the landing detector says we've landed and throttle is at minimum
//        if (ap.land_complete && (ap.throttle_zero || failsafe.radio)) {
//            init_disarm_motors();
//        }
//#else
        // disarm when the landing detector says we've landed
        if (ap.land_complete==1) {
			// ------------------------  ²å×®µã ---------------------------------
			gettimeofday(&startTime, NULL);
			// ------------------------  ²å×®¼¤Àø --------------------------------- 
            init_disarm_motors();
			gettimeofday(&endTime, NULL);
			supt->setCurProcessResult("init_disarm_motors", startTime, endTime);
        }
//#endif
       // return;
    }

    // relax loiter target if we might be landed
    //if (ap.land_complete_maybe) {

	//FixÐÞ¸ÄV1.9
	ap.land_complete_maybe = false;
	if (supt->getParamValueWithNameAndKey("loiter_soften_for_landing", "ap.land_complete_maybe") == 1)
		ap.land_complete_maybe = true;

	if (ap.land_complete_maybe == 1) {
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        wp_nav.loiter_soften_for_landing();
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("loiter_soften_for_landing", startTime, endTime);
    }

    // process pilot inputs

	//FixÐÞ¸ÄV1.9
	failsafe.radio = false;
	if (supt->getParamValueWithNameAndKey("get_pilot_desired_yaw_rate", "failsafe.radio") == 1)
		failsafe.radio = true;
	if (failsafe.radio == 0) {

		//FixÐÞ¸ÄV1.9
		g.land_repositioning = false;
		if (supt->getParamValueWithNameAndKey("update_simple_mode", "g.land_repositioning") == 1)
			g.land_repositioning = true;

		if (g.land_repositioning == 1) {
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

            // record if pilot has overriden roll or pitch
            if (roll_control != 0 || pitch_control != 0) {
                ap.land_repo_active = true;
            }
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

#if PRECISION_LANDING == ENABLED
    // run precision landing
    if (!ap.land_repo_active) {
        wp_nav.shift_loiter_target(precland.get_target_shift(wp_nav.get_loiter_target()));
    }
#endif

    // run loiter controller
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("update_loiter", startTime, endTime);

    // call attitude controller

	//FixÐÞ¸ÄV1.9
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 

    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);

	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("get_roll", startTime, endTime);
	supt->setCurProcessResult("get_pitch", startTime, endTime);
	supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", startTime, endTime);

    // pause 4 seconds before beginning land descent
    float cmb_rate;
    if(land_pause && millis()-land_start_time < 4000) {
        cmb_rate = 0;
    } else {
        land_pause = false;
        cmb_rate = get_land_descent_speed();
    }

    // record desired climb rate for logging
    desired_climb_rate = cmb_rate;

    // update altitude target and call position controller
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
}

// land_nogps_run - runs the land controller
//      pilot controls roll and pitch angles
//      should be called at 100hz or more
void Copter::land_nogps_run()
{
    float target_roll = 0.0f, target_pitch = 0.0f;
    float target_yaw_rate = 0;
	struct timeval startTime, endTime;
    // process pilot inputs

	//FixÐÞ¸ÄV1.4
	string str[] = { "2", "update_simple_mode", "get_pilot_desired_yaw_rate" };
	int r = supt->getParamValueFormNamesWithKey(str, "failsafe.radio");
	if (r == 1)
		failsafe.radio = true;
	else
		failsafe.radio = false;
	r = supt->getParamValueFormNamesWithKey(str, "g.land_repositioning");
	if (r == 1)
		g.land_repositioning = true;
	else
		g.land_repositioning = false;

    if (!failsafe.radio) {
        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
			// ------------------------  ²å×®µã ---------------------------------
			gettimeofday(&startTime, NULL);
			// ------------------------  ²å×®¼¤Àø --------------------------------- 
            update_simple_mode();
			gettimeofday(&endTime, NULL);
			supt->setCurProcessResult("update_simple_mode", startTime, endTime);

            // get pilot desired lean angles
			// ------------------------  ²å×®µã ---------------------------------
			gettimeofday(&startTime, NULL);
			// ------------------------  ²å×®¼¤Àø --------------------------------- 
            get_pilot_desired_lean_angles(channel_roll->control_in, channel_pitch->control_in, target_roll, target_pitch, aparm.angle_max);
			gettimeofday(&endTime, NULL);
			supt->setCurProcessResult("get_pilot_desired_lean_angles", startTime, endTime);
		}

        // get pilot's desired yaw rate
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("get_pilot_desired_yaw_rate", startTime, endTime);
    }
    // if not auto armed or landed or motor interlock not enabled set throttle to zero and exit immediately
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
	bool has_interlock = motors.get_interlock();
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("get_interlock", startTime, endTime);
	
	//FixÐÞ¸ÄV1.4
	ap.auto_armed = supt->getParamValueWithNameAndKey("set_throttle_out_unstabilized","ap.auto_armed");
	ap.land_complete = supt->getParamValueWithNameAndKey("set_throttle_out_unstabilized", "ap.land_complete");
	has_interlock = supt->getParamValueWithNameAndKey("set_throttle_out_unstabilized","has_interlock");

	if (ap.auto_armed==0 || ap.land_complete==1 || has_interlock==false) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("set_throttle_out_unstabilized", startTime, endTime);
#endif
#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        // disarm when the landing detector says we've landed and throttle is at minimum
        if (ap.land_complete && (ap.throttle_zero || failsafe.radio)) {
            init_disarm_motors();
        }
#else
        // disarm when the landing detector says we've landed
        if (ap.land_complete==1) {
			// ------------------------  ²å×®µã ---------------------------------
			gettimeofday(&startTime, NULL);
			// ------------------------  ²å×®¼¤Àø --------------------------------- 
            //init_disarm_motors();
			gettimeofday(&endTime, NULL);
			supt->setCurProcessResult("init_disarm_motors", startTime, endTime);
        }
#endif
       // return;
    }
    // call attitude controller
	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw_smooth", startTime, endTime);
    // pause 4 seconds before beginning land descent
    float cmb_rate;
    if(land_pause && millis()-land_start_time < LAND_WITH_DELAY_MS) {
        cmb_rate = 0;
    } else {
        land_pause = false;
        cmb_rate = get_land_descent_speed();
    }

    // record desired climb rate for logging
    desired_climb_rate = cmb_rate;

    // call position controller
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
}

// get_land_descent_speed - high level landing logic
//      returns climb rate (in cm/s) which should be passed to the position controller
//      should be called at 100hz or higher
float Copter::get_land_descent_speed()
{
#if CONFIG_SONAR == ENABLED
    bool sonar_ok = sonar_enabled && (sonar.status() == RangeFinder::RangeFinder_Good);
#else
    bool sonar_ok = false;
#endif
    // if we are above 10m and the sonar does not sense anything perform regular alt hold descent
    if (pos_control.get_pos_target().z >= pv_alt_above_origin(LAND_START_ALT) && !(sonar_ok && sonar_alt_health >= SONAR_ALT_HEALTH_MAX)) {
        return pos_control.get_speed_down();
    }else{
        return -abs(g.land_speed);
    }
}

// land_do_not_use_GPS - forces land-mode to not use the GPS but instead rely on pilot input for roll and pitch
//  called during GPS failsafe to ensure that if we were already in LAND mode that we do not use the GPS
//  has no effect if we are not already in LAND mode
void Copter::land_do_not_use_GPS()
{
    land_with_gps = false;
}

// set_mode_land_with_pause - sets mode to LAND and triggers 4 second delay before descent starts
//  this is always called from a failsafe so we trigger notification to pilot
void Copter::set_mode_land_with_pause()
{
    set_mode(LAND);
    land_pause = true;

    // alert pilot to mode change
    AP_Notify::events.failsafe_mode_change = 1;
}

// landing_with_GPS - returns true if vehicle is landing using GPS
bool Copter::landing_with_GPS() {
    return (control_mode == LAND && land_with_gps);
}
