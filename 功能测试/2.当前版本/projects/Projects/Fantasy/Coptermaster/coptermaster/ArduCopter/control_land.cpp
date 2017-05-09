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
	cout << "---------- land_run begin -----------" << endl;
	long start, end;
	//FixÐÞ¸ÄV1.4
	string str[] = { "2", "land_gps_run", "land_nogps_run" };
	int r = supt->getParamValueFormNamesWithKey(str, "land_with_gps");
	cout << "land_with_gps:" << r << endl;
	land_with_gps = false;
	if (r == 1)
		land_with_gps = true;
    if (land_with_gps) {
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("land_gps_run", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        land_gps_run();
		end = clock();
		this->supt->setCurProcessResult("land_gps_run", end, 2);
		this->supt->setCurProcessResult("land_gps_run", (end - start), 3);
    }else{
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("land_nogps_run", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
		
        land_nogps_run();
		end = clock();
		this->supt->setCurProcessResult("land_nogps_run", end, 2);
		this->supt->setCurProcessResult("land_nogps_run", (end - start), 3);

    }
	cout << "---------- land_run end -----------" << endl;
}

// land_run - runs the land controller
//      horizontal position controlled with loiter controller
//      should be called at 100hz or more
void Copter::land_gps_run()
{
	long start, end;
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;

    // if not auto armed or landed or motor interlock not enabled set throttle to zero and exit immediately
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("get_interlock", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
	bool has_interlock = motors.get_interlock();
	end = clock();
	this->supt->setCurProcessResult("get_interlock", end, 2);
	this->supt->setCurProcessResult("get_interlock", (end - start), 3);
	if(ap.auto_armed==0 || ap.land_complete==1 || has_interlock==false) {
	//if(!ap.auto_armed || ap.land_complete || !motors.get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
		start = clock();
		this->supt->setCurProcessResult("set_throttle_out_unstabilized", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
		end = clock();
		this->supt->setCurProcessResult("set_throttle_out_unstabilized", end, 2);
		this->supt->setCurProcessResult("set_throttle_out_unstabilized", (end - start), 3);
#endif
		start = clock();
		this->supt->setCurProcessResult("init_loiter_target", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        wp_nav.init_loiter_target();
		end = clock();
		this->supt->setCurProcessResult("init_loiter_target", end, 2);
		this->supt->setCurProcessResult("init_loiter_target", (end - start), 3);

#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        // disarm when the landing detector says we've landed and throttle is at minimum
        if (ap.land_complete && (ap.throttle_zero || failsafe.radio)) {
            init_disarm_motors();
        }
#else
        // disarm when the landing detector says we've landed
        if (ap.land_complete==1) {
			start = clock();
			this->supt->setCurProcessResult("init_disarm_motors", start, 1);
			// ------------------------  ²å×®¼¤Àø ---------------------------------
            init_disarm_motors();
			end = clock();
			this->supt->setCurProcessResult("init_disarm_motors", end, 2);
			this->supt->setCurProcessResult("init_disarm_motors", (end - start), 3);
        }
#endif
        return;
    }

    // relax loiter target if we might be landed
    //if (ap.land_complete_maybe) {
	if (ap.land_complete_maybe == 1) {
		start = clock();
		this->supt->setCurProcessResult("loiter_soften_for_landing", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        wp_nav.loiter_soften_for_landing();
		end = clock();
		this->supt->setCurProcessResult("loiter_soften_for_landing", end, 2);
		this->supt->setCurProcessResult("loiter_soften_for_landing", (end - start), 3);
    }

    // process pilot inputs
   /* if (!failsafe.radio) {
        if (g.land_repositioning) {*/
	if (failsafe.radio == 0) {
		if (g.land_repositioning == 1) {
            // apply SIMPLE mode transform to pilot inputs
			start = clock();
			this->supt->setCurProcessResult("update_simple_mode", start, 1);
			// ------------------------  ²å×®¼¤Àø ---------------------------------
            update_simple_mode();
			end = clock();
			this->supt->setCurProcessResult("update_simple_mode", end, 2);
			this->supt->setCurProcessResult("update_simple_mode", (end - start), 3);

            // process pilot's roll and pitch input
            roll_control = channel_roll->control_in;
            pitch_control = channel_pitch->control_in;

            // record if pilot has overriden roll or pitch
            if (roll_control != 0 || pitch_control != 0) {
                ap.land_repo_active = true;
            }
        }

        // get pilot's desired yaw rate
		start = clock();
		this->supt->setCurProcessResult("get_pilot_desired_yaw_rate", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
		end = clock();
		this->supt->setCurProcessResult("get_pilot_desired_yaw_rate", end, 2);
		this->supt->setCurProcessResult("get_pilot_desired_yaw_rate", (end - start), 3);
    }

    // process roll, pitch inputs
	start = clock();
	this->supt->setCurProcessResult("set_pilot_desired_acceleration", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
    wp_nav.set_pilot_desired_acceleration(roll_control, pitch_control);
	end = clock();
	this->supt->setCurProcessResult("set_pilot_desired_acceleration", end, 2);
	this->supt->setCurProcessResult("set_pilot_desired_acceleration", (end - start), 3);

#if PRECISION_LANDING == ENABLED
    // run precision landing
    if (!ap.land_repo_active) {
        wp_nav.shift_loiter_target(precland.get_target_shift(wp_nav.get_loiter_target()));
    }
#endif

    // run loiter controller
	start = clock();
	this->supt->setCurProcessResult("update_loiter", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);
	end = clock();
	this->supt->setCurProcessResult("update_loiter", end, 2);
	this->supt->setCurProcessResult("update_loiter", (end - start), 3);

    // call attitude controller
	start = clock();
	this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
	end = clock();
	this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", end, 2);
	this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", (end - start), 3);

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
	start = clock();
	this->supt->setCurProcessResult("set_alt_target_from_climb_rate", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
    pos_control.set_alt_target_from_climb_rate(cmb_rate, G_Dt, true);
	end = clock();
	this->supt->setCurProcessResult("set_alt_target_from_climb_rate", end, 2);
	this->supt->setCurProcessResult("set_alt_target_from_climb_rate", (end - start), 3);

	start = clock();
	this->supt->setCurProcessResult("update_z_controller", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
    pos_control.update_z_controller();
	end = clock();
	this->supt->setCurProcessResult("update_z_controller", end, 2);
	this->supt->setCurProcessResult("update_z_controller", (end - start), 3);
}

// land_nogps_run - runs the land controller
//      pilot controls roll and pitch angles
//      should be called at 100hz or more
void Copter::land_nogps_run()
{
    float target_roll = 0.0f, target_pitch = 0.0f;
    float target_yaw_rate = 0;
	long start, end;
    // process pilot inputs

	//FixÐÞ¸ÄV1.4
	string str[] = { "2", "update_simple_mode", "get_pilot_desired_yaw_rate" };
	cout << "failsafe.radio:" << endl;
	int r = supt->getParamValueFormNamesWithKey(str, "failsafe.radio");
	cout << "failsafe.radio:" << r << endl;
	if (r == 1)
		failsafe.radio = true;
	else
		failsafe.radio = false;
	r = supt->getParamValueFormNamesWithKey(str, "g.land_repositioning");
	cout << "g.land_repositioning:" << r << endl;
	if (r == 1)
		g.land_repositioning = true;
	else
		g.land_repositioning = false;

    if (!failsafe.radio) {
        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
			// ------------------------  ²å×®µã ---------------------------------
			start = clock();
			this->supt->setCurProcessResult("update_simple_mode", start, 1);
			// ------------------------  ²å×®¼¤Àø ---------------------------------
            update_simple_mode();
			end = clock();
			this->supt->setCurProcessResult("update_simple_mode", end, 2);
			this->supt->setCurProcessResult("update_simple_mode", (end - start), 3);

            // get pilot desired lean angles
			// ------------------------  ²å×®µã ---------------------------------
			start = clock();
			this->supt->setCurProcessResult("get_pilot_desired_lean_angles", start, 1);
			// ------------------------  ²å×®¼¤Àø ---------------------------------
            get_pilot_desired_lean_angles(channel_roll->control_in, channel_pitch->control_in, target_roll, target_pitch, aparm.angle_max);
			end = clock();
			this->supt->setCurProcessResult("get_pilot_desired_lean_angles", end, 2);
			this->supt->setCurProcessResult("get_pilot_desired_lean_angles", (end - start), 3);
		}

        // get pilot's desired yaw rate
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("get_pilot_desired_yaw_rate", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
		end = clock();
		this->supt->setCurProcessResult("get_pilot_desired_yaw_rate", end, 2);
		this->supt->setCurProcessResult("get_pilot_desired_yaw_rate", (end - start), 3);
    }
	cout << "@@@@@@@@@@@@@@ 1" << endl;
    // if not auto armed or landed or motor interlock not enabled set throttle to zero and exit immediately
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("get_interlock", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
	bool has_interlock = motors.get_interlock();
	end = clock();
	this->supt->setCurProcessResult("get_interlock", end, 2);
	this->supt->setCurProcessResult("get_interlock", (end - start), 3);
	
	cout << "@@@@@@@@@@@@@@ 2" << endl;
	
	//FixÐÞ¸ÄV1.4
	ap.auto_armed = supt->getParamValueWithNameAndKey("set_throttle_out_unstabilized","ap.auto_armed");
	ap.land_complete = supt->getParamValueWithNameAndKey("set_throttle_out_unstabilized", "ap.land_complete");
	has_interlock = supt->getParamValueWithNameAndKey("set_throttle_out_unstabilized","has_interlock");

	if (ap.auto_armed==0 || ap.land_complete==1 || has_interlock==false) {
		cout << "@@@@@@@@@@@@@@ 2.1" << endl;
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
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
		cout << "@@@@@@@@@@@@@@ 2.2" << endl;
#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        // disarm when the landing detector says we've landed and throttle is at minimum
        if (ap.land_complete && (ap.throttle_zero || failsafe.radio)) {
            init_disarm_motors();
        }
#else
        // disarm when the landing detector says we've landed
        if (ap.land_complete==1) {
			cout << "@@@@@@@@@@@@@@ 2.3" << endl;
			// ------------------------  ²å×®µã ---------------------------------
			start = clock();
			this->supt->setCurProcessResult("init_disarm_motors", start, 1);
			// ------------------------  ²å×®¼¤Àø ---------------------------------
            //init_disarm_motors();
			end = clock();
			this->supt->setCurProcessResult("init_disarm_motors", end, 2);
			this->supt->setCurProcessResult("init_disarm_motors", (end - start), 3);
        }
		cout << "@@@@@@@@@@@@@@ 2.4" << endl;
#endif
        return;
    }
	cout << "@@@@@@@@@@@@@@ 3" << endl;
    // call attitude controller
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw_smooth", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
	end = clock();
	this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw_smooth", end, 2);
	this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw_smooth", (end - start), 3);
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
	start = clock();
	this->supt->setCurProcessResult("set_alt_target_from_climb_rate", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
    pos_control.set_alt_target_from_climb_rate(cmb_rate, G_Dt, true);
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
	cout << "@@@@@@@@@@@@@@ 4" << endl;
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
