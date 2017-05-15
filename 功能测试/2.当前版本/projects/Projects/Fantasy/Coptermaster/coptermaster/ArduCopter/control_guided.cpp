/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_guided.pde - init and run calls for guided flight mode
 */

#ifndef GUIDED_LOOK_AT_TARGET_MIN_DISTANCE_CM
 # define GUIDED_LOOK_AT_TARGET_MIN_DISTANCE_CM     500     // point nose at target if it is more than 5m away
#endif

#define GUIDED_POSVEL_TIMEOUT_MS    3000    // guided mode's position-velocity controller times out after 3seconds with no new updates
#define GUIDED_ATTITUDE_TIMEOUT_MS  1000    // guided mode's attitude controller times out after 1 second with no new updates

static Vector3f posvel_pos_target_cm;
static Vector3f posvel_vel_target_cms;
static uint32_t posvel_update_time_ms;
static uint32_t vel_update_time_ms;

struct {
    uint32_t update_time_ms;
    float roll_cd;
    float pitch_cd;
    float yaw_cd;
    float climb_rate_cms;
} static guided_angle_state = {0,0.0f, 0.0f, 0.0f, 0.0f};

struct Guided_Limit {
    uint32_t timeout_ms;  // timeout (in seconds) from the time that guided is invoked
    float alt_min_cm;   // lower altitude limit in cm above home (0 = no limit)
    float alt_max_cm;   // upper altitude limit in cm above home (0 = no limit)
    float horiz_max_cm; // horizontal position limit in cm from where guided mode was initiated (0 = no limit)
    uint32_t start_time;// system time in milliseconds that control was handed to the external computer
    Vector3f start_pos; // start position as a distance from home in cm.  used for checking horiz_max limit
} guided_limit;

// guided_init - initialise guided controller
bool Copter::guided_init(bool ignore_checks)
{
    if (position_ok() || ignore_checks) {
        // initialise yaw
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
        // start in position control mode
        guided_pos_control_start();
        return true;
    }else{
        return false;
    }
}

// guided_takeoff_start - initialises waypoint controller to implement take-off
void Copter::guided_takeoff_start(float final_alt_above_home)
{
	long start, end;
    guided_mode = Guided_TakeOff;
    
    // initialise wpnav destination
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	supt->setCurProcessResult("get_position", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    Vector3f target_pos = inertial_nav.get_position();
	end = clock();
	supt->setCurProcessResult("get_position", end, 2);
	supt->setCurProcessResult("get_position", (end - start), 3);

	target_pos.f_print_z();
	target_pos.f_print_x();
	target_pos.f_print_y();
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	supt->setCurProcessResult("pv_alt_above_origin", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
	target_pos.z = pv_alt_above_origin(final_alt_above_home);   
	end = clock();
	supt->setCurProcessResult("pv_alt_above_origin", end, 2);
	supt->setCurProcessResult("pv_alt_above_origin", (end - start), 3);

	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	supt->setCurProcessResult("set_wp_destination", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    wp_nav.set_wp_destination(target_pos);
	end = clock();
	supt->setCurProcessResult("set_wp_destination", end, 2);
	supt->setCurProcessResult("set_wp_destination", (end - start), 3);

    // initialise yaw
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	supt->setCurProcessResult("set_auto_yaw_mode", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    set_auto_yaw_mode(AUTO_YAW_HOLD);
	end = clock();
	supt->setCurProcessResult("set_auto_yaw_mode", end, 2);
	supt->setCurProcessResult("set_auto_yaw_mode", (end - start), 3);

    // clear i term when we're taking off
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	supt->setCurProcessResult("set_throttle_takeoff", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    set_throttle_takeoff();
	end = clock();
	supt->setCurProcessResult("set_throttle_takeoff", end, 2);
	supt->setCurProcessResult("set_throttle_takeoff", (end - start), 3);
}

// initialise guided mode's position controller
void Copter::guided_pos_control_start()
{
    // set to position control mode
	long start, end;
    guided_mode = Guided_WP;

    // initialise waypoint and spline controller
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	Copter::supt->setCurProcessResult("wp_and_spline_init", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    wp_nav.wp_and_spline_init();
	end = clock();
	Copter::supt->setCurProcessResult("wp_and_spline_init", end, 2);
	Copter::supt->setCurProcessResult("wp_and_spline_init", (end - start), 3);

    // initialise wpnav to stopping point at current altitude
    // To-Do: set to current location if disarmed?
    // To-Do: set to stopping point altitude?
    Vector3f stopping_point;  
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	Copter::supt->setCurProcessResult("get_altitude", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
	stopping_point.z = inertial_nav.get_altitude(); 
	end = clock();
	Copter::supt->setCurProcessResult("get_altitude", end, 2);
	Copter::supt->setCurProcessResult("get_altitude", (end - start), 3);


	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	Copter::supt->setCurProcessResult("get_wp_stopping_point_xy", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    wp_nav.get_wp_stopping_point_xy(stopping_point);
	end = clock();
	Copter::supt->setCurProcessResult("get_wp_stopping_point_xy", end, 2);
	Copter::supt->setCurProcessResult("get_wp_stopping_point_xy", (end - start), 3);


	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	Copter::supt->setCurProcessResult("set_wp_destination", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    wp_nav.set_wp_destination(stopping_point);
	end = clock();
	Copter::supt->setCurProcessResult("set_wp_destination", end, 2);
	Copter::supt->setCurProcessResult("set_wp_destination", (end - start), 3);

    // initialise yaw
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	Copter::supt->setCurProcessResult("set_auto_yaw_mode", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    set_auto_yaw_mode(get_default_auto_yaw_mode(false));
	end = clock();
	Copter::supt->setCurProcessResult("set_auto_yaw_mode", end, 2);
	Copter::supt->setCurProcessResult("set_auto_yaw_mode", (end - start), 3);
}

// initialise guided mode's velocity controller
void Copter::guided_vel_control_start()
{
	long start, end;
    // set guided_mode to velocity controller
    guided_mode = Guided_Velocity;

    // initialize vertical speeds and leash lengths
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	Copter::supt->setCurProcessResult("set_speed_z", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
	end = clock();
	Copter::supt->setCurProcessResult("set_speed_z", end, 2);
	Copter::supt->setCurProcessResult("set_speed_z", (end - start), 3);


	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	Copter::supt->setCurProcessResult("set_accel_z", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    pos_control.set_accel_z(g.pilot_accel_z);
	end = clock();
	Copter::supt->setCurProcessResult("set_accel_z", end, 2);
	Copter::supt->setCurProcessResult("set_accel_z", (end - start), 3);

    // initialise velocity controller
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	Copter::supt->setCurProcessResult("init_vel_controller_xyz", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    pos_control.init_vel_controller_xyz();
	end = clock();
	Copter::supt->setCurProcessResult("init_vel_controller_xyz", end, 2);
	Copter::supt->setCurProcessResult("init_vel_controller_xyz", (end - start), 3);
}

// initialise guided mode's posvel controller
void Copter::guided_posvel_control_start()
{
	long start, end;
    // set guided_mode to velocity controller
    guided_mode = Guided_PosVel;

	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	Copter::supt->setCurProcessResult("init_xy_controller", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    pos_control.init_xy_controller();
	end = clock();
	Copter::supt->setCurProcessResult("init_xy_controller", end, 2);
	Copter::supt->setCurProcessResult("init_xy_controller", (end - start), 3);

    // set speed and acceleration from wpnav's speed and acceleration
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	Copter::supt->setCurProcessResult("set_speed_xy", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    pos_control.set_speed_xy(wp_nav.get_speed_xy());
	end = clock();
	Copter::supt->setCurProcessResult("set_speed_xy", end, 2);
	Copter::supt->setCurProcessResult("set_speed_xy", (end - start), 3);

	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	Copter::supt->setCurProcessResult("set_accel_xy", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    pos_control.set_accel_xy(wp_nav.get_wp_acceleration());
	end = clock();
	Copter::supt->setCurProcessResult("set_accel_xy", end, 2);
	Copter::supt->setCurProcessResult("set_accel_xy", (end - start), 3);

    const Vector3f& curr_pos = inertial_nav.get_position();
    const Vector3f& curr_vel = inertial_nav.get_velocity();

    // set target position and velocity to current position and velocity
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	Copter::supt->setCurProcessResult("set_xy_target", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    pos_control.set_xy_target(curr_pos.x, curr_pos.y);
	end = clock();
	Copter::supt->setCurProcessResult("set_xy_target", end, 2);
	Copter::supt->setCurProcessResult("set_xy_target", (end - start), 3);

	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	Copter::supt->setCurProcessResult("set_desired_velocity_xy", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    pos_control.set_desired_velocity_xy(curr_vel.x, curr_vel.y);
	end = clock();
	Copter::supt->setCurProcessResult("set_desired_velocity_xy", end, 2);
	Copter::supt->setCurProcessResult("set_desired_velocity_xy", (end - start), 3);

    // set vertical speed and acceleration
    pos_control.set_speed_z(wp_nav.get_speed_down(), wp_nav.get_speed_up());
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	Copter::supt->setCurProcessResult("set_accel_z", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    pos_control.set_accel_z(wp_nav.get_accel_z());
	end = clock();
	Copter::supt->setCurProcessResult("set_accel_z", end, 2);
	Copter::supt->setCurProcessResult("set_accel_z", (end - start), 3);

    // pilot always controls yaw
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	Copter::supt->setCurProcessResult("set_auto_yaw_mode", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    set_auto_yaw_mode(AUTO_YAW_HOLD);
	end = clock();
	Copter::supt->setCurProcessResult("set_auto_yaw_mode", end, 2);
	Copter::supt->setCurProcessResult("set_auto_yaw_mode", (end - start), 3);
}

// initialise guided mode's angle controller
void Copter::guided_angle_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = Guided_Angle;

    // set vertical speed and acceleration
    pos_control.set_speed_z(wp_nav.get_speed_down(), wp_nav.get_speed_up());
    pos_control.set_accel_z(wp_nav.get_accel_z());

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    // initialise targets
    guided_angle_state.update_time_ms = millis();
    guided_angle_state.roll_cd = ahrs.roll_sensor;
    guided_angle_state.pitch_cd = ahrs.pitch_sensor;
    guided_angle_state.yaw_cd = ahrs.yaw_sensor;
    guided_angle_state.climb_rate_cms = 0.0f;

    // pilot always controls yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// guided_set_destination - sets guided mode's target destination
void Copter::guided_set_destination(const Vector3f& destination)
{
	long start, end;
    // ensure we are in position control mode
    if (guided_mode != Guided_WP) {
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		Copter::supt->setCurProcessResult("guided_pos_control_start", start, 1);

		// ------------------------  ²å×®¼¤Àø ---------------------------------
        guided_pos_control_start();
		end = clock();
		Copter::supt->setCurProcessResult("guided_pos_control_start", end, 2);
		Copter::supt->setCurProcessResult("guided_pos_control_start", (end - start), 3);
    }
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	Copter::supt->setCurProcessResult("set_wp_destination", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    wp_nav.set_wp_destination(destination);
	end = clock();
	Copter::supt->setCurProcessResult("set_wp_destination", end, 2);
	Copter::supt->setCurProcessResult("set_wp_destination", (end - start), 3);
}

// guided_set_velocity - sets guided mode's target velocity
void Copter::guided_set_velocity(const Vector3f& velocity)
{
	long start, end;
    // check we are in velocity control mode
    if (guided_mode != Guided_Velocity) {
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		Copter::supt->setCurProcessResult("guided_vel_control_start", start, 1);

		// ------------------------  ²å×®¼¤Àø ---------------------------------
        guided_vel_control_start();
		end = clock();
		Copter::supt->setCurProcessResult("guided_vel_control_start", end, 2);
		Copter::supt->setCurProcessResult("guided_vel_control_start", (end - start), 3);
    }

    vel_update_time_ms = millis();

    // set position controller velocity target
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	Copter::supt->setCurProcessResult("set_desired_velocity", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    pos_control.set_desired_velocity(velocity);
	end = clock();
	Copter::supt->setCurProcessResult("set_desired_velocity", end, 2);
	Copter::supt->setCurProcessResult("set_desired_velocity", (end - start), 3);
}

// set guided mode posvel target
void Copter::guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity) {
    // check we are in velocity control mode
	long start, end;
    if (guided_mode != Guided_PosVel) {
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		Copter::supt->setCurProcessResult("guided_posvel_control_start", start, 1);

		// ------------------------  ²å×®¼¤Àø ---------------------------------
		guided_posvel_control_start();
		end = clock();
		Copter::supt->setCurProcessResult("guided_posvel_control_start", end, 2);
		Copter::supt->setCurProcessResult("guided_posvel_control_start", (end - start), 3);
    }
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	Copter::supt->setCurProcessResult("millis", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    posvel_update_time_ms = millis();
	end = clock();
	Copter::supt->setCurProcessResult("millis", end, 2);
	Copter::supt->setCurProcessResult("millis", (end - start), 3);

    posvel_pos_target_cm = destination;
    posvel_vel_target_cms = velocity;

	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	Copter::supt->setCurProcessResult("set_pos_target", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
    pos_control.set_pos_target(posvel_pos_target_cm);
	end = clock();
	Copter::supt->setCurProcessResult("set_pos_target", end, 2);
	Copter::supt->setCurProcessResult("set_pos_target", (end - start), 3);
}

// set guided mode angle target
void Copter::guided_set_angle(const Quaternion &q, float climb_rate_cms)
{
    // check we are in velocity control mode
    if (guided_mode != Guided_Angle) {
        guided_angle_control_start();
    }

    // convert quaternion to euler angles
    q.to_euler(guided_angle_state.roll_cd, guided_angle_state.pitch_cd, guided_angle_state.yaw_cd);
    guided_angle_state.roll_cd = ToDeg(guided_angle_state.roll_cd) * 100.0f;
    guided_angle_state.pitch_cd = ToDeg(guided_angle_state.pitch_cd) * 100.0f;
    guided_angle_state.yaw_cd = wrap_180_cd_float(ToDeg(guided_angle_state.yaw_cd) * 100.0f);

    guided_angle_state.climb_rate_cms = climb_rate_cms;
    guided_angle_state.update_time_ms = millis();
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void Copter::guided_run()
{
    // call the correct auto controller
	long start, end;
	
	int b=5;
	//FixÐÞ¸ÄV1.6
	string processNames[] = { "guided_takeoff_run", "guided_pos_control_run", "guided_vel_control_run", "guided_posvel_control_run", "guided_angle_control_run"};
	for (int i = 0; i<5; i++){
		if (supt->getCurrentTestCase()->getParamValueWithNameAndKey(processNames[i], "guided_mode") != ""){
			b = i;
			break;
		}
	}

	switch (b){
		case 0:
			guided_mode = Guided_TakeOff;
			break;
		case 1:
			guided_mode = Guided_WP;
			break;
		case 2:
			guided_mode = Guided_Velocity;
			break;
		case 3:
			guided_mode = Guided_PosVel;
			break;
		case 4:
			guided_mode = Guided_Angle;
			break;
		default:
			break;
	}
	cout << "GUIDED_MODE:" << b << endl;
    switch (guided_mode) {

		case Guided_TakeOff:
			// run takeoff controller

			// ------------------------  ²å×®µã ---------------------------------
			start = clock();
			this->supt->setCurProcessResult("guided_takeoff_run", start, 1);
			// ------------------------  ²å×®¼¤Àø ---------------------------------
			guided_takeoff_run();
			end = clock();
			this->supt->setCurProcessResult("guided_takeoff_run", end, 2);
			this->supt->setCurProcessResult("guided_takeoff_run", (end - start), 3);
		
			break;

		case Guided_WP:
			// ------------------------  ²å×®µã ---------------------------------
			start = clock();
			this->supt->setCurProcessResult("guided_pos_control_run", start, 1);
			// ------------------------  ²å×®¼¤Àø ---------------------------------
			// run position controller
			guided_pos_control_run();
			end = clock();
			this->supt->setCurProcessResult("guided_pos_control_run", end, 2);
			this->supt->setCurProcessResult("guided_pos_control_run", (end - start), 3);
			break;

		case Guided_Velocity:
			// ------------------------  ²å×®µã ---------------------------------
			start = clock();
			this->supt->setCurProcessResult("guided_vel_control_run", start, 1);
			// ------------------------  ²å×®¼¤Àø ---------------------------------
			// run velocity controller
			guided_vel_control_run();
			end = clock();
			this->supt->setCurProcessResult("guided_vel_control_run", end, 2);
			this->supt->setCurProcessResult("guided_vel_control_run", (end - start), 3);
			break;

		case Guided_PosVel:
			// ------------------------  ²å×®µã ---------------------------------
			start = clock();
			this->supt->setCurProcessResult("guided_posvel_control_run", start, 1);
			// ------------------------  ²å×®¼¤Àø ---------------------------------
			// run position-velocity controller
			guided_posvel_control_run();
			end = clock();
			this->supt->setCurProcessResult("guided_posvel_control_run", end, 2);
			this->supt->setCurProcessResult("guided_posvel_control_run", (end - start), 3);
			break;

		case Guided_Angle:
			// ------------------------  ²å×®µã ---------------------------------
			start = clock();
			this->supt->setCurProcessResult("guided_angle_control_run", start, 1);
			// ------------------------  ²å×®¼¤Àø ---------------------------------
			// run angle controller
			guided_angle_control_run();
			end = clock();
			this->supt->setCurProcessResult("guided_angle_control_run", end, 2);
			this->supt->setCurProcessResult("guided_angle_control_run", (end - start), 3);
			break;
    }
 }

// guided_takeoff_run - takeoff in guided mode
//      called by guided_run at 100hz or more
void Copter::guided_takeoff_run()
{
	long start, end;
	cout << "guided_takeoff_run begin" << endl;
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
	//FixÐÞ¸Ä1.8
	start = clock();
	this->supt->setCurProcessResult("get_interlock", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
	bool get_interlock = motors.get_interlock();
	
	end = clock();
	this->supt->setCurProcessResult("get_interlock", end, 2);
	this->supt->setCurProcessResult("get_interlock", (end - start), 3);

	//FixÐÞ¸ÄV1.8
	ap.auto_armed = supt->getParamValueWithNameAndKey("update_wpnav", "ap.auto_armed");
	get_interlock = supt->getParamValueWithNameAndKey("update_wpnav", "get_interlock");


	if (ap.auto_armed==0 || get_interlock==false) {
    //if (!ap.auto_armed || !motors.get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("set_throttle_out_unstabilized", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
		attitude_control.set_throttle_out_unstabilized(0, true, g.throttle_filt);
		end = clock();
		this->supt->setCurProcessResult("set_throttle_out_unstabilized", end, 2);
		this->supt->setCurProcessResult("set_throttle_out_unstabilized", (end - start), 3);
        
#endif
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;

	//FixÐÞ¸ÄV1.8
	failsafe.radio = true;
	if (supt->getParamValueWithNameAndKey("get_pilot_desired_yaw_rate", "failsafe.radio") == 0){
		failsafe.radio = false;
	}
	if (failsafe.radio == 0) {
 //   if (!failsafe.radio) {
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
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("update_wpnav", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
    // run waypoint controller
    wp_nav.update_wpnav();
	end = clock();
	this->supt->setCurProcessResult("update_wpnav", end, 2);
	this->supt->setCurProcessResult("update_wpnav", (end - start), 3);

	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("update_z_controller", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();
	end = clock();
	this->supt->setCurProcessResult("update_z_controller", end, 2);
	this->supt->setCurProcessResult("update_z_controller", (end - start), 3);

	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
	end = clock();
	this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", end, 2);
	this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", (end - start), 3);
}

// guided_pos_control_run - runs the guided position controller
// called from guided_run
void Copter::guided_pos_control_run()
{
	long start, end;
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("get_interlock", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
	bool get_interlock = motors.get_interlock();
	end = clock();
	this->supt->setCurProcessResult("get_interlock", end, 2);
	this->supt->setCurProcessResult("get_interlock", (end - start), 3);

	//FixÐÞ¸ÄV1.6
	ap.auto_armed = supt->getParamValueWithNameAndKey("update_wpnav","ap.auto_armed");
	get_interlock = supt->getParamValueWithNameAndKey("update_wpnav", "get_interlock");
	ap.land_complete = supt->getParamValueWithNameAndKey("update_wpnav", "ap.land_complete");

	if (ap.auto_armed==0 || get_interlock==false || ap.land_complete==1) {
   // if (!ap.auto_armed || !motors.get_interlock() || ap.land_complete) {
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
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;

	//FixÐÞ¸ÄV1.6
	failsafe.radio = true;
	if (supt->getParamValueWithNameAndKey("get_pilot_desired_yaw_rate","failsafe.radio") == 0){
		failsafe.radio = false;
	}
	if (failsafe.radio == 0) {
	//if (!failsafe.radio) {
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("get_pilot_desired_yaw_rate", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
		end = clock();
		this->supt->setCurProcessResult("get_pilot_desired_yaw_rate", end, 2);
		this->supt->setCurProcessResult("get_pilot_desired_yaw_rate", (end - start), 3);
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("update_wpnav", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
    // run waypoint controller
    wp_nav.update_wpnav();
	end = clock();
	this->supt->setCurProcessResult("update_wpnav", end, 2);
	this->supt->setCurProcessResult("update_wpnav", (end - start), 3);

	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("update_z_controller", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();
	end = clock();
	this->supt->setCurProcessResult("update_z_controller", end, 2);
	this->supt->setCurProcessResult("update_z_controller", (end - start), 3);
	
	//FixÐÞ¸Ä1.7
	auto_yaw_mode = 0;
	string tmp[] = {"2","angle_ef_roll_pitch_rate_ef_yaw","angle_ef_roll_pitch_yaw"};
	auto_yaw_mode = supt->getParamValueFormNamesWithKey(tmp,"auto_yaw_mode");

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
		end = clock();
		this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", end, 2);
		this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", (end - start), 3);
    }else{
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("angle_ef_roll_pitch_yaw", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(), true);
		end = clock();
		this->supt->setCurProcessResult("angle_ef_roll_pitch_yaw", end, 2);
		this->supt->setCurProcessResult("angle_ef_roll_pitch_yaw", (end - start), 3);
    }
}

// guided_vel_control_run - runs the guided velocity controller
// called from guided_run
void Copter::guided_vel_control_run()
{
	long start, end;
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("get_interlock", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
	bool get_interlock = motors.get_interlock();
	end = clock();
	this->supt->setCurProcessResult("get_interlock", end, 2);
	this->supt->setCurProcessResult("get_interlock", (end - start), 3);

	//FixÐÞ¸ÄV1.8
	string str[] = { "3", "init_vel_controller_xyz", "get_pilot_desired_yaw_rate","millis"};
	ap.auto_armed = supt->getParamValueFormNamesWithKey(str, "ap.auto_armed");
	get_interlock = supt->getParamValueFormNamesWithKey(str, "get_interlock");
	ap.land_complete = supt->getParamValueFormNamesWithKey(str, "ap.land_complete");

	if (ap.auto_armed == 0 || get_interlock == false || ap.land_complete == 1) {
	// if (!ap.auto_armed || !motors.get_interlock() || ap.land_complete) {
        // initialise velocity controller
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("init_vel_controller_xyz", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        pos_control.init_vel_controller_xyz();
		end = clock();
		this->supt->setCurProcessResult("init_vel_controller_xyz", end, 2);
		this->supt->setCurProcessResult("init_vel_controller_xyz", (end - start), 3);

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
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;

	//FixÐÞ¸Ä1.7
	failsafe.radio = true;
	if (supt->getParamValueWithNameAndKey("get_pilot_desired_yaw_rate", "failsafe.radio") == 0)
		failsafe.radio = false;

	if (failsafe.radio == 0) {
	//if (!failsafe.radio) {
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
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set velocity to zero if no updates received for 3 seconds
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("millis", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
    uint32_t tnow = millis();
	end = clock();
	this->supt->setCurProcessResult("millis", end, 2);
	this->supt->setCurProcessResult("millis", (end - start), 3);

   // if (tnow - vel_update_time_ms > GUIDED_POSVEL_TIMEOUT_MS && !pos_control.get_desired_velocity().is_zero()) {
	bool is_zero = pos_control.get_desired_velocity().is_zero();

	//FixÐÞ¸Ä1.8
	if (supt->getParamValueWithNameAndKey("set_desired_velocity", "is_zero") != NOTFIND)
		is_zero = supt->getParamValueWithNameAndKey("set_desired_velocity", "is_zero");
	tnow = supt->getParamValueWithNameAndKey("set_desired_velocity", "is_zero");
	vel_update_time_ms = supt->getParamValueWithNameAndKey("vel_update_time_ms", "is_zero");

	if (tnow - vel_update_time_ms > GUIDED_POSVEL_TIMEOUT_MS && is_zero == false) {
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("set_desired_velocity", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
		pos_control.set_desired_velocity(Vector3f(0,0,0));
		end = clock();
		this->supt->setCurProcessResult("set_desired_velocity", end, 2);
		this->supt->setCurProcessResult("set_desired_velocity", (end - start), 3);
    }

    // calculate dt
    float dt = pos_control.time_since_last_xy_update();

    // update at poscontrol update rate
    if (dt >= pos_control.get_dt_xy()) {
        // sanity check dt
        if (dt >= 0.2f) {
            dt = 0.0f;
        }

        // call velocity controller which includes z axis controller
        pos_control.update_vel_controller_xyz(ekfNavVelGainScaler);
    }

	//FixÐÞ¸Ä1.7
	auto_yaw_mode = 0;
	string tmp[] = { "2", "angle_ef_roll_pitch_rate_ef_yaw", "angle_ef_roll_pitch_yaw" };
	auto_yaw_mode = supt->getParamValueFormNamesWithKey(tmp, "auto_yaw_mode");

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(pos_control.get_roll(), pos_control.get_pitch(), target_yaw_rate);
		end = clock();
		this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", end, 2);
		this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", (end - start), 3);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("angle_ef_roll_pitch_yaw", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        attitude_control.angle_ef_roll_pitch_yaw(pos_control.get_roll(), pos_control.get_pitch(), get_auto_heading(), true);
		end = clock();
		this->supt->setCurProcessResult("angle_ef_roll_pitch_yaw", end, 2);
		this->supt->setCurProcessResult("angle_ef_roll_pitch_yaw", (end - start), 3);
    }
}

// guided_posvel_control_run - runs the guided spline controller
// called from guided_run
void Copter::guided_posvel_control_run()
{
	long start, end;
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("get_interlock", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
	bool get_interlock = motors.get_interlock();
	end = clock();
	this->supt->setCurProcessResult("get_interlock", end, 2);
	this->supt->setCurProcessResult("get_interlock", (end - start), 3);

	//FixÐÞ¸ÄV1.8
	string str[] = { "2", "set_pos_target", "get_pilot_desired_yaw_rate" };
	ap.auto_armed = supt->getParamValueFormNamesWithKey(str, "ap.auto_armed");
	get_interlock = supt->getParamValueFormNamesWithKey(str, "get_interlock");
	ap.land_complete = supt->getParamValueFormNamesWithKey(str, "ap.land_complete");

	if (ap.auto_armed == 0 || get_interlock == false || ap.land_complete == 1) {
	//if (!ap.auto_armed || !motors.get_interlock() || ap.land_complete) {
        // set target position and velocity to current position and velocity
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("set_pos_target", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        pos_control.set_pos_target(inertial_nav.get_position());
		end = clock();
		this->supt->setCurProcessResult("set_pos_target", end, 2);
		this->supt->setCurProcessResult("set_pos_target", (end - start), 3);

		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("set_desired_velocity", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        pos_control.set_desired_velocity(Vector3f(0,0,0));
		end = clock();
		this->supt->setCurProcessResult("set_desired_velocity", end, 2);
		this->supt->setCurProcessResult("set_desired_velocity", (end - start), 3);
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
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;

	//FixÐÞ¸Ä1.8
	failsafe.radio = true;
	if (supt->getParamValueWithNameAndKey("get_pilot_desired_yaw_rate", "failsafe.radio") == 0)
		failsafe.radio = false;

	if (failsafe.radio == 0) {
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("get_pilot_desired_yaw_rate", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
		end = clock();
		this->supt->setCurProcessResult("get_pilot_desired_yaw_rate", end, 2);
		this->supt->setCurProcessResult("get_pilot_desired_yaw_rate", (end - start), 3);
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set velocity to zero if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - posvel_update_time_ms > GUIDED_POSVEL_TIMEOUT_MS && !posvel_vel_target_cms.is_zero()) {
        posvel_vel_target_cms.zero();
    }

    // calculate dt
    float dt = pos_control.time_since_last_xy_update();

    // update at poscontrol update rate
    if (dt >= pos_control.get_dt_xy()) {
        // sanity check dt
        if (dt >= 0.2f) {
            dt = 0.0f;
        }

        // advance position target using velocity target
        posvel_pos_target_cm += posvel_vel_target_cms * dt;

        // send position and velocity targets to position controller
        pos_control.set_pos_target(posvel_pos_target_cm);
        pos_control.set_desired_velocity_xy(posvel_vel_target_cms.x, posvel_vel_target_cms.y);

        // run position controller
        pos_control.update_xy_controller(AC_PosControl::XY_MODE_POS_AND_VEL_FF, ekfNavVelGainScaler, false);
    }
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("update_z_controller", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
    pos_control.update_z_controller();
	end = clock();
	this->supt->setCurProcessResult("update_z_controller", end, 2);
	this->supt->setCurProcessResult("update_z_controller", (end - start), 3);

	//FixÐÞ¸Ä1.7
	auto_yaw_mode = 0;
	string tmp[] = { "2", "angle_ef_roll_pitch_rate_ef_yaw", "angle_ef_roll_pitch_yaw" };
	auto_yaw_mode = supt->getParamValueFormNamesWithKey(tmp, "auto_yaw_mode");

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(pos_control.get_roll(), pos_control.get_pitch(), target_yaw_rate);
		end = clock();
		this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", end, 2);
		this->supt->setCurProcessResult("angle_ef_roll_pitch_rate_ef_yaw", (end - start), 3);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("angle_ef_roll_pitch_yaw", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        attitude_control.angle_ef_roll_pitch_yaw(pos_control.get_roll(), pos_control.get_pitch(), get_auto_heading(), true);
		end = clock();
		this->supt->setCurProcessResult("angle_ef_roll_pitch_yaw", end, 2);
		this->supt->setCurProcessResult("angle_ef_roll_pitch_yaw", (end - start), 3);
    }
}

// guided_angle_control_run - runs the guided angle controller
// called from guided_run
void Copter::guided_angle_control_run()
{
	long start, end;
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("get_interlock", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
	bool get_interlock = motors.get_interlock();
	end = clock();
	this->supt->setCurProcessResult("get_interlock", end, 2);
	this->supt->setCurProcessResult("get_interlock", (end - start), 3);

	//FixÐÞ¸ÄV1.7
	string str[] = { "2", "set_throttle_out_unstabilized", "get_pilot_desired_yaw_rate" };
	ap.auto_armed = supt->getParamValueFormNamesWithKey(str, "ap.auto_armed");
	get_interlock = supt->getParamValueFormNamesWithKey(str, "get_interlock");
	ap.land_complete = supt->getParamValueFormNamesWithKey(str, "ap.land_complete");

	if (ap.auto_armed == 0 || get_interlock == false || ap.land_complete == 1) {
	//if (!ap.auto_armed || !motors.get_interlock() || ap.land_complete) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0.0f, 0.0f, 0.0f, get_smoothing_gain());
        attitude_control.set_throttle_out(0.0f,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("set_throttle_out_unstabilized", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        attitude_control.set_throttle_out_unstabilized(0.0f,true,g.throttle_filt);
		end = clock();
		this->supt->setCurProcessResult("set_throttle_out_unstabilized", end, 2);
		this->supt->setCurProcessResult("set_throttle_out_unstabilized", (end - start), 3);
#endif
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		this->supt->setCurProcessResult("relax_alt_hold_controllers", start, 1);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        pos_control.relax_alt_hold_controllers(0.0f);
		end = clock();
		this->supt->setCurProcessResult("relax_alt_hold_controllers", end, 2);
		this->supt->setCurProcessResult("relax_alt_hold_controllers", (end - start), 3);
       // return;
    }

    // constrain desired lean angles
    float roll_in = guided_angle_state.roll_cd;
    float pitch_in = guided_angle_state.pitch_cd;
    float total_in = pythagorous2(roll_in, pitch_in);
    float angle_max = minf(attitude_control.get_althold_lean_angle_max(), aparm.angle_max);
    if (total_in > angle_max) {
        float ratio = angle_max / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // wrap yaw request
    float yaw_in = wrap_180_cd_float(guided_angle_state.yaw_cd);

    // constrain climb rate
    float climb_rate_cms = constrain_float(guided_angle_state.climb_rate_cms, -fabs(wp_nav.get_speed_down()), wp_nav.get_speed_up());

    // check for timeout - set lean angles and climb rate to zero if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - guided_angle_state.update_time_ms > GUIDED_ATTITUDE_TIMEOUT_MS) {
        roll_in = 0.0f;
        pitch_in = 0.0f;
        climb_rate_cms = 0.0f;
    }

    // call attitude controller
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("angle_ef_roll_pitch_yaw", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
    attitude_control.angle_ef_roll_pitch_yaw(roll_in, pitch_in, yaw_in, true);
	end = clock();
	this->supt->setCurProcessResult("angle_ef_roll_pitch_yaw", end, 2);
	this->supt->setCurProcessResult("angle_ef_roll_pitch_yaw", (end - start), 3);

    // call position controller
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("set_alt_target_from_climb_rate", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
    pos_control.set_alt_target_from_climb_rate(climb_rate_cms, G_Dt, false);
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

// Guided Limit code

// guided_limit_clear - clear/turn off guided limits
void Copter::guided_limit_clear()
{
    guided_limit.timeout_ms = 0;
    guided_limit.alt_min_cm = 0.0f;
    guided_limit.alt_max_cm = 0.0f;
    guided_limit.horiz_max_cm = 0.0f;
}

// guided_limit_set - set guided timeout and movement limits
void Copter::guided_limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm)
{
    guided_limit.timeout_ms = timeout_ms;
    guided_limit.alt_min_cm = alt_min_cm;
    guided_limit.alt_max_cm = alt_max_cm;
    guided_limit.horiz_max_cm = horiz_max_cm;
}

// guided_limit_init_time_and_pos - initialise guided start time and position as reference for limit checking
//  only called from AUTO mode's auto_nav_guided_start function
void Copter::guided_limit_init_time_and_pos()
{
    // initialise start time
    guided_limit.start_time = hal.scheduler->millis();

    // initialise start position from current position
    guided_limit.start_pos = inertial_nav.get_position();
}

// guided_limit_check - returns true if guided mode has breached a limit
//  used when guided is invoked from the NAV_GUIDED_ENABLE mission command
bool Copter::guided_limit_check()
{
    // check if we have passed the timeout
    if ((guided_limit.timeout_ms > 0) && (millis() - guided_limit.start_time >= guided_limit.timeout_ms)) {
        return true;
    }

    // get current location
    const Vector3f& curr_pos = inertial_nav.get_position();

    // check if we have gone below min alt
    if (!is_zero(guided_limit.alt_min_cm) && (curr_pos.z < guided_limit.alt_min_cm)) {
        return true;
    }

    // check if we have gone above max alt
    if (!is_zero(guided_limit.alt_max_cm) && (curr_pos.z > guided_limit.alt_max_cm)) {
        return true;
    }

    // check if we have gone beyond horizontal limit
    if (guided_limit.horiz_max_cm > 0.0f) {
        float horiz_move = pv_get_horizontal_distance_cm(guided_limit.start_pos, curr_pos);
        if (horiz_move > guided_limit.horiz_max_cm) {
            return true;
        }
    }

    // if we got this far we must be within limits
    return false;
}
