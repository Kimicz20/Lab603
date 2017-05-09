#include "Copter.h"
#include "../AP_Scheduler/AP_Scheduler.h"
#include "../AP_HAL/functor.h"

//#define SCHED_TASK(func, _interval_ticks, _max_time_micros) {\
//    .function = FUNCTOR_BIND(&copter, &Copter::func, void),\
//    AP_SCHEDULER_NAME_INITIALIZER(func)\
//    .interval_ticks = _interval_ticks,\
//    .max_time_micros = _max_time_micros,\
//}
#define SCHED_TASK(func, _interval_ticks, _max_time_micros) {\
    FUNCTOR_BIND(&copter, &Copter::func, void),\
    AP_SCHEDULER_NAME_INITIALIZER(func)\
    _interval_ticks,\
    _max_time_micros\
}
// 
/*
scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
should be listed here, along with how often they should be called
(in 2.5ms units) and the maximum time they are expected to take (in
microseconds)
1    = 400hz
2    = 200hz
4    = 100hz
8    = 50hz
20   = 20hz
40   = 10hz
133  = 3hz
400  = 1hz
4000 = 0.1hz
*/
//
//const AP_Scheduler::Task Copter::task = { FUNCTOR_BIND(&copter, &Copter::rc_loop, void), AP_SCHEDULER_NAME_INITIALIZER(rc_loop)  4, 130 };  //SCHED_TASK(rc_loop, 4, 130)£»

const AP_Scheduler::Task Copter::scheduler_tasks[] = {
	SCHED_TASK(rc_loop, 4, 130),
	//    SCHED_TASK(throttle_loop,          8,     75),
	SCHED_TASK(update_GPS, 8, 200),
	//#if OPTFLOW == ENABLED
	//    SCHED_TASK(update_optical_flow,    2,    160),
	//#endif
	SCHED_TASK(update_batt_compass, 40, 120),
	//    SCHED_TASK(read_aux_switches,     40,     50),
	SCHED_TASK(arm_motors_check, 40, 50),
	//    SCHED_TASK(auto_trim,             40,     75),
	SCHED_TASK(update_altitude, 40, 140),
	//    SCHED_TASK(run_nav_updates,        8,    100),
	//    SCHED_TASK(update_thr_average,     4,     90),
	SCHED_TASK(three_hz_loop, 133, 75),
	SCHED_TASK(compass_accumulate, 4, 100),
	SCHED_TASK(barometer_accumulate, 8, 90),
	//#if PRECISION_LANDING == ENABLED
	//    SCHED_TASK(update_precland,        8,     50),
	//#endif
	//#if FRAME_CONFIG == HELI_FRAME
	//    SCHED_TASK(check_dynamic_flight,   8,     75),
	//#endif
	//    SCHED_TASK(update_notify,          8,     90),
	SCHED_TASK(one_hz_loop, 400, 100),
	//    SCHED_TASK(ekf_check,             40,     75),
	//    SCHED_TASK(landinggear_update,    40,     75),
	//SCHED_TASK(lost_vehicle_check,    40,     50),
	SCHED_TASK(gcs_check_input, 1, 180),
	//    SCHED_TASK(gcs_send_heartbeat,   400,    110),
	//    SCHED_TASK(gcs_send_deferred,      8,    550),
	SCHED_TASK(gcs_data_stream_send, 8, 550),
	//    SCHED_TASK(update_mount,           8,     75),
	//    SCHED_TASK(ten_hz_logging_loop,   40,    350),
	//    SCHED_TASK(fifty_hz_logging_loop,  8,    110),
	//    SCHED_TASK(full_rate_logging_loop, 1,    100),
	//    SCHED_TASK(dataflash_periodic,     1,    300),
	//    SCHED_TASK(perf_update,         4000,     75),
	//    SCHED_TASK(read_receiver_rssi,    40,     75),
	//    SCHED_TASK(rpm_update,            40,    200),
	//    SCHED_TASK(compass_cal_update,    4,    100),
	//#if FRSKY_TELEM_ENABLED == ENABLED
	//    SCHED_TASK(frsky_telemetry_send,  80,     75),
	//#endif
	//#if EPM_ENABLED == ENABLED
	//    SCHED_TASK(epm_update,            40,     75),
	//#endif
	//#ifdef USERHOOK_FASTLOOP
	//    SCHED_TASK(userhook_FastLoop,      4,     75),
	//#endif
	//#ifdef USERHOOK_50HZLOOP
	//    SCHED_TASK(userhook_50Hz,          8,     75),
	//#endif
	//#ifdef USERHOOK_MEDIUMLOOP
	//    SCHED_TASK(userhook_MediumLoop,   40,     75),
	//#endif
	//#ifdef USERHOOK_SLOWLOOP
	//    SCHED_TASK(userhook_SlowLoop,     120,    75),
	//#endif
	//#ifdef USERHOOK_SUPERSLOWLOOP
	//   SCHED_TASK(userhook_SuperSlowLoop, 400,   75),
	//#endif  
};

//AP_Scheduler scheduler;

void Copter::setup()
{
	cout << "---- set up begin ----" << endl;
	long start, end;
	cliSerial = hal.console;

	// Load the default values of variables listed in var_info[]s
	AP_Param::setup_sketch_defaults();

	// setup storage layout for copter
	StorageManager::set_layout_copter();

	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	supt->setCurProcessResult("init_ardupilot", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
	init_ardupilot();

	end = clock();
	supt->setCurProcessResult("init_ardupilot", end, 2);
	supt->setCurProcessResult("init_ardupilot", (end - start), 3);

	// initialise the main loop scheduler
	scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));

	// setup initial performance counters
	perf_info_reset();
	fast_loopTimer = hal.scheduler->micros();
	cout << "---- set up end ----" << endl;
}

/*
if the compass is enabled then try to accumulate a reading
*/
void Copter::compass_accumulate(void)
{
	long start, end;
	if (g.compass_enabled) {
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		supt->setCurProcessResult("accumulate", start, 1);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
		compass.accumulate();
		end = clock();
		supt->setCurProcessResult("accumulate", end, 2);
		supt->setCurProcessResult("accumulate", (end - start), 3);
	}
}

/*
try to accumulate a baro reading
*/
void Copter::barometer_accumulate(void)
{
	barometer.accumulate();
}

void Copter::perf_update(void)
{
	if (should_log(MASK_LOG_PM))
		Log_Write_Performance();
	if (scheduler.debug()) {
		// gcs_send_text_fmt(PSTR("PERF: %u/%u %lu %lu\n"),
		//   (unsigned)perf_info_get_num_long_running(),
		//   (unsigned)perf_info_get_num_loops(),
		//   (unsigned long)perf_info_get_max_time(),
		//   (unsigned long)perf_info_get_min_time());
	}
	perf_info_reset();
	pmTest1 = 0;
}

void Copter::loop()
{
	cout << "---- loop begin ----" << endl;
	long end, start;
	// wait for an INS sample
	//ins.wait_for_sample(); 

	// ------------------------  ²å×®¼¤Àø ---------------------------------
	uint32_t timer = micros();

	// check loop time
	perf_info_check_loop_time(timer - fast_loopTimer);

	// used by PI Loops
	G_Dt = (float)(timer - fast_loopTimer) / 1000000.0f;
	fast_loopTimer = timer;

	// for mainloop failure monitoring
	mainLoop_count++;

	// Execute the fast loop

	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("fast_loop", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
	fast_loop();

	end = clock();
	this->supt->setCurProcessResult("fast_loop", end, 2);
	this->supt->setCurProcessResult("fast_loop", (end - start), 3);


	// tell the scheduler one tick has passed

	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("tick", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
	scheduler.tick();

	end = clock();
	this->supt->setCurProcessResult("tick", end, 2);
	this->supt->setCurProcessResult("tick", (end - start), 3);

	// run all the tasks that are due to run. Note that we only
	// have to call this once per loop, as the tasks are scheduled
	// in multiples of the main loop tick. So if they don't run on
	// the first call to the scheduler they won't run on a later
	// call until scheduler.tick() is called again

	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("micros", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
	uint32_t time_available = (timer + MAIN_LOOP_MICROS) - micros();

	end = clock();
	this->supt->setCurProcessResult("micros", end, 2);
	this->supt->setCurProcessResult("micros", (end - start), 3);

	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("run", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
	scheduler.run(time_available);
	end = clock();
	this->supt->setCurProcessResult("run", end, 2);
	this->supt->setCurProcessResult("run", (end - start), 3);

	//while (1);
	cout << "---- loop end ----" << endl;
}


// Main loop - 400hz
void Copter::fast_loop()
{
	cout << "---- fast_loop begin ----" << endl;
	long start, end;
	// IMU DCM Algorithm
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("read_AHRS", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
	read_AHRS();

	end = clock();
	this->supt->setCurProcessResult("read_AHRS", end, 2);
	this->supt->setCurProcessResult("read_AHRS", (end - start), 3);

	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("rate_controller_run", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
	// run low level rate controllers that only require IMU data
	//FixÐÞ¸ÄV1.3
	attitude_control.supt = supt;
	attitude_control.rate_controller_run();

	end = clock();
	this->supt->setCurProcessResult("rate_controller_run", end, 2);
	this->supt->setCurProcessResult("rate_controller_run", (end - start), 3);

#if FRAME_CONFIG == HELI_FRAME
	update_heli_control_dynamics();
#endif //HELI_FRAME

	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("motors_output", start, 1);
	// ------------------------  ²å×®¼¤Àø ---------------------------------
	// send outputs to the motors library
	motors_output();

	end = clock();
	this->supt->setCurProcessResult("motors_output", end, 2);
	this->supt->setCurProcessResult("motors_output", (end - start), 3);

	// Inertial Nav
	// --------------------
	read_inertia();

	// check if ekf has reset target heading
	check_ekf_yaw_reset();

	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("update_flight_mode", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
	// run the attitude controllers
	update_flight_mode();

	end = clock();
	this->supt->setCurProcessResult("update_flight_mode", end, 2);
	this->supt->setCurProcessResult("update_flight_mode", (end - start), 3);

	// update home from EKF if necessary
	update_home_from_EKF();

	// check if we've landed or crashed
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	supt->setCurProcessResult("update_land_and_crash_detectors", start, 1);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
	update_land_and_crash_detectors();
	end = clock();
	supt->setCurProcessResult("update_land_and_crash_detectors", end, 2);
	supt->setCurProcessResult("update_land_and_crash_detectors", (end - start), 3);

	// log sensor health
	/* if (should_log(MASK_LOG_ANY)) {
	Log_Sensor_Health();
	}*/
	cout << "---- fast_loop end ----" << endl;
}

// rc_loops - reads user input from transmitter/receiver
// called at 100hz
void Copter::rc_loop()
{
	// Read radio and 3-position switch on radio
	// ------------------------  ²å×®µã ---------------------------------
	long start, end;
	start = clock();
	this->supt->setCurProcessResult("read_radio", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
	read_radio();
	end = clock();
	this->supt->setCurProcessResult("read_radio", end, 2);
	this->supt->setCurProcessResult("read_radio", (end - start), 3);

	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("read_control_switch", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
	read_control_switch();
	end = clock();
	this->supt->setCurProcessResult("read_control_switch", end, 2);
	this->supt->setCurProcessResult("read_control_switch", (end - start), 3);
}

// throttle_loop - should be run at 50 hz
// ---------------------------
void Copter::throttle_loop()
{
	// get altitude and climb rate from inertial lib
	read_inertial_altitude();

	// update throttle_low_comp value (controls priority of throttle vs attitude control)
	update_throttle_thr_mix();

	// check auto_armed status
	update_auto_armed();

#if FRAME_CONFIG == HELI_FRAME
	// update rotor speed
	heli_update_rotor_speed_targets();

	// update trad heli swash plate movement
	heli_update_landing_swash();
#endif
}

// update_mount - update camera mount position
// should be run at 50hz
//void Copter::update_mount()
//{
//#if MOUNT == ENABLED
// update camera mount's position
// camera_mount.update();
//#endif

//#if CAMERA == ENABLED
// camera.trigger_pic_cleanup();
//#endif
//}

// update_batt_compass - read battery and compass
// should be called at 10hz
void Copter::update_batt_compass(void)
{
	// read battery before compass because it may be used for motor interference compensation
	long start, end;
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	supt->setCurProcessResult("read_battery", start, 1);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
	read_battery();
	end = clock();
	supt->setCurProcessResult("read_battery", end, 2);
	supt->setCurProcessResult("read_battery", (end - start), 3);
	if (g.compass_enabled) {
		// update compass with throttle value - used for compassmot
		compass.set_throttle(motors.get_throttle() / 1000.0f);
		compass.read();
		// log compass information
		if (should_log(MASK_LOG_COMPASS)) {
			//DataFlash.Log_Write_Compass(compass);
		}
	}
}

// ten_hz_logging_loop
// should be run at 10hz
void Copter::ten_hz_logging_loop()
{
//	// log attitude data if we're not already logging at the higher rate
//	if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
//		//Log_Write_Attitude();
//		//Log_Write_Rate();
//		if (should_log(MASK_LOG_PID)) {
//			/* DataFlash.Log_Write_PID(LOG_PIDR_MSG, g.pid_rate_roll.get_pid_info() );
//			DataFlash.Log_Write_PID(LOG_PIDP_MSG, g.pid_rate_pitch.get_pid_info() );
//			DataFlash.Log_Write_PID(LOG_PIDY_MSG, g.pid_rate_yaw.get_pid_info() );
//			DataFlash.Log_Write_PID(LOG_PIDA_MSG, g.pid_accel_z.get_pid_info() );*/
//		}
//	}
//	if (should_log(MASK_LOG_MOTBATT)) {
//		// Log_Write_MotBatt();
//	}
//	if (should_log(MASK_LOG_RCIN)) {
//		DataFlash.Log_Write_RCIN();
//		if (rssi.enabled()) {
//			DataFlash.Log_Write_RSSI(rssi);
//		}
//	}
//	if (should_log(MASK_LOG_RCOUT)) {
//		DataFlash.Log_Write_RCOUT();
//	}
//	if (should_log(MASK_LOG_NTUN) && (mode_requires_GPS(control_mode) || landing_with_GPS())) {
//		Log_Write_Nav_Tuning();
//	}
//	if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
//		DataFlash.Log_Write_Vibration(ins);
//	}
//#if FRAME_CONFIG == HELI_FRAME
//	Log_Write_Heli();
//#endif
}

// fifty_hz_logging_loop
// should be run at 50hz
void Copter::fifty_hz_logging_loop()
{
//#if HIL_MODE != HIL_MODE_DISABLED
//	// HIL for a copter needs very fast update of the servo values
//	//gcs_send_message(MSG_RADIO_OUT);
//#endif
//
//#if HIL_MODE == HIL_MODE_DISABLED
//	if (should_log(MASK_LOG_ATTITUDE_FAST)) {
//		Log_Write_Attitude();
//		Log_Write_Rate();
//		if (should_log(MASK_LOG_PID)) {
//			/*DataFlash.Log_Write_PID(LOG_PIDR_MSG, g.pid_rate_roll.get_pid_info() );
//			DataFlash.Log_Write_PID(LOG_PIDP_MSG, g.pid_rate_pitch.get_pid_info() );
//			DataFlash.Log_Write_PID(LOG_PIDY_MSG, g.pid_rate_yaw.get_pid_info() );
//			DataFlash.Log_Write_PID(LOG_PIDA_MSG, g.pid_accel_z.get_pid_info() );*/
//		}
//	}
//
//	// log IMU data if we're not already logging at the higher rate
//	if (should_log(MASK_LOG_IMU) && !(should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW))) {
//		//DataFlash.Log_Write_IMU(ins);
//	}
//#endif
}

// full_rate_logging_loop
// should be run at the MAIN_LOOP_RATE
void Copter::full_rate_logging_loop()
{
	/*if (should_log(MASK_LOG_IMU_FAST) && !should_log(MASK_LOG_IMU_RAW)) {
		DataFlash.Log_Write_IMU(ins);
	}
	if (should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
		DataFlash.Log_Write_IMUDT(ins);
	}*/
}

void Copter::dataflash_periodic(void)
{
	//DataFlash.periodic_tasks();
}

// three_hz_loop - 3.3hz loop
void Copter::three_hz_loop()
{
	// check if we've lost contact with the ground station
	long start, end;
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	supt->setCurProcessResult("failsafe_gcs_check", start, 1);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
	failsafe_gcs_check();
	end = clock();
	supt->setCurProcessResult("failsafe_gcs_check", end, 2);
	supt->setCurProcessResult("failsafe_gcs_check", (end - start), 3);

#if AC_FENCE == ENABLED
	// check if we have breached a fence
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	supt->setCurProcessResult("fence_check", start, 1);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
	fence_check();
	end = clock();
	supt->setCurProcessResult("fence_check", end, 2);
	supt->setCurProcessResult("fence_check", (end - start), 3);
#endif // AC_FENCE_ENABLED

#if SPRAYER == ENABLED
	sprayer.update();
#endif
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	supt->setCurProcessResult("update_events", start, 1);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
	update_events();
	end = clock();
	supt->setCurProcessResult("update_events", end, 2);
	supt->setCurProcessResult("update_events", (end - start), 3);

	// update ch6 in flight tuning
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	supt->setCurProcessResult("tuning", start, 1);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
	tuning();
	end = clock();
	supt->setCurProcessResult("tuning", end, 2);
	supt->setCurProcessResult("tuning", (end - start), 3);
}

// one_hz_loop - runs at 1Hz
void Copter::one_hz_loop()
{
	/*if (should_log(MASK_LOG_ANY)) {
	Log_Write_Data(DATA_AP_STATE, ap.value);
	}*/

	// perform pre-arm checks & display failures every 30 seconds
	static uint8_t pre_arm_display_counter = 15;
	pre_arm_display_counter++;
	long start, end;
	if (pre_arm_display_counter >= 30) {
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		supt->setCurProcessResult("pre_arm_checks", start, 1);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
		pre_arm_checks(true);
		end = clock();
		supt->setCurProcessResult("pre_arm_checks", end, 2);
		supt->setCurProcessResult("pre_arm_checks", (end - start), 3);
		pre_arm_display_counter = 0;
	}
	else{
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		supt->setCurProcessResult("pre_arm_checks", start, 1);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
		pre_arm_checks(false);
		end = clock();
		supt->setCurProcessResult("pre_arm_checks", end, 2);
		supt->setCurProcessResult("pre_arm_checks", (end - start), 3);
	}

	// auto disarm checks
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	supt->setCurProcessResult("auto_disarm_check", start, 1);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
	auto_disarm_check();
	end = clock();
	supt->setCurProcessResult("auto_disarm_check", end, 2);
	supt->setCurProcessResult("auto_disarm_check", (end - start), 3);
	bool isarmed = motors.armed();
	if (isarmed == false) {
		//if (!motors.armed()) {
		// make it possible to change ahrs orientation at runtime during initial config
		ahrs.set_orientation();

		// check the user hasn't updated the frame orientation
		motors.set_frame_orientation(g.frame_orientation);

#if FRAME_CONFIG != HELI_FRAME
		// set all throttle channel settings
		motors.set_throttle_range(g.throttle_min, channel_throttle->radio_min, channel_throttle->radio_max);
		// set hover throttle
		motors.set_hover_throttle(g.throttle_mid);
#endif
	}

	// update assigned functions and enable auxiliary servos
	RC_Channel_aux::enable_aux_servos();

	check_usb_mux();

#if AP_TERRAIN_AVAILABLE
	terrain.update();

	// tell the rangefinder our height, so it can go into power saving
	// mode if available
#if CONFIG_SONAR == ENABLED
	float height;
	if (terrain.height_above_terrain(height, true)) {
		sonar.set_estimated_terrain_height(height);
	}
#endif
#endif

	// update position controller alt limits
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	supt->setCurProcessResult("update_poscon_alt_max", start, 1);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
	update_poscon_alt_max();
	end = clock();
	supt->setCurProcessResult("update_poscon_alt_max", end, 2);
	supt->setCurProcessResult("update_poscon_alt_max", (end - start), 3);

	// enable/disable raw gyro/accel logging
	//ins.set_raw_logging(should_log(MASK_LOG_IMU_RAW));
}

// called at 50hz
void Copter::update_GPS(void)
{
	static uint32_t last_gps_reading[GPS_MAX_INSTANCES];   // time of last gps message
	bool gps_updated = false;
	long start, end;
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("update", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
	gps.update();
	end = clock();
	this->supt->setCurProcessResult("update", end, 2);
	this->supt->setCurProcessResult("update", (end - start), 3);

	// log after every gps message
	for (uint8_t i = 0; i<gps.num_sensors(); i++) {
		if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
			last_gps_reading[i] = gps.last_message_time_ms(i);

			// log GPS message
			/*if (should_log(MASK_LOG_GPS)) {
			DataFlash.Log_Write_GPS(gps, i, current_loc.alt);
			}*/

			gps_updated = true;
		}
	}

	if (gps_updated) {
		// set system time if necessary
		set_system_time_from_GPS();

		// checks to initialise home and take location based pictures
		if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {

#if CAMERA == ENABLED
			//            if (camera.update_location(current_loc) == true) {
			//  do_take_picture();
			// }
#endif
		}
	}
}

void Copter::init_simple_bearing()
{
	// capture current cos_yaw and sin_yaw values
	simple_cos_yaw = ahrs.cos_yaw();
	simple_sin_yaw = ahrs.sin_yaw();

	// initialise super simple heading (i.e. heading towards home) to be 180 deg from simple mode heading
	super_simple_last_bearing = wrap_360_cd(ahrs.yaw_sensor + 18000);
	super_simple_cos_yaw = simple_cos_yaw;
	super_simple_sin_yaw = simple_sin_yaw;

	// log the simple bearing to dataflash
	/*if (should_log(MASK_LOG_ANY)) {
	Log_Write_Data(DATA_INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
	}*/
}

// update_simple_mode - rotates pilot input if we are in simple mode
void Copter::update_simple_mode(void)
{
	float rollx, pitchx;

	// exit immediately if no new radio frame or not in simple mode
	if (ap.simple_mode == 0 || !ap.new_radio_frame) {
		return;
	}

	// mark radio frame as consumed
	ap.new_radio_frame = false;

	if (ap.simple_mode == 1) {
		// rotate roll, pitch input by -initial simple heading (i.e. north facing)
		rollx = channel_roll->control_in*simple_cos_yaw - channel_pitch->control_in*simple_sin_yaw;
		pitchx = channel_roll->control_in*simple_sin_yaw + channel_pitch->control_in*simple_cos_yaw;
	}
	else{
		// rotate roll, pitch input by -super simple heading (reverse of heading to home)
		rollx = channel_roll->control_in*super_simple_cos_yaw - channel_pitch->control_in*super_simple_sin_yaw;
		pitchx = channel_roll->control_in*super_simple_sin_yaw + channel_pitch->control_in*super_simple_cos_yaw;
	}

	// rotate roll, pitch input from north facing to vehicle's perspective
	channel_roll->control_in = rollx*ahrs.cos_yaw() + pitchx*ahrs.sin_yaw();
	channel_pitch->control_in = -rollx*ahrs.sin_yaw() + pitchx*ahrs.cos_yaw();
}

// update_super_simple_bearing - adjusts simple bearing based on location
// should be called after home_bearing has been updated
void Copter::update_super_simple_bearing(bool force_update)
{
	// check if we are in super simple mode and at least 10m from home
	if (force_update || (ap.simple_mode == 2 && home_distance > SUPER_SIMPLE_RADIUS)) {
		// check the bearing to home has changed by at least 5 degrees
		if (labs(super_simple_last_bearing - home_bearing) > 500) {
			super_simple_last_bearing = home_bearing;
			float angle_rad = radians((super_simple_last_bearing + 18000) / 100);
			super_simple_cos_yaw = cosf(angle_rad);
			super_simple_sin_yaw = sinf(angle_rad);
		}
	}
}

void Copter::read_AHRS(void)
{
	cout << "---- read_AHRS begin ----" << endl;
	// Perform IMU calculations and get attitude info
	long start, end;
	//-----------------------------------------------
#if HIL_MODE != HIL_MODE_DISABLED
	// update hil before ahrs update 
	/*start = clock();
	this->supt->setCurProcessResult("gcs_check_input", start, 1);*/
	gcs_check_input();
	/*end = clock();
	this->supt->setCurProcessResult("gcs_check_input", end, 2);
	this->supt->setCurProcessResult("gcs_check_input", (end - start), 3);*/

#endif
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("update", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
	//FixÐÞ¸ÄV1.3
	ahrs.supt = supt;
	ahrs.update();

	end = clock();
	this->supt->setCurProcessResult("update", end, 2);
	this->supt->setCurProcessResult("update", (end - start), 3);
	cout << "---- read_AHRS end ----" << endl;
}

// read baro and sonar altitude at 10hz
void Copter::update_altitude()
{
	cout << "------ update_altitude begin -----" << endl;
	// read in baro altitude
	long start, end;
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("read_barometer", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
	read_barometer();

	end = clock();
	this->supt->setCurProcessResult("read_barometer", end, 2);
	this->supt->setCurProcessResult("read_barometer", (end - start), 3);

	// read in sonar altitude
	sonar_alt = read_sonar();

	// write altitude info to dataflash logs
	/*if (should_log(MASK_LOG_CTUN)) {
	Log_Write_Control_Tuning();
	}*/
	cout << "------ update_altitude end -----" << endl;
}
AP_HAL_MAIN_CALLBACKS(&copter);
