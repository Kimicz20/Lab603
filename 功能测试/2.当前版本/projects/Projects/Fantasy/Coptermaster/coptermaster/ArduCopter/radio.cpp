// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"


// Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

//void Copter::default_dead_zones()
//{
//    channel_roll->set_default_dead_zone(30);
//    channel_pitch->set_default_dead_zone(30);
////#if FRAME_CONFIG == HELI_FRAME
////    channel_throttle->set_default_dead_zone(10);
////    channel_yaw->set_default_dead_zone(15);
////    g.rc_8.set_default_dead_zone(10);
////#else
//    channel_throttle->set_default_dead_zone(30);
//    channel_yaw->set_default_dead_zone(40);
////#endif
////    g.rc_6.set_default_dead_zone(0);
//}
void Copter::default_dead_zones()
{
	channel_roll->set_default_dead_zone(30);
	channel_pitch->set_default_dead_zone(30);
#if FRAME_CONFIG == HELI_FRAME
	channel_throttle->set_default_dead_zone(10);
	channel_yaw->set_default_dead_zone(15);
	g.rc_8.set_default_dead_zone(10);
#else
	channel_throttle->set_default_dead_zone(30);
	channel_yaw->set_default_dead_zone(40);
#endif
	g.rc_6.set_default_dead_zone(0);
}
//void Copter::init_rc_in()
//{
//	cout <<"init_rc_in()" << endl;
//    channel_roll     = RC_Channel::rc_channel(0);
//    channel_pitch    = RC_Channel::rc_channel(1);
//    channel_throttle = RC_Channel::rc_channel(2);
//    channel_yaw      = RC_Channel::rc_channel(3);
//	//add mode channnel
//	channel_mode = RC_Channel::rc_channel(4);
//    // set rc channel ranges
//    channel_roll->set_angle(ROLL_PITCH_INPUT_MAX);
//    channel_pitch->set_angle(ROLL_PITCH_INPUT_MAX);
//    channel_yaw->set_angle(4500);
//    channel_throttle->set_range(g.throttle_min, THR_MAX);
//
//    channel_roll->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
//    channel_pitch->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
//    channel_yaw->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
//
//    //set auxiliary servo ranges
//    //g.rc_5.set_range(0,1000);
//    //g.rc_6.set_range(0,1000);
//    //g.rc_7.set_range(0,1000);
//    //g.rc_8.set_range(0,1000);
//
//    // set default dead zones
//    //default_dead_zones();
//
//	//ghq replace default_dead_zones() from RC_CHANNEL;
//	channel_roll->set_default_dead_zone(30);
//	channel_pitch->set_default_dead_zone(30);
//	channel_yaw->set_default_dead_zone(40);
//	channel_throttle->set_default_dead_zone(30);
//
//    // initialise throttle_zero flag
//    ap.throttle_zero = true;
//}
//
// // init_rc_out -- initialise motors and check if pilot wants to perform ESC calibration
//void Copter::init_rc_out()
//{ 
//    motors.set_update_rate(g.rc_speed);
//    motors.set_frame_orientation(g.frame_orientation);
//    motors.Init();                                              // motor initialisation
//#if FRAME_CONFIG != HELI_FRAME
//    motors.set_throttle_range(g.throttle_min, channel_throttle->radio_min, channel_throttle->radio_max);
//    motors.set_hover_throttle(g.throttle_mid);
//#endif
//
//    for(uint8_t i = 0; i < 5; i++) {
//        delay(20);
//        read_radio();
//    }  
//    // we want the input to be scaled correctly
//    channel_throttle->set_range_out(0,1000);
//
//    // check if we should enter esc calibration mode
//    esc_calibration_startup_check();
//
//    // enable output to motors
//    pre_arm_rc_checks();
//    if (ap.pre_arm_rc_check) {
//        enable_motor_output();
//    }
//
//    // refresh auxiliary channel to function map
//    RC_Channel_aux::update_aux_servo_function();
//
//    // setup correct scaling for ESCs like the UAVCAN PX4ESC which
//    // take a proportion of speed. 
//	hal.rcout->set_esc_scaling(channel_throttle->radio_min, channel_throttle->radio_max);
//}

	void Copter::init_rc_in()
	{
		channel_roll = RC_Channel::rc_channel(0);
		channel_pitch = RC_Channel::rc_channel(1);
		channel_throttle = RC_Channel::rc_channel(2);
		channel_yaw = RC_Channel::rc_channel(3);
		//add mode channnel
		channel_mode = RC_Channel::rc_channel(4);
		// set rc channel ranges
		channel_roll->set_angle(ROLL_PITCH_INPUT_MAX);
		channel_pitch->set_angle(ROLL_PITCH_INPUT_MAX);
		channel_yaw->set_angle(4500);

		struct timeval startTime, endTime;
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		g.throttle_min = supt->getParamValueWithNameAndKey("set_range","g.throttle_min");
		
		// ------------------------  ²å×®¼¤Àø ---------------------------------
		channel_throttle->set_range(g.throttle_min, THR_MAX);

		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("set_range", startTime, endTime);
		

		channel_roll->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
		channel_pitch->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
		channel_yaw->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

		//set auxiliary servo ranges
		g.rc_5.set_range(0,1000);
		g.rc_6.set_range(0,1000);
		g.rc_7.set_range(0,1000);
		g.rc_8.set_range(0,1000);

		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
		// set default dead zones
		default_dead_zones();

		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("default_dead_zones", startTime, endTime);

		// initialise throttle_zero flag
		ap.throttle_zero = true;
	}

	// init_rc_out -- initialise motors and check if pilot wants to perform ESC calibration
void Copter::init_rc_out()
{	
	// ------------------------  ²å×®µã ---------------------------------
		struct timeval startTime, endTime;
		gettimeofday(&startTime, NULL);
		g.rc_speed = supt->getParamValueWithNameAndKey("set_update_rate", "g.rc_speed");

		// ------------------------  ²å×®¼¤Àø ---------------------------------
		motors.set_update_rate(g.rc_speed);

		
		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("set_update_rate", startTime, endTime);

		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
		g.frame_orientation = supt->getParamValueWithNameAndKey("set_frame_orientation", "g.frame_orientation");

		motors.set_frame_orientation(g.frame_orientation);

		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("set_frame_orientation", startTime, endTime);

		motors.Init();                                              // motor initialisation
#if FRAME_CONFIG != HELI_FRAME
		
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
		g.throttle_min = supt->getParamValueWithNameAndKey("set_throttle_range", "g.throttle_min");

		motors.set_throttle_range(g.throttle_min, channel_throttle->radio_min, channel_throttle->radio_max);

		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("set_throttle_range", startTime, endTime);

		motors.set_hover_throttle(g.throttle_mid);
#endif
		////////////////////////////////////////////////////////////////////
		//delay(20);
		//read_radio();
		///////////////////////////////////////////////////////////////////
		for (uint8_t i = 0; i < 1; i++) {
			// ------------------------  ²å×®µã ---------------------------------
			gettimeofday(&startTime, NULL);
			// ------------------------  ²å×®¼¤Àø --------------------------------- 
			delay(20);

			gettimeofday(&endTime, NULL);
			supt->setCurProcessResult("delay", startTime, endTime);

			// ------------------------  ²å×®µã ---------------------------------
			gettimeofday(&startTime, NULL);
			// ------------------------  ²å×®¼¤Àø --------------------------------- 
			read_radio();

			gettimeofday(&endTime, NULL);
			supt->setCurProcessResult("read_radio", startTime, endTime);
		}  
		// we want the input to be scaled correctly
		channel_throttle->set_range_out(0, 1000);

		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
		// check if we should enter esc calibration mode
		// FIXÐÞ¸ÄV1.1
		//esc_calibration_startup_check();

		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("esc_calibration_startup_check", startTime, endTime);

		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
		// enable output to motors
		pre_arm_rc_checks();

		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("pre_arm_rc_checks", startTime, endTime);
		// FIXÐÞ¸ÄV1.1
		ap.pre_arm_rc_check = supt->getParamValueWithNameAndKey("enable_motor_output","ap.pre_arm_rc_check");
		if (ap.pre_arm_rc_check) {
			// ------------------------  ²å×®µã ---------------------------------
			gettimeofday(&startTime, NULL);
			// ------------------------  ²å×®¼¤Àø --------------------------------- 
			enable_motor_output();

			gettimeofday(&endTime, NULL);
			supt->setCurProcessResult("enable_motor_output", startTime, endTime);
		}

		// refresh auxiliary channel to function map
		RC_Channel_aux::update_aux_servo_function();

		// setup correct scaling for ESCs like the UAVCAN PX4ESC which
		// take a proportion of speed. 
		hal.rcout->set_esc_scaling(channel_throttle->radio_min, channel_throttle->radio_max);
	}

// enable_motor_output() - enable and output lowest possible value to motors
void Copter::enable_motor_output()
{
    // enable motors
    motors.enable();
    motors.output_min();
}

void Copter::read_radio()
{ 
	struct timeval startTime, endTime;

	//get values from barometer.init();
	static uint32_t last_update_ms = 0;

	// ------------------------  ²å×®µã ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ²å×®¼¤Àø --------------------------------- 
    uint32_t tnow_ms = millis(); 

	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("millis", startTime, endTime);

	bool new_input = false;

	if (supt->getParamValueWithNameAndKey("set_pwm", "has_new_input") == 1){
		new_input = true;
	}

	// ------------------------  ²å×®¼¤Àø ---------------------------------
	set_has_new_input(new_input);

	if (has_new_input == 1) {
        last_update_ms = tnow_ms;
        ap.new_radio_frame = true;
        RC_Channel::set_pwm_all();
		int16_t roll_pwm;
		int16_t pitch_pwm;
		int16_t throttle_pwm;
		int16_t yaw_pwm;
		//int16_t mode_pwm;
		//void input_pwm_method();

		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
		roll_pwm = supt->getParamValueWithNameAndKey("set_pwm", "roll_pwm");

		(*channel_roll).set_pwm(roll_pwm);

		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("set_pwm", startTime, endTime);

		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		roll_pwm = supt->getParamValueWithNameAndKey("set_pwm", "pitch_pwm");

		// ------------------------  ²å×®¼¤Àø ---------------------------------
		(*channel_pitch).set_pwm(pitch_pwm);

		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("set_pwm", startTime, endTime);

		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		roll_pwm = supt->getParamValueWithNameAndKey("set_pwm", "throttle_pwm");

		// ------------------------  ²å×®¼¤Àø ---------------------------------
		(*channel_throttle).set_pwm(throttle_pwm);

		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("set_pwm", startTime, endTime);

		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		roll_pwm = supt->getParamValueWithNameAndKey("set_pwm", "yaw_pwm");

		// ------------------------  ²å×®¼¤Àø ---------------------------------
		(*channel_yaw).set_pwm(yaw_pwm);

		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("set_pwm", startTime, endTime);
		
		/*channel_roll->set_pwm(roll_pwm);
		channel_pitch->set_pwm(pitch_pwm);
		channel_throttle->set_pwm(throttle_pwm);
		channel_yaw->set_pwm(yaw_pwm);*/
		//channel_mode->set_pwm(mode_pwm); 

		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        set_throttle_and_failsafe(channel_throttle->radio_in);

		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("set_throttle_and_failsafe", startTime, endTime);

		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
        set_throttle_zero_flag(channel_throttle->control_in); 

		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("set_throttle_zero_flag", startTime, endTime);

        // flag we must have an rc receiver attached
        if (!failsafe.rc_override_active) {
            ap.rc_receiver_present = true;
        }

        // update output on any aux channels, for manual passthru
        RC_Channel_aux::output_ch_all();
    }else{
        uint32_t elapsed = tnow_ms - last_update_ms;
        // turn on throttle failsafe if no update from the RC Radio for 500ms or 2000ms if we are using RC_OVERRIDE

		//FixÐÞ¸ÄV1.6
		failsafe.rc_override_active == true;
		failsafe.radio = true;

		if (supt->getParamValueWithNameAndKey("armed", "failsafe.rc_override_active") == 0)
			failsafe.rc_override_active = false;

		if (supt->getParamValueWithNameAndKey("armed", "failsafe.radio") == 0)
			failsafe.radio = false;

		if (supt->getParamValueWithNameAndKey("armed", "elapsed") != NOTFIND)
			elapsed = supt->getParamValueWithNameAndKey("armed", "elapsed");
			
		if (((failsafe.rc_override_active == false && (elapsed >= FS_RADIO_TIMEOUT_MS)) || (failsafe.rc_override_active == true && (elapsed >= FS_RADIO_RC_OVERRIDE_TIMEOUT_MS))) && failsafe.radio == false) {
			// ------------------------  ²å×®µã ---------------------------------
			gettimeofday(&startTime, NULL);
			// ------------------------  ²å×®¼¤Àø ---------------------------------
			bool is_armed = motors.armed();

			gettimeofday(&endTime, NULL);
			supt->setCurProcessResult("armed", startTime, endTime);

			if (g.failsafe_throttle == 1 && (ap.rc_receiver_present == true || is_armed == true))
			{
				//Log_Write_Error(ERROR_SUBSYSTEM_RADIO, ERROR_CODE_RADIO_LATE_FRAME);
				// ------------------------  ²å×®µã ---------------------------------
				gettimeofday(&startTime, NULL);
				// ------------------------  ²å×®¼¤Àø ---------------------------------
				set_failsafe_radio(true);

				gettimeofday(&endTime, NULL);
				supt->setCurProcessResult("set_failsafe_radio", startTime, endTime);
			}
		}
		if (failsafe.radio == 0) { 
			set_failsafe_radio(1);
		}
	} 
	return;
}



#define FS_COUNTER 3        // radio failsafe kicks in after 3 consecutive throttle values below failsafe_throttle_value
void Copter::set_throttle_and_failsafe(uint16_t throttle_pwm)
{
	struct timeval startTime, endTime;
	//FixÐÞ¸ÄV1.2
	string str[] = { "2", "set_pwm", "set_failsafe_radio" };
	g.failsafe_throttle = supt->getParamValueFormNamesWithKey(str, "g.failsafe_throttle");
	g.failsafe_throttle_value = supt->getParamValueFormNamesWithKey(str, "g.failsafe_throttle_value");
	// if failsafe not enabled pass through throttle and exit
	if (g.failsafe_throttle == FS_THR_DISABLED) {
		// ------------------------  ²å×®µã ---------------------------------
		gettimeofday(&startTime, NULL);
		// ------------------------  ²å×®¼¤Àø ---------------------------------
		channel_throttle->set_pwm(throttle_pwm);

		gettimeofday(&endTime, NULL);
		supt->setCurProcessResult("set_pwm", startTime, endTime);
	}
	else
	{
		//FixÐÞ¸ÄV1.2
		throttle_pwm = 0;
		//check for low throttle value
		if (throttle_pwm < (uint16_t)g.failsafe_throttle_value) {
			// if we are already in failsafe or motors not armed pass through throttle and exit
			// ------------------------  ²å×®µã ---------------------------------
			gettimeofday(&startTime, NULL);
			// ------------------------  ²å×®¼¤Àø ---------------------------------
			bool has_armed = motors.armed();

			gettimeofday(&endTime, NULL);
			supt->setCurProcessResult("armed", startTime, endTime);
			//FixÐÞ¸ÄV1.2
			string str[] = {"2","set_failsafe_radio","set_pwm"};
			failsafe.radio = supt->getParamValueFormNamesWithKey(str,"failsafe.radio");
			ap.rc_receiver_present = supt->getParamValueFormNamesWithKey(str, "ap.rc_receiver_present");
			has_armed = supt->getParamValueFormNamesWithKey(str, "has_armed");
			if (failsafe.radio == true || (ap.rc_receiver_present == false && has_armed == false)) {

				// ------------------------  ²å×®µã ---------------------------------
				gettimeofday(&startTime, NULL);
				// ------------------------  ²å×®¼¤Àø ---------------------------------
				channel_throttle->set_pwm(throttle_pwm);

				gettimeofday(&endTime, NULL);
				supt->setCurProcessResult("set_pwm", startTime, endTime);
			}
			else{
				// check for 3 low throttle values
				// Note: we do not pass through the low throttle until 3 low throttle values are recieved
				failsafe.radio_counter++;
				//FixÐÞ¸ÄV1.2
				failsafe.radio_counter = supt->getParamValueWithNameAndKey("set_failsafe_radio", "failsafe.radio_counter");
				if (failsafe.radio_counter >= FS_COUNTER) {
					failsafe.radio_counter = FS_COUNTER;  // check to ensure we don't overflow the counter
					// ------------------------  ²å×®µã ---------------------------------
					gettimeofday(&startTime, NULL);
					// ------------------------  ²å×®¼¤Àø ---------------------------------
					set_failsafe_radio(true);

					gettimeofday(&endTime, NULL);
					supt->setCurProcessResult("set_failsafe_radio", startTime, endTime);

					// ------------------------  ²å×®µã ---------------------------------
					gettimeofday(&startTime, NULL);
					// ------------------------  ²å×®¼¤Àø ---------------------------------
					channel_throttle->set_pwm(throttle_pwm);   // pass through failsafe throttle

					gettimeofday(&endTime, NULL);
					supt->setCurProcessResult("set_pwm", startTime, endTime);
				}
			}
		}
		else{
			// we have a good throttle so reduce failsafe counter
			failsafe.radio_counter--;
			if (failsafe.radio_counter <= 0) {
				failsafe.radio_counter = 0;   // check to ensure we don't underflow the counter
				// disengage failsafe after three (nearly) consecutive valid throttle values
				//FixÐÞ¸ÄV1.2
				failsafe.radio = supt->getParamValueWithNameAndKey("set_failsafe_radio", "failsafe.radio");
				if (failsafe.radio == true) {
					// ------------------------  ²å×®µã ---------------------------------
					gettimeofday(&startTime, NULL);
					// ------------------------  ²å×®¼¤Àø ---------------------------------
					set_failsafe_radio(false);

					gettimeofday(&endTime, NULL);
					supt->setCurProcessResult("set_failsafe_radio", startTime, endTime);

				}
			}
			// ------------------------  ²å×®µã ---------------------------------
			gettimeofday(&startTime, NULL);
			// ------------------------  ²å×®¼¤Àø ---------------------------------
			// pass through throttle
			channel_throttle->set_pwm(throttle_pwm);

			gettimeofday(&endTime, NULL);
			supt->setCurProcessResult("set_pwm", startTime, endTime);
		}
	}
  //  // if failsafe not enabled pass through throttle and exit
  //  if(g.failsafe_throttle == FS_THR_DISABLED) {
  //      //channel_throttle->set_pwm(throttle_pwm);
  //      return;
  //  }

  //  //check for low throttle value
  //  if (throttle_pwm < (uint16_t)g.failsafe_throttle_value) {

  //      // if we are already in failsafe or motors not armed pass through throttle and exit 
  //      if (failsafe.radio || !(ap.rc_receiver_present || motors.armed())) {
  //          channel_throttle->set_pwm(throttle_pwm);
  //          return;
  //      }

  //      // check for 3 low throttle values
  //      // Note: we do not pass through the low throttle until 3 low throttle values are recieved
  //      failsafe.radio_counter++;
  //      if( failsafe.radio_counter >= FS_COUNTER ) {
  //          failsafe.radio_counter = FS_COUNTER;  // check to ensure we don't overflow the counter
  //          set_failsafe_radio(true);
  //          channel_throttle->set_pwm(throttle_pwm);   // pass through failsafe throttle
  //      }
  //  }else{
  //      // we have a good throttle so reduce failsafe counter
  //      failsafe.radio_counter--;
  //      if( failsafe.radio_counter <= 0 ) {
  //          failsafe.radio_counter = 0;   // check to ensure we don't underflow the counter

  //          // disengage failsafe after three (nearly) consecutive valid throttle values
  //         // if (failsafe.radio) {
		//	if (failsafe.radio==true) {
  //              set_failsafe_radio(false);
  //          }
  //      }
  //      // pass through throttle
		//channel_throttle->set_pwm(throttle_pwm);
  //  }
}

#define THROTTLE_ZERO_DEBOUNCE_TIME_MS 400
// set_throttle_zero_flag - set throttle_zero flag from debounced throttle control
// throttle_zero is used to determine if the pilot intends to shut down the motors
// Basically, this signals when we are not flying.  We are either on the ground
// or the pilot has shut down the copter in the air and it is free-falling
void Copter::set_throttle_zero_flag(int16_t throttle_control)
{
    static uint32_t last_nonzero_throttle_ms = 0;
    uint32_t tnow_ms = millis();

    // if not using throttle interlock and non-zero throttle and not E-stopped,
    // or using motor interlock and it's enabled, then motors are running, 
    // and we are flying. Immediately set as non-zero
    if ((!ap.using_interlock && (throttle_control > 0) && !ap.motor_emergency_stop) || (ap.using_interlock && motors.get_interlock())) {
        last_nonzero_throttle_ms = tnow_ms;
        ap.throttle_zero = false;
    } else if (tnow_ms - last_nonzero_throttle_ms > THROTTLE_ZERO_DEBOUNCE_TIME_MS) {
        ap.throttle_zero = true;
    }
}
