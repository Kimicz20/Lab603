// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_STEER_CONTROLLER_H__
#define __AP_STEER_CONTROLLER_H__

#include "../AP_AHRS/AP_AHRS.h"
#include "../AP_Common/AP_Common.h"
#include "../AP_Vehicle/AP_Vehicle.h"
#include "../DataFlash/DataFlash.h"

class AP_SteerController {
public:
	AP_SteerController(AP_AHRS &ahrs) :
        _ahrs(ahrs)
    { 
		//AP_Param::setup_object_defaults(this, var_info);
		_tau = 0.75f;
		_K_P = 1.8f;
		_K_I = 0.2f;
		_K_D = 0.005f;
		_imax = 1500;
		_minspeed = 1.0f;
		_K_FF = 0;
	}

    /*
      return a steering servo output from -4500 to 4500 given a
      desired lateral acceleration rate in m/s/s. Positive lateral
      acceleration is to the right.
     */
	int32_t get_steering_out_lat_accel(float desired_accel);

    /*
      return a steering servo output from -4500 to 4500 given a
      desired yaw rate in degrees/sec. Positive yaw is to the right.
     */
	int32_t get_steering_out_rate(float desired_rate);

    /*
      return a steering servo output from -4500 to 4500 given a
      yaw error in centi-degrees
     */
	int32_t get_steering_out_angle_error(int32_t angle_err);

    /*
      return the steering radius (half diameter). Assumed to be half
      the P value.
     */
    float get_turn_radius(void) const { return _K_P * 0.5f; }

	void reset_I();

	static const struct AP_Param::GroupInfo var_info[];

    const DataFlash_Class::PID_Info& get_pid_info(void) const { return _pid_info; }

private:
	AP_Float _tau;
	AP_Float _K_FF;
	AP_Float _K_P;
	AP_Float _K_I;
	AP_Float _K_D;
	AP_Float _minspeed;
    AP_Int16  _imax;
	uint32_t _last_t;
	float _last_out;

    DataFlash_Class::PID_Info _pid_info {};

	AP_AHRS &_ahrs;
};

#endif // __AP_STEER_CONTROLLER_H__
