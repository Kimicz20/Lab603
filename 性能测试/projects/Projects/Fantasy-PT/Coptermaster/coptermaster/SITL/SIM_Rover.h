#ifndef _SIM_ROVER_H
#define _SIM_ROVER_H

#include "SIM_Aircraft.h"

/*
  a rover simulator
 */
class Rover : public Aircraft
{
public:
    Rover(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new Rover(home_str, frame_str);
    }

private:
    float max_speed;
    float max_accel;
    float wheelbase;
    float wheeltrack;
    float max_wheel_turn;
    float turning_circle;
    float skid_turn_rate;
    bool skid_steering;

    float turn_circle(float steering);
    float calc_yaw_rate(float steering, float speed);
    float calc_lat_accel(float steering_angle, float speed);
};


#endif // _SIM_ROVER_H
