#ifndef _SIM_HELICOPTER_H
#define _SIM_HELICOPTER_H

#include "SIM_Aircraft.h"

/*
  a helicopter simulator
 */
class Helicopter : public Aircraft
{
public:
    Helicopter(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new Helicopter(home_str, frame_str);
    }

private:
    float terminal_rotation_rate = 4*radians(360.0f);
    float hover_throttle = 0.65f;
    float terminal_velocity = 40;
    float hover_lean = 8.0f;
    float yaw_zero = 0.1f;
    float rotor_rot_accel = radians(20);
    float roll_rate_max = radians(1400);
    float pitch_rate_max = radians(1400);
    float yaw_rate_max = radians(1400);
    float rsc_setpoint = 0.8f;
    float thrust_scale;
    float tail_thrust_scale;
    enum frame_types {
        HELI_FRAME_CONVENTIONAL,
        HELI_FRAME_DUAL,
        HELI_FRAME_COMPOUND
    } frame_type = HELI_FRAME_CONVENTIONAL;
    bool gas_heli = false;
};


#endif // _SIM_HELICOPTER_H
