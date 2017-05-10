#ifndef _SIM_BALLOON_H
#define _SIM_BALLOON_H

#include "SIM_Aircraft.h"

/*
  a balloon simulator
 */
class Balloon : public Aircraft
{
public:
    Balloon(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new Balloon(home_str, frame_str);
    }

private:
    float terminal_rotation_rate = radians(100);
    float climb_rate = 20;
    float terminal_velocity = 40;
    float burst_altitude = 20000;
    bool burst = false;
    bool released = false;
};


#endif // _SIM_BALLOON_H
