/*
  antenna-tracker simulator class
*/

#ifndef _SIM_TRACKER_H
#define _SIM_TRACKER_H

#include "SIM_Aircraft.h"

/*
  a antenna tracker simulator
 */
class Tracker : public Aircraft
{
public:
    Tracker(const char *home_str, const char *frame_str);
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new Tracker(home_str, frame_str);
    }

private:

    const bool onoff = false;
    const float yawrate = 9.0f;
    const float pitchrate = 1.0f;
    const float pitch_range = 45;
    const float yaw_range = 170;
    const float zero_yaw = 270;  // yaw direction at startup
    const float zero_pitch = 10; // pitch at startup
    bool verbose = false;
    uint64_t last_debug_us = 0;

    float pitch_input;
    float yaw_input;
    float yaw_current_relative;
    float pitch_current_relative;

    void update_position_servos(float delta_time, float &yaw_rate, float &pitch_rate);
    void update_onoff_servos(float &yaw_rate, float &pitch_rate);
};

#endif
