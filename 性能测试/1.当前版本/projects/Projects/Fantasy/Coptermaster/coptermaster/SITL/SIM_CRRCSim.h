#ifndef _SIM_CRRCSIM_H
#define _SIM_CRRCSIM_H

#include "SIM_Aircraft.h"
#include "../AP_HAL/utility/Socket.h"

/*
  a CRRCSim simulator
 */
class CRRCSim : public Aircraft
{
public:
    CRRCSim(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new CRRCSim(home_str, frame_str);
    }

private:
    /*
      packet sent to CRRCSim
     */
    struct servo_packet {
        float roll_rate;
        float pitch_rate;
        float throttle;
        float yaw_rate;
        float col_pitch;
    };
    
    /*
      reply packet sent from CRRCSim to ArduPilot
     */
    struct fdm_packet {
        double timestamp;
        double latitude, longitude;
        double altitude;
        double heading;
        double speedN, speedE, speedD;
        double xAccel, yAccel, zAccel;
        double rollRate, pitchRate, yawRate;
        double roll, pitch, yaw;
        double airspeed;
    };

    void send_servos_heli(const struct sitl_input &input);
    void send_servos_fixed_wing(const struct sitl_input &input);
    void recv_fdm(const struct sitl_input &input);
    void send_servos(const struct sitl_input &input);

    bool heli_servos;
    double last_timestamp;
    SocketAPM sock;
};


#endif // _SIM_CRRCSIM_H
