/*
  simulator connection for ardupilot version of last_letter
*/

#ifndef _SIM_LAST_LETTER_H
#define _SIM_LAST_LETTER_H

#include "SIM_Aircraft.h"
#include "../AP_HAL/Socket.h"

/*
  a last_letter simulator
 */
class last_letter : public Aircraft
{
public:
    last_letter(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new last_letter(home_str, frame_str);
    }

private:
    static const uint16_t fdm_port = 9002;

    /*
      packet sent to last_letter
     */
    struct servo_packet {
        uint16_t servos[16];
    };
    
    /*
      reply packet sent from last_letter to ArduPilot
     */
    struct fdm_packet {
        uint64_t timestamp_us; // simulation time in microseconds
        double latitude, longitude;
        double altitude;
        double heading;
        double speedN, speedE, speedD;
        double xAccel, yAccel, zAccel;
        double rollRate, pitchRate, yawRate;
        double roll, pitch, yaw;
        double airspeed;
    };

    void recv_fdm(const struct sitl_input &input);
    void send_servos(const struct sitl_input &input);
    void start_last_letter(void);

    uint64_t last_timestamp_us;
    SocketAPM sock;

    const char *frame_str;
};


#endif // _SIM_LAST_LETTER_H
