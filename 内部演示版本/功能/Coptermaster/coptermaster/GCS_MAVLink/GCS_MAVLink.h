// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	GCS_MAVLink.h
/// @brief	One size fits all header for MAVLink integration.

#ifndef GCS_MAVLink_h
#define GCS_MAVLink_h

#include "../AP_HAL/AP_HAL.h"
#include "../AP_Param/AP_Param.h"
#include "../AP_Math/AP_Math.h"

// we have separate helpers disabled to make it possible
// to select MAVLink 1.0 in the arduino GUI build
#define MAVLINK_SEPARATE_HELPERS

#define MAVLINK_SEND_UART_BYTES(chan, buf, len) comm_send_buffer(chan, buf, len)

// define our own MAVLINK_MESSAGE_CRC() macro to allow it to be put
// into 
#define MAVLINK_MESSAGE_CRC(msgid) mavlink_get_message_crc(msgid)

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2
#include <util/crc16.h>
#define HAVE_CRC_ACCUMULATE
// only two telemetry ports on APM1/APM2
#define MAVLINK_COMM_NUM_BUFFERS 2
#else
// allow four telemetry ports on other boards
#define MAVLINK_COMM_NUM_BUFFERS 4
#endif

/*
  The MAVLink protocol code generator does its own alignment, so
  alignment cast warnings can be ignored
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"

#include "../GCS_MAVLink/version.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2
// this allows us to make mavlink_message_t much smaller. It means we
// can't support the largest messages in common.xml, but we don't need
// those for APM1/APM2
#define MAVLINK_MAX_PAYLOAD_LEN 104
#else
#define MAVLINK_MAX_PAYLOAD_LEN 255
#endif

#include "../GCS_MAVLink/mavlink_types.h"

/// MAVLink stream used for uartA
 extern AP_HAL::UARTDriver	*mavlink_comm_port[MAVLINK_COMM_NUM_BUFFERS];

/// MAVLink system definition
 MAV(
	 typedef struct __mavlink_system {
	 uint8_t sysid;   ///< Used by the MAVLink message_xx_send() convenience function
	 uint8_t compid;  ///< Used by the MAVLink message_xx_send() convenience function
 }) mavlink_system_t;

extern mavlink_system_t mavlink_system;

/// Send a byte to the nominated MAVLink channel
///
/// @param chan		Channel to send to
/// @param ch		Byte to send
///
static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    // sanity check chan
    if (chan >= MAVLINK_COMM_NUM_BUFFERS) {
        return;
    }
    mavlink_comm_port[chan]->write(ch);
}

void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len);

/// Read a byte from the nominated MAVLink channel
///
/// @param chan		Channel to receive on
/// @returns		Byte read
///
uint8_t comm_receive_ch(mavlink_channel_t chan);

/// Check for available data on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
uint16_t comm_get_available(mavlink_channel_t chan);


/// Check for available transmit space on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
//uint16_t comm_get_txspace(mavlink_channel_t chan);

//#ifdef HAVE_CRC_ACCUMULATE
// use the AVR C library implementation. This is a bit over twice as
// fast as the C version
//static inline void crc_accumulate(uint8_t data, uint16_t *crcAccum)
//{
//	*crcAccum = _crc_ccitt_update(*crcAccum, data);
//}
//#endif

/*
  return true if the MAVLink parser is idle, so there is no partly parsed
  MAVLink message being processed
 */
bool comm_is_idle(mavlink_channel_t chan);

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "../GCS_MAVLink/mavlink.h"

// return a MAVLink variable type given a AP_Param type
//uint8_t mav_var_type(enum ap_var_type t);

// return CRC byte for a mavlink message ID
uint8_t mavlink_get_message_crc(uint8_t msgid);

#pragma GCC diagnostic pop

#endif // GCS_MAVLink_h
