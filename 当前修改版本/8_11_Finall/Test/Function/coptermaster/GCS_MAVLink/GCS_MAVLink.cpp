//2017_01_09 GCS_MAVLINK.cpp from ArduCopter 
#include "../ArduCopter/Copter.h" 
#include "../GCS_MAVLink/version.h"
#include "GCS_MAVLink.h"

bool initialised(void) {
	return true;
}

/*
*  look for incoming commands on the GCS links
*/
void Copter::gcs_check_input(void)
{
	struct timeval startTime, endTime;
	supt->Cout("gcs_check_input");
	//Fix�޸�V2.0
	// ------------------------  ��׮�� ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ��׮���� --------------------------------- 

	for (uint8_t i = 0; i<1; i++) {
		gcs[i].initialised = false;
		if (supt->getParamValueWithNameAndKey("handleMessage", "gcs[I].initialised") == 1)
			gcs[i].initialised = true;
		if (gcs[i].initialised == true) {
		//if (1){
//#if CLI_ENABLED == ENABLED
//			gcs[i].update(g.cli_enabled == 1 ? FUNCTOR_BIND_MEMBER(&Copter::run_cli, void, AP_HAL::UARTDriver *) : NULL);
//#else 

			// ------------------------  ��׮�� ---------------------------------
			gettimeofday(&startTime, NULL);
			// ------------------------  ��׮���� --------------------------------- 

			gcs[i].supt = supt;
			gcs[i].update(NULL);

			gettimeofday(&endTime, NULL);
			supt->setCurProcessResult("update", startTime, endTime);

//#endif
		}
	}
	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("gcs_check_input", startTime, endTime);

}

void Copter::gcs_data_stream_send(void)
{
	supt->Cout("gcs_data_stream_send");
	//Fix�޸�2.1

	struct timeval startTime, endTime;
	// ------------------------  ��׮�� ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ��׮���� --------------------------------- 
	gcs[0].data_stream_send();

	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("gcs_data_stream_send", startTime, endTime);
	
	/*for (uint8_t i = 0; i<1; i++) {

		if (gcs[i].initialised) {
			gcs[i].data_stream_send();
		}
	}*/
}

void GCS_MAVLINK::data_stream_send(void)
{
	//Fix�޸�2.1
	struct timeval startTime, endTime;
	// ------------------------  ��׮�� ---------------------------------
	gettimeofday(&startTime, NULL);
	// ------------------------  ��׮���� --------------------------------- 
	send_message(MSG_EXTENDED_STATUS1);

	gettimeofday(&endTime, NULL);
	supt->setCurProcessResult("send_location", startTime, endTime);
}
uint16_t comm_get_available(mavlink_channel_t)
{
	int16_t bytes = 1;
	return (uint16_t)bytes;
}

void Copter::gcs_send_heartbeat(void)
{
	//gcs_send_message(MSG_HEARTBEAT);
}

void Copter::gcs_send_deferred(void)
{
	//gcs_send_message(MSG_RETRY_DEFERRED);
	//GCS_MAVLINK::service_statustext();
}

//Fix�޸�V2.0
#define MAV_CMD_COMPONENT_CZ_ARM_DISARM 40
#define MAV_CMD_COMPONENT_CZ 11
//void GCS_MAVLINK_Copter::handleMessage(mavlink_message_t* msg)
void GCS_MAVLINK::handleMessage(mavlink_message_t* msg)
{
	uint8_t result = MAV_RESULT_FAILED;         // assume failure.  Each messages id is responsible for return ACK or NAK if required
	struct timeval startTime, endTime;
	//int a = 76; 
	//mavlink_message_t a, *msg;
	//msg = &a;
	int choose = 0;
	
	
	//Fix�޸�V2.0
	string str[] = { "6","handle_set_mode","do_user_takeoff", "guided_set_destination_posvel", "guided_set_velocity", "guided_set_destination", "mode_has_manual_throttle" };
	choose = supt->getParamValueFormNamesWithKey(str,"msg.msgid");
	if (choose == 76){
		msg->msgid = MAVLINK_MSG_ID_COMMAND_LONG;
	}
	else if (choose == 84){
		msg->msgid = MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED;
	}
	else if (choose == 86){
		msg->msgid = MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT;
	}
	else if (choose == 400){
		msg->msgid = MAV_CMD_COMPONENT_CZ_ARM_DISARM;
	}
	else if (choose == 11){
		msg->msgid = MAV_CMD_COMPONENT_CZ;
	}

	switch (msg->msgid) {
		//Pre-Flight calibration requests
		case MAVLINK_MSG_ID_COMMAND_LONG:       // MAV ID: 76
		{
			// decode packet
			mavlink_command_long_t packet;
			mavlink_msg_command_long_decode(msg, &packet);
			packet.command = supt->getParamValueWithNameAndKey("do_user_takeoff","packet.command");
			switch (packet.command) {
				//�޸��Զ�Ӧģ��
				/*case MAV_CMD_START_RX_PAIR:
					result = handle_rc_bind(packet);
					break;*/

				case MAV_CMD_NAV_TAKEOFF: {//22
					// param3 : horizontal navigation by pilot acceptable
					// param4 : yaw angle   (not supported)
					// param5 : latitude    (not supported)
					// param6 : longitude   (not supported)
					// param7 : altitude [metres]

					float takeoff_alt = packet.param7 * 100;      // Convert m to cm
					// ------------------------  ��׮�� ---------------------------------
					gettimeofday(&startTime, NULL);
					// ------------------------  ��׮���� --------------------------------- 
					if (copter.do_user_takeoff(takeoff_alt, is_zero(packet.param3))) {
						result = MAV_RESULT_ACCEPTED;
					}
					else {
						result = MAV_RESULT_FAILED;
					}
					gettimeofday(&endTime, NULL);
					supt->setCurProcessResult("do_user_takeoff", startTime, endTime);
					break;
				}
			}
			break;
		}
		case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:     // MAV ID: 84
		{
			// decode packet
			mavlink_set_position_target_local_ned_t packet;
			mavlink_msg_set_position_target_local_ned_decode(msg, &packet);

			// exit if vehicle is not in Guided mode or Auto-Guided mode
			/*if ((copter.control_mode != GUIDED) && !(copter.control_mode == AUTO && copter.auto_mode == Auto_NavGuided)) {
				break;
			}

			// check for supported coordinate frames
			if (packet.coordinate_frame != MAV_FRAME_LOCAL_NED &&
				packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED &&
				packet.coordinate_frame != MAV_FRAME_BODY_NED &&
				packet.coordinate_frame != MAV_FRAME_BODY_OFFSET_NED) {
				break;
			}*/

			bool pos_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
			bool vel_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
			bool acc_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;

			/*
			* for future use:
			* bool force           = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE;
			* bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
			* bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;
			*/

			// prepare position
			Vector3f pos_vector;
			if (!pos_ignore) {
				// convert to cm
				pos_vector = Vector3f(packet.x * 100.0f, packet.y * 100.0f, -packet.z * 100.0f);
				// rotate to body-frame if necessary
				if (packet.coordinate_frame == MAV_FRAME_BODY_NED ||
					packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
					copter.rotate_body_frame_to_NE(pos_vector.x, pos_vector.y);
				}
				// add body offset if necessary
				if (packet.coordinate_frame == MAV_FRAME_LOCAL_OFFSET_NED ||
					packet.coordinate_frame == MAV_FRAME_BODY_NED ||
					packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
					pos_vector += copter.inertial_nav.get_position();
				}
				else {
					// convert from alt-above-home to alt-above-ekf-origin
					pos_vector.z = copter.pv_alt_above_origin(pos_vector.z);
				}
			}

			// prepare velocity
			Vector3f vel_vector;
			if (!vel_ignore) {
				// convert to cm
				vel_vector = Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f);
				// rotate to body-frame if necessary
				if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
					copter.rotate_body_frame_to_NE(vel_vector.x, vel_vector.y);
				}
			}
			//Fix�޸�V2.0
			string tmp[] = { "3", "guided_set_destination_posvel", "guided_set_velocity", "guided_set_destination" };
			pos_ignore = supt->getParamValueFormNamesWithKey(tmp, "pos_ignore");
			vel_ignore = supt->getParamValueFormNamesWithKey(tmp, "vel_ignore");
			acc_ignore = supt->getParamValueFormNamesWithKey(tmp, "acc_ignore");

			// send request
			if (!pos_ignore && !vel_ignore && acc_ignore) { 
				// ------------------------  ��׮�� ---------------------------------
				gettimeofday(&startTime, NULL);
				// ------------------------  ��׮���� --------------------------------- 
				copter.guided_set_destination_posvel(pos_vector, vel_vector);
				gettimeofday(&endTime, NULL);
				supt->setCurProcessResult("guided_set_destination_posvel", startTime, endTime);
			}
			else if (pos_ignore && !vel_ignore && acc_ignore) {
				// ------------------------  ��׮�� ---------------------------------
				gettimeofday(&startTime, NULL);
				// ------------------------  ��׮���� --------------------------------- 
				copter.guided_set_velocity(vel_vector);
				gettimeofday(&endTime, NULL);
				supt->setCurProcessResult("guided_set_velocity", startTime, endTime);
			}
			else if (!pos_ignore && vel_ignore && acc_ignore) {
				//Fix�޸�V2.0
				// ------------------------  ��׮�� ---------------------------------
				gettimeofday(&startTime, NULL);
				// ------------------------  ��׮���� --------------------------------- 
				copter.guided_set_destination(pos_vector);
				gettimeofday(&endTime, NULL);
				supt->setCurProcessResult("guided_set_destination", startTime, endTime);
			}
			else {
				result = MAV_RESULT_FAILED;
			}

			break;
		}

		case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:    // MAV ID: 86
		{
			// decode packet
			mavlink_set_position_target_global_int_t packet;
			mavlink_msg_set_position_target_global_int_decode(msg, &packet);

			// exit if vehicle is not in Guided mode or Auto-Guided mode
			/*if ((copter.control_mode != GUIDED) && !(copter.control_mode == AUTO && copter.auto_mode == Auto_NavGuided)) {
				break;
			}

			// check for supported coordinate frames
			if (packet.coordinate_frame != MAV_FRAME_GLOBAL_INT &&
				packet.coordinate_frame != MAV_FRAME_GLOBAL_RELATIVE_ALT && // solo shot manager incorrectly sends RELATIVE_ALT instead of RELATIVE_ALT_INT
				packet.coordinate_frame != MAV_FRAME_GLOBAL_RELATIVE_ALT_INT &&
				packet.coordinate_frame != MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) {
				break;
			}*/

			// for mavlink SET_POSITION_TARGET messages
			#define MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE      ((1<<0) | (1<<1) | (1<<2))
			#define MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE      ((1<<3) | (1<<4) | (1<<5))
			#define MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE      ((1<<6) | (1<<7) | (1<<8))
			#define MAVLINK_SET_POS_TYPE_MASK_FORCE           (1<<9)
			#define MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE      (1<<10)
			#define MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE (1<<11)


			bool pos_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
			bool vel_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
			bool acc_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;

			/*
			* for future use:
			* bool force           = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE;
			* bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
			* bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;
			*/

			Vector3f pos_ned;
			/*bool check_latlng(int32_t lat, int32_t lng)
			{
			return check_latlng(lat) && check_latlng(lng);
			}*/
			/*bool check_latlng(float lat, float lng)
			{
				return check_latlng(lat) && check_latlng(lng);
			}*/
			if (!pos_ignore) {
				// sanity check location
				/*if (!check_latlng(packet.lat_int, packet.lon_int)) {
					result = MAV_RESULT_FAILED;
					break;
				}*/
				Location loc;
				loc.lat = packet.lat_int;
				loc.lng = packet.lon_int;
				loc.alt = packet.alt * 100;
				switch (packet.coordinate_frame) {
				case MAV_FRAME_GLOBAL_RELATIVE_ALT: // solo shot manager incorrectly sends RELATIVE_ALT instead of RELATIVE_ALT_INT
				case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
					loc.flags.relative_alt = true;
					loc.flags.terrain_alt = false;
					break;
				case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
					loc.flags.relative_alt = true;
					loc.flags.terrain_alt = true;
					break;
				case MAV_FRAME_GLOBAL_INT:
				default:
					// Copter does not support navigation to absolute altitudes. This convert the WGS84 altitude
					// to a home-relative altitude before passing it to the navigation controller
					loc.alt -= copter.ahrs.get_home().alt;
					Location loc;
					loc.flags.relative_alt = true;
					loc.flags.terrain_alt = false;
					break;
				}
				pos_ned = copter.pv_location_to_vector(loc);
			}
			//Fix�޸�V2.0
			string tmp[] = {"3","guided_set_destination_posvel","guided_set_velocity","guided_set_destination"};
			pos_ignore = supt->getParamValueFormNamesWithKey(tmp,"pos_ignore");
			vel_ignore = supt->getParamValueFormNamesWithKey(tmp, "vel_ignore");
			acc_ignore = supt->getParamValueFormNamesWithKey(tmp, "acc_ignore");
			if (!pos_ignore && !vel_ignore && acc_ignore) {

				// ------------------------  ��׮�� ---------------------------------
				gettimeofday(&startTime, NULL);
				// ------------------------  ��׮���� --------------------------------- 
				copter.guided_set_destination_posvel(pos_ned, Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f));
				gettimeofday(&endTime, NULL);
				supt->setCurProcessResult("guided_set_destination_posvel", startTime, endTime);
			}
			else if (pos_ignore && !vel_ignore && acc_ignore) {
				// ------------------------  ��׮�� ---------------------------------
				gettimeofday(&startTime, NULL);
				// ------------------------  ��׮���� --------------------------------- 
				copter.guided_set_velocity(Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f));
				gettimeofday(&endTime, NULL);
				supt->setCurProcessResult("guided_set_velocity", startTime, endTime);
			}
			else if (!pos_ignore && vel_ignore && acc_ignore) {
				// ------------------------  ��׮�� ---------------------------------
				gettimeofday(&startTime, NULL);
				// ------------------------  ��׮���� --------------------------------- 
				copter.guided_set_destination(pos_ned);
				
				gettimeofday(&endTime, NULL);
				supt->setCurProcessResult("guided_set_destination", startTime, endTime);
			}
			else {
				result = MAV_RESULT_FAILED;
			}
			break;
		}
		//Fix�޸�V2.0
		case MAV_CMD_COMPONENT_CZ_ARM_DISARM:    // MAV ID: 400
		{
			// ------------------------  ��׮�� ---------------------------------
			gettimeofday(&startTime, NULL);
			// ------------------------  ��׮���� --------------------------------- 
			gettimeofday(&endTime, NULL);
			supt->setCurProcessResult("mode_has_manual_throttle", startTime, endTime);
			bool is_equal_1 = false;
			if (supt->getParamValueWithNameAndKey("init_disarm_motors", "is_equal_1") == 1)
				is_equal_1 = true;
			if (is_equal_1){
				// ------------------------  ��׮�� ---------------------------------
				gettimeofday(&startTime, NULL);
				// ------------------------  ��׮���� --------------------------------- 
				gettimeofday(&endTime, NULL);
				supt->setCurProcessResult("init_disarm_motors", startTime, endTime);
			}
			else{
				// ------------------------  ��׮�� ---------------------------------
				gettimeofday(&startTime, NULL);
				// ------------------------  ��׮���� --------------------------------- 
				gettimeofday(&endTime, NULL);
				supt->setCurProcessResult("init_disarm_motors", startTime, endTime);
			}
			break;
		}
		//Fix�޸�V2.2
		case MAV_CMD_COMPONENT_CZ:    // MAV ID: 11
		{
			// ------------------------  ��׮�� ---------------------------------
			gettimeofday(&startTime, NULL);
			// ------------------------  ��׮���� --------------------------------- 
			gettimeofday(&endTime, NULL);
			supt->setCurProcessResult("handle_set_mode", startTime, endTime);
			supt->setCurProcessResult("set_mode", startTime, endTime);
			break;
		}

	}
} 
 
