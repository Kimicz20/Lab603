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
	cout << "----- gcs_check_input begin -----" << endl;
	long start, end;
	for (uint8_t i = 0; i<num_gcs; i++) {
		gcs[i].initialised = 1;//设置为1来进入
		if (gcs[i].initialised == true) {
		//if (1){
//#if CLI_ENABLED == ENABLED
//			gcs[i].update(g.cli_enabled == 1 ? FUNCTOR_BIND_MEMBER(&Copter::run_cli, void, AP_HAL::UARTDriver *) : NULL);
//#else 

			// ------------------------  插桩点 ---------------------------------
			start = clock();
			supt->setCurProcessResult("update", start, 1);

			// ------------------------  插桩激励 ---------------------------------
			gcs[i].update(NULL);

			end = clock();
			supt->setCurProcessResult("update", end, 2);
			supt->setCurProcessResult("update", (end - start), 3);

//#endif
		}
	}
	cout << "----- gcs_check_input end -----" << endl;
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

//void GCS_MAVLINK_Copter::handleMessage(mavlink_message_t* msg)
void GCS_MAVLINK::handleMessage(mavlink_message_t* msg)
{
	cout << "----- handleMessage begin -----" << endl;
	uint8_t result = MAV_RESULT_FAILED;         // assume failure.  Each messages id is responsible for return ACK or NAK if required
	long start, end;
	//int a = 76; 
	//mavlink_message_t a, *msg;
	//msg = &a;
	cout << MAVLINK_MSG_ID_COMMAND_LONG << endl
		<< MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED << endl
		<< MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT << endl;
	int choose = 0;
	
	cout << "输入已选择进入的方法（可选值76 84 86）" << endl;

	if (Copter::supt->getParamValueWithNameAndKey("do_user_takeoff","msg.msgid") != -1){
		
		choose = Copter::supt->getParamValueWithNameAndKey("do_user_takeoff", "msg.msgid");

	}else if (Copter::supt->getParamValueWithNameAndKey("guided_set_destination_posvel", "msg.msgid") != -1){

		choose = Copter::supt->getParamValueWithNameAndKey("guided_set_destination_posvel", "msg.msgid");

	}
	else if (Copter::supt->getParamValueWithNameAndKey("guided_set_velocity", "msg.msgid") != -1){

		choose = Copter::supt->getParamValueWithNameAndKey("guided_set_velocity", "msg.msgid");

	}
	else if (Copter::supt->getParamValueWithNameAndKey("guided_set_destination", "msg.msgid") != -1){

		choose = Copter::supt->getParamValueWithNameAndKey("guided_set_destination", "msg.msgid");

	}
	else if (Copter::supt->getParamValueWithNameAndKey("mode_has_manual_throttle", "msg.msgid") != -1){

		choose = Copter::supt->getParamValueWithNameAndKey("mode_has_manual_throttle", "msg.msgid");

	}


	if (choose == 76){
		msg->msgid = MAVLINK_MSG_ID_COMMAND_LONG;
	}
	else if (choose == 84){
		msg->msgid = MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED;
	}
	else if (choose == 86){
		msg->msgid = MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT;
	}
	else{
		cout << "wrong number" << endl;
	}
	
	switch (msg->msgid) {//模型仅有76 84 86 故修改以与其对应
		//Pre-Flight calibration requests
	case MAVLINK_MSG_ID_COMMAND_LONG:       // MAV ID: 76
	{
		cout << "in MAVLINK_MSG_ID_COMMAND_LONG";
		// decode packet
		mavlink_command_long_t packet;
		mavlink_msg_command_long_decode(msg, &packet);

		switch (packet.command) {
			//修改以对应模型
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
			// ------------------------  插桩点 ---------------------------------
			start = clock();
			Copter::supt->setCurProcessResult("do_user_takeoff", start, 1);

			// ------------------------  插桩激励 ---------------------------------
			if (copter.do_user_takeoff(takeoff_alt, is_zero(packet.param3))) {
				result = MAV_RESULT_ACCEPTED;
			}
			else {
				result = MAV_RESULT_FAILED;
			}
			end = clock();
			Copter::supt->setCurProcessResult("do_user_takeoff", end, 2);
			Copter::supt->setCurProcessResult("do_user_takeoff", (end - start), 3);
			break;
		}
		}
	}
		case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:     // MAV ID: 84
		{
			cout << "in MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED";
			// decode packet
			mavlink_set_position_target_local_ned_t packet;
			mavlink_msg_set_position_target_local_ned_decode(msg, &packet);

			// exit if vehicle is not in Guided mode or Auto-Guided mode
			if ((copter.control_mode != GUIDED) && !(copter.control_mode == AUTO && copter.auto_mode == Auto_NavGuided)) {
				break;
			}

			// check for supported coordinate frames
			if (packet.coordinate_frame != MAV_FRAME_LOCAL_NED &&
				packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED &&
				packet.coordinate_frame != MAV_FRAME_BODY_NED &&
				packet.coordinate_frame != MAV_FRAME_BODY_OFFSET_NED) {
				break;
			}

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

			// send request
			if (!pos_ignore && !vel_ignore && acc_ignore) { 
				// ------------------------  插桩点 ---------------------------------
				start = clock();
				Copter::supt->setCurProcessResult("guided_set_destination_posvel", start, 1);

				// ------------------------  插桩激励 ---------------------------------
				copter.guided_set_destination_posvel(pos_vector, vel_vector);
				end = clock();
				Copter::supt->setCurProcessResult("guided_set_destination_posvel", end, 2);
				Copter::supt->setCurProcessResult("guided_set_destination_posvel", (end - start), 3);
			}
			else if (pos_ignore && !vel_ignore && acc_ignore) {
				// ------------------------  插桩点 ---------------------------------
				start = clock();
				Copter::supt->setCurProcessResult("guided_set_velocity", start, 1);

				// ------------------------  插桩激励 ---------------------------------
				copter.guided_set_velocity(vel_vector);
				end = clock();
				Copter::supt->setCurProcessResult("guided_set_velocity", end, 2);
				Copter::supt->setCurProcessResult("guided_set_velocity", (end - start), 3);
			}
			else if (!pos_ignore && vel_ignore && acc_ignore) {
				// ------------------------  插桩点 ---------------------------------
				start = clock();
				Copter::supt->setCurProcessResult("guided_set_destination", start, 1);

				// ------------------------  插桩激励 ---------------------------------
				copter.guided_set_destination(pos_vector);
				end = clock();
				Copter::supt->setCurProcessResult("guided_set_destination", end, 2);
				Copter::supt->setCurProcessResult("guided_set_destination", (end - start), 3);
			}
			else {
				result = MAV_RESULT_FAILED;
			}

			break;
		}

		case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:    // MAV ID: 86
		{
			cout << "in MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT";
			// decode packet
			mavlink_set_position_target_global_int_t packet;
			mavlink_msg_set_position_target_global_int_decode(msg, &packet);

			// exit if vehicle is not in Guided mode or Auto-Guided mode
			if ((copter.control_mode != GUIDED) && !(copter.control_mode == AUTO && copter.auto_mode == Auto_NavGuided)) {
				break;
			}

			// check for supported coordinate frames
			if (packet.coordinate_frame != MAV_FRAME_GLOBAL_INT &&
				packet.coordinate_frame != MAV_FRAME_GLOBAL_RELATIVE_ALT && // solo shot manager incorrectly sends RELATIVE_ALT instead of RELATIVE_ALT_INT
				packet.coordinate_frame != MAV_FRAME_GLOBAL_RELATIVE_ALT_INT &&
				packet.coordinate_frame != MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) {
				break;
			}

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

			if (!pos_ignore && !vel_ignore && acc_ignore) {

				// ------------------------  插桩点 ---------------------------------
				start = clock();
				Copter::supt->setCurProcessResult("guided_set_destination_posvel", start, 1);

				// ------------------------  插桩激励 ---------------------------------
				copter.guided_set_destination_posvel(pos_ned, Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f));
				end = clock();
				Copter::supt->setCurProcessResult("guided_set_destination_posvel", end, 2);
				Copter::supt->setCurProcessResult("guided_set_destination_posvel", (end - start), 3);
			}
			else if (pos_ignore && !vel_ignore && acc_ignore) {
				// ------------------------  插桩点 ---------------------------------
				start = clock();
				Copter::supt->setCurProcessResult("guided_set_velocity", start, 1);

				// ------------------------  插桩激励 ---------------------------------
				copter.guided_set_velocity(Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f));
				end = clock();
				Copter::supt->setCurProcessResult("guided_set_velocity", end, 2);
				Copter::supt->setCurProcessResult("guided_set_velocity", (end - start), 3);
			}
			else if (!pos_ignore && vel_ignore && acc_ignore) {
				/*if (!copter.guided_set_destination(pos_ned)) {
					result = MAV_RESULT_FAILED;
					}*/
			}
			else {
				result = MAV_RESULT_FAILED;
			}
			break;
		}
		}
		cout << "----- handleMessage end -----" << endl;
	} 
 
