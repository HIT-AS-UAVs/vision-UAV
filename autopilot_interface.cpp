/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "autopilot_interface.h"


// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}


// ----------------------------------------------------------------------------------
//   设置目标点的函数
// ----------------------------------------------------------------------------------

// choose one of the next three
/*
 * Set target local ned position
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target XYZ locations
 * in the Local NED frame, in meters.
 */
void
set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;
	sp.coordinate_frame = MAV_FRAME_BODY_NED;

	sp.x   = x;
	sp.y   = y;
	sp.z   = z;
	printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);

}

/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */
void
set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
			MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	//sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.coordinate_frame = MAV_FRAME_BODY_NED;
	sp.vx  = vx;
	sp.vy  = vy;
	sp.vz  = vz;
	//printf("VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);
}

/*
 * Set target local ned acceleration
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target AX AY AZ
 * accelerations in the Local NED frame, in meters per second squared.
 */
void
set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp)
{

	// NOT IMPLEMENTED
	fprintf(stderr,"set_acceleration doesn't work yet \n");
	throw 1;
	sp.type_mask =
			MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION &
			MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	//sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.coordinate_frame = MAV_FRAME_BODY_NED;
	sp.afx  = ax;
	sp.afy  = ay;
	sp.afz  = az;
}

// the next two need to be called after one of the above
/*
 * Set target local ned yaw
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw
 * in the Local NED frame, in radians.
 */
void
set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;

	sp.yaw  = yaw;

	printf("POSITION SETPOINT YAW = %.4f \n", sp.yaw);

}

/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
void
set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE ;

	sp.yaw_rate  = yaw_rate;
}


/*
 * 设置全局目标坐标点，坐标系GLOBAL_RELATIVE_ALT_INT
 */
void
set_global_position(float x,float y,float z,mavlink_set_position_target_global_int_t &sp)
{
	sp.type_mask = MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_POSITION;
	sp.coordinate_frame =
            MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;

	sp.lat_int   = x;
	sp.lon_int   = y;
	sp.alt   = z;
}

/*
 * 设置global_velocity
 */
void
set_global_velocity(float vx, float vy, float vz, mavlink_set_position_target_global_int_t &sp)
{
	sp.type_mask =
			MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_VELOCITY ;

	sp.coordinate_frame =
	        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
	sp.vx  = vx;
	sp.vy  = vy;
	sp.vz  = vz;
}

/*
 * 设置全局加速度
 * MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
 */
void
set_global_acceleration(float ax, float ay, float az, mavlink_set_position_target_global_int_t &sp)
{

	// NOT IMPLEMENTED
	fprintf(stderr,"set_acceleration doesn't work yet \n");
	throw 1;

	sp.type_mask =
			MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_ACCELERATION &
			MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_VELOCITY;
	sp.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
	sp.afx  = ax;
	sp.afy  = ay;
	sp.afz  = az;

}

// the next two need to be called after one of the above

/*
 * 设置指向
 */
void
set_global_yaw(float yaw, mavlink_set_position_target_global_int_t &sp)
{
	sp.type_mask &= MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_YAW_ANGLE;
	sp.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
	sp.yaw  = yaw;

	printf("POSITION SETPOINT YAW = %.4f \n", sp.yaw);
}

/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
void
set_global_yaw_rate(float yaw_rate, mavlink_set_position_target_global_int_t &sp)
{
	sp.type_mask &=
			MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_YAW_RATE ;

	sp.yaw_rate  = yaw_rate;
}


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::
Autopilot_Interface(Serial_Port *serial_port_)
{
	// initialize attributes
	write_count = 0;

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
	control_status = 0;      // whether the autopilot is in offboard control mode
	time_to_exit   = false;  // flag to signal thread exit

	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	serial_port = serial_port_; // serial port management object

}

Autopilot_Interface::
~Autopilot_Interface()
{}


// ------------------------------------------------------------------------------
//   Update Global Setpoint
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
update_global_setpoint(mavlink_set_position_target_global_int_t set_global_point)
{
	current_global_setpoint = set_global_point;
	write_global_setpoint();
}

// ------------------------------------------------------------------------------
//   Update Local Setpoint
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
update_local_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
	current_local_setpoint = setpoint;
	write_local_setpoint();
}



// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void Autopilot_Interface::
read_messages()
{
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;
	std::ofstream outf1;
	outf1.open("messageID.txt");
	// Blocking wait for new data
	while ( !received_all and !time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = serial_port->read_message(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{

			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;
			std::cout<<"messageID:"<<(float)message.msgid<<std::endl;
			outf1 << "messageID:"<<(float)message.msgid<<std::endl;

			// Handle Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					printf("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					printf("MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					break;
				}

				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					break;
				}

				case MAVLINK_MSG_ID_RADIO_STATUS:
				{
					printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
					mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
					current_messages.time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
					break;
				}

				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
					printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
					mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
					current_messages.time_stamps.local_position_ned = get_time_usec();
					this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
					break;
				}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
                    std::cout<<"lat:"<<current_messages.global_position_int.lat<<std::endl;
					current_messages.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
				{
					printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
					current_messages.time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
					std::cout<<"local_position.x:"<<current_messages.local_position_ned.x<<std::endl
                             <<"local_position.y:"<<current_messages.local_position_ned.y<<std::endl
                             <<"local_position.z:"<<current_messages.local_position_ned.z<<std::endl;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
					printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
					current_messages.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					break;
				}

				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE:
				{
					printf("MAVLINK_MSG_ID_ATTITUDE\n");
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					break;
				}
				case MAVLINK_MSG_ID_SET_MODE:
				{
					printf("MAVLINK_MSG_ID_SET_MODE\n");
					mavlink_msg_set_mode_decode(&message, &(current_messages.set_mode));
					std::cout<<"base_mode:"<<current_messages.set_mode.base_mode<<std::endl
							 <<"custom_mode:"<<current_messages.set_mode.custom_mode<<std::endl;
					current_messages.time_stamps.setmode = get_time_usec();
					this_timestamps.setmode = current_messages.time_stamps.setmode;
					break;
				}
				case MAVLINK_MSG_ID_COMMAND_LONG:
				{
					printf("MAVLINK_MSG_ID_COMMAND_LONG\n");
					mavlink_msg_command_long_decode(&message, &(current_messages.command_long));
					std::cout<<"command:"<<current_messages.command_long.command<<std::endl
							 <<"confirmation:"<<(float)current_messages.command_long.confirmation<<std::endl
							<<"param1:"<<current_messages.command_long.param1<<std::endl
					<<"param2:"<<current_messages.command_long.param2<<std::endl
					<<"param3:"<<current_messages.command_long.param3<<std::endl
					<<"param4:"<<current_messages.command_long.param4<<std::endl
					<<"param5:"<<current_messages.command_long.param5<<std::endl
					<<"param6:"<<current_messages.command_long.param6<<std::endl
					<<"param7:"<<current_messages.command_long.param7<<std::endl;
					current_messages.time_stamps.command_long = get_time_usec();
					this_timestamps.command_long = current_messages.time_stamps.command_long;
					break;
				}
				case MAVLINK_MSG_ID_MISSION_ITEM:
				{
					printf("MAVLINK_MSG_ID_MISSION_ITEM\n");
					mavlink_msg_mission_item_decode(&message, &(current_messages.mission_item));
					std::cout<<"frame:"<<(float)current_messages.mission_item.frame<<std::endl
							 <<"command:"<<current_messages.mission_item.command<<std::endl
							 <<"current:"<<(float)current_messages.mission_item.current<<std::endl
							 <<"param2:"<<current_messages.mission_item.param2<<std::endl
							 <<"param3:"<<current_messages.mission_item.param3<<std::endl
							 <<"param4:"<<current_messages.mission_item.param4<<std::endl
							 <<"param1:"<<current_messages.mission_item.param1<<std::endl
							 <<"autocontinue:"<<(float)current_messages.mission_item.autocontinue<<std::endl
							 <<"z:"<<current_messages.mission_item.z<<std::endl;
					current_messages.time_stamps.mission_item = get_time_usec();
					this_timestamps.mission_item = current_messages.time_stamps.mission_item;
					break;
				}
				case MAVLINK_MSG_ID_COMMAND_ACK:
                {
                    printf("MAVLINK_MSG_ID_COMMAND_ACK\n");
                    mavlink_msg_command_ack_decode(&message, &(current_messages.command_ack));
                    std::cout<<"command:"<<(float)current_messages.command_ack.command<<std::endl
                             <<"result:"<<(float)current_messages.command_ack.result<<std::endl;
                    current_messages.time_stamps.command_ack = get_time_usec();
                    this_timestamps.command_ack = current_messages.time_stamps.command_ack;
                    break;
                }

				case MAVLINK_MSG_ID_PARAM_VALUE:
				{
					printf("MAVLINK ID PARAM_VALUE!\n");
					mavlink_msg_param_value_decode(&message,&(current_messages.param_value));
					this_timestamps.param_value = current_messages.time_stamps.param_value;
					std::cout<<"param_id:"<<current_messages.param_value.param_id<<std::endl
							 <<"param_value:"<<current_messages.param_value.param_value<<std::endl
							 <<"param_type:"<<(float)current_messages.param_value.param_type<<std::endl;
					break;
				}
				case MAVLINK_MSG_ID_STATUSTEXT:
				{
					printf("Mavlink ID statustext!\n");
					mavlink_msg_statustext_decode(&message,&(current_messages.statustext));
					this_timestamps.statustext = current_messages.time_stamps.statustext;
					std::cout<<"severity:"<<(float)current_messages.statustext.severity<<std::endl
							 <<"text:"<<current_messages.statustext.text<<std::endl;
					break;
				}

                case MAVLINK_MSG_ID_MISSION_CURRENT:
                {
                    printf("Mavlink ID MISSIONCURRENT!\n");
                    mavlink_msg_mission_current_decode(&message,&(current_messages.mission_current));
                    this_timestamps.mission_current = current_messages.time_stamps.mission_current;
                    std::cout<<"missioncurrent.seq:"<<current_messages.mission_current.seq<<std::endl;
                }

				default:
				{
					 printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}


			} // end: switch msgid

		} // end: if read message

		// Check for receipt of all items
		received_all =
				this_timestamps.heartbeat                  &&
//				this_timestamps.battery_status             &&
//				this_timestamps.radio_status               &&
//				this_timestamps.local_position_ned         &&
//				this_timestamps.global_position_int        &&
//				this_timestamps.position_target_local_ned  &&
//				this_timestamps.position_target_global_int &&
//				this_timestamps.highres_imu                &&
//				this_timestamps.attitude                   &&
				this_timestamps.sys_status
				;

		// give the write thread time to use the port
		if ( writing_status > false ) {
			usleep(100); // look for components of batches at 10kHz
		}

	} // end: while not received all

	return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
write_message(mavlink_message_t message)
{
	// do the write
	int len = serial_port->write_message(message);

	// book keep
	write_count++;

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_global_setpoint()
{
	mavlink_set_position_target_global_int_t sp = current_global_setpoint;
	// double check some system parameters
	if ( not sp.time_boot_ms )
		sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	sp.target_system    = system_id;
	sp.target_component = autopilot_id;
	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------
	mavlink_message_t message;
	mavlink_msg_set_position_target_global_int_encode(system_id, companion_id, &message, &sp);
	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------
	int len = write_message(message);
	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send POSITION_TARGET_GLOBAL_INT \n");
	else
		printf("%lu Global_POSITION_TARGET  = [ %4f , %4f , %4f ] \n", write_count, (float)sp.lat_int, (float)sp.lon_int, sp.alt);
	std::cout<<sp.type_mask<<std::endl;
	return;
}

void
Autopilot_Interface::
write_local_setpoint()
{
	// pull from position target
	mavlink_set_position_target_local_ned_t sp = current_local_setpoint;

	// double check some system parameters
	if ( not sp.time_boot_ms )
		sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	sp.target_system    = system_id;
	sp.target_component = autopilot_id;
	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------
	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);
	// do the write
	int len = write_message(message);
	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
	else
		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, sp.x, sp.y, sp.z);
	std::cout<<sp.type_mask<<std::endl;

	return;
}


// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
enable_offboard_control()
{
	// Should only send this command once
	if ( control_status == false )
	{
		printf("ENABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to go off-board
		int success = toggle_offboard_control( true );

		// Check the command was written
		if ( success )
			control_status = true;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if not offboard_status

}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
disable_offboard_control()
{

	// Should only send this command once
	if ( control_status == true )
	{
		printf("DISABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to stop off-board
//		int success = toggle_offboard_control( false );

		// Check the command was written
		if (true )
			control_status = false;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if offboard_status
//MAV_CMD_MISSION_START
}


// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
toggle_offboard_control( bool flag )
{
	// Prepare command for off-board mode
//	mavlink_command_long_t com = { 0 };
//	com.target_system    = system_id;
//	com.target_component = autopilot_id;
//	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
//	com.confirmation     = true;
//	com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop
	//////////////////////自稳模式
    mavlink_set_mode_t com = { 0 };
	com.base_mode = 1;
	com.target_system = 01;
	com.custom_mode = 00;
	// Encode
	mavlink_message_t message;
	mavlink_msg_set_mode_encode(255, 190, &message, &com);

	// Send the message
		int len = serial_port->write_message(message);
	// Done!

	///////请求数据流(关闭ALL)
//	mavlink_request_data_stream_t com7 = { 0 };
//	com7.target_system= 01;
//	com7.target_component = 01;
//	com7.req_stream_id = MAV_DATA_STREAM_ALL;
//	com7.req_message_rate = 2;
//	com7.start_stop = 0;
//
//	mavlink_message_t message7;
//	mavlink_msg_request_data_stream_encode(255, 190, &message7, &com7);

//	 Send the message
//	int len7 = serial_port->write_message(message7);

	/////////////////////////////////请求数据流
	mavlink_request_data_stream_t comdata = { 0 };
	comdata.req_message_rate = 2;
	comdata.req_stream_id = MAV_DATA_STREAM_POSITION;
	comdata.start_stop = 1;
	comdata.target_system = 1;
	comdata.target_component = 1;
	mavlink_message_t Rmassage;
	mavlink_msg_request_data_stream_encode(255,190,&Rmassage,&comdata);
	//重复发送确保指令收到
	for (int i = 0; i < 3; ++i)
	{
		int Rlen = serial_port->write_message(Rmassage);
		usleep(100);
	}
	sleep(2);
	printf("current global message %f",(float)current_messages.global_position_int.lat);
//    std::cout<<"lat:"<<current_messages.global_position_int.lat<<std::endl
//             <<"lon:"<<current_messages.global_position_int.lon<<std::endl;


/*
	////////////////////////////////////////解锁
	mavlink_command_long_t com1 = { 0 };
	com1.target_system= 01;
	com1.target_component = 01;
	com1.command = MAV_CMD_MISSION_START;
	com1.param1 = 1;
	mavlink_message_t message1;
	mavlink_msg_command_long_encode(255, 190, &message1, &com1);
	// Send the message
	int len1 = serial_port->write_message(message1);
	usleep(100);
*/

	//////////////////////guided模式
	mavlink_set_mode_t com5 = { 0 };
	com5.base_mode = 1;
	com5.target_system = 01;
	com5.custom_mode = 04;
	// Encode
	mavlink_message_t message5;
	mavlink_msg_set_mode_encode(255, 190, &message5, &com5);
	// Send the message
		int len5 = serial_port->write_message(message5);
	// Done!

/*
	/////////////////////////////////起飞
	mavlink_command_long_t com2 = { 0 };
	com2.target_system= 01;
	com2.target_component = 01;
	com2.command = 22;
	com2.param7 = 10;//高度设定10m

	mavlink_message_t message2;
	mavlink_msg_command_long_encode(255, 190, &message2, &com2);

	// Send the message
	int len2 = serial_port->write_message(message2);


    ///////////////////////////////////mission本体FLU设目标点
    mavlink_command_int_t command;
    command.target_system = 01;
    command.target_component = 01;
    command.command = 16;
    command.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;//高度设定10m
    command.autocontinue = 1;
    command.current = 1;
    command.x = current_messages.global_position_int.lat;
    command.y = current_messages.global_position_int.lon;
    command.z = 20;
    command.param1 = 1;
    command.param2 = 1;
    command.param3 = 0;
    mavlink_message_t Bmessage;
    mavlink_msg_command_int_encode(255, 190, &Bmessage, &command);
    int lenB = write_message(Bmessage);
    if (lenB <= 0)
    {
        printf("Body position write wrong!");
        printf("[%f,%f,%f]", command.x, command.y, command.z);
    }
    else {
        printf("成功写入本体坐标系坐标点\n");
        std::cout<<"commission.x:"<<command.x<<std::endl<<"commission.y:"<<command.y<<std::endl;
    }

*/
  /*  ///////////////////////////////////mission全局设目标点
	mavlink_mission_item_t commission11;
	commission11.target_system = 01;
	commission11.target_component = 01;
	commission11.command = 16;
	commission11.seq = 1;
	commission11.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
	commission11.autocontinue = 1;
	commission11.current = 1;
	commission11.x = current_messages.global_position_int.lat;
	commission11.y = current_messages.global_position_int.lon;
	commission11.z = 25;
	commission11.param1 = 1;
	commission11.param2 = 1;
	commission11.param3 = 0;

	mavlink_message_t GBmessage;
	mavlink_msg_mission_item_encode(255, 190, &GBmessage, &commission11);
	int lenGB = write_message(GBmessage);
	if (lenGB <= 0)
	{
		printf("Body position write wrong!");
		printf("[%f,%f,%f]", commission11.x, commission11.y, commission11.z);
	}
	else {
		printf("成功写入mission 全局坐标系坐标点\n");
		printf("[%f,%f,%f]\n", (float)commission11.x, (float)commission11.y, (float)commission11.z);
	}

    ///////////////////////////////////mission全局设目标点
    mavlink_mission_item_t commission12;
    commission12.target_system = 01;
    commission12.target_component = 01;
    commission12.command = 16;
    commission12.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    commission12.autocontinue = 1;
    commission12.seq = 2;
    commission12.current = 1;
    commission12.x = current_messages.global_position_int.lat;
    commission12.y = current_messages.global_position_int.lon;
    commission12.z = 20;
    commission12.param1 = 1;
    commission12.param2 = 1;
    commission12.param3 = 0;

    mavlink_message_t G2Bmessage;
    mavlink_msg_mission_item_encode(255, 190, &G2Bmessage, &commission12);
    int lenG2B = write_message(G2Bmessage);
    if (lenG2B <= 0)
    {
        printf("Body position write wrong!");
        printf("[%f,%f,%f]", commission12.x, commission12.y, commission12.z);
    }
    else {
        printf("成功写入mission 全局坐标系坐标点\n");
        printf("[%f,%f,%f]\n", (float)commission12.x, (float)commission12.y, (float)commission12.z);
    }
*/
	//////////////////////////////////////请求mission
	mavlink_mission_clear_all_t Rcomm1;
	Rcomm1.target_system = 01;
	Rcomm1.target_component = 01;
	mavlink_message_t R_comm1;
	mavlink_msg_mission_clear_all_encode(255,190,&R_comm1,&Rcomm1);
	for (int j = 0; j < 4; ++j)
	{
		int Rlen = write_message(R_comm1);
	}
	sleep(2);

  //////////////////////////测试mission
	mavlink_mission_item_int_t commission12;
	commission12.target_system = 01;
	commission12.target_component = 01;
	commission12.command = 16;
	commission12.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
	commission12.autocontinue = 1;
	commission12.seq = 1;
	commission12.current = 1;
	commission12.x = current_messages.global_position_int.lat;
	commission12.y = current_messages.global_position_int.lon;
	commission12.z = 20;
	commission12.param1 = 1;
	commission12.param2 = 1;
	commission12.param3 = 0;

	mavlink_message_t G2Bmessage;
	mavlink_msg_mission_item_int_encode(255, 190, &G2Bmessage, &commission12);
	int lenG2B = write_message(G2Bmessage);


	mavlink_mission_item_int_t commission13;
	commission13.target_system = 01;
	commission13.target_component = 01;
	commission13.command = 16;
	commission13.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
	commission13.autocontinue = 1;
	commission13.seq = 2;
	commission13.current = 1;
	commission13.x = current_messages.global_position_int.lat;
	commission13.y = current_messages.global_position_int.lon;
	commission13.z = 20;
	commission13.param1 = 1;
	commission13.param2 = 1;
	commission13.param3 = 0;
	mavlink_message_t GBmessage;
	mavlink_msg_mission_item_int_encode(255, 190, &GBmessage, &commission13);
	int lenGB = write_message(GBmessage);

	//////////////////////////////////////请求mission
    mavlink_mission_request_list_t Rcomm;
    Rcomm.target_system = 01;
    Rcomm.target_component = 01;
    mavlink_message_t R_comm;
    mavlink_msg_mission_request_list_encode(255,190,&R_comm,&Rcomm);
	for (int j = 0; j < 3; ++j)
	{
		int Rlen = write_message(R_comm);
	}


    mavlink_mission_request_int_t req_int;
	req_int.target_system =01;
	req_int.target_component =01;
	req_int.seq = 1;
	mavlink_message_t Rint;
	mavlink_msg_mission_request_int_encode(255,190,&Rint,&req_int);
	for (int k = 0; k < 3; ++k) {
		int lennn = write_message(Rint);
	}


    //////////////////////AUTO模式
    mavlink_set_mode_t com15 = { 0 };
    com15.base_mode = 1;
    com15.target_system = 01;
    com15.custom_mode = 03;
    // Encode
    mavlink_message_t message15;
    mavlink_msg_set_mode_encode(255, 190, &message15, &com15);
    // Send the message
    int len15 = serial_port->write_message(message15);


    //////////////////////////////////开始misiion
    mavlink_command_long_t mission_start = { 0 };
    mission_start.target_system= 01;
    mission_start.target_component = 01;
    mission_start.command = 300;
    mission_start.confirmation = 1;
    mission_start.param1 = 0;
    mission_start.param2 = 2;

    mavlink_message_t Mission_starmessage;
    mavlink_msg_command_long_encode(255, 190, &Mission_starmessage, &mission_start);

    // Send the message
    int lenmission = serial_port->write_message(Mission_starmessage);
    if (lenmission <= 0)
    {
        printf("Start Mission error!\n");
        //printf("[%f,%f,%f]", commission12.x, commission12.y, commission12.z);
    }
    else {
        printf("Start Mission!\n");
        //printf("[%f,%f,%f]\n", (float)commission12.x, (float)commission12.y, (float)commission12.z);
    }



    /*****************
     /////////////////////////////////返航
    mavlink_command_long_t com3 = { 0 };
    com3.target_system= 01;
    com3.target_component = 01;
    com3.command = 20;

    mavlink_message_t message3;
    mavlink_msg_command_long_encode(255, 190, &message3, &com3);

    // Send the message
    int len3 = serial_port->write_message(message3);
**************/
	return len;
}


// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void Autopilot_Interface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK SERIAL PORT
	// --------------------------------------------------------------------------

	if ( serial_port->status != 1 ) // SERIAL_PORT_OPEN
	{
		fprintf(stderr,"ERROR: serial port not open\n");
		throw 1;
	}


	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------

	printf("START READ THREAD \n");

	result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
	if ( result ) throw result;

	// now we're reading messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   CHECK FOR MESSAGES
	// --------------------------------------------------------------------------

	printf("CHECK FOR MESSAGES\n");

	while ( not current_messages.sysid )
	{
		if ( time_to_exit )
			return;
		usleep(500000); // check at 2Hz
	}

	printf("Found\n");

	// now we know autopilot is sending messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------

	// This comes from the heartbeat, which in theory should only come from
	// the autopilot we're directly connected to it.  If there is more than one
	// vehicle then we can't expect to discover id's like this.
	// In which case set the id's manually.

	// System ID
	if ( not system_id )
	{
		system_id = current_messages.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}

	// Component ID
	if ( not autopilot_id )
	{
		autopilot_id = current_messages.compid;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		printf("\n");
	}


	// --------------------------------------------------------------------------
	//   GET INITIAL POSITION
	// --------------------------------------------------------------------------

	// Wait for initial position ned
//	while ( not ( current_messages.time_stamps.local_position_ned &&
//				  current_messages.time_stamps.attitude            )  )
	while ( not true )
	{
		if ( time_to_exit )
			return;
		usleep(500000);
	}

	// copy initial position ned
	Mavlink_Messages local_data = current_messages;
	initial_position.x        = local_data.local_position_ned.x;
	initial_position.y        = local_data.local_position_ned.y;
	initial_position.z        = local_data.local_position_ned.z;
	initial_position.vx       = local_data.local_position_ned.vx;
	initial_position.vy       = local_data.local_position_ned.vy;
	initial_position.vz       = local_data.local_position_ned.vz;
	initial_position.yaw      = local_data.attitude.yaw;
	initial_position.yaw_rate = local_data.attitude.yawspeed;

	//初始化global_position
	initial_global_position.lat_int = local_data.global_position_int.lat;
	initial_global_position.lon_int = local_data.global_position_int.lon;
	initial_global_position.alt = local_data.global_position_int.alt;
	initial_global_position.vx = local_data.global_position_int.vx;
	initial_global_position.vy = local_data.global_position_int.vy;
	initial_global_position.vz = local_data.global_position_int.vz;
	initial_global_position.yaw = local_data.global_position_int.hdg;
	printf("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
	printf("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
	printf("\n");

	// we need this before starting the write thread


	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	printf("START WRITE THREAD \n");

	result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this );
	if ( result ) throw result;

	// wait for it to be started
	while ( not writing_status )
		usleep(100000); // 10Hz

	// now we're streaming setpoint commands
	printf("\n");


	// Done!
	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);

	// now the read and write threads are closed
	printf("\n");

	// still need to close the serial_port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_read_thread()
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_write_thread(void)
{
	if ( not writing_status == false )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
handle_quit( int sig )
{

	disable_offboard_control();

	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}

}



// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_thread()
{
	reading_status = true;

	while ( ! time_to_exit )
	{
		read_messages();
		usleep(1000); // Read batches at 10Hz
	}

	reading_status = false;

	return;
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_thread(void)
{
	// signal startup
	writing_status = 2;

	// prepare an initial setpoint, just stay put
	mavlink_set_position_target_local_ned_t sp;
//	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
//				   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
//	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
//	sp.vx       = 0.0;
//	sp.vy       = 0.0;
//	sp.vz       = 0.0;
//	sp.yaw_rate = 0.0;
//
	// set position target
//	current_setpoint = sp;

	// write a message and signal writing
//	write_setpoint();
	writing_status = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while ( !time_to_exit )
	{
		usleep(250000);   // Stream at 4Hz
//		write_setpoint();
	}

	// signal end
	writing_status = false;

	return;

}

// End Autopilot_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_autopilot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_read_thread();

	// done!
	return NULL;
}

void*
start_autopilot_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_write_thread();

	// done!
	return NULL;
}



