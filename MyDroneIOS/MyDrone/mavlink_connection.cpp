//
//  mavlink_connection.cpp
//  MyDrone
//
//  Created by kangzhiyong on 2020/2/26.
//
#include "mavlink/common/common.hpp"
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink/common/mavlink.h"
#include "mavlink/ardupilotmega/ardupilotmega.h"

#include "mavlink_connection.hpp"
#include "StringUtils.h"
#include "mavlink_utils.hpp"

MavlinkConnection::MavlinkConnection(std::string destAddr, bool threaded, bool PX4, double send_rate, time_t timeout)
{
    /*
    Constructor for Mavlink based drone connection.
    Note: When threaded, the read loop runs as a daemon, meaning once all
    other processes stop the thread immediately dies, therefore some
    acitivty (e.g. a while True loop) needs to be running on the main
    thread for this thread to survive.

    Args:
        device: address to the drone, e.g. "tcp:127.0.0.1:5760" (see mavutil mavlink connection for valid options)
        threaded: bool for whether or not to run the message read loop on a separate thread
        PX4: bool for whether or not connected to a PX4 autopilot. Determines the behavior of the
            command loop (write thread)
        send_rate: the rate in Hertz (Hz) to send messages to the drone
        timeout: how long to wait for a message before the connection times out (seconds)
    */
    // create the connection
    _master = new mavudp(destAddr);
    
    int mavlinkChannel = reserveMavlinkChannel();
    if (mavlinkChannel != 0) {
        _target_channel = mavlinkChannel;
    } else {
        printf("Ran out of mavlink channels\n");
        return;
    }
    
    _threaded = threaded;

    // PX4 management
    _using_px4 = PX4;
    _send_rate = send_rate;

    // seconds to wait of no messages before termination
    _timeout = timeout;
}

int MavlinkConnection::reserveMavlinkChannel(void)
{
    // Find a mavlink channel to use for this link, Channel 0 is reserved for internal use.
    for (uint8_t mavlinkChannel = 1; mavlinkChannel < MAVLINK_COMM_NUM_BUFFERS; mavlinkChannel++) {
        if (!(_mavlinkChannelsUsedBitMask & 1 << mavlinkChannel)) {
            mavlink_reset_channel_status(mavlinkChannel);
            // Start the channel on Mav 1 protocol
            mavlink_status_t* mavlinkStatus = mavlink_get_channel_status(mavlinkChannel);
            mavlinkStatus->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
            _mavlinkChannelsUsedBitMask |= 1 << mavlinkChannel;
            return mavlinkChannel;
        }
    }
    return 0;   // All channels reserved
}

bool MavlinkConnection::open()
{
    return (_master->get_socket_fd() == -1)? false : true;
}

void MavlinkConnection::dispatch_loop()
{
    /*
    Main loop to read from the drone.

    Continually listens to the drone connection for incoming messages.
    for each new message, parses out the mavlink, creates messages as
    defined in `message_types.py`, and triggers all callbacks registered
    for that type of message.
    Also keeps an eye on the state of connection, and if nothing has
    happened in more than 5 seconds, sends a special termination message
    to indicate that the drone connection has died.

    THIS SHOULD NOT BE CALLED DIRECTLY BY AN OUTSIDE CLASS!
    */
    bool valid_msg = false;
    time_t last_msg_time = time(nullptr);
    while(_running)
    {
        // wait for a new message
        mavlink_message_t msg;
        valid_msg = wait_for_message(msg);

        // if no message or a bad message was received, just move along
        if (!valid_msg)
            continue;

        time_t current_time = time(nullptr);

        // print("Time between messages", current_time - last_msg_time)

        // if we haven't heard a message in a given amount of time
        // send a termination message
//        if (current_time - last_msg_time > _timeout)
//        {
//            MessageBase data;
//            // notify listeners that the connection is closing
//            notify_message_listeners(CONNECTION_CLOSED, data);
//
//            // stop this read loop
//            _running = false;
//        }
        // update the time of the last message
        last_msg_time = current_time;

        dispatch_message(msg);
        // give the write thread time to use the port
        if ( writing_status > false ) {
            usleep(100); // look for components of batches at 10kHz
        }
    }
}

/*
 parse out the message based on the type and call
 the appropriate callbacks
 */
void MavlinkConnection::dispatch_message(mavlink_message_t msg)
{
    _target_system = msg.sysid;
    autopilot_id = msg.compid;
    
    switch (msg.msgid) {
        // http://mavlink.org/messages/common/#GLOBAL_POSITION_INT
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {
            mavlink_global_position_int_t gpi_msg;
            mavlink_msg_global_position_int_decode(&msg, &gpi_msg);
            
            uint32_t timestamp = gpi_msg.time_boot_ms / 1000.0;
            // parse out the gps position and trigger that callback
            GlobalFrameMessage gps(timestamp, double(gpi_msg.lat) / 1e7, double(gpi_msg.lon) / 1e7, double(gpi_msg.alt) / 1000);
            notify_message_listeners(GLOBAL_POSITION, &gps);

            // parse out the velocity and trigger that callback
            LocalFrameMessage vel(timestamp, double(gpi_msg.vx) / 100, double(gpi_msg.vy) / 100, double(gpi_msg.vz) / 100);
            notify_message_listeners(LOCAL_VELOCITY, &vel);
            break;
        }
        // http://mavlink.org/messages/common/#HEARTBEAT
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            mavlink_heartbeat_t hrt_msg;
            mavlink_msg_heartbeat_decode(&msg, &hrt_msg);
            
            uint32_t timestamp = 0.0;
            uint8_t motors_armed = (hrt_msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;

            // determine if want to broadcast all current mode types
            // not just boolean on manual
            bool guided_mode = false;

            // extract whether or not we are in offboard mode for PX4
            // (the main mode)
            uint32_t main_mode = (hrt_msg.custom_mode & 0x000F0000) >> 16;
            if (main_mode == PX4_MODE_OFFBOARD)
            {
                guided_mode = true;
            }
            StateMessage state(timestamp, motors_armed, guided_mode, hrt_msg.system_status);
            notify_message_listeners(STATE, &state);
            break;
        }
        // http://mavlink.org/messages/common#LOCAL_POSITION_NED
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        {
            mavlink_local_position_ned_t lpn_msg;
            mavlink_msg_local_position_ned_decode(&msg, &lpn_msg);
            
            uint32_t timestamp = lpn_msg.time_boot_ms / 1000.0;
            // parse out the local positin and trigger that callback
            LocalFrameMessage pos(timestamp, lpn_msg.x, lpn_msg.y, lpn_msg.z);
            notify_message_listeners(LOCAL_POSITION, &pos);

            // parse out the velocity and trigger that callback
            LocalFrameMessage vel(timestamp, lpn_msg.vx, lpn_msg.vy, lpn_msg.vz);
            notify_message_listeners(LOCAL_VELOCITY, &vel);
            break;
        }
        // http://mavlink.org/messages/common#HOME_POSITION
        case MAVLINK_MSG_ID_HOME_POSITION:
        {
            mavlink_home_position_t hp_msg;
            mavlink_msg_home_position_decode(&msg, &hp_msg);
            uint32_t timestamp = 0.0;
            GlobalFrameMessage home(timestamp, double(hp_msg.latitude) / 1e7, double(hp_msg.longitude) / 1e7, double(hp_msg.altitude) / 1000);
            notify_message_listeners(GLOBAL_HOME, &home);
            break;
        }
        // http://mavlink.org/messages/common/#SCALED_IMU
        case MAVLINK_MSG_ID_SCALED_IMU:
        {
            mavlink_scaled_imu_t si_msg;
            mavlink_msg_scaled_imu_decode(&msg, &si_msg);
            
            uint32_t timestamp = si_msg.time_boot_ms / 1000.0;
            // break out the message into its respective messages for here
            BodyFrameMessage accel(timestamp, si_msg.xacc / 1000.0, si_msg.yacc / 1000.0, si_msg.zacc / 1000.0);  // units -> [mg]
            notify_message_listeners(RAW_ACCELEROMETER, &accel);

            BodyFrameMessage gyro(timestamp, si_msg.xgyro / 1000.0, si_msg.ygyro / 1000.0, si_msg.zgyro / 1000.0);  // units -> [millirad/sec]
            notify_message_listeners(RAW_GYROSCOPE, &gyro);
            break;
        }
        // http://mavlink.org/messages/common#SCALED_PRESSURE
        case MAVLINK_MSG_ID_SCALED_PRESSURE:
        {
            mavlink_scaled_pressure_t sp_msg;
            mavlink_msg_scaled_pressure_decode(&msg, &sp_msg);
            
            uint32_t timestamp = sp_msg.time_boot_ms / 1000.0;
            BodyFrameMessage pressure(timestamp, 0, 0, sp_msg.press_abs);  // unit is [hectopascal]
            notify_message_listeners(BAROMETER, &pressure);
            break;
        }
        // http://mavlink.org/messages/common#DISTANCE_SENSOR
        case MAVLINK_MSG_ID_DISTANCE_SENSOR:
        {
            mavlink_distance_sensor_t ds_msg;
            mavlink_msg_distance_sensor_decode(&msg, &ds_msg);
            
            uint32_t timestamp = ds_msg.time_boot_ms / 1000.0;
            double direction = 0;
            // TODO: parse orientation
            // orientation = msg.orientation
            DistanceSensorMessage meas(timestamp,
                                            double(ds_msg.min_distance) / 100,
                                            double(ds_msg.max_distance) / 100, direction,
                                            double(ds_msg.current_distance) / 100,
                                            double(ds_msg.covariance) / 100);
            notify_message_listeners(DISTANCE_SENSOR, &meas);
            break;
        }
        // http://mavlink.org/messages/common#ATTITUDE_QUATERNION
        case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
        {
            mavlink_attitude_quaternion_t aq_msg;
            mavlink_msg_attitude_quaternion_decode(&msg, &aq_msg);
            
            uint32_t timestamp = aq_msg.time_boot_ms / 1000.0;
            // TODO: check if mask notifies us to ignore a field

            FrameMessage fm(timestamp, aq_msg.q1, aq_msg.q2, aq_msg.q3, aq_msg.q4);
            notify_message_listeners(ATTITUDE, &fm);

            BodyFrameMessage gyro(timestamp, aq_msg.rollspeed, aq_msg.pitchspeed, aq_msg.yawspeed);
            notify_message_listeners(RAW_GYROSCOPE, &gyro);
            break;
        }
        case MAVLINK_MSG_ID_GPS_RAW_INT:
        {
            mavlink_gps_raw_int_t gri_msg;
            mavlink_msg_gps_raw_int_decode(&msg, &gri_msg);
            uint32_t timestamp = gri_msg.time_usec / 1000.0;
            // parse out the gps position and trigger that callback
            GlobalFrameMessage gps(timestamp, gri_msg.lat  / (double)1E7, gri_msg.lon / (double)1E7, gri_msg.alt  / 1000.0);
            notify_message_listeners(GLOBAL_POSITION, &gps);
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_ACK:
            handleCommandAck(msg);
            break;
        // DEBUG
        case MAVLINK_MSG_ID_STATUSTEXT:
        {
            mavlink_statustext_t sm;
            mavlink_msg_statustext_decode(&msg, &sm);
            printf("%s\n", sm.text);
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE_TARGET:
            handleAttitudeTarget(msg);
            break;
        default:
        {
//            printf("Warning, did not handle message id %i\n", msg.msgid);
            break;
        }
    }
}

void MavlinkConnection::handleAttitudeTarget(mavlink_message_t& message)
{
    mavlink_attitude_target_t attitudeTarget;

    mavlink_msg_attitude_target_decode(&message, &attitudeTarget);

    float roll, pitch, yaw;
    mavlink_quaternion_to_euler(attitudeTarget.q, &roll, &pitch, &yaw);
//    printf("AttitudeTarget:%f, %f, %f\n", roll, pitch, yaw);
}

void MavlinkConnection::handleCommandAck(mavlink_message_t& message)
{
    bool showError = true;
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&message, &ack);

    if (ack.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES && ack.result != MAV_RESULT_ACCEPTED) {
        printf("Vehicle responded to MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES with error(%d). Setting no capabilities.\n", ack.result);
    }

    if (ack.command == MAV_CMD_REQUEST_PROTOCOL_VERSION) {
        if (ack.result == MAV_RESULT_ACCEPTED) {
            // The vehicle should be sending a PROTOCOL_VERSION message in a mavlink 2 packet. This may or may not make it through the pipe.
            // So we wait for it to come and timeout if it doesn't.
//            if (!_mavlinkProtocolRequestComplete) {
//                QTimer::singleShot(1000, this, &Vehicle::_protocolVersionTimeOut);
//            }
        } else {
            printf("Vehicle responded to MAV_CMD_REQUEST_PROTOCOL_VERSION with error(%d).", ack.result);
        }
    }

    if (ack.command == MAV_CMD_DO_SET_ROI_LOCATION) {
        if (ack.result == MAV_RESULT_ACCEPTED) {
            printf("MAV_CMD_DO_SET_ROI_LOCATION\r\n");
        }
    }

    if (ack.command == MAV_CMD_DO_SET_ROI_NONE) {
        if (ack.result == MAV_RESULT_ACCEPTED) {
            printf("MAV_CMD_DO_SET_ROI_NONE\r\n");
        }
    }

#if !defined(NO_ARDUPILOT_DIALECT)
    if (ack.command == MAV_CMD_FLASH_BOOTLOADER && ack.result == MAV_RESULT_ACCEPTED) {
        printf("Bootloader flash succeeded\n");
    }
#endif

    if (showError) {
        switch (ack.result) {
        case MAV_RESULT_TEMPORARILY_REJECTED:
            printf("%d command temporarily rejected\n", ack.command);
            break;
        case MAV_RESULT_DENIED:
            printf("%d command denied\n", ack.command);
            break;
        case MAV_RESULT_UNSUPPORTED:
            printf("%d command not supported\n", ack.command);
            break;
        case MAV_RESULT_FAILED:
            printf("%d command failed\n", ack.command);
            break;
        default:
            // Do nothing
            break;
        }
    }
}

void MavlinkConnection::command_loop()
{
    /*
    Main loop for sending commands.

    Loop that is run a separate thread to be able to send messages to the
    target drone.  Uses the message queue `_out_msg_queue` as the
    queue of messages to run.
    */

    /*
     default to sending a velocity command to (0,0,0)
     this needs to be sending commands at a rate of at lease 2Hz in order
     for PX4 to allow a switch into offboard control.
     */
    
    if ( not writing_status == false )
    {
        fprintf(stderr,"write thread already running\n");
        return;
    }
    
    mavlink_set_position_target_local_ned_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY & MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
    packet.target_system = _target_system;
    packet.target_component = autopilot_id;
    packet.coordinate_frame = MAV_FRAME_LOCAL_NED;
    packet.vx       = 0.0;
    packet.vy       = 0.0;
    packet.vz       = 0.0;
    packet.yaw_rate = 0.0;
    
    mavlink_message_t high_rate_command;
    memset(&high_rate_command, 0, sizeof(mavlink_message_t));
//    mavlink_msg_set_position_target_local_ned_encode_chan(_target_system, _target_component, _target_channel, &high_rate_command, &packet);
    mavlink_msg_set_position_target_local_ned_encode(_target_system, _target_component, &high_rate_command, &packet);
    send_message_immediately(high_rate_command);
    writing_status = true;
    
    time_t last_write_time = time(nullptr);
    while(_running)
    {
        // empty out the queue of pending messages
        // NOTE: Queue class is synchronized and is thread safe already!
        mavlink_message_t msg;
        memset(&msg, 0, sizeof(Message));
        
        msg_queue_mutex.lock();
        while (_out_msg_queue.size() > 0)
        {
            memcpy(&msg, &_out_msg_queue.front(), sizeof(mavlink_message_t));
            _out_msg_queue.pop();
            // either set this is as the high rate command
            // to repeatedly send or send it immediately
            if (msg.msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED || msg.msgid == MAVLINK_MSG_ID_SET_ATTITUDE_TARGET)
            {
                memcpy(&high_rate_command, &msg, sizeof(mavlink_message_t));
            }
            // either way, want to send this command immediately
            send_message_immediately(msg);
        }
        msg_queue_mutex.unlock();
        
        // rate limit the loop
        // though only do this after we have handled any new messages to send
        // this ensures messages get sent off immediately
        time_t current_time = time(nullptr);
        if ((current_time - last_write_time) < 1.0 / _send_rate)
        {
            continue;
        }
        last_write_time = current_time;

        // continually want to send the high rate command
        send_message_immediately(high_rate_command);
    }
    writing_status = false;
}

bool MavlinkConnection::wait_for_message(mavlink_message_t &msg)
{
    /*
    Wait for a new mavlink message calls mavlink's blocking read function to read
    a next message, blocking for up to a timeout of 1s.

    Returns:
        Mavlink message that was read or `None` if the message was invalid.
    */

    // NOTE: this returns a mavlink message
    // this function should not be called outside of this class!
    if (_master->recv_match(&msg, true, 1))
    {
        // send a heartbeat message back, since this needs to be
        // constantly sent so the autopilot knows this exists
        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
        {
            // send -> type, autopilot, base mode, custom mode, system status
            mavlink_heartbeat_t packet;
            memset(&packet, 0, sizeof(packet));
            packet.custom_mode = 0;
            packet.type = MAV_TYPE_GCS;
            packet.autopilot = MAV_AUTOPILOT_INVALID;
            packet.base_mode = 0;
            packet.system_status = MAV_STATE_ACTIVE;
            packet.mavlink_version = 3;
            
            mavlink_message_t outmsg;
//            mavlink_msg_heartbeat_encode_chan(_target_system, _target_component, _target_channel, &outmsg, &packet);
            mavlink_msg_heartbeat_encode(_target_system, _target_component, &outmsg, &packet);
            send_message(outmsg);
        }

        return true;
    }
    return false;
}

void MavlinkConnection::start()
{
    // start the main thread
    _running = true;

// start the dispatch loop, either threaded or not
    if (_threaded)
    {
        _read_handle = new thread(&MavlinkConnection::dispatch_loop, this);
        printf("CHECK FOR MESSAGES\n");
    
        while ( not _target_system )
        {
            if ( not _running )
                return;
            usleep(500000); // check at 2Hz
        }
    
        printf("Found\n");
        
        _read_handle_daemon = true;
//        _read_handle->join();
    }
    else
    {
        _read_handle = nullptr;
        dispatch_loop();
    }
    
    /* start the command loop
     this is only needed when working with PX4 and we need to enforce
     a certain rate of messages being sent
    */
    if (_using_px4)
    {
        _write_handle = new thread(&MavlinkConnection::command_loop, this);
        
        // wait for it to be started
        while ( not writing_status )
            usleep(100000); // 10Hz
        
        _write_handle_daemon = true;
//        _write_handle->join();
    }
}

void MavlinkConnection::stop()
{
    // stop the dispatch and command while loops
    _running = false;
    writing_status = false;
    sleep(2);

    // NOTE: no need to call join on the threads
    // as both threads are daemon threads

    printf("Closing connection ...\r\n");
    _master->close();
    if (_master) {
        delete _master;
        _master = nullptr;
    }
}

void MavlinkConnection::send_message(mavlink_message_t msg)
{
    /*
    Send a given mavlink message to the drone. If connected with a PX4 autopilot,
    add the MAVLinkMessage to the command queue to be handled by the command loop
    (running in the write thread).  Otherwise immediately send the message.

    :param msg: MAVLinkMessage to be sent to the drone
    */

    /* if we are using PX4, means we are also using out command loop
     in a separate thread and therefore need to send the data to that
     thread using the queue
    
     if we are not using PX4, then just immediately send the message
     */
    if (_using_px4)
    {
        msg_queue_mutex.lock();
        _out_msg_queue.push(msg);
        msg_queue_mutex.unlock();
    }
    else
    {
        send_message_immediately(msg);
    }
}

void MavlinkConnection::send_message_immediately(mavlink_message_t msg)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN] = {0};
    memset(buffer, 0, MAVLINK_MAX_PACKET_LEN);
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    _master->write(buffer, len);
}

void MavlinkConnection::send(const vector<uint8_t>& packet)
{
  _master->write(&packet[0], (int)packet.size());
}

void MavlinkConnection::send_long_command(uint16_t command_type, double param1, double param2, double param3, double param4, double param5, double param6, double param7)
{
    /*
    Packs and sends a Mavlink COMMAND_LONG message

    Args:
        command_type: the command type, as defined by MAV_CMD_*
        param1: param1 as defined by the specific command
        param2: param2 as defined by the specific command (default: {0})
        param3: param3 as defined by the specific command (default: {0})
        param4: param4 as defined by the specific command (default: {0})
        param5: param5 (x) as defined by the specific command (default: {0})
        param6: param6 (y) as defined by the specific command (default: {0})
        param7: param7 (z) as defined by the specific command (default: {0})
    */
    mavlink_command_long_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.param7 = param7;
    packet.command = command_type;
    packet.target_system = _target_system;
    packet.target_component = autopilot_id;
    packet.confirmation = true; //may want this as an input.... used for repeat messages
    
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(mavlink_message_t));
    
//    mavlink_msg_command_long_encode_chan(_target_system, _target_component, _target_channel, &msg, &packet);
    mavlink_msg_command_long_encode(_target_system, _target_component, &msg, &packet);
    send_message(msg);
}

void MavlinkConnection::arm()
{
    send_long_command(MAV_CMD_COMPONENT_ARM_DISARM, 1, 21196);
}

void MavlinkConnection::disarm()
{
    send_long_command(MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196);
}

void MavlinkConnection::take_control()
{
    send_long_command(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_MODE_OFFBOARD, 0);
}

void MavlinkConnection::release_control()
{
    send_long_command(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_MODE_MANUAL, 0);
}

void MavlinkConnection::cmd_attitude_target_send(time_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint16_t type_mask, double roll, double pitch, double yaw, double body_roll_rate, double body_pitch_rate, double body_yaw_rate, double thrust)
{
    mavlink_set_attitude_target_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.time_boot_ms = (uint32_t)time_boot_ms;
    packet.body_roll_rate = body_roll_rate;
    packet.body_pitch_rate = body_pitch_rate;
    packet.body_yaw_rate = body_yaw_rate;
    packet.thrust = thrust;
    packet.target_system = target_system;
    packet.target_component = autopilot_id;
    packet.type_mask = type_mask;
    
    // convert the attitude to a quaternion
    FrameMessage frame_msg(time_boot_ms, roll, pitch, yaw);
    memset(packet.q, 0, sizeof(double) * 4);
    packet.q[0] = frame_msg.q0();
    packet.q[1] = frame_msg.q1();
    packet.q[2] = frame_msg.q2();
    packet.q[3] = frame_msg.q3();
    
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(mavlink_message_t));
//    mavlink_msg_set_attitude_target_encode_chan(target_system, target_component, _target_channel, &msg, &packet);
    mavlink_msg_set_attitude_target_encode(target_system, target_component, &msg, &packet);
    send_message(msg);
}

void MavlinkConnection::cmd_attitude(double roll, double pitch, double yaw, double thrust)
{
    cmd_attitude_target_send(0, _target_system, _target_component, MASK_IGNORE_RATES, roll, pitch, yaw, 0, 0, 0, thrust);
}

void MavlinkConnection::cmd_attitude_rate(double roll_rate, double pitch_rate, double yaw_rate, double thrust)
{
    cmd_attitude_target_send(0, _target_system, _target_component, MASK_IGNORE_ATTITUDE, 0, 0, 0, roll_rate, pitch_rate, yaw_rate, thrust);
}

void MavlinkConnection::cmd_moment(double roll_moment, double pitch_moment, double yaw_moment, double thrust, time_t t)
{
    cmd_attitude_target_send(t * 1000, _target_system, _target_component, 0b10000000, 0, 0, 0, roll_moment, pitch_moment, yaw_moment, thrust);
}

void MavlinkConnection::cmd_position_target_local_ned_send(time_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, double x, double y, double z, double vx, double vy, double vz, double afx, double afy, double afz, double yaw, double yaw_rate)
{
    mavlink_set_position_target_local_ned_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.time_boot_ms = (uint32_t)time_boot_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.afx = afx;
    packet.afy = afy;
    packet.afz = afz;
    packet.yaw = yaw;
    packet.yaw_rate = yaw_rate;
    packet.type_mask = type_mask;
    packet.target_system = target_system;
    packet.target_component = autopilot_id;
    packet.coordinate_frame = coordinate_frame;

    mavlink_message_t msg;
    memset(&msg, 0, sizeof(mavlink_message_t));
//    mavlink_msg_set_position_target_local_ned_encode_chan(target_system, target_component, _target_channel, &msg, &packet);
    mavlink_msg_set_position_target_local_ned_encode(target_system, target_component, &msg, &packet);

    send_message(msg);
}

void MavlinkConnection::cmd_velocity(double vn, double ve, double vd, double heading)
{
    cmd_position_target_local_ned_send(0, _target_system, _target_component, MAV_FRAME_LOCAL_NED, MASK_IGNORE_YAW_RATE | MASK_IGNORE_ACCELERATION | MASK_IGNORE_POSITION, 0, 0, 0, vn, ve, vd, 0, 0, 0, heading, 0);
}

void MavlinkConnection::cmd_position(double n, double e, double d, double heading)
{
    // when using the simualtor, d is actually interpreted as altitude
    // therefore need to do a sign change on d
    if (_using_px4)
    {
        d = -1.0 * d;
    }
    cmd_position_target_local_ned_send(0, _target_system, _target_component, MAV_FRAME_LOCAL_NED, MASK_IGNORE_YAW_RATE | MASK_IGNORE_ACCELERATION | MASK_IGNORE_VELOCITY, n, e, d, 0, 0, 0, 0, 0, 0, heading, 0);
}
    
void MavlinkConnection::cmd_controls(float *controls, time_t t)
{
    mavlink_set_actuator_control_target_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.time_usec = (t * 1000000);
    packet.group_mlx = 1;
    packet.target_system = _target_system;
    packet.target_component = autopilot_id;
    
    // ensure controls_out is of length 8 even if controls isn't
    memset(packet.controls, 0, sizeof(float) * 8);
    mav_array_memcpy(packet.controls, controls, sizeof(float)*8);
    
    mavlink_message_t msg;
//    mavlink_msg_set_actuator_control_target_encode_chan(_target_system, _target_component, _target_channel, &msg, &packet);
    mavlink_msg_set_actuator_control_target_encode(_target_system, _target_component, &msg, &packet);

    send_message(msg);
}

void MavlinkConnection::takeoff(double n, double e, double d)
{
    /*
     for mavlink to PX4 need to specify the NED location for landing
     since connection doesn't keep track of this info, have drone send it
     abstract away that part in the drone class
     */
    cmd_position_target_local_ned_send(0, _target_system, _target_component, MAV_FRAME_LOCAL_NED, MASK_IS_LAND | MASK_IGNORE_YAW_RATE | MASK_IGNORE_YAW | MASK_IGNORE_ACCELERATION | MASK_IGNORE_VELOCITY, n, e, d, 0, 0, 0, 0, 0, 0, 0, 0);
//    send_long_command(MAV_CMD_NAV_TAKEOFF, -1, 0, 0, NAN, NAN, NAN, d);
}

void MavlinkConnection::land(double n, double e)
{
    // for mavlink to PX4 need to specify the NED location for landing
    // since connection doesn't keep track of this info, have drone send it
    // abstract away that part in the drone class
    cmd_position_target_local_ned_send(0, _target_system, _target_component, MAV_FRAME_LOCAL_NED, MASK_IS_TAKEOFF | MASK_IGNORE_YAW_RATE | MASK_IGNORE_YAW | MASK_IGNORE_ACCELERATION | MASK_IGNORE_VELOCITY, n, e, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

void MavlinkConnection::set_home_position(double lat, double lon, double alt)
{
    send_long_command(MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, lat, lon, alt);
}

void MavlinkConnection::local_position_target(double n, double e, double d, time_t t)
{
    cmd_position_target_local_ned_send(t, _target_system, _target_component, MAV_FRAME_LOCAL_NED, 0b1111111111111000, n, e, d, 0, 0, 0, 0, 0, 0, 0, 0);
}

void MavlinkConnection::local_velocity_target(double vn, double ve, double vd, time_t t)
{
    cmd_position_target_local_ned_send(t, _target_system, _target_component, MAV_FRAME_LOCAL_NED, 0b1111111111000111, 0, 0, 0, vn, ve, vd, 0, 0, 0, 0, 0);
}

void MavlinkConnection::local_acceleration_target(double an, double ae, double ad, time_t t)
{
    cmd_position_target_local_ned_send(t, _target_system, _target_component, MAV_FRAME_LOCAL_NED, 0b1111111000111111, 0, 0, 0, 0, 0, 0, an, ae, ad, 0, 0);
}

void MavlinkConnection::attitude_target(double roll, double pitch, double yaw, time_t t)
{
    cmd_attitude_target_send(t, _target_system, _target_component, 0b01111111, roll, pitch, yaw, 0, 0, 0, 0);
}

void MavlinkConnection::body_rate_target(double p, double q, double r, time_t t)
{
    cmd_attitude_target_send(t, _target_system, _target_component, 0b11111000, 0, 0, 0, p, q, r, 0);
}

void MavlinkConnection::set_sub_mode(int sub_mode)
{
    send_long_command(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_MODE_OFFBOARD, sub_mode);
}

void MavlinkConnection::set_notify_callback(notify_message_callback fn)
{
    _notify_message_callback = fn;
}

void MavlinkConnection::notify_message_listeners(message_ids name, void *msg)
{
    (_drone->*_notify_message_callback)(name, msg);
}

// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
void MavlinkConnection::cmd_offboard_control(bool flag)
{
    // Prepare command for off-board mode
    mavlink_command_long_t com = { 0 };
    com.target_system    = _target_system;
    com.target_component = autopilot_id;
    com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
    com.confirmation     = true;
    com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop

    // Encode
    mavlink_message_t message;
//    mavlink_msg_command_long_encode_chan(_target_system, _target_component, _target_channel, &message, &com);
    mavlink_msg_command_long_encode(_target_system, _target_component, &message, &com);

    // Send the message
    send_message(message);
}
