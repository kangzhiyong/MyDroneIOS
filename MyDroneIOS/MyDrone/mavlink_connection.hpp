//
//  mavlink_connection.hpp
//  MyDrone
//
//  Created by kangzhiyong on 2020/2/26.
//

#ifndef mavlink_connection_hpp
#define mavlink_connection_hpp

#include <ctime>
#include <queue>
#include <thread>
#include <mutex>
#include <string>
#include <vector>
using namespace std;

#include "mavlink_udp.hpp"
#include "message_ids.h"
#include "message_types.hpp"

class MyDrone;
/*
Implementation of the required communication to a drone executed
over the Mavlink protocol. Specifically designed with the PX4 autopilot in mind,
and currently been tested against that autopilot software.

Example:

    # TCP connection, protocol:ip:port
    conn = MavlinkConnection('tcp:127.0.0.1:5760')

    # Serial connection, port:baud
    conn = MavlinkConnection('5760:921600')
 */
class MavlinkConnection
{
public:
    typedef void (MyDrone::*notify_message_callback)(message_ids, void *);
    MavlinkConnection(std::string destAddr, bool threaded = false, bool PX4 = false, double send_rate = 5, time_t timeout = 5);
    
    void notify_message_listeners(message_ids name, void *msg);
    
    /*
     parse out the message based on the type and call
     the appropriate callbacks
     */
    void dispatch_message(mavlink_message_t *msg);
    
    bool open();
    void dispatch_loop();
    void dispatch_message(mavlink_message_t msg);
    void command_loop();
    bool wait_for_message(mavlink_message_t &msg);
    void start();
    void stop();
    void send_message(mavlink_message_t msg);
    void send_message_immediately(mavlink_message_t msg);
    void send(const vector<uint8_t>& packet);
    void send_long_command(uint16_t command_type, double param1, double param2=0, double param3=0, double param4=0, double param5=0, double param6=0, double param7=0);
    void arm();
    void disarm();
    void take_control();
    void release_control();
    void cmd_attitude_target_send(time_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint16_t type_mask, double roll, double pitch, double yaw, double body_roll_rate, double body_pitch_rate, double body_yaw_rate, double thrust);
    void cmd_attitude(double roll, double pitch, double yaw, double thrust);
    void cmd_attitude_rate(double roll_rate, double pitch_rate, double yaw_rate, double thrust);
    void cmd_moment(double roll_moment, double pitch_moment, double yaw_moment, double thrust, time_t t=0.0);
    void cmd_position_target_local_ned_send(time_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, double x, double y, double z, double vx, double vy, double vz, double afx, double afy, double afz, double yaw, double yaw_rate);
    void cmd_velocity(double vn, double ve, double vd, double heading);
    void cmd_position(double n, double e, double d, double heading);
    void cmd_controls(double *controls, time_t t=0);
    void takeoff(double n, double e, double d);
    void land(double n, double e);
    void set_home_position(double lat, double lon, double alt);
    void local_position_target(double n, double e, double d, time_t t=0);
    void local_velocity_target(double vn, double ve, double vd, time_t t=0);
    void local_acceleration_target(double an, double ae, double ad, time_t t=0);
    void attitude_target(double roll, double pitch, double yaw, time_t t=0);
    void body_rate_target(double p, double q, double r, time_t t=0);
    void set_sub_mode(int sub_mode);
    void set_notify_callback(notify_message_callback);
    void set_drone(MyDrone *drone)
    {
        _drone = drone;
    }
private:
    mavudp *_master;
    queue<mavlink_message_t> _out_msg_queue;
    thread *_read_handle;
    bool _read_handle_daemon{false};
    thread *_write_handle;
    bool _write_handle_daemon{false};
    // management
    bool _running{false};
    uint8_t _target_system{1};
    uint8_t _target_component{1};
    
    // PX4 management
    bool _using_px4{false};
    double _send_rate{5};

    // seconds to wait of no messages before termination
    time_t _timeout{5};
    mutex msg_queue_mutex;
    bool _threaded{false};
    MyDrone *_drone;
    notify_message_callback _notify_message_callback;
};
#endif /* mavlink_connection_hpp */
