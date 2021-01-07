//
//  MyDrone.hpp
//  MyDrone
//
//  Created by kangzhiyong on 2020/2/25.
//

#ifndef MyDrone_hpp
#define MyDrone_hpp

#include <map>
#include <vector>
using namespace std;

#include "mavlink_connection.hpp"
#include "my_point.hpp"
class MyDrone
{
private:
    time_t _message_time{0};
    double _message_frequency{0.0};
    time_t _time_bias{0};
    // Global position in degrees (int)
    // Altitude is in meters
    double _longitude{0.0};
    double _latitude{0.0};
    double _altitude{0.0};
    time_t _global_position_time{0};
    double _global_position_frequency{0.0};

    // Reference home position in degrees (int)
    // Altitude is in meters
    double _home_longitude{0.0};
    double _home_latitude{0.0};
    double _home_altitude{0.0};
    time_t _home_position_time{0};
    double _home_position_frequency{0.0};

    // Local positions in meters from the global home (double)
    // In NED frame
    double _north{0.0};
    double _east{0.0};
    double _down{0.0};
    time_t _local_position_time{0};
    double _local_position_frequency{0.0};

    // Locally oriented velocity in meters/second
    // In NED frame
    double _velocity_north{0.0};
    double _velocity_east{0.0};
    double _velocity_down{0.0};
    time_t _local_velocity_time{0};
    double _local_velocity_frequency{0.0};

    // If the drone is armed the motors are powered and the rotors are spinning.
    bool _armed{false};

    // If the drone is guided it is being autonomously controlled,
    // the other opposite would be manual control.
    bool _guided{false};
    
    // An integer to pass along random status changes specific for different vehicles
    int _status{0};
    time_t _state_time{0};
    double _state_frequency{0.0};

    // Euler angles in radians
    double _roll{0.0};
    double _pitch{0.0};
    double _yaw{0.0};
    time_t _attitude_time{0};
    double _attitude_frequency{0.0};

    // Drone body accelerations
    double _acceleration_x{0.0};
    double _acceleration_y{0.0};
    double _acceleration_z{0.0};
    time_t _acceleration_time{0};
    double _acceleration_frequency{0.0};

    // Drone gyro rates or angular velocities in radians/second
    double _gyro_x{0.0};
    double _gyro_y{0.0};
    double _gyro_z{0.0};
    time_t  _gyro_time{0};
    double _gyro_frequency{0.0};

    // Barometer
    double _baro_altitude{0.0};
    double _baro_time{0.0};
    double _baro_frequency{0.0};

    typedef void (MyDrone::*update)(void *data);
    typedef void (MyDrone::*user_callback)();
    typedef map<message_ids, update> update_property_t;
    typedef map<message_ids,  vector<user_callback>> user_callback_t;
    
    update_property_t _update_property;
    user_callback_t _callbacks;
    MavlinkConnection *m_conn;
public:
    MyDrone(MavlinkConnection *conn);
    void on_message_receive(message_ids msg_name, void *msg);
    point3D global_position();
    time_t global_position_time();
    void _update_global_position(void *msg);
    point3D global_home();
    time_t home_position_time();
    void _update_global_home(void *msg);
    point3D local_position();
    time_t local_position_time();
    void _update_local_position(void *msg);
    void _update_local_position(point3D p);
    point3D local_velocity();
    time_t local_velocity_time();
    void _update_local_velocity(void *msg);
    bool armed();
    bool guided();
    bool connected();
    time_t state_time();
    int status();
    void _update_state(void *msg);
    
    // Roll, pitch, yaw euler angles in radians
    point3D attitude();
    time_t attitude_time();
    void _update_attitude(void *msg);
    point3D acceleration_raw();
    time_t acceleration_time();
    void _update_acceleration_raw(void *msg);
    
    // Angular velocites in radians/second
    point3D gyro_raw();
    time_t gyro_time();
    void _update_gyro_raw(void *msg);
    double barometer();
    time_t barometer_time();
    void _update_barometer(void *msg);

    // Handling of internal messages for callbacks
    void register_callback(message_ids name, user_callback fn);
    void remove_callback(message_ids name, user_callback fn);
    void notify_callbacks(message_ids name);
    
    // Command method wrappers
    void arm();
    void disarm();
    void take_control();
    void release_control();
    void cmd_position( double north, double east, double altitude, double heading);
    void takeoff(double target_altitude);
    void land();
    void cmd_attitude( double roll, double pitch, double yaw, double thrust);
    void cmd_attitude_rate( double roll_rate, double pitch_rate, double yaw_rate, double thrust);
    void cmd_moment( double roll_moment, double pitch_moment, double yaw_moment, double thrust);
    void cmd_velocity( double velocity_north, double velocity_east, double velocity_down, double heading);
    void set_home_position( double longitude, double latitude, double altitude);
    void set_home_as_current_position();
    void start();
    void stop();
    MavlinkConnection * getConnection()
    {
        return m_conn;
    }
    void cmd_offboard_control(bool flag);
};
#endif /* MyDrone_hpp */
