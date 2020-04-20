/*
Message Types
custom set of message types to use between a specific connection type and
the drone class.
this enables abstracting away the protocol specific messages so different
protocols can be used with the same student facing interface and code.
NOTE: besides the state message, all messages are in the frame they are
defined in.
NOTE: to ensure minimal errors due to typos, use the MSG_* constants when
registering for specific messages
Attributes:
    MSG_ALL: flag to be used to register a listener for all messages
    MSG_STATE: name of the state message
    MSG_GLOBAL_POSITION: name of the global position message [StateMessage]
    MSG_LOCAL_POSITION: name of the local position message [LocalFrameMessage]
    MSG_GLOBAL_HOME: name of the global home message [GlobalFrameMessage]
    MSG_VELOCITY: name of the velocity message [LocalFrameMessage]
    MSG_CONNECTION_CLOSED: name of the message sent when the connection is closed (no data)
    MSG_RAW_GYROSCOPE: name of the raw gyro message [BodyFrameMessage]
    MSG_RAW_ACCELEROMETER: name of the raw acceleromater message [BodyFrameMessage]
    MSG_BAROMETER: name of the barometer message [LocalFrameMessage - only down populated]
    MSG_ATTITUDE: name of attitude message [FrameMessage]
*/

#ifndef message_types_hpp
#define message_types_hpp

#include <cmath>
#include <vector>
using namespace std;

class MessageBase
{
    //Message super class
public:
    MessageBase(){_time = 0;}
    MessageBase(int64_t t);
    int64_t getTime();
private:
    int64_t _time;
};

class StateMessage: public MessageBase
{
    /*
     State information message
    message to carry drone state information
    Attributes:
        _armed: whether or not drone is armed
        _guided: whether or not drone can be commanded from script
    */

public:
    StateMessage(){};
    StateMessage(int64_t time, bool armed, bool guided, int status = 0);
    bool armed();
    bool guided();
    int status();
private:
    bool _armed{false};
    bool _guided{false};
    int _status{0};
};


class GlobalFrameMessage: public MessageBase
{
    /*
     Global frame message
    message to carry information in a global frame
    Attributes:
        _latitude: latitude in degrees
        _longitude: longitude in degrees
        _altitude: altitude in meters above mean sea level (AMSL)
    */
public:
    GlobalFrameMessage( int64_t time, double latitude, double longitude, double altitude);
    double longitude();
    double latitude();
    double altitude();
    vector<double> global_vector();
private:
    double _longitude;
    double _latitude;
    double _altitude;
};

class LocalFrameMessage: public MessageBase
{
    /*
     Local frame message
    message to carry information in a local (NED) frame
    Attributes:
        _north: north position in meters
        _east: east position in meters
        _down: down position in meters (above takeoff point, positive down)
    */
public:
    LocalFrameMessage( int64_t time, double north, double east, double down);
    double north();
    double east();
    double down();
    vector<double> local_vector();
private:
    double _north;
    double _east;
    double _down;
};

class BodyFrameMessage: public MessageBase
{
    /*
     Body frame message
    message to carry information in a body frame
    Attributes:
        _x: x value
        _y: y value
        _z: z value
    */
public:
    BodyFrameMessage(int64_t time, double x, double y, double z);
    double x();
    double y();
    double z();
    vector<double> body_vector();
private:
    double _x;
    double _y;
    double _z;
};

class FrameMessage: public MessageBase
{
    /*
        Message representating frame information
        Messages defining the rotation between frames (Euler angles or Quaternions)
        Attributes:
            _roll: drone roll in radians
            _pitch: drone pitch in radians
            _yaw: drone yaw in radians
            _q0: 0th element of quaterion
            _q1: 1th element of quaterion
            _q2: 2th element of quaterion
            _q3: 3th element of quaterion
    */
public:
    FrameMessage(int64_t time, double roll, double pitch, double yaw);
    FrameMessage(int64_t time, double q0, double q1, double q2, double q3);
    double roll();
    double pitch();
    double yaw();
    double q0();
    double q1();
    double q2();
    double q3();
    vector<double> euler_angles();
    vector<double> quaternions();
    
private:
    double _roll;
    double _pitch;
    double _yaw;
    double _q0;
    double _q1;
    double _q2;
    double _q3;
};

class DistanceSensorMessage: public MessageBase
{
    /*
    Message for distance sensor (e.g. Lidar) information
    the properties of and measurement from a given distance sensor onboard
    the drone.
    Attributes:
        _min_distance: minimum detectable distance in meters
        _max_distance: maximum detectable distance in meters
        _direction: the heading of the sensor for this measurement in radians
        _measurement: the distance measured in meters
        _covariance: the covariance of the measurement
    */
public:
    DistanceSensorMessage(int64_t time, double min_distance, double max_distance, double direction, double measurement, double covariance);
private:
    vector<double> measuremen;
    vector<double> properties;
};

#endif /* message_types_hpp */
