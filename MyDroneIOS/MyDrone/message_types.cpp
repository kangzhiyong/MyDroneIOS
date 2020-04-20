//
//  message_types.cpp
//  MyDrone
//
//  Created by kangzhiyong on 2020/2/25.
//

#include "message_types.hpp"

MessageBase::MessageBase(int64_t t)
{
    _time = t;
}
int64_t MessageBase::getTime()
{
    return _time;
}

StateMessage::StateMessage(int64_t time, bool armed, bool guided, int status):MessageBase(time)
{
    _armed = armed;
    _guided = guided;
    _status = status;
}

bool StateMessage::armed()
{
    //bool: true if the drone is armed and ready to fly
    return _armed;
}

bool StateMessage::guided()
{
    //bool: true if the drone can be commanded from python
    return _guided;
}
int StateMessage::status()
{
    //int: status value from the autopilot not corresponding to anything in particular
    return _status;
}

GlobalFrameMessage::GlobalFrameMessage( int64_t time, double latitude, double longitude, double altitude):MessageBase(time)
{
    _longitude = longitude;
    _latitude = latitude;
    _altitude = altitude;
}

double GlobalFrameMessage::longitude()
{
    //double: longitude in degrees
    return _longitude;
}

double GlobalFrameMessage::latitude()
{
    //double: latitude in degrees
    return _latitude;
}

double GlobalFrameMessage::altitude()
{
    //double: altitude in meters above sea level
    return _altitude;
}

vector<double> GlobalFrameMessage::global_vector()
{
    vector<double> gv;
    gv.push_back(_longitude);
    gv.push_back(_latitude);
    gv.push_back(_altitude);
    return gv;
}

LocalFrameMessage::LocalFrameMessage( int64_t time, double north, double east, double down):MessageBase(time)
{
    _north = north;
    _east = east;
    _down = down;
}

double LocalFrameMessage::north()
{
    //double: north position in meters
    return _north;
}

double LocalFrameMessage::east()
{
    //double: east position in meters
    return _east;
}

double LocalFrameMessage::down()
{
    //double: down position in meters
    return _down;
}

vector<double> LocalFrameMessage::local_vector()
{
    vector<double> lv;
    lv.push_back(_north);
    lv.push_back(_east);
    lv.push_back(_down);
    return lv;
}

BodyFrameMessage::BodyFrameMessage(int64_t time, double x, double y, double z):MessageBase(time)
{
    _x = x;
    _y = y;
    _z = z;
}

double BodyFrameMessage::x()
{
    //double: x value
    return _x;
}

double BodyFrameMessage::y()
{
    //double: y value
    return _y;
}

double BodyFrameMessage::z()
{
    //double: z value
    return _z;
}

vector<double> BodyFrameMessage::body_vector()
{
    vector<double> bv;
    bv.push_back(_x);
    bv.push_back(_y);
    bv.push_back(_z);
    return bv;
}

FrameMessage::FrameMessage(int64_t time, double roll, double pitch, double yaw): MessageBase(time)
{
    _roll = roll;
    _pitch = pitch;
    _yaw = yaw;

    double sp = sin(pitch / 2.0);
    double cp = cos(pitch / 2.0);
    double sr = sin(roll / 2.0);
    double cr = cos(roll / 2.0);
    double sy = sin(yaw / 2.0);
    double cy = cos(yaw / 2.0);

    _q0 = cr * cp * cy + sr * sp * sy;
    _q1 = sr * cp * cy - cr * sp * sy;
    _q2 = cr * sp * cy + sr * cp * sy;
    _q3 = cr * cp * sy - sr * sp * cy;
}

FrameMessage::FrameMessage(int64_t time, double q0, double q1, double q2, double q3): MessageBase(time)
{
    
    _q0 = q0;
    _q1 = q1;
    _q2 = q2;
    _q3 = q3;

    _roll = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (pow(q1, 2) + pow(q2, 2)));
    _pitch = asin(2.0 * (q0 * q2 - q3 * q1));
    _yaw = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (pow(q2, 2) + pow(q3, 2)));
}

double FrameMessage::roll()
{
    //roll in radians
    return _roll;
}

double FrameMessage::pitch()
{
    //pitch in radians
    return _pitch;
}

double FrameMessage::yaw()
{
    //yaw in radians
    return _yaw;
}

double FrameMessage::q0()
{
    //double: 0th element of quaternion
    return _q0;
}

double FrameMessage::q1()
{
    //double: 1st element of quaternion
    return _q1;
}

double FrameMessage::q2()
{
    //double: 2nd element of quaternion
    return _q2;
}

double FrameMessage::q3()
{
    //double: 3rd element of quaternion
    return _q3;
}

vector<double> FrameMessage::euler_angles()
{
    vector<double> eav;
    eav.push_back(_roll);
    eav.push_back(_pitch);
    eav.push_back(_yaw);
    return eav;
}

vector<double> FrameMessage::quaternions()
{
    vector<double> qv;
    qv.push_back(_q0);
    qv.push_back(_q1);
    qv.push_back(_q2);
    qv.push_back(_q3);
    return qv;
}

DistanceSensorMessage::DistanceSensorMessage(int64_t time, double min_distance, double max_distance, double direction, double measurement, double covariance): MessageBase(time)
{
    measuremen.push_back(direction);
    measuremen.push_back(measurement);
    measuremen.push_back(covariance);
    properties.push_back(min_distance);
    properties.push_back(max_distance);
}
