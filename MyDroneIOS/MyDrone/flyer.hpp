//
//  flyer.hpp
//  MyDrone
//
//  Created by kangzhiyong on 2020/2/29.
//  Copyright © 2020 kangzhiyong. All rights reserved.
//

#ifndef flyer_hpp
#define flyer_hpp

#include <queue>
#include <vector>
using namespace std;

#include "my_drone.hpp"
#include "mavlink_connection.hpp"
#include "StringUtils.h"
#include "MissionPlanning/my_point.hpp"
#include "my_sampler.hpp"
#include "graph.hxx"

typedef std::vector<point3D> VPoint3Ds;
typedef std::pair<int, size_t> PVertexType;
typedef std::vector<size_t> VVertexType;
typedef void (*CallBackFunc)(void *userData, double lo, double la, double alt);

enum States
{
    MANUAL = 0,
    ARMING,
    TAKEOFF,
    WAYPOINT,
    LANDING,
    DISARMING,
    PLANNING
};

class Flyer: public MyDrone
{
private:
    queue<point4D> all_waypoints;
    point4D target_position{0, 0, 0, 0};
    bool in_mission{true};
    vector<States> check_state;
    States flight_state{MANUAL};
    MySampler *m_mSampler;
    VPoint3Ds m_vNodes;
    andres::graph::Graph<> m_Graph;
    vector<point3D> m_vGraphVertices;
    int m_iFlyType;
    CallBackFunc m_pCallBackFunc;
    void *m_pViewController;
    string m_sAppPath;
    bool m_bControlStatus{false};
public:
    Flyer(MavlinkConnection *conn);
    void local_position_callback();
    void global_position_callback();
    void velocity_callback();
    void state_callback();
    void calculate_box();
    void arming_transition();
    void takeoff_transition();
    void waypoint_transition();
    void landing_transition();
    void disarming_transition();
    void manual_transition();
    void start(int t);
    void stop();
    void plan_path();
    void create_graph(int k);
    bool can_connect(point3D p1, point3D p2);
    bool bresenham_check(point3D p1, point3D p2);
    VVertexType prune_path(VVertexType path);
    size_t find_closed_vertice(point3D p);
    VVertexType a_star_for_graph(andres::graph::Graph<> graph, size_t start, size_t goal);
    void setFlyType(int t)
    {
        m_iFlyType = t;
    }
    int getFlyType()
    {
        return m_iFlyType;
    }
    /**
    * @brief Project a geodetic coordinate on to local tangential plane (LTP) as coordinate with East,
    * North, and Down components in meters.
    * @param[in] coord Geodetic coordinate to project onto LTP.
    * @param[in] origin Geoedetic origin for LTP projection.
    */
    point3D global_to_local(point3D coord, point3D origin);
    
    /**
    * @brief Transform a local (East, North, and Down) coordinate into a geodetic coordinate.
    * @param[in] local coordinate in meters.
    * @param[in] origin Geoedetic origin for LTP.
    */
    point3D local_to_global(point3D local, point3D origin);
    queue<point4D> getAllWayPoint();
    void getCurrLocation(double &lo, double &la);
    void registerCallBackFunction(void *, CallBackFunc func);
    void handleInformation(double lo, double la, double alt);
    void setAppPath(const char *path);
};

#endif /* flyer_hpp */
