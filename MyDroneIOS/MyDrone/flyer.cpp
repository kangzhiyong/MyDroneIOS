//
//  flyer.cpp
//  MyDrone
//
//  Created by kangzhiyong on 2020/2/29.
//  Copyright © 2020 kangzhiyong. All rights reserved.
//

#include "flyer.hpp"
#include "my_data.hpp"
#include "planning_utils.hpp"

#include <iostream>
#include <cmath>
using namespace std;

#define M_DEG_TO_RAD (M_PI / 180.0)
#define M_RAD_TO_DEG (180.0 / M_PI)
#define CONSTANTS_RADIUS_OF_EARTH 6371000 // meters (m)

static const double epsilon = std::numeric_limits<double>::epsilon();

point3D Flyer::global_to_local(point3D coord, point3D origin)
{
    if (coord == origin) {
        // Short circuit to prevent NaNs in calculation
        return {0, 0, 0};
    }

    double lat_rad = coord[1] * M_DEG_TO_RAD;
    double lon_rad = coord[0] * M_DEG_TO_RAD;

    double ref_lon_rad = origin[0] * M_DEG_TO_RAD;
    double ref_lat_rad = origin[1] * M_DEG_TO_RAD;

    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double cos_d_lon = cos(lon_rad - ref_lon_rad);

    double ref_sin_lat = sin(ref_lat_rad);
    double ref_cos_lat = cos(ref_lat_rad);

    double c = acos(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon);
    double k = (fabs(c) < epsilon) ? 1.0 : (c / sin(c));

    double x = k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
    double y = k * cos_lat * sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH;
    double z = -(coord[2] - origin[2]);
    return {x, y, z};
}

point3D Flyer::local_to_global(point3D local, point3D origin) {
    double x_rad = local[0] / CONSTANTS_RADIUS_OF_EARTH;
    double y_rad = local[1] / CONSTANTS_RADIUS_OF_EARTH;
    double c = sqrt(x_rad * x_rad + y_rad * y_rad);
    double sin_c = sin(c);
    double cos_c = cos(c);

    double ref_lon_rad = origin[0] * M_DEG_TO_RAD;
    double ref_lat_rad = origin[1] * M_DEG_TO_RAD;

    double ref_sin_lat = sin(ref_lat_rad);
    double ref_cos_lat = cos(ref_lat_rad);

    double lat_rad;
    double lon_rad;

    if (fabs(c) > epsilon) {
        lat_rad = asin(cos_c * ref_sin_lat + (x_rad * sin_c * ref_cos_lat) / c);
        lon_rad = (ref_lon_rad + atan2(y_rad * sin_c, c * ref_cos_lat * cos_c - x_rad * ref_sin_lat * sin_c));

    } else {
        lat_rad = ref_lat_rad;
        lon_rad = ref_lon_rad;
    }

    return {lon_rad * (double)M_RAD_TO_DEG, lat_rad * (double)M_RAD_TO_DEG, -local[2] + origin[2]};
}

double heuristic(point3D position, point3D goal_position)
{
    return norm(position[0], position[1], position[2],
                     goal_position[0], goal_position[1], goal_position[2]);
}

Flyer::Flyer(MavlinkConnection *conn): MyDrone(conn)
{
    check_state.clear();
    m_mSampler = nullptr;
    
    // register all your callbacks here
    register_callback(LOCAL_POSITION, ((void (MyDrone::*)())&Flyer::local_position_callback));
    register_callback(LOCAL_VELOCITY, ((void (MyDrone::*)())&Flyer::velocity_callback));
    register_callback(STATE, ((void (MyDrone::*)())&Flyer::state_callback));
    register_callback(GLOBAL_POSITION, ((void (MyDrone::*)())&Flyer::global_position_callback));
}

void Flyer::global_position_callback()
{
    point3D p = global_position();
    handleInformation(p[0], p[1], p[2]);
}

void Flyer::local_position_callback()
{
    point3D p = local_to_global(local_position(), global_home());
    handleInformation(p[0], p[1], p[2]);
    if (flight_state == TAKEOFF)
    {
        if (-1.0 * local_position()[2] > 0.95 * target_position[2])
        {
            waypoint_transition();
        }
    }
    else if (flight_state == WAYPOINT)
    {
        if (norm(target_position[0], target_position[1], target_position[2],
                      local_position()[0], local_position()[1], local_position()[2]) < 1.0)
        {
            if (all_waypoints.size() > 0)
            {
                waypoint_transition();
            }
            else
            {
                if (norm(local_velocity()[0], local_velocity()[1], local_velocity()[2], 0, 0, 0) < 1.0)
                {
                    landing_transition();
                }
            }
        }
    }
}

void Flyer::velocity_callback()
{
    if (flight_state == LANDING)
    {
        if (global_position()[2] - global_home()[2] < 0.1)
        {
            if (abs(local_position()[2]) < 0.01)
            {
                disarming_transition();
            }
        }
    }
}

void Flyer::state_callback()
{
    if (in_mission)
    {
        if (flight_state == MANUAL)
        {
            arming_transition();
        }
        else if (flight_state == ARMING)
        {
            if (armed())
            {
                switch (m_iFlyType) {
                    case 0:
                        calculate_box();
                        break;
                    case 1:
                        plan_path();
                        break;
                    default:
                        break;
                }
            }
        }
        else if (flight_state == PLANNING)
        {
            takeoff_transition();
        }
        else if (flight_state == DISARMING)
        {
            if (!armed() && !guided())
            {
                manual_transition();
            }
        }
    }
}

void Flyer::calculate_box()
{
    flight_state = PLANNING;
    cout << "calculate_box\r\n" << endl;
    point4D cp({local_position()[0], local_position()[1], -local_position()[2], 0});
    point4D p1({2.0, 0.0, 2.0, 0}), p2({2.0, 2.0, 2.0, 0}), p3({0.0, 2.0, 2.0, 0}), p4({0.0, 0.0, 2.0, 0});
    all_waypoints.push(cp + p1);
    all_waypoints.push(cp + p2);
    all_waypoints.push(cp + p3);
    all_waypoints.push(cp + p4);
}

void Flyer::arming_transition()
{
    cout << "arming transition\r\n" << endl;
    flight_state = ARMING;
    if (!m_bControlStatus) {
        printf("Enable offboard mode\n");
        cmd_offboard_control(true);
        m_bControlStatus = true;
    }
    arm();
//    take_control();
}

void Flyer::takeoff_transition()
{
    cout << "takeoff transition\r\n" << endl;
    flight_state = TAKEOFF;
    takeoff(target_position[2]);
}

void Flyer::waypoint_transition()
{
    if (all_waypoints.size() == 0) {
        return;
    }
    cout << "waypoint transition" << endl;
    flight_state = WAYPOINT;
    target_position = all_waypoints.front();
    all_waypoints.pop();
    target_position.print();
    cmd_position(target_position[0], target_position[1], target_position[2], target_position[3]);
    point3D p = local_to_global(point3D({target_position[0], target_position[1], target_position[2]}), global_home());
    handleInformation(p[0], p[1], p[2]);
}

void Flyer::landing_transition()
{
    cout << "landing transition" << endl;
    flight_state = LANDING;
    land();
}

void Flyer::disarming_transition()
{
    cout << "disarm transition" << endl;
    flight_state = DISARMING;
    if (m_bControlStatus) {
        printf("Disable offboard mode\n");
        cmd_offboard_control(false);
        m_bControlStatus = false;
    }
    disarm();
//    release_control();
}

void Flyer::manual_transition()
{
    cout << "manual transition" << endl;
    flight_state = MANUAL;
//    stop();
    in_mission = false;
}

void Flyer::plan_path()
{
    cout << "plan path" << endl;
    flight_state = PLANNING;
    double target_altitude = 1;
    int safety_distance = 5;
    target_position[2] = target_altitude;
    string dataPath = m_sAppPath + "/colliders.csv";
    MyData data(dataPath, ",");
//    set_home_position(data.getLon(), data.getLat(), 0);
//    _update_local_position(global_to_local(global_position(), global_home()));
    m_mSampler = new MySampler(data, safety_distance);
    m_vNodes = m_mSampler->sample(300);
    create_graph(10);
//    point3D home = global_home();
//    point3D gHome({-112.992182, 34.764013, 0.});
//    point3D gGoal({-112.993182, 34.765513, 3});
//    point3D lStart = global_to_local(gHome, gHome);
//    point3D lGoal = global_to_local(gGoal, gHome);
    point3D lStart = local_position();
    point3D lGoal = lStart + 3;
    lStart.print();
    lGoal.print();
    size_t vStart = find_closed_vertice(lStart);
    size_t vGoal = find_closed_vertice(lGoal);
    
    VVertexType path = a_star_for_graph(m_Graph, vStart, vGoal);
    if (path.size() > 0)
    {
        VVertexType pPath = prune_path(path);
        point3D p;
        for (int i = 0; i < pPath.size(); i++)
        {
            p = m_vGraphVertices[pPath[i]];
            all_waypoints.push(point4D({p[0], p[1], target_altitude, 0}));
//            point3D pg = local_to_global(p, global_home());
//            handleInformation(-pg[0], pg[1], pg[2]);
        }
    }
}
    
void Flyer::start(int t)
{
    cout << "starting connection" << endl;
    m_iFlyType = t;
    MyDrone::start();
}

void Flyer::stop()
{
    cout << "stop connection" << endl;
    MyDrone::stop();
}
bool Flyer::can_connect(point3D p1, point3D p2)
{
    return m_mSampler->can_connect(p1, p2);
}

void Flyer::create_graph(int k)
{
    tree3D tree(m_vNodes.begin(), m_vNodes.end());
    for (int i = 0; i < m_vNodes.size(); i++)
    {
        point3D p = m_vNodes[i];
        tree.nearest(p, k);
        for (int j = 0; j < tree.m_vNbrNodes.size(); j++)
        {
            point3D p1 = tree.m_vNbrNodes[j].point_;
            if (p == p1)
            {
                continue;
            }
            if (can_connect(p, p1))
            {
                size_t index, index1 = 0;
                auto itr = find(m_vGraphVertices.begin(), m_vGraphVertices.end(), p);
                if (itr == m_vGraphVertices.end())
                {
                    m_vGraphVertices.emplace_back(p);
                    index = m_Graph.insertVertex();
                }
                else
                {
                    index = distance(m_vGraphVertices.begin(), itr);
                }
                
                itr = find(m_vGraphVertices.begin(), m_vGraphVertices.end(), p1);
                if (itr == m_vGraphVertices.end())
                {
                    m_vGraphVertices.emplace_back(p1);
                    index1 = m_Graph.insertVertex();
                }
                else
                {
                    index1 = distance(m_vGraphVertices.begin(), itr);
                }
                m_Graph.insertEdge(index, index1);
            }
        }
    }
}
bool Flyer::bresenham_check(point3D p1, point3D p2)
{
    double x, y, x1, y1, x2, y2, dx, dy, z, m = 0;
    int eps = 0;
    x1 = p1[0];
    y1 = p1[1];
    x2 = p2[0];
    y2 = p2[1];
    dx = x2 - x1;
    dy = y2 - y1;
    x = x1;
    y = y1;
    z = p2[2];
    if (dx == 0)
    {
        return can_connect(p1, p2);
    }
    
    m = dy / dx;
    // region 1
    if (m >= 0 && m <= 1 && x1 < x2)
    {
        for (x = x1; x <= x2; x++)
        {
            if (!can_connect(p1, point3D({x, y, z})))
            {
                return false;
            }
            eps += dy;
            if ((eps << 1) >= dx)
            {
                y += 1;
                eps -= dx;
            }
        }
    }
    // region 2
    else if (m > 1 && y1 < y2)
    {
        for (y = y1; y < y2; y++)
        {
            if (!can_connect(p1, point3D({x, y, z})))
            {
                return false;
            }
            eps += dx;
            if ((eps << 1) >= dy)
            {
                x += 1;
                eps -= dy;
            }
        }
    }
    // region 3
    else if (m < -1 && y1 < y2)
    {
        for (y = y1; y <= y2; y++)
        {
            if (!can_connect(p1, point3D({x, y, z})))
            {
                return false;
            }
            eps += dx;
            if ((eps << 1) <= -dy)
            {
                x -= 1;
                eps += dy;
            }
        }
    }
    // region 4
    else if (-1 <= m && m <= 0 && x2 < x1)
    {
        for (x = x1; x >= x2; x--)
        {
            if (!can_connect(p1, point3D({x, y, z})))
            {
                return false;
            }
            eps += dy;
            if ((eps << 1) >= -dx)
            {
                y += 1;
                eps += dx;
            }
        }
    }
    // region 5
    else if (0 < m && m <= 1 && x2 < x1)
    {
        for (x = x1; x >= x2; x--)
        {
            if (!can_connect(p1, point3D({x, y, z})))
            {
                return false;
            }
            eps += dy;
            if ((eps << 1) <= dx)
            {
                y -= 1;
                eps -= dx;
            }
        }
    }
    // region 6
    else if (m > 1 && y2 < y1)
    {
        for (y = y1; y >= y2; y--)
        {
            if (!can_connect(p1, point3D({x, y, z})))
            {
                return false;
            }
            eps += dx;
            if ((eps << 1) <= dy)
            {
                x -= 1;
                eps -= dy;
            }
        }
    }
    // region 7
    else if (m < -1 && y2 < y1)
    {
        for (y = y1; y >= y2; y--)
        {
            if (!can_connect(p1, point3D({x, y, z})))
            {
                return false;
            }
            eps += dx;
            if ((eps << 1) >= -dy)
            {
                x += 1;
                eps += dy;
            }
        }
    }
    // region 8
    else if (-1 <= m && m < 0 && x1 < x2)
    {
        for (x = x1; x <= x2; x++)
        {
            if (!can_connect(p1, point3D({x, y, z})))
            {
                return false;
            }
            eps += dy;
            if ((eps << 1) <= -dx)
            {
                y -= 1;
                eps += dx;
            }
        }
    }
    return true;
}
VVertexType Flyer::prune_path(VVertexType path)
{
    VVertexType p = path;
    point3D p1, p2, p3;
    int i = 0;
    while (i < (p.size() - 2))
    {
        p1 = m_vGraphVertices[p[i]];
        p2 = m_vGraphVertices[p[i + 1]];
        p3 = m_vGraphVertices[p[i + 2]];
        // First check if p2 can be removed, if it can be checked whether p3 can be removed, if both are satisfied, remove p2
        if (bresenham_check(p1, p2) && bresenham_check(p1, p3))
        {
            p.erase(p. begin() + i + 1);
        }
        else
        {
            i += 1;
        }
    }
    return p;
}

size_t Flyer::find_closed_vertice(point3D p)
{
    std::vector<double> dists;
    for (int i = 0; i < m_vGraphVertices.size(); i++)
    {
        dists.push_back(p.distance(m_vGraphVertices.at(i)));
    }
    auto m = std::min_element(dists.begin(), dists.end());
    return std::distance(dists.begin(), m);
}

VVertexType Flyer::a_star_for_graph(andres::graph::Graph<> graph, size_t start, size_t goal)
{
    // Graph-based A * search algorithm
    VVertexType path;
    int path_cost = 0, current_cost = 0, branch_cost = 0, queue_cost = 0;
    std::priority_queue<PVertexType> queue;
    VVertexType visited;
    std::map<size_t, PVertexType> branch;
    bool found = false;
    
    if (start == goal)
    {
        return path;
    }
    
    queue.push(PVertexType(0, start));
    visited.emplace_back(start);

    while (!queue.empty())
    {
        PVertexType item = queue.top();
        queue.pop();
        size_t current_node = item.second;
        if (current_node == start)
        {
            current_cost = 0.0;
        }
        else
        {
            current_cost = branch[current_node].first;
        }
            
        if (current_node == goal)
        {
            found = true;
            break;
        }
        else
        {
            for(typename andres::graph::Graph<>::AdjacencyIterator it = graph.adjacenciesFromVertexBegin(current_node); it != graph.adjacenciesFromVertexEnd(current_node); ++it)
            {
                size_t adj = it->vertex();
                branch_cost = current_cost + 1; //Weight=1
                queue_cost = branch_cost + heuristic(m_vGraphVertices.at(adj), m_vGraphVertices.at(goal));
                if (find(visited.begin(), visited.end(), adj) == visited.end())
                {
                    visited.emplace_back(adj);
                    branch.insert(pair<size_t, PVertexType>(adj, PVertexType(branch_cost, current_node)));
                    queue.push(PVertexType(queue_cost, adj));
                }
            }
        }
    }
             
    if (found)
    {
        // retrace steps
        size_t n = goal;
        path_cost = branch[n].first;
        path.emplace_back(goal);
        printf("\r\npath : ");
        while (branch[n].second != start)
        {
            printf("%d ", n);
            path.emplace_back(branch[n].second);
            n = branch[n].second;
        }
        printf("\r\n");
        path.emplace_back(branch[n].second);
    }
    else
    {
        printf("**********************");
        printf("Failed to find a path!");
        printf("**********************");
    }
    reverse(path.begin(), path.end());
    return path;
}

queue<point4D> Flyer::getAllWayPoint()
{
    return all_waypoints;
}

void Flyer::handleInformation(double lo, double la, double alt)
{
    if (m_pCallBackFunc != nullptr)
    {
        (*m_pCallBackFunc)(m_pViewController, lo, la, alt);
    }
}

void Flyer::registerCallBackFunction(void * data, CallBackFunc func)
{
    m_pViewController = data;
    m_pCallBackFunc = func;
}

void Flyer::setAppPath(const char *path)
{
    m_sAppPath = path;
}
