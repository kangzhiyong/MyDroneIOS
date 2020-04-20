//
//  planning_utils.hpp
//  FCND-CPPSim
//
//  Created by kangzhiyong on 2020/3/16.
//  Copyright © 2020 Fotokite. All rights reserved.
//

#ifndef planning_utils_hpp
#define planning_utils_hpp

#include <cmath>
using namespace std;

inline double norm(double x0, double y0, double z0, double x1, double y1, double z1)
{
    return sqrt(pow(x0 - x1, 2) + pow(y0 - y1, 2) + pow(z0 - z1, 2));
}

#endif /* planning_utils_hpp */
