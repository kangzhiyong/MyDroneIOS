//
//  my_sampler.hpp
//  FCND-CPPSim
//
//  Created by kangzhiyong on 2020/3/11.
//  Copyright © 2020 Fotokite. All rights reserved.
//

#ifndef my_sampler_hpp
#define my_sampler_hpp

#include "k_d_tree.hpp"
#include "my_data.hpp"
#include "my_polygon.hpp"

#include <vector>

class MySampler
{
private:
    std::vector<MyPolygon> m_mpPoloygons;
    double m_fXmin{0};
    double m_fXmax{0};
    double m_fYmin{0};
    double m_fYmax{0};
    double m_fZmin{0};
    double m_fZmax{0};
    double m_fMaxPolyXY{0};
    tree2D  m_kdTree2D;
    
public:
    MySampler(MyData datas, double safe_distance);
    void extract_polygons(MyData datas, double safe_distance);
    ~MySampler();
    VFloat uniform(double min, double max, int num);
    std::vector<point3D> sample(int num);
    bool can_connect(point3D p1, point3D p2);
};
#endif /* my_sampler_hpp */
