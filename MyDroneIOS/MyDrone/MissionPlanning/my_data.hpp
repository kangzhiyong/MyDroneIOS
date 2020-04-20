//
//  my_data.hpp
//  FCND-CPPSim
//
//  Created by kangzhiyong on 2020/3/11.
//  Copyright © 2020 Fotokite. All rights reserved.
//

#ifndef my_data_hpp
#define my_data_hpp

#include <algorithm>
#include <vector>

typedef std::vector<double> VFloat;

class MyData
{
private:
    double m_dLat;
    double m_dLon;
    VFloat m_qvNorths;
    VFloat m_qvEasts;
    VFloat m_qvAlts;
    VFloat m_qvDNorths;
    VFloat m_qvDEasts;
    VFloat m_qvDAlts;
public:
    MyData(std::string fileName, std::string delimiter);
    ~MyData();
    double getXMin();
    double getXMax();
    double getYMin();
    double getYMax();
    double getZMin();
    double getZMax();
    double getMaxPolyXY();
    size_t dataCount();
    double getNorth(int i);
    double getEast(int i);
    double getAlt(int i);
    double getDNorth(int i);
    double getDEast(int i);
    double getDAlt(int i);
    double getLat();
    double getLon();
};
#endif /* my_data_hpp */
