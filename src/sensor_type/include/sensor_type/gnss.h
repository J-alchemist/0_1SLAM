#ifndef __GNSS_TYPE_H
#define __GNSS_TYPE_H

#include <iostream>
#include <deque>
#include "../../GeographicLib/include/Geocentric/LocalCartesian.hpp"

class GNSSData {  
  public:
    double time = 0.0;
    double longitude = 0.0;
    double latitude = 0.0;
    double altitude = 0.0;
    double local_E = 0.0;
    double local_N = 0.0;
    double local_U = 0.0;
    int status = 0;
    int service = 0;

    static double origin_longitude;   // 存储原点
    static double origin_latitude;
    static double origin_altitude;

  private:
    static GeographicLib::LocalCartesian geo_converter; 
    static bool origin_position_inited; 

  public: 
    void InitOriginPosition();
    void UpdateXYZ();
    // 数据插值
    static bool SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time);
};


#endif
