#ifndef __POINTS_FILTER_INTERFACE_H
#define __POINTS_FILTER_INTERFACE_H

#include "sensor_type/cloud.h"

// 虚基类  
class CloudFilterInterface {
  public:
    virtual ~CloudFilterInterface() = default;

    virtual bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) = 0;
};

#endif
