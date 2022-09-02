#ifndef __BOX_FILTER_H
#define __BOX_FILTER_H

#include "tools/points_filter_interface.h"
#include <yaml-cpp/yaml.h>
#include <vector>
#include <iostream>
#include <pcl/filters/crop_box.h>

class BoxFilter: public CloudFilterInterface {
  public:
    BoxFilter(YAML::Node node);
    BoxFilter() = default;

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

    void SetSize(std::vector<float> size);
    void SetOrigin(std::vector<float> origin);
    std::vector<float> GetEdge();

  private:
    void CalculateEdge();

  private:
    pcl::CropBox<CloudData::POINT> pcl_box_filter_;  

    std::vector<float> origin_; //窗口的中心点 
    std::vector<float> size_;   //窗口大小
    std::vector<float> edge_;   //根据origin_和size_计算的方形窗口的两个点 
};



#endif
