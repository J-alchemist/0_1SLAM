#ifndef __POINTS_VOXEL_FILTER
#define __POINTS_VOXEL_FILTER

#include "tools/points_filter_interface.h"
#include <pcl/filters/voxel_grid.h>
#include <yaml-cpp/yaml.h>


class VoxelFilter: public CloudFilterInterface {
  public: 
    VoxelFilter(const YAML::Node& node);
    VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

  private:
    bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);  // m为单位，空间划分为删格

  private:
    pcl::VoxelGrid<CloudData::POINT> voxel_filter_; //VoxelGrid以体素栅格重心来代表这个立方体
    //pcl::ApproximateVoxelGrid<CloudData::POINT> voxel_filter_;  // 中心，效果略差，速度稍快
};

#endif
