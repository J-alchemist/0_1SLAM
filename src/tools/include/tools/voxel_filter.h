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
    bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

  private:
    pcl::VoxelGrid<CloudData::POINT> voxel_filter_;
    //pcl::ApproximateVoxelGrid<CloudData::POINT> voxel_filter_;
};

#endif
