#ifndef __REGISTRATION_INTERFACE_H
#define __REGISTRATION_INTERFACE_H

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "sensor_type/cloud.h"

class RegistrationInterface
{  
   //虚基类，后续继承不同的匹配算法  （多态）
    public:
      virtual ~RegistrationInterface() = default;

      virtual bool SetInputTarget(const CloudData::CLOUD_PTR &input_target) = 0;
      virtual bool ScanMatch(const CloudData::CLOUD_PTR &input_source,
                              const Eigen::Matrix4f &predict_pose,
                              CloudData::CLOUD_PTR &result_cloud_ptr,
                              Eigen::Matrix4f &result_pose) = 0;
      virtual float GetFitnessScore() = 0;
};

#endif