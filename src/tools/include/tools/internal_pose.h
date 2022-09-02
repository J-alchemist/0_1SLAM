#ifndef __INTERNAL_POSE_H
#define __INTERNAL_POSE_H

#include <iostream>
#include <deque>
#include <Eigen/Dense>
#include "sensor_type/cloud.h"


// 前后端+地图匹配，里程计位姿
class PoseData {
  public:
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    double time = 0.0;
};

// 前端，关键帧: 点云+位姿
class Frame {
  public:
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    CloudData cloud_data; 
};

// 后端==> 回环
class KeyFrame { 
  public:
    double time = 0.0;
    unsigned int index = 0;       // 关键帧的id
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

  public:
    Eigen::Quaternionf GetQuaternion(); // 旋转矩阵转四元素
};

// 回环==> 后端
class LoopPose { 
  public:
    double time = 0.0;            // 当前帧的time
    unsigned int index0 = 0;      // 闭环帧的index
    unsigned int index1 = 0;      // 当前帧的index
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity(); // 闭环帧与当前帧的相对位姿  

  public:
    Eigen::Quaternionf GetQuaternion();
};














#endif
