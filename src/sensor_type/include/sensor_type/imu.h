#ifndef __IMU_TYPE_H
#define __IMU_TYPE_H

#include <iostream>
#include <deque>
#include <cmath>
#include <mutex>
#include <Eigen/Dense>


class IMUData {
  public:

    struct LinearAcceleration {   //线加速度
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    struct AngularVelocity {    //角速度
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    class Orientation {     //姿态
      public:
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;
      
      public:
        void Normlize() {
          double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));    //归一化四元素，便于Eigen库直接用于旋转变换
          x /= norm;
          y /= norm;
          z /= norm;
          w /= norm;
        }
    };

    double time = 0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    Orientation orientation;
  
  public:
    // 把四元数转换成旋转矩阵送出去
    Eigen::Matrix3f GetOrientationMatrix();
    //时间同步，插值
    static bool SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time); 
  
};



#endif

