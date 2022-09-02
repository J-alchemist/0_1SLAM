#ifndef __DATA_SYNC_H
#define __DATA_SYNC_H

#include <deque>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include "sensor_type/imu.h"
#include "sensor_type/gnss.h"
#include "sensor_type/cloud.h"
#include "tools/tf_convert.h"
#include "tools/data_publisher.h"
#include <mutex>
#include <fstream>

class DataSync { 

    public:
        DataSync(ros::NodeHandle& nh, std::string& imu_frame_id, 
                        std::string& lidar_frame_id, std::string& mode, double* origin);
        ~DataSync();
        typedef std::shared_ptr<DataSync> ptr;  

    public:
        ros::NodeHandle nh_;
        ros::Subscriber cloudSub_, imuSub_, gnssSub_;
        // std::shared_ptr<ros::Subscriber> test_;
        ros::Publisher cloudPub_, gnssPub_, imuPub_;
        std::shared_ptr<CloudPublisher> cloudPub_ptr_;
        std::shared_ptr<OdomPublisher> gnssPub_ptr_;

    public:
        std::mutex buff_mutex_;
        const char* env_p = nullptr;            //保存gnss原点
        std::fstream fout;
        std::deque<IMUData> new_imu_data_;      //存放原始数据
        std::deque<GNSSData> new_gnss_data_;
        std::deque<CloudData> new_cloud_data_;
        std::deque<IMUData> synced_imu_data_;     //同步后数据
        std::deque<GNSSData> synced_gnss_data_;
        std::deque<CloudData> synced_cloud_data_;
        IMUData current_imu_data_;
        GNSSData current_gnss_data_;
        CloudData current_cloud_data_;

        TFConvert::ptr lidar_to_imu_ptr_;      
        Eigen::Matrix4f lidar_to_imu_;       // 坐标变换矩阵
        // Eigen::Isometry3f lidar_to_imu_;  // 这种类型4x4矩阵可以直接乘以3维非齐次向量

        Eigen::Matrix4f gnss_pose_;

        std::string lidar_frame_id_,imu_frame_id_;

    private:     
        //畸变
        float scan_period_ = 0.1;   // 0.1s 雷达扫描周期
        Eigen::Vector3f velocity_, angular_rate_;

    private:
        std::string sys_mode_;
        double origin_lon_, origin_lat_, origin_alt_;

    private: 
        void imuMsgCallBack( const sensor_msgs::ImuConstPtr& imu_msg_ptr);
        void gnssMsgCallBack( const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);
        void cloudMsgCallBack( const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);
        bool syncImuData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time);
        bool syncGnssData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time);
        template<class T1,class T2>
        void ParseData(std::deque<T1>& callback_buff, std::deque<T2>& operate_buff);
        void SaveGnssOrigin(double& lon, double& lat, double& al);
    public:
        bool Exec();

    private: 
        bool DataInterpolation();
        bool HasData();
        bool ValidData();
        bool InitGNSS();
        bool InitCalibration();
        bool TransformData();       // 畸变处理
        bool PublishData();
        // 畸变实际处理函数
        void VelocityToLidar(Eigen::Matrix4f transform_matrix);
        bool AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr);
};




#endif
