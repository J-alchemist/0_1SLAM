#ifndef __DATA_SYNC
#define __DATA_SYNC

#include <deque>
#include <iostream>
#include <ros/ros.h>
#include "sensor_type/imu.h"
#include "sensor_type/gnss.h"
#include "sensor_type/cloud.h"
#include <sensor_msgs/Imu.h>
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/NavSatFix.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_type/tf_convert.h"
#include "nav_msgs/Odometry.h"

class DataSync { 

    public:
        DataSync();
        ~DataSync();
        typedef std::shared_ptr<DataSync> ptr;  

    public:
        std::string workDir_;
        ros::NodeHandle nh_;
        ros::Subscriber cloudSub_, imuSub_, gnssSub_;
        ros::Publisher cloudPub_, imuPub_, gnssPub_;

    public:
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
        Eigen::Matrix4f lidar_to_imu_;

        Eigen::Matrix4f gnss_pose_;

        std::string lidar_frame_id_,imu_frame_id_;

        void imuMsgCallBack( const sensor_msgs::ImuConstPtr& imu_msg_ptr);
        void gnssMsgCallBack( const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);
        void cloudMsgCallBack( const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);
        bool syncImuData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time);
        bool syncGnssData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time);
    
    public:


    public:
        void parseImuData(std::deque<IMUData>& deque_data);
        void parseGnssData(std::deque<GNSSData>& deque_data);
        void parseCloudData(std::deque<CloudData>& deque_data);


    public:
        bool Run();

    private: 
        bool ReadData();
        bool HasData();
        bool ValidData();
        bool InitGNSS();
        bool InitCalibration();
        bool TransformData();
        bool PublishData();
        bool AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr);
};




#endif
