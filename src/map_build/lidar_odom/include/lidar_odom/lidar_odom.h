#ifndef __LIDAR_ODOMETRY_H
#define __LIDAR_ODOMETRY_H

#include <deque>
#include <iostream>
#include <boost/filesystem.hpp>
#include <fstream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <nav_msgs/Odometry.h>
#include "sensor_type/imu.h"
#include "sensor_type/gnss.h"
#include "sensor_type/cloud.h"
#include "tools/tf_convert.h" 
#include "tools/voxel_filter.h"
#include "tools/ndt_registration.h"
#include "tools/icp_registration.h"
#include "tools/global_defination.h"
#include "tools/file_manager.h"
#include "tools/internal_pose.h"
#include "tools/data_publisher.h"
#include <mutex>

// 里程计类: 与gnss解耦,留有接口
// 不再有地图保存功能
class LidarOdom {

    public:
        LidarOdom(ros::NodeHandle& nh, std::string& imu_frame_id, 
                         std::string& lidar_frame_id);
        ~LidarOdom();
        typedef std::shared_ptr<LidarOdom> ptr;

    public:
        ros::NodeHandle nh_;
        ros::Subscriber cloudSub_, gnssOdomSub_;   // 发布、订阅
        ros::Publisher laser_odom_pub_;
        std::shared_ptr<OdomPublisher> laser_odom_pub_ptr_;
  
        std::string data_path_;                         // root路径
        std::string imu_frame_id_, lidar_frame_id_;

        std::mutex buff_mutex_;
    public:
        CloudData::CLOUD_PTR local_map_ptr_;       
        CloudData::CLOUD_PTR global_map_ptr_;
        CloudData::CLOUD_PTR current_scan_ptr_;
        CloudData::CLOUD_PTR result_cloud_ptr_;     // 配准的结果点云
        
    public:
        std::deque<PoseData> new_pose_data_;
        std::deque<CloudData> new_cloud_data_;
        void getSyncCloudMsgCallBack(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);
        void getGnssOdomMsgCallBack(const nav_msgs::OdometryConstPtr& odom_msg_ptr);
        void getSyncImuMsgCallBack(const sensor_msgs::ImuConstPtr& imu_msg_ptr);
        
        std::deque<PoseData> gnss_data_buff_;
        std::deque<CloudData> cloud_data_buff_;
        template<class T1,class T2>
        void ParseData(std::deque<T1>& callback_buff, std::deque<T2>& operate_buff);

    private:
        Eigen::Matrix4f gnss_odometry_ = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
        CloudData current_cloud_data_;
        PoseData current_gnss_data_;
        Frame current_frame_;

        std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;          // 滤波、匹配
        std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
        std::shared_ptr<RegistrationInterface> registration_ptr_; 

        std::deque<Frame> local_map_frames_;    // 局部地图
        std::deque<Frame> global_map_frames_;   // 只是存储关键帧位姿,点云存储到硬盘

        float key_frame_distance_ = 2.0;        // 2m存储一个关键帧 
        int local_frame_num_ = 20;              // 局部地图滑动的窗口大小

        bool has_new_global_map_ = false;
        bool has_new_local_map_ = false;

    public:
        void Exec();

    private: 
        bool UpdateLaserOdometry();
        bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);
        bool UpdateWithNewKeyFrame(const Frame& new_key_frame);
        bool UpdateGNSSOdometry();
        bool SaveMap();                 // 配准前端的全局地图,并保存
        bool SaveTrajectory();          // 保存轨迹文件
        bool PublishData();

    private:
        bool ReadData();
        bool HasData();
        bool ValidData();

    private:
        bool InitWithConfig();
        bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);
        bool InitRegistrationMethod(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
        bool InitDataPath(const YAML::Node& config_node);       //创建文件夹
        bool InitParam(const YAML::Node& config_node);

};





#endif

