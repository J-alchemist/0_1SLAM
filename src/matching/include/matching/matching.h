#ifndef __POINTS_MATCHING_H
#define __POINTS_MATCHING_H

#include <ros/ros.h>
#include <deque>
#include <vector>
#include <Eigen/Dense>
#include "sensor_type/imu.h"
#include "sensor_type/gnss.h"
#include "sensor_type/cloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>
#include "tools/registration_interface.h"
#include "tools/icp_registration.h"
#include "tools/ndt_registration.h"
#include "tools/points_filter_interface.h"
#include "tools/box_filter.h"
#include "tools/voxel_filter.h"
#include "tools/tf_convert.h"
#include "tools/global_defination.h"
#include "tools/internal_pose.h"
#include "tools/data_publisher.h"
#include <mutex>
#include <geometry_msgs/Pose2D.h>

class Matching  { 

    public:
        Matching(ros::NodeHandle& nh, std::string& lidar_frame_id);
        ~Matching();
        typedef std::shared_ptr<Matching> ptr; 
        
    public:
        ros::NodeHandle nh_;
        ros::Subscriber cloudSub_, gnssSub_;
        ros::Publisher global_map_pub_, local_map_pub_, current_scan_pub_,laser_odom_pub_;
        ros::Publisher laser_2dpose_pub_;
        TFConvert::ptr laser_tf_pub_;
        std::shared_ptr<OdomPublisher> laser_odom_pub_ptr_;
        std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;
        std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
        std::shared_ptr<CloudPublisher> global_map_pub_ptr_;

    private:
        std::mutex buff_mutex_;
        std::string lidar_frame_id_="";
        std::string data_path_ = "";
        std::deque<PoseData> new_pose_data_;
        std::deque<CloudData> new_cloud_data_;
        void getSyncCloudMsgCallBack(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);
        void getGnssOdomMsgCallBack(const nav_msgs::OdometryConstPtr& odom_msg_ptr);
        
        std::deque<PoseData> gnss_data_buff_;
        std::deque<CloudData> cloud_data_buff_;
        template<class T1, class T2>
        void ParseData(std::deque<T1>& callback_buff, std::deque<T2>& operate_buff);

    public:
        void Exec();

    private:
        CloudData current_cloud_data_;
        PoseData current_gnss_data_;
        Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();  // 雷达里程计

    private:
        // 滤波 & 匹配
        std::string map_path_ = "";                                 // 配置文件中地图的存放路径
        std::shared_ptr<RegistrationInterface> registration_ptr_;   // 匹配方式
        std::shared_ptr<BoxFilter> box_filter_ptr_;                 // 分割
        std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;
        
    private:
        bool ReadData();
        bool HasData();
        bool ValidData();
        bool UpdateMatching();
        bool PublishData();

        bool InitWithConfig();
        bool InitRegistrationMethod(std::shared_ptr<RegistrationInterface>& registration_ptr, 
                                                const YAML::Node& config_node);
        bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, 
                                                    const YAML::Node& config_node);

    private:
        bool has_new_global_map_ = false;   //获取全局地图
        bool has_new_local_map_ = false;    //分割出局部地图
        bool has_inited_ = false;           //gnss数据已经给到了 局部地图初始化

        CloudData::CLOUD_PTR global_map_ptr_;
        CloudData::CLOUD_PTR local_map_ptr_;
        CloudData::CLOUD_PTR current_scan_ptr_;

        Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();       // 匹配的初始值
        Eigen::Matrix4f current_gnss_pose_ = Eigen::Matrix4f::Identity();
        
        // Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();

    public:
        bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);
        bool InitGlobalMap();

        bool SetGNSSPose(const Eigen::Matrix4f& gnss_pose);
        bool ResetLocalMap(float x, float y, float z);

        void GetGlobalMap(CloudData::CLOUD_PTR& global_map);
        void GetLocalMap(CloudData::CLOUD_PTR& global_map);
        
};



#endif
