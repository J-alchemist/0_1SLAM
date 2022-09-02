#ifndef __LOOP_CLOSING_H
#define __LOOP_CLOSING_H


#include <iostream>
#include <deque>
#include <ros/ros.h>
#include <Eigen/Dense> 
#include "sensor_type/imu.h"
#include "sensor_type/gnss.h"
#include "sensor_type/cloud.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include "tools/voxel_filter.h"
#include "tools/ndt_registration.h"
#include "tools/icp_registration.h"
#include "tools/global_defination.h"
#include "custom_msgs/poseWithIndex.h"
#include "tools/internal_pose.h"
#include "tools/data_publisher.h"
#include <mutex>

class LoopClosing {

    public:
        LoopClosing(ros::NodeHandle& nh, std::string& imu_frame_id, 
                                std::string& lidar_frame_id);
        ~LoopClosing();
        typedef std::shared_ptr<LoopClosing> ptr;

        ros::NodeHandle nh_;
        ros::Subscriber key_frame_sub_, key_gnss_sub_;   // 发布、订阅
        ros::Publisher loop_pose_pub_; 
        std::shared_ptr<LoopPosePublisher> loop_pose_pub_ptr_;

    private:
        std::mutex buff_mutex_;
        std::shared_ptr<CloudFilterInterface> scan_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> map_filter_ptr_;
        std::shared_ptr<RegistrationInterface> registration_ptr_;

        std::string data_path_ = "";            // ws root
        std::string key_frames_path_ = "";      // 关键帧存储位置

        int extend_frame_num_ = 3;              // 某个历史帧前后3帧，配准为local_map，进行scan2map回环检测
        int loop_step_ = 10;
        int diff_num_ = 100;
        float detect_area_ = 10.0;
        float fitness_score_limit_ = 2.0;

    public: 
        void Exec();    // 上层函数

    private:
        std::deque<KeyFrame> new_key_frame_;        // 回调数据
        std::deque<KeyFrame> new_gnss_frame_;

        std::deque<KeyFrame> key_frame_buff_;       // 现在这个直接接收回调关键帧
        std::deque<KeyFrame> key_gnss_buff_;
        KeyFrame current_key_frame_;                // 从buf取出当前帧
        KeyFrame current_key_gnss_;
        LoopPose current_loop_pose_;                // 发布到后端的闭环帧

        std::deque<KeyFrame> all_key_frames_;       // 存储所有合法的关键帧 
        std::deque<KeyFrame> all_key_gnss_;
        bool has_new_loop_pose_ = false;            // 检测到新的回环   

    public:
        bool Update(const KeyFrame key_frame, const KeyFrame key_gnss); // 上层函数

    private:
        bool DetectNearestKeyFrame(int& index);     // 回环检测核心函数
        bool CloudRegistration(int closing_id);
        bool JointMap(int id, CloudData::CLOUD_PTR& map_cloud_ptr, Eigen::Matrix4f& map_pose);
        bool JointScan(CloudData::CLOUD_PTR& scan_cloud_ptr, Eigen::Matrix4f& scan_pose);
        
    private:
        bool ReadData();
        bool HasData();
        bool ValidData();
        template<class T1,class T2>
        void ParseData(std::deque<T1>& callback_buff, std::deque<T2>& operate_buff);
        void keyFrameMsgCallback(const custom_msgs::poseWithIndex::ConstPtr& msg_ptr);
        void keyGnssMsgCallback(const custom_msgs::poseWithIndex::ConstPtr& msg_ptr);     
        bool PublishData();

    private:
        bool InitWithConfig();
        bool InitParamAndPath(const YAML::Node& config_node);
        bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);
        bool InitRegistrationMethod(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
    

};









#endif
