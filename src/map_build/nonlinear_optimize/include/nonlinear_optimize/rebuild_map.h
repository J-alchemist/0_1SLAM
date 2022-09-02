#ifndef __REBUILD_MAP_H
#define __REBUILD_MAP_H

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
#include "tools/voxel_filter.h"
#include "tools/ndt_registration.h"
#include "tools/icp_registration.h"
#include "tools/global_defination.h"
#include "tools/tf_convert.h" 
#include "tools/file_manager.h"
#include "tools/data_publisher.h"
#include "custom_msgs/poseWithIndex.h"
#include "graph_optimizer/g2o/g2o_graph_optimizer.h"
#include "tools/internal_pose.h"
#include "custom_msgs/saveMap.h"
#include "custom_msgs/optimizeMap.h"
#include <mutex> 

// 重构优化后的全局地图
/**
 *需要的数据: synced_cloud\key_frame\transformed_laser_odom\转换后的关键帧位姿
*/
 class RebuildMap {   
            
    public:
        RebuildMap(ros::NodeHandle& nh, std::string& imu_frame_id, 
                                std::string& lidar_frame_id);
        ~RebuildMap();
        typedef std::shared_ptr<RebuildMap> ptr;

    private:
        ros::NodeHandle nh_;     // 发布、订阅
        ros::Subscriber cloud_sub_, transformed_odom_sub_, key_frame_sub_, optimized_frames_pose_sub_;  
        ros::Publisher optimized_odom_pub_, current_scan_pub_, global_map_pub_, local_map_pub_;
        std::shared_ptr<OdomPublisher>  optimized_odom_pub_ptr_;
        std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
        std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
        std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;

        ros::ServiceServer saveMap_Server_;        // 服务

    private:
        std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;
        bool InitWithConfig();
        bool InitParam(const YAML::Node& config_node);
        bool InitDataPath(const YAML::Node& config_node);
        bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);

    private:
        std::string imu_frame_id_ = "";
        std::string lidar_frame_id_ = "";
        std::string data_path_ = "";
        std::string key_frames_path_ = ""; 
        std::string map_path_ = ""; 
        int local_frame_num_  = 20;
        bool has_new_local_map_ = false;
        bool has_new_global_map_ = false; 
        bool need_save_map_ = false;	 		// Server

        std::deque<KeyFrame> new_key_frame_buff_;    // 回调数据
        std::deque<PoseData> new_transformed_odom_buff_;  
        std::deque<CloudData> new_cloud_data_buff_;       
        std::deque<KeyFrame> new_optimized_key_frames_; 

        std::deque<KeyFrame> key_frame_buff_;        // /key_frame 
        std::deque<PoseData> transformed_odom_buff_; // /transformed_laser_odom, 存储对齐到gnss原点的
        std::deque<CloudData> cloud_data_buff_;      // /sync_cloud  两个容器存储 nonlinear和rebuild分离
        std::deque<KeyFrame> optimized_key_frames_;  // /optimized_key_frames 优化了帧的pose
        std::deque<KeyFrame> all_key_frames_; 
        
        CloudData current_cloud_data_, optimized_cloud_;
        PoseData current_transformed_odom_;

        Eigen::Matrix4f pose_to_optimize_ = Eigen::Matrix4f::Identity();    // 当前位置与优化位置的相对位置
        PoseData optimized_odom_;       // 当前帧的优化位置
        std::deque<KeyFrame> final_optimized_key_frames_;   // 最后一次优化的位姿,所有顶点
    public:
        void Exec();

    private:
        std::mutex buff_mutex_;     // 数据校验lock
        template<class T1,class T2>
        void ParseData(std::deque<T1>& callback_buff, std::deque<T2>& operate_buff);
        void keyFrameMsgCallback(const custom_msgs::poseWithIndex::ConstPtr& msg_ptr);
        void laserOdomCallBack(const nav_msgs::OdometryConstPtr& odom_msg_ptr);
        void syncCloudMsgCallBack(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);
        void optimizedKeyFrameMsgCallback(const nav_msgs::Path::ConstPtr& key_frames_msg_ptr);
        bool saveMapCallBack(custom_msgs::saveMap::Request& req, custom_msgs::saveMap::Response& res);
        bool ReadData();
        bool HasData();
        bool ValidData(); 
        bool UpdateWithNewKeyFrame(std::deque<KeyFrame>& new_key_frames,
                                    PoseData transformed_data,
                                    CloudData cloud_data);
        bool UpdateWithOptimizedKeyFrames();
        bool OptimizeKeyFrames();

    private:
        bool SaveMap(); 
        bool JointCloudMap(const std::deque<KeyFrame>& key_frames, CloudData::CLOUD_PTR& map_cloud_ptr);
        bool JointLocalMap(CloudData::CLOUD_PTR& local_map_ptr); 
        bool JointGlobalMap(CloudData::CLOUD_PTR& global_map_ptr); 
        void PublishLocalData();
        void PublishGlobalData();
};





#endif

