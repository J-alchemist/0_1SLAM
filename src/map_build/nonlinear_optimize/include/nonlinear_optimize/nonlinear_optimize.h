#ifndef __NONLINEAR_OPTIMIZE_H
#define __NONLINEAR_OPTIMIZE_H

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

class NonlinearOpt {

    public:
        NonlinearOpt(ros::NodeHandle& nh, std::string& imu_frame_id, 
                                std::string& lidar_frame_id);
        ~NonlinearOpt();
        typedef std::shared_ptr<NonlinearOpt> ptr;

        ros::NodeHandle nh_;
        ros::Subscriber cloudSub_;   // 发布、订阅
        ros::Subscriber laser_odom_sub_, gnss_odom_sub_, loop_pose_sub_;
        ros::Publisher  key_frame_pub_,  key_gnss_pub_,  odom_pub_, optimized_key_frames_pub_;
        std::shared_ptr<KeyFramePublisher> key_frame_pub_ptr_;
        std::shared_ptr<KeyFramePublisher> key_gnss_pub_ptr_;
        std::shared_ptr<OdomPublisher>     odom_pub_ptr_;
        std::shared_ptr<KeyFramePublisher> optimized_key_frames_pub_ptr_;

        ros::ServiceServer optimizeMap_Server_;  // 服务

        std::string data_path_ = "";            // 路径
        std::string key_frames_path_ = "";
        std::string trajectory_path_ = "";

        std::ofstream ground_truth_ofs_;        // 轨迹文件
        std::ofstream laser_odom_ofs_;
        std::ofstream optimized_pose_ofs_;
        
        std::string imu_frame_id_ = "";
        std::string lidar_frame_id_ = "";
    private:
        std::mutex buff_mutex_;
        float key_frame_distance_ = 2.0; 
        std::deque<KeyFrame> key_frame_buff_;
        KeyFrame current_key_frame_;            // 是否将current_cloud/gnss作为keyframe，index是同步的
        KeyFrame current_key_gnss_;
        bool has_new_key_frame_ = false;        // 该帧是否作为关键帧
        bool has_new_optimized_ = false;        // 是否产生了新的优化

    private:
        bool last_optimize_map_ = false;        // server

    private:
        std::deque<CloudData> new_cloud_data_buff_;     // 话题回调数据
        std::deque<PoseData> new_gnss_odom_data_buff_;
        std::deque<PoseData> new_laser_odom_data_buff_;
        std::deque<LoopPose> new_loop_pose_data_buff_;
        std::deque<CloudData> cloud_data_buff_;         // 算法输入数据
        std::deque<PoseData> gnss_odom_data_buff_;
        std::deque<PoseData> laser_odom_data_buff_;
        std::deque<LoopPose> loop_pose_data_buff_;
        CloudData current_cloud_data_;
        PoseData current_gnss_odom_data_;
        PoseData current_laser_odom_data_;
        LoopPose current_loop_pose_data_;

    private:
        std::deque<Eigen::Matrix4f> optimized_pose_;        // 发布出去 优化后的位姿 
        std::shared_ptr<InterfaceGraphOptimizer> graph_optimizer_ptr_;  // 优化器
        
        class GraphOptimizerConfig {        
            public:
                GraphOptimizerConfig() {
                    odom_edge_noise.resize(6);
                    close_loop_noise.resize(6); // 回环的xyz+三个角
                    gnss_noise.resize(3);       // gnss只加入了xyz的位置观测
                }

            public:
                // 关闭之后=开环的前端
                bool use_gnss = true;
                bool use_loop_close = false;
                 //噪声
                Eigen::VectorXd odom_edge_noise;       
                Eigen::VectorXd close_loop_noise;
                Eigen::VectorXd gnss_noise;
                // 策略
                int optimize_step_with_key_frame = 100;
                int optimize_step_with_gnss = 100;
                int optimize_step_with_loop = 10;
        };
        GraphOptimizerConfig graph_optimizer_config_;

        // 执行每次优化前，需要的数据帧数
        int new_gnss_cnt_ = 0;
        int new_loop_cnt_ = 0;
        int new_key_frame_cnt_ = 0;
    
    public:
        void Exec();

    private:
        template<class T1,class T2>
        void ParseData(std::deque<T1>& callback_buff, std::deque<T2>& operate_buff);
        bool ReadData();
        bool HasData();
        bool ValidData();
        void SyncCloudMsgCallBack(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);
        void GnssOdomMsgCallBack(const nav_msgs::OdometryConstPtr& odom_msg_ptr);
        void FrontLaserOdomCallBack(const nav_msgs::OdometryConstPtr& odom_msg_ptr);
        void LoopPoseCallBack(const custom_msgs::poseWithIndex::ConstPtr& loop_pose_ptr);
        void PoseTypeMatrix2KeyFrame(std::deque<KeyFrame>& buff, std::deque<Eigen::Matrix4f>& optimized_pose);
        bool PublishData();

    private:
        bool InitWithConfig();
        bool InitGraphOptimizer(const YAML::Node& cn);
        bool InitDataPathAndParam(const YAML::Node& cn);

    public:
        // 插入回环边
        bool IsInsertLoopPose();
        // 后端优化更新
        bool UpdateBackEnd();
    private:
        bool Update(const CloudData& cloud_data, const PoseData& laser_odom, const PoseData& gnss_pose);
        bool IsNewKeyFrame(const CloudData& cloud_data, const PoseData& laser_odom, const PoseData& gnss_odom);
        bool AddNodeAndEdge(const PoseData& gnss_data);
        bool IsOptimized(); // 是否开始优化
        bool ForceOptimize();
        bool GetOptimizedPose();// 获取优化后的位姿, 并保存.txt
        bool SavePose(std::ofstream& ofs, const Eigen::Matrix4f& pose);

    private:
        bool forceOptimizeMapCallBack(custom_msgs::optimizeMap::Request& req, 
                                custom_msgs::optimizeMap::Response& res);
        bool saveMapCallBack(custom_msgs::saveMap::Request& req, 
                                custom_msgs::saveMap::Response& res);

};

#endif
