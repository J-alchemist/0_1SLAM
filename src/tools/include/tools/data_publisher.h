#ifndef __DATA_PUBLISHER_H
#define __DATA_PUBLISHER_H

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "tools/internal_pose.h"
#include "custom_msgs/poseWithIndex.h"
#include "sensor_type/cloud.h"


class OdomPublisher {

    public:
        OdomPublisher(ros::Publisher& puber, std::string frame_id, std::string child_frame_id);
    public:
        void Publish(Eigen::Matrix4f& trans_matrix);
        void Publish(Eigen::Matrix4f& trans_matrix, double time);

    private:
        nav_msgs::Odometry msg_;
        ros::Publisher publisher_;
        std::string frame_id_, child_frame_id_;
         
};


class KeyFramePublisher {

    public:
        KeyFramePublisher(ros::Publisher& puber, std::string frame_id);

    public:
        void Publish(KeyFrame& key_frame);
        void Publish(const std::deque<KeyFrame>& key_frames);// 关键帧群

    private:
        custom_msgs::poseWithIndex msg_;
        ros::Publisher publisher_;
        std::string frame_id_;
};

class LoopPosePublisher {

    public:
        LoopPosePublisher(ros::Publisher& puber, std::string frame_id);

    public:
        void Publish(LoopPose& loop_frame);
        
    private:
        custom_msgs::poseWithIndex msg_;
        ros::Publisher publisher_;
        std::string frame_id_;
};


class CloudPublisher {

    public:
        CloudPublisher(ros::Publisher& puber, std::string frame_id);

    public:
        void Publish(CloudData::CLOUD_PTR&  cloud_ptr_input);
        void Publish(CloudData::CLOUD_PTR&  cloud_ptr_input, double time);

    private:
        ros::Publisher publisher_;
        std::string frame_id_;


};



#endif