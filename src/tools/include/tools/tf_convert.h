#ifndef __TF_CONVERT_H
#define __TF_CONVERT_H

#include <string>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


class TFConvert {

    public:
        TFConvert(std::string parent_frame, std::string child_frame);
        ~TFConvert();
        typedef std::shared_ptr<TFConvert> ptr; 

    public:    
        bool LookupData(Eigen::Matrix4f& transform_matrix);   //获取某两个坐标系的变换
        void SendTransform(Eigen::Matrix4f pose, double time);

    private:
        //ros的tf转为Eigen变换矩阵发出
        bool TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix);  
        bool TransformToMatrix(const geometry_msgs::TransformStamped& trans, Eigen::Matrix4f& transform_matrix);  

    private:
        std::string parent_frame_id_;
        std::string child_frame_id_;
        tf::TransformListener listener_;

        tf::StampedTransform transform_;
        tf::TransformBroadcaster broadcaster_; 
};



#endif


