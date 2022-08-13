#ifndef __TF_CONVERT
#define __TF_CONVERT


#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string>

class TFConvert{

    public:
        TFConvert(std::string base_frame, std::string child_frame);
        ~TFConvert();
        typedef std::shared_ptr<TFConvert> ptr; 

    public:    
        bool LookupData(Eigen::Matrix4f& transform_matrix);   //获取某两个坐标系的变换
    
    private:
        //ros的tf转为Eigen变换矩阵发出
        bool TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix);  
        bool TransformToMatrix(const geometry_msgs::TransformStamped& trans, Eigen::Matrix4f& transform_matrix);  

    private:
        std::string base_frame_id_;
        std::string child_frame_id_;
        tf::TransformListener listener_;
};



#endif


