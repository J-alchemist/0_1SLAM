/**
 * 2022/08/25 by GuoJian
 * 前端里程计 也是以gnss位姿初始化，后续不使用
*/
#include "lidar_odom/lidar_odom.h"

int main(int argc, char** argv) { 

    ros::init(argc, argv, "lidar_odom_node");
    ros::NodeHandle nh;

    std::string imu_frame_id, lidar_frame_id;
    nh.param<std::string>("lidar_frame_id", lidar_frame_id, "/rslidar");
    nh.param<std::string>("imu_frame_id", imu_frame_id, "/imu");

    LidarOdom::ptr lidar_odom_ptr = std::make_shared<LidarOdom>(nh, imu_frame_id, lidar_frame_id); 
    
    ros::Rate r(100);
    while(ros::ok()) { 

        ros::spinOnce();

        lidar_odom_ptr->Exec();

        r.sleep();


    }


    return 0;
}

