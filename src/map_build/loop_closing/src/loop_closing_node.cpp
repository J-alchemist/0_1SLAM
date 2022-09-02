/**
 * 2022/08/27 by GuoJian
 * 回环检测
*/

#include "loop_closing/loop_closing.h"

int main(int argc, char** argv) { 

    ros::init(argc, argv, "loop_closing_node");
    ros::NodeHandle nh;

    std::string imu_frame_id, lidar_frame_id;
    nh.param<std::string>("lidar_frame_id", lidar_frame_id, "/rslidar");
    nh.param<std::string>("imu_frame_id", imu_frame_id, "/imu");

    LoopClosing::ptr loop_closing_ptr = std::make_shared<LoopClosing>(nh, imu_frame_id, lidar_frame_id); 

    ros::Rate r(100);
    while(ros::ok()) { 

        ros::spinOnce();

        loop_closing_ptr->Exec();

        r.sleep();

    }

    return 0;
}
