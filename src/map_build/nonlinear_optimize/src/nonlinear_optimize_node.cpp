/**
 * 2022/08/28 by GuoJian
 * 后端优化
*/
#include "nonlinear_optimize/nonlinear_optimize.h"

int main(int argc, char** argv) {  

    ros::init(argc, argv, "nonlinear_optimize_node");
    ros::NodeHandle nh;

    std::string imu_frame_id, lidar_frame_id;
    nh.param<std::string>("lidar_frame_id", lidar_frame_id, "/rslidar");
    nh.param<std::string>("imu_frame_id", imu_frame_id, "/imu");

    NonlinearOpt::ptr nonlinear_optimize_ptr = std::make_shared<NonlinearOpt>(nh, imu_frame_id, lidar_frame_id); 

    // rosservice call /opimize_map
   
    ros::Rate r(100);
    while(ros::ok()) { 

        ros::spinOnce();

        nonlinear_optimize_ptr->Exec();

        r.sleep();


    }


    return 0;
}

