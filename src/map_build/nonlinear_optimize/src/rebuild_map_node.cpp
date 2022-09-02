/**
 * 2022/08/31 by GuoJian
 * 后端建图
*/
#include "nonlinear_optimize/rebuild_map.h"

int main(int argc, char** argv) {  

    ros::init(argc, argv, "rebuild_map_node");
    ros::NodeHandle nh;

    std::string imu_frame_id, lidar_frame_id;
    nh.param<std::string>("lidar_frame_id", lidar_frame_id, "/rslidar");
    nh.param<std::string>("imu_frame_id", imu_frame_id, "/imu");

    RebuildMap::ptr rebuild_map_ptr = std::make_shared<RebuildMap>(nh, imu_frame_id, lidar_frame_id); 

    // rosservice call /save_map
   
    ros::Rate r(100);
    while(ros::ok()) { 

        ros::spinOnce();

        rebuild_map_ptr->Exec();

        r.sleep();
    }

    return 0;
}

