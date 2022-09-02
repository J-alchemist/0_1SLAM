/**
 * 2022/08/24/晚 by GuoJian
 * 以gnss初始化后的位姿为起点，后续基于此位置递推匹配
*/
#include "matching/matching.h"

int main(int argc, char** argv) { 

    ros::init(argc, argv, "matching_node");
    ros::NodeHandle nh;

    std::string lidar_frame_id;
    nh.param<std::string>("lidar_frame_id", lidar_frame_id, "/rslidar"); 
    Matching::ptr match_ptr = std::make_shared<Matching>(nh, lidar_frame_id); 

    ros::Rate r(100);
    while(ros::ok()) { 

        ros::spinOnce();

        match_ptr->Exec();

        r.sleep();
    }


    return 0;
}

