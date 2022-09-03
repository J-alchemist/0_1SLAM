/**
 * 2022/08/24/晚 by GuoJian
 * 以gnss定位分割一个局部地图用于匹配
 * 若从原点开机，可以解耦gnss，初始（0,0,0）进行分割
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

