#include "data_sync/data_sync.h"

int main(int argc, char **argv){

    ros::init(argc, argv, "data_sync_node");
    
    ros::NodeHandle nh;

    std::string imu_frame_id, lidar_frame_id;
    nh.param<std::string>("lidar_frame_id", lidar_frame_id, "/rslidar");
    nh.param<std::string>("imu_frame_id", imu_frame_id, "/imu");

    std::string Sys_Mode;
    double MyOrigin[3];   // lon lat alt
    nh.param<std::string>("sys_mode", Sys_Mode, "mapping");
    nh.param<double>("origin_lon", MyOrigin[0], 8.39045533533);
    nh.param<double>("origin_lat", MyOrigin[1], 48.9826576154);
    nh.param<double>("origin_alt", MyOrigin[2], 116.39641207);

    DataSync::ptr ds_ptr = std::make_shared<DataSync>(nh, imu_frame_id, lidar_frame_id, Sys_Mode, MyOrigin);

    ros::Rate r(100);
    while (ros::ok()) { 
        ros::spinOnce();

        ds_ptr->Exec();

        r.sleep();
    }

    return 0;
}
