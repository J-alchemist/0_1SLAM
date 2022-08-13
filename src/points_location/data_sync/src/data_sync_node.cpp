#include "data_sync/data_sync.h"


int main(int argc, char **argv){

    ros::init(argc, argv, "data_sync_node");
    ros::NodeHandle nh;
    DataSync::ptr ds = std::make_shared<DataSync>();

    ros::Rate r(10);
    while (ros::ok()) {
        ros::spinOnce();

        r.sleep();
    }

    return 0;
}