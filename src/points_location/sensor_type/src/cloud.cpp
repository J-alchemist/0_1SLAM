#include "sensor_type/cloud.h"


CloudData::CloudData() : cloud_ptr( new CLOUD() ) {     // CLOUD==>  pcl::PointCloud< pcl::PointXYZ >

}

CloudData::~CloudData() {


}

 

