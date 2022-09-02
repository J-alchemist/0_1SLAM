#ifndef __CLOUD_TYPE_H
#define __CLOUD_TYPE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class CloudData {  
    public:
        using POINT = pcl::PointXYZ;    //using取别名
        using CLOUD = pcl::PointCloud<POINT>;
        using CLOUD_PTR = CLOUD::Ptr;
        
    public:
        CloudData(); 
        ~CloudData(); 
        
    public:
        CLOUD_PTR cloud_ptr;   //等价  pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_ptr
        double time = 0.0;    
};


#endif
