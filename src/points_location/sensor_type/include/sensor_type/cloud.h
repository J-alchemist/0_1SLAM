#ifndef __CLOUD_TYPE
#define __CLOUD_TYPE

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class CloudData {  
    public:
        using POINT = pcl::PointXYZ;    //using取别名
        using CLOUD = pcl::PointCloud<POINT>;
        using CLOUD_PTR = CLOUD::Ptr;
        CLOUD_PTR cloud_ptr;            //CLOUD_PTR 等价  pcl::PointCloud<pcl::PointXYZ>::Ptr
        
    public:
        CloudData(); 
        ~CloudData(); 

    public:
        double time = 0.0;
    
};


#endif