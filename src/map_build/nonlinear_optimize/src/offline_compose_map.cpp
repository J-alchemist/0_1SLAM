#include <iostream>  
#include <fstream> 
#include <Eigen/Dense>  
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include "tools/voxel_filter.h"
#include "tools/points_filter_interface.h"
#include "sensor_type/cloud.h"
/**
 * 在线配准地图内存不够,采用离线构建优化完成的地图
 *    what():  std::bad_alloc
*/

// 全局变量
std::shared_ptr<VoxelFilter>  map_filter_ptr_;
CloudData::CLOUD_PTR  global_map_ptr_;
std::string key_frames_path_;
std::string optimize_pose_path_;

// 声明
bool ComposePoseWithPoints();

// main
int main(int argc, char** argv) { 
	std::cout << "\n_________Start_Offiline_Compose_Map________\n" << std::endl;
    // 1-init
    map_filter_ptr_ = std::make_shared<VoxelFilter>(0.5, 0.5, 0.5);
    global_map_ptr_.reset(new CloudData::CLOUD());
    key_frames_path_ = "/home/gj/Desktop/0_1SLAM/src/mapsData/key_frames";
    optimize_pose_path_ = "/home/gj/Desktop/0_1SLAM/src/mapsData/trajectory/optimized.txt";

    // 2-start
    ComposePoseWithPoints();

    return 0;
}


bool ComposePoseWithPoints() { 
    static size_t now_frame_id = 0;
    static bool start_compose_flag = false;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
	std::vector<double> vec;
    // 文件读取 
	std::ifstream datas(optimize_pose_path_);
	if(!datas.is_open()) {
		std::cout << "open falied\n";
	}

	double data;
	while (datas >> data) { 
        // 组合出一个姿态
        vec.push_back(data);
        if (vec.size() == 12) {
            pose = Eigen::Matrix4d::Identity();

            for (int i = 0, k=0; i < 3; ++i) {
                for (int j = 0; j < 4; ++j) {
                   pose(i, j) =  vec.at(k++);
                }
            }
            start_compose_flag = true;
            vec.clear();
        }
        // 配准
        if (start_compose_flag) {

            CloudData::CLOUD_PTR  frame_cloud_ptr;
            frame_cloud_ptr.reset(new CloudData::CLOUD());
            std::string fp = key_frames_path_ + "/key_frame_" + std::to_string(now_frame_id) + ".pcd";
            pcl::io::loadPCDFile(fp, *frame_cloud_ptr);
            pcl::transformPointCloud(*frame_cloud_ptr, *frame_cloud_ptr, pose);
            *global_map_ptr_ += *frame_cloud_ptr;
            now_frame_id++;
            start_compose_flag = false;

            if (now_frame_id%100==0)
                std::cout << "already compose points map: " << now_frame_id << std::endl;
        }
        if (now_frame_id==1200) 	// 偶滴笔记本内存不够只能配准到1200帧
            break;
	
    } 
    datas.close();

    // 保存原地图
    std::string fp = key_frames_path_ + "/../map.pcd";
    pcl::io::savePCDFileBinary(fp, *global_map_ptr_);
    // 保存滤波后地图
    if (global_map_ptr_->points.size() > 1000000) { 
        map_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_);
    }
    fp = key_frames_path_ + "/../filtered_map.pcd";
    pcl::io::savePCDFileBinary(fp, *global_map_ptr_);

    std::cout << "Map build over, Map save in: " << fp << std::endl << std::endl;		

    return 0; 
} 

