#include "tools/box_filter.h"

BoxFilter::BoxFilter(YAML::Node node) {
    size_.resize(6);
    edge_.resize(6);
    origin_.resize(3); 

    for (size_t i = 0; i < size_.size(); i++) {
        size_.at(i) = node["box_filter_size"][i].as<float>();
    }
    SetSize(size_);     //计算大小
}

/***
 * 实际上是进行局部分割
 * */
bool BoxFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr,
                       CloudData::CLOUD_PTR& output_cloud_ptr) {
    output_cloud_ptr->clear();
    //设置窗口大小 两个对角点 每个点用4维表示
    pcl_box_filter_.setMin(Eigen::Vector4f(edge_.at(0), edge_.at(2), edge_.at(4), 1.0e-6)); 
    pcl_box_filter_.setMax(Eigen::Vector4f(edge_.at(1), edge_.at(3), edge_.at(5), 1.0e6)); 
    pcl_box_filter_.setInputCloud(input_cloud_ptr);     //根据以上尺寸，分离出局部小地图
    pcl_box_filter_.setNegative(false);                 //false: 保留立方体内   true: 保留外
    pcl_box_filter_.filter(*output_cloud_ptr); 

    return true;  
}

void BoxFilter::SetSize(std::vector<float> size) { 
    size_ = size;
    std::cout << "Box Filter Size: " << std::endl 
              << "min_x: " << size.at(0) << ", "
              << "max_x: " << size.at(1) << ", "
              << "min_y: " << size.at(2) << ", "
              << "max_y: " << size.at(3) << ", "
              << "min_z: " << size.at(4) << ", "
              << "max_z: " << size.at(5)
              << std::endl << std::endl;
    
    CalculateEdge();
}

void BoxFilter::SetOrigin(std::vector<float> origin) {
    origin_ = origin;
    CalculateEdge();
}

/**
 * 以origin的为立方体中心点，分割 
 **/
void BoxFilter::CalculateEdge() { 

    for (size_t i = 0; i < origin_.size(); ++i) {  
        //size:  0 2 4  +  origin: 0 1 2  ==  Pmin
        edge_.at(2 * i) = size_.at(2 * i) + origin_.at(i);  
        //size:  1 3 5  +  origin: 0 1 2  ==  Pmax
        edge_.at(2 * i + 1) = size_.at(2 * i + 1) + origin_.at(i); 
    } 
} 

std::vector<float> BoxFilter::GetEdge() {
    return edge_;
}