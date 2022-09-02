#include "tools/ndt_registration.h"

NDTRegistration::NDTRegistration(const YAML::Node& node)    
    :ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {
    
    float res = node["res"].as<float>();
    float step_size = node["step_size"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps, int max_iter)     
    :ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

bool NDTRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter) {
    ndt_ptr_->setResolution(res);
    ndt_ptr_->setStepSize(step_size);
    ndt_ptr_->setTransformationEpsilon(trans_eps);
    ndt_ptr_->setMaximumIterations(max_iter);

    std::cout << "NDT-Param: " << std::endl
              << "res: " << res << ", "
              << "step_size: " << step_size << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter 
              << std::endl << std::endl; 

    return true;
}

bool NDTRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    ndt_ptr_->setInputTarget(input_target);

    return true;
}

//执行匹配
//输入当前帧
//预测位姿（匹配算法需要一个良好的初值），该初值第一次由gnss提供，后续由帧间估计的结果提供
//配准后的结果点云：从源转换到目标点云系下，存储配准变换后的源点云
//位姿矩阵
bool NDTRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) { 
    ndt_ptr_->setInputSource(input_source);            //给出源点云
    ndt_ptr_->align(*result_cloud_ptr, predict_pose);    //result_cloud_ptr：结果点云，predict_pose：初值
    result_pose = ndt_ptr_->getFinalTransformation();       //获取位姿变换：源到目标的坐标变换矩阵 

    return true; 
} 
 
float NDTRegistration::GetFitnessScore() { 
    return ndt_ptr_->getFitnessScore(); 
}
