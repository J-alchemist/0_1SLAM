#include "tools/icp_registration.h"

ICPRegistration::ICPRegistration(const YAML::Node& node)    
    :icp_ptr_(new pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>()) {
    
    float res = node["dis"].as<float>();
    float step_size = node["Eps"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

ICPRegistration::ICPRegistration(float res, float step_size, float trans_eps, int max_iter)     
    :icp_ptr_(new pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>()) {

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

bool ICPRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter) {
   
   //参数与ndt有所不同
    icp_ptr_->setMaxCorrespondenceDistance(res);
    icp_ptr_->setEuclideanFitnessEpsilon(step_size);
    icp_ptr_->setTransformationEpsilon(trans_eps);
    icp_ptr_->setMaximumIterations(max_iter);

    std::cout << "ICP-Param: " << std::endl
              << "dis: " << res << ", "
              << "Eps: " << step_size << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter 
              << std::endl << std::endl; 

    return true;
}

bool ICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    icp_ptr_->setInputTarget(input_target);

    return true;
}

bool ICPRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    icp_ptr_->setInputSource(input_source);
    icp_ptr_->align(*result_cloud_ptr, predict_pose);    
    result_pose = icp_ptr_->getFinalTransformation();

    return true;
}

float ICPRegistration::GetFitnessScore() {
    return icp_ptr_->getFitnessScore();     
}
