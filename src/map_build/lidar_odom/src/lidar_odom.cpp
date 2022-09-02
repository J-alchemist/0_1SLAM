#include "lidar_odom/lidar_odom.h"
/**
 * 以第一帧雷达数据为原点，开始递推
 * 未用gnss数据，此处只是留一个接口
 * */
LidarOdom::LidarOdom(ros::NodeHandle& nh, std::string& imu_frame_id, 
                         std::string& lidar_frame_id) : 
                            nh_(nh), 
                            imu_frame_id_(imu_frame_id),
                            lidar_frame_id_(lidar_frame_id) { 
// root                                                   
    data_path_ = WORK_SPACE_PATH; 

// 订阅
    cloudSub_ = nh_.subscribe("/synced_cloud", 100, &LidarOdom::getSyncCloudMsgCallBack, this);
    // gnssOdomSub_ = nh_.subscribe("/synced_gnss_odom", 100, &LidarOdom::getGnssOdomMsgCallBack, this);
    
// 发布 里程计数据 
    laser_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/front_laser_odom", 100);
    laser_odom_pub_ptr_ = std::make_shared<OdomPublisher>(laser_odom_pub_, "/map", lidar_frame_id_);

// malloc
    // current_scan_ptr_.reset(new CloudData::CLOUD());
    // local_map_ptr_.reset(new CloudData::CLOUD());
    // global_map_ptr_.reset(new CloudData::CLOUD());
    result_cloud_ptr_.reset(new CloudData::CLOUD());

// 初始化配置
    InitWithConfig();
}

LidarOdom::~LidarOdom() { 
    std::cout << "__________Lidar_Odom_node free..." << std::endl;
}

void LidarOdom::getSyncCloudMsgCallBack(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
    CloudData cloud_data; 
    cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    //ros格式数据转为pcl格式
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));   // cloud_msg_ptr 转换为 cloud_data.cloud_ptr 

    new_cloud_data_.push_back(cloud_data); 
}

// void LidarOdom::getGnssOdomMsgCallBack(const nav_msgs::OdometryConstPtr& odom_msg_ptr) {

//     PoseData pose_data;
//     pose_data.time = odom_msg_ptr->header.stamp.toSec();

//     //set the position
//     pose_data.pose(0,3) = odom_msg_ptr->pose.pose.position.x;
//     pose_data.pose(1,3) = odom_msg_ptr->pose.pose.position.y;
//     pose_data.pose(2,3) = odom_msg_ptr->pose.pose.position.z;

//     Eigen::Quaternionf q;
//     q.x() = odom_msg_ptr->pose.pose.orientation.x;
//     q.y() = odom_msg_ptr->pose.pose.orientation.y;
//     q.z() = odom_msg_ptr->pose.pose.orientation.z;
//     q.w() = odom_msg_ptr->pose.pose.orientation.w;
//     pose_data.pose.block<3,3>(0,0) = q.matrix(); 

//     new_pose_data_.push_back(pose_data);
// }

// void LidarOdom::getSyncImuMsgCallBack(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
//     // 未实现
// }

void LidarOdom::Exec() { 

    if (!ReadData())
        return;
    
    while(HasData()) {
       if (!ValidData())
            continue;
    
        // 更新gnss里程计位姿
        // UpdateGNSSOdometry();
        // 更新前端lidar里程计位姿
        if (UpdateLaserOdometry()) {
            PublishData();
            // SaveTrajectory();   // 创建轨迹文件.txt
        }

    }    
    return;
}

template<class T1,class T2>
void LidarOdom::ParseData(std::deque<T1>& callback_buff, std::deque<T2>& operate_buff) {

    if (callback_buff.size() > 0){   
        operate_buff.insert(operate_buff.end(), callback_buff.begin(), callback_buff.end());  
        callback_buff.clear();
    } 
}

bool LidarOdom::ReadData() { 

    std::lock_guard<std::mutex> LockGuard(buff_mutex_); 
    this->ParseData(new_cloud_data_, cloud_data_buff_);
    // this->ParseData(new_pose_data_, gnss_data_buff_);

    return true;
} 
 
bool LidarOdom::HasData() {

    if (cloud_data_buff_.size()==0)
        return false; 

    // if (gnss_data_buff_.size()==0)    
    //     return false;

    return true;
}

bool LidarOdom::ValidData() {
// 使用gnss数据
    // current_cloud_data_ = cloud_data_buff_.front();
    // current_gnss_data_ = gnss_data_buff_.front();
    // double diff_time = current_cloud_data_.time - current_gnss_data_.time;
    // if (diff_time < -0.05) { 
    //     cloud_data_buff_.pop_front();
    //     return false;
    // }
    // if (diff_time > 0.05) {
    //     gnss_data_buff_.pop_front();
    //     return false;
    // }
    // cloud_data_buff_.pop_front();
    // gnss_data_buff_.pop_front();

// 未使用gnss数据
    current_cloud_data_ = cloud_data_buff_.front();
    cloud_data_buff_.pop_front(); 

    return true;
}

bool LidarOdom::UpdateGNSSOdometry() { 
    gnss_odometry_ = current_gnss_data_.pose;
}

bool LidarOdom::UpdateLaserOdometry() {

    static bool first_pose_update = false;
    if (!first_pose_update) {           // 第一帧更新
        first_pose_update = true;
        // init_pose_ = gnss_odometry_;             // 以第一帧gnss姿态开始递推
        init_pose_ = Eigen::Matrix4f::Identity();   // 以第一帧雷达位姿递推
    }

    return this->Update(current_cloud_data_, laser_odometry_);
}

/**
 * inut: 当前帧点云
 * output: 里程计
 * scan2can  ==> scan2map
 * UpdateWithNewKeyFrame: 拼接局部小地图
 * */
bool LidarOdom::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose) {

    current_frame_.cloud_data.time = cloud_data.time;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices);

    // 滤波
    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
    frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr, filtered_cloud_ptr);

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_; 

    static Eigen::Matrix4f last_key_frame_pose = init_pose_;
    cloud_pose = Eigen::Matrix4f::Identity();   // reset
    // 局部地图容器中没有关键帧，代表是第一帧数据，位姿数据就用gnssodom的
    // 此时把当前帧数据作为第一个关键帧，并更新局部地图容器和全局地图容器
    if (local_map_frames_.size() == 0) { 
        current_frame_.pose = init_pose_;  
        cloud_pose = current_frame_.pose;  
        last_key_frame_pose = current_frame_.pose;
        
        UpdateWithNewKeyFrame(current_frame_);     // 局部地图-目标点云：local_map
        return true;
    }
    // 不是第一帧，就正常匹配  
    // 源点云：当前帧
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr_, current_frame_.pose);
    cloud_pose = current_frame_.pose; // 里程计传出
    
    // 更新相邻两帧的相对运动
    step_pose = last_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;
    last_pose = current_frame_.pose;

    // 上一帧与当前帧的 曼哈顿距离
    // 是否更新关键帧
    if (fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) + 
        fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
        fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > key_frame_distance_) {
        UpdateWithNewKeyFrame(current_frame_);
        last_key_frame_pose = current_frame_.pose; 
    }

    return true;
}

/**
 * 关键帧的对应的点云存储到硬盘
 * 关键帧对应的位姿存在global_map_frames_容器,而
 * local_map_frames_同时存储了位姿和点云
 * new_key_frame: 新的关键帧
 * pcl的ptr相当于c++的共享指针shared_ptr
*/
bool LidarOdom::UpdateWithNewKeyFrame(const Frame& new_key_frame) {

    Frame key_frame = new_key_frame;
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
    CloudData::CLOUD_PTR aft_cloud_ptr(new CloudData::CLOUD()); 

    // 把关键帧点云存储到硬盘里，节省内存
    // 路径：mapsData/key_frames
    // 命名：key_frame_xx.pcd
    // std::string path = data_path_ + "/mapsData/key_frames/key_frame_" + std::to_string(global_map_frames_.size()) + ".pcd";
    // pcl::io::savePCDFileBinary(path, *key_frame.cloud_data.cloud_ptr); 

    // 更新局部地图 滑窗
    local_map_frames_.push_back(key_frame); 
    while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_)) {
        local_map_frames_.pop_front();
    }
    local_map_ptr_.reset(new CloudData::CLOUD());
    for (size_t i = 0; i < local_map_frames_.size(); ++i) {
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr, 
                                 *aft_cloud_ptr, 
                                 local_map_frames_.at(i).pose);
        *local_map_ptr_ += *aft_cloud_ptr;
    }
    has_new_local_map_ = true;
    // 更新ndt匹配的目标点云
    // 关键帧数量还比较少的时候不滤波，因为点云本来就不多，太稀疏影响匹配效果
    if (local_map_frames_.size() < 10) {
        registration_ptr_->SetInputTarget(local_map_ptr_);
    } else {
        CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
        local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
        registration_ptr_->SetInputTarget(filtered_local_map_ptr);
    }

    // 保存所有关键帧信息在容器里
    // 存储之前，点云要先释放，因为点云已经存到了硬盘里
    // key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD());
    // global_map_frames_.push_back(key_frame);        // 只保存关键帧的位姿，点云已经存到磁盘

    return true;

}
/*
bool LidarOdom::SaveMap() {

    global_map_ptr_.reset(new CloudData::CLOUD());  //存储拼接好的全局地图

    std::string path = "";
    CloudData::CLOUD_PTR key_frame_cloud_ptr(new CloudData::CLOUD());
    CloudData::CLOUD_PTR aft_cloud_ptr(new CloudData::CLOUD());

    for (size_t i = 0; i < global_map_frames_.size(); ++i) {
        path = data_path_ + "/mapsData/key_frames/key_frame_" + std::to_string(i) + ".pcd";
        
        pcl::io::loadPCDFile(path, *key_frame_cloud_ptr);   // 从头取出关键帧，进行地图连接
        pcl::transformPointCloud(*key_frame_cloud_ptr,      // 都转换到相对于第一帧的位姿进行配准
                                 *aft_cloud_ptr, 
                                 global_map_frames_.at(i).pose);
        *global_map_ptr_ += *aft_cloud_ptr;
    }
    
    path = data_path_ + "/mapsData/map.pcd"; 
    pcl::io::savePCDFileBinary(path, *global_map_ptr_);
    has_new_global_map_ = true; 

    return true;
}
*/

bool LidarOdom::PublishData() {

    // 前端雷达里程计
    laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);

    return true;
}

/*
bool LidarOdom::SaveTrajectory() {
    static std::ofstream ground_truth, laser_odom;
    static bool is_file_created = false;
      
    if (!is_file_created) {

        std::string trajectory_path = data_path_ + "/mapsData/trajectory";

        if (!FileManager::CreateDirectory(trajectory_path))     // 创建 trajectory 文件夹
            return false;
        if (!FileManager::CreateFile(ground_truth, trajectory_path + "/ground_truth.txt"))      // 创建文件
            return false;
        if (!FileManager::CreateFile(laser_odom, trajectory_path + "/front_laser_odom.txt"))
            return false;
        is_file_created = true;
    }

    // evo分析的格式
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ground_truth << gnss_odometry_(i, j);
            laser_odom << laser_odometry_(i, j);
            if (i == 2 && j == 3) {
                ground_truth << std::endl;
                laser_odom << std::endl;
            } else {
                ground_truth << " ";
                laser_odom << " ";
            }
        }
    }

    return true;
}
*/

bool LidarOdom::InitWithConfig() {

    std::string cfp = data_path_ + "/map_build/lidar_odom/config/front_lidar_odom.yaml";
    YAML::Node cn = YAML::LoadFile(cfp);

    std::cout << std::endl << "----------------->>Front-Lidar-Odom-Init-------------"<< std::endl << std::endl;  

    // 1-文件清空及创建(移到后端里面)
    // InitDataPath(cn);
    // 2-匹配
    InitRegistrationMethod(registration_ptr_, cn);
    // 3-滤波
    InitFilter("local_map", local_map_filter_ptr_, cn);
    InitFilter("frame", frame_filter_ptr_, cn);
    // 4-参数
    InitParam(cn); 

    return true;
}


bool LidarOdom::InitParam(const YAML::Node& config_node) {
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    local_frame_num_ = config_node["local_frame_num"].as<int>();

    return true; 
}

bool LidarOdom::InitDataPath(const YAML::Node& config_node) {

    std::string datas_path = data_path_ + "/mapsData"; 

    if (!FileManager::CreateDirectory(datas_path))     // 创建 mapsData文件夹
        return false;

    std::string key_frames_path = datas_path + "/key_frames"; 
    if (!FileManager::CreateDirectory(key_frames_path))     // 创建 key_frames文件夹
        return false;

    return true;
}

bool LidarOdom::InitRegistrationMethod(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    std::cout << "Lidar-Odom-Method: " << registration_method << std::endl;

    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } else {
        ROS_ERROR("Not Found Lidar-Odom-Matching-Method: %s", registration_method.c_str());
        return false;
    }

    return true;
}

bool LidarOdom::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    std::cout << "[Lidar-Odom for]: " << filter_user << " "
                << "[Filter_Method]: " << filter_mothod << std::endl;

    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else {
        ROS_ERROR("Not Found Filter-Method: [%s]", filter_mothod.c_str());
        return false;
    }

    return true;
}

