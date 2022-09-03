#include "matching/matching.h"

Matching::Matching(ros::NodeHandle& nh, std::string& lidar_frame_id) : 
        nh_(nh),
        lidar_frame_id_(lidar_frame_id), 
        local_map_ptr_(new CloudData::CLOUD()),
        global_map_ptr_(new CloudData::CLOUD()),
        current_scan_ptr_(new CloudData::CLOUD()) {

    std::cout << std::endl << "----------------->>Scan2Matching-Init-------------------"<< std::endl << std::endl;  
// root
    data_path_ = WORK_SPACE_PATH;
// 订阅
    cloudSub_ = nh_.subscribe("synced_cloud", 100, &Matching::getSyncCloudMsgCallBack, this);
    // 注意此数据 是lidar传感器在map下的位姿
    gnssSub_ = nh_.subscribe("synced_gnss_odom", 100, &Matching::getGnssOdomMsgCallBack, this);
// 发布 
    global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_map", 100);
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(global_map_pub_, "/map");

    local_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/local_map", 100);
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(local_map_pub_, "/map");

    current_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/current_scan", 100);
    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(current_scan_pub_, "/map");
  
    laser_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/laser_localization", 100);
    laser_odom_pub_ptr_ = std::make_shared<OdomPublisher>(laser_odom_pub_, "/map", lidar_frame_id_);
    // 发布一个2d位姿
    laser_2dpose_pub_ = nh.advertise<geometry_msgs::Pose2D>("/laser_2d_pose", 100);

    laser_tf_pub_ = std::make_shared<TFConvert>("/map", lidar_frame_id_);
    
// Other Init 
    InitWithConfig();                //初始化配置参数
    InitGlobalMap();                 //加载全局地图进行滤波
    ResetLocalMap(0.0, 0.0, 0.0);    //局部地图分割
}

Matching::~Matching() {
    std::cout << "_________Matching_node free..." << std::endl;
}

void Matching::Exec() {

    // 全局
    if (has_new_global_map_ && global_map_pub_.getNumSubscribers()!=0 ) {
        CloudData::CLOUD_PTR  global_map(new CloudData::CLOUD());
        this->GetGlobalMap(global_map);     // 全局地图显示卡可以适当 稀疏 
        global_map_pub_ptr_->Publish(global_map);       // 发布全局地图
        has_new_global_map_ = false;
    } 
    // 局部
    if (has_new_local_map_ && local_map_pub_.getNumSubscribers()!=0 ) { 
        CloudData::CLOUD_PTR  local_map(new CloudData::CLOUD());
        this->GetLocalMap(local_map);       
        local_map_pub_ptr_->Publish(local_map);
        has_new_local_map_ = false;
    }

    if (!ReadData()) 
        return;

    while (HasData()) {
        if (!ValidData()) 
            continue;
        
        if (UpdateMatching())    //进行匹配      
            PublishData();
    }

} 

bool Matching::UpdateMatching() { 

    if (!has_inited_) {
        // 以第一帧gnss数据去分割一个小地图进行匹配，后续不需要gnss
        // 此处需要注意,是否需要存储建图时的gnss原点
        this->SetGNSSPose(current_gnss_data_.pose); 
    } 

    return this->Update(current_cloud_data_, laser_odometry_);
}

bool Matching::PublishData() {
    // 雷达定位tf数据
    laser_tf_pub_->SendTransform(laser_odometry_, current_cloud_data_.time);
    // 雷达匹配定位话题数据
    laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);
    // 当前匹配帧
    current_scan_pub_ptr_->Publish(current_scan_ptr_);

    // 2d位姿发布
    geometry_msgs::Pose2D pose2d;
    pose2d.x = laser_odometry_(0,3);
    pose2d.y =  laser_odometry_(1,3);
    pose2d.theta = 0; 
    laser_2dpose_pub_.publish(pose2d); 

}

void Matching::getSyncCloudMsgCallBack(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
    CloudData cloud_data; 
    cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    //ros格式数据转为pcl格式
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));   // cloud_msg_ptr 转换为 cloud_data.cloud_ptr 

    new_cloud_data_.push_back(cloud_data); 
}

void Matching::getGnssOdomMsgCallBack(const nav_msgs::OdometryConstPtr& odom_msg_ptr) {
    PoseData pose_data;
    pose_data.time = odom_msg_ptr->header.stamp.toSec();

    //set the position
    pose_data.pose(0,3) = odom_msg_ptr->pose.pose.position.x;
    pose_data.pose(1,3) = odom_msg_ptr->pose.pose.position.y;
    pose_data.pose(2,3) = odom_msg_ptr->pose.pose.position.z;

    Eigen::Quaternionf q;
    q.x() = odom_msg_ptr->pose.pose.orientation.x;
    q.y() = odom_msg_ptr->pose.pose.orientation.y;
    q.z() = odom_msg_ptr->pose.pose.orientation.z;
    q.w() = odom_msg_ptr->pose.pose.orientation.w;
    pose_data.pose.block<3,3>(0,0) = q.matrix(); 

    new_pose_data_.push_back(pose_data);
}

template<class T1, class T2>
void Matching::ParseData(std::deque<T1>& callback_buff, std::deque<T2>& operate_buff) {

    if (callback_buff.size() > 0){   
        operate_buff.insert(operate_buff.end(), callback_buff.begin(), callback_buff.end());  
        callback_buff.clear();
    } 
}

bool Matching::ReadData() { 
    std::lock_guard<std::mutex> LockGuard(buff_mutex_); 
    this->ParseData(new_cloud_data_, cloud_data_buff_);
    this->ParseData(new_pose_data_, gnss_data_buff_);

    return true;
} 

bool Matching::HasData() {

    if (cloud_data_buff_.size()==0)
        return false; 

    if (has_inited_)
        return true;

    if (gnss_data_buff_.size()==0)      // 一定放在最后判断  因为初始化完成后，就不需要gnss了，此条件不重要了
        return false;


    return true;
}

bool Matching::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();

    // gnss数据已经给到了局部地图初始化，后续不需要gnss数据，不用再对齐了
    if (has_inited_) {
        cloud_data_buff_.pop_front();
        gnss_data_buff_.clear();
        return true;
    }

    // gnss数据还未使用，执行对齐
    current_gnss_data_ = gnss_data_buff_.front();

    double diff_time = current_cloud_data_.time - current_gnss_data_.time;
    if (diff_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_time > 0.05) {
        gnss_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;
}

bool Matching::InitWithConfig() { 

    std::string cfp = data_path_ + "/matching/config/matching.yaml";

    YAML::Node cn = YAML::LoadFile(cfp); 

    // 地图存放路径
    map_path_ = cn["map_path"].as<std::string>(); 
    // 读取匹配方式 
    InitRegistrationMethod(registration_ptr_, cn);
    // 获取滤波方式
    InitFilter("global_map", global_map_filter_ptr_, cn);
    InitFilter("local_map", local_map_filter_ptr_, cn); 
    InitFilter("frame", frame_filter_ptr_, cn); 
    // 滑动窗口分割
    box_filter_ptr_ = std::make_shared<BoxFilter>(cn);

    return true;

}

/**
 * registration_ptr： 匹配基类
 * config_node： yaml句柄
 */
bool Matching::InitRegistrationMethod(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    std::cout << "Map_Matching-Method: " << registration_method << std::endl;

    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } else if (registration_method == "ICP") {
        registration_ptr = std::make_shared<ICPRegistration>(config_node[registration_method]);  
    }
    else {
        ROS_ERROR("Not Found Map-Matching-Method: %s", registration_method.c_str());
        return false;
    }

    return true;
}
/**
 * filter_user： 对哪个滤波
 * filter_ptr：  滤波基类
 * config_node： yaml句柄
*/
bool Matching::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    std::cout << "[Map_Matching for]: " << filter_user << " " 
            << "[Filter_Method]: " << filter_mothod << std::endl;

    if (filter_mothod == "voxel_filter") { 
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);    //config_node[voxel_filter][global_map]
    } else if (filter_mothod == "no_filter") {
        // filter_ptr = std::make_shared<NoFilter>();
        ROS_ERROR("Filter-Method:[no_filter] doesn't exist this no_filter.cpp, you can do it!!!");    // 暂时未写
    } else {

        ROS_ERROR("Not Found Filter-Method: [%s]", filter_mothod.c_str());
        return false;
    }

    return true;
}

/***
 * scan-map 
 * 流程：去除无效点->当前帧滤波->匹配->利用匹配结果，转换滤波前的当前帧
 * input:  当前帧
 * output： 当前帧位姿
 * frame0：当前帧
 * frame1：下一帧
 * frame_-1：上一帧
 * 每一帧估计出来都是相对于map的位姿
 ***/
bool Matching::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose) {

    std::vector<int> indices;   // 存储保留点云的序列号
    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();   // 帧间位姿   
    static Eigen::Matrix4f last_pose = init_pose_;      //init_pose_ 由初始时刻gnss获取得到
    static Eigen::Matrix4f predict_pose = init_pose_;   //ndt匹配的初值
    if (!has_inited_) {    
        predict_pose = current_gnss_pose_;       
    }
    //注意：cloud_in->is_dense = true（意思：该点云为密集，不存在无效点云） 的点云不能使用removeNaNFromPointCloud去除无效点
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *cloud_data.cloud_ptr, indices);  // 去除无效点：输入点云，滤除无效点的点云

    // 帧滤波
    CloudData::CLOUD_PTR  filtered_frame_cloud_ptr(new CloudData::CLOUD()); 
    frame_filter_ptr_->Filter(cloud_data.cloud_ptr, filtered_frame_cloud_ptr); 
    // 存储匹配结果
    CloudData::CLOUD_PTR  result_cloud_ptr(new CloudData::CLOUD());   
    // 只是将滤波后的帧 变换（配准）到了目标点云系下
    registration_ptr_->ScanMatch(filtered_frame_cloud_ptr, predict_pose, result_cloud_ptr, cloud_pose);
    // 源点云(当前帧)配准到目标点云(地图)坐标系下, cloud_pose是 T目标-源 = Tmap-frame0
    pcl::transformPointCloud(*cloud_data.cloud_ptr, *current_scan_ptr_, cloud_pose); 

    // 里程计位姿递推 (*****!!!*****)
    step_pose = last_pose.inverse() * cloud_pose;  // Tframe_-1==frame_0 :=: (Tmap==frame_-1).inverse() * Tmap==frame_0
    // 将step_pose作为下一帧的与当前帧的步进，去估计下一帧的预测值
    // 此处可以改为imu预测
    predict_pose = cloud_pose * step_pose;         // Tmap==frame_1 :=: Tmap==frame_0 * Tframe_0==frame_1
    last_pose = cloud_pose;                        // Tmap==frame_0  

    // 匹配之后判断是否需要更新局部地图
    // 计算当前位置到当前局部地图的x，y，z三个方向的距离，有一个方向小于50，则
    // 以当前位置为原点origin，重新分割出一个局部地图
    std::vector<float> edge = box_filter_ptr_->GetEdge();  
    for (int i = 0; i < 3; i++) { 
        if (fabs(cloud_pose(i, 3) - edge.at(2 * i)) > 50.0 &&
            fabs(cloud_pose(i, 3) - edge.at(2 * i + 1)) > 50.0)
            continue;
        ResetLocalMap(cloud_pose(0,3), cloud_pose(1,3), cloud_pose(2,3)); 
        break;
    }

    return true;
}

bool Matching::InitGlobalMap() { 

    pcl::io::loadPCDFile(map_path_, *global_map_ptr_);    // 加载
    std::cout << "     raw global map size: " << global_map_ptr_->points.size() << std::endl;
    if (global_map_ptr_->points.size()==0)
        return false; 

    local_map_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_);    //地图滤波 
    std::cout << "filtered global map size: " << global_map_ptr_->points.size() << std::endl;

    has_new_global_map_ = true;     // 拿到全局地图  

    return true;    
}

bool Matching::SetGNSSPose(const Eigen::Matrix4f& gnss_pose) { 
    current_gnss_pose_ = gnss_pose; 

    static int gnss_cnt = 0; 
    if (gnss_cnt == 0) {            // 第一帧gnss数据拿去分割局部地图，等待一会儿

        init_pose_ = gnss_pose;     // gnss位姿作为分割区域的中心点
        ResetLocalMap(init_pose_(0,3), init_pose_(1,3), init_pose_(2,3));   //分割出局部地图

    } else if (gnss_cnt > 3) { 
        has_inited_ = true;     // gnss数据已经给到了局部地图初始化
    } 
    gnss_cnt ++; 
    return true; 
}

/**
 * 分割局部地图 
*/
bool Matching::ResetLocalMap(float x, float y, float z) {

    if (!has_new_global_map_) 
        return false;

    std::vector<float> origin = {x, y, z};
    box_filter_ptr_->SetOrigin(origin);
    box_filter_ptr_->Filter(global_map_ptr_, local_map_ptr_);   // 从全局分割局部地图
    // 局部地图，作为目标点云，后续进行scan-to-map的匹配，在Update()函数中
    registration_ptr_->SetInputTarget(local_map_ptr_);       
    has_new_local_map_ = true;  

    std::vector<float> edge = box_filter_ptr_->GetEdge();

    // std::cout  << "new local map: [" << edge.at(0) << " " <<  edge.at(2) << " " << edge.at(4) << "] ["               \
                                    <<  edge.at(1) << " " <<  edge.at(3) << " " << edge.at(5) << "]" << std::endl;
    std::cout << "new local map generate..." << std::endl << std::endl;

    return true;
}

void Matching::GetGlobalMap(CloudData::CLOUD_PTR& global_map) {
    global_map_filter_ptr_->Filter(global_map_ptr_, global_map); 
}

void Matching::GetLocalMap(CloudData::CLOUD_PTR& global_map) {
    global_map = local_map_ptr_;
}


