#include "nonlinear_optimize/rebuild_map.h"

RebuildMap::RebuildMap(ros::NodeHandle& nh, std::string& imu_frame_id, 
                                std::string& lidar_frame_id) : 
                                nh_(nh) ,
                                lidar_frame_id_(lidar_frame_id), 
                                imu_frame_id_(imu_frame_id) {

    data_path_ = WORK_SPACE_PATH;
// 订阅
    cloud_sub_ = nh_.subscribe("/synced_cloud", 100, &RebuildMap::syncCloudMsgCallBack, this);
    transformed_odom_sub_ = nh_.subscribe("/transformed_laser_odom", 100, &RebuildMap::laserOdomCallBack, this);
    key_frame_sub_ = nh_.subscribe("/key_frame", 100, &RebuildMap::keyFrameMsgCallback, this);
    optimized_frames_pose_sub_ = nh_.subscribe("/optimized_key_frames", 100, &RebuildMap::optimizedKeyFrameMsgCallback, this);
// 发布
    optimized_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/optimized_laser_odom", 100);
    optimized_odom_pub_ptr_ = std::make_shared<OdomPublisher>(optimized_odom_pub_, "/map", lidar_frame_id_);
    
    global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_map", 100);
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(global_map_pub_, "/map");

    local_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/local_map", 100);
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(local_map_pub_, "/map");

    current_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/current_scan", 100);
    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(current_scan_pub_, "/map");

// server
    saveMap_Server_ = nh_.advertiseService("/save_map", &RebuildMap::saveMapCallBack, this);

// init
    InitWithConfig();
}

RebuildMap::~RebuildMap() { 

    std::cout << "__________Rebuild_Map node free..." << std::endl; 
}

bool RebuildMap::saveMapCallBack(custom_msgs::saveMap::Request& req, 
                                custom_msgs::saveMap::Response& res) {
    need_save_map_ = true;
    res.succeed = need_save_map_;     // 保存地图             
    return res.succeed;
}

void RebuildMap::keyFrameMsgCallback(const custom_msgs::poseWithIndex::ConstPtr& msg_ptr) {

    KeyFrame key_frame;
    key_frame.time = msg_ptr->header.stamp.toSec();
    key_frame.index = msg_ptr->current_frame_index;

    key_frame.pose(0,3) = msg_ptr->position.x;
    key_frame.pose(1,3) = msg_ptr->position.y;
    key_frame.pose(2,3) = msg_ptr->position.z;

    Eigen::Quaternionf q;
    q.x() = msg_ptr->orientation.x;
    q.y() = msg_ptr->orientation.y;
    q.z() = msg_ptr->orientation.z;
    q.w() = msg_ptr->orientation.w;
    key_frame.pose.block<3,3>(0,0) = q.matrix();

    new_key_frame_buff_.push_back(key_frame);
}

void RebuildMap::laserOdomCallBack(const nav_msgs::OdometryConstPtr& odom_msg_ptr) {

    // ROS_INFO("________topic___In");
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

    new_transformed_odom_buff_.push_back(pose_data);
}

void RebuildMap::syncCloudMsgCallBack(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
    CloudData cloud_data; 
    cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    //ros格式数据转为pcl格式
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));   // cloud_msg_ptr 转换为 cloud_data.cloud_ptr 

    new_cloud_data_buff_.push_back(cloud_data); 
}

void RebuildMap::optimizedKeyFrameMsgCallback(const nav_msgs::Path::ConstPtr& key_frames_msg_ptr) {

    for (size_t i = 0; i < key_frames_msg_ptr->poses.size(); i++) {
        KeyFrame key_frame;
        key_frame.time = key_frames_msg_ptr->poses.at(i).header.stamp.toSec();
        key_frame.index = (unsigned int)i;

        key_frame.pose(0,3) = key_frames_msg_ptr->poses.at(i).pose.position.x;
        key_frame.pose(1,3) = key_frames_msg_ptr->poses.at(i).pose.position.y;
        key_frame.pose(2,3) = key_frames_msg_ptr->poses.at(i).pose.position.z;

        Eigen::Quaternionf q;
        q.x() = key_frames_msg_ptr->poses.at(i).pose.orientation.x;
        q.y() = key_frames_msg_ptr->poses.at(i).pose.orientation.y;
        q.z() = key_frames_msg_ptr->poses.at(i).pose.orientation.z;
        q.w() = key_frames_msg_ptr->poses.at(i).pose.orientation.w;
        key_frame.pose.block<3,3>(0,0) = q.matrix();

        new_optimized_key_frames_.push_back(key_frame);
    }

}

void RebuildMap::Exec() {

    if (!ReadData()) 
        return;

    if (need_save_map_) {       // save map
        this->SaveMap();
        need_save_map_ = false;
        return;
    }
    /*****step1****/
    while(HasData()) {
        // 3
        if (ValidData()) {
            this->UpdateWithNewKeyFrame(key_frame_buff_, 
                                        current_transformed_odom_, 
                                        current_cloud_data_
                                        );
            // 发布当前scan和局部地图 & 优化后的里程计
            this->PublishLocalData();
        }
    }
    /*****step2****/
    // +1 
    //有优化位姿产生 optimized_key_frames_
    if (optimized_key_frames_.size()>0) { 
        final_optimized_key_frames_ = optimized_key_frames_;    // 存储最后一次的优化数据
        this->UpdateWithOptimizedKeyFrames();
        // 发布全局地图 
        this->PublishGlobalData();
    }

    return;
}

template<class T1,class T2>
void RebuildMap::ParseData(std::deque<T1>& callback_buff, std::deque<T2>& operate_buff) {

    if (callback_buff.size() > 0){    // 点云
        operate_buff.insert(operate_buff.end(), callback_buff.begin(), callback_buff.end());  
        callback_buff.clear();
    } 
}
// 上锁操作
bool RebuildMap::ReadData() { 

    std::lock_guard<std::mutex> LockGuard(buff_mutex_);

    optimized_key_frames_.clear();
    this->ParseData(new_optimized_key_frames_, optimized_key_frames_);
    new_optimized_key_frames_.clear(); 
    
    this->ParseData(new_cloud_data_buff_, cloud_data_buff_);
    this->ParseData(new_transformed_odom_buff_, transformed_odom_buff_);
    this->ParseData(new_key_frame_buff_, key_frame_buff_);

    return true;
}

bool RebuildMap::HasData() { 
   // 点云和激光里程计
    if (!transformed_odom_buff_.size() || !cloud_data_buff_.size())
        return false;

    return true;
}

// key_frame和transformed_odom的位姿都是对齐到了gnss的
// 对齐到gnss原点的里程计 & 点云
bool RebuildMap::ValidData() { 
 
    current_cloud_data_ = cloud_data_buff_.front();
    current_transformed_odom_ = transformed_odom_buff_.front();

    double diff_odom_time = current_cloud_data_.time - current_transformed_odom_.time;
    if (diff_odom_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_odom_time > 0.05) {
        transformed_odom_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    transformed_odom_buff_.pop_front();   

    return true;     
} 

// 3
// 关键帧位姿(对齐到了gnss原点)
// transformed_data转到gnss坐标系的前端里程计
// 点云旋转之后: 对齐gnss坐标系
// 核心: pose_to_optimize_,另一个函数更新: Tmap优化_mapG
bool RebuildMap::UpdateWithNewKeyFrame(std::deque<KeyFrame>& new_key_frames,
                                   PoseData transformed_data,
                                   CloudData cloud_data) {
    has_new_local_map_ = false;

    if (new_key_frames.size() > 0) {
        KeyFrame key_frame;
        for (size_t i = 0; i < new_key_frames.size(); ++i) {
            key_frame = new_key_frames.at(i);
            key_frame.pose = pose_to_optimize_ * key_frame.pose;    // 1\对齐gnss坐标系里程计优化
            all_key_frames_.push_back(key_frame);   // push
        }
        new_key_frames.clear();
        has_new_local_map_ = true;
    }

    // 将里程计位姿优化 此处的key_frame.pose 与 optimized_odom_.pose 是一样的,可以直接使用
    optimized_odom_ = transformed_data;
    optimized_odom_.pose = pose_to_optimize_ * optimized_odom_.pose;    // 2\对齐gnss坐标系里程计优化

    optimized_cloud_ = cloud_data;
    pcl::transformPointCloud(*cloud_data.cloud_ptr, *optimized_cloud_.cloud_ptr, optimized_odom_.pose);

    return true;
}

// 1
// input:优化后的位姿
// 更新了 pose_to_optimize_
bool RebuildMap::UpdateWithOptimizedKeyFrames() {
    has_new_global_map_ = false;
    
    if (optimized_key_frames_.size() > 0) {
        OptimizeKeyFrames();        // core
        has_new_global_map_ = true;
    }

    return has_new_global_map_;
}
// all_key_frames_.at(i).index 是 关键帧的index,实际就是deque的大小依次递增的
// optimized_key_frames_.at(i).index 是加入了图优化的所有顶点(关键帧)的优化位姿
// 后端里所有的关键帧都加入了优化
bool RebuildMap::OptimizeKeyFrames() {
    size_t optimized_index = 0;
    size_t all_index = 0;

    // 将all_key_frames_中替换为优化后的位姿
    while (optimized_index < optimized_key_frames_.size() && all_index < all_key_frames_.size()) {

        if (optimized_key_frames_.at(optimized_index).index < all_key_frames_.at(all_index).index) {
            optimized_index ++;
        } else if (optimized_key_frames_.at(optimized_index).index > all_key_frames_.at(all_index).index) {
            all_index ++;

        } else { 
            // 更新 pose_to_optimize_
            // Tmap优化_frame * Tframe_mapG = Tmap优化_mapG
            // pose_to_optimize_ 保存的是当前优化顶点的最后一个顶点的优化位姿
            pose_to_optimize_ = optimized_key_frames_.at(optimized_index).pose * all_key_frames_.at(all_index).pose.inverse();
            all_key_frames_.at(all_index) = optimized_key_frames_.at(optimized_index);  // 优化后的位姿,作为关键帧的位姿
            optimized_index ++;
            all_index ++;
        }
    }
    // 将all_key_frames_中未在图优化中的的关键帧,按照最后一帧优化帧的位姿,进行旋转
    while (all_index < all_key_frames_.size()) {
        all_key_frames_.at(all_index).pose = pose_to_optimize_ * all_key_frames_.at(all_index).pose;
        all_index ++;
    }

    return true;
}


bool RebuildMap::JointCloudMap(const std::deque<KeyFrame>& key_frames, CloudData::CLOUD_PTR& map_cloud_ptr) {
    map_cloud_ptr.reset(new CloudData::CLOUD());

    CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
    std::string file_path = "";

    for (size_t i = 0; i < key_frames.size(); ++i) {   
        file_path = key_frames_path_ + "/key_frame_" + std::to_string(key_frames.at(i).index) + ".pcd";
        pcl::io::loadPCDFile(file_path, *cloud_ptr);
        pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, key_frames.at(i).pose);
        *map_cloud_ptr += *cloud_ptr;
    }
    return true;
}

// 把 all_key_frames_里最近的 local_frame_num_ 个帧配准成局部地图 
bool RebuildMap::JointLocalMap(CloudData::CLOUD_PTR& local_map_ptr) {
    size_t begin_index = 0;
    if (all_key_frames_.size() > (size_t)local_frame_num_)
        begin_index = all_key_frames_.size() - (size_t)local_frame_num_;

    std::deque<KeyFrame> local_key_frames;
    for (size_t i = begin_index; i < all_key_frames_.size(); ++i) {
        local_key_frames.push_back(all_key_frames_.at(i));
    }

    JointCloudMap(local_key_frames, local_map_ptr);
    return true;
}

bool RebuildMap::JointGlobalMap(CloudData::CLOUD_PTR& global_map_ptr) { 
    JointCloudMap(optimized_key_frames_, global_map_ptr);
    return true;
}


bool RebuildMap::SaveMap() { 

    if (final_optimized_key_frames_.size() == 0) {
        ROS_ERROR("Don't received last optimized key frames!!!\n");
        return false; 
    }    
        
    // 生成地图
    CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
    JointCloudMap(final_optimized_key_frames_, global_map_ptr);
    // 保存原地图
    std::string map_file_path = map_path_ + "/map.pcd";
    pcl::io::savePCDFileBinary(map_file_path, *global_map_ptr);
    // 保存滤波后地图
    if (global_map_ptr->points.size() > 1000000) {
        std::shared_ptr<VoxelFilter> map_filter_ptr = std::make_shared<VoxelFilter>(0.5, 0.5, 0.5);
        map_filter_ptr->Filter(global_map_ptr, global_map_ptr);
    }
    std::string filtered_map_file_path = map_path_ + "/filtered_map.pcd";
    pcl::io::savePCDFileBinary(filtered_map_file_path, *global_map_ptr);

    std::cout << "Map build over, Map save in: " << map_path_ << std::endl << std::endl;

    return true;

}

void RebuildMap::PublishLocalData() {

    CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());

    // 1-发布当前scan
    frame_filter_ptr_->Filter(optimized_cloud_.cloud_ptr, cloud_ptr);
    current_scan_pub_ptr_->Publish(cloud_ptr);

    // 2-发布优化后的当前位姿
    optimized_odom_pub_ptr_->Publish(optimized_odom_.pose);

    // 3-发布此时的局部地图
    if ( has_new_local_map_ && (local_map_pub_.getNumSubscribers() != 0) ) {
        cloud_ptr.reset(new CloudData::CLOUD());
        JointLocalMap(cloud_ptr);
        local_map_filter_ptr_->Filter(cloud_ptr, cloud_ptr);
        local_map_pub_ptr_->Publish(cloud_ptr);
    }
}

void RebuildMap::PublishGlobalData() {

    if ( has_new_global_map_ && (global_map_pub_.getNumSubscribers() != 0) ) {
        CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
        JointGlobalMap(cloud_ptr);
        global_map_filter_ptr_->Filter(cloud_ptr, cloud_ptr);
        global_map_pub_ptr_->Publish(cloud_ptr);
    }
}

bool RebuildMap::InitWithConfig() {
    std::string cfp = data_path_ + "/map_build/nonlinear_optimize/config/rebuild_map.yaml";
    YAML::Node cn = YAML::LoadFile(cfp);

    std::cout << std::endl << "----------------->>Rebuild-Map-Init-------------------"<< std::endl << std::endl;  
    InitParam(cn);
    InitDataPath(cn);
    InitFilter("frame", frame_filter_ptr_, cn);
    InitFilter("local_map", local_map_filter_ptr_, cn);
    InitFilter("global_map", global_map_filter_ptr_, cn);

    return true;
}

bool RebuildMap::InitParam(const YAML::Node& config_node) {
    local_frame_num_ = config_node["local_frame_num"].as<int>();
    return true;
}

bool RebuildMap::InitDataPath(const YAML::Node& config_node) {
// 硬盘关键帧存放位姿
    key_frames_path_ = data_path_ + "/mapsData/key_frames";
// 全局点云地图存放位置    
    map_path_ = data_path_ + "/mapsData";

    return true;
}

bool RebuildMap::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();

    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else {
        std::cout << "Can't find filter_method: " << filter_mothod << ", for user: " << filter_user << std::endl;
        return false;
    }

    return true;
}



