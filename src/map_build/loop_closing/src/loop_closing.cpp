#include "loop_closing/loop_closing.h"

LoopClosing::LoopClosing(ros::NodeHandle &nh, std::string& imu_frame_id, 
                            std::string& lidar_frame_id) : 
                            nh_(nh) {

    data_path_ = WORK_SPACE_PATH;
// 后端发送过来的
    key_frame_sub_ = nh_.subscribe("key_frame", 100, &LoopClosing::keyFrameMsgCallback, this);
    key_gnss_sub_ = nh_.subscribe("key_gnss", 100, &LoopClosing::keyGnssMsgCallback, this);
    // bind函数给回调传参
    // key_frame_sub_ = nh_.subscribe("key_frame", 1000, boost::bind(&LoopClosing::msg_callback, _1, true), this);
// 回环帧 类型LoopPose
    loop_pose_pub_ = nh_.advertise<custom_msgs::poseWithIndex>("loop_pose", 100);
    loop_pose_pub_ptr_ = std::make_shared<LoopPosePublisher>(loop_pose_pub_, "/map");
// Init
    InitWithConfig(); 
}

LoopClosing::~LoopClosing() {   
    std::cout << "__________Loop_Closing_node free..." << std::endl;
}

void LoopClosing::Exec() {

    if (!ReadData())
        return;

    while(HasData()) { 

        if (!ValidData())
            continue;

        Update(current_key_frame_, current_key_gnss_);
        PublishData();
    }
    
    return;
}

void LoopClosing::keyFrameMsgCallback(const custom_msgs::poseWithIndex::ConstPtr& msg_ptr) {
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

    new_key_frame_.push_back(key_frame);
}

void LoopClosing::keyGnssMsgCallback(const custom_msgs::poseWithIndex::ConstPtr& msg_ptr) {
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
   
    new_gnss_frame_.push_back(key_frame);
}

bool LoopClosing::InitWithConfig() { 

    std::string cfp = data_path_ + "/map_build/loop_closing/config/loop_closing.yaml";
    YAML::Node cn = YAML::LoadFile(cfp);

    std::cout << std::endl << "----------------->>Loop-Closing-Init-------------------"<< std::endl << std::endl;  

    InitParamAndPath(cn);
    InitRegistrationMethod(registration_ptr_, cn);
    InitFilter("map", map_filter_ptr_, cn);
    InitFilter("scan", scan_filter_ptr_, cn);

    return true;

}

bool LoopClosing::InitParamAndPath(const YAML::Node& config_node) {
    extend_frame_num_ = config_node["extend_frame_num"].as<int>();
    loop_step_ = config_node["loop_step"].as<int>();
    diff_num_ = config_node["diff_num"].as<int>();
    detect_area_ = config_node["detect_area"].as<float>();
    fitness_score_limit_ = config_node["fitness_score_limit"].as<float>();

    // 关键帧存放路径
    key_frames_path_ = data_path_ + "/mapsData/key_frames";

    return true; 
}


bool LoopClosing::InitRegistrationMethod(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    std::cout << "LoopClosing-Method: " << registration_method << std::endl;

    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } else {
        ROS_ERROR("Not Found LoopClosing-Method: %s", registration_method.c_str());
        return false;
    }

    return true;
}

bool LoopClosing::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    std::cout << "[LoopClosing for]: " << filter_user << " "
                << "[Filter_Method]: " << filter_mothod << std::endl;

    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else {
        ROS_ERROR("Not Found Filter-Method: [%s]", filter_mothod.c_str());
        return false;
    }

    return true;
}

/***
 * 回环检测上层函数
 * */
bool LoopClosing::Update(const KeyFrame key_frame, const KeyFrame key_gnss) {

    has_new_loop_pose_ = false;

    all_key_frames_.push_back(key_frame);    // 存储后端过来的关键帧
    all_key_gnss_.push_back(key_gnss);

    int index = -1;
    if (!DetectNearestKeyFrame(index))      // 历史帧中寻找闭环帧
        return false;

    if (!CloudRegistration(index))          // 生成历史帧局部地图，scan2map
        return false;

    has_new_loop_pose_ = true;

    return true;
}

// 回环检测核心函数
// 每进来一次，过掉一个关键帧
// 传出参数：闭环帧id 
/**
 * 条件1：loop_step_
 * 条件2：diff_num_
 * 条件3：extend_frame_num_  
 * 条件3：detect_area_
 * */ 
bool LoopClosing::DetectNearestKeyFrame(int& index) {

    static int skip_cnt = 0;
    static int skip_num = loop_step_;  // 会自适应更新
    if (++skip_cnt < skip_num)      // 每隔loop_step_个关键帧做一次闭环
        return false; 

    int key_num = all_key_frames_.size() - 1;       // 去除当前帧
    if (key_num  < diff_num_)                   // 当前帧与间隔 diff_num_大的历史帧去搜索   避免小回环无意义
        return false;

    float min_distance = 1000000.0;
    float calc_distance = 0.0; 
    for (size_t i = 0; i < key_num; i++) {      // 当前帧与间隔 diff_num_大的历史帧去搜索
        if (key_num - i < diff_num_)
            break; 

        // 最远的历史帧开始取过来 计算曼哈顿最小距离的历史帧 
        KeyFrame history_key_frame = all_key_gnss_.at(i);      
        calc_distance = fabs(current_key_frame_.pose(0,3) - history_key_frame.pose(0,3)) + 
                        fabs(current_key_frame_.pose(1,3) - history_key_frame.pose(1,3)) + 
                        fabs(current_key_frame_.pose(2,3) - history_key_frame.pose(2,3));
        if (calc_distance < min_distance) {
            min_distance = calc_distance;
            index = i;      // 可能的闭环帧id
        }
    }   
    if (index < extend_frame_num_)  // 可能的闭环帧 前后的历史帧数目不满足 scan2map条件
        return false;    

    skip_cnt = 0;
    
    if (min_distance < detect_area_) {     // 隔了很多帧, 并且两帧距离小
        skip_num = loop_step_;
        return true;
    } else {
        skip_num = std::max((int)(min_distance / 2.0), loop_step_); // 关键帧取得是2m一帧
        return false;
    }
   
    return true;
}

// input: 闭环帧id
bool LoopClosing::CloudRegistration(int closing_id) {

    // 1\回环帧的前后extend_frame_num_个帧拼接, 回环帧组成局部地图(并且已经转到gnss坐标系下) 2\拿出回环帧对应的gnsspose
    CloudData::CLOUD_PTR map_cloud_ptr(new CloudData::CLOUD());
    Eigen::Matrix4f map_pose = Eigen::Matrix4f::Identity();
    JointMap(closing_id, map_cloud_ptr, map_pose);

    // 1\硬盘拿出当前关键帧点云, 2\拿出当前帧对应的gnsspose,作为匹配预测
    CloudData::CLOUD_PTR scan_cloud_ptr(new CloudData::CLOUD());
    Eigen::Matrix4f scan_pose = Eigen::Matrix4f::Identity();
    JointScan(scan_cloud_ptr, scan_pose);

    // 匹配
    CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
    Eigen::Matrix4f result_pose = Eigen::Matrix4f::Identity();
    registration_ptr_->SetInputTarget(map_cloud_ptr);
    // result_pose 当前帧相对于TGmap的位姿,可是为什么???????????????????
    // result_pose : 当前帧与回环小地图的位姿
    registration_ptr_->ScanMatch(scan_cloud_ptr, scan_pose, result_cloud_ptr, result_pose); 

    // 计算相对位姿
    //  Tframe回环_Gmap * TGmap_frame当前 = Tframe回环_frame当前
    //  Tframe回环_Gmap *  T回环map_frame当前
    current_loop_pose_.pose = map_pose.inverse() * result_pose;   

    // fitness_score_limit_  匹配得分 越小越准  
    if (registration_ptr_->GetFitnessScore() > fitness_score_limit_)
        return false;
    
    // 闭环帧数
    static int loop_close_cnt = 0;
    loop_close_cnt ++;

    std::cout << "____already check ClosingFrame times: "<<  loop_close_cnt << std::endl
              << "frame_" << current_loop_pose_.index0 
              << "------>" 
              << "frame_" << current_loop_pose_.index1 << std::endl
              << "fitness score: " << registration_ptr_->GetFitnessScore() 
              << std::endl << std::endl;
                 
    return true;
}

// 生成局部地图，供回环使用
bool LoopClosing::JointMap(int id, CloudData::CLOUD_PTR& map_cloud_ptr, 
                                            Eigen::Matrix4f& map_pose) {

    map_pose = all_key_gnss_.at(id).pose;  // 回环帧（和当前帧位姿最相近的历史帧） 对应的 key_gnss                                
    current_loop_pose_.index0 = all_key_frames_.at(id).index; // 回环帧id

    // TGmap_frame回环 * Tframe回环_Lmap = TGmap_TLmap
    // 得到雷达里程计原点到gnss原点的 坐标变换矩阵(相对关系)
    Eigen::Matrix4f pose_to_gnss = map_pose * all_key_frames_.at(id).pose.inverse(); 

    // 拼接回环小地图
    for (int i = id - extend_frame_num_; i < id + extend_frame_num_; i++) { 
        // 加载硬盘的闭环帧
        std::string fp = key_frames_path_ + "/key_frame_" + std::to_string(all_key_frames_.at(i).index) + ".pcd";
        CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
        pcl::io::loadPCDFile(fp, *cloud_ptr);
        // TGmap_TLmap * TLmap_frame回环 = TGmap_frame回环
        // 小地图位姿 旋转到 gnss原点坐标系 
        Eigen::Matrix4f cloud_pose = pose_to_gnss * all_key_frames_.at(i).pose;
        pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, cloud_pose);

        *map_cloud_ptr += *cloud_ptr;
    }
    map_filter_ptr_->Filter(map_cloud_ptr, map_cloud_ptr);

    return true;
}

// 硬盘加载当前帧的点云，内存里只存在帧的位姿
bool LoopClosing::JointScan(CloudData::CLOUD_PTR& scan_cloud_ptr, Eigen::Matrix4f& scan_pose) {

    scan_pose = current_key_gnss_.pose;                     // 当前帧对应的 key_gnss
    current_loop_pose_.index1 = current_key_frame_.index;   // 当前帧id
    current_loop_pose_.time = current_key_frame_.time;

    std::string fp = key_frames_path_ + "/key_frame_" + std::to_string(current_key_frame_.index) + ".pcd";
    pcl::io::loadPCDFile(fp, *scan_cloud_ptr);
    scan_filter_ptr_->Filter(scan_cloud_ptr, scan_cloud_ptr);

    return true;
} 

bool LoopClosing::PublishData() { 

    if (has_new_loop_pose_) 
        loop_pose_pub_ptr_->Publish(current_loop_pose_);
    
    return true;
}

/********************************************************************************/
template<class T1,class T2>
void LoopClosing::ParseData(std::deque<T1>& callback_buff, std::deque<T2>& operate_buff) {

    if (callback_buff.size() > 0){   
        operate_buff.insert(operate_buff.end(), callback_buff.begin(), callback_buff.end());  
        callback_buff.clear();
    } 
}

bool LoopClosing::ReadData() { 
    
    std::lock_guard<std::mutex> LockGuard(buff_mutex_); 
    this->ParseData(new_key_frame_, key_frame_buff_);
    this->ParseData(new_gnss_frame_, key_gnss_buff_);

    return true;
} 

bool LoopClosing::HasData() {

    if (key_frame_buff_.size()==0)
        return false; 

    if (key_gnss_buff_.size()==0)    
        return false;

    return true;
}

bool LoopClosing::ValidData() {

    current_key_frame_ = key_frame_buff_.front();
    current_key_gnss_ = key_gnss_buff_.front();

    double diff_time = current_key_frame_.time - current_key_gnss_.time;
    if (diff_time < -0.05) { 
        key_frame_buff_.pop_front();
        return false;
    }

    if (diff_time > 0.05) {
        key_gnss_buff_.pop_front();
        return false;
    }

    key_frame_buff_.pop_front();
    key_gnss_buff_.pop_front();

    return true;
}


