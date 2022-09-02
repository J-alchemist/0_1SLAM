#include "nonlinear_optimize/nonlinear_optimize.h"

NonlinearOpt::NonlinearOpt(ros::NodeHandle& nh, std::string& imu_frame_id, 
                                std::string& lidar_frame_id) : 
                                nh_(nh) ,
                                lidar_frame_id_(lidar_frame_id), 
                                imu_frame_id_(imu_frame_id) {

// root
    data_path_ = WORK_SPACE_PATH;

// 订阅
    cloudSub_ = nh_.subscribe("/synced_cloud", 100, &NonlinearOpt::SyncCloudMsgCallBack, this);
    gnss_odom_sub_ = nh_.subscribe("/synced_gnss_odom", 100, &NonlinearOpt::GnssOdomMsgCallBack, this);
    laser_odom_sub_ = nh_.subscribe("/front_laser_odom", 100, &NonlinearOpt::FrontLaserOdomCallBack, this);
    loop_pose_sub_ = nh_.subscribe("/loop_pose", 100,  &NonlinearOpt::LoopPoseCallBack, this);

// 发布
    key_frame_pub_ = nh_.advertise<custom_msgs::poseWithIndex>("/key_frame", 100);   // 位姿也是对齐到gnss坐标系的
    key_frame_pub_ptr_ = std::make_shared<KeyFramePublisher>(key_frame_pub_, "/map");

    key_gnss_pub_ = nh_.advertise<custom_msgs::poseWithIndex>("/key_gnss", 100);
    key_gnss_pub_ptr_ = std::make_shared<KeyFramePublisher>(key_gnss_pub_, "/map");

    // 前端里程计 对齐到gnss原点的前端雷达里程计
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/transformed_laser_odom", 100);
    odom_pub_ptr_ = std::make_shared<OdomPublisher>(odom_pub_, "/map", lidar_frame_id_);

    optimized_key_frames_pub_ = nh_.advertise<nav_msgs::Path>("/optimized_key_frames", 100);
    optimized_key_frames_pub_ptr_ = std::make_shared<KeyFramePublisher>(optimized_key_frames_pub_, "/map");

// 服务
    optimizeMap_Server_ = nh_.advertiseService("/optimize_map", &NonlinearOpt::forceOptimizeMapCallBack, this);

// other init
    InitWithConfig();

}

NonlinearOpt::~NonlinearOpt() {
    std::cout << "__________Nonlinear_Optimize_node free..." << std::endl;    
}

bool NonlinearOpt::forceOptimizeMapCallBack(custom_msgs::optimizeMap::Request& req, 
                                custom_msgs::optimizeMap::Response& res) {
    last_optimize_map_ = true;   
    res.succeed = last_optimize_map_;  // 强制优化
    return res.succeed; 
}

void NonlinearOpt::Exec() { 

    if (last_optimize_map_ ) {    // 执行最后数据优化,表示建图已经完成,发布最后的优化数据
        ForceOptimize();
        
        std::deque<KeyFrame> buff; 
        PoseTypeMatrix2KeyFrame(buff, optimized_pose_); 
        optimized_key_frames_pub_ptr_->Publish(buff);   // 发布   
        has_new_optimized_ = false;
        last_optimize_map_ = false;
        return;
    }

    if (!ReadData())
        return;

    IsInsertLoopPose();        // 添加回环到优化器

    while(HasData()) { 

        if(!ValidData())
            continue;
        UpdateBackEnd();      // 添加gnss观测到优化器, 关键帧保存. 优化后位姿的获取
        PublishData();        // 发布数据
    }

    return;
}

void NonlinearOpt::SyncCloudMsgCallBack(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
    CloudData cloud_data; 
    cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    //ros格式数据转为pcl格式
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));   // cloud_msg_ptr 转换为 cloud_data.cloud_ptr 

    new_cloud_data_buff_.push_back(cloud_data); 
}

void NonlinearOpt::GnssOdomMsgCallBack(const nav_msgs::OdometryConstPtr& odom_msg_ptr) {

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

    new_gnss_odom_data_buff_.push_back(pose_data);
}

void NonlinearOpt::FrontLaserOdomCallBack(const nav_msgs::OdometryConstPtr& odom_msg_ptr) {

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

    new_laser_odom_data_buff_.push_back(pose_data);
}

void NonlinearOpt::LoopPoseCallBack(const custom_msgs::poseWithIndex::ConstPtr& loop_pose_ptr) {
    LoopPose msg;

    msg.time = loop_pose_ptr->header.stamp.toSec();
    msg.pose(0,3) = loop_pose_ptr->position.x;
    msg.pose(1,3) = loop_pose_ptr->position.y;
    msg.pose(2,3) = loop_pose_ptr->position.z;

    Eigen::Quaternionf q;
    q.x() = loop_pose_ptr->orientation.x;
    q.y() = loop_pose_ptr->orientation.y;
    q.z() = loop_pose_ptr->orientation.z;
    q.w() = loop_pose_ptr->orientation.w;
    msg.pose.block<3,3>(0,0) = q.matrix();

    msg.index0 = loop_pose_ptr->closing_frame_index;
    msg.index1 = loop_pose_ptr->current_frame_index;    

    new_loop_pose_data_buff_.push_back(msg);
}

template<class T1,class T2>
void NonlinearOpt::ParseData(std::deque<T1>& callback_buff, std::deque<T2>& operate_buff) {

    if (callback_buff.size() > 0){    
        operate_buff.insert(operate_buff.end(), callback_buff.begin(), callback_buff.end());  
        callback_buff.clear();
    } 
}

bool NonlinearOpt::ReadData() { 
    std::lock_guard<std::mutex> LockGuard(buff_mutex_); 

    this->ParseData(new_cloud_data_buff_,      cloud_data_buff_);       // 
    this->ParseData(new_laser_odom_data_buff_, laser_odom_data_buff_);  // 
    this->ParseData(new_gnss_odom_data_buff_, gnss_odom_data_buff_);    // 
    this->ParseData(new_loop_pose_data_buff_, loop_pose_data_buff_);    // 

    return true;
}

bool NonlinearOpt::HasData() {

    if (cloud_data_buff_.size() == 0)
        return false;
    if (gnss_odom_data_buff_.size() == 0)
        return false;
    if (laser_odom_data_buff_.size() == 0)
        return false;

    return true;
}

bool NonlinearOpt::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_gnss_odom_data_ = gnss_odom_data_buff_.front();
    current_laser_odom_data_ = laser_odom_data_buff_.front();

    double diff_gnss_time = current_cloud_data_.time - current_gnss_odom_data_.time;
    double diff_laser_time = current_cloud_data_.time - current_laser_odom_data_.time;

    if (diff_gnss_time < -0.05 || diff_laser_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_gnss_time > 0.05) {
        gnss_odom_data_buff_.pop_front();
        return false;
    }

    if (diff_laser_time > 0.05) {
        laser_odom_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    gnss_odom_data_buff_.pop_front();
    laser_odom_data_buff_.pop_front();

    return true;

}

bool NonlinearOpt::PublishData() {
    // 1\转到gnss原点的前端雷达里程计
    odom_pub_ptr_->Publish(current_laser_odom_data_.pose, current_laser_odom_data_.time);
    // 2\关键帧:gnss pose & cloud pose
    if (has_new_key_frame_) { 
        key_frame_pub_ptr_->Publish(current_key_frame_);    // 都是同步了时间和id的
        key_gnss_pub_ptr_->Publish(current_key_gnss_);
    }
    // 3\优化后的位姿
    if (has_new_optimized_) {   
        std::deque<KeyFrame> optimized_key_frames;
        PoseTypeMatrix2KeyFrame(optimized_key_frames, optimized_pose_);
        optimized_key_frames_pub_ptr_->Publish(optimized_key_frames);
    }

    return true;
}

// optimized_pose_ 为Eigen::Matrix4f类型
// 发送出去的每组优化位姿index都从0开始 (图优化中有多少个顶点就有多少个优化位姿)
void NonlinearOpt::PoseTypeMatrix2KeyFrame(std::deque<KeyFrame>& buff, std::deque<Eigen::Matrix4f>& optimized_pose) {

    KeyFrame optimized_key_frame;
    for (size_t i = 0; i < optimized_pose.size(); ++i) {   // optimized_pose_ 为Eigen::Matrix4f类型
        optimized_key_frame.pose = optimized_pose.at(i);
        optimized_key_frame.index = (unsigned int)i;    // 发送出去的每组优化位姿index都从0开始 (图优化中有多少个顶点就有多少个优化位姿)
        buff.push_back(optimized_key_frame);
    }
}

bool NonlinearOpt::InitWithConfig() {
    std::cout << std::endl << "----------------->>Nonlinear-Optimizer-Init------------"<< std::endl << std::endl;  

    std::string cfp = data_path_ + "/map_build/nonlinear_optimize/config/back_end.yaml";
    YAML::Node cn = YAML::LoadFile(cfp);                                                                                                                 
    
    InitDataPathAndParam(cn);
    InitGraphOptimizer(cn);

    return true;
}

bool NonlinearOpt::InitDataPathAndParam(const YAML::Node& cn) {

    // path
    std::string datas_path = data_path_ + "/mapsData";

    if (!FileManager::CreateDirectory(datas_path))      // 创建 mapsData 文件夹
        return false;

    key_frames_path_ = data_path_ + "/mapsData/key_frames";
    trajectory_path_ = data_path_ + "/mapsData/trajectory";

    if (!FileManager::CreateDirectory(key_frames_path_, "key_frames file"))
        return false;
    if (!FileManager::CreateDirectory(trajectory_path_, "trajectory file"))
        return false;

    if (!FileManager::CreateFile(ground_truth_ofs_, trajectory_path_ + "/ground_truth.txt"))
        return false;
    if (!FileManager::CreateFile(laser_odom_ofs_, trajectory_path_ + "/laser_odom.txt"))
        return false;
    if (!FileManager::CreateFile(optimized_pose_ofs_, trajectory_path_ + "/optimized.txt"))
        return false;
    
    // param：关键帧取的距离 
    key_frame_distance_ = cn["key_frame_distance"].as<float>();

    return true;
}

bool NonlinearOpt::InitGraphOptimizer(const YAML::Node& cn) {

    std::string graph_optimizer_type = cn["graph_optimizer_type"].as<std::string>();
    if (graph_optimizer_type == "g2o") {

        graph_optimizer_ptr_ = std::make_shared<G2oGraphOptimizer>("lm_var");
        std::cout << "Now the optimizer type: " << graph_optimizer_type << std::endl << std::endl;

    } else {
        ROS_ERROR("Can't find the optimizer type: %s", graph_optimizer_type.c_str());
        return false;
    }

    graph_optimizer_config_.use_gnss = cn["use_gnss"].as<bool>();
    graph_optimizer_config_.use_loop_close = cn["use_loop_close"].as<bool>();

    graph_optimizer_config_.optimize_step_with_key_frame = cn["optimize_step_with_key_frame"].as<int>();
    graph_optimizer_config_.optimize_step_with_gnss = cn["optimize_step_with_gnss"].as<int>();
    graph_optimizer_config_.optimize_step_with_loop = cn["optimize_step_with_loop"].as<int>();

    for (int i = 0; i < 6; ++i) {
        graph_optimizer_config_.odom_edge_noise(i) =
            cn[graph_optimizer_type + "_param"]["odom_edge_noise"][i].as<double>();
        graph_optimizer_config_.close_loop_noise(i) =
            cn[graph_optimizer_type + "_param"]["close_loop_noise"][i].as<double>();

        if (i<3) {
            graph_optimizer_config_.gnss_noise(i) =
                cn[graph_optimizer_type + "_param"]["gnss_noise"][i].as<double>();
        }

    }

    return true;

}

bool NonlinearOpt::IsInsertLoopPose() {

    if (!graph_optimizer_config_.use_loop_close || !graph_optimizer_config_.use_gnss)    // 是否使用了回环优化(回环是依赖gnss的回环)
        return false;

    // 将所有回环插入到 凸优化器
    while (loop_pose_data_buff_.size() > 0) {

        current_loop_pose_data_ = loop_pose_data_buff_.front();

        Eigen::Isometry3d isometry;
        isometry.matrix() = current_loop_pose_data_.pose.cast<double>();    // 两个帧的相对位姿
        // 回环帧id，当前帧id
        graph_optimizer_ptr_->AddSe3Edge(current_loop_pose_data_.index0, current_loop_pose_data_.index1, isometry, graph_optimizer_config_.close_loop_noise);

        new_loop_cnt_++;

        loop_pose_data_buff_.pop_front();
    }
    return true;
}


bool NonlinearOpt::UpdateBackEnd() { 
    static bool odometry_inited = false;
    static Eigen::Matrix4f pose2gnss = Eigen::Matrix4f::Identity();

    if (!odometry_inited) { 
        odometry_inited = true;
        // TGmap_frame当前 * Tframe当前_Lmap = TGmap_Lmap
        pose2gnss = current_gnss_odom_data_.pose * current_laser_odom_data_.pose.inverse();
    }
    // 雷达里程计位姿初始与gnss轨迹对齐之后，进行优化(gnss系下的雷达里程计位姿)
    // TGmap_Lmap * TLmap_frame = TGmap_frame
    current_laser_odom_data_.pose = pose2gnss * current_laser_odom_data_.pose;

    return this->Update(current_cloud_data_, current_laser_odom_data_, current_gnss_odom_data_);
}

/**
 * 当前帧雷达数据
 * 前端里程计位姿
 * gnss对应的观测位姿
*/
bool NonlinearOpt::Update(const CloudData& cloud_data, const PoseData& laser_odom, const PoseData& gnss_pose) {    

// 是否产生了新的优化
    has_new_optimized_ = false;

// 有关键帧到来，才进行优化
    if (IsNewKeyFrame(cloud_data, laser_odom, gnss_pose)) {     // 判定是否属于新的关键帧、保存到硬盘\同步key_gnss (下标和时间) 
        SavePose(ground_truth_ofs_, gnss_pose.pose);
        SavePose(laser_odom_ofs_, laser_odom.pose);

        AddNodeAndEdge(gnss_pose);  // 为当前关键帧, 添加gnss观测    

        if (IsOptimized()) {        // 是否满足优化条件
            GetOptimizedPose();
        }
    }

    return true;
}

bool NonlinearOpt::IsOptimized() {

    bool need_optimize = false; 

    // 任何一个条件达到，即开始优化
    // 图模型中 有足够多的: 回环约束\gnss位置约束\足够多的顶点(关键帧)
    // 所有的关键帧都要加入凸优化
    if (new_gnss_cnt_ >= graph_optimizer_config_.optimize_step_with_gnss)
        need_optimize = true;   // gnss观测数
    if (new_loop_cnt_ >= graph_optimizer_config_.optimize_step_with_loop)
        need_optimize = true;   // 回环观测数
    if (new_key_frame_cnt_ >= graph_optimizer_config_.optimize_step_with_key_frame)
        need_optimize = true;   // 关键帧 优化间隔

    if (!need_optimize)
        return false;

    // reset param
    new_gnss_cnt_ = 0;
    new_loop_cnt_ = 0;
    new_key_frame_cnt_ = 0;

    // 调用优化器
    if (graph_optimizer_ptr_->Optimize())      
        has_new_optimized_ = true;

    return true;
}

// 最后一次，数据可能不够，也强制优化
bool NonlinearOpt::ForceOptimize() {
    if (graph_optimizer_ptr_->GetNodeNum() == 0)
        return false;
        
    if (graph_optimizer_ptr_->Optimize()) {
        has_new_optimized_ = true;
        this->GetOptimizedPose();     // 获取优化后的位姿保存:optimized_pose_ Eigen::Matrixf4f

        return true; 
    }
        
    return false; 
}

// 判定是否新的关键帧、保存到硬盘
bool NonlinearOpt::IsNewKeyFrame(const CloudData& cloud_data, const PoseData& laser_odom, const PoseData& gnss_odom) {
    // reset
    has_new_key_frame_ = false;
    static Eigen::Matrix4f last_key_pose = laser_odom.pose;

    if (key_frame_buff_.size() == 0) {    // 第一帧
        has_new_key_frame_ = true;
        last_key_pose = laser_odom.pose;
    }

    // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
    if (fabs(laser_odom.pose(0,3) - last_key_pose(0,3)) + 
        fabs(laser_odom.pose(1,3) - last_key_pose(1,3)) +
        fabs(laser_odom.pose(2,3) - last_key_pose(2,3)) > key_frame_distance_) {

        has_new_key_frame_ = true;
        last_key_pose = laser_odom.pose;
    }

    if (has_new_key_frame_) {
        // 把关键帧点云存储到硬盘里
        std::string file_path = key_frames_path_ + "/key_frame_" + std::to_string(key_frame_buff_.size()) + ".pcd";
        pcl::io::savePCDFileBinary(file_path, *cloud_data.cloud_ptr);

        KeyFrame key_frame;
        key_frame.time = laser_odom.time;
        key_frame.pose = laser_odom.pose;
        key_frame.index = (unsigned int)key_frame_buff_.size();       // 存储index
        key_frame_buff_.push_back(key_frame);

        current_key_frame_ = key_frame;     // 作为当前关键帧

        // 同步key_frame和key_gnss
        current_key_gnss_.time = gnss_odom.time;
        current_key_gnss_.index = key_frame.index;
        current_key_gnss_.pose = gnss_odom.pose;
    }

    return has_new_key_frame_;
}



// 添加节点和边
// 添加的节点: 关键帧的位姿
bool NonlinearOpt::AddNodeAndEdge(const PoseData& gnss_data) { 

    Eigen::Isometry3d isometry;
    // 添加关键帧节点
    isometry.matrix() = current_key_frame_.pose.cast<double>();
    if (!graph_optimizer_config_.use_gnss && graph_optimizer_ptr_->GetNodeNum() == 0)   // 图模型的第一个节点
        graph_optimizer_ptr_->AddSe3Node(isometry, true);      // 有观测第一个节点就不固定
    else
        graph_optimizer_ptr_->AddSe3Node(isometry, false);
    new_key_frame_cnt_ ++;

    // 添加激光里程计对应的边
    static KeyFrame last_key_frame = current_key_frame_;
    int node_num = graph_optimizer_ptr_->GetNodeNum();  // 待优化的节点数
    if (node_num > 1) {
        // 两节点之间的相对位姿 : Tlastframe_curframe
        Eigen::Matrix4f relative_pose = last_key_frame.pose.inverse() * current_key_frame_.pose;
        isometry.matrix() = relative_pose.cast<double>();
        // 连接最近的两个节点: 上一个节点和当前节点 
        graph_optimizer_ptr_->AddSe3Edge(node_num-2, node_num-1, isometry, graph_optimizer_config_.odom_edge_noise);
    }
    last_key_frame = current_key_frame_;

    // 为当前节点, 添加gnss位置对应的先验边
    if (graph_optimizer_config_.use_gnss) { 
        Eigen::Vector3d xyz(static_cast<double>(gnss_data.pose(0,3)),
                            static_cast<double>(gnss_data.pose(1,3)),
                            static_cast<double>(gnss_data.pose(2,3)));

        graph_optimizer_ptr_->AddSe3PriorXYZEdge(node_num - 1, xyz, graph_optimizer_config_.gnss_noise);
        new_gnss_cnt_ ++;
    }
    // 添加gnss姿态观测
    // ...

    return true;
}

// 获取优化后的位姿然后保存
bool NonlinearOpt::GetOptimizedPose() {
    if (graph_optimizer_ptr_->GetNodeNum() == 0)    
        return false;
	// 获取优化后的位姿存于: optimized_pose_    Eigen::Matrix4f
    graph_optimizer_ptr_->GetOptimizedPose(optimized_pose_);

    // 保存前先把上次的删除了
    if (!FileManager::CreateFile(optimized_pose_ofs_, trajectory_path_ + "/optimized.txt")) {
        return false;
    }
    // 写入  
    for (size_t i = 0; i < optimized_pose_.size(); ++i) { 
        SavePose(optimized_pose_ofs_, optimized_pose_.at(i));
    }

    return true;
}

bool NonlinearOpt::SavePose(std::ofstream& ofs, const Eigen::Matrix4f& pose) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ofs << pose(i, j);
            
            if (i == 2 && j == 3) {
                ofs << std::endl;
            } else {
                ofs << " ";
            }
        }
    }

    return true;
}

