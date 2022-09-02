#include "data_sync/data_sync.h"

DataSync::DataSync(ros::NodeHandle& nh, std::string& imu_frame_id, 
                    std::string& lidar_frame_id, std::string& mode, double* origin) 
                    : nh_(nh), imu_frame_id_(imu_frame_id), lidar_frame_id_(lidar_frame_id), 
                      sys_mode_(mode), origin_lon_(origin[0]), origin_lat_(origin[1]), origin_alt_(origin[2]) {
                        
    std::cout << std::endl << "----------------->>Data-Sync-Working-------------------"<< std::endl << std::endl;  

    // 指针发布测试
    // test_ = std::make_shared<ros::Subscriber>();
    // *test_ = nh_.subscribe("raw_imu", 1, &DataSync::imuMsgCallBack, this);
// 订阅
    cloudSub_ = nh_.subscribe("/kitti/velo/pointcloud", 10000, &DataSync::cloudMsgCallBack, this);
    imuSub_   = nh_.subscribe("/kitti/oxts/imu", 10000, &DataSync::imuMsgCallBack, this);
    gnssSub_  = nh_.subscribe("/kitti/oxts/gps/fix", 10000, &DataSync::gnssMsgCallBack, this);

// 发布
    cloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/synced_cloud",100);
    cloudPub_ptr_ = std::make_shared<CloudPublisher>(cloudPub_, lidar_frame_id_);

    gnssPub_ = nh_.advertise<nav_msgs::Odometry>("/synced_gnss_odom",100);   // 注意此数据 是lidar传感器在map下的位姿
    gnssPub_ptr_ = std::make_shared<OdomPublisher>(gnssPub_, "/map", lidar_frame_id_);
    // imuPub_ = nh_.advertise<sensor_msgs::Imu>("/synced_imu",100);  

// imu与雷达的静态变换
// kitti: imu_link    velo_link
    // lidar_to_imu_ptr_ = std::make_shared<TFConvert>(imu_frame_id_, lidar_frame_id_);
    lidar_to_imu_ptr_ = std::make_shared<TFConvert>("/imu_link", "/velo_link");

} 

DataSync::~DataSync() { 
    std::cout << "__________Data_Sync_node free..." << std::endl;
}


bool DataSync::Exec() { 
    if (!DataInterpolation())       //数据插值，实现同步
        return false;

    if (!InitCalibration())         //获取lidar-imu静态变换 
        return false;

    if (!InitGNSS())                //初始化gnss原点位姿
        return false;

    while(HasData()) { 
        if (!ValidData())           //对齐的数据是否正常
            continue;
        //gnss定位转到雷达上
        TransformData();           
        // 畸变处理(暂时未用)
        // VelocityToLidar(lidar_to_imu_);
        // AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);
        //发布 gnss_odom 和 cloud
        PublishData();              
    }

    return true;
}


void DataSync::imuMsgCallBack( const sensor_msgs::ImuConstPtr& imu_msg_ptr){

    IMUData imu_data;
    imu_data.time = imu_msg_ptr->header.stamp.toSec();

    imu_data.linear_acceleration.x = imu_msg_ptr->linear_acceleration.x;
    imu_data.linear_acceleration.y = imu_msg_ptr->linear_acceleration.y;
    imu_data.linear_acceleration.z = imu_msg_ptr->linear_acceleration.z;

    imu_data.angular_velocity.x = imu_msg_ptr->angular_velocity.x;
    imu_data.angular_velocity.y = imu_msg_ptr->angular_velocity.y;
    imu_data.angular_velocity.z = imu_msg_ptr->angular_velocity.z;

    imu_data.orientation.x = imu_msg_ptr->orientation.x;
    imu_data.orientation.y = imu_msg_ptr->orientation.y;
    imu_data.orientation.z = imu_msg_ptr->orientation.z;
    imu_data.orientation.w = imu_msg_ptr->orientation.w;

    new_imu_data_.push_back(imu_data);
}

void DataSync::gnssMsgCallBack( const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr){
    GNSSData gnss_data;
    gnss_data.time = nav_sat_fix_ptr->header.stamp.toSec();
    gnss_data.latitude = nav_sat_fix_ptr->latitude;
    gnss_data.longitude = nav_sat_fix_ptr->longitude;
    gnss_data.altitude = nav_sat_fix_ptr->altitude;
    gnss_data.status = nav_sat_fix_ptr->status.status;
    gnss_data.service = nav_sat_fix_ptr->status.service;

    new_gnss_data_.push_back(gnss_data);
}

void DataSync::cloudMsgCallBack( const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr){

    CloudData cloud_data; 
    cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    //ros格式数据转为pcl格式
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));   // cloud_msg_ptr 转换为 cloud_data.cloud_ptr 

    new_cloud_data_.push_back(cloud_data); 
}
/**
 * @brief sync_time : lidar time
 *  以雷达时间为基准（imu快，雷达慢），对imu数据进行插值计算，得到雷达数据的相应时刻时候的imu数据
 *  线性插值：两个数据a和b，时刻分别是0和1，那么我要得到任意时刻t（0<t<1）时刻的插值就是 a*(1-t)+b*t
 */
bool DataSync::syncImuData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time) {
    // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
    // 即找到与同步时间相邻的左右两个数据
    // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
    // 1
    while (UnsyncedData.size() >= 2) {
        if (UnsyncedData.front().time > sync_time)  //imu数据比该帧雷达数据慢，即插入时刻的前面没有数据，那么就无从插入，直接退出
            return false;
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            continue;
        }
        if (sync_time - UnsyncedData.front().time > 0.2) {
            UnsyncedData.pop_front();
            break;
        }
        
        if (UnsyncedData.at(1).time - sync_time > 0.2) {
            UnsyncedData.pop_front();
            break;
        }
        break;
    }
    // 2
    if (UnsyncedData.size() < 2)    //帅选之后要还存在2个数据
        return false;

    IMUData front_data = UnsyncedData.at(0);
    IMUData back_data = UnsyncedData.at(1);
    IMUData synced_data; 

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time); //雷达数据处在两帧imu之间
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.linear_acceleration.x = front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
    synced_data.linear_acceleration.y = front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
    synced_data.linear_acceleration.z = front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;
    synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
    synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
    synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;
    // 四元数（旋转）插值算法：线性插值和球面插值，球面插值更准确，但是两个四元数差别不大时，二者精度相当
    // 线性插值
    synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
    synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
    synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
    synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;
    // 球面插值：p0->p1的插值计算：p0 * pow( p0.inverse()*p1, t ), t在p0帧时刻与p1帧时刻之间，编程采用简化公式
    // GuoJian 2022/05/13   Eigen库的Slerp函数 
    // synced_data.slerp(sync_time, front_data.orientation);

    // 线性插值之后要归一化
    synced_data.orientation.Normlize(); 

    SyncedData.push_back(synced_data); 

    return true;

}

bool DataSync::syncGnssData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time){
        // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
        // 即找到与同步时间相邻的左右两个数据
        // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
        while (UnsyncedData.size() >= 2) {
            if (UnsyncedData.front().time > sync_time)
                return false;
            if (UnsyncedData.at(1).time < sync_time) {
                UnsyncedData.pop_front();
                continue;
            }
            if (sync_time - UnsyncedData.front().time > 0.2) {
                UnsyncedData.pop_front();
                break;
            }
            if (UnsyncedData.at(1).time - sync_time > 0.2) {
                UnsyncedData.pop_front();
                break;
            }
            break;
        }
        if (UnsyncedData.size() < 2)
            return false;

        GNSSData front_data = UnsyncedData.at(0);
        GNSSData back_data = UnsyncedData.at(1);
        GNSSData synced_data;

        double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
        double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
        synced_data.time = sync_time;
        synced_data.status = back_data.status;
        synced_data.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;
        synced_data.latitude = front_data.latitude * front_scale + back_data.latitude * back_scale;
        synced_data.altitude = front_data.altitude * front_scale + back_data.altitude * back_scale;
        synced_data.local_E = front_data.local_E * front_scale + back_data.local_E * back_scale;
        synced_data.local_N = front_data.local_N * front_scale + back_data.local_N * back_scale;
        synced_data.local_U = front_data.local_U * front_scale + back_data.local_U * back_scale;

        SyncedData.push_back(synced_data);
        
        return true;
}


template<class T1,class T2>
void DataSync::ParseData(std::deque<T1>& callback_buff, std::deque<T2>& operate_buff) {
        //new数据插入到deque里面
        //deque_data.end()前 插入区间new_cloud_data_[begin, end)的所有元素
        //==在deque末尾插入新元素
    if (callback_buff.size() > 0){    // 点云
        operate_buff.insert(operate_buff.end(), callback_buff.begin(), callback_buff.end());  
        callback_buff.clear();
    } 
}

/* 时间同步
 * 以雷达数据时间为准，线性插值其他数据
 */
bool DataSync::DataInterpolation() { 

    static std::deque<IMUData> unsynced_imu_;
    static std::deque<GNSSData> unsynced_gnss_;     //拿去处理的存放未同步的数据
    {
        std::lock_guard<std::mutex> LockGuard(buff_mutex_);     // 作用域内自动枷锁解锁

        this->ParseData(new_imu_data_, unsynced_imu_);          // imu
        this->ParseData(new_gnss_data_, unsynced_gnss_);        // gnss
        this->ParseData(new_cloud_data_, synced_cloud_data_);   // cloud
    }

    if (!synced_cloud_data_.size())
        return false;

    double cloud_time = synced_cloud_data_.front().time;    //插值时间
    bool valid_imu = syncGnssData(unsynced_gnss_, synced_gnss_data_, cloud_time);
    bool valid_gnss = syncImuData(unsynced_imu_, synced_imu_data_, cloud_time);
    
    static bool sensor_inited = false;
    if (!sensor_inited){        // 传感器是否初始化

        if (!valid_imu || !valid_gnss) {
            synced_cloud_data_.pop_front();
            return false;
        }
        sensor_inited = true;
    }

    return true;
}

bool DataSync::HasData(){

    if ( !synced_imu_data_.size() || !synced_gnss_data_.size() || !synced_cloud_data_.size())
        return false;

    return true;
}

bool DataSync::ValidData() { 

    current_cloud_data_ = synced_cloud_data_.front();
    current_imu_data_ = synced_imu_data_.front();
    current_gnss_data_ = synced_gnss_data_.front(); 

    double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
    double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time;
    if (diff_imu_time < -0.05 || diff_gnss_time < -0.05) {  // 点云快多了
        synced_cloud_data_.pop_front();
        return false;
    }

    if (diff_imu_time > 0.05) {     // imu快多了
        synced_imu_data_.pop_front(); 
        return false;
    }

    if (diff_gnss_time > 0.05) {    // gnss快多了
        synced_gnss_data_.pop_front();
        return false;
    }

    synced_cloud_data_.pop_front();
    synced_imu_data_.pop_front();
    synced_gnss_data_.pop_front();

    return true;

}

void DataSync::SaveGnssOrigin(double& lon, double& lat, double& al) {
    env_p = std::getenv("SEED_HOME");
    if(env_p == nullptr){
        std::cout << "exception happened while reading env variable \"SEED_HOME\" " << std::endl;
        exit(1);
    }
    std::string home_path = env_p;
    fout.precision(12);
    fout.open(home_path+"/src/mapsData/GnssMapOrigin.txt", std::ios::out);
    fout << lon << " " << lat << " " << al << std::endl;
    fout.close(); 
}

bool DataSync::InitGNSS() {
    static bool gnss_inited = false;
    if (!gnss_inited) {

        GNSSData gnss_data;
        if (sys_mode_ == "mapping")
            gnss_data = synced_gnss_data_.front();
        else if (sys_mode_ == "matching") { 
            gnss_data.longitude = origin_lon_;
            gnss_data.latitude = origin_lat_;
            gnss_data.altitude = origin_alt_;
        } else {
            std::cout << "sys_mode choose false!!!, please choose one from \"mapping\" or \"matching\" " 
						<< std::endl<< std::endl;
        }
        gnss_data.InitOriginPosition();         //gnss初始位姿，原点
        gnss_inited = true; 

        SaveGnssOrigin(gnss_data.origin_longitude, gnss_data.origin_latitude, gnss_data.origin_altitude);
    }

    return gnss_inited;
}

bool DataSync::InitCalibration() {
    static bool calibration_received = false;
    
    if (!calibration_received) {
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) { //获取雷达到imu的坐标变换 Til
            calibration_received = true;
        }
    }

    return calibration_received;
}

bool DataSync::TransformData() {

    /*1-imu在map下的位姿**/
    // gnss 位置
    current_gnss_data_.UpdateXYZ();     //lla ==> enu
    gnss_pose_ = Eigen::Matrix4f::Identity();
    gnss_pose_(0,3) = current_gnss_data_.local_E;   // xyz 平移
    gnss_pose_(1,3) = current_gnss_data_.local_N; 
    gnss_pose_(2,3) = current_gnss_data_.local_U;
    // imu 姿态
    gnss_pose_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix(); // 旋转

    /*2-lidar转到map下**/
    // Tmap_lidar = Tmap_imu * Timu_lidar 
    gnss_pose_ = gnss_pose_ * lidar_to_imu_;    // 此时gnss_pose_已经是lidar在map下的位姿了（组合导航获取的雷达传感器在map的位置）

    return true; 
}

bool DataSync::PublishData() {

    /***** 1-发布点云 ******/
    cloudPub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);
    /***** 2-发布里程计( lidar传感器在map下的位姿 ) *****/
    gnssPub_ptr_->Publish(gnss_pose_, current_gnss_data_.time);
}

// 一帧点云的矫正
bool DataSync::AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr) {
    // init
    CloudData::CLOUD_PTR  origin_cloud_ptr(new CloudData::CLOUD(*input_cloud_ptr)); 
    output_cloud_ptr.reset(new CloudData::CLOUD());
    // calc start point angle
    float start_orientation = atan2(origin_cloud_ptr->points[0].y,  origin_cloud_ptr->points[0].x); //计算起始点z轴的旋转角度 弧度返回 雷达坐标系x向前，y向左
    // 构造旋转矩阵  rotate_matrix = Rfix_point0 
    Eigen::Matrix3f rotate_matrix = Eigen::AngleAxisf(start_orientation, Eigen::Vector3f::UnitZ()).matrix();
    // 构造变换矩阵
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();      
    transform_matrix.block<3,3>(0,0) = rotate_matrix;   
    // 所有点旋转到起始点的雷达坐标系下   记起始点的雷达坐标系为Tstart   
    pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, transform_matrix.inverse()); 
   
    // 将当前角速度和线速度旋转到沿Tstart坐标系
    velocity_ = rotate_matrix.inverse() * velocity_;  //velocity_ & angular_rate_已经是转到了雷达的速度
    angular_rate_ = rotate_matrix.inverse() * angular_rate_; 

    float orientation_space = 2.0 * M_PI;       //雷达一周旋转360°，周期scan_period_ = 100ms 
    // int lidar_line = 32; 

    // 雷达旋转方向 
    for (size_t point_index = 0; point_index < origin_cloud_ptr->points.size(); ++point_index) {
        if(point_index==0){
            continue;
        }
 
        float orientation = atan2(origin_cloud_ptr->points[point_index].y, origin_cloud_ptr->points[point_index].x);   
        if (orientation < 0.0)          // 0-360
            orientation += 2.0 * M_PI;    
            
        float real_time = ( fabs(orientation) / orientation_space * scan_period_ ) - scan_period_ / 2.0;    
        Eigen::Vector4f origin_point(origin_cloud_ptr->points[point_index].x,
                                     origin_cloud_ptr->points[point_index].y,
                                     origin_cloud_ptr->points[point_index].z,
                                     1 ); 
        Eigen::Vector3f angle = angular_rate_ * real_time;      // 匀速运动假设
        Eigen::AngleAxisf t_Vz(angle(2), Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf t_Vy(angle(1), Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf t_Vx(angle(0), Eigen::Vector3f::UnitX());
        Eigen::Matrix4f current_matrix = Eigen::Matrix4f::Identity();
        current_matrix.block<3,3>(0,0) = (t_Vz * t_Vy * t_Vx).matrix();
        current_matrix.block<3,1>(0,3) =  velocity_ * real_time;
        Eigen::Vector4f aft_point = current_matrix * origin_point; // 也可以Rx+t

        CloudData::POINT point;
        point.x = aft_point(0);
        point.y = aft_point(1);
        point.z = aft_point(2);
        output_cloud_ptr->points.push_back(point);
    }

    pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr, transform_matrix); // 将矫正后的点云恢复到原雷达系下
    return true;

}

// 速度转换到雷达上，为畸变准备
// 输入：lidar到imu的坐标变换矩阵 Ti_l
void DataSync::VelocityToLidar(Eigen::Matrix4f transform_matrix) { 
   
    Eigen::Matrix4d matrix = (transform_matrix.cast<double>()).inverse();
    Eigen::Matrix3d t_R = matrix.block<3,3>(0,0);

    // Eigen::Vector3d w(angular_velocity.x, angular_velocity.y, angular_velocity.z);
    // Eigen::Vector3d v(linear_velocity.x, linear_velocity.y, linear_velocity.z);
    Eigen::Vector3d w(0, 0, 0);
    Eigen::Vector3d v(0, 0, 0);

    w = t_R * w;    // Vlidar = Ti_l.inverse() * Vimu 
    v = t_R * v; 

    Eigen::Vector3d r(matrix(0,3), matrix(1,3), matrix(2,3));   // imu坐标系如何移动到lidar
    Eigen::Vector3d delta_v;
    // 杆臂速度（线速度） = 角速度 [叉乘] 杆臂(半径)
    // 必须相对于同一个坐标系，才能加
    delta_v(0) = w(1) * r(2) - w(2) * r(1);     // w叉乘r（外积）：delta_v垂直与wr平面（2维=面积，3维=法向量）
    delta_v(1) = w(2) * r(0) - w(0) * r(2);
    delta_v(2) = w(0) * r(1) - w(1) * r(0); 
    v = v + delta_v;    // 原点的线速度 + 角速度引起的线速度

    // angular_velocity.x = w(0);
    // angular_velocity.y = w(1);
    // angular_velocity.z = w(2);
    // linear_velocity.x = v(0);
    // linear_velocity.y = v(1);
    // linear_velocity.z = v(2);

    return;    
}
