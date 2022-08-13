#include "data_sync/data_sync.h"

DataSync::DataSync() { 

    workDir_ = "/";
    lidar_frame_id_ = "rslidar";
    imu_frame_id_ = "imu";

    cloudSub_ = nh_.subscribe("raw_lidar", 10000, &DataSync::cloudMsgCallBack, this);
    imuSub_   = nh_.subscribe("raw_imu", 10000, &DataSync::imuMsgCallBack, this);
    gnssSub_ = nh_.subscribe("raw_gnss", 10000, &DataSync::gnssMsgCallBack, this);
    lidar_to_imu_ptr_ = std::make_shared<TFConvert>(imu_frame_id_, lidar_frame_id_);

    cloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("synced_cloud",100);
    gnssPub_ = nh_.advertise<nav_msgs::Odometry>("synced_gnss",100);
    imuPub_ = nh_.advertise<sensor_msgs::Imu>("synced_imu",100);
}

DataSync::~DataSync(){ 

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

void DataSync::parseImuData(std::deque<IMUData>& deque_data){

    if (new_imu_data_.size() > 0){
        //new数据插入到deque里面
        //deque_data.end()前 插入区间new_cloud_data_[begin, end)的所有元素
        //==在deque末尾插入新元素
        deque_data.insert(deque_data.end(), new_imu_data_.begin(), new_imu_data_.end());  
        new_imu_data_.clear();
    } 


}

void DataSync::parseGnssData(std::deque<GNSSData>& deque_data){

    if (new_gnss_data_.size() > 0){
        deque_data.insert(deque_data.end(), new_gnss_data_.begin(), new_gnss_data_.end());  
        new_gnss_data_.clear();
    } 

}

void DataSync::parseCloudData(std::deque<CloudData>& deque_data){
    if (new_cloud_data_.size() > 0){
        deque_data.insert(deque_data.end(), new_cloud_data_.begin(), new_cloud_data_.end());  
        new_cloud_data_.clear();
    } 


}

/* 
 * 先读取一部分数据，进行时间对齐，然后拿去用，用完了(HasData()跳出)，再重复
 */
bool DataSync::Run() { 
    if (!ReadData())                //数据同步
        return false;

    if (!InitCalibration())         //获取lidar-imu静态变换 
        return false;

    if (!InitGNSS())                //初始化gnss位姿
        return false;

    while(HasData()) {
        if (!ValidData())           //对齐的数据是否正常
            continue;

        TransformData();
        PublishData();
    }

    return true;
}

/* 时间同步
 * 以雷达数据时间为准，线性插值其他数据
 */
bool DataSync::ReadData() { 

    static std::deque<IMUData> unsynced_imu_;
    static std::deque<GNSSData> unsynced_gnss_;     //存放未同步的数据, new_imu_data_往里面放

    parseGnssData(unsynced_gnss_);
    parseImuData(unsynced_imu_);
    parseCloudData(synced_cloud_data_);

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

bool DataSync::InitGNSS() {
    static bool gnss_inited = false;
    if (!gnss_inited) {
        GNSSData gnss_data = synced_gnss_data_.front();
        gnss_data.InitOriginPosition();         //gnss初始位姿，原点
        gnss_inited = true; 
    }

    return gnss_inited;

}

bool DataSync::InitCalibration() {
    static bool calibration_received = false;
    
    if (!calibration_received) {
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) { //获取雷达到imu的变换 
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
    gnss_pose_(0,3) = current_gnss_data_.local_E;   // xyz
    gnss_pose_(1,3) = current_gnss_data_.local_N; 
    gnss_pose_(2,3) = current_gnss_data_.local_U;
    // imu 姿态
    gnss_pose_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix(); 

    /*2-lidar转到map下**/
    gnss_pose_ = gnss_pose_ * lidar_to_imu_;    // 此时gnss_pose_已经是lidar在map下的位姿了（组合导航获取的雷达传感器在map的位置）

    /*3-lidar畸变矫正**/
    // 暂时。。。


    return true;
}


bool DataSync::PublishData() {

    /***** 1-发布点云 ******/
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    ros::Time lidar_ros_time((float)current_cloud_data_.time);

    pcl::toROSMsg(*(current_cloud_data_.cloud_ptr), *cloud_ptr_output);
    cloud_ptr_output->header.stamp = lidar_ros_time;
    cloud_ptr_output->header.frame_id = lidar_frame_id_; 
    cloudPub_.publish(*cloud_ptr_output); 

    /***** 2-发布里程计( lidar传感器在map下的位姿 ) *****/
    nav_msgs::Odometry odometry; 
    ros::Time gnss_ros_time((float)current_gnss_data_.time);
    odometry.header.stamp = gnss_ros_time;

    odometry.pose.pose.position.x = gnss_pose_(0,3);
    odometry.pose.pose.position.y = gnss_pose_(1,3);
    odometry.pose.pose.position.z = gnss_pose_(2,3);

    Eigen::Quaternionf q;
    q = gnss_pose_.block<3,3>(0,0);
    odometry.pose.pose.orientation.x = q.x();
    odometry.pose.pose.orientation.y = q.y();
    odometry.pose.pose.orientation.z = q.z();
    odometry.pose.pose.orientation.w = q.w();
    gnssPub_.publish(odometry);

}

bool DataSync::AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr) {
    // init
    CloudData::CLOUD_PTR  origin_cloud_ptr(new CloudData::CLOUD(*input_cloud_ptr)); 
    output_cloud_ptr.reset(new CloudData::CLOUD());
    // calc start point angle
    float start_orientation = atan2(origin_cloud_ptr->points[0].y,  origin_cloud_ptr->points[0].x); //计算起始点z轴的旋转角度 弧度返回
    // 构造旋转矩阵
    Eigen::Matrix3f rotate_matrix = Eigen::AngleAxisf(start_orientation, Eigen::Vector3f::UnitZ()).matrix();
    // 构造变换矩阵
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();      
    transform_matrix.block<3,3>(0,0) = rotate_matrix.inverse();    
    pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, transform_matrix);  //？？？？？？？？？？？？？？

    for (size_t point_index = 1; point_index < origin_cloud_ptr->points.size(); ++point_index) {




    }

    // pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr, transform_matrix.inverse()); 
    return true;

}


