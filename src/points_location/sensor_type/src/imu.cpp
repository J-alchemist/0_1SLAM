#include "sensor_type/imu.h"
#include <cmath>

//Eigen中的四元素顺序wxyz，ros中xyzw
Eigen::Matrix3f IMUData::GetOrientationMatrix() {

    Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
    Eigen::Matrix3f matrix = q.matrix().cast<float>();

    return matrix;
}

/**
 * @brief sync_time : lidar time
 *  以雷达时间为基准（imu快，雷达慢），对imu数据进行插值计算，得到雷达数据的相应时刻时候的imu数据
 *  线性插值：两个数据a和b，时刻分别是0和1，那么我要得到任意时刻t（0<t<1）时刻的插值就是 a*(1-t)+b*t
 */
bool IMUData::SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time) {
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
