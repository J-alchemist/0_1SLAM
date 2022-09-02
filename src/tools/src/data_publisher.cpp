#include "tools/data_publisher.h" 


OdomPublisher::OdomPublisher(ros::Publisher& puber, std::string frame_id, std::string child_frame_id) {
    publisher_ = puber;
    frame_id_ = frame_id;
    child_frame_id_ = child_frame_id;
}

void OdomPublisher::Publish(Eigen::Matrix4f& trans_matrix) {
    msg_.header.stamp = ros::Time::now();
    msg_.header.frame_id  = frame_id_; 
    msg_.child_frame_id = child_frame_id_;
    msg_.pose.pose.position.x = trans_matrix(0,3);
    msg_.pose.pose.position.y = trans_matrix(1,3);
    msg_.pose.pose.position.z = trans_matrix(2,3);
    Eigen::Quaternionf q;
    q = trans_matrix.block<3, 3>(0,0); 
    msg_.pose.pose.orientation.x = q.x();
    msg_.pose.pose.orientation.y = q.y();
    msg_.pose.pose.orientation.z = q.z();
    msg_.pose.pose.orientation.w = q.w();
    publisher_.publish(msg_);
}

void OdomPublisher::Publish(Eigen::Matrix4f& trans_matrix, double time) { 
    msg_.header.stamp = ros::Time(time);
    msg_.header.frame_id  = frame_id_; 
    msg_.child_frame_id = child_frame_id_;
    msg_.pose.pose.position.x = trans_matrix(0,3);
    msg_.pose.pose.position.y = trans_matrix(1,3);
    msg_.pose.pose.position.z = trans_matrix(2,3);
    Eigen::Quaternionf q;
    q = trans_matrix.block<3, 3>(0,0); 
    msg_.pose.pose.orientation.x = q.x();
    msg_.pose.pose.orientation.y = q.y();
    msg_.pose.pose.orientation.z = q.z();
    msg_.pose.pose.orientation.w = q.w();
    publisher_.publish(msg_);
}


// /*************************************************************/
KeyFramePublisher::KeyFramePublisher(ros::Publisher& puber, std::string frame_id) {
    publisher_ = puber;
    frame_id_ = frame_id;
}

void KeyFramePublisher::Publish(KeyFrame& key_frame) {
    
    msg_.header.stamp = ros::Time(key_frame.time);
    msg_.header.frame_id = frame_id_;

    msg_.position.x = (key_frame.pose)(0,3);
    msg_.position.y = (key_frame.pose)(1,3);
    msg_.position.z = (key_frame.pose)(2,3);

    Eigen::Quaternionf q = key_frame.GetQuaternion();
    msg_.orientation.x = q.x();
    msg_.orientation.y = q.y();
    msg_.orientation.z = q.z();
    msg_.orientation.w = q.w();

    msg_.current_frame_index = (double)key_frame.index;
    publisher_.publish(msg_);
}

void KeyFramePublisher::Publish(const std::deque<KeyFrame>& key_frames) {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = frame_id_;

    for (size_t i = 0; i < key_frames.size(); ++i) {
        KeyFrame key_frame;
        geometry_msgs::PoseStamped pose_stamped;

        key_frame = key_frames.at(i);
        pose_stamped.header.stamp = ros::Time(key_frame.time);
        pose_stamped.header.frame_id = frame_id_;
        pose_stamped.header.seq = key_frame.index;
        pose_stamped.pose.position.x = (key_frame.pose)(0,3);
        pose_stamped.pose.position.y = (key_frame.pose)(1,3);
        pose_stamped.pose.position.z = (key_frame.pose)(2,3);
        Eigen::Quaternionf q = key_frame.GetQuaternion();
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        path.poses.push_back(pose_stamped);
    }

    publisher_.publish(path);
}
// /*********************************************************************/

LoopPosePublisher::LoopPosePublisher(ros::Publisher& puber, std::string frame_id) {
    publisher_ = puber;
    frame_id_ = frame_id;
}

void LoopPosePublisher::Publish(LoopPose& loop_frame) {
    
    msg_.header.stamp = ros::Time(loop_frame.time);
    msg_.header.frame_id = frame_id_;

    msg_.position.x = (loop_frame.pose)(0,3);
    msg_.position.y = (loop_frame.pose)(1,3);
    msg_.position.z = (loop_frame.pose)(2,3);

    Eigen::Quaternionf q = loop_frame.GetQuaternion();
    msg_.orientation.x = q.x();
    msg_.orientation.y = q.y();
    msg_.orientation.z = q.z();
    msg_.orientation.w = q.w();

    msg_.closing_frame_index = (double)loop_frame.index0;
    msg_.current_frame_index = (double)loop_frame.index1;
    publisher_.publish(msg_);
}


/********************************************************************/
CloudPublisher::CloudPublisher(ros::Publisher& puber, std::string frame_id) {
    publisher_ = puber;
    frame_id_ = frame_id;
}

void CloudPublisher::Publish(CloudData::CLOUD_PTR&  cloud_ptr_input) {

    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);

    cloud_ptr_output->header.stamp = ros::Time::now();
    cloud_ptr_output->header.frame_id = frame_id_; 
    publisher_.publish(*cloud_ptr_output);

}

void CloudPublisher::Publish(CloudData::CLOUD_PTR&  cloud_ptr_input, double time) {

    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);

    cloud_ptr_output->header.stamp = ros::Time(time);
    cloud_ptr_output->header.frame_id = frame_id_; 
    publisher_.publish(*cloud_ptr_output);

}
