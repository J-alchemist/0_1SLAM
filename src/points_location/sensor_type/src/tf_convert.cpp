#include "sensor_type/tf_convert.h"

TFConvert::TFConvert(std::string base_frame, std::string child_frame){
    child_frame_id_ = child_frame;
    base_frame_id_ = base_frame;
}

TFConvert::~TFConvert() {


}

bool TFConvert::LookupData(Eigen::Matrix4f &transform_matrix) {
    try {
        tf::StampedTransform transform;
        // child_frame_id_ to base_frame_id_ 变换
        listener_.lookupTransform(base_frame_id_, child_frame_id_, ros::Time(0), transform); 
        TransformToMatrix(transform, transform_matrix); 
        return true;
    } catch (tf::TransformException &ex) {
        return false;
    }


}

bool TFConvert::TransformToMatrix(const tf::StampedTransform &transform, Eigen::Matrix4f &transform_matrix) {

    double roll, pitch, yaw;
    //平移
    Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
    //姿态
    tf::Matrix3x3( transform.getRotation() ).getEulerYPR(yaw, pitch, roll); //ros中的角，getRPY内部也是调用getEulerYPR
    //tf::Matrix3x3( transform.getRotation() ).getRPY(roll, pitch,yaw); //ros中的角

    Eigen::AngleAxisf rot_x_btol( roll, Eigen::Vector3f::UnitX() );     //转为Eigen中的旋转向量， 以x轴旋转roll弧度
    Eigen::AngleAxisf rot_y_btol( pitch, Eigen::Vector3f::UnitY() ); 
    Eigen::AngleAxisf rot_z_btol( yaw, Eigen::Vector3f::UnitZ() ); 

    //TF变换转换到变换矩阵 
    transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();  

    return true; 



}
bool TFConvert::TransformToMatrix(const geometry_msgs::TransformStamped& trans, Eigen::Matrix4f& transform_matrix) {  

    double roll, pitch, yaw;
	tf::StampedTransform transform;
	//
	transform.setOrigin( tf::Vector3(trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z) );
	transform.setRotation( tf::Quaternion(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w) );
    //平移 
    Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
    //姿态
    tf::Matrix3x3( transform.getRotation() ).getEulerYPR(yaw, pitch, roll); //ros中的角

    Eigen::AngleAxisf rot_x_btol( roll, Eigen::Vector3f::UnitX() );     //转为Eigen中的旋转向量， 以x轴旋转roll弧度
    Eigen::AngleAxisf rot_y_btol( pitch, Eigen::Vector3f::UnitY() ); 
    Eigen::AngleAxisf rot_z_btol( yaw, Eigen::Vector3f::UnitZ() ); 

    //TF变换转换到变换矩阵 
    transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();  //右乘

    return true; 
}