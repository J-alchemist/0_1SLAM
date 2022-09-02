#include "tools/internal_pose.h"


Eigen::Quaternionf KeyFrame::GetQuaternion() {

    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);

    return q;

}


Eigen::Quaternionf LoopPose::GetQuaternion() {

    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);

    return q;

}
