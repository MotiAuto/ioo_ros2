#ifndef POSE_HPP_
#define POSE_HPP_

#include <Eigen/Core>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace ioo_ros2
{
    Eigen::Vector3d CalcPose(Eigen::Vector3d linear_accel, tf2::Quaternion est_posture, float dt);

    Eigen::Vector3d gravityRemove(Eigen::Vector3d linear_accel, tf2::Quaternion est_posture);

    Eigen::Vector3d simpleIntegral(Eigen::Vector3d value, float delta_time);

    Eigen::Vector3d quat2euler(tf2::Quaternion q);
}

#endif