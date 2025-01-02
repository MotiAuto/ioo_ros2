#include "ioo_ros2/pose.hpp"

namespace ioo_ros2
{
    Eigen::Vector3d CalcPose(Eigen::Vector3d linear_accel, tf2::Quaternion est_posture, float dt)
    {
        const Eigen::Vector3d g_removed = gravityRemove(linear_accel, est_posture);

        const Eigen::Vector3d velocity = simpleIntegral(g_removed, dt);

        const Eigen::Vector3d robot_value = simpleIntegral(velocity, dt);

        const Eigen::Vector3d posture_euler = quat2euler(est_posture);

        const auto world_x = cos(posture_euler.z()) * robot_value.x() + sin(posture_euler.z()) * robot_value.y();
        const auto world_y = -1.0 * sin(posture_euler.z()) * robot_value.x() + cos(posture_euler.z()) * robot_value.y();

        return Eigen::Vector3d(world_x, world_y, 0.0);
    }

    Eigen::Vector3d gravityRemove(Eigen::Vector3d linear_accel, tf2::Quaternion est_posture)
    {
        static tf2::Vector3 g(0.0, 0.0, -9.81);
        tf2::Matrix3x3 rotation_mat(est_posture);
    
        tf2::Vector3 g_vec = rotation_mat.inverse() * g;

        return Eigen::Vector3d(linear_accel.x() - g_vec.x(), linear_accel.y() - g_vec.y(), linear_accel.z() - g_vec.z());
    }

    Eigen::Vector3d simpleIntegral(Eigen::Vector3d value, float delta_time)
    {
        const auto x = value.x() * delta_time;
        const auto y = value.y() * delta_time;
        const auto z = value.z() * delta_time;

        return Eigen::Vector3d(x, y, z);
    }

    Eigen::Vector3d quat2euler(tf2::Quaternion q)
    {
        tf2::Matrix3x3 mat(q);

        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        return Eigen::Vector3d(roll, pitch, yaw);   
    }
}