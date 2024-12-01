#ifndef POSE_HPP_
#define POSE_HPP_

#include <Eigen/Core>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace ioo_ros2
{
    class PoseEstimater
    {
        public:
        PoseEstimater();

        Eigen::Vector3d estimate(Eigen::Vector3d linear_accel, tf2::Quaternion est_posture);

        private:
        Eigen::Vector3d remove_g_noise(Eigen::Vector3d acc, tf2::Matrix3x3 rotation_mat)
        {
            Eigen::Vector3d g(0.0, 0.0, -9.8);
            Eigen::Matrix3d rotationMatrix(rotation_mat);

            return g;
        }
    };

    
}

#endif