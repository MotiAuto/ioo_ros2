#ifndef POSTURE_HPP_
#define POSTURE_HPP_

#include <Eigen/Core>

namespace ioo_ros2
{
    class PostureEstimater
    {
        public:
        PostureEstimater();

        Eigen::Vector3d estimate(Eigen::Vector3d angular, Eigen::Vector3d linear_accel);

        private:
        Eigen::Matrix3d cov_;
        Eigen::Matrix3d estimation_noise_;
        Eigen::Matrix2d observation_noise_;
        Eigen::MatrixXd kalman_gain_;
        Eigen::Vector3d estimation_;
    };
}

#endif