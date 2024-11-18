#include "ioo_ros2/posture.hpp"

namespace ioo_ros2
{
    PostureEstimater::PostureEstimater()
    : estimation_(Eigen::Vector3d(0.0, 0.0, 0.0)),
    cov_(Eigen::Matrix3d::Identity()),
    estimation_noise_(Eigen::Matrix3d::Identity()),
    observation_noise_(Eigen::Matrix2d::Zero()),
    kalman_gain_(Eigen::Matrix<double, 3, 2>::Zero())
    {
        cov_(0, 0) = 0.0174*0.001*0.001;
        cov_(1, 1) = 0.0174*0.001*0.001;
        cov_(2, 2) = 0.0174*0.001*0.001;

        estimation_noise_(0, 0) = 0.0174*0.001*0.001;
        estimation_noise_(1, 1) = 0.0174*0.001*0.001;
        estimation_noise_(2, 2) = 0.0174*0.001*0.001;
    }

    Eigen::Vector3d getInputMatrix(double angular_x, double angular_y, double angular_z)
    {
        return Eigen::Vector3d(
            angular_x* 0.001,
            angular_y* 0.001,
            angular_z* 0.001
        );
    }

    Eigen::Matrix<double, 3, 2> h()
    {
        return Eigen::Matrix<double, 3, 2>::Zero();
    }

    Eigen::Matrix3d jacob(const Eigen::Vector3d input_matrix, const Eigen::Vector3d estimation)
    {
        auto cos_roll = cos(estimation.x());
        auto sin_roll = sin(estimation.x());
        auto cos_pitch = cos(estimation.y());
        auto sin_pitch = sin(estimation.y());

        auto m11 = 1.0 + input_matrix.y();
    }
}