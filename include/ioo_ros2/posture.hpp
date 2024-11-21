#ifndef POSTURE_HPP_
#define POSTURE_HPP_

#include <Eigen/Core>

namespace ioo_ros2
{
    class PostureEstimater
    {
        public:
        PostureEstimater();

        Eigen::Vector3d estimate(Eigen::Vector3d input_matrix, Eigen::Vector3d linear_accel);

        private:
        Eigen::Matrix3d cov_;
        Eigen::Matrix3d estimation_noise_;
        Eigen::Matrix2d observation_noise_;
        Eigen::MatrixXd kalman_gain_;
        Eigen::Vector3d estimation_;
    };

    Eigen::Vector3d getInputMatrix(double angular_x, double angular_y, double angular_z);

    Eigen::Matrix<double, 3, 2> h();

    Eigen::Matrix3d jacob(const Eigen::Vector3d input_matrix, const Eigen::Vector3d estimation);

    Eigen::Vector3d predict_x(const Eigen::Vector3d input_matrix, const Eigen::Vector3d estimation);

    Eigen::Matrix3d predict_cov(const Eigen::Matrix3d jacob, const Eigen::Matrix3d cov, const Eigen::Matrix3d est_noise);

    Eigen::Vector2d update_residual(const Eigen::Vector2d obs, const Eigen::Vector3d est);
}

#endif