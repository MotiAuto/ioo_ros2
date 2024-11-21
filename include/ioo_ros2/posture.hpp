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

    Eigen::Vector3d getInputMatrix(double angular_x, double angular_y, double angular_z);

    Eigen::Matrix<double, 3, 2> h();

    Eigen::Matrix3d jacob(const Eigen::Vector3d input_matrix, const Eigen::Vector3d estimation);

    Eigen::Vector3d predictX(const Eigen::Vector3d input_matrix, const Eigen::Vector3d estimation);

    Eigen::Matrix3d predictCov(const Eigen::Matrix3d jacob, const Eigen::Matrix3d cov, const Eigen::Matrix3d est_noise);

    Eigen::Vector2d updateResidual(const Eigen::Vector2d obs, const Eigen::Vector3d est);

    Eigen::Matrix2d updateS(const Eigen::Matrix3d cov_, const Eigen::Matrix2d obs_noise);

    Eigen::Matrix<double, 3, 2> updateKalmanGain(const Eigen::Matrix2d s, const Eigen::Matrix3d cov);

    Eigen::Vector3d updateX(const Eigen::Vector3d est, const Eigen::Matrix<double, 3, 2> kalman_gain_, const Eigen::Vector2d residual);

    Eigen::Matrix3d updateCov(const Eigen::Matrix<double, 3, 2> kalman_gain, const Eigen::Matrix3d cov);

    Eigen::Vector2d obsModel(const Eigen::Vector3d linear_accel);
}

#endif