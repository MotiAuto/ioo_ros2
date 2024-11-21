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
        Eigen::Matrix<double, 3, 2> h_;
        h_.setZero();
        h_(0, 0) = 1.0;
        h_(1, 1) = 1.0;
        
        return h_;
    }

    Eigen::Matrix3d jacob(const Eigen::Vector3d input_matrix, const Eigen::Vector3d estimation)
    {
        auto cos_roll = cos(estimation.x());
        auto sin_roll = sin(estimation.x());
        auto cos_pitch = cos(estimation.y());
        auto sin_pitch = sin(estimation.y());

        auto m_11 = 1.0 + input_matrix.y() * ((cos_roll*sin_pitch)/cos_pitch) - input_matrix.z() * ((sin_roll*sin_pitch)/cos_pitch);
        auto m_12 = input_matrix.y()*(sin_roll/(cos_pitch*cos_pitch))+input_matrix.z()*((cos_roll/(cos_pitch*cos_pitch)));
        auto m_21 = -1.0*input_matrix.y()*sin_roll - input_matrix.z()*cos_roll;
        auto m_31 = input_matrix.y()*(cos_roll/cos_pitch) - input_matrix.z()*(sin_roll/cos_pitch);
        auto m_32 = input_matrix.y()*((sin_roll*sin_pitch)/(cos_pitch*cos_pitch))+input_matrix.z()*((cos_roll*sin_pitch)/(cos_pitch*cos_pitch));

        Eigen::Matrix3d mat;
        mat.setZero();
        mat(0, 0) = m_11;
        mat(0, 1) = m_12;
        mat(1, 0) = m_21;
        mat(1, 1) = 1.0;
        mat(2, 0) = m_31;
        mat(2, 1) = m_32;

        return mat;
    }

    Eigen::Vector3d predict_x(const Eigen::Vector3d input_matrix, const Eigen::Vector3d estimation)
    {
        auto cos_roll = cos(estimation.x());
        auto sin_roll = sin(estimation.x());
        auto cos_pitch = cos(estimation.y());
        auto sin_pitch = sin(estimation.y());

        Eigen::Vector3d est;
        est(0) = estimation.x() + input_matrix.x() + input_matrix.y()*((sin_roll*sin_pitch)/cos_pitch)+input_matrix.z()*((cos_roll*sin_pitch)/cos_pitch);
        est(1) = estimation.y() + input_matrix.y() * cos_roll - input_matrix.z()*sin_roll;
        est(2) = estimation.z() + input_matrix.z() + input_matrix.y()*(sin_roll/cos_pitch) + input_matrix.z()*(cos_roll/cos_pitch);

        return est;
    }

    Eigen::Matrix3d predict_cov(const Eigen::Matrix3d jacob, const Eigen::Matrix3d cov, const Eigen::Matrix3d est_noise)
    {
        auto t_jacob = jacob.transpose();
        auto jacob_cov = jacob * cov;

        Eigen::Matrix3d new_cov;
        new_cov.setZero();

        auto multiplied = jacob_cov * t_jacob;

        new_cov = multiplied + est_noise;

        return new_cov;
    }

    Eigen::Vector2d update_residual(const Eigen::Vector2d obs, const Eigen::Vector3d est)
    {
        Eigen::Vector2d result;
        auto h_ = h();
        auto h_est = h_ * est;

        result(0) = obs.x() - h_est.x();
        result(1) = obs.y() - h_est.y();

        return result;
    }
}