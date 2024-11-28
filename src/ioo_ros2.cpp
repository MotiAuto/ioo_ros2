#include "ioo_ros2/ioo_ros2.hpp"

namespace ioo_ros2
{
    ImuOnlyOdometryROS2::ImuOnlyOdometryROS2(const rclcpp::NodeOptions& option) : Node("ImuOnlyOdometry", option)
    {
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu",
            qos_settings,
            std::bind(&ImuOnlyOdometryROS2::topic_callback, this, _1));

        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 0);

        timer_ = this->create_wall_timer(1ms, std::bind(&ImuOnlyOdometryROS2::timer_callback, this));

        posture_estimater = std::make_shared<PostureEstimater>();
    }

    void ImuOnlyOdometryROS2::topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        get_msg = msg;
    }

    void ImuOnlyOdometryROS2::timer_callback()
    {
        auto linear_accel = getEigenVec3(get_msg->linear_acceleration.x, get_msg->linear_acceleration.y, get_msg->linear_acceleration.z);
        auto angular_vel = getEigenVec3(get_msg->angular_velocity.x, get_msg->angular_velocity.y, get_msg->angular_velocity.z);
        
        auto estimated_posture = posture_estimater->estimate(angular_vel, linear_accel);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ioo_ros2::ImuOnlyOdometryROS2)