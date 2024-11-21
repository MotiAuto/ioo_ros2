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
    }

    void ImuOnlyOdometryROS2::topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        get_msg = msg;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ioo_ros2::ImuOnlyOdometryROS2)