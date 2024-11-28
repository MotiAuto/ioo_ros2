#ifndef IOO_ROS2_HPP_
#define IOO_ROS2_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <chrono>
using namespace std::chrono_literals;
using std::placeholders::_1;

#include "posture.hpp"

namespace ioo_ros2
{
    class ImuOnlyOdometryROS2 : public rclcpp::Node
    {
        public:
        explicit ImuOnlyOdometryROS2(const rclcpp::NodeOptions& option = rclcpp::NodeOptions());

        void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void timer_callback();

        private:
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        sensor_msgs::msg::Imu::SharedPtr get_msg;

        std::shared_ptr<PostureEstimater> posture_estimater;
    };
}

#endif