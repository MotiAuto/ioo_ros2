#ifndef IOO_ROS2_HPP_
#define IOO_ROS2_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
using namespace std::chrono_literals;
using std::placeholders::_1;

#include "posture.hpp"
#include "pose.hpp"

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
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr rpy_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        sensor_msgs::msg::Imu::SharedPtr get_msg;
        bool get_msg_flag;
        bool enable_pose_estimate;
        std::string frame_id;

        std::shared_ptr<PostureEstimater> posture_estimater;
        geometry_msgs::msg::PoseStamped odom_msg;
    };
}

#endif