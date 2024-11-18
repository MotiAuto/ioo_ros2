#ifndef IOO_ROS2_HPP_
#define IOO_ROS2_HPP_

#include <rclcpp/rclcpp.hpp>

namespace ioo_ros2
{
    class ImuOnlyOdometryROS2 : public rclcpp::Node
    {
        public:
        explicit ImuOnlyOdometryROS2(const rclcpp::NodeOptions& option = rclcpp::NodeOptions());
    };
}

#endif