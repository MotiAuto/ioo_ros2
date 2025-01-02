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

        rpy_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("/posture", 0);

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_stamped", 0);

        timer_ = this->create_wall_timer(1ms, std::bind(&ImuOnlyOdometryROS2::timer_callback, this));

        posture_estimater = std::make_shared<PostureEstimater>();
        get_msg_flag = false;

        this->declare_parameter("enable_pose_estimate", true);
        this->get_parameter("enable_pose_estimate", enable_pose_estimate);

        this->declare_parameter("frame_id", "map");
        this->get_parameter("frame_id", frame_id);

        odom_msg.header.frame_id = frame_id;

        RCLCPP_INFO(this->get_logger(), "Start IOO_ROS2");
    }

    void ImuOnlyOdometryROS2::topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        get_msg = msg;
        get_msg_flag = true;
    }

    void ImuOnlyOdometryROS2::timer_callback()
    {
        if(get_msg_flag)
        {
            auto coef = M_PI / 180.0;
            auto linear_accel = getEigenVec3(get_msg->linear_acceleration.x, get_msg->linear_acceleration.y, get_msg->linear_acceleration.z);
            auto angular_vel = getEigenVec3(get_msg->angular_velocity.x*coef, get_msg->angular_velocity.y*coef, get_msg->angular_velocity.z*coef);
            
            auto estimated_posture = posture_estimater->estimate(angular_vel, linear_accel);
            auto rpy_msg = geometry_msgs::msg::Vector3();
            rpy_msg.x = estimated_posture.x();
            rpy_msg.y = estimated_posture.y();
            rpy_msg.z = estimated_posture.z() / 2.0;

            tf2::Quaternion q;
            q.setRPY(estimated_posture.x(), estimated_posture.y(), estimated_posture.z()/2.0);
            odom_msg.pose.orientation.w = q.w();
            odom_msg.pose.orientation.x = q.x();
            odom_msg.pose.orientation.y = q.y();
            odom_msg.pose.orientation.z = q.z();

            if(enable_pose_estimate)
            {
                const auto calc_pose = CalcPose(linear_accel, q, 0.001);
                odom_msg.pose.position.x += calc_pose.x();
                odom_msg.pose.position.y += calc_pose.y();
            }

            publisher_->publish(odom_msg);
            rpy_publisher_->publish(rpy_msg);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Can't get IMU Message on topic : %s", this->subscriber_->get_topic_name());
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ioo_ros2::ImuOnlyOdometryROS2)