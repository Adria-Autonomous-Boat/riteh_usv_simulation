
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <Eigen/Eigen>

#include "libs/frame_transforms.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "rosgraph_msgs/msg/clock.hpp"


class FramePublisher : public rclcpp::Node
{
public:
    FramePublisher();

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _vehicle_odometry;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

    geometry_msgs::msg::TransformStamped _odom_transform;
    nav_msgs::msg::Odometry _odom_data;

    void vehicleOdometryTransform(const px4_msgs::msg::VehicleOdometry &msg);

};