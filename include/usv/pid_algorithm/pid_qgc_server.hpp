
#pragma once

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/position_setpoint_triplet.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/goto_setpoint.hpp"

#include <Eigen/Eigen>

#include "libs/geo.hpp"
#include "libs/frame_transforms.hpp"
#include "utils/math_funcs.hpp"


class PIDQGCServer : public rclcpp::Node
{

public:
    PIDQGCServer();

private:

    rclcpp::Publisher<px4_msgs::msg::GotoSetpoint>::SharedPtr _goto_setpoint_pub;

    rclcpp::Subscription<px4_msgs::msg::PositionSetpointTriplet>::SharedPtr _qground_position_setpoint_triplet;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _vehicle_local_position_sub;


    void geoToCartesianTransformInit(const px4_msgs::msg::VehicleLocalPosition &msg);
    void QGroundPositionSetpointCallback(const px4_msgs::msg::PositionSetpointTriplet &msg);

    // Transform global to ned coordinates
    MapProjection _global_ned_proj_ref{};

    px4_msgs::msg::GotoSetpoint _goto_setpoint;

    Eigen::Vector2f _curr_pos{};
    Eigen::Vector2f _prev_wp{};
	Eigen::Vector2f _curr_wp{};
    Eigen::Vector2f _next_wp{};


};