

#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "px4_msgs/msg/actuator_servos.hpp"

#include <utils/math_funcs.hpp>


class DifferentialDriveKinematics : public rclcpp::Node
{
public:
    DifferentialDriveKinematics();

private:
    rclcpp::QoS qos = rclcpp::QoS(1).best_effort();

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
    rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr _actuator_servos_pub;

    /**
    * @brief Calculate normalized motor commands using PID controllers
    */
    void computeNormalizedMotorCommands(const geometry_msgs::msg::Twist &msg);

    /**
    * @brief Calculate differential drive inverse kinematics
    */
    void calculateInverseKinematics(float forward_speed_normalized, const float speed_diff_normalized);


    px4_msgs::msg::ActuatorServos actuator_servos{};

    // Vehicle parameters
    float _engine_separation_width{2.1f};
    float _max_thr_lin_vel{5.0f};
    float _max_thr_yaw_rate{1.0f};



};
