/****************************************************************************
 *
 *   Copyright (c) 2023-2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/differential_drive_setpoint.hpp"
#include "px4_msgs/msg/goto_setpoint.hpp"
#include "px4_msgs/msg/mode_completed.hpp"

#include <Eigen/Eigen>
#include "libs/frame_transforms.hpp"
#include "libs/pid.hpp"
#include "libs/geo.hpp"

#include <utils/math_funcs.hpp>
#include <utils/timer.hpp>


using namespace time_literals;

typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleCallbackReturn;

enum class GuidanceState {
	TURNING, ///< The vehicle is currently turning.
	DRIVING, ///< The vehicle is currently driving straight.
	GOAL_REACHED ///< The vehicle has reached its goal.
};

class DifferentialGuidance : public rclcpp_lifecycle::LifecycleNode
{
public:
    DifferentialGuidance();

    void initParameters();

    void computeGuidance();

    void computeNormalizedMotorCommands(
        const float desired_yaw, const float desired_forward_speed, const float dt
    );

    /**
    * @brief Lifecycle node state transition callbacks
    */
    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &);
    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &);
    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

private:
    rclcpp::QoS qos = rclcpp::QoS(1).best_effort();

    rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::DifferentialDriveSetpoint>::SharedPtr _differential_drive_setpoint_pub;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr _setpoint_triplet_update_pub;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr _task_done_pub;

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _vehicle_odom_sub;
    rclcpp::Subscription<px4_msgs::msg::GotoSetpoint>::SharedPtr _goto_setpoint_sub;
    rclcpp::Subscription<px4_msgs::msg::ModeCompleted>::SharedPtr _mode_completed_sub;

    rclcpp::TimerBase::SharedPtr _guidance_timer;

    void vehicleOdometryCallback(const px4_msgs::msg::VehicleOdometry &msg);
    void gotoSetpointCallback(const px4_msgs::msg::GotoSetpoint &msg);


    hrt_abstime _time_stamp_last{0}; /**< time stamp when task was last updated */

    ////////////// CONTROL ////////////////
    Eigen::Quaterniond _vehicle_attitude_quaternion{};
    float _vehicle_yaw{0.f};

	float _vehicle_yaw_rate{0.f};
	float _vehicle_forward_speed{0.f};

    PID_t _pid_yaw_rate; // The PID controller for the closed loop yaw rate control
    PID_t _pid_throttle; // The PID controller for the closed loop speed control
    PID_t _pid_yaw; // The PID controller for the closed loop yaw control

    // Parameters
    float _engine_separation_width{1.f};
    float _max_forward_speed{3.3f}; ///< The maximum speed.
    float _max_yaw_rate{1.5f}; ///< The maximum angular velocity.
    float _final_speed_from_distance{2.2f};

    float _param_rd_max_thr_spd{3.3f};
    float _param_rd_max_thr_yaw_r{1.5f};

    float _param_rd_yaw_rate_p{0.6f};
    float _param_rd_yaw_rate_i{0.f};

    float _param_rd_p_gain_speed{0.5f};
    float _param_rd_i_gain_speed{0.1f};

    float _param_rd_p_gain_yaw{5.f};
    float _param_rd_i_gain_yaw{0.f};

    float _param_rd_trans_drv_trn{0.349066};
    float _param_rd_trans_trn_drv{0.174533};

    float _param_rdd_max_jerk{6.f};     //Limit for forwards acc/deceleration change (m/s^3).
    float _param_rdd_max_accel{2.f};

    float _param_nav_acc_rad{10.0}; // Radius in meters around a waypoint when it is considered reached


    float TURN_MAX_VELOCITY = 2.2f; // Velocity threshhold for starting the spot turn [m/s]
    float YAW_RATE_THRESHOLD = 0.02f; // [rad/s] The minimum threshold for the yaw rate measurement not to be interpreted as zero
    float SPEED_THRESHOLD = 0.1f; // [m/s] The minimum threshold for the speed measurement not to be interpreted as zero

    ////////////// GUIDANCE ////////////////

    GuidanceState _currentState{GuidanceState::DRIVING}; ///< The current state of guidance.

    // Waypoints
    Eigen::Vector2f _curr_pos{};
    Eigen::Vector2f _curr_wp{};
    float _waypoint_transition_angle{0.f}; // Angle between the prevWP-currWP and currWP-nextWP line segments [rad]

    bool _trigger_setpoint_update{true};
};

