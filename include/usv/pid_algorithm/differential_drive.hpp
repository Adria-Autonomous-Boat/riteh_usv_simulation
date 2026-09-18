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
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

class DifferentialDrive : public rclcpp::Node
{
public:
    DifferentialDrive();

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_sub;

    //Differential guidance node lifecycle controller
    static constexpr char const * _controlled_node_name = "differential_guidance";
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> _change_state_client;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> _get_state_client;

    rclcpp::TimerBase::SharedPtr _delay_timer;


    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus &msg);
    void sendChangeStateRequest(const uint8_t transition_id);
    void sendGetStateRequest();

    void stateChangeCallback(rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future);
    void getStateCallback(rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future);

    bool _change_arm_mode = false;
    bool _change_nav_mode = true;
    bool _req_transition_success = false;
    bool _req_state_success = false;

    uint8_t _current_state{lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED};


};