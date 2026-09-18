/****************************************************************************
 *
 *   Copyright (C) 2023-2024 PX4 Development Team. All rights reserved.
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

#include "usv/pid_algorithm/differential_kinematics.hpp"



DifferentialKinematics::DifferentialKinematics() 
: Node("differential_kinematics")
{
    auto qos = rclcpp::QoS(1).best_effort();

    _differential_drive_control_output_sub = 
        this->create_subscription<px4_msgs::msg::DifferentialDriveSetpoint>(
            "/fmu/in/differential_drive_setpoint", 
            qos,
            std::bind(
                &DifferentialKinematics::inverseKinematicsCallback, this, std::placeholders::_1
            )
        );

    _actuator_servos_pub = 
        this->create_publisher<px4_msgs::msg::ActuatorServos>("/actuator_servos", qos);
            
}


void DifferentialKinematics::inverseKinematicsCallback(const px4_msgs::msg::DifferentialDriveSetpoint &msg)
{

    hrt_abstime now = hrt_absolute_time();
    const bool setpoint_timeout = (msg.timestamp + 100_ms) < now;

    float forward_speed_normalized = msg.speed;
    const float speed_diff_normalized = msg.yaw_rate;

    float max_motor_command = fabsf(forward_speed_normalized) + fabsf(speed_diff_normalized);

	if (max_motor_command > 1.f) { // Prioritize yaw rate if a normalized motor command exceeds limit of 1
		float excess = fabsf(max_motor_command - 1.f);
		forward_speed_normalized -= math::sign(forward_speed_normalized) * excess;
	}

    // Calculate the left and right engine speeds
    float left_engine_speed = forward_speed_normalized + speed_diff_normalized;
	float right_engine_speed = forward_speed_normalized - speed_diff_normalized;



    if (setpoint_timeout) {
        left_engine_speed = 0.0;
		right_engine_speed = 0.0;
	}


    actuator_servos.control[0] = left_engine_speed;
    actuator_servos.control[1] = right_engine_speed;
    //actuator_motors.reversible_flags = _param_r_rev;
    actuator_servos.timestamp = now;

    // RCLCPP_INFO(this->get_logger(), "Right: %s; Left: %s", 
    //     std::to_string(right_engine_speed).c_str(), std::to_string(left_engine_speed).c_str());

    _actuator_servos_pub->publish(actuator_servos);
    
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DifferentialKinematics>());
  rclcpp::shutdown();
  return 0;
}