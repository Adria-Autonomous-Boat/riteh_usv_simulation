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

#include "usv/pid_algorithm/differential_guidance.hpp"


DifferentialGuidance::DifferentialGuidance()
: LifecycleNode("differential_guidance", rclcpp::NodeOptions().use_intra_process_comms(false))
{   

    initParameters();

    _task_done_pub = 
        this->create_publisher<std_msgs::msg::Bool>("/task_finished", qos);

    _guidance_timer =
        this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&DifferentialGuidance::computeGuidance, this)
        );

    _vehicle_odom_sub = 
        this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", 
            qos,
            std::bind(
                &DifferentialGuidance::vehicleOdometryCallback, this, std::placeholders::_1
            )
        );


    _goto_setpoint_sub = 
        this->create_subscription<px4_msgs::msg::GotoSetpoint>(
            "/fmu/in/goto_setpoint", 
            qos,
            std::bind(
                &DifferentialGuidance::gotoSetpointCallback, this, std::placeholders::_1
            )
        );

    _mode_completed_sub =
        this->create_subscription<px4_msgs::msg::ModeCompleted>(
            "/fmu/out/mode_completed", 
            qos,
            [this](const px4_msgs::msg::ModeCompleted &msg) {
                // Used for activating MISSION MODE done condition
                _currentState = GuidanceState::GOAL_REACHED;

                auto task_done_message = std_msgs::msg::Bool();
                task_done_message.data = true;
                _task_done_pub->publish(task_done_message);
                
                (void)msg;
            }
        );


    _differential_drive_setpoint_pub = 
        this->create_publisher<px4_msgs::msg::DifferentialDriveSetpoint>("/fmu/in/differential_drive_setpoint", qos);

    _setpoint_triplet_update_pub = 
        this->create_publisher<std_msgs::msg::Bool>("/update_setpoint_triplet", qos);

    
}


void DifferentialGuidance::gotoSetpointCallback(const px4_msgs::msg::GotoSetpoint &msg)
{

    if (__builtin_isfinite(msg.position[0])
        && __builtin_isfinite(msg.position[1])) {
        _curr_wp = Eigen::Vector2f(
            msg.position[0], msg.position[1]
        );

        if (msg.flag_control_heading) {
            _waypoint_transition_angle = msg.heading;
        }
        else {
            _waypoint_transition_angle = 0.0;
        }


    }
    else {
        _currentState = GuidanceState::GOAL_REACHED;
    }
}


void DifferentialGuidance::vehicleOdometryCallback(const px4_msgs::msg::VehicleOdometry &msg)
{
    _curr_pos = Eigen::Vector2f(msg.position[0], msg.position[1]);
    Eigen::Vector3d velocity_in_local_frame(msg.velocity[0], msg.velocity[1], msg.velocity[2]);
    _vehicle_attitude_quaternion = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(msg.q);
    
    // Velocity data rotation from NED earth frame to FRD body frame
    Eigen::Vector3f velocity_in_body_frame = px4_ros_com::frame_transforms::ned_to_aircraft_frame(velocity_in_local_frame, _vehicle_attitude_quaternion).cast<float>();

    _vehicle_forward_speed = sqrt(velocity_in_body_frame(0)*velocity_in_body_frame(0) + velocity_in_body_frame(1)*velocity_in_body_frame(1));

    _vehicle_forward_speed = fabsf(_vehicle_forward_speed) > SPEED_THRESHOLD ? _vehicle_forward_speed : 0.f;

    _vehicle_yaw_rate = msg.angular_velocity[2];
    _vehicle_yaw_rate = fabsf(_vehicle_yaw_rate) > YAW_RATE_THRESHOLD ? _vehicle_yaw_rate : 0.f;

    _vehicle_yaw = static_cast<float>(px4_ros_com::frame_transforms::utils::quaternion::quaternion_get_yaw(_vehicle_attitude_quaternion));
}

void DifferentialGuidance::computeGuidance()
{   

    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        return;
    

    hrt_abstime now = hrt_absolute_time();
    const float dt = math::min((now - _time_stamp_last), 5000_ms) / 1e6f;
    _time_stamp_last = now;
    

    const float distance_to_curr_wp = get_distance_to_next_waypoint_cartesian(_curr_pos(0), _curr_pos(1),
                                        _curr_wp(0), _curr_wp(1));

    float desired_yaw = get_bearing_to_next_waypoint_cartesian(_curr_pos(0), _curr_pos(1), 
                                        _curr_wp(0), _curr_wp(1));

    float heading_error = math::wrap_pi(desired_yaw - _vehicle_yaw);


    /************  State machine  ***************/
    if (distance_to_curr_wp <= _param_nav_acc_rad && _trigger_setpoint_update) {
        _trigger_setpoint_update = false;

        auto update_setpoint_msg = std_msgs::msg::Bool();
        update_setpoint_msg.data = true;
        _setpoint_triplet_update_pub->publish(update_setpoint_msg);
    }
    else if (distance_to_curr_wp > _param_nav_acc_rad && !_trigger_setpoint_update) {
        _trigger_setpoint_update = true;
    }


    if (_currentState == GuidanceState::DRIVING && fabsf(heading_error) > _param_rd_trans_drv_trn) {
        _currentState = GuidanceState::TURNING;

    } else if (_currentState == GuidanceState::TURNING && fabsf(heading_error) < _param_rd_trans_trn_drv) {
        _currentState = GuidanceState::DRIVING;
    }


    float desired_forward_speed{0.f};

    switch (_currentState) {
    case GuidanceState::TURNING:
        if (_vehicle_forward_speed > TURN_MAX_VELOCITY) {
            desired_yaw = _vehicle_yaw; // Wait for the rover to stop
        }

        break;

    case GuidanceState::DRIVING: {

        desired_forward_speed = _max_forward_speed; // Drive at max speed on the straight

        //If there is a sharper turn (angle between waypoints), then slow down before the turn
        if (_waypoint_transition_angle < M_PI_FLOAT - _param_rd_trans_drv_trn) {
            desired_forward_speed = trajectory::computeMaxSpeedFromDistance(_param_rdd_max_jerk,
                    _param_rdd_max_accel, distance_to_curr_wp, _final_speed_from_distance);
            desired_forward_speed = math::constrain(desired_forward_speed, -_max_forward_speed, _max_forward_speed);
        }
        break;
    }

    case GuidanceState::GOAL_REACHED:
        if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        }
        _currentState = GuidanceState::DRIVING;
        desired_forward_speed = NAN;
        desired_yaw = NAN;
        break;

    }

    computeNormalizedMotorCommands(desired_yaw, desired_forward_speed, dt);
}

void DifferentialGuidance::computeNormalizedMotorCommands(
    const float desired_yaw, const float desired_forward_speed, const float dt
) {

    float yaw_rate_setpoint{NAN};

    // Closed loop yaw control (Overrides yaw rate setpoint)
    if (__builtin_isfinite(desired_yaw)) {
        float heading_error = math::wrap_pi(desired_yaw - _vehicle_yaw);
        yaw_rate_setpoint = pid_calculate(&_pid_yaw, heading_error, 0, 0, dt);
    }
    

    // Yaw rate control
    float speed_diff_normalized{0.f};

    if (__builtin_isfinite(yaw_rate_setpoint)) {
        // Feedforward
        const float speed_diff = yaw_rate_setpoint * _engine_separation_width / 2.f;
        speed_diff_normalized = math::interpolate<float>(speed_diff, -_param_rd_max_thr_yaw_r, 
                _param_rd_max_thr_yaw_r, -1.f, 1.f
        );

        float speed_diff_correction_normalized = math::interpolate<float>(
            pid_calculate(&_pid_yaw_rate, yaw_rate_setpoint, _vehicle_yaw_rate, 0, dt), 
            -_param_rd_max_thr_yaw_r, _param_rd_max_thr_yaw_r,
            -1.f, 1.f
        );
        
        speed_diff_normalized = math::constrain(speed_diff_normalized + 
            speed_diff_correction_normalized, -1.f, 1.f
        ); // Feedback

    } else {
        speed_diff_normalized = 0.f;
    }

    // Speed control
    float forward_speed_normalized{0.f};

    if (__builtin_isfinite(desired_forward_speed)) {
        // Feedforward
        forward_speed_normalized = math::interpolate<float>(desired_forward_speed,
                -_param_rd_max_thr_spd, _param_rd_max_thr_spd,
                -1.f, 1.f
        );
        
        float forward_speed_correction_normalized = math::interpolate<float>(
            pid_calculate(&_pid_throttle, 
                desired_forward_speed, _vehicle_forward_speed, 0, dt
            ), 
            -_param_rd_max_thr_spd, _param_rd_max_thr_spd,
            -1.f, 1.f
        );
        
        forward_speed_normalized = math::constrain(forward_speed_normalized + 
            forward_speed_correction_normalized, -1.f, 1.f
        ); // Feedback

    } else {
        forward_speed_normalized = 0.f;
    }

    px4_msgs::msg::DifferentialDriveSetpoint differential_drive_control_output;

    differential_drive_control_output.speed = forward_speed_normalized;
    differential_drive_control_output.yaw_rate = speed_diff_normalized;
    differential_drive_control_output.timestamp = hrt_absolute_time();

    _differential_drive_setpoint_pub->publish(differential_drive_control_output);
}


/************** Lifecycle state transitions *****************/

LifecycleCallbackReturn DifferentialGuidance::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "on_configure() is called.");

    pid_init(&_pid_yaw_rate, PID_MODE_DERIVATIV_NONE, 0.001f);
    pid_init(&_pid_throttle, PID_MODE_DERIVATIV_NONE, 0.001f);
    pid_init(&_pid_yaw, PID_MODE_DERIVATIV_NONE, 0.001f);

    pid_set_parameters(&_pid_yaw_rate,
        _param_rd_yaw_rate_p, // Proportional gain
        _param_rd_yaw_rate_i, // Integral gain
        0.f, // Derivative gain
        _param_rd_max_thr_yaw_r, // Integral limit
        _param_rd_max_thr_yaw_r); // Output limit
    pid_set_parameters(&_pid_throttle,
        _param_rd_p_gain_speed, // Proportional gain
        _param_rd_i_gain_speed, // Integral gain
        0.f, // Derivative gain
        _param_rd_max_thr_spd, // Integral limit
        _param_rd_max_thr_spd); // Output limit
    pid_set_parameters(&_pid_yaw,
        _param_rd_p_gain_yaw,  // Proportional gain
        _param_rd_i_gain_yaw,  // Integral gain
        0.f,  // Derivative gain
        _max_yaw_rate,  // Integral limit
        _max_yaw_rate);  // Output limit


    _differential_drive_setpoint_pub->on_activate();
    _setpoint_triplet_update_pub->on_activate();
    _task_done_pub->on_activate();

    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn DifferentialGuidance::on_activate(const rclcpp_lifecycle::State &)
{   
    RCLCPP_INFO(this->get_logger(), "on_activate() is called.");

    _currentState = GuidanceState::DRIVING;

    // Start the algorithm callback
    _guidance_timer->reset();

    auto update_setpoint_msg = std_msgs::msg::Bool();
    update_setpoint_msg.data = true;
    _setpoint_triplet_update_pub->publish(update_setpoint_msg);

    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn DifferentialGuidance::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "on_deactivate() is called.");

    // Stop the algorithm callback
    _guidance_timer->cancel();

    // Reset state machine and turn off engines
    _currentState = GuidanceState::DRIVING;
    computeNormalizedMotorCommands(NAN, NAN, 0.0);
    
    // Reset waypoint updater
    auto update_setpoint_msg = std_msgs::msg::Bool();
    update_setpoint_msg.data = false;
    _setpoint_triplet_update_pub->publish(update_setpoint_msg);
    
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn DifferentialGuidance::on_cleanup(const rclcpp_lifecycle::State &)
{

    RCLCPP_INFO(this->get_logger(), "on_cleanup() is called.");

    pid_reset_integral(&_pid_throttle);
    pid_reset_integral(&_pid_yaw_rate);
    pid_reset_integral(&_pid_yaw);

    _differential_drive_setpoint_pub->on_deactivate();
    _setpoint_triplet_update_pub->on_deactivate();
    _task_done_pub->on_deactivate();

    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn DifferentialGuidance::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "on_shutdown() is called.");
    return LifecycleCallbackReturn::SUCCESS;
}

void DifferentialGuidance::initParameters()
{
    // Declare and get parameters
    this->declare_parameter("engine_separation_width", _engine_separation_width);
    this->declare_parameter("max_forward_speed", _max_forward_speed);
    this->declare_parameter("max_yaw_rate", _max_yaw_rate);
    this->declare_parameter("final_speed_from_distance", _final_speed_from_distance);
    this->declare_parameter("turn_max_velocity", TURN_MAX_VELOCITY);
    this->declare_parameter("yaw_rate_threshold", YAW_RATE_THRESHOLD);
    this->declare_parameter("speed_threshold", SPEED_THRESHOLD);
    this->declare_parameter("param_rd_max_thr_spd", _param_rd_max_thr_spd);
    this->declare_parameter("param_rd_max_thr_yaw_r", _param_rd_max_thr_yaw_r);
    this->declare_parameter("param_rdd_max_jerk", _param_rdd_max_jerk);
    this->declare_parameter("param_rdd_max_accel", _param_rdd_max_accel);
    this->declare_parameter("param_rd_trans_drv_trn", _param_rd_trans_drv_trn);
    this->declare_parameter("param_rd_trans_trn_drv", _param_rd_trans_trn_drv);
    this->declare_parameter("param_rd_yaw_rate_p", _param_rd_yaw_rate_p);
    this->declare_parameter("param_rd_yaw_rate_i", _param_rd_yaw_rate_i);
    this->declare_parameter("param_rd_p_gain_speed", _param_rd_p_gain_speed);
    this->declare_parameter("param_rd_i_gain_speed", _param_rd_i_gain_speed);
    this->declare_parameter("param_rd_p_gain_yaw", _param_rd_p_gain_yaw);
    this->declare_parameter("param_rd_i_gain_yaw", _param_rd_i_gain_yaw);
    this->declare_parameter("param_nav_acc_rad", _param_nav_acc_rad);

    _engine_separation_width = static_cast<float>(this->get_parameter("engine_separation_width").as_double());
    _max_forward_speed = static_cast<float>(this->get_parameter("max_forward_speed").as_double());
    _max_yaw_rate = static_cast<float>(this->get_parameter("max_yaw_rate").as_double());
    _final_speed_from_distance = static_cast<float>(this->get_parameter("final_speed_from_distance").as_double());
    TURN_MAX_VELOCITY = static_cast<float>(this->get_parameter("turn_max_velocity").as_double());
    YAW_RATE_THRESHOLD = static_cast<float>(this->get_parameter("yaw_rate_threshold").as_double());
    SPEED_THRESHOLD = static_cast<float>(this->get_parameter("speed_threshold").as_double());
    _param_rd_max_thr_spd = static_cast<float>(this->get_parameter("param_rd_max_thr_spd").as_double());
    _param_rd_max_thr_yaw_r = static_cast<float>(this->get_parameter("param_rd_max_thr_yaw_r").as_double());
    _param_rdd_max_jerk = static_cast<float>(this->get_parameter("param_rdd_max_jerk").as_double());
    _param_rdd_max_accel = static_cast<float>(this->get_parameter("param_rdd_max_accel").as_double());
    _param_rd_trans_drv_trn = static_cast<float>(this->get_parameter("param_rd_trans_drv_trn").as_double());
    _param_rd_trans_trn_drv = static_cast<float>(this->get_parameter("param_rd_trans_trn_drv").as_double());
    _param_rd_yaw_rate_p = static_cast<float>(this->get_parameter("param_rd_yaw_rate_p").as_double());
    _param_rd_yaw_rate_i = static_cast<float>(this->get_parameter("param_rd_yaw_rate_i").as_double());
    _param_rd_p_gain_speed = static_cast<float>(this->get_parameter("param_rd_p_gain_speed").as_double());
    _param_rd_i_gain_speed = static_cast<float>(this->get_parameter("param_rd_i_gain_speed").as_double());
    _param_rd_p_gain_yaw = static_cast<float>(this->get_parameter("param_rd_p_gain_yaw").as_double());
    _param_rd_i_gain_yaw = static_cast<float>(this->get_parameter("param_rd_i_gain_yaw").as_double());
    _param_nav_acc_rad = static_cast<float>(this->get_parameter("param_nav_acc_rad").as_double());
}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DifferentialGuidance>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
