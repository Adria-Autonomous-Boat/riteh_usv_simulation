

#include "usv/differential_drive_kinematics.hpp"

DifferentialDriveKinematics::DifferentialDriveKinematics() 
: Node("differential_drive_kinematics")
{
    
    _cmd_vel_sub =
        this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(
                &DifferentialDriveKinematics::computeNormalizedMotorCommands, this, std::placeholders::_1
            )
        );

    _actuator_servos_pub = 
        this->create_publisher<px4_msgs::msg::ActuatorServos>("/actuator_servos", qos);

}


void DifferentialDriveKinematics::computeNormalizedMotorCommands(const geometry_msgs::msg::Twist &msg)
{

    const float yaw_rate_setpoint = msg.angular.z;
    const float desired_forward_speed = msg.linear.x;

    // Yaw rate control
	float speed_diff_normalized{0.f};

    if (__builtin_isfinite(yaw_rate_setpoint)) {
        // Feedforward
		const float speed_diff = yaw_rate_setpoint * _engine_separation_width / 2.f;
		speed_diff_normalized = math::interpolate<float>(speed_diff, -_max_thr_yaw_rate, 
            _max_thr_yaw_rate, -1.f, 1.f
		);

    } else {
		speed_diff_normalized = 0.f;
	}

    // Speed control
	float forward_speed_normalized{0.f};

	if (__builtin_isfinite(desired_forward_speed)) {
		// Feedforward
		forward_speed_normalized = math::interpolate<float>(desired_forward_speed,
				-_max_thr_lin_vel, _max_thr_lin_vel,
				-1.f, 1.f
		);

	} else {
		forward_speed_normalized = 0.f;
	}

    calculateInverseKinematics(forward_speed_normalized, speed_diff_normalized);
}

void DifferentialDriveKinematics::calculateInverseKinematics(
    float forward_speed_normalized, const float speed_diff_normalized
)
{

    float max_motor_command = fabsf(forward_speed_normalized) + fabsf(speed_diff_normalized);

	if (max_motor_command > 1.f) { // Prioritize yaw rate if a normalized motor command exceeds limit of 1
		float excess = fabsf(max_motor_command - 1.f);
		forward_speed_normalized -= math::sign(forward_speed_normalized) * excess;
	}

    // Calculate the left and right engine speeds
    float left_engine_speed = forward_speed_normalized - speed_diff_normalized;
	float right_engine_speed = forward_speed_normalized + speed_diff_normalized;

    actuator_servos.control[0] = left_engine_speed;
    actuator_servos.control[1] = right_engine_speed;


    _actuator_servos_pub->publish(actuator_servos);
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DifferentialDriveKinematics>());
    rclcpp::shutdown();
    return 0;
}