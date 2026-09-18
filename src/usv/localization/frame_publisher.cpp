
#include "usv/localization/frame_publisher.hpp"


FramePublisher::FramePublisher() 
: Node("frame_publisher")
{

    auto qos = rclcpp::QoS(1).best_effort();

    _vehicle_odometry = 
        this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", 
            qos,
            std::bind(
                &FramePublisher::vehicleOdometryTransform, this, std::placeholders::_1
            )
        );

    _odom_pub = 
        this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    _tf_broadcaster =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            
}

void FramePublisher::vehicleOdometryTransform(const px4_msgs::msg::VehicleOdometry &msg)
{

    Eigen::Quaterniond q = 
        px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(msg.q);

    Eigen::Quaterniond enu_q = px4_ros_com::frame_transforms::ned_to_enu_orientation(
        px4_ros_com::frame_transforms::baselink_to_aircraft_orientation(
            q
        )
    );

    Eigen::Vector3d position = Eigen::Vector3d(msg.position[0], msg.position[1], msg.position[2]);
    Eigen::Vector3d enu_position = px4_ros_com::frame_transforms::ned_to_enu_local_frame(position);

    Eigen::Vector3d velocity = Eigen::Vector3d(msg.velocity[0], msg.velocity[1], msg.velocity[2]);
    Eigen::Vector3d enu_velocity = px4_ros_com::frame_transforms::ned_to_enu_local_frame(velocity);

    builtin_interfaces::msg::Time current_time = this->get_clock()->now();

    //Odom topic data
    _odom_data.header.frame_id = "odom";
    _odom_data.header.stamp = current_time;
    _odom_data.child_frame_id = "base_link";
    _odom_data.pose.pose.position.x = enu_position.x();
    _odom_data.pose.pose.position.y = enu_position.y();
    _odom_data.pose.pose.position.z = 0.0;

    _odom_data.pose.pose.orientation.x = (float)enu_q.x();
    _odom_data.pose.pose.orientation.y = (float)enu_q.y();
    _odom_data.pose.pose.orientation.z = (float)enu_q.z();
    _odom_data.pose.pose.orientation.w = (float)enu_q.w();

    _odom_data.twist.twist.linear.x = enu_velocity.x();
    _odom_data.twist.twist.linear.y = enu_velocity.y();
    _odom_data.twist.twist.linear.z = enu_velocity.z();

    _odom_data.twist.twist.angular.x = msg.angular_velocity[0];
    _odom_data.twist.twist.angular.y = -msg.angular_velocity[1];
    _odom_data.twist.twist.angular.z = -msg.angular_velocity[2];

    // Odom transform

    _odom_transform.header.frame_id = "odom";
    _odom_transform.header.stamp = current_time;
    _odom_transform.child_frame_id = "base_link";

    _odom_transform.transform.translation.x = enu_position.x();
    _odom_transform.transform.translation.y = enu_position.y();
    _odom_transform.transform.translation.z = 0.0;

    _odom_transform.transform.rotation.x = (float)enu_q.x();
    _odom_transform.transform.rotation.y = (float)enu_q.y();
    _odom_transform.transform.rotation.z = (float)enu_q.z();
    _odom_transform.transform.rotation.w = (float)enu_q.w();

    _tf_broadcaster->sendTransform(_odom_transform);
    _odom_pub->publish(_odom_data);

    // Map --> Odom transform

    _odom_transform.header.frame_id = "map";
    _odom_transform.header.stamp = current_time;
    _odom_transform.child_frame_id = "odom";

    _odom_transform.transform.translation.x = 0.0;
    _odom_transform.transform.translation.y = 0.0;
    _odom_transform.transform.translation.z = 0.0;

    _odom_transform.transform.rotation.x = 0.0;
    _odom_transform.transform.rotation.y = 0.0;
    _odom_transform.transform.rotation.z = 0.0;
    _odom_transform.transform.rotation.w = 1.0;

    _tf_broadcaster->sendTransform(_odom_transform);

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}