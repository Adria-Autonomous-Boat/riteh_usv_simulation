
#include "usv/nav2_qgc_server.hpp"

Nav2QGCServer::Nav2QGCServer()
: Node("nav2_qgc_server")
{
    auto qos = rclcpp::QoS(1).best_effort();

    _callback_group = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant
    );

    _options.callback_group = _callback_group;

    _vehicle_local_position_sub = 
        this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", 
            qos,
            std::bind(
                &Nav2QGCServer::geoToCartesianTransformInit, this, std::placeholders::_1
            )
        );

    _vehicle_status_sub =
        this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", 
            qos,
            std::bind(
                &Nav2QGCServer::controllerStateChange, this, std::placeholders::_1
            )
        );

    _nav2_follow_waypoints_client = 
        rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
            this, "follow_waypoints", _callback_group
        );
}


void Nav2QGCServer::geoToCartesianTransformInit(const px4_msgs::msg::VehicleLocalPosition &msg)
{
    if (!_global_ned_proj_ref.isInitialized() || 
        (_global_ned_proj_ref.getProjectionReferenceTimestamp() != msg.ref_timestamp)) 
    {
        _global_ned_proj_ref.initReference(msg.ref_lat, msg.ref_lon, msg.ref_timestamp);
    }

    _curr_pos = px4_ros_com::frame_transforms::ned_to_enu_local_frame(
        Eigen::Vector3d(msg.x, msg.y, 0.0)
    ); 
    
}


void Nav2QGCServer::controllerStateChange(const px4_msgs::msg::VehicleStatus &msg)
{

    bool armed = (msg.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);

    if (armed && !_loaded_initial_waypoints) {
    
        _loaded_initial_waypoints = true;
        loadQGCWaypoints();

    }
    else if (!armed && _loaded_initial_waypoints) {

        _loaded_initial_waypoints = false;
    }


    if(armed) {
        _change_arm_mode = true;
        switch (msg.nav_state) {
            case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION: // For QGC simulation
                if (_change_nav_mode) {

                    _change_nav_mode = false;

                    if (_nav2_follow_waypoints_client->action_server_is_ready()) {

                        auto goal_msg = nav2_msgs::action::FollowWaypoints::Goal();
                        goal_msg.poses = _enu_waypoint_poses;
                        RCLCPP_INFO(this->get_logger(), "Sending waypoints to Nav2...");

                        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
                        _nav2_follow_waypoints_client->async_send_goal(goal_msg, send_goal_options);
                    } 
                    else {
                        RCLCPP_INFO(this->get_logger(), "Nav2 server is not ready...");
                    }

                }

                break;
            case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER:
                if (!_change_nav_mode) {

                    _change_nav_mode = true;

                    RCLCPP_INFO(this->get_logger(), "Cancelling Nav2 goals...");
                    _nav2_follow_waypoints_client->async_cancel_all_goals();
                }
                
                break;
        }
    }
    else if (!armed && _change_arm_mode) {

        _change_arm_mode = false;

        RCLCPP_INFO(this->get_logger(), "Cancelling Nav2 goals...");
        _nav2_follow_waypoints_client->async_cancel_all_goals();
    }


}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<Nav2QGCServer>();
    
    // Use single-threaded executor instead of rclcpp::spin()
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}