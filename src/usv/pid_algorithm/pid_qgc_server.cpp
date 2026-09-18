#include "usv/pid_algorithm/pid_qgc_server.hpp"

PIDQGCServer::PIDQGCServer()
: Node("pid_qgc_server")
{
    auto qos = rclcpp::QoS(1).best_effort();

    _goto_setpoint_pub =
        this->create_publisher<px4_msgs::msg::GotoSetpoint>("fmu/in/goto_setpoint", qos);


    _qground_position_setpoint_triplet =
        this->create_subscription<px4_msgs::msg::PositionSetpointTriplet>(
            "/fmu/out/position_setpoint_triplet", 
            qos,
            std::bind(
                &PIDQGCServer::QGroundPositionSetpointCallback, this, std::placeholders::_1
            )
        );


    _vehicle_local_position_sub = 
        this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", 
            qos,
            std::bind(
                &PIDQGCServer::geoToCartesianTransformInit, this, std::placeholders::_1
            )
        );
}


void PIDQGCServer::geoToCartesianTransformInit(const px4_msgs::msg::VehicleLocalPosition &msg)
{
    if (!_global_ned_proj_ref.isInitialized() || 
        (_global_ned_proj_ref.getProjectionReferenceTimestamp() != msg.ref_timestamp)) 
    {
        _global_ned_proj_ref.initReference(msg.ref_lat, msg.ref_lon, msg.ref_timestamp);
    }
    
    _curr_pos = Eigen::Vector2f(msg.x, msg.y);
}


void PIDQGCServer::QGroundPositionSetpointCallback(const px4_msgs::msg::PositionSetpointTriplet &msg)
{
    if (msg.current.valid) {

        // Global waypoint coordinates
        if (msg.current.valid && __builtin_isfinite(msg.current.lat)
            && __builtin_isfinite(msg.current.lon)) {
            _curr_wp = _global_ned_proj_ref.project(msg.current.lat, msg.current.lon);
        } 
        else {
            _curr_wp = _curr_pos;
        }

        if (msg.previous.valid && __builtin_isfinite(msg.previous.lat)
            && __builtin_isfinite(msg.previous.lon)) {
            _prev_wp = _global_ned_proj_ref.project(msg.previous.lat, msg.previous.lon);
        } 
        else {
            _prev_wp = _curr_pos;
        }

        if (msg.next.valid && __builtin_isfinite(msg.next.lat)
            && __builtin_isfinite(msg.next.lon)) {
            _next_wp = _global_ned_proj_ref.project(msg.next.lat, msg.next.lon);
        } 
        else {
            _next_wp = _curr_pos;
        }

        // Waypoint distances
        const Eigen::Vector2f curr_to_next_wp_ned = _next_wp - _curr_wp;
        const Eigen::Vector2f curr_to_prev_wp_ned = _prev_wp - _curr_wp;
        
        //Calculating the angle between the two waypoint vectors (using the dot product)
        float cosin = curr_to_prev_wp_ned.normalized().dot(curr_to_next_wp_ned.normalized());
        cosin = math::constrain<float>(cosin, -1.f, 1.f); // Protect against float precision problem
        float _waypoint_transition_angle = acosf(cosin);
        
        _goto_setpoint.position[0] = _curr_wp(0);
        _goto_setpoint.position[1] = _curr_wp(1);
        _goto_setpoint.position[2] = 0.0;

        _goto_setpoint.flag_control_heading = true;
        _goto_setpoint.heading = _waypoint_transition_angle;

        _goto_setpoint_pub->publish(_goto_setpoint);

    }

}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDQGCServer>());
    rclcpp::shutdown();
    return 0;
}