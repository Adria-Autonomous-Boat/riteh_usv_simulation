
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/callback_group.hpp"

#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include <Eigen/Eigen>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mission/mission.h>

#include "libs/geo.hpp"
#include "utils/math_funcs.hpp"
#include "libs/frame_transforms.hpp"

#include "nav2_msgs/action/follow_waypoints.hpp"


class Nav2QGCServer : public rclcpp::Node
{
public:
    Nav2QGCServer();


    void loadQGCWaypoints() {

        try {

            mavsdk::Mavsdk::Configuration config(mavsdk::ComponentType::GroundStation);

            std::unique_ptr<mavsdk::Mavsdk> _mavsdk =
                std::make_unique<mavsdk::Mavsdk>(config);
            
            std::shared_ptr<mavsdk::System> _system;
            std::shared_ptr<mavsdk::Mission> _mission;

            RCLCPP_INFO(this->get_logger(), "Connecting to MAVSDK at: %s", _mavsdk_connection_url.c_str());
            
            auto result = _mavsdk->add_any_connection(_mavsdk_connection_url);
            if (result != mavsdk::ConnectionResult::Success) {
                RCLCPP_ERROR(this->get_logger(), "MAVSDK connection failed: %d", static_cast<int>(result));
                return;
            }
            
            // Wait for system to connect with timeout
            auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
            auto fut = prom.get_future();
            
            _mavsdk->subscribe_on_new_system([this, &prom, &_mavsdk]() {
                auto system = _mavsdk->systems().back();
                if (system->has_autopilot()) {
                    RCLCPP_INFO(this->get_logger(), "Autopilot discovered");
                    prom.set_value(system);
                }
            });
            
            // Wait for connection with timeout
            if (fut.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
                RCLCPP_ERROR(this->get_logger(), "No autopilot found, connection timeout!");
                return;
            }
            
            _system = fut.get();
            _mission = std::make_shared<mavsdk::Mission>(_system);
            RCLCPP_INFO(this->get_logger(), "Successfully connected to autopilot!");
            
            // Download mission from vehicle
            auto download_result = _mission->download_mission();
            if (download_result.first != mavsdk::Mission::Result::Success) {
                RCLCPP_ERROR(this->get_logger(), "Failed to download mission: %d",
                            static_cast<int>(download_result.first));
                return;
            }
            
            auto mission_plan = download_result.second;
            RCLCPP_INFO(this->get_logger(), "Downloaded mission with %zu items", 
                        mission_plan.mission_items.size());
            
            
            // Convert cartesian waypoints
            Eigen::Vector2f _waypoint_ned;
            Eigen::Vector3d _enu_position;
            _enu_waypoint_coords.clear();

            _enu_waypoint_coords.push_back(
                Eigen::Vector2f(_curr_pos(0), _curr_pos(1))
            );

            for (const auto& item : mission_plan.mission_items) {
                
                _waypoint_ned = _global_ned_proj_ref.project(
                    item.latitude_deg, item.longitude_deg
                );

                _enu_position = px4_ros_com::frame_transforms::ned_to_enu_local_frame(
                    Eigen::Vector3d(_waypoint_ned(0), _waypoint_ned(1), 0.0)
                ); 

                _enu_waypoint_coords.push_back(
                    Eigen::Vector2f(_enu_position(0), _enu_position(1))
                );
                
                
                RCLCPP_INFO(this->get_logger(), "Waypoint - Lat: %lf, Lon: %lf",
                            item.latitude_deg, item.longitude_deg);
            }

            if (!_enu_waypoint_coords.empty()) {
                interpolateCoordinates();
            }

        }
        catch(std::exception const &e) {
            RCLCPP_WARN(this->get_logger(), "MAVSDK exception: %s", e.what());
        }

    }

    void interpolateCoordinates()
    {


        for (size_t i = 1; i < _enu_waypoint_coords.size(); ++i) {
            const Eigen::Vector2f prev = _enu_waypoint_coords[i - 1];
            const Eigen::Vector2f curr = _enu_waypoint_coords[i];
            
            // Calculate Euclidean distance between consecutive points
            float dx = curr(0) - prev(0);
            float dy = curr(1) - prev(1);
            float distance = std::sqrt(dx * dx + dy * dy);
            
            if (distance > _max_waypoints_distance) {
                // Calculate number of interpolation points needed
                int num_segments = static_cast<int>(std::ceil(distance / _max_waypoints_distance));
                
                // Add interpolated points
                for (int j = 1; j < num_segments; ++j) {
                    double ratio = static_cast<double>(j) / num_segments;

                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.frame_id = "map";
                    pose.header.stamp = this->now();
                    pose.pose.position.x = prev(0) + ratio * dx;
                    pose.pose.position.y = prev(1) + ratio * dy;
                    pose.pose.position.z = 0.0;
                    pose.pose.orientation.w = 1.0;
                    pose.pose.orientation.z = 0.0;

                    _enu_waypoint_poses.push_back(pose);
                }
            }

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = this->now();
            pose.pose.position.x = curr(0);
            pose.pose.position.y = curr(1);
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            pose.pose.orientation.z = 0.0;

            _enu_waypoint_poses.push_back(pose);
            
        }
    }

private:

    std::string _mavsdk_connection_url = "udp://:14540";

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _vehicle_local_position_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_sub;

    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr _nav2_follow_waypoints_client;

    rclcpp::CallbackGroup::SharedPtr _callback_group;
    rclcpp::SubscriptionOptions _options;

    void geoToCartesianTransformInit(const px4_msgs::msg::VehicleLocalPosition &msg);
    void controllerStateChange(const px4_msgs::msg::VehicleStatus &msg);

    // Transform global to ned coordinates.
    MapProjection _global_ned_proj_ref{};

    Eigen::Vector3d _curr_pos{};
    std::vector<Eigen::Vector2f> _enu_waypoint_coords{};
    std::vector<geometry_msgs::msg::PoseStamped> _enu_waypoint_poses;

    bool _loaded_initial_waypoints{false};
    bool _change_nav_mode{true};
    bool _change_arm_mode{false};
    const float _max_waypoints_distance{40.0};

};