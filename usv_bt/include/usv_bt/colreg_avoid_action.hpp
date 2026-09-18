#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>
#include <thread>
#include <chrono>
#include <deque>
#include <vector>

namespace usv_bt {
    enum class ColregRule { NONE, HEAD_ON, CROSSING_FROM_RIGHT, CROSSING_FROM_LEFT };

    class ColregAvoid : public BT::StatefulActionNode {
    public:
        ColregAvoid(const std::string &name, const BT::NodeConfig &config);

        ~ColregAvoid();

        static BT::PortsList providedPorts() {
            return {
                BT::InputPort<double>("starboard_offset", 5.0,
                                      "Lateral offset (m) to starboard for HEAD-ON avoidance waypoint"),
                BT::InputPort<double>("yield_offset", 7.0,
                                      "Lateral offset (m) to starboard for GIVE-WAY sidestep"),
                BT::InputPort<double>("yield_clearance", 3.0,
                                      "Distance (m) to fall behind Otter's stern along its track"),
                BT::InputPort<double>("replan_interval_s", 1.0, "Seconds between waypoint replans"),
                BT::InputPort<double>("emergency_range", 10.0,
                                      "Distance (m) at which CROSSING_FROM_LEFT triggers clockwise arc avoidance"),
                BT::InputPort<double>("arc_radius", 3.5,
                                      "Radius (m) of the clockwise emergency arc for CROSSING_FROM_LEFT"),
            };
        }

        BT::NodeStatus onStart() override;

        BT::NodeStatus onRunning() override;

        void onHalted() override;

    private:
        using NavToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandle = rclcpp_action::ClientGoalHandle<NavToPose>;

        rclcpp::Node::SharedPtr node_;
        rclcpp::executors::SingleThreadedExecutor executor_;
        std::thread spin_thread_;

        rclcpp_action::Client<NavToPose>::SharedPtr nav_client_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr vessel_sub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pause_pub_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::mutex vessel_mutex_;
        geometry_msgs::msg::PointStamped::SharedPtr vessel_pos_;

        // Rolling history of Otter detections used to estimate its heading
        struct VesselSample {
            double x, y;
            std::chrono::steady_clock::time_point t;
        };

        std::deque<VesselSample> vessel_history_;
        static constexpr size_t HISTORY_SIZE = 8; // ~1s at 6-8 Hz
        static constexpr double MIN_MOVEMENT = 0.3; // m — discard stationary noise

        GoalHandle::SharedPtr goal_handle_;
        std::mutex goal_mutex_;

        bool goal_sent_{false};
        bool goal_done_{false};
        bool goal_succeeded_{false};

        double initial_yaw_{0.0};
        double avoidance_x_{0.0};
        double avoidance_y_{0.0};
        std::chrono::steady_clock::time_point last_replan_time_{};

        // Clockwise emergency arc for CROSSING_FROM_LEFT
        struct Waypoint {
            double x, y;
        };

        std::vector<Waypoint> arc_waypoints_;
        size_t arc_wp_index_{0};

        // CPA-based rule classification
        // otter_yaw: estimated Otter heading in radians (NaN if unknown)
        ColregRule classify_situation(
            double bx, double by, double byaw,
            double vx, double vy, double otter_yaw) const;

        bool send_nav_goal(double sx, double sy, double yaw);

        void publish_pause(bool paused);

        // Estimates Otter heading from position history. Returns false if insufficient data.
        // Must be called with vessel_mutex_ already held.
        bool estimate_otter_heading(double &heading_out) const;

        // Returns true if vessel is closing (range decreasing), false if departing.
        // Must be called with vessel_mutex_ already held.
        bool is_vessel_closing(double bx, double by) const;

        // heading threshold parameters (radians)
        static constexpr double HEAD_ON_HALF_ANGLE = 0.175; // ±10° — tighter head-on zone, wider crossing arcs
        static constexpr double CROSSING_HALF_ANGLE = 1.963; // ±112.5° (sidelight arc)
        static constexpr double DEPARTING_RANGE_RATE = 0.2; // m/s — above this = departing
    };
}
