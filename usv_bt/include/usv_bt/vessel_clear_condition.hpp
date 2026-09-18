#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>
#include <thread>
#include <chrono>
#include <deque>

namespace usv_bt {
    class VesselClear : public BT::ConditionNode {
    public:
        VesselClear(const std::string &name, const BT::NodeConfig &config);

        ~VesselClear();

        static BT::PortsList providedPorts() {
            return {
                BT::InputPort<double>("clear_range", 25.0, "Distance (m) at which vessel is considered cleared"),
            };
        }

        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::executors::SingleThreadedExecutor executor_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr vessel_sub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pause_pub_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::mutex detection_mutex_;
        geometry_msgs::msg::PointStamped::SharedPtr last_detection_;
        std::chrono::steady_clock::time_point last_received_at_;

        struct VesselSample {
            double x, y;
            std::chrono::steady_clock::time_point t;
        };

        std::deque<VesselSample> vessel_history_;
        static constexpr size_t HISTORY_SIZE = 10;

        std::thread spin_thread_;
    };
}
