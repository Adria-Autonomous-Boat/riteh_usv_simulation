#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>
#include <thread>
#include <chrono>
#include <deque>

namespace usv_bt {
    class VesselInRange : public BT::ConditionNode {
    public:
        VesselInRange(const std::string &name, const BT::NodeConfig &config);

        ~VesselInRange();

        static BT::PortsList providedPorts() {
            return {
                BT::InputPort<double>("range_threshold", 20.0, "Distance (m) to trigger avoidance"),
                BT::InputPort<double>("min_speed", 0.3,
                                      "Minimum detection speed (m/s) to confirm a vessel vs static object"),
                BT::InputPort<double>("max_speed", 3.0,
                                      "Maximum plausible vessel speed (m/s) — faster detections are lidar noise"),
            };
        }

        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::executors::SingleThreadedExecutor executor_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        struct DetectionSample {
            double x, y;
            std::chrono::steady_clock::time_point t;
        };

        std::mutex detection_mutex_;
        geometry_msgs::msg::PointStamped::SharedPtr last_detection_;
        std::chrono::steady_clock::time_point last_received_at_;
        std::deque<DetectionSample> history_;
        static constexpr size_t HISTORY_SIZE = 10;

        std::thread spin_thread_;
        rclcpp::Clock wall_clock_{RCL_STEADY_TIME};
    };
}
