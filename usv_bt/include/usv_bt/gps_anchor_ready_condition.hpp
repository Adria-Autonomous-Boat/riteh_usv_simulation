#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace usv_bt {
    class GpsAnchorReady : public BT::ConditionNode {
    public:
        GpsAnchorReady(const std::string &name, const BT::NodeConfig &config);

        static BT::PortsList providedPorts() { return {}; }

        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::executors::SingleThreadedExecutor executor_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
        bool anchor_ready_{false};
    };
}
