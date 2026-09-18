#include "usv_bt/gps_anchor_ready_condition.hpp"

namespace usv_bt {
    GpsAnchorReady::GpsAnchorReady(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config) {
        node_ = rclcpp::Node::make_shared("gps_anchor_ready_bt_node");
        executor_.add_node(node_);
        sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/channel_nav/anchor_ready", rclcpp::QoS(1).transient_local(),
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                anchor_ready_ = msg->data;
            });
    }

    BT::NodeStatus GpsAnchorReady::tick() {
        // Spin with a short timeout so the transient_local latch is delivered
        // even if the publisher started before this node subscribed.
        executor_.spin_some(std::chrono::milliseconds(20));
        return anchor_ready_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
}
