#include "usv_bt/northing_above_condition.hpp"
#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace usv_bt {
    NorthingAbove::NorthingAbove(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config) {
        rclcpp::NodeOptions opts;
        opts.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
        node_ = rclcpp::Node::make_shared("northing_above_bt_node", opts);
        executor_.add_node(node_);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_);

        spin_thread_ = std::thread([this]() { executor_.spin(); });
    }

    NorthingAbove::~NorthingAbove() {
        executor_.cancel();
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
    }

    BT::NodeStatus NorthingAbove::tick() {
        double min_y = 16.0;
        getInput("min_y", min_y);

        try {
            auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            double by = tf.transform.translation.y;
            return (by >= min_y) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        } catch (const tf2::TransformException &) {
            return BT::NodeStatus::FAILURE;
        }
    }
}
