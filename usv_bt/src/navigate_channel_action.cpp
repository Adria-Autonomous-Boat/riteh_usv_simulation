#include "usv_bt/navigate_channel_action.hpp"

namespace usv_bt {
    NavigateChannel::NavigateChannel(const std::string &name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config) {
        rclcpp::NodeOptions opts;
        opts.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
        node_ = rclcpp::Node::make_shared("navigate_channel_bt_node", opts);

        set_goal_client_ = node_->create_client<riteh_usv_sim::srv::SetGpsGoal>("/channel_nav/set_gps_goal");

        goal_reached_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/channel_nav/goal_reached", rclcpp::QoS(1).transient_local(),
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                goal_reached_ = msg->data;
            });
    }

    BT::NodeStatus NavigateChannel::onStart() {
        double lat, lon;
        if (!getInput("goal_lat", lat) || !getInput("goal_lon", lon)) {
            RCLCPP_ERROR(node_->get_logger(), "NavigateChannel: missing goal_lat or goal_lon port");
            return BT::NodeStatus::FAILURE;
        }

        // Wait for service (up to 5s)
        if (!set_goal_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "NavigateChannel: /channel_nav/set_gps_goal not available");
            return BT::NodeStatus::FAILURE;
        }

        goal_reached_ = false;
        goal_sent_ = false;

        auto req = std::make_shared<riteh_usv_sim::srv::SetGpsGoal::Request>();
        req->lat = lat;
        req->lon = lon;

        auto future = set_goal_client_->async_send_request(req,
                                                           [this](
                                                       rclcpp::Client<riteh_usv_sim::srv::SetGpsGoal>::SharedFuture f) {
                                                               auto resp = f.get();
                                                               if (resp->success) {
                                                                   RCLCPP_INFO(
                                                                       node_->get_logger(),
                                                                       "NavigateChannel: goal accepted — %s",
                                                                       resp->message.c_str());
                                                                   goal_sent_ = true;
                                                               } else {
                                                                   RCLCPP_WARN(
                                                                       node_->get_logger(),
                                                                       "NavigateChannel: goal rejected — %s",
                                                                       resp->message.c_str());
                                                               }
                                                           });

        (void) future;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus NavigateChannel::onRunning() {
        rclcpp::spin_some(node_);

        if (!goal_sent_) {
            // Still waiting for service response
            return BT::NodeStatus::RUNNING;
        }

        if (goal_reached_) {
            RCLCPP_INFO(node_->get_logger(), "NavigateChannel: GPS-6 reached — SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void NavigateChannel::onHalted() {
        // Stop sending new goals — channel_navigator will coast
        goal_reached_ = false;
        goal_sent_ = false;
        RCLCPP_INFO(node_->get_logger(), "NavigateChannel: halted");
    }
}
