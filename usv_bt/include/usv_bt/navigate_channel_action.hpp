#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <riteh_usv_sim/srv/set_gps_goal.hpp>

namespace usv_bt {
    class NavigateChannel : public BT::StatefulActionNode {
    public:
        NavigateChannel(const std::string &name, const BT::NodeConfig &config);

        static BT::PortsList providedPorts() {
            return {
                BT::InputPort<double>("goal_lat", "Destination latitude in decimal degrees"),
                BT::InputPort<double>("goal_lon", "Destination longitude in decimal degrees"),
            };
        }

        BT::NodeStatus onStart() override;

        BT::NodeStatus onRunning() override;

        void onHalted() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<riteh_usv_sim::srv::SetGpsGoal>::SharedPtr set_goal_client_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_sub_;
        bool goal_reached_{false};
        bool goal_sent_{false};
    };
}
