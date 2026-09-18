#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>

namespace usv_bt {
    class NorthingAbove : public BT::ConditionNode {
    public:
        NorthingAbove(const std::string &name, const BT::NodeConfig &config);

        ~NorthingAbove();

        static BT::PortsList providedPorts() {
            return {
                BT::InputPort<double>("min_y", 16.0,
                                      "Minimum map-frame Y (northing) before COLREG checks are enabled"),
            };
        }

        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::executors::SingleThreadedExecutor executor_;
        std::thread spin_thread_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    };
}
