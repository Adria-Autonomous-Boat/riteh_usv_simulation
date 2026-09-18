/**
 * Standalone BT runner for the channel-gate task.
 */
#include <filesystem>
#include <stdexcept>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include "rclcpp/rclcpp.hpp"

#include "usv_bt/gps_anchor_ready_condition.hpp"
#include "usv_bt/navigate_channel_action.hpp"
#include "usv_bt/vessel_in_range_condition.hpp"
#include "usv_bt/colreg_avoid_action.hpp"
#include "usv_bt/vessel_clear_condition.hpp"
#include "usv_bt/northing_above_condition.hpp"

// Logger that writes BT state changes to the ROS log (and thus to .log files)
class RclcppBtLogger : public BT::StatusChangeLogger {
public:
    explicit RclcppBtLogger(const BT::Tree &tree)
        : BT::StatusChangeLogger(tree.rootNode()),
          logger_(rclcpp::get_logger("bt")) {
    }

    void callback(BT::Duration /*ts*/, const BT::TreeNode &node,
                  BT::NodeStatus /*prev*/, BT::NodeStatus status) override {
        RCLCPP_INFO(logger_, "[BT] %-30s %s",
                    node.name().c_str(), BT::toStr(status, true).c_str());
    }

    void flush() override {
    }

private:
    rclcpp::Logger logger_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<usv_bt::GpsAnchorReady>("GpsAnchorReady");
    factory.registerNodeType<usv_bt::NavigateChannel>("NavigateChannel");
    factory.registerNodeType<usv_bt::VesselInRange>("VesselInRange");
    factory.registerNodeType<usv_bt::ColregAvoid>("ColregAvoid");
    factory.registerNodeType<usv_bt::VesselClear>("VesselClear");
    factory.registerNodeType<usv_bt::NorthingAbove>("NorthingAbove");

    const std::string share_dir =
            ament_index_cpp::get_package_share_directory("riteh_usv_sim");
    const std::string tree_path = share_dir + "/trees/channel_gate_task.xml";

    if (!std::filesystem::exists(tree_path)) {
        throw std::runtime_error("Tree file not found: " + tree_path);
    }

    auto tree = factory.createTreeFromFile(tree_path);
    // BT::StdCoutLogger cout_logger(tree);
    // RclcppBtLogger ros_logger(tree);

    rclcpp::Rate rate(10);
    BT::NodeStatus status = BT::NodeStatus::RUNNING;

    RCLCPP_INFO(rclcpp::get_logger("channel_gate_runner"), "BT started — ticking at 10 Hz");

    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        status = tree.tickExactlyOnce();
        rate.sleep();
    }

    if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("channel_gate_runner"), "BT finished: SUCCESS");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("channel_gate_runner"), "BT finished: FAILURE");
    }

    rclcpp::shutdown();
    return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}
