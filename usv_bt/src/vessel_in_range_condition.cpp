#include "usv_bt/vessel_in_range_condition.hpp"
#include <cmath>
#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace usv_bt {
    VesselInRange::VesselInRange(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config) {
        rclcpp::NodeOptions opts;
        opts.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
        node_ = rclcpp::Node::make_shared("vessel_in_range_bt_node", opts);
        executor_.add_node(node_);

        sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
            "/vessel_detection/closest", rclcpp::QoS(10),
            [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
                RCLCPP_INFO(node_->get_logger(),
                            "VesselInRange: got detection at (%.1f, %.1f)", msg->point.x, msg->point.y);
                std::lock_guard<std::mutex> lock(detection_mutex_);
                last_detection_ = msg;
                last_received_at_ = std::chrono::steady_clock::now();
                history_.push_back({msg->point.x, msg->point.y, last_received_at_});
                if (history_.size() > HISTORY_SIZE) {
                    history_.pop_front();
                }
            });

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_);

        spin_thread_ = std::thread([this]() {
            executor_.spin();
        });
    }

    VesselInRange::~VesselInRange() {
        executor_.cancel();
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
    }

    BT::NodeStatus VesselInRange::tick() {
        // Copy shared state under lock, then release immediately
        geometry_msgs::msg::PointStamped::SharedPtr detection;
        std::chrono::steady_clock::time_point received_at;
        {
            std::lock_guard<std::mutex> lock(detection_mutex_);
            detection = last_detection_;
            received_at = last_received_at_;
        }

        if (!detection) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), wall_clock_, 2000,
                                 "VesselInRange: no detection yet");
            return BT::NodeStatus::FAILURE;
        }

        auto age_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - received_at).count();
        if (age_s > 3.0) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), wall_clock_, 2000,
                                 "VesselInRange: detection stale (%.1fs)", age_s);
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO_THROTTLE(node_->get_logger(), wall_clock_, 2000,
                             "VesselInRange: age=%.2fs", age_s);

        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException &e) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), wall_clock_, 2000,
                                 "VesselInRange: TF lookup failed: %s", e.what());
            return BT::NodeStatus::FAILURE;
        }

        double range_threshold = 20.0;
        getInput("range_threshold", range_threshold);

        double bx = tf.transform.translation.x;
        double by = tf.transform.translation.y;
        auto &q = tf.transform.rotation;
        double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        double vx = detection->point.x;
        double vy = detection->point.y;
        double dist = std::hypot(vx - bx, vy - by);
        double fwd = std::cos(yaw) * (vx - bx) + std::sin(yaw) * (vy - by);

        RCLCPP_INFO_THROTTLE(node_->get_logger(), wall_clock_, 1000,
                             "VesselInRange: boat=(%.1f,%.1f) yaw=%.2f vessel=(%.1f,%.1f) dist=%.1fm fwd=%.1fm threshold=%.1fm → %s",
                             bx, by, yaw, vx, vy, dist, fwd, range_threshold,
                             (dist <= range_threshold && fwd >= 5.0) ? "TRIGGER" : "clear");

        if (dist > range_threshold || fwd < 5.0) {
            return BT::NodeStatus::FAILURE;
        }

        // Check detection is actually moving — static objects (walls, docks) have near-zero speed
        double min_speed = 0.3;
        double max_speed = 8.0;
        getInput("min_speed", min_speed);
        getInput("max_speed", max_speed);

        DetectionSample oldest, newest;
        bool have_history = false;
        {
            std::lock_guard<std::mutex> lock(detection_mutex_);
            if (history_.size() >= 2) {
                oldest = history_.front();
                newest = history_.back();
                have_history = true;
            }
        }

        if (have_history) {
            double dt = std::chrono::duration<double>(newest.t - oldest.t).count();
            if (dt > 0.5) {
                double dx = newest.x - oldest.x;
                double dy = newest.y - oldest.y;
                double speed = std::hypot(dx, dy) / dt;
                if (speed < min_speed) {
                    RCLCPP_INFO_THROTTLE(node_->get_logger(), wall_clock_, 2000,
                                         "VesselInRange: detection speed=%.2fm/s < %.2fm/s — likely static, ignoring",
                                         speed, min_speed);
                    return BT::NodeStatus::FAILURE;
                }
                if (speed > max_speed) {
                    RCLCPP_INFO_THROTTLE(node_->get_logger(), wall_clock_, 2000,
                                         "VesselInRange: detection speed=%.2fm/s > %.2fm/s — likely lidar noise, ignoring",
                                         speed, max_speed);
                    return BT::NodeStatus::FAILURE;
                }
                RCLCPP_INFO_THROTTLE(node_->get_logger(), wall_clock_, 1000,
                                     "VesselInRange: detection speed=%.2fm/s — confirmed moving vessel",
                                     speed);
            }
        }

        return BT::NodeStatus::SUCCESS;
    }
}
