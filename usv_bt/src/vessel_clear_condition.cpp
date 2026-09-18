#include "usv_bt/vessel_clear_condition.hpp"
#include <cmath>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <std_msgs/msg/bool.hpp>

namespace usv_bt {
    VesselClear::VesselClear(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config) {
        rclcpp::NodeOptions opts;
        opts.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
        node_ = rclcpp::Node::make_shared("vessel_clear_bt_node", opts);
        executor_.add_node(node_);

        rclcpp::QoS pause_qos(1);
        pause_qos.transient_local().reliable();
        pause_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/channel_nav/paused", pause_qos);

        vessel_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
            "/vessel_detection/closest", rclcpp::QoS(10),
            [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(detection_mutex_);
                last_detection_ = msg;
                last_received_at_ = std::chrono::steady_clock::now();
                vessel_history_.push_back({
                    msg->point.x, msg->point.y,
                    std::chrono::steady_clock::now()
                });
                if (vessel_history_.size() > HISTORY_SIZE)
                    vessel_history_.pop_front();
            });

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_);

        spin_thread_ = std::thread([this]() {
            executor_.spin();
        });
    }

    VesselClear::~VesselClear() {
        executor_.cancel();
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
    }

    BT::NodeStatus VesselClear::tick() {
        geometry_msgs::msg::PointStamped::SharedPtr detection;
        std::chrono::steady_clock::time_point received_at;
        VesselSample history_front{}, history_back{};
        bool have_history = false;
        {
            std::lock_guard<std::mutex> lock(detection_mutex_);
            detection = last_detection_;
            received_at = last_received_at_;
            if (vessel_history_.size() >= 2) {
                history_front = vessel_history_.front();
                history_back = vessel_history_.back();
                have_history = true;
            }
        }

        auto publish_resume = [this]() {
            std_msgs::msg::Bool msg;
            msg.data = false;
            pause_pub_->publish(msg);
        };

        if (!detection) {
            publish_resume();
            return BT::NodeStatus::SUCCESS;
        }

        auto age_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - received_at).count();
        if (age_s > 2.0) {
            publish_resume();
            return BT::NodeStatus::SUCCESS;
        }

        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException &) {
            return BT::NodeStatus::FAILURE;
        }

        double clear_range = 25.0;
        getInput("clear_range", clear_range);

        double bx = tf.transform.translation.x;
        double by = tf.transform.translation.y;
        auto &q = tf.transform.rotation;
        double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        double vx = detection->point.x;
        double vy = detection->point.y;
        double dist = std::hypot(vx - bx, vy - by);
        double fwd = std::cos(yaw) * (vx - bx) + std::sin(yaw) * (vy - by);

        // Observational only — log range_rate for tuning, not yet used in decision
        if (have_history) {
            double dt = std::chrono::duration<double>(history_back.t - history_front.t).count();
            if (dt > 0.5) {
                double old_dist = std::hypot(history_front.x - bx, history_front.y - by);
                double new_dist = std::hypot(history_back.x - bx, history_back.y - by);
                double range_rate = (new_dist - old_dist) / dt;
                RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                     "VesselClear: dist=%.1fm  fwd=%.1fm  range_rate=%.2fm/s",
                                     dist, fwd, range_rate);
            }
        }

        if (dist > clear_range || fwd < -3.0) {
            RCLCPP_INFO(node_->get_logger(),
                        "VesselClear: vessel %.1fm away fwd=%.1fm → CLEAR", dist, fwd);
            publish_resume();
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }
}
