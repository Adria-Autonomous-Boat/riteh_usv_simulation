#include "usv_bt/colreg_avoid_action.hpp"
#include <cmath>
#include <limits>
#include <tf2/time.h>

namespace usv_bt {
    ColregAvoid::ColregAvoid(const std::string &name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config) {
        rclcpp::NodeOptions opts;
        opts.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
        node_ = rclcpp::Node::make_shared("colreg_avoid_bt_node", opts);
        executor_.add_node(node_);

        nav_client_ = rclcpp_action::create_client<NavToPose>(node_, "navigate_to_pose");

        rclcpp::QoS pause_qos(1);
        pause_qos.transient_local().reliable();
        pause_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/channel_nav/paused", pause_qos);

        vessel_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
            "/vessel_detection/closest", rclcpp::QoS(1),
            [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(vessel_mutex_);
                vessel_pos_ = msg;
                vessel_history_.push_back({
                    msg->point.x, msg->point.y,
                    std::chrono::steady_clock::now()
                });
                if (vessel_history_.size() > HISTORY_SIZE) {
                    vessel_history_.pop_front();
                }
            });

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_);

        spin_thread_ = std::thread([this]() { executor_.spin(); });
    }

    ColregAvoid::~ColregAvoid() {
        executor_.cancel();
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
    }

    // ---------------------------------------------------------------------------
    // CPA-based COLREGS rule classifier
    // ---------------------------------------------------------------------------
    ColregRule ColregAvoid::classify_situation(
        double bx, double by, double byaw,
        double vx, double vy, double otter_yaw) const {
        // Bearing of vessel relative to our heading (body frame)
        double dx = vx - bx;
        double dy = vy - by;
        double bearing = std::atan2(dy, dx) - byaw;
        // Normalise to [-π, π]
        bearing = std::remainder(bearing, 2.0 * M_PI);

        // Wide head-on zone (±22.5°) for initial bearing check
        static constexpr double HEAD_ON_WIDE = 0.393;

        if (std::abs(bearing) <= HEAD_ON_WIDE) {
            // Only treat as HEAD-ON if Otter heading is also roughly opposite ours (±22.5°)
            if (!std::isnan(otter_yaw)) {
                double opposite = std::remainder(byaw + M_PI, 2.0 * M_PI);
                double hdg_diff = std::abs(std::remainder(otter_yaw - opposite, 2.0 * M_PI));
                if (hdg_diff > HEAD_ON_WIDE) {
                    // Otter is in front but crossing — treat as port crossing (stand-on)
                    return ColregRule::CROSSING_FROM_LEFT;
                }
            }
            return ColregRule::HEAD_ON;
        }
        // Starboard side: bearing negative in ENU body frame — give-way
        if (bearing < -HEAD_ON_WIDE && bearing >= -CROSSING_HALF_ANGLE) {
            return ColregRule::CROSSING_FROM_RIGHT;
        }
        // Port side: bearing positive in ENU body frame — stand-on
        if (bearing > HEAD_ON_WIDE && bearing <= CROSSING_HALF_ANGLE) {
            return ColregRule::CROSSING_FROM_LEFT;
        }
        return ColregRule::NONE;
    }

    // ---------------------------------------------------------------------------
    // Send a Nav2 goal — Nav2 handles preemption of the previous goal natively
    // ---------------------------------------------------------------------------
    bool ColregAvoid::send_nav_goal(double sx, double sy, double yaw) {
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = node_->get_clock()->now();
        goal_pose.pose.position.x = sx;
        goal_pose.pose.position.y = sy;
        goal_pose.pose.position.z = 0.0;

        tf2::Quaternion quat;
        quat.setRPY(0, 0, yaw);
        goal_pose.pose.orientation = tf2::toMsg(quat);

        auto goal_msg = NavToPose::Goal();
        goal_msg.pose = goal_pose;

        auto send_opts = rclcpp_action::Client<NavToPose>::SendGoalOptions();
        send_opts.goal_response_callback =
                [this](const GoalHandle::SharedPtr &handle) {
                    std::lock_guard<std::mutex> lock(goal_mutex_);
                    if (!handle) {
                        RCLCPP_WARN(node_->get_logger(), "ColregAvoid: goal rejected");
                        goal_done_ = true;
                        goal_succeeded_ = false;
                    } else {
                        goal_handle_ = handle;
                        goal_sent_ = true;
                    }
                };
        send_opts.result_callback =
                [this](const GoalHandle::WrappedResult &result) {
                    std::lock_guard<std::mutex> lock(goal_mutex_);
                    if (result.code == rclcpp_action::ResultCode::CANCELED) {
                        return; // preempted by a newer goal — not a real failure
                    }
                    goal_done_ = true;
                    goal_succeeded_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
                };

        nav_client_->async_send_goal(goal_msg, send_opts);
        return true;
    }

    // ---------------------------------------------------------------------------
    // Estimate Otter heading from rolling position history (oldest → newest).
    // Must be called with vessel_mutex_ held.
    // ---------------------------------------------------------------------------
    bool ColregAvoid::estimate_otter_heading(double &heading_out) const {
        if (vessel_history_.size() < 2) return false;

        const auto &oldest = vessel_history_.front();
        const auto &newest = vessel_history_.back();

        double dt = std::chrono::duration<double>(newest.t - oldest.t).count();
        if (dt < 0.1) return false;

        double dx = newest.x - oldest.x;
        double dy = newest.y - oldest.y;

        if (std::hypot(dx, dy) < MIN_MOVEMENT) return false;

        heading_out = std::atan2(dy, dx);
        return true;
    }

    // ---------------------------------------------------------------------------
    // Returns true if vessel is closing (range decreasing), false if departing.
    // Must be called with vessel_mutex_ held.
    // ---------------------------------------------------------------------------
    bool ColregAvoid::is_vessel_closing(double bx, double by) const {
        if (vessel_history_.size() < 2) return true;
        const auto &oldest = vessel_history_.front();
        const auto &newest = vessel_history_.back();
        double dt = std::chrono::duration<double>(newest.t - oldest.t).count();
        if (dt < 0.3) return true;
        double old_dist = std::hypot(oldest.x - bx, oldest.y - by);
        double new_dist = std::hypot(newest.x - bx, newest.y - by);
        double range_rate = (new_dist - old_dist) / dt;
        return range_rate < DEPARTING_RANGE_RATE;
    }

    // ---------------------------------------------------------------------------
    // Compute the fixed avoidance waypoint once from the initial encounter geometry
    // ---------------------------------------------------------------------------
    BT::NodeStatus ColregAvoid::onStart() {
        goal_sent_ = false;
        goal_done_ = false;
        goal_succeeded_ = false;
        last_replan_time_ = std::chrono::steady_clock::time_point{}; // force immediate first send
        arc_waypoints_.clear();
        arc_wp_index_ = 0;

        geometry_msgs::msg::PointStamped::SharedPtr vessel;
        {
            std::lock_guard<std::mutex> lock(vessel_mutex_);
            vessel = vessel_pos_;
        }

        if (!vessel) {
            RCLCPP_WARN(node_->get_logger(), "ColregAvoid: no vessel detection data");
            return BT::NodeStatus::FAILURE;
        }

        if (!nav_client_->wait_for_action_server(std::chrono::seconds(3))) {
            RCLCPP_ERROR(node_->get_logger(), "ColregAvoid: navigate_to_pose not available");
            return BT::NodeStatus::FAILURE;
        }

        // Capture initial boat pose
        double bx = 0.0, by = 0.0;
        try {
            auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            bx = tf.transform.translation.x;
            by = tf.transform.translation.y;
            auto &q = tf.transform.rotation;
            initial_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        } catch (const tf2::TransformException &) {
            initial_yaw_ = 0.0;
        }

        double offset = 5.0;
        getInput("starboard_offset", offset);

        double vx = vessel->point.x;
        double vy = vessel->point.y;

        // Check if vessel is departing — skip all manoeuvres if so
        bool closing;
        {
            std::lock_guard<std::mutex> lock(vessel_mutex_);
            closing = is_vessel_closing(bx, by);
        }
        if (!closing) {
            RCLCPP_INFO(node_->get_logger(),
                        "ColregAvoid: vessel at (%.1f,%.1f) is departing — no action", vx, vy);
            return BT::NodeStatus::FAILURE;
        }

        double otter_yaw = std::numeric_limits<double>::quiet_NaN();
        {
            std::lock_guard<std::mutex> lock(vessel_mutex_);
            if (estimate_otter_heading(otter_yaw)) {
                RCLCPP_INFO(node_->get_logger(),
                            "ColregAvoid: otter_heading=%.1f° (%.2frad)",
                            otter_yaw * 180.0 / M_PI, otter_yaw);
            } else {
                RCLCPP_INFO(node_->get_logger(), "ColregAvoid: otter heading unknown (insufficient history)");
            }
        }

        ColregRule rule = classify_situation(bx, by, initial_yaw_, vx, vy, otter_yaw);

        if (rule == ColregRule::CROSSING_FROM_LEFT) {
            double emergency_range = 10.0;
            getInput("emergency_range", emergency_range);
            double dist = std::hypot(vx - bx, vy - by);

            if (dist > emergency_range) {
                RCLCPP_INFO(node_->get_logger(),
                            "ColregAvoid [STAND-ON]: vessel on port at (%.1f,%.1f) dist=%.1fm > %.1fm, maintaining course",
                            vx, vy, dist, emergency_range);
                return BT::NodeStatus::FAILURE;
            }

            // Emergency: Otter too close and not giving way — clockwise arc to starboard
            double arc_radius = 3.5;
            getInput("arc_radius", arc_radius);

            // 2 waypoints: -90° (starboard) then -180° (behind) relative to current heading
            arc_waypoints_.clear();
            for (int i = 1; i <= 2; ++i) {
                double angle = initial_yaw_ - i * (M_PI / 2.0);
                arc_waypoints_.push_back({
                    bx + arc_radius * std::cos(angle),
                    by + arc_radius * std::sin(angle)
                });
            }
            arc_wp_index_ = 0;

            RCLCPP_INFO(node_->get_logger(),
                        "ColregAvoid [EMERGENCY]: vessel at (%.1f,%.1f) dist=%.1fm — clockwise arc, r=%.1fm",
                        vx, vy, dist, arc_radius);
            publish_pause(true);
            send_nav_goal(arc_waypoints_[0].x, arc_waypoints_[0].y, initial_yaw_ - M_PI_2);
            return BT::NodeStatus::RUNNING;
        }

        if (rule == ColregRule::CROSSING_FROM_RIGHT) {
            double yield_offset = 7.0;
            getInput("yield_offset", yield_offset);
            double yield_clearance = 3.0;
            getInput("yield_clearance", yield_clearance);

            double wx, wy;
            if (!std::isnan(otter_yaw)) {
                // Otter track frame: ox/oy = forward along Otter's heading
                double ox = std::cos(otter_yaw);
                double oy = std::sin(otter_yaw);

                // Vector from Otter to us
                double rx = bx - vx;
                double ry = by - vy;

                // Decompose into along-track and across-track components
                // along > 0 means we are ahead of Otter, across > 0 means we are to Otter's right
                double along = rx * ox + ry * oy;
                double across = rx * (-oy) + ry * ox;

                // Fall back behind Otter's stern by yield_clearance.
                // If we are already behind (along < 0), push back a bit further still.
                double target_along = std::min(along - yield_clearance, -yield_clearance);

                // Keep the same across-track side we are on — we don't cross Otter's path
                double target_across = across;

                // Convert back to map frame
                wx = vx + target_along * ox - target_across * oy;
                wy = vy + target_along * oy + target_across * ox;

                RCLCPP_INFO(node_->get_logger(),
                            "ColregAvoid [GIVE-WAY]: boat=(%.1f,%.1f) yaw=%.2frad otter=(%.1f,%.1f) hdg=%.1f° "
                            "along=%.1fm across=%.1fm → yield waypoint=(%.1f,%.1f)",
                            bx, by, initial_yaw_, vx, vy,
                            otter_yaw * 180.0 / M_PI,
                            along, across, wx, wy);
            } else {
                // Otter heading unknown — fall back to pure starboard sidestep
                double sb_x = std::cos(initial_yaw_ - M_PI_2);
                double sb_y = std::sin(initial_yaw_ - M_PI_2);
                wx = bx + yield_offset * sb_x;
                wy = by + yield_offset * sb_y;

                RCLCPP_INFO(node_->get_logger(),
                            "ColregAvoid [GIVE-WAY]: boat=(%.1f,%.1f) yaw=%.2frad otter=(%.1f,%.1f) hdg=unknown "
                            "→ starboard fallback waypoint=(%.1f,%.1f)",
                            bx, by, initial_yaw_, vx, vy, wx, wy);
            }

            avoidance_x_ = wx;
            avoidance_y_ = wy;
            publish_pause(true);
            return BT::NodeStatus::RUNNING;
        }

        const char *rule_str = (rule == ColregRule::HEAD_ON) ? "HEAD-ON" : "NONE";

        double mx = (bx + vx) / 2.0;
        double my = (by + vy) / 2.0;
        avoidance_x_ = mx + offset * std::cos(initial_yaw_ - M_PI_2);
        avoidance_y_ = my + offset * std::sin(initial_yaw_ - M_PI_2);

        RCLCPP_INFO(node_->get_logger(),
                    "ColregAvoid [%s]: boat=(%.1f,%.1f) yaw=%.2frad vessel=(%.1f,%.1f) → fixed waypoint=(%.1f,%.1f)",
                    rule_str, bx, by, initial_yaw_, vx, vy, avoidance_x_, avoidance_y_);

        publish_pause(true);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus ColregAvoid::onRunning() {
        // Clockwise emergency arc progression
        if (!arc_waypoints_.empty()) {
            if (goal_done_) {
                if (!goal_succeeded_) {
                    RCLCPP_WARN(node_->get_logger(), "ColregAvoid [EMERGENCY]: wp %zu aborted, retrying",
                                arc_wp_index_);
                    goal_done_ = false;
                    goal_succeeded_ = false;
                    send_nav_goal(arc_waypoints_[arc_wp_index_].x, arc_waypoints_[arc_wp_index_].y,
                                  initial_yaw_ - M_PI_2);
                    return BT::NodeStatus::RUNNING;
                }
                arc_wp_index_++;
                goal_done_ = false;
                goal_succeeded_ = false;
                if (arc_wp_index_ >= arc_waypoints_.size()) {
                    arc_waypoints_.clear();
                    arc_wp_index_ = 0;
                    publish_pause(false);
                    RCLCPP_INFO(node_->get_logger(), "ColregAvoid [EMERGENCY]: arc complete, resuming");
                    return BT::NodeStatus::SUCCESS;
                }
                RCLCPP_INFO(node_->get_logger(), "ColregAvoid [EMERGENCY]: arc wp %zu → (%.1f, %.1f)",
                            arc_wp_index_,
                            arc_waypoints_[arc_wp_index_].x, arc_waypoints_[arc_wp_index_].y);
                send_nav_goal(arc_waypoints_[arc_wp_index_].x, arc_waypoints_[arc_wp_index_].y,
                              initial_yaw_ - M_PI_2);
            }
            return BT::NodeStatus::RUNNING;
        }

        if (goal_done_) {
            if (goal_succeeded_) {
                publish_pause(false);
                RCLCPP_INFO(node_->get_logger(), "ColregAvoid: starboard waypoint reached");
                return BT::NodeStatus::SUCCESS;
            }
            // Goal was ABORTED by Nav2 (planner failed) — wait replan_interval_s before retrying
            RCLCPP_WARN(node_->get_logger(), "ColregAvoid: Nav2 goal aborted, waiting before retry");
            last_replan_time_ = std::chrono::steady_clock::now();
            goal_done_ = false;
            goal_succeeded_ = false;
        }

        // Time-based replan — resend the same fixed waypoint until Nav2 accepts and reaches it
        double replan_interval_s = 1.0;
        getInput("replan_interval_s", replan_interval_s);

        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_replan_time_).count();
        if (elapsed < replan_interval_s) {
            return BT::NodeStatus::RUNNING;
        }
        last_replan_time_ = now;

        goal_done_ = false;
        goal_succeeded_ = false;
        send_nav_goal(avoidance_x_, avoidance_y_, initial_yaw_);
        return BT::NodeStatus::RUNNING;
    }

    void ColregAvoid::onHalted() {
        publish_pause(false);
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            if (goal_handle_) {
                nav_client_->async_cancel_goal(goal_handle_);
                goal_handle_.reset();
            }
        }
        goal_sent_ = false;
        goal_done_ = false;
        arc_waypoints_.clear();
        arc_wp_index_ = 0;
        RCLCPP_INFO(node_->get_logger(), "ColregAvoid: halted");
    }

    void ColregAvoid::publish_pause(bool paused) {
        std_msgs::msg::Bool msg;
        msg.data = paused;
        pause_pub_->publish(msg);
        RCLCPP_INFO(node_->get_logger(),
                    "ColregAvoid: channel_navigator %s", paused ? "PAUSED" : "RESUMED");
    }
}
