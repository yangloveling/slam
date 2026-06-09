#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <lite_task_controller/srv/set_relative_goal.hpp>

#include <unordered_map>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>
#include <sstream>
#include <set>
#include <tuple>
#include <string>
#include <regex>
#include <chrono>

#include "robot_common/r2_field_map.hpp"

class R2AStarPlannerNode : public rclcpp::Node {
public:
    R2AStarPlannerNode() : Node("r2_astar_planner_node") {
        declare_parameter<int>("start_block", 0);
        declare_parameter<int>("start_yaw", 0);

        declare_parameter<int>("type_empty", 0);
        declare_parameter<int>("type_r1", 1);
        declare_parameter<int>("type_r2", 2);
        declare_parameter<int>("type_fake", 3);

        declare_parameter<bool>("allow_pass_r1", true);
        declare_parameter<bool>("allow_pass_fake", false);
        declare_parameter<std::string>("id_type_topic", "/id_type");

        declare_parameter<double>("neighbor_distance", 1.20);
        declare_parameter<double>("neighbor_tolerance", 0.15);

        declare_parameter<double>("cost_forward_flat", 1.0);
        // Kept for launch/config compatibility. Flat reverse moves are not
        // generated because lite executes block paths by turning first, then moving.
        declare_parameter<double>("cost_reverse_flat", 1.5);
        declare_parameter<double>("cost_turn_90", 2.0);
        declare_parameter<double>("cost_climb_up", 6.0);
        declare_parameter<double>("cost_reverse_down", 10.0);

        declare_parameter<double>("cost_height_20", 0.0);
        declare_parameter<double>("cost_height_40", 2.0);
        declare_parameter<double>("cost_height_60", 8.0);

        declare_parameter<double>("cost_start_to_block", 1.0);
        declare_parameter<double>("cost_r1_wait", 0.0);
        declare_parameter<double>("cost_fake_penalty", 100000.0);

        declare_parameter<int>("target_count", 2);

        declare_parameter<bool>("auto_plan_when_update", true);
        declare_parameter<bool>("wait_all_blocks_before_plan", false);

        declare_parameter<bool>("enable_initial_observation_turn", true);
        declare_parameter<bool>("plan_requires_initial_observation", true);
        declare_parameter<double>("initial_observation_dyaw_deg", 45.0);
        declare_parameter<double>("initial_observation_wait_after_accept_s", 1.5);
        declare_parameter<int>("initial_observation_retry_period_ms", 500);
        declare_parameter<std::string>("relative_goal_service", "/set_relative_goal");

        declare_parameter<double>("pickup_prefer_left_cost", -20.0);
        declare_parameter<double>("pickup_prefer_bottom_cost", -18.0);
        declare_parameter<double>("pickup_penalty_right_cost", 20.0);
        declare_parameter<double>("pickup_penalty_top_cost", 18.0);

        declare_parameter<std::string>("odom_topic", "/odometry/filtered");
        declare_parameter<double>("block_half_size_m", 0.60);

        declare_parameter<std::string>("vision_kfs_topic", "/vision/kfs_camera_xyz");
        declare_parameter<std::string>("r1_type_prefix", "R1_KFS");

        declare_parameter<double>("lost_confirm_time", 0.5);
        declare_parameter<int>("r1_monitor_period_ms", 50);

        declare_parameter<double>("camera_mount_offset_x_m", 0.32);
        declare_parameter<double>("camera_mount_offset_z_m", 0.125);

        declare_parameter<std::string>(
            "camera_gimbal_state_topic",
            "/camera_gimbal_look_backward"
        );

        // Camera coordinate: z+ = forward, x+ = right, y+ = up
        // Base coordinate: x+ = robot front, y+ = robot left
        // Backward gimbal position: rear 6 cm => x=-0.06, left 32 cm => y=+0.32
        declare_parameter<double>("camera_forward_offset_x_m", 0.06);
        declare_parameter<double>("camera_forward_offset_y_m", -0.32);
        declare_parameter<double>("camera_backward_offset_x_m", -0.06);
        declare_parameter<double>("camera_backward_offset_y_m", 0.32);

        declare_parameter<bool>("use_odom_yaw_for_camera_transform", true);

        declare_parameter<bool>("backward_camera_x_positive_to_robot_left", false);
        declare_parameter<double>("locked_r1_refresh_radius_m", 0.55);


        /*
          重要：
          默认 false。
          因为 R1_KFS 被移开后，block_types_ 可能被改成 empty。
          后面你再把 R1_KFS 放回去，如果还要求 /id_type 是 R1，就会被忽略。
        */
        declare_parameter<bool>("only_track_id_type_r1_block", false);

        /*
          true 表示仍然做路径相关判断。
          但现在路径相关不是整条路径，而是只判断 current_next_block_。
        */
        declare_parameter<bool>("only_track_path_relevant_r1", true);

        declare_parameter<bool>("replan_after_r1_cleared", false);
        declare_parameter<bool>("use_current_block_as_replan_start", false);

        /*
          0 表示不限制 R1_KFS 等待/释放次数。
          后面同一个 block 再次出现 R1_KFS，也可以再次等待。
        */
        declare_parameter<int>("r1_release_limit", 0);

        start_block_ = get_parameter("start_block").as_int();
        start_yaw_ = normalize_yaw(get_parameter("start_yaw").as_int());

        type_empty_ = get_parameter("type_empty").as_int();
        type_r1_ = get_parameter("type_r1").as_int();
        type_r2_ = get_parameter("type_r2").as_int();
        type_fake_ = get_parameter("type_fake").as_int();

        allow_pass_r1_ = get_parameter("allow_pass_r1").as_bool();
        allow_pass_fake_ = get_parameter("allow_pass_fake").as_bool();
        id_type_topic_ = get_parameter("id_type_topic").as_string();

        target_count_ = get_parameter("target_count").as_int();
        auto_plan_when_update_ = get_parameter("auto_plan_when_update").as_bool();
        wait_all_blocks_before_plan_ = get_parameter("wait_all_blocks_before_plan").as_bool();

        enable_initial_observation_turn_ =
            get_parameter("enable_initial_observation_turn").as_bool();
        plan_requires_initial_observation_ =
            get_parameter("plan_requires_initial_observation").as_bool();
        initial_observation_dyaw_rad_ =
            get_parameter("initial_observation_dyaw_deg").as_double() * M_PI / 180.0;
        initial_observation_wait_after_accept_s_ =
            get_parameter("initial_observation_wait_after_accept_s").as_double();
        initial_observation_retry_period_ms_ =
            get_parameter("initial_observation_retry_period_ms").as_int();
        relative_goal_service_ =
            get_parameter("relative_goal_service").as_string();

        pickup_prefer_left_cost_ = get_parameter("pickup_prefer_left_cost").as_double();
        pickup_prefer_bottom_cost_ = get_parameter("pickup_prefer_bottom_cost").as_double();
        pickup_penalty_right_cost_ = get_parameter("pickup_penalty_right_cost").as_double();
        pickup_penalty_top_cost_ = get_parameter("pickup_penalty_top_cost").as_double();

        odom_topic_ = get_parameter("odom_topic").as_string();
        block_half_size_m_ = get_parameter("block_half_size_m").as_double();

        vision_kfs_topic_ = get_parameter("vision_kfs_topic").as_string();
        r1_type_prefix_ = get_parameter("r1_type_prefix").as_string();

        lost_confirm_time_ = get_parameter("lost_confirm_time").as_double();
        r1_monitor_period_ms_ = get_parameter("r1_monitor_period_ms").as_int();

        camera_mount_offset_x_m_ = get_parameter("camera_mount_offset_x_m").as_double();
        camera_mount_offset_z_m_ = get_parameter("camera_mount_offset_z_m").as_double();

        camera_gimbal_state_topic_ =
            get_parameter("camera_gimbal_state_topic").as_string();
        camera_forward_offset_x_m_ =
            get_parameter("camera_forward_offset_x_m").as_double();
        camera_forward_offset_y_m_ =
            get_parameter("camera_forward_offset_y_m").as_double();
        camera_backward_offset_x_m_ =
            get_parameter("camera_backward_offset_x_m").as_double();
        camera_backward_offset_y_m_ =
            get_parameter("camera_backward_offset_y_m").as_double();

        use_odom_yaw_for_camera_transform_ =
    get_parameter("use_odom_yaw_for_camera_transform").as_bool();

backward_camera_x_positive_to_robot_left_ =
    get_parameter("backward_camera_x_positive_to_robot_left").as_bool();

locked_r1_refresh_radius_m_ =
    get_parameter("locked_r1_refresh_radius_m").as_double();


        only_track_id_type_r1_block_ =
            get_parameter("only_track_id_type_r1_block").as_bool();

        only_track_path_relevant_r1_ =
            get_parameter("only_track_path_relevant_r1").as_bool();

        replan_after_r1_cleared_ =
            get_parameter("replan_after_r1_cleared").as_bool();

        use_current_block_as_replan_start_ =
            get_parameter("use_current_block_as_replan_start").as_bool();

        r1_release_limit_ = get_parameter("r1_release_limit").as_int();

        init_blocks();

        if (blocks_.find(start_block_) == blocks_.end()) {
            RCLCPP_WARN(get_logger(), "Invalid start_block=%d, fallback to 0", start_block_);
            start_block_ = 0;
        }

        init_edges_by_position();

        for (int i = 1; i <= 12; ++i) {
            block_types_[i] = type_empty_;
        }
        block_types_[0] = type_empty_;

        exit_blocks_ = robot_common::r2::exit_blocks();

        id_type_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
            id_type_topic_,
            10,
            std::bind(&R2AStarPlannerNode::id_type_callback, this, std::placeholders::_1)
        );

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_,
            10,
            std::bind(&R2AStarPlannerNode::odom_callback, this, std::placeholders::_1)
        );

        kfs_sub_ = create_subscription<std_msgs::msg::String>(
            vision_kfs_topic_,
            10,
            std::bind(&R2AStarPlannerNode::r1_kfs_callback, this, std::placeholders::_1)
        );

        camera_gimbal_state_sub_ = create_subscription<std_msgs::msg::Bool>(
            camera_gimbal_state_topic_,
            10,
            std::bind(
                &R2AStarPlannerNode::camera_gimbal_state_callback,
                this,
                std::placeholders::_1)
        );

        r1_kfs_monitor_timer_ = create_wall_timer(
            std::chrono::milliseconds(r1_monitor_period_ms_),
            std::bind(&R2AStarPlannerNode::r1_kfs_monitor_timer_callback, this)
        );

        path_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("/r2_planned_path", 10);
        target_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("/r2_target_kfs", 10);
        action_pub_ = create_publisher<std_msgs::msg::String>("/r2_action_sequence", 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/r2_path_marker", 10);

        r1_wait_pub_ = create_publisher<std_msgs::msg::Int32>("/r2_wait_for_r1_kfs", 10);
        r1_cleared_pub_ = create_publisher<std_msgs::msg::Int32>("/r2_r1_kfs_cleared", 10);

        relative_goal_client_ =
            create_client<lite_task_controller::srv::SetRelativeGoal>(
                relative_goal_service_);

        initial_observation_timer_ = create_wall_timer(
            std::chrono::milliseconds(initial_observation_retry_period_ms_),
            std::bind(&R2AStarPlannerNode::initial_observation_timer_callback, this)
        );

        RCLCPP_INFO(get_logger(), "R2 A* planner started.");
        RCLCPP_INFO(get_logger(), "Start block: %d, start yaw: %d", start_block_, start_yaw_);
        RCLCPP_INFO(get_logger(), "Planner id_type input topic: %s", id_type_topic_.c_str());
        RCLCPP_INFO(get_logger(), "Zero block 0: x=2.160, y=-1.530, h=0.00");
        {
            std::ostringstream oss;
            oss << "Exit blocks: ";
            for (const auto b : exit_blocks_) {
                oss << b << " ";
            }
            RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
        }

        RCLCPP_WARN(
    get_logger(),
    "R1 wait monitor enabled. vision_topic=%s, lost_confirm_time=%.2f, "
    "gimbal_topic=%s, look_backward=%d, "
    "forward_offset=(%.3f, %.3f), backward_offset=(%.3f, %.3f), "
    "camera_axis: z=forward, x=right, y=up, "
    "only_track_id_type_r1=%d, only_track_next_block_only=%d, "
    "r1_release_limit=%d, replan_after_r1_cleared=%d, "
    "backward_x_positive_to_robot_left=%d, locked_r1_refresh_radius=%.3f",
    vision_kfs_topic_.c_str(),
    lost_confirm_time_,
    camera_gimbal_state_topic_.c_str(),
    camera_look_backward_ ? 1 : 0,
    camera_forward_offset_x_m_,
    camera_forward_offset_y_m_,
    camera_backward_offset_x_m_,
    camera_backward_offset_y_m_,
    only_track_id_type_r1_block_,
    only_track_path_relevant_r1_,
    r1_release_limit_,
    replan_after_r1_cleared_,
    backward_camera_x_positive_to_robot_left_ ? 1 : 0,
    locked_r1_refresh_radius_m_
);


        RCLCPP_WARN(
            get_logger(),
            "Pickup support preference enabled: left=%.2f, bottom=%.2f, right_penalty=%.2f, top_penalty=%.2f",
            pickup_prefer_left_cost_,
            pickup_prefer_bottom_cost_,
            pickup_penalty_right_cost_,
            pickup_penalty_top_cost_
        );

        RCLCPP_WARN(
            get_logger(),
            "Initial observation turn: enabled=%d, require_before_plan=%d, "
            "service=%s, dyaw=%.3f rad, wait_after_accept=%.2f s",
            enable_initial_observation_turn_ ? 1 : 0,
            plan_requires_initial_observation_ ? 1 : 0,
            relative_goal_service_.c_str(),
            initial_observation_dyaw_rad_,
            initial_observation_wait_after_accept_s_
        );
    }

private:
    struct Block {
        int id;
        double x;
        double y;
        double h;
    };

    struct State {
        int block;
        int yaw;

        bool operator==(const State& other) const {
            return block == other.block && yaw == other.yaw;
        }
    };

    struct StateKeyHash {
        std::size_t operator()(const State& s) const {
            return std::hash<int>()(s.block * 1000 + s.yaw);
        }
    };

    struct PrevInfo {
        State prev;
        std::string action;
    };

    struct QueueItem {
        double cost;
        State state;

        bool operator>(const QueueItem& other) const {
            return cost > other.cost;
        }
    };

    struct SegmentResult {
        double cost = 0.0;
        std::vector<State> states;
        std::vector<std::string> actions;
    };

    struct PickupTask {
        int target = -1;
        int support = -1;
        int yaw = 0;
    };

    struct PlanResult {
        double cost = 0.0;
        std::vector<int> targets;
        std::vector<State> states;
        std::vector<std::string> actions;
        std::vector<PickupTask> pickup_tasks;
        int exit_block = -1;
    };

    enum class R1WaitState {
        IDLE,
        TRACKING_AND_WAITING
    };

    struct R1TrackState {
        bool seen_once = false;
        rclcpp::Time last_seen_time;
    };

private:
    void initial_observation_timer_callback() {
        if (!enable_initial_observation_turn_) {
            initial_observation_ready_ = true;
            initial_observation_timer_->cancel();
            return;
        }

        if (initial_observation_ready_) {
            initial_observation_timer_->cancel();
            return;
        }

        if (!initial_observation_request_sent_) {
            if (!relative_goal_client_->service_is_ready()) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Waiting for lite relative goal service: %s",
                    relative_goal_service_.c_str()
                );
                return;
            }

            send_initial_observation_turn_request();
            return;
        }

        if (!initial_observation_accepted_) {
            return;
        }

        const double elapsed = (now() - initial_observation_accept_time_).seconds();
        if (elapsed < initial_observation_wait_after_accept_s_) {
            return;
        }

        initial_observation_ready_ = true;
        initial_observation_timer_->cancel();

        RCLCPP_WARN(
            get_logger(),
            "Initial observation turn wait finished. received_blocks=%ld/12. "
            "A* planning is now allowed.",
            received_blocks_.size()
        );

        if (auto_plan_when_update_ && !received_blocks_.empty()) {
            try_plan_after_observation("initial observation finished");
        }
    }

    void send_initial_observation_turn_request() {
        auto request =
            std::make_shared<lite_task_controller::srv::SetRelativeGoal::Request>();

        request->dx = 0.0;
        request->dy = 0.0;
        request->dyaw = initial_observation_dyaw_rad_;

        initial_observation_request_sent_ = true;

        RCLCPP_WARN(
            get_logger(),
            "Send initial observation turn request to lite: dx=0 dy=0 dyaw=%.3f rad",
            initial_observation_dyaw_rad_
        );

        relative_goal_client_->async_send_request(
            request,
            [this](
                rclcpp::Client<lite_task_controller::srv::SetRelativeGoal>::SharedFuture future
            ) {
                try {
                    const auto response = future.get();

                    if (!response->success) {
                        RCLCPP_WARN(
                            get_logger(),
                            "Initial observation turn rejected by lite: %s. Retry later.",
                            response->message.c_str()
                        );
                        initial_observation_request_sent_ = false;
                        return;
                    }

                    initial_observation_accepted_ = true;
                    initial_observation_accept_time_ = now();

                    RCLCPP_WARN(
                        get_logger(),
                        "Initial observation turn accepted by lite: %s. "
                        "Wait %.2f s before planning.",
                        response->message.c_str(),
                        initial_observation_wait_after_accept_s_
                    );
                } catch (const std::exception& e) {
                    RCLCPP_WARN(
                        get_logger(),
                        "Initial observation turn service call failed: %s. Retry later.",
                        e.what()
                    );
                    initial_observation_request_sent_ = false;
                }
            }
        );
    }

    bool can_plan_after_initial_observation() const {
        if (!plan_requires_initial_observation_) {
            return true;
        }

        if (!enable_initial_observation_turn_) {
            return true;
        }

        return initial_observation_ready_;
    }

    void try_plan_after_observation(const std::string& reason) {
        if (!can_plan_after_initial_observation()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Skip A* planning before initial observation is ready. reason=%s, received=%ld/12",
                reason.c_str(),
                received_blocks_.size()
            );
            return;
        }

        plan_and_publish();
    }

    void camera_gimbal_state_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        camera_look_backward_ = msg->data;

        RCLCPP_WARN(
            get_logger(),
            "Camera gimbal state updated from topic %s: look_backward=%d",
            camera_gimbal_state_topic_.c_str(),
            camera_look_backward_ ? 1 : 0
        );
    }

    void init_blocks() {
        blocks_.clear();
        for (const auto & block : robot_common::r2::blocks()) {
            blocks_[block.id] = {block.id, block.x, block.y, block.h};
        }
    }

    void init_edges_by_position() {
        double neighbor_distance = get_parameter("neighbor_distance").as_double();
        double tolerance = get_parameter("neighbor_tolerance").as_double();

        for (const auto& kv : blocks_) {
            adjacency_[kv.first] = {};
        }

        for (const auto& from_kv : blocks_) {
            int i = from_kv.first;

            for (const auto& to_kv : blocks_) {
                int j = to_kv.first;

                if (i == j) continue;

                double dx = blocks_[j].x - blocks_[i].x;
                double dy = blocks_[j].y - blocks_[i].y;
                double d = std::sqrt(dx * dx + dy * dy);

                if (std::fabs(d - neighbor_distance) <= tolerance) {
                    adjacency_[i].push_back(j);
                }
            }
        }

        RCLCPP_INFO(get_logger(), "Generated adjacency:");
        for (const auto& kv : adjacency_) {
            std::ostringstream oss;
            oss << "Block " << kv.first << " -> ";
            for (int v : kv.second) {
                oss << v << " ";
            }
            RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_yaw_rad_ = quaternion_to_yaw(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );

        robot_yaw_discrete_ = normalize_yaw(rad_to_discrete_deg(robot_yaw_rad_));
        has_odom_ = true;

        int block = find_block_by_field_position(robot_x_, robot_y_);

        if (block >= 0) {
            if (!has_current_block_ || block != current_block_) {
                RCLCPP_WARN(
                    get_logger(),
                    "R2 current block changed: %d -> %d, odom=(%.3f, %.3f), yaw=%.3f",
                    current_block_,
                    block,
                    robot_x_,
                    robot_y_,
                    robot_yaw_rad_
                );
            }

            current_block_ = block;
            has_current_block_ = true;

            update_current_next_block();

            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Current block=%d, current_next_block=%d",
                current_block_,
                current_next_block_
            );
        } else {
            current_block_ = -1;
            has_current_block_ = false;
            update_current_next_block();
        }
    }

    double quaternion_to_yaw(double x, double y, double z, double w) const {
        const double siny_cosp = 2.0 * (w * z + x * y);
        const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    int rad_to_discrete_deg(double yaw_rad) const {
        double deg = yaw_rad * 180.0 / M_PI;
        int ideg = static_cast<int>(std::round(deg));
        return ideg;
    }

    int find_block_by_field_position(double field_x, double field_y) const {
        for (const auto& kv : blocks_) {
            int block_id = kv.first;

            const auto& c = kv.second;

            bool inside_x =
                field_x >= c.x - block_half_size_m_
                && field_x <= c.x + block_half_size_m_;

            bool inside_y =
                field_y >= c.y - block_half_size_m_
                && field_y <= c.y + block_half_size_m_;

            if (inside_x && inside_y) {
                return block_id;
            }
        }

        return -1;
    }

    void update_current_next_block() {
        current_next_block_ = -1;

        if (current_ordered_block_path_.empty()) {
            return;
        }

        /*
          如果还不知道当前 block：
          默认先看路径中的第一个区块。
        */
        if (!has_current_block_ || current_block_ < 0) {
            current_next_block_ = current_ordered_block_path_.front();
            return;
        }

        int idx = -1;

        for (size_t i = 0; i < current_ordered_block_path_.size(); ++i) {
            if (current_ordered_block_path_[i] == current_block_) {
                idx = static_cast<int>(i);
                break;
            }
        }

        /*
          如果当前 block 不在规划路径里，可能在边界或刚启动。
          这时找距离当前机器人最近的 path block，然后取它的下一个。
        */
        if (idx < 0) {
            double best_d = 1e9;
            int best_idx = -1;

            for (size_t i = 0; i < current_ordered_block_path_.size(); ++i) {
                int b = current_ordered_block_path_[i];

                if (blocks_.find(b) == blocks_.end()) continue;

                const auto& blk = blocks_.at(b);
                double d = std::hypot(robot_x_ - blk.x, robot_y_ - blk.y);

                if (d < best_d) {
                    best_d = d;
                    best_idx = static_cast<int>(i);
                }
            }

            if (best_idx >= 0) {
                if (best_idx + 1 < static_cast<int>(current_ordered_block_path_.size())) {
                    current_next_block_ = current_ordered_block_path_[best_idx + 1];
                } else {
                    current_next_block_ = -1;
                }
            }

            return;
        }

        /*
          核心逻辑：
          当前在 path[idx]，只看 path[idx + 1]。
        */
        if (idx + 1 < static_cast<int>(current_ordered_block_path_.size())) {
            current_next_block_ = current_ordered_block_path_[idx + 1];
        } else {
            current_next_block_ = -1;
        }
    }

    bool is_next_block_r1_obstacle(int kfs_block) const {
        if (!only_track_path_relevant_r1_) {
            return true;
        }

        if (current_next_block_ < 0) {
            return false;
        }

        return kfs_block == current_next_block_;
    }
    
    bool is_detection_inside_locked_block_radius(
    int locked_block,
    double field_x,
    double field_y,
    double& distance_out
) const {
    distance_out = INF_;

    auto it = blocks_.find(locked_block);
    if (it == blocks_.end()) {
        return false;
    }

    const auto& b = it->second;
    distance_out = std::hypot(field_x - b.x, field_y - b.y);

    return distance_out <= locked_r1_refresh_radius_m_;
}

    void r1_kfs_callback(const std_msgs::msg::String::SharedPtr msg) {
        const std::string text = trim(msg->data);

        if (!starts_with(text, r1_type_prefix_)) {
            return;
        }

        if (!has_odom_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Received R1_KFS but odom is unknown. Ignore. msg='%s'",
                text.c_str()
            );
            return;
        }

        if (r1_release_limit_ > 0 && r1_released_count_ >= r1_release_limit_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "R1 release limit reached: %d/%d. Ignore R1_KFS.",
                r1_released_count_,
                r1_release_limit_
            );
            return;
        }

        double cam_x = 0.0;
        double cam_y = 0.0;
        double cam_z = 0.0;

        if (!parse_kfs_camera_xyz(text, cam_x, cam_y, cam_z)) {
            RCLCPP_WARN(
                get_logger(),
                "Failed to parse R1_KFS xyz from msg='%s'",
                text.c_str()
            );
            return;
        }

        double field_x = 0.0;
        double field_y = 0.0;

        camera_detection_to_field_position(cam_x, cam_y, cam_z, field_x, field_y);

        const int kfs_block = find_block_by_field_position(field_x, field_y);
        
        RCLCPP_WARN_THROTTLE(
    get_logger(),
    *get_clock(),
    300,
    "R1_KFS projected result: text='%s', cam=(%.3f, %.3f, %.3f), "
    "field=(%.3f, %.3f), kfs_block=%d, current_block=%d, current_next_block=%d, "
    "look_backward=%d, robot=(%.3f, %.3f, yaw=%.3f)",
    text.c_str(),
    cam_x,
    cam_y,
    cam_z,
    field_x,
    field_y,
    kfs_block,
    current_block_,
    current_next_block_,
    camera_look_backward_ ? 1 : 0,
    robot_x_,
    robot_y_,
    robot_yaw_rad_
);


        if (kfs_block < 0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "R1_KFS out of blocks. cam=(%.3f, %.3f, %.3f), field=(%.3f, %.3f), robot=(%.3f, %.3f, %.3f)",
                cam_x,
                cam_y,
                cam_z,
                field_x,
                field_y,
                robot_x_,
                robot_y_,
                robot_yaw_rad_
            );
            return;
        }

        /*
          不再因为 dynamic_passable_blocks_ 忽略。
          否则同一个位置 R1_KFS 清除后再次放上去，会被永久忽略。
        */

        if (only_track_id_type_r1_block_) {
            if (block_types_[kfs_block] != type_r1_) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "R1_KFS block=%d detected by vision, but /id_type type=%d not R1=%d. "
                    "Continue because visual R1_KFS is trusted.",
                    kfs_block,
                    block_types_[kfs_block],
                    type_r1_
                );
            }
        }

       update_current_next_block();

if (!is_next_block_r1_obstacle(kfs_block)) {
    /*
      如果当前已经在等待某个 locked block，则不要因为 kfs_block != current_next_block
      提前 return。锁定期间应该进入 locked radius 判断。
    */
    if (r1_wait_state_ != R1WaitState::TRACKING_AND_WAITING) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Ignore R1_KFS block=%d because current_next_block=%d. "
            "Only the next entering block is checked.",
            kfs_block,
            current_next_block_
        );
        return;
    }
}

if (r1_wait_state_ == R1WaitState::TRACKING_AND_WAITING) {
    double locked_dist = INF_;

    const bool inside_locked_radius =
        is_detection_inside_locked_block_radius(
            locked_r1_block_,
            field_x,
            field_y,
            locked_dist
        );

    if (!inside_locked_radius) {
        const auto& lb = blocks_.at(locked_r1_block_);

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            500,
            "Already waiting locked R1_KFS block=%d. "
            "Detection projected block=%d, field=(%.3f, %.3f), "
            "locked_center=(%.3f, %.3f), dist=%.3f > radius=%.3f. "
            "Ignore and DO NOT refresh last_seen_time.",
            locked_r1_block_,
            kfs_block,
            field_x,
            field_y,
            lb.x,
            lb.y,
            locked_dist,
            locked_r1_refresh_radius_m_
        );

        return;
    }

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        500,
        "Locked R1_KFS block=%d refreshed by detection. "
        "projected_block=%d, field=(%.3f, %.3f), dist_to_locked=%.3f <= radius=%.3f.",
        locked_r1_block_,
        kfs_block,
        field_x,
        field_y,
        locked_dist,
        locked_r1_refresh_radius_m_
    );
}

if (r1_wait_state_ == R1WaitState::IDLE) {
    locked_r1_block_ = kfs_block;
    has_locked_r1_block_ = true;
    r1_wait_state_ = R1WaitState::TRACKING_AND_WAITING;

    std_msgs::msg::Int32 wait_msg;
    wait_msg.data = locked_r1_block_;
    r1_wait_pub_->publish(wait_msg);

    RCLCPP_WARN(
        get_logger(),
        "R1_KFS detected on CURRENT NEXT BLOCK. current_block=%d, next_block=%d. "
        "Lock block=%d and publish /r2_wait_for_r1_kfs.",
        current_block_,
        current_next_block_,
        locked_r1_block_
    );
}

auto& state = r1_track_by_block_[locked_r1_block_];
state.seen_once = true;
state.last_seen_time = now();

RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    500,
    "Tracked R1_KFS next-block obstacle. block=%d, current_block=%d, next_block=%d, "
    "cam=(%.3f, %.3f, %.3f), field=(%.3f, %.3f), robot=(%.3f, %.3f, yaw=%.3f)",
    locked_r1_block_,
    current_block_,
    current_next_block_,
    cam_x,
    cam_y,
    cam_z,
    field_x,
    field_y,
    robot_x_,
    robot_y_,
    robot_yaw_rad_
);


        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            500,
            "Tracked R1_KFS next-block obstacle. block=%d, current_block=%d, next_block=%d, "
            "cam=(%.3f, %.3f, %.3f), field=(%.3f, %.3f), robot=(%.3f, %.3f, yaw=%.3f)",
            locked_r1_block_,
            current_block_,
            current_next_block_,
            cam_x,
            cam_y,
            cam_z,
            field_x,
            field_y,
            robot_x_,
            robot_y_,
            robot_yaw_rad_
        );
    }

    void camera_detection_to_field_position(
    double cam_x,
    double cam_y,
    double cam_z,
    double& field_x,
    double& field_y
) {
    /*
      Camera coordinate:
        cam_z: camera optical forward positive
        cam_x: camera right positive
        cam_y: camera up positive, not used in horizontal projection

      Base coordinate:
        base x: robot front positive
        base y: robot left positive

      Front gimbal:
        detection_base_x =  cam_z
        detection_base_y = -cam_x

      Backward gimbal:
        detection_base_x = -cam_z

        detection_base_y 的符号由参数控制：
          backward_camera_x_positive_to_robot_left = true:
              detection_base_y = cam_x
          backward_camera_x_positive_to_robot_left = false:
              detection_base_y = -cam_x

      你当前日志显示，后视时 detection_base_y = cam_x 会把 6 号投到 3 号附近。
      所以默认 false，即后视使用 detection_base_y = -cam_x。
    */

    double robot_yaw = robot_yaw_rad_;

    if (!use_odom_yaw_for_camera_transform_) {
        robot_yaw = discrete_yaw_to_rad(robot_yaw_discrete_);
    }

    double cam_offset_base_x = camera_forward_offset_x_m_;
    double cam_offset_base_y = camera_forward_offset_y_m_;

    double detection_base_x = 0.0;
    double detection_base_y = 0.0;

    if (!camera_look_backward_) {
        cam_offset_base_x = camera_forward_offset_x_m_;
        cam_offset_base_y = camera_forward_offset_y_m_;

        detection_base_x = cam_z;
        detection_base_y = -cam_x;
    } else {
        cam_offset_base_x = camera_backward_offset_x_m_;
        cam_offset_base_y = camera_backward_offset_y_m_;

        detection_base_x = -cam_z;

        if (backward_camera_x_positive_to_robot_left_) {
            detection_base_y = cam_x;
        } else {
            detection_base_y = -cam_x;
        }
    }

    const double base_x = cam_offset_base_x + detection_base_x;
    const double base_y = cam_offset_base_y + detection_base_y;

    const double c = std::cos(robot_yaw);
    const double s = std::sin(robot_yaw);

    field_x = robot_x_ + c * base_x - s * base_y;
    field_y = robot_y_ + s * base_x + c * base_y;

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        500,
        "Camera projection BASE model: look_backward=%d, "
        "cam=(x/right=%.3f, y/up=%.3f, z/forward=%.3f), "
        "det_base=(forward/x=%.3f, left/y=%.3f), "
        "offset_base=(%.3f, %.3f), base_total=(%.3f, %.3f), "
        "robot=(%.3f, %.3f, yaw=%.3f), field=(%.3f, %.3f), "
        "backward_x_positive_to_robot_left=%d",
        camera_look_backward_ ? 1 : 0,
        cam_x,
        cam_y,
        cam_z,
        detection_base_x,
        detection_base_y,
        cam_offset_base_x,
        cam_offset_base_y,
        base_x,
        base_y,
        robot_x_,
        robot_y_,
        robot_yaw,
        field_x,
        field_y,
        backward_camera_x_positive_to_robot_left_ ? 1 : 0
    );
}


    double discrete_yaw_to_rad(int yaw_deg) const {
        return static_cast<double>(normalize_yaw(yaw_deg)) * M_PI / 180.0;
    }

    double normalize_angle_rad(double a) const {
        while (a > M_PI) {
            a -= 2.0 * M_PI;
        }

        while (a < -M_PI) {
            a += 2.0 * M_PI;
        }

        return a;
    }

    void r1_kfs_monitor_timer_callback() {
        if (r1_wait_state_ != R1WaitState::TRACKING_AND_WAITING) {
            return;
        }

        if (!has_locked_r1_block_ || locked_r1_block_ < 0 || locked_r1_block_ > 12) {
            return;
        }

        auto it = r1_track_by_block_.find(locked_r1_block_);
        if (it == r1_track_by_block_.end()) {
            return;
        }

        auto& state = it->second;

        if (!state.seen_once) {
            return;
        }

        if (state.last_seen_time.nanoseconds() == 0) {
            return;
        }

        const double lost_time = (now() - state.last_seen_time).seconds();

        if (lost_time < lost_confirm_time_) {
            return;
        }

        add_r1_released_passable_block(locked_r1_block_, lost_time);

        r1_released_count_++;

        std_msgs::msg::Int32 cleared_msg;
        cleared_msg.data = locked_r1_block_;
        r1_cleared_pub_->publish(cleared_msg);

        RCLCPP_WARN(
            get_logger(),
            "R1_KFS block=%d cleared after lost %.3f sec. Publish /r2_r1_kfs_cleared.",
            locked_r1_block_,
            lost_time
        );

        /*
          清除后立刻回到 IDLE。
          这样同一个 block 后续再次出现 R1_KFS，也可以再次等待。
        */
        r1_track_by_block_.erase(locked_r1_block_);
        has_locked_r1_block_ = false;
        locked_r1_block_ = -1;
        r1_wait_state_ = R1WaitState::IDLE;
    }

    void add_r1_released_passable_block(int block, double lost_time) {
        if (block < 1 || block > 12) {
            return;
        }

        /*
          仍然可以把 block 类型改成 empty，方便后续规划。
          但不要在 r1_kfs_callback 里因为 dynamic_passable_blocks_ 而忽略视觉。
        */
        dynamic_passable_blocks_.insert(block);
        block_types_[block] = type_empty_;
        received_blocks_.insert(block);

        RCLCPP_WARN(
            get_logger(),
            "R1_KFS block=%d lost for %.3f sec. Mark block as dynamically passable and empty.",
            block,
            lost_time
        );

        if (replan_after_r1_cleared_ && auto_plan_when_update_) {
            if (wait_all_blocks_before_plan_ && received_blocks_.size() < 12) {
                RCLCPP_WARN(
                    get_logger(),
                    "R1 cleared but received=%ld/12, skip replan.",
                    received_blocks_.size()
                );
                return;
            }

            RCLCPP_WARN(
                get_logger(),
                "Trigger replan after R1 block=%d cleared.",
                block
            );

            try_plan_after_observation("R1 cleared");
        } else {
            RCLCPP_WARN(
                get_logger(),
                "No immediate replan after R1 cleared because replan_after_r1_cleared=false."
            );
        }
    }

    bool parse_kfs_camera_xyz(
        const std::string& text,
        double& x,
        double& y,
        double& z
    ) const {
        static const std::regex pattern(
            R"(.*-x:([-+]?[0-9]*\.?[0-9]+)-y:([-+]?[0-9]*\.?[0-9]+)-z:([-+]?[0-9]*\.?[0-9]+).*)"
        );

        std::smatch match;
        if (!std::regex_match(text, match, pattern)) {
            return false;
        }

        try {
            x = std::stod(match[1].str());
            y = std::stod(match[2].str());
            z = std::stod(match[3].str());
        } catch (...) {
            return false;
        }

        return true;
    }

    static bool starts_with(const std::string& text, const std::string& prefix) {
        if (text.size() < prefix.size()) return false;
        return text.compare(0, prefix.size(), prefix) == 0;
    }

    static std::string trim(const std::string& input) {
        const auto begin = input.find_first_not_of(" \t\r\n");
        if (begin == std::string::npos) return "";
        const auto end = input.find_last_not_of(" \t\r\n");
        return input.substr(begin, end - begin + 1);
    }

    void id_type_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) {
            RCLCPP_WARN(get_logger(), "Invalid /id_type msg, size < 2");
            return;
        }

        int id = static_cast<int>(msg->data[0]);
        int type = static_cast<int>(msg->data[1]);

        if (id < 1 || id > 12) {
            RCLCPP_WARN(get_logger(), "Invalid block id: %d", id);
            return;
        }

        int old_type = block_types_[id];
        bool first_received = (received_blocks_.count(id) == 0);
        bool type_changed = (old_type != type);

        /*
          如果 dynamic_passable_blocks_ 中的 block 后续 /id_type 又报告 R1，
          这里允许恢复成 R1。
          这样真实环境重新放回 R1 时，地图状态也能更新。
        */
        block_types_[id] = type;

        if (type == type_r1_) {
            dynamic_passable_blocks_.erase(id);
        }

        received_blocks_.insert(id);

        RCLCPP_INFO(
            get_logger(),
            "Update KFS info: block=%d, type=%d, old=%d, changed=%d, first=%d, received=%ld/12",
            id,
            type,
            old_type,
            type_changed,
            first_received,
            received_blocks_.size()
        );

        if (!first_received && !type_changed) {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Duplicated /id_type block=%d type=%d, skip replan.",
                id,
                type
            );
            return;
        }

        if (auto_plan_when_update_) {
            if (wait_all_blocks_before_plan_ && received_blocks_.size() < 12) {
                RCLCPP_INFO(
                    get_logger(),
                    "Wait all blocks before planning: received=%ld/12",
                    received_blocks_.size()
                );
                return;
            }

            try_plan_after_observation("/id_type update");
        }
    }

    void plan_and_publish() {
        std::vector<int> r2_blocks;

        for (int i = 1; i <= 12; ++i) {
            if (block_types_[i] == type_r2_) {
                r2_blocks.push_back(i);
            }
        }

        if (r2_blocks.empty()) {
            RCLCPP_WARN(get_logger(), "No R2 KFS blocks found. Skip planning.");
            return;
        }

        int need_count = target_count_;
        if (static_cast<int>(r2_blocks.size()) < target_count_) {
            need_count = static_cast<int>(r2_blocks.size());
        }

        RCLCPP_INFO(get_logger(), "R2 candidate blocks:");
        {
            std::ostringstream oss;
            for (int b : r2_blocks) oss << b << " ";
            RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
        }

        PlanResult best;
        best.cost = INF_;

        if (need_count == 1) {
            for (int a : r2_blocks) {
                evaluate_order({a}, best);
            }
        } else {
            for (size_t i = 0; i < r2_blocks.size(); ++i) {
                for (size_t j = 0; j < r2_blocks.size(); ++j) {
                    if (i == j) continue;
                    evaluate_order({r2_blocks[i], r2_blocks[j]}, best);
                }
            }
        }

        if (best.cost >= INF_ / 2.0) {
            RCLCPP_ERROR(get_logger(), "No feasible path found.");
            return;
        }

        update_path_relevant_blocks(best);
        publish_plan(best);
    }

    void update_path_relevant_blocks(const PlanResult& plan) {
        current_path_blocks_.clear();
        current_path_relevant_blocks_.clear();
        current_ordered_block_path_.clear();
        current_next_block_ = -1;

        int last = -999;

        for (const auto& s : plan.states) {
            const int b = s.block;

            if (blocks_.find(b) == blocks_.end()) {
                continue;
            }

            current_path_blocks_.insert(b);
            current_path_relevant_blocks_.insert(b);

            if (b != last) {
                current_ordered_block_path_.push_back(b);
                last = b;
            }
        }

        update_current_next_block();

        std::ostringstream oss;
        oss << "Current ordered path blocks: ";
        for (int b : current_ordered_block_path_) {
            oss << b << " ";
        }

        oss << ", current_block=" << current_block_
            << ", current_next_block=" << current_next_block_;

        RCLCPP_WARN(get_logger(), "%s", oss.str().c_str());
    }

    void evaluate_order(const std::vector<int>& order, PlanResult& best) {
        active_targets_.clear();
        for (int t : order) {
            active_targets_.insert(t);
        }

        for (int exit_block : exit_blocks_) {
            PlanResult partial;
            partial.cost = 0.0;
            partial.targets = order;
            partial.exit_block = exit_block;

            bool feasible = true;

            int planning_start_block = start_block_;
            int planning_start_yaw = start_yaw_;

            if (use_current_block_as_replan_start_ && has_current_block_) {
                planning_start_block = current_block_;
                planning_start_yaw = robot_yaw_discrete_;
            }

            State current = {planning_start_block, planning_start_yaw};

            for (int target_block : order) {
                std::vector<State> pickup_states = generate_pickup_states(target_block);

                SegmentResult best_segment;
                best_segment.cost = INF_;

                double best_pickup_eval_cost = INF_;

                for (const State& goal_state : pickup_states) {
                    SegmentResult seg = astar(current, goal_state);

                    if (seg.cost >= INF_ / 2.0 || seg.states.empty()) {
                        continue;
                    }

                    double eval_cost =
                        seg.cost + pickup_support_preference_cost(target_block, goal_state);

                    if (eval_cost < best_pickup_eval_cost) {
                        best_pickup_eval_cost = eval_cost;
                        best_segment = seg;
                    }
                }

                if (best_segment.cost >= INF_ / 2.0 || best_segment.states.empty()) {
                    feasible = false;
                    break;
                }

                append_segment(partial, best_segment);

                State pickup_support_state = best_segment.states.back();

                PickupTask task;
                task.target = target_block;
                task.support = pickup_support_state.block;
                task.yaw = pickup_support_state.yaw;
                partial.pickup_tasks.push_back(task);

                std::ostringstream pickup_oss;
                pickup_oss
                    << "pickup target=" << task.target
                    << " support=" << task.support
                    << " yaw=" << task.yaw;

                partial.actions.push_back(pickup_oss.str());

                partial.cost += best_pickup_eval_cost;

                current = pickup_support_state;
            }

            if (!feasible) continue;

            SegmentResult best_exit_segment;
            best_exit_segment.cost = INF_;

            for (int yaw : yaws_) {
                State exit_state{exit_block, yaw};
                SegmentResult seg = astar(current, exit_state);
                if (seg.cost < best_exit_segment.cost) {
                    best_exit_segment = seg;
                }
            }

            if (best_exit_segment.cost >= INF_ / 2.0 || best_exit_segment.states.empty()) {
                continue;
            }

            append_segment(partial, best_exit_segment);
            partial.actions.push_back("exit_block_" + std::to_string(exit_block));
            partial.cost += best_exit_segment.cost;

            if (partial.cost < best.cost) {
                best = partial;
            }
        }
    }

    std::vector<State> generate_pickup_states(int kfs_block) {
        std::vector<State> result;

        for (int neighbor : adjacency_[kfs_block]) {
            if (neighbor == 0) continue;

            int yaw_to_kfs = direction_between(neighbor, kfs_block);
            if (yaw_to_kfs < 0) continue;

            result.push_back({neighbor, yaw_to_kfs});
        }

        if (result.empty()) {
            for (int yaw : yaws_) {
                result.push_back({kfs_block, yaw});
            }
        }

        return result;
    }

    SegmentResult astar(const State& start, const State& goal) {
        std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq;

        std::unordered_map<State, double, StateKeyHash> g_score;
        std::unordered_map<State, PrevInfo, StateKeyHash> prev;

        for (const auto& kv : blocks_) {
            for (int yaw : yaws_) {
                g_score[{kv.first, yaw}] = INF_;
            }
        }

        g_score[start] = 0.0;
        pq.push({heuristic_cost(start, goal), start});

        while (!pq.empty()) {
            QueueItem item = pq.top();
            pq.pop();

            State cur = item.state;

            const double expected_f = g_score[cur] + heuristic_cost(cur, goal);
            if (item.cost > expected_f + 1e-9) {
                continue;
            }

            if (cur == goal) {
                break;
            }

            std::vector<std::pair<State, std::pair<std::string, double>>> neighbors;
            get_neighbors(cur, neighbors);

            for (const auto& nb : neighbors) {
                State next = nb.first;
                std::string action = nb.second.first;
                double edge_cost = nb.second.second;

                double tentative_g = g_score[cur] + edge_cost;

                if (tentative_g < g_score[next]) {
                    g_score[next] = tentative_g;
                    prev[next] = {cur, action};
                    pq.push({tentative_g + heuristic_cost(next, goal), next});
                }
            }
        }

        SegmentResult result;
        result.cost = g_score[goal];

        if (result.cost >= INF_ / 2.0) {
            return result;
        }

        std::vector<State> rev_states;
        std::vector<std::string> rev_actions;

        State cur = goal;
        rev_states.push_back(cur);

        while (!(cur == start)) {
            auto it = prev.find(cur);
            if (it == prev.end()) {
                result.cost = INF_;
                return result;
            }

            PrevInfo p = it->second;
            rev_actions.push_back(p.action);
            cur = p.prev;
            rev_states.push_back(cur);
        }

        std::reverse(rev_states.begin(), rev_states.end());
        std::reverse(rev_actions.begin(), rev_actions.end());

        result.states = rev_states;
        result.actions = rev_actions;

        return result;
    }

    double heuristic_cost(const State& from, const State& goal) const {
        auto from_it = blocks_.find(from.block);
        auto goal_it = blocks_.find(goal.block);

        if (from_it == blocks_.end() || goal_it == blocks_.end()) {
            return 0.0;
        }

        const double dx = goal_it->second.x - from_it->second.x;
        const double dy = goal_it->second.y - from_it->second.y;
        const double distance = std::hypot(dx, dy);

        const double neighbor_distance = get_parameter("neighbor_distance").as_double();
        if (neighbor_distance <= 0.0) {
            return 0.0;
        }

        double min_move_cost = std::min({
            get_parameter("cost_forward_flat").as_double(),
            get_parameter("cost_reverse_flat").as_double(),
            get_parameter("cost_climb_up").as_double(),
            get_parameter("cost_start_to_block").as_double()
        });

        if (min_move_cost < 0.0) {
            min_move_cost = 0.0;
        }

        return (distance / neighbor_distance) * min_move_cost;
    }

    void get_neighbors(
        const State& cur,
        std::vector<std::pair<State, std::pair<std::string, double>>>& out
    ) {
        out.clear();

        double cost_turn_90 = get_parameter("cost_turn_90").as_double();

        out.push_back({
            {cur.block, normalize_yaw(cur.yaw + 90)},
            {"turn_left", cost_turn_90}
        });

        out.push_back({
            {cur.block, normalize_yaw(cur.yaw - 90)},
            {"turn_right", cost_turn_90}
        });

        for (int nb : adjacency_[cur.block]) {
            int move_dir = direction_between(cur.block, nb);
            if (move_dir < 0) continue;

            double dh = blocks_[nb].h - blocks_[cur.block].h;

            if (std::fabs(dh) > 0.201) {
                continue;
            }

            double type_cost = block_type_cost(nb);
            if (type_cost >= INF_ / 2.0) {
                continue;
            }

            double height_cost = height_penalty(blocks_[nb].h);

            if (cur.block == 0 && nb == 2) {
                if (cur.yaw == move_dir) {
                    double c = get_parameter("cost_start_to_block").as_double()
                             + height_cost
                             + type_cost;

                    out.push_back({
                        {nb, cur.yaw},
                        {"start_forward_to_2", c}
                    });
                }
                continue;
            }

            if (nb == 0) {
                continue;
            }

            if (cur.yaw == move_dir) {
                if (std::fabs(dh) < 0.001) {
                    double c = get_parameter("cost_forward_flat").as_double()
                             + height_cost
                             + type_cost;

                    out.push_back({
                        {nb, cur.yaw},
                        {"forward_to_" + std::to_string(nb), c}
                    });
                } else if (std::fabs(dh - 0.20) < 0.001) {
                    double c = get_parameter("cost_climb_up").as_double()
                             + height_cost
                             + type_cost;

                    out.push_back({
                        {nb, cur.yaw},
                        {"climb_up_to_" + std::to_string(nb), c}
                    });
                }
            }

            if (opposite_yaw(cur.yaw) == move_dir) {
                if (std::fabs(dh + 0.20) < 0.001) {
                    double c = get_parameter("cost_reverse_down").as_double()
                             + height_cost
                             + type_cost;

                    out.push_back({
                        {nb, cur.yaw},
                        {"reverse_down_to_" + std::to_string(nb), c}
                    });
                }
            }
        }
    }

    double block_type_cost(int block_id) {
        if (block_id == 0) {
            return 0.0;
        }

        if (dynamic_passable_blocks_.count(block_id) > 0) {
            return 0.0;
        }

        int type = block_types_[block_id];

        if (type == type_empty_) {
            return 0.0;
        }

        if (type == type_r1_) {
            if (!allow_pass_r1_) {
                return INF_;
            }
            return get_parameter("cost_r1_wait").as_double();
        }

        if (type == type_r2_) {
            if (active_targets_.count(block_id) > 0) {
                return 0.0;
            }
            return INF_;
        }

        if (type == type_fake_) {
            if (!allow_pass_fake_) {
                return INF_;
            }
            return get_parameter("cost_fake_penalty").as_double();
        }

        return 0.0;
    }

    double height_penalty(double h) {
        if (std::fabs(h - 0.20) < 0.001) {
            return get_parameter("cost_height_20").as_double();
        }

        if (std::fabs(h - 0.40) < 0.001) {
            return get_parameter("cost_height_40").as_double();
        }

        if (std::fabs(h - 0.60) < 0.001) {
            return get_parameter("cost_height_60").as_double();
        }

        return 0.0;
    }

    double pickup_support_preference_cost(int target_block, const State& support_state) const {
        if (blocks_.find(target_block) == blocks_.end()) {
            return 0.0;
        }

        if (blocks_.find(support_state.block) == blocks_.end()) {
            return 0.0;
        }

        const auto& t = blocks_.at(target_block);
        const auto& s = blocks_.at(support_state.block);

        const double dx = t.x - s.x;
        const double dy = t.y - s.y;

        if (dx > 0.5 && std::fabs(dy) < 0.15 && support_state.yaw == 0) {
            return pickup_prefer_left_cost_;
        }

        if (dy > 0.5 && std::fabs(dx) < 0.15 && support_state.yaw == 90) {
            return pickup_prefer_bottom_cost_;
        }

        if (dx < -0.5 && std::fabs(dy) < 0.15 && support_state.yaw == 180) {
            return pickup_penalty_right_cost_;
        }

        if (dy < -0.5 && std::fabs(dx) < 0.15 && support_state.yaw == 270) {
            return pickup_penalty_top_cost_;
        }

        return 0.0;
    }

    int direction_between(int from, int to) const {
        double dx = blocks_.at(to).x - blocks_.at(from).x;
        double dy = blocks_.at(to).y - blocks_.at(from).y;

        if (std::fabs(dx) > std::fabs(dy)) {
            if (dx > 0) return 0;
            else return 180;
        } else {
            if (dy > 0) return 90;
            else return 270;
        }
    }

    int normalize_yaw(int yaw) const {
        yaw %= 360;
        if (yaw < 0) yaw += 360;

        int candidates[4] = {0, 90, 180, 270};
        int best = 0;
        int best_diff = 999;

        for (int c : candidates) {
            int diff = std::abs(c - yaw);
            diff = std::min(diff, 360 - diff);
            if (diff < best_diff) {
                best_diff = diff;
                best = c;
            }
        }

        return best;
    }

    int opposite_yaw(int yaw) const {
        return normalize_yaw(yaw + 180);
    }

    void append_segment(PlanResult& plan, const SegmentResult& seg) {
        if (seg.states.empty()) return;

        if (plan.states.empty()) {
            plan.states.insert(plan.states.end(), seg.states.begin(), seg.states.end());
        } else {
            plan.states.insert(plan.states.end(), seg.states.begin() + 1, seg.states.end());
        }

        plan.actions.insert(plan.actions.end(), seg.actions.begin(), seg.actions.end());
    }

    void publish_plan(const PlanResult& plan) {
        std::vector<int> block_path;
        int last = -1;

        for (const auto& s : plan.states) {
            if (s.block != last) {
                block_path.push_back(s.block);
                last = s.block;
            }
        }

        std_msgs::msg::Int32MultiArray path_msg;
        for (int b : block_path) {
            path_msg.data.push_back(b);
        }
        path_pub_->publish(path_msg);

        std_msgs::msg::Int32MultiArray target_msg;
        for (int t : plan.targets) {
            target_msg.data.push_back(t);
        }
        target_pub_->publish(target_msg);

        std_msgs::msg::String action_msg;
        std::ostringstream oss;

        oss << "cost=" << plan.cost << "\n";

        oss << "targets: ";
        for (int t : plan.targets) {
            oss << t << " ";
        }
        oss << "\n";

        oss << "pickup_sequence:\n";
        for (const auto& p : plan.pickup_tasks) {
            oss
                << "  - target=" << p.target
                << " support=" << p.support
                << " yaw=" << p.yaw
                << "\n";
        }

        oss << "path: ";
        for (int b : block_path) {
            oss << b << " ";
        }
        oss << "\n";

        oss << "states: ";
        for (const auto& s : plan.states) {
            oss << "(" << s.block << "," << s.yaw << ") ";
        }
        oss << "\n";

        oss << "actions:\n";
        for (const auto& a : plan.actions) {
            oss << "  - " << a << "\n";
        }

        action_msg.data = oss.str();
        action_pub_->publish(action_msg);

        publish_marker(block_path);

        RCLCPP_INFO(get_logger(), "========== R2 Plan ==========");
        RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
    }

    void publish_marker(const std::vector<int>& block_path) {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "map";
        marker.header.stamp = now();
        marker.ns = "r2_astar_path";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        for (int b : block_path) {
            if (blocks_.find(b) == blocks_.end()) continue;

            geometry_msgs::msg::Point p;
            p.x = blocks_[b].x;
            p.y = blocks_[b].y;
            p.z = blocks_[b].h + 0.05;
            marker.points.push_back(p);
        }

        marker_pub_->publish(marker);
    }

private:
    const double INF_ = 1e12;

    std::unordered_map<int, Block> blocks_;
    std::unordered_map<int, std::vector<int>> adjacency_;
    std::unordered_map<int, int> block_types_;
    std::set<int> received_blocks_;

    std::set<int> active_targets_;

    std::vector<int> exit_blocks_;
    std::vector<int> yaws_ = {0, 90, 180, 270};

    int start_block_;
    int start_yaw_;

    int type_empty_;
    int type_r1_;
    int type_r2_;
    int type_fake_;

    bool allow_pass_r1_;
    bool allow_pass_fake_;
    bool auto_plan_when_update_;
    bool wait_all_blocks_before_plan_;

    bool enable_initial_observation_turn_{true};
    bool plan_requires_initial_observation_{true};
    bool initial_observation_request_sent_{false};
    bool initial_observation_accepted_{false};
    bool initial_observation_ready_{false};
    double initial_observation_dyaw_rad_{M_PI / 4.0};
    double initial_observation_wait_after_accept_s_{1.5};
    int initial_observation_retry_period_ms_{500};
    std::string relative_goal_service_{"/set_relative_goal"};

    int target_count_;

    double pickup_prefer_left_cost_{-20.0};
    double pickup_prefer_bottom_cost_{-18.0};
    double pickup_penalty_right_cost_{20.0};
    double pickup_penalty_top_cost_{18.0};

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr id_type_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr kfs_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_gimbal_state_sub_;

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr target_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr r1_wait_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr r1_cleared_pub_;

    rclcpp::TimerBase::SharedPtr r1_kfs_monitor_timer_;
    rclcpp::TimerBase::SharedPtr initial_observation_timer_;
    rclcpp::Client<lite_task_controller::srv::SetRelativeGoal>::SharedPtr relative_goal_client_;
    rclcpp::Time initial_observation_accept_time_;

    std::string odom_topic_{"/odometry/filtered"};
    std::string id_type_topic_{"/id_type"};
    std::string vision_kfs_topic_{"/vision/kfs_camera_xyz"};
    std::string r1_type_prefix_{"R1_KFS"};
    std::string camera_gimbal_state_topic_{"/camera_gimbal_look_backward"};

    double block_half_size_m_{0.60};

    double robot_x_{0.0};
    double robot_y_{0.0};
    double robot_yaw_rad_{0.0};
    int robot_yaw_discrete_{0};

    bool has_odom_{false};
    int current_block_{-1};
    bool has_current_block_{false};

    double lost_confirm_time_{0.5};
    int r1_monitor_period_ms_{50};

    double camera_mount_offset_x_m_{0.32};
    double camera_mount_offset_z_m_{0.125};

    double camera_forward_offset_x_m_{0.06};
    double camera_forward_offset_y_m_{-0.32};
    double camera_backward_offset_x_m_{-0.06};
    double camera_backward_offset_y_m_{0.32};
    bool camera_look_backward_{false};

    bool use_odom_yaw_for_camera_transform_{true};
bool backward_camera_x_positive_to_robot_left_{false};
double locked_r1_refresh_radius_m_{0.55};

bool only_track_id_type_r1_block_{false};

    bool only_track_path_relevant_r1_{true};
    bool replan_after_r1_cleared_{false};
    bool use_current_block_as_replan_start_{false};

    int r1_release_limit_{0};
    int r1_released_count_{0};

    R1WaitState r1_wait_state_{R1WaitState::IDLE};

    int locked_r1_block_{-1};
    bool has_locked_r1_block_{false};

    std::unordered_map<int, R1TrackState> r1_track_by_block_;

    std::set<int> dynamic_passable_blocks_;

    std::set<int> current_path_blocks_;
    std::set<int> current_path_relevant_blocks_;

    /*
      有序路径，例如：
        0 2 3 6 9 12

      R1_KFS 等待逻辑只看 current_next_block_。
    */
    std::vector<int> current_ordered_block_path_;
    int current_next_block_{-1};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<R2AStarPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
