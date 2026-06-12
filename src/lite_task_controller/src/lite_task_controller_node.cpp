#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <algorithm>
#include <functional>
#include <sstream>
#include <vector>
#include <cctype>
#include <cstring>
#include <cerrno>
#include <regex>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <set>
#include <tuple>


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/bool.hpp"

#include "lite_task_controller/srv/set_absolute_goal.hpp"
#include "lite_task_controller/srv/set_relative_goal.hpp"
#include "robot_common/r2_field_map.hpp"

using namespace std::chrono_literals;

class LiteTaskController : public rclcpp::Node
{
public:
  LiteTaskController()
  : Node("lite_task_controller")
  {
    declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    declare_parameter<std::string>("target_yaw_topic", "/target_yaw");
    declare_parameter<std::string>("status_topic", "/lite_task_status");
    declare_parameter<std::string>("camera_gimbal_state_topic", "/camera_gimbal_look_backward");

    /*
      注意：
      你的 planner 当前发布的是：
        /r2_planned_path_raw
        /r2_action_sequence_raw

      如果 launch 已经 remap 到：
        /r2_planned_path
        /r2_action_sequence

      那这里可以保持原样。
      如果没有 remap，建议改成 raw：
        "/r2_planned_path_raw"
        "/r2_action_sequence_raw"
    */
    declare_parameter<std::string>("r2_planned_path_topic", "/r2_planned_path");
    declare_parameter<std::string>("r2_action_sequence_topic", "/r2_action_sequence");
    declare_parameter<bool>("enable_r2_path_subscribe", true);
    declare_parameter<bool>("clear_queue_when_new_r2_path", true);
    declare_parameter<bool>("cancel_active_goal_when_new_r2_path", false);

    // R1_KFS 等待/清除 topic
    declare_parameter<std::string>("r2_wait_for_r1_topic", "/r2_wait_for_r1_kfs");
    declare_parameter<std::string>("r2_r1_cleared_topic", "/r2_r1_kfs_cleared");
    // ---------------- Vision lateral correction ----------------
declare_parameter<bool>("enable_vision_lateral_correction", true);

declare_parameter<std::string>(
  "vision_alignment_request_topic",
  "/vision/alignment/request");

declare_parameter<std::string>(
  "vision_recognition_reset_topic",
  "/vision/recognition_reset_cmd");

declare_parameter<std::string>(
  "left_snapshot_offset_topic",
  "/vision/left_arm/snapshot_offset_avg");

declare_parameter<std::string>(
  "right_snapshot_offset_topic",
  "/vision/right_arm/snapshot_offset_avg");

declare_parameter<std::string>(
  "lateral_correction_cmd_topic",
  "/lite_task_controller/lateral_correction_cmd");

declare_parameter<double>("vision_lateral_k", 0.8);
declare_parameter<double>("vision_lateral_tolerance", 0.015);
declare_parameter<double>("vision_lateral_max", 0.25);
declare_parameter<double>("vision_lateral_min", 0.0);
declare_parameter<double>("vision_lateral_stable_time", 0.30);
declare_parameter<double>("vision_offset_timeout", 2.0);
declare_parameter<double>("vision_lateral_sign", -1.0);
declare_parameter<bool>("auto_publish_reset_kfs_after_lateral_done", true);

    declare_parameter<bool>("enable_r1_wait_subscribe", true);

    declare_parameter<double>("r2_default_yaw", 0.0);
    declare_parameter<double>("r2_back_to_stair_yaw", 3.1415926);

    declare_parameter<std::string>("step_cmd_topic", "/step_cmd");
    declare_parameter<std::string>("arm_grab_done_topic", "/arm_grab_done");
    declare_parameter<std::string>("pickup_vision_offset_topic", "/vision/left_arm/snapshot_offset_avg");

    declare_parameter<std::string>("can_interface", "can0");
    declare_parameter<bool>("enable_can_step_cmd", true);

    declare_parameter<bool>("enable_pickup_action", true);
    declare_parameter<double>("pickup_forward_distance", 0.20);
    declare_parameter<double>("pickup_forward_speed", 0.06);
    declare_parameter<double>("pickup_reverse_speed", 0.06);
    declare_parameter<double>("pickup_yaw_match_tolerance", 0.35);

    declare_parameter<double>("control_frequency", 30.0);

    declare_parameter<double>("k_forward", 1.5);
    declare_parameter<double>("k_lateral", 1.2);
    declare_parameter<double>("kd_forward", 0.15);
    declare_parameter<double>("kd_lateral", 0.10);
    declare_parameter<double>("kw", 1.0);

    declare_parameter<double>("forward_max", 4.0);
    declare_parameter<double>("lateral_max", 2.0);
    declare_parameter<double>("wz_max", 0.3);

    declare_parameter<double>("forward_min", 0.0);
    declare_parameter<double>("lateral_min", 0.0);
    declare_parameter<double>("lateral_stop_tolerance", 0.04);
    declare_parameter<bool>("enable_straight_lateral_lock", true);
    declare_parameter<double>("straight_lateral_lock_goal_tolerance", 0.05);
    declare_parameter<double>("straight_lateral_lock_max_lateral_ratio", 0.15);
    declare_parameter<double>("straight_lateral_lock_min_forward", 0.20);
    declare_parameter<double>("straight_lateral_finish_tolerance", 0.12);
    declare_parameter<bool>("enable_yaw_lateral_feedforward", true);
    declare_parameter<double>("yaw_lateral_feedforward_gain", 1.0);
    declare_parameter<double>("yaw_lateral_feedforward_max", 0.25);
    declare_parameter<bool>("enable_r2_lane_drive", true);

    declare_parameter<double>("slow_down_radius", 0.60);
    declare_parameter<double>("pos_tolerance", 0.06);
    declare_parameter<double>("pos_release_tolerance", 0.10);
    declare_parameter<double>("yaw_tolerance", 0.05);

    declare_parameter<double>("pos_stable_time", 0.20);
    declare_parameter<double>("yaw_stable_time", 0.30);
    declare_parameter<double>("hold_time", 0.30);

    declare_parameter<double>("pre_align_yaw_threshold", 0.30);
    declare_parameter<double>("block0_after_open_wait_sec", 1.0);

    declare_parameter<bool>("publish_angular_z", false);
    declare_parameter<bool>("keep_yaw_during_move", true);
    declare_parameter<bool>("use_current_yaw_as_move_target", false);

    declare_parameter<bool>("debug_log", true);
    declare_parameter<bool>("debug_safe_mode", false);
    declare_parameter<bool>("debug_force_zero_angular_z", false);
    declare_parameter<int>("debug_print_every_ms", 1000);

    declare_parameter<double>("debug_safe_forward_max", 0.30);
    declare_parameter<double>("debug_safe_lateral_max", 0.30);
    declare_parameter<double>("debug_safe_wz_max", 0.20);

    declare_parameter<bool>("enable_goal_timeout", true);
    declare_parameter<double>("goal_timeout_sec", 80.0);

    declare_parameter<double>("control_forward_sign", 1.0);
    declare_parameter<double>("control_lateral_sign", 1.0);

    declare_parameter<double>("display_forward_sign", -1.0);
    declare_parameter<double>("display_lateral_sign", -1.0);

    declare_parameter<double>("cmd_x_sign", 1.0);
    declare_parameter<double>("cmd_y_sign", 1.0);

    declare_parameter<bool>("enable_x_slow_zone", true);
    declare_parameter<double>("x_slow_zone_min", 1.0);
    declare_parameter<double>("x_slow_zone_max", 7.5);
    declare_parameter<double>("x_slow_zone_scale", 0.05);
    declare_parameter<double>("x_slow_zone_hysteresis", 0.05);
    declare_parameter<bool>("x_slow_scale_angular", true);

    declare_parameter<bool>("enable_x_boost_zone", true);
    declare_parameter<double>("x_boost_zone_min", 8.195);
    declare_parameter<double>("x_boost_zone_max", 10.595);
    declare_parameter<double>("x_boost_zone_scale", 1.5);
    declare_parameter<double>("x_boost_zone_hysteresis", 0.05);
    declare_parameter<bool>("x_boost_scale_angular", false);

    declare_parameter<bool>("enable_constant_x_speed_zone", false);
    declare_parameter<double>("constant_x_speed_zone_min", 1.0);
    declare_parameter<double>("constant_x_speed_zone_max", 7.5);
    declare_parameter<double>("constant_x_speed_value", 200.0);

    declare_parameter<bool>("enable_acc_limit", false);
    declare_parameter<double>("max_acc_x", 2.0);
    declare_parameter<double>("max_acc_y", 1.2);
    declare_parameter<double>("max_acc_wz", 0.8);

    declare_parameter<double>("max_forward_error_for_control", 3.0);
    declare_parameter<double>("max_lateral_error_for_control", 1.5);

    declare_parameter<bool>("enable_min_approach_speed", true);
    declare_parameter<double>("min_approach_linear_speed", 0.08);
    declare_parameter<double>("min_slow_scale", 0.18);
    declare_parameter<bool>("min_approach_boost_lateral", false);

    declare_parameter<bool>("enable_stuck_release", true);
    declare_parameter<double>("stuck_release_time", 0.80);
    declare_parameter<double>("progress_epsilon", 0.003);

    loadParameters();
    initCanInterface();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20,
      std::bind(&LiteTaskController::odomCallback, this, std::placeholders::_1));

    if (enable_r2_path_subscribe_) {
      r2_path_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
        r2_planned_path_topic_, 10,
        std::bind(&LiteTaskController::r2PathCallback, this, std::placeholders::_1));

      r2_action_sub_ = create_subscription<std_msgs::msg::String>(
        r2_action_sequence_topic_, 10,
        std::bind(&LiteTaskController::r2ActionCallback, this, std::placeholders::_1));

      RCLCPP_INFO(
        get_logger(),
        "R2 path subscriber enabled: path_topic=%s, action_topic=%s",
        r2_planned_path_topic_.c_str(),
        r2_action_sequence_topic_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "R2 path subscriber disabled.");
    }

    if (enable_r1_wait_subscribe_) {
      r1_wait_sub_ = create_subscription<std_msgs::msg::Int32>(
        r2_wait_for_r1_topic_,
        10,
        std::bind(&LiteTaskController::r1WaitCallback, this, std::placeholders::_1));

      r1_cleared_sub_ = create_subscription<std_msgs::msg::Int32>(
        r2_r1_cleared_topic_,
        10,
        std::bind(&LiteTaskController::r1ClearedCallback, this, std::placeholders::_1));

      RCLCPP_WARN(
        get_logger(),
        "R1 wait subscriber enabled: wait_topic=%s, cleared_topic=%s",
        r2_wait_for_r1_topic_.c_str(),
        r2_r1_cleared_topic_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "R1 wait subscriber disabled.");
    }

    arm_grab_done_sub_ = create_subscription<std_msgs::msg::Bool>(
      arm_grab_done_topic_,
      10,
      std::bind(&LiteTaskController::armGrabDoneCallback, this, std::placeholders::_1));

    pickup_vision_offset_sub_ = create_subscription<std_msgs::msg::Float64>(
      pickup_vision_offset_topic_,
      10,
      std::bind(&LiteTaskController::pickupVisionOffsetCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 20);
    target_yaw_pub_ = create_publisher<std_msgs::msg::Float64>(target_yaw_topic_, 20);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
    step_cmd_pub_ = create_publisher<std_msgs::msg::UInt8>(step_cmd_topic_, 10);
    camera_gimbal_state_pub_ = create_publisher<std_msgs::msg::Bool>(camera_gimbal_state_topic_, 10);
        vision_alignment_request_pub_ =
      create_publisher<std_msgs::msg::String>(
        vision_alignment_request_topic_,
        10);

    vision_recognition_reset_pub_ =
      create_publisher<std_msgs::msg::String>(
        vision_recognition_reset_topic_,
        10);

    left_snapshot_offset_sub_ =
      create_subscription<std_msgs::msg::Float64>(
        left_snapshot_offset_topic_,
        10,
        std::bind(
          &LiteTaskController::leftSnapshotOffsetCallback,
          this,
          std::placeholders::_1));

    right_snapshot_offset_sub_ =
      create_subscription<std_msgs::msg::Float64>(
        right_snapshot_offset_topic_,
        10,
        std::bind(
          &LiteTaskController::rightSnapshotOffsetCallback,
          this,
          std::placeholders::_1));

    lateral_correction_cmd_sub_ =
      create_subscription<std_msgs::msg::String>(
        lateral_correction_cmd_topic_,
        10,
        std::bind(
          &LiteTaskController::lateralCorrectionCmdCallback,
          this,
          std::placeholders::_1));


    absolute_goal_srv_ = create_service<lite_task_controller::srv::SetAbsoluteGoal>(
      "/set_absolute_goal",
      std::bind(&LiteTaskController::handleAbsoluteGoal, this,
                std::placeholders::_1, std::placeholders::_2));

    relative_goal_srv_ = create_service<lite_task_controller::srv::SetRelativeGoal>(
      "/set_relative_goal",
      std::bind(&LiteTaskController::handleRelativeGoal, this,
                std::placeholders::_1, std::placeholders::_2));

    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_frequency_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&LiteTaskController::controlLoop, this));

    state_enter_time_ = now();
    pos_reached_since_ = now();
    yaw_reached_since_ = now();
    active_goal_start_time_ = now();
    last_control_time_ = now();
    last_progress_time_ = now();
    pickup_state_start_time_ = now();
    pickup_forward_start_time_ = now();
    pickup_reverse_start_time_ = now();
    latest_pickup_vision_offset_time_ = now();
        last_left_offset_time_ = now();
    last_right_offset_time_ = now();
    vision_lateral_reached_since_ = now();


    RCLCPP_INFO(
      get_logger(),
      "LiteTaskController started: R2 PATH + PICKUP_SEQUENCE + R1_WAIT + CAMERA_GIMBAL_STATE VERSION.");

    RCLCPP_WARN(
      get_logger(),
      "Camera gimbal state publisher enabled: topic=%s, false=front/cmd0x04, true=backward/cmd0x03",
      camera_gimbal_state_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Pickup config: enable=%d, step_topic=%s, arm_done_topic=%s, forward=%.3f, speed=%.3f",
      enable_pickup_action_ ? 1 : 0,
      step_cmd_topic_.c_str(),
      arm_grab_done_topic_.c_str(),
      pickup_forward_distance_,
      pickup_forward_speed_);

    publishStatus();
  }

  ~LiteTaskController()
  {
    if (can_socket_ >= 0) {
      close(can_socket_);
      can_socket_ = -1;
    }
  }

private:
  struct Pose2D
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  enum class MoveMode
  {
    FREE_2D
  };

  struct Waypoint
  {
    Pose2D pose;
    MoveMode mode{MoveMode::FREE_2D};
    std::string source{"unknown"};
    int path_block{-1};
    bool force_pre_align_yaw{false};
    bool use_goal_yaw_during_move{false};
    bool use_lane_drive{false};
    double drive_yaw{0.0};

    bool is_pickup{false};
    int pickup_block{-1};

    // false: CAN ID 0x77 data 0x04, camera looks forward
    // true : CAN ID 0x77 data 0x03, camera looks backward
    bool camera_look_backward{false};
    bool allow_block0_can{true};
    bool use_custom_pos_tolerance{false};
    double custom_pos_tolerance{0.0};
    double custom_pos_release_tolerance{0.0};
  };

  struct PickupTask
  {
    int target{-1};
    int support{-1};
    int yaw_deg{0};
    bool inserted{false};
  };
  enum class ActiveArm
{
  NONE,
  LEFT,
  RIGHT
};

ActiveArm active_vision_arm_{ActiveArm::NONE};

double latest_left_offset_{0.0};
double latest_right_offset_{0.0};

bool left_offset_received_{false};
bool right_offset_received_{false};

rclcpp::Time last_left_offset_time_;
rclcpp::Time last_right_offset_time_;

rclcpp::Time vision_lateral_reached_since_;

bool lateral_correction_active_{false};


  enum class State
  {
    IDLE,
    MOVE_TO_POS,
    BLOCK0_WAIT_AFTER_OPEN,
    ALIGN_YAW,
    HOLD,
    PICKUP_FORWARD,
    PICKUP_WAIT_ARM_DONE,
    VISION_LATERAL_ALIGN,
    PICKUP_REVERSE,
    DONE
  };

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr r2_path_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr r2_action_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_grab_done_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pickup_vision_offset_sub_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr r1_wait_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr r1_cleared_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vision_alignment_request_pub_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vision_recognition_reset_pub_;

rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_snapshot_offset_sub_;
rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_snapshot_offset_sub_;

rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lateral_correction_cmd_sub_;


  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_yaw_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr step_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr camera_gimbal_state_pub_;
  

  rclcpp::Service<lite_task_controller::srv::SetAbsoluteGoal>::SharedPtr absolute_goal_srv_;
  rclcpp::Service<lite_task_controller::srv::SetRelativeGoal>::SharedPtr relative_goal_srv_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string target_yaw_topic_;
  std::string status_topic_;
  std::string camera_gimbal_state_topic_{"/camera_gimbal_look_backward"};

  std::string r2_planned_path_topic_;
  std::string r2_action_sequence_topic_;
  std::string r2_wait_for_r1_topic_;
  std::string r2_r1_cleared_topic_;

  std::string step_cmd_topic_;
  std::string arm_grab_done_topic_;
  std::string pickup_vision_offset_topic_{"/vision/left_arm/snapshot_offset_avg"};

  std::string can_interface_{"can0"};
  bool enable_can_step_cmd_{true};
  int can_socket_{-1};

  bool enable_r2_path_subscribe_{true};
  bool clear_queue_when_new_r2_path_{true};
  bool cancel_active_goal_when_new_r2_path_{false};

  bool enable_r1_wait_subscribe_{true};
  bool waiting_for_r1_kfs_{false};
  int waiting_r1_block_{-1};

  double r2_default_yaw_{0.0};
  double r2_back_to_stair_yaw_{3.1415926};

  bool enable_pickup_action_{true};
  double pickup_forward_distance_{0.20};
  double pickup_forward_speed_{0.06};
  double pickup_reverse_speed_{0.06};

  double pickup_yaw_match_tolerance_{0.35};

  double control_frequency_{30.0};

  double k_forward_{1.5};
  double k_lateral_{1.2};
  double kd_forward_{0.15};
  double kd_lateral_{0.10};
  double kw_{1.0};

  double forward_max_{4.0};
  double lateral_max_{2.0};
  double wz_max_{0.3};

  double forward_min_{0.0};
  double lateral_min_{0.0};
  double lateral_stop_tolerance_{0.04};
  double current_lateral_stop_tolerance_{0.04};
  bool enable_straight_lateral_lock_{true};
  double straight_lateral_lock_goal_tolerance_{0.05};
  double straight_lateral_lock_max_lateral_ratio_{0.15};
  double straight_lateral_lock_min_forward_{0.20};
  double straight_lateral_finish_tolerance_{0.12};
  bool enable_yaw_lateral_feedforward_{true};
  double yaw_lateral_feedforward_gain_{1.0};
  double yaw_lateral_feedforward_max_{0.25};

  double slow_down_radius_{0.60};
  double pos_tolerance_{0.06};
  double pos_release_tolerance_{0.10};
  double current_pos_tolerance_{0.06};
  double current_pos_release_tolerance_{0.10};
  double yaw_tolerance_{0.05};

  double pos_stable_time_{0.20};
  double yaw_stable_time_{0.30};
  double hold_time_{0.30};

  double pre_align_yaw_threshold_{0.30};

  bool publish_angular_z_{false};
  bool keep_yaw_during_move_{true};
  bool use_current_yaw_as_move_target_{false};

  bool debug_log_{true};
  bool debug_safe_mode_{false};
  bool debug_force_zero_angular_z_{false};
  int debug_print_every_ms_{1000};

  double debug_safe_forward_max_{0.30};
  double debug_safe_lateral_max_{0.30};
  double debug_safe_wz_max_{0.20};

  bool enable_goal_timeout_{true};
  double goal_timeout_sec_{80.0};

  double control_forward_sign_{1.0};
  double control_lateral_sign_{1.0};

  double display_forward_sign_{-1.0};
  double display_lateral_sign_{-1.0};

  double cmd_x_sign_{1.0};
  double cmd_y_sign_{1.0};

  bool enable_x_slow_zone_{true};
  double x_slow_zone_min_{1.0};
  double x_slow_zone_max_{7.5};
  double x_slow_zone_scale_{0.05};
  double x_slow_zone_hysteresis_{0.05};
  bool x_slow_scale_angular_{true};
  bool enable_vision_lateral_correction_{true};

std::string vision_alignment_request_topic_{"/vision/alignment/request"};
std::string vision_recognition_reset_topic_{"/vision/recognition_reset_cmd"};

std::string left_snapshot_offset_topic_{"/vision/left_arm/snapshot_offset_avg"};
std::string right_snapshot_offset_topic_{"/vision/right_arm/snapshot_offset_avg"};

std::string lateral_correction_cmd_topic_{"/lite_task_controller/lateral_correction_cmd"};

double vision_lateral_k_{0.8};
double vision_lateral_tolerance_{0.015};
double vision_lateral_max_{0.25};
double vision_lateral_min_{0.0};
double vision_lateral_stable_time_{0.30};
double vision_offset_timeout_{2.0};
double vision_lateral_sign_{1.0};

bool auto_publish_reset_kfs_after_lateral_done_{true};


  bool enable_x_boost_zone_{true};
  double x_boost_zone_min_{8.195};
  double x_boost_zone_max_{10.595};
  double x_boost_zone_scale_{1.5};
  double x_boost_zone_hysteresis_{0.05};
  bool x_boost_scale_angular_{false};

  bool enable_constant_x_speed_zone_{false};
  double constant_x_speed_zone_min_{1.0};
  double constant_x_speed_zone_max_{7.5};
  double constant_x_speed_value_{200.0};

  bool enable_acc_limit_{false};
  double max_acc_x_{2.0};
  double max_acc_y_{1.2};
  double max_acc_wz_{0.8};

  double max_forward_error_for_control_{3.0};
  double max_lateral_error_for_control_{1.5};

  bool enable_min_approach_speed_{true};
  double min_approach_linear_speed_{0.08};
  double min_slow_scale_{0.18};
  bool min_approach_boost_lateral_{false};

  bool enable_stuck_release_{true};
  double stuck_release_time_{0.80};
  double progress_epsilon_{0.003};

  Pose2D current_pose_;
  Pose2D goal_pose_;
  bool odom_received_{false};

  State state_{State::IDLE};
  MoveMode move_mode_{MoveMode::FREE_2D};

  std::deque<Waypoint> goal_queue_;
  bool active_goal_{false};

  rclcpp::Time state_enter_time_;
  rclcpp::Time pos_reached_since_;
  rclcpp::Time yaw_reached_since_;
  rclcpp::Time active_goal_start_time_;
  rclcpp::Time last_control_time_;
  rclcpp::Time last_progress_time_;

  rclcpp::Time pickup_state_start_time_;
  rclcpp::Time pickup_forward_start_time_;
  rclcpp::Time pickup_reverse_start_time_;

  double move_phase_target_yaw_{0.0};

  bool yaw_only_mode_{false};
  bool pre_align_mode_{false};

  geometry_msgs::msg::Twist last_cmd_;

  double last_forward_err_{0.0};
  double last_lateral_err_{0.0};
  bool last_err_valid_{false};

  double best_dist_{1e9};

  bool x_slow_zone_active_{false};
  bool x_boost_zone_active_{false};

  std::vector<int> r2_pickup_targets_;          // 兼容旧格式，只打印，不再用它猜 pickup
  std::vector<PickupTask> r2_pickup_tasks_;     // 新格式：target/support/yaw
  bool has_r2_action_sequence_{false};
  bool has_pickup_sequence_{false};

	  bool current_waypoint_is_pickup_{false};
	  int current_pickup_block_{-1};
	  int current_path_block_{-1};
	  std::string current_waypoint_source_{"unknown"};
		  bool current_force_pre_align_yaw_{false};
		  bool current_use_goal_yaw_during_move_{false};
		  bool current_r2_lane_drive_{false};
		  bool current_r2_lane_goal_accepted_{false};
		  bool straight_lateral_lock_active_{false};
		  bool current_position_accepted_{false};
		  bool current_allow_block0_can_{true};
		  double current_r2_lane_drive_sign_{1.0};
		  bool arm_grab_done_received_{false};
  bool pickup_step_off_sent_{false};
  bool pickup_waiting_vision_lateral_{false};
bool pickup_vision_lateral_done_{false};
bool pickup_reset_kfs_sent_{false};
bool pickup_reverse_after_arm_done_{true};
int pickup_vision_lateral_block_{-1};


  bool pickup_step_on_sent_{false};
  bool infrared_open_sent_at_block0_{false};
  double block0_after_open_wait_sec_{1.0};

  bool has_pickup_vision_offset_{false};
  double latest_pickup_vision_offset_{0.0};
  rclcpp::Time latest_pickup_vision_offset_time_;

  // -1 unknown, 0 front, 180 backward
  int current_camera_gimbal_mode_{-1};

	  std_msgs::msg::Int32MultiArray latest_r2_path_;
	  bool has_latest_r2_path_{false};
	  bool rebuilding_r2_queue_{false};
	  std_msgs::msg::Int32MultiArray deferred_r2_path_;
	  bool has_deferred_r2_path_{false};
	  bool pending_r2_path_waiting_for_odom_{false};

  void loadParameters()
  {
    odom_topic_ = get_parameter("odom_topic").as_string();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    target_yaw_topic_ = get_parameter("target_yaw_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    camera_gimbal_state_topic_ = get_parameter("camera_gimbal_state_topic").as_string();

    r2_planned_path_topic_ = get_parameter("r2_planned_path_topic").as_string();
    r2_action_sequence_topic_ = get_parameter("r2_action_sequence_topic").as_string();
    r2_wait_for_r1_topic_ = get_parameter("r2_wait_for_r1_topic").as_string();
    r2_r1_cleared_topic_ = get_parameter("r2_r1_cleared_topic").as_string();
    enable_r1_wait_subscribe_ = get_parameter("enable_r1_wait_subscribe").as_bool();

    step_cmd_topic_ = get_parameter("step_cmd_topic").as_string();
    arm_grab_done_topic_ = get_parameter("arm_grab_done_topic").as_string();
    pickup_vision_offset_topic_ = get_parameter("pickup_vision_offset_topic").as_string();

    can_interface_ = get_parameter("can_interface").as_string();
    enable_can_step_cmd_ = get_parameter("enable_can_step_cmd").as_bool();

    enable_r2_path_subscribe_ = get_parameter("enable_r2_path_subscribe").as_bool();
    clear_queue_when_new_r2_path_ = get_parameter("clear_queue_when_new_r2_path").as_bool();
    cancel_active_goal_when_new_r2_path_ = get_parameter("cancel_active_goal_when_new_r2_path").as_bool();

    r2_default_yaw_ = get_parameter("r2_default_yaw").as_double();
    r2_back_to_stair_yaw_ = get_parameter("r2_back_to_stair_yaw").as_double();

    enable_pickup_action_ = get_parameter("enable_pickup_action").as_bool();
    pickup_forward_distance_ = get_parameter("pickup_forward_distance").as_double();
    pickup_forward_speed_ = get_parameter("pickup_forward_speed").as_double();
    pickup_reverse_speed_ = get_parameter("pickup_reverse_speed").as_double();

    pickup_yaw_match_tolerance_ = get_parameter("pickup_yaw_match_tolerance").as_double();

    control_frequency_ = get_parameter("control_frequency").as_double();

    k_forward_ = get_parameter("k_forward").as_double();
    k_lateral_ = get_parameter("k_lateral").as_double();
    kd_forward_ = get_parameter("kd_forward").as_double();
    kd_lateral_ = get_parameter("kd_lateral").as_double();
    kw_ = get_parameter("kw").as_double();

    forward_max_ = get_parameter("forward_max").as_double();
    lateral_max_ = get_parameter("lateral_max").as_double();
    wz_max_ = get_parameter("wz_max").as_double();

    forward_min_ = get_parameter("forward_min").as_double();
    lateral_min_ = get_parameter("lateral_min").as_double();
    lateral_stop_tolerance_ =
      std::max(0.0, get_parameter("lateral_stop_tolerance").as_double());
    enable_straight_lateral_lock_ =
      get_parameter("enable_straight_lateral_lock").as_bool();
    straight_lateral_lock_goal_tolerance_ =
      std::max(0.0, get_parameter("straight_lateral_lock_goal_tolerance").as_double());
    straight_lateral_lock_max_lateral_ratio_ =
      std::max(0.0, get_parameter("straight_lateral_lock_max_lateral_ratio").as_double());
    straight_lateral_lock_min_forward_ =
      std::max(0.0, get_parameter("straight_lateral_lock_min_forward").as_double());
    straight_lateral_finish_tolerance_ =
      std::max(0.0, get_parameter("straight_lateral_finish_tolerance").as_double());
    enable_yaw_lateral_feedforward_ =
      get_parameter("enable_yaw_lateral_feedforward").as_bool();
    yaw_lateral_feedforward_gain_ =
      get_parameter("yaw_lateral_feedforward_gain").as_double();
    yaw_lateral_feedforward_max_ =
      std::max(0.0, get_parameter("yaw_lateral_feedforward_max").as_double());

    slow_down_radius_ = get_parameter("slow_down_radius").as_double();
    pos_tolerance_ = get_parameter("pos_tolerance").as_double();
    pos_release_tolerance_ = get_parameter("pos_release_tolerance").as_double();
    yaw_tolerance_ = get_parameter("yaw_tolerance").as_double();

    pos_stable_time_ = get_parameter("pos_stable_time").as_double();
    yaw_stable_time_ = get_parameter("yaw_stable_time").as_double();
    hold_time_ = get_parameter("hold_time").as_double();

    pre_align_yaw_threshold_ = get_parameter("pre_align_yaw_threshold").as_double();

    publish_angular_z_ = get_parameter("publish_angular_z").as_bool();
    keep_yaw_during_move_ = get_parameter("keep_yaw_during_move").as_bool();
    use_current_yaw_as_move_target_ = get_parameter("use_current_yaw_as_move_target").as_bool();

    debug_log_ = get_parameter("debug_log").as_bool();
    debug_safe_mode_ = get_parameter("debug_safe_mode").as_bool();
    debug_force_zero_angular_z_ = get_parameter("debug_force_zero_angular_z").as_bool();
    debug_print_every_ms_ =
      std::max(100, static_cast<int>(get_parameter("debug_print_every_ms").as_int()));

    debug_safe_forward_max_ = get_parameter("debug_safe_forward_max").as_double();
    debug_safe_lateral_max_ = get_parameter("debug_safe_lateral_max").as_double();
    debug_safe_wz_max_ = get_parameter("debug_safe_wz_max").as_double();

    enable_goal_timeout_ = get_parameter("enable_goal_timeout").as_bool();
    goal_timeout_sec_ = get_parameter("goal_timeout_sec").as_double();

    control_forward_sign_ = get_parameter("control_forward_sign").as_double();
    control_lateral_sign_ = get_parameter("control_lateral_sign").as_double();

    display_forward_sign_ = get_parameter("display_forward_sign").as_double();
    display_lateral_sign_ = get_parameter("display_lateral_sign").as_double();

    cmd_x_sign_ = get_parameter("cmd_x_sign").as_double();
    cmd_y_sign_ = get_parameter("cmd_y_sign").as_double();

    enable_x_slow_zone_ = get_parameter("enable_x_slow_zone").as_bool();
    x_slow_zone_min_ = get_parameter("x_slow_zone_min").as_double();
    x_slow_zone_max_ = get_parameter("x_slow_zone_max").as_double();
    x_slow_zone_scale_ = get_parameter("x_slow_zone_scale").as_double();
    x_slow_zone_hysteresis_ = get_parameter("x_slow_zone_hysteresis").as_double();
    x_slow_scale_angular_ = get_parameter("x_slow_scale_angular").as_bool();

    enable_x_boost_zone_ = get_parameter("enable_x_boost_zone").as_bool();
    x_boost_zone_min_ = get_parameter("x_boost_zone_min").as_double();
    x_boost_zone_max_ = get_parameter("x_boost_zone_max").as_double();
    x_boost_zone_scale_ = get_parameter("x_boost_zone_scale").as_double();
    x_boost_zone_hysteresis_ = get_parameter("x_boost_zone_hysteresis").as_double();
    x_boost_scale_angular_ = get_parameter("x_boost_scale_angular").as_bool();

    enable_constant_x_speed_zone_ = get_parameter("enable_constant_x_speed_zone").as_bool();
    constant_x_speed_zone_min_ = get_parameter("constant_x_speed_zone_min").as_double();
    constant_x_speed_zone_max_ = get_parameter("constant_x_speed_zone_max").as_double();
    constant_x_speed_value_ = get_parameter("constant_x_speed_value").as_double();

    enable_acc_limit_ = get_parameter("enable_acc_limit").as_bool();
    max_acc_x_ = get_parameter("max_acc_x").as_double();
    max_acc_y_ = get_parameter("max_acc_y").as_double();
    max_acc_wz_ = get_parameter("max_acc_wz").as_double();

    max_forward_error_for_control_ = get_parameter("max_forward_error_for_control").as_double();
    max_lateral_error_for_control_ = get_parameter("max_lateral_error_for_control").as_double();

    enable_min_approach_speed_ = get_parameter("enable_min_approach_speed").as_bool();
    min_approach_linear_speed_ = get_parameter("min_approach_linear_speed").as_double();
    min_slow_scale_ = get_parameter("min_slow_scale").as_double();
    min_approach_boost_lateral_ = get_parameter("min_approach_boost_lateral").as_bool();

    enable_stuck_release_ = get_parameter("enable_stuck_release").as_bool();
    stuck_release_time_ = get_parameter("stuck_release_time").as_double();
    progress_epsilon_ = get_parameter("progress_epsilon").as_double();
    block0_after_open_wait_sec_ =
  get_parameter("block0_after_open_wait_sec").as_double();
      enable_vision_lateral_correction_ =
      get_parameter("enable_vision_lateral_correction").as_bool();

    vision_alignment_request_topic_ =
      get_parameter("vision_alignment_request_topic").as_string();

    vision_recognition_reset_topic_ =
      get_parameter("vision_recognition_reset_topic").as_string();

    left_snapshot_offset_topic_ =
      get_parameter("left_snapshot_offset_topic").as_string();

    right_snapshot_offset_topic_ =
      get_parameter("right_snapshot_offset_topic").as_string();

    lateral_correction_cmd_topic_ =
      get_parameter("lateral_correction_cmd_topic").as_string();

    vision_lateral_k_ =
      get_parameter("vision_lateral_k").as_double();

    vision_lateral_tolerance_ =
      std::max(0.0001, get_parameter("vision_lateral_tolerance").as_double());

    vision_lateral_max_ =
      std::max(0.0, get_parameter("vision_lateral_max").as_double());

    vision_lateral_min_ =
      std::max(0.0, get_parameter("vision_lateral_min").as_double());

    vision_lateral_stable_time_ =
      std::max(0.0, get_parameter("vision_lateral_stable_time").as_double());

    vision_offset_timeout_ =
      std::max(0.05, get_parameter("vision_offset_timeout").as_double());

    vision_lateral_sign_ =
      get_parameter("vision_lateral_sign").as_double();

    auto_publish_reset_kfs_after_lateral_done_ =
      get_parameter("auto_publish_reset_kfs_after_lateral_done").as_bool();



    if (enable_constant_x_speed_zone_) {
      enable_x_slow_zone_ = true;
      x_slow_zone_min_ = constant_x_speed_zone_min_;
      x_slow_zone_max_ = constant_x_speed_zone_max_;
    }
  }

  void initCanInterface()
  {
    if (!enable_can_step_cmd_) {
      RCLCPP_WARN(get_logger(), "CAN step command disabled by parameter.");
      return;
    }

    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to create CAN socket for interface %s: %s",
        can_interface_.c_str(),
        std::strerror(errno));
      enable_can_step_cmd_ = false;
      return;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", can_interface_.c_str());

    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_WARN(
        get_logger(),
        "CAN interface %s is unavailable: %s. Disable CAN step/camera commands for this run.",
        can_interface_.c_str(),
        std::strerror(errno));

      close(can_socket_);
      can_socket_ = -1;
      enable_can_step_cmd_ = false;
      return;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to bind CAN socket to %s: %s. Disable CAN step/camera commands for this run.",
        can_interface_.c_str(),
        std::strerror(errno));

      close(can_socket_);
      can_socket_ = -1;
      enable_can_step_cmd_ = false;
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "CAN step command enabled: interface=%s, can_id=0x66",
      can_interface_.c_str());
  }

  bool sendCanStepCommand(uint8_t cmd)
  {
    if (!enable_can_step_cmd_) {
      return true;
    }

    if (cmd != 0x01 && cmd != 0x02) {
      RCLCPP_WARN(get_logger(), "Invalid CAN step cmd: 0x%02X", cmd);
      return false;
    }

    if (can_socket_ < 0) {
      RCLCPP_ERROR(
        get_logger(),
        "CAN socket not available. Failed to send cmd=0x%02X on %s",
        cmd,
        can_interface_.c_str());
      return false;
    }

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));

    frame.can_id = 0x66;
    frame.can_dlc = 1;
    frame.data[0] = cmd;

    const ssize_t nbytes = write(can_socket_, &frame, sizeof(frame));
    if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to send CAN frame: interface=%s, id=0x66, data=0x%02X, nbytes=%zd, error=%s",
        can_interface_.c_str(),
        cmd,
        nbytes,
        std::strerror(errno));
      return false;
    }

    RCLCPP_WARN(
      get_logger(),
      "CAN STEP CMD sent: interface=%s, id=0x66, data=0x%02X",
      can_interface_.c_str(),
      cmd);

    return true;
  }

  bool sendCanCameraGimbalCommand(uint8_t cmd)
  {
    if (!enable_can_step_cmd_) {
      RCLCPP_WARN(get_logger(), "CAN disabled. Skip camera gimbal cmd=0x%02X", cmd);
      return true;
    }

    if (cmd != 0x03 && cmd != 0x04) {
      RCLCPP_WARN(get_logger(), "Invalid camera gimbal cmd: 0x%02X. Valid: 0x03 or 0x04", cmd);
      return false;
    }

    if (can_socket_ < 0) {
      RCLCPP_ERROR(
        get_logger(),
        "CAN socket not available. Failed to send camera gimbal cmd=0x%02X on %s",
        cmd,
        can_interface_.c_str());
      return false;
    }

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x77;
    frame.can_dlc = 1;
    frame.data[0] = cmd;

    const ssize_t nbytes = write(can_socket_, &frame, sizeof(frame));
    if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to send CAMERA GIMBAL CAN frame: interface=%s, id=0x77, data=0x%02X, nbytes=%zd, error=%s",
        can_interface_.c_str(), cmd, nbytes, std::strerror(errno));
      return false;
    }

    RCLCPP_WARN(
      get_logger(),
      "CAMERA GIMBAL CAN CMD sent: interface=%s, id=0x77, data=0x%02X",
      can_interface_.c_str(), cmd);

    return true;
  }

  void publishCameraGimbalState(bool look_backward, const std::string & reason)
  {
    if (!camera_gimbal_state_pub_) {
      return;
    }

    std_msgs::msg::Bool msg;
    msg.data = look_backward;
    camera_gimbal_state_pub_->publish(msg);

    RCLCPP_WARN(
      get_logger(),
      "Camera gimbal state published: topic=%s, look_backward=%d, reason=%s",
      camera_gimbal_state_topic_.c_str(),
      look_backward ? 1 : 0,
      reason.c_str());
  }

  void setCameraLookDirection(bool look_backward, const std::string & reason)
  {
    const int target_mode = look_backward ? 180 : 0;
    if (current_camera_gimbal_mode_ == target_mode) {
      RCLCPP_INFO(get_logger(), "Camera gimbal already in mode=%d deg. Skip CAN. reason=%s", target_mode, reason.c_str());
      publishCameraGimbalState(look_backward, "already in target mode, " + reason);
      return;
    }

    const uint8_t cmd = look_backward ? 0x03 : 0x04;
    const bool ok = sendCanCameraGimbalCommand(cmd);
    if (ok) {
      current_camera_gimbal_mode_ = target_mode;
      publishCameraGimbalState(look_backward, "CAN success, " + reason);
      RCLCPP_WARN(get_logger(), "Set camera gimbal to %d deg, can_id=0x77, cmd=0x%02X, reason=%s", target_mode, cmd, reason.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to set camera gimbal to %d deg, can_id=0x77, cmd=0x%02X, reason=%s", target_mode, cmd, reason.c_str());
    }
  }

  static double normalizeAngle(double a)
  {
    return std::atan2(std::sin(a), std::cos(a));
  }

  static double degToRad(double deg)
  {
    return deg * M_PI / 180.0;
  }

  static double angleDiff(double target, double current)
  {
    return normalizeAngle(target - current);
  }

  static double clampAbs(double v, double lim)
  {
    return std::clamp(v, -std::abs(lim), std::abs(lim));
  }

  static double quatToYaw(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static double limitRate(double target, double current, double max_delta)
  {
    const double delta = target - current;
    if (delta > max_delta) return current + max_delta;
    if (delta < -max_delta) return current - max_delta;
    return target;
  }

  static void applyMinPlanarSpeed(
    double & vx,
    double & vy,
    double min_speed,
    double max_vx,
    double max_vy,
    bool boost_lateral)
  {
    const double min_s = std::abs(min_speed);

    if (!boost_lateral) {
      if (std::abs(vx) < 1e-9 || std::abs(vx) >= min_s) return;
      vx = std::copysign(min_s, vx);
      vx = clampAbs(vx, max_vx);
      vy = clampAbs(vy, max_vy);
      return;
    }

    const double speed = std::hypot(vx, vy);

    if (speed < 1e-9 || speed >= min_s) return;

    const double ratio = min_s / speed;
    vx *= ratio;
    vy *= ratio;

    vx = clampAbs(vx, max_vx);
    vy = clampAbs(vy, max_vy);
  }

  geometry_msgs::msg::Twist applyAccelerationLimit(const geometry_msgs::msg::Twist & target_cmd)
  {
    if (!enable_acc_limit_) {
      last_cmd_ = target_cmd;
      return target_cmd;
    }

    const double dt = 1.0 / std::max(1.0, control_frequency_);

    geometry_msgs::msg::Twist out;
    out.linear.x = limitRate(target_cmd.linear.x, last_cmd_.linear.x, std::abs(max_acc_x_) * dt);
    out.linear.y = limitRate(target_cmd.linear.y, last_cmd_.linear.y, std::abs(max_acc_y_) * dt);
    out.angular.z = limitRate(target_cmd.angular.z, last_cmd_.angular.z, std::abs(max_acc_wz_) * dt);

    last_cmd_ = out;
    return out;
  }

const char* stateToString(State s) const
{
  switch (s) {
    case State::IDLE: return "IDLE";
    case State::MOVE_TO_POS: return "MOVE_TO_POS";
    case State::BLOCK0_WAIT_AFTER_OPEN: return "BLOCK0_WAIT_AFTER_OPEN";
    case State::ALIGN_YAW: return "ALIGN_YAW";
    case State::HOLD: return "HOLD";
    case State::PICKUP_FORWARD: return "PICKUP_FORWARD";
    case State::PICKUP_WAIT_ARM_DONE: return "PICKUP_WAIT_ARM_DONE";
    case State::VISION_LATERAL_ALIGN: return "VISION_LATERAL_ALIGN";
    case State::PICKUP_REVERSE: return "PICKUP_REVERSE";
    case State::DONE: return "DONE";
    default: return "UNKNOWN";
  }
}



  const char* moveModeToString(MoveMode m) const
  {
    switch (m) {
      case MoveMode::FREE_2D: return "FREE_2D";
      default: return "UNKNOWN";
    }
  }

  void publishStatus()
  {
    std_msgs::msg::String msg;
    msg.data = stateToString(state_);
    status_pub_->publish(msg);
  }

  void publishStop()
  {
    geometry_msgs::msg::Twist cmd;
    last_cmd_ = cmd;
    cmd_vel_pub_->publish(cmd);
  }

  void publishHardStop()
  {
    geometry_msgs::msg::Twist cmd;
    last_cmd_ = cmd;
    cmd_vel_pub_->publish(cmd);
  }

  void publishTargetYaw(double yaw_rad)
  {
    std_msgs::msg::Float64 msg;
    msg.data = yaw_rad;
    target_yaw_pub_->publish(msg);
  }

  void publishStepCommand(uint8_t cmd, const std::string & reason)
  {
    if (cmd != 0x01 && cmd != 0x02) {
      RCLCPP_WARN(get_logger(), "Invalid step cmd: 0x%02X", cmd);
      return;
    }

    std_msgs::msg::UInt8 msg;
    msg.data = cmd;
    step_cmd_pub_->publish(msg);

    RCLCPP_WARN(
      get_logger(),
      "STEP CMD published: topic=%s, data=0x%02X, reason=%s",
      step_cmd_topic_.c_str(),
      cmd,
      reason.c_str());

    const bool can_ok = sendCanStepCommand(cmd);

    if (!can_ok) {
      RCLCPP_ERROR(
        get_logger(),
        "STEP CMD CAN send failed: interface=%s, id=0x66, data=0x%02X, reason=%s",
        can_interface_.c_str(),
        cmd,
        reason.c_str());
    }
  }

void publishInfraredOpenOnceAtBlock0(int block_id, const std::string & reason)
{
  if (block_id != 0 || infrared_open_sent_at_block0_) {
    return;
  }

  publishStepCommand(
    0x01,
    "arrived block 0 waypoint x=2.160 y=-1.530, send CAN id=0x66 data=0x01 once, " + reason);

  infrared_open_sent_at_block0_ = true;
}


  void publishInfraredOpenOnceIfCurrentNearBlock0(const std::string & reason)
  {
    if (infrared_open_sent_at_block0_) {
      return;
    }

    double block0_x = 0.0;
    double block0_y = 0.0;
    if (!getBlockCenter(0, block0_x, block0_y)) {
      return;
    }

    const double dist = std::hypot(current_pose_.x - block0_x, current_pose_.y - block0_y);
    if (dist > 0.30) {
      return;
    }

    publishStepCommand(
      0x01,
      "current odom is near path block 0, open infrared ranging module once, " +
      reason + ", dist=" + std::to_string(dist));

    infrared_open_sent_at_block0_ = true;
  }

  void r1WaitCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    waiting_for_r1_kfs_ = true;
    waiting_r1_block_ = msg->data;

    publishHardStop();

    RCLCPP_WARN(
      get_logger(),
      "Received /r2_wait_for_r1_kfs block=%d. Pause current motion and keep queue.",
      waiting_r1_block_);
  }

  void r1ClearedCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    const int cleared_block = msg->data;

    if (!waiting_for_r1_kfs_) {
      RCLCPP_WARN(
        get_logger(),
        "Received /r2_r1_kfs_cleared block=%d but controller is not waiting. Ignore.",
        cleared_block);
      return;
    }

    if (cleared_block != waiting_r1_block_) {
      RCLCPP_WARN(
        get_logger(),
        "Received /r2_r1_kfs_cleared block=%d but waiting block=%d. Ignore.",
        cleared_block,
        waiting_r1_block_);
      return;
    }

    waiting_for_r1_kfs_ = false;
    waiting_r1_block_ = -1;

    last_control_time_ = now();
    last_progress_time_ = now();
    if (active_goal_) {
      active_goal_start_time_ = now();
    }

    RCLCPP_WARN(
      get_logger(),
      "R1_KFS block=%d cleared. Resume current task.",
      cleared_block);
  }

  void armGrabDoneCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      return;
    }

    if (state_ != State::PICKUP_WAIT_ARM_DONE) {
      RCLCPP_WARN(
        get_logger(),
        "Arm grab done ignored because current state is %s, current_pickup_block=%d",
        stateToString(state_),
        current_pickup_block_);
      return;
    }

    arm_grab_done_received_ = true;

    RCLCPP_WARN(
      get_logger(),
      "Arm grab done accepted. current_pickup_block=%d, state=%s",
      current_pickup_block_,
      stateToString(state_));
  }

  void pickupVisionOffsetCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    latest_pickup_vision_offset_ = msg->data;
    latest_pickup_vision_offset_time_ = now();
    has_pickup_vision_offset_ = true;

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      300,
      "Pickup vision offset received: topic=%s offset=%.4f m",
      pickup_vision_offset_topic_.c_str(),
      latest_pickup_vision_offset_);
  }
    const char * activeArmToString(ActiveArm arm) const
  {
    switch (arm) {
      case ActiveArm::LEFT:
        return "LEFT";
      case ActiveArm::RIGHT:
        return "RIGHT";
      case ActiveArm::NONE:
      default:
        return "NONE";
    }
  }

  void leftSnapshotOffsetCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Invalid left snapshot offset received.");
      return;
    }

    latest_left_offset_ = msg->data;
    left_offset_received_ = true;
    last_left_offset_time_ = now();

    RCLCPP_WARN(
      get_logger(),
      "[VISION_OFFSET][LEFT] %.6f m",
      latest_left_offset_);
  }

  void rightSnapshotOffsetCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Invalid right snapshot offset received.");
      return;
    }

    latest_right_offset_ = msg->data;
    right_offset_received_ = true;
    last_right_offset_time_ = now();

    RCLCPP_WARN(
      get_logger(),
      "[VISION_OFFSET][RIGHT] %.6f m",
      latest_right_offset_);
  }

  void lateralCorrectionCmdCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string cmd = msg->data;

    if (!enable_vision_lateral_correction_) {
      RCLCPP_WARN(get_logger(), "Vision lateral correction disabled.");
      return;
    }

    if (!odom_received_) {
      RCLCPP_WARN(get_logger(), "Cannot start vision lateral correction: no odom.");
      return;
    }

    if (cmd == "left" || cmd == "LEFT" || cmd == "A" || cmd == "a") {
      startVisionLateralCorrection(ActiveArm::LEFT);
    } else if (cmd == "right" || cmd == "RIGHT" || cmd == "B" || cmd == "b") {
      startVisionLateralCorrection(ActiveArm::RIGHT);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Unknown lateral correction cmd: '%s'. Use left/right/A/B.",
        cmd.c_str());
    }
  }

    void startVisionLateralCorrection(ActiveArm arm)
  {
    if (arm == ActiveArm::NONE) {
      RCLCPP_WARN(get_logger(), "Cannot start vision lateral correction: arm=NONE.");
      return;
    }

    active_vision_arm_ = arm;
    lateral_correction_active_ = true;
    vision_lateral_reached_since_ = now();

    if (arm == ActiveArm::LEFT) {
      left_offset_received_ = false;
    } else if (arm == ActiveArm::RIGHT) {
      right_offset_received_ = false;
    }

    publishHardStop();

    /*
      注意：
      不要 clear goal_queue_。
      pickup 流程中横向校准是路径中间动作，清空队列会导致后续路径丢失。
    */
    active_goal_ = true;
    active_goal_start_time_ = now();

    std_msgs::msg::String req;
    req.data = "Start_chassis_lateral_correction";
    vision_alignment_request_pub_->publish(req);

    RCLCPP_WARN(
      get_logger(),
      "[VISION_LATERAL] Start correction. arm=%s. Publish %s='%s'",
      activeArmToString(arm),
      vision_alignment_request_topic_.c_str(),
      req.data.c_str());

    setState(State::VISION_LATERAL_ALIGN);
  }


  bool getActiveVisionOffset(double & offset, rclcpp::Time & stamp) const
  {
    if (active_vision_arm_ == ActiveArm::LEFT) {
      if (!left_offset_received_) {
        return false;
      }
      offset = latest_left_offset_;
      stamp = last_left_offset_time_;
      return true;
    }

    if (active_vision_arm_ == ActiveArm::RIGHT) {
      if (!right_offset_received_) {
        return false;
      }
      offset = latest_right_offset_;
      stamp = last_right_offset_time_;
      return true;
    }

    return false;
  }

  void publishResetKfs()
  {
    std_msgs::msg::String reset_msg;
    reset_msg.data = "reset_KFS";
    vision_recognition_reset_pub_->publish(reset_msg);

    RCLCPP_WARN(
      get_logger(),
      "[VISION_LATERAL] Publish %s='%s'",
      vision_recognition_reset_topic_.c_str(),
      reset_msg.data.c_str());
  }


  bool isYawReached(double eyaw) const
  {
    return std::abs(eyaw) <= yaw_tolerance_;
  }

  bool isPositionReached(double dist) const
  {
    return dist <= current_pos_tolerance_;
  }

  bool isPositionReleaseReached(double dist) const
  {
    return dist <= std::max(current_pos_tolerance_, current_pos_release_tolerance_);
  }

  bool getBlockCenter(int block_id, double & x, double & y) const
  {
    return robot_common::r2::get_block_center(block_id, x, y);
  }

  bool getBlockHeight(int block_id, double & h) const
  {
    return robot_common::r2::get_block_height(block_id, h);
  }

	  double yawFromBlockToBlock(int from_block, int to_block) const
	  {
    double from_x = 0.0;
    double from_y = 0.0;
    double to_x = 0.0;
    double to_y = 0.0;

    if (!getBlockCenter(from_block, from_x, from_y)) {
      RCLCPP_WARN(get_logger(), "yawFromBlockToBlock failed: invalid from_block=%d", from_block);
      return r2_default_yaw_;
    }

    if (!getBlockCenter(to_block, to_x, to_y)) {
      RCLCPP_WARN(get_logger(), "yawFromBlockToBlock failed: invalid to_block=%d", to_block);
      return r2_default_yaw_;
    }

	    return std::atan2(to_y - from_y, to_x - from_x);
	  }

	  double targetYawForLeavingBlock(int from_block, int to_block) const
	  {
	    const double move_yaw = yawFromBlockToBlock(from_block, to_block);

	    double from_h = 0.0;
	    double to_h = 0.0;
	    if (getBlockHeight(from_block, from_h) && getBlockHeight(to_block, to_h)) {
	      const double dh = to_h - from_h;
	      if (dh < -0.05) {
	        return normalizeAngle(move_yaw + M_PI);
	      }
	    }

	    return move_yaw;
	  }

  bool isAdjacentBlock(int a, int b) const
  {
    double ax = 0.0;
    double ay = 0.0;
    double bx = 0.0;
    double by = 0.0;

    if (!getBlockCenter(a, ax, ay)) return false;
    if (!getBlockCenter(b, bx, by)) return false;

    const double d = std::hypot(ax - bx, ay - by);
    return std::abs(d - 1.20) <= 0.08;
  }

  int parsePickupBlockIdFromLine(const std::string & line) const
  {
    const std::string key = "pickup_block_";
    const auto pos = line.find(key);

    if (pos == std::string::npos) {
      return -1;
    }

    size_t start = pos + key.size();
    size_t end = start;

    while (end < line.size() && std::isdigit(static_cast<unsigned char>(line[end]))) {
      ++end;
    }

    if (end == start) {
      return -1;
    }

    try {
      return std::stoi(line.substr(start, end - start));
    } catch (...) {
      return -1;
    }
  }

  std::vector<int> parsePickupTargetsFromActionText(const std::string & text) const
  {
    std::vector<int> out;
    std::istringstream iss(text);
    std::string line;

    while (std::getline(iss, line)) {
      int id = parsePickupBlockIdFromLine(line);
      if (id >= 1 && id <= 12) {
        out.push_back(id);
      }
    }

    return out;
  }

  std::vector<PickupTask> parsePickupSequenceFromActionText(const std::string & text) const
{
  std::vector<PickupTask> out;
  std::set<std::tuple<int, int, int>> seen;

  /*
    只解析 pickup_sequence 段：

      pickup_sequence:
        - target=9 support=6 yaw=0
        - target=8 support=9 yaw=90

    不解析 actions 里的：

      - pickup target=9 support=6 yaw=0

    否则会重复插入 pickup waypoint。
  */

  static const std::regex pattern(
    R"(target\s*=\s*([0-9]+)\s+support\s*=\s*([0-9]+)\s+yaw\s*=\s*([-+]?[0-9]+))"
  );

  std::istringstream iss(text);
  std::string line;

  bool in_pickup_sequence = false;

  while (std::getline(iss, line)) {
    std::string trimmed = line;

    trimmed.erase(
      trimmed.begin(),
      std::find_if(
        trimmed.begin(),
        trimmed.end(),
        [](unsigned char ch) { return !std::isspace(ch); }
      )
    );

    trimmed.erase(
      std::find_if(
        trimmed.rbegin(),
        trimmed.rend(),
        [](unsigned char ch) { return !std::isspace(ch); }
      ).base(),
      trimmed.end()
    );

    if (trimmed == "pickup_sequence:") {
      in_pickup_sequence = true;
      continue;
    }

    if (in_pickup_sequence) {
      /*
        pickup_sequence 段结束条件。
        防止继续解析 actions 里的 pickup target=...
      */
      if (trimmed.rfind("path:", 0) == 0 ||
          trimmed.rfind("states:", 0) == 0 ||
          trimmed.rfind("actions:", 0) == 0 ||
          trimmed.rfind("cost=", 0) == 0 ||
          trimmed.rfind("targets:", 0) == 0) {
        break;
      }

      std::smatch match;
      if (std::regex_search(trimmed, match, pattern)) {
        PickupTask task;
        task.target = std::stoi(match[1].str());
        task.support = std::stoi(match[2].str());
        task.yaw_deg = std::stoi(match[3].str());
        task.inserted = false;

        if (task.target >= 1 && task.target <= 12 &&
            task.support >= 0 && task.support <= 12) {
          auto key = std::make_tuple(task.target, task.support, task.yaw_deg);

          if (seen.count(key) == 0) {
            seen.insert(key);
            out.push_back(task);
          }
        }
      }
    }
  }

  return out;
}


  bool shouldInsertPickupAfterBlock(int support_block, int pickup_block, double support_yaw) const
  {
    if (!enable_pickup_action_) {
      return false;
    }

    if (!isAdjacentBlock(support_block, pickup_block)) {
      return false;
    }

    const double yaw_to_target = yawFromBlockToBlock(support_block, pickup_block);
    const double dyaw = std::abs(angleDiff(yaw_to_target, support_yaw));

    return dyaw <= pickup_yaw_match_tolerance_;
  }


  bool insertPickupTasksAfterSupportBlock(
  int support_block,
  double support_x,
  double support_y,
  std::vector<PickupTask> & tasks)
{
  if (!enable_pickup_action_) {
    return false;
  }

  bool inserted_any = false;

  /*
    注意：
    去重应该只针对当前这次函数里真正准备插入的任务。
    不能在 task.support != support_block 之前就标记 inserted。
  */
  std::set<std::tuple<int, int, int>> local_inserted_keys;

  for (auto & task : tasks) {
    if (task.inserted) {
      continue;
    }

    /*
      必须先判断 support 是否匹配当前路径 block。
      例如 target=9 support=6，只能在走到 block=6 时插入。
      在 block=0/2/3 时不能提前标记为已插入。
    */
    if (task.support != support_block) {
      continue;
    }

    auto key = std::make_tuple(task.target, task.support, task.yaw_deg);

    if (local_inserted_keys.count(key) > 0) {
      RCLCPP_WARN(
        get_logger(),
        "Duplicated pickup task in same support block. Skip duplicate: target=%d support=%d yaw=%d",
        task.target,
        task.support,
        task.yaw_deg);

      task.inserted = true;
      continue;
    }

    local_inserted_keys.insert(key);

    if (!isAdjacentBlock(task.support, task.target)) {
      RCLCPP_WARN(
        get_logger(),
        "Pickup task invalid adjacency: target=%d support=%d. Skip.",
        task.target,
        task.support);

      task.inserted = true;
      continue;
    }

    Waypoint pickup_wp;
    pickup_wp.pose.x = support_x;
    pickup_wp.pose.y = support_y;
    pickup_wp.pose.yaw = normalizeAngle(degToRad(task.yaw_deg));
    pickup_wp.mode = MoveMode::FREE_2D;
    pickup_wp.source =
      "pickup_target_" + std::to_string(task.target) +
      "_from_support_" + std::to_string(task.support) +
      "_yaw_" + std::to_string(task.yaw_deg);
    pickup_wp.is_pickup = true;
    pickup_wp.pickup_block = task.target;
    pickup_wp.camera_look_backward = false;

    bool pickup_success = false;
    std::string pickup_message;

    enqueueOrStartWaypoint(pickup_wp, pickup_success, pickup_message);

    RCLCPP_WARN(
      get_logger(),
      "Insert pickup waypoint by pickup_sequence: target=%d support=%d "
      "x=%.3f y=%.3f yaw_deg=%d yaw_rad=%.3f result=%s",
      task.target,
      task.support,
      pickup_wp.pose.x,
      pickup_wp.pose.y,
      task.yaw_deg,
      pickup_wp.pose.yaw,
      pickup_message.c_str());

    task.inserted = true;
    inserted_any = true;
  }

  return inserted_any;
}


  void startPickupForward()
  {
    publishHardStop();

    arm_grab_done_received_ = false;
    pickup_step_off_sent_ = false;
    pickup_step_on_sent_ = false;

    pickup_waiting_vision_lateral_ = false;
    pickup_vision_lateral_done_ = false;
    pickup_reset_kfs_sent_ = false;
    pickup_vision_lateral_block_ = current_pickup_block_;


    pickup_state_start_time_ = now();
    pickup_forward_start_time_ = now();

    publishStepCommand(
      0x02,
      "before pickup block " + std::to_string(current_pickup_block_) +
      ", close infrared / enable stair mode");

    pickup_step_off_sent_ = true;

    RCLCPP_WARN(
      get_logger(),
      "Pickup block %d: send 0x02, start forward %.3f m at speed %.3f m/s",
      current_pickup_block_,
      pickup_forward_distance_,
      pickup_forward_speed_);

    setState(State::PICKUP_FORWARD);
  }


  void publishPickupLinearCommand(double vx)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = cmd_x_sign_ * vx;
    cmd.linear.y = 0.0;
    cmd.angular.z = 0.0;
    last_cmd_ = cmd;
    cmd_vel_pub_->publish(cmd);
  }

  double pickupMotionDuration(double distance, double speed) const
  {
    const double v = std::max(0.01, std::abs(speed));
    return std::abs(distance) / v;
  }

  bool isStairBlock(int block_id) const
  {
    return block_id == 13 || block_id == 14 || block_id == 15;
  }

  bool isEnteringStairBlock(int from_block, int to_block) const
  {
    return !isStairBlock(from_block) && isStairBlock(to_block);
  }

  bool updateAndGetXSlowZone()
  {
    if (!enable_x_slow_zone_) {
      x_slow_zone_active_ = false;
      return false;
    }

    const double x = current_pose_.x;
    const double x_min = std::min(x_slow_zone_min_, x_slow_zone_max_);
    const double x_max = std::max(x_slow_zone_min_, x_slow_zone_max_);
    const double h = std::abs(x_slow_zone_hysteresis_);

    if (!x_slow_zone_active_) {
      if (x >= x_min && x <= x_max) {
        x_slow_zone_active_ = true;
        RCLCPP_WARN(
          get_logger(),
          "Enter X slow zone: odom_x=%.3f, range=(%.3f, %.3f), scale=%.3f",
          x, x_min, x_max, x_slow_zone_scale_);
      }
    } else {
      if (x < x_min - h || x > x_max + h) {
        x_slow_zone_active_ = false;
        RCLCPP_WARN(
          get_logger(),
          "Exit X slow zone: odom_x=%.3f, range=(%.3f, %.3f), hysteresis=%.3f",
          x, x_min, x_max, h);
      }
    }

    return x_slow_zone_active_;
  }

  bool updateAndGetXBoostZone()
  {
    if (!enable_x_boost_zone_) {
      x_boost_zone_active_ = false;
      return false;
    }

    const double x = current_pose_.x;
    const double x_min = std::min(x_boost_zone_min_, x_boost_zone_max_);
    const double x_max = std::max(x_boost_zone_min_, x_boost_zone_max_);
    const double h = std::abs(x_boost_zone_hysteresis_);

    if (!x_boost_zone_active_) {
      if (x >= x_min && x <= x_max) {
        x_boost_zone_active_ = true;
        RCLCPP_WARN(
          get_logger(),
          "Enter X boost zone: odom_x=%.3f, range=(%.3f, %.3f), scale=%.3f",
          x, x_min, x_max, x_boost_zone_scale_);
      }
    } else {
      if (x < x_min - h || x > x_max + h) {
        x_boost_zone_active_ = false;
        RCLCPP_WARN(
          get_logger(),
          "Exit X boost zone: odom_x=%.3f, range=(%.3f, %.3f), hysteresis=%.3f",
          x, x_min, x_max, h);
      }
    }

    return x_boost_zone_active_;
  }

void applyXZoneScaleToFinalCmd(
  geometry_msgs::msg::Twist & cmd,
  bool in_x_slow_zone,
  bool in_x_boost_zone) const
{
  if (in_x_slow_zone) {
    const double s = std::clamp(std::abs(x_slow_zone_scale_), 0.001, 1.0);

    /*
      x 慢速区只限制前进方向 linear.x。
      不再缩放 linear.y。

      原来同时缩放 y：
        cmd.linear.y *= s;

      会导致：
        - 普通路径横向收敛很慢
        - 视觉横向纠偏在慢速区内几乎不动
    */
    cmd.linear.x *= s;

    if (x_slow_scale_angular_) {
      cmd.angular.z *= s;
    }

    return;
  }

  if (in_x_boost_zone) {
    const double s = std::clamp(std::abs(x_boost_zone_scale_), 1.0, 5.0);

    /*
      x boost 区也只 boost 前进方向。
      避免横向被放大后甩动。
    */
    cmd.linear.x *= s;

    if (x_boost_scale_angular_) {
      cmd.angular.z *= s;
    }

    return;
  }
}


	  void resetControllerMemory()
	  {
	    last_err_valid_ = false;
	    last_forward_err_ = 0.0;
	    last_lateral_err_ = 0.0;
    best_dist_ = 1e9;
    last_progress_time_ = now();
	    last_control_time_ = now();
	  }

	  double goalYawForwardError() const
	  {
	    const double ex = goal_pose_.x - current_pose_.x;
	    const double ey = goal_pose_.y - current_pose_.y;
	    const double gy = std::cos(goal_pose_.yaw);
	    const double sy = std::sin(goal_pose_.yaw);
	    return control_forward_sign_ * (gy * ex + sy * ey);
	  }

		  double goalYawLateralError() const
		  {
	    const double ex = goal_pose_.x - current_pose_.x;
	    const double ey = goal_pose_.y - current_pose_.y;
	    const double gy = std::cos(goal_pose_.yaw);
	    const double sy = std::sin(goal_pose_.yaw);
		    return -sy * ex + gy * ey;
		  }

		  double selectMovePhaseTargetYaw() const
		  {
		    if (current_use_goal_yaw_during_move_ || !keep_yaw_during_move_) {
		      return goal_pose_.yaw;
		    }

		    if (use_current_yaw_as_move_target_) {
		      return current_pose_.yaw;
		    }

		    return move_phase_target_yaw_;
		  }
	
		  void setState(State new_state)
		  {
    if (state_ == new_state) return;

    RCLCPP_WARN(get_logger(), "State: %s -> %s", stateToString(state_), stateToString(new_state));

    state_ = new_state;
    state_enter_time_ = now();
    pos_reached_since_ = now();
    yaw_reached_since_ = now();

	    if (state_ == State::MOVE_TO_POS) {
	      resetControllerMemory();
	
	      move_phase_target_yaw_ = selectMovePhaseTargetYaw();
	
	      RCLCPP_WARN(
	        get_logger(),
	        "MOVE_TO_POS yaw target selected: move_phase_target_yaw=%.3f "
	        "(current_yaw=%.3f, goal_yaw=%.3f, keep_yaw=%d, use_current=%d, use_goal_during_move=%d)",
	        move_phase_target_yaw_,
	        current_pose_.yaw,
	        goal_pose_.yaw,
	        keep_yaw_during_move_ ? 1 : 0,
	        use_current_yaw_as_move_target_ ? 1 : 0,
	        current_use_goal_yaw_during_move_ ? 1 : 0);
	    }

    publishStatus();
  }

	  void startWaypoint(const Waypoint & wp)
	  {
    goal_pose_ = wp.pose;
    move_mode_ = wp.mode;
    active_goal_ = true;
    active_goal_start_time_ = now();

    setCameraLookDirection(
      wp.camera_look_backward,
      "start waypoint source=" + wp.source);

    current_waypoint_is_pickup_ = wp.is_pickup;
    current_pickup_block_ = wp.pickup_block;
	    current_path_block_ = wp.path_block;
	    current_waypoint_source_ = wp.source;
	    current_force_pre_align_yaw_ = wp.force_pre_align_yaw;
		    current_use_goal_yaw_during_move_ = wp.use_goal_yaw_during_move;
		    current_r2_lane_drive_ = false;
		    current_r2_lane_goal_accepted_ = false;
		    straight_lateral_lock_active_ = false;
		    current_position_accepted_ = false;
	    current_allow_block0_can_ = wp.allow_block0_can;
    if (wp.use_custom_pos_tolerance) {
      current_pos_tolerance_ = std::max(0.001, wp.custom_pos_tolerance);
      current_pos_release_tolerance_ =
        std::max(current_pos_tolerance_, wp.custom_pos_release_tolerance);
      current_lateral_stop_tolerance_ =
        std::min(std::abs(lateral_stop_tolerance_), current_pos_tolerance_ * 0.5);
    } else {
      current_pos_tolerance_ = pos_tolerance_;
      current_pos_release_tolerance_ = pos_release_tolerance_;
      current_lateral_stop_tolerance_ = std::abs(lateral_stop_tolerance_);
    }

    const double ex = goal_pose_.x - current_pose_.x;
    const double ey = goal_pose_.y - current_pose_.y;
    const double dist = std::hypot(ex, ey);
    const double eyaw = angleDiff(goal_pose_.yaw, current_pose_.yaw);

	    yaw_only_mode_ = dist <= current_pos_tolerance_;
    pre_align_mode_ = false;

			    move_phase_target_yaw_ = current_pose_.yaw;
			    move_phase_target_yaw_ = selectMovePhaseTargetYaw();

	    const double cy0 = std::cos(move_phase_target_yaw_);
	    const double sy0 = std::sin(move_phase_target_yaw_);
	    const double initial_forward_err =
	      control_forward_sign_ * (cy0 * ex + sy0 * ey);
	    const double initial_lateral_err =
	      control_lateral_sign_ * (-sy0 * ex + cy0 * ey);
	    const bool initial_lateral_small =
	      std::abs(initial_lateral_err) <= straight_lateral_lock_goal_tolerance_;
	    const bool initial_lateral_ratio_small =
	      std::abs(initial_lateral_err) <=
	      std::abs(initial_forward_err) * straight_lateral_lock_max_lateral_ratio_;

	    straight_lateral_lock_active_ =
	      enable_straight_lateral_lock_ &&
	      std::abs(initial_forward_err) >= straight_lateral_lock_min_forward_ &&
	      (initial_lateral_small || initial_lateral_ratio_small);

		    if (current_r2_lane_drive_) {
	      const double lane_forward_err = goalYawForwardError();
	      current_r2_lane_drive_sign_ = lane_forward_err < 0.0 ? -1.0 : 1.0;

	      RCLCPP_WARN(
	        get_logger(),
	        "R2 lane-drive waypoint enabled: block=%d lane_forward_err=%.3f drive_sign=%.0f "
	        "(linear_y will be held at 0; passed target line will be accepted)",
	        current_path_block_,
	        lane_forward_err,
	        current_r2_lane_drive_sign_);
		    } else {
		      current_r2_lane_drive_sign_ = 1.0;
		    }

	    if (straight_lateral_lock_active_) {
	      RCLCPP_WARN(
	        get_logger(),
	        "Straight lateral lock enabled: initial_forward_err=%.3f "
	        "initial_lateral_err=%.3f goal_tol=%.3f ratio_tol=%.3f finish_tol=%.3f",
	        initial_forward_err,
	        initial_lateral_err,
	        straight_lateral_lock_goal_tolerance_,
	        straight_lateral_lock_max_lateral_ratio_,
	        straight_lateral_finish_tolerance_);
	    }

	    resetControllerMemory();

    RCLCPP_WARN(
      get_logger(),
      "Start waypoint: goal=(%.3f, %.3f, %.3f rad), mode=%s, source=%s, queue_remain=%zu, "
      "initial_dist=%.3f, initial_eyaw=%.3f, yaw_only_mode=%d, force_pre_align=%d, "
		      "use_goal_yaw_during_move=%d, r2_lane_drive=%d, straight_lock=%d, "
		      "is_pickup=%d, pickup_block=%d, pos_tol=%.4f, release_tol=%.4f, lateral_deadband=%.4f",
      goal_pose_.x, goal_pose_.y, goal_pose_.yaw,
      moveModeToString(move_mode_),
      wp.source.c_str(),
      goal_queue_.size(),
      dist,
      eyaw,
      yaw_only_mode_,
	      current_force_pre_align_yaw_ ? 1 : 0,
		      current_use_goal_yaw_during_move_ ? 1 : 0,
		      current_r2_lane_drive_ ? 1 : 0,
		      straight_lateral_lock_active_ ? 1 : 0,
		      current_waypoint_is_pickup_ ? 1 : 0,
		      current_pickup_block_,
      current_pos_tolerance_,
      current_pos_release_tolerance_,
      current_lateral_stop_tolerance_);

	    if (yaw_only_mode_) {
	      setState(State::ALIGN_YAW);
	      return;
	    }

	    const double pre_align_threshold = std::abs(pre_align_yaw_threshold_);
	    if (current_force_pre_align_yaw_ ||
	        (pre_align_threshold > 1e-6 && std::abs(eyaw) >= pre_align_threshold)) {
	      pre_align_mode_ = true;
	      RCLCPP_WARN(
	        get_logger(),
	        "Pre-align before MOVE_TO_POS: eyaw=%.3f threshold=%.3f force=%d",
	        eyaw,
	        pre_align_threshold,
	        current_force_pre_align_yaw_ ? 1 : 0);
	      setState(State::ALIGN_YAW);
	      return;
	    }
	
	    setState(State::MOVE_TO_POS);
	  }

  bool startNextWaypointFromQueue()
  {
	    if (goal_queue_.empty()) {
	      active_goal_ = false;
	      current_waypoint_is_pickup_ = false;
	      current_pickup_block_ = -1;
		      current_path_block_ = -1;
		      current_waypoint_source_ = "unknown";
		      current_position_accepted_ = false;
			      current_allow_block0_can_ = true;
			      current_force_pre_align_yaw_ = false;
		      current_use_goal_yaw_during_move_ = false;
		      straight_lateral_lock_active_ = false;
	      RCLCPP_INFO(get_logger(), "All waypoints completed.");
	      setState(State::IDLE);
	      return false;
    }

    Waypoint wp = goal_queue_.front();
    goal_queue_.pop_front();
    startWaypoint(wp);
    return true;
  }

  void enqueueOrStartWaypoint(const Waypoint & wp, bool & success, std::string & message)
  {
    if (!active_goal_ && state_ == State::IDLE) {
      startWaypoint(wp);
      success = true;
      message = "Goal accepted and started immediately.";
    } else {
      goal_queue_.push_back(wp);
      success = true;
      message = "Goal accepted and queued.";

      RCLCPP_INFO(
        get_logger(),
	        "Waypoint queued: goal=(%.3f, %.3f, %.3f rad), mode=%s, source=%s, "
	        "force_pre_align=%d, use_goal_yaw_during_move=%d, pickup=%d, pickup_block=%d, queue_size=%zu",
	        wp.pose.x, wp.pose.y, wp.pose.yaw,
	        moveModeToString(wp.mode),
	        wp.source.c_str(),
	        wp.force_pre_align_yaw ? 1 : 0,
	        wp.use_goal_yaw_during_move ? 1 : 0,
	        wp.is_pickup ? 1 : 0,
	        wp.pickup_block,
	        goal_queue_.size());
    }
  }

  void r2PathCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    if (msg->data.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty R2 planned path.");
      return;
    }

    latest_r2_path_ = *msg;
    has_latest_r2_path_ = true;

    if (!odom_received_) {
      pending_r2_path_waiting_for_odom_ = true;
      RCLCPP_WARN(
        get_logger(),
        "Received R2 path before odometry. Cache it and build queue after odometry is ready.");
      return;
    }

    pending_r2_path_waiting_for_odom_ = false;

    if (enable_pickup_action_ && !has_r2_action_sequence_) {
      RCLCPP_WARN(
        get_logger(),
        "Received R2 path before action sequence. Cache path and wait for action sequence before building queue.");
      return;
    }

    RCLCPP_INFO(get_logger(), "Received R2 planned path, size=%zu", msg->data.size());

    std::ostringstream path_oss;
    path_oss << "R2 path blocks: ";
    for (const auto & b : msg->data) {
      path_oss << b << " ";
    }
    RCLCPP_INFO(get_logger(), "%s", path_oss.str().c_str());

    std::vector<PickupTask> pending_tasks = r2_pickup_tasks_;

    if (enable_pickup_action_) {
      if (pending_tasks.empty()) {
        RCLCPP_WARN(
          get_logger(),
          "R2 path received but no pickup_sequence parsed yet. "
          "No pickup waypoint will be inserted until action sequence arrives.");
      } else {
        RCLCPP_WARN(get_logger(), "Use parsed pickup_sequence to insert pickup waypoints.");
        for (const auto & t : pending_tasks) {
          RCLCPP_WARN(
            get_logger(),
            "  pending pickup task: target=%d support=%d yaw=%d",
            t.target,
            t.support,
            t.yaw_deg);
        }
      }
    }

    if (active_goal_ && !cancel_active_goal_when_new_r2_path_) {
      if (clear_queue_when_new_r2_path_) {
        goal_queue_.clear();
      }

      deferred_r2_path_ = *msg;
      has_deferred_r2_path_ = true;

      RCLCPP_WARN(
        get_logger(),
        "New R2 path arrived while active goal is running. "
        "Defer rebuilding queue until current waypoint completes. Old queued waypoints cleared=%d.",
        clear_queue_when_new_r2_path_ ? 1 : 0);
      return;
    }

    if (clear_queue_when_new_r2_path_) {
      goal_queue_.clear();
      RCLCPP_WARN(get_logger(), "New R2 path received. Old queued waypoints cleared.");
    }

    if (cancel_active_goal_when_new_r2_path_) {
      publishHardStop();
	      active_goal_ = false;
	      current_waypoint_is_pickup_ = false;
	      current_pickup_block_ = -1;
		      current_path_block_ = -1;
		      current_waypoint_source_ = "unknown";
		      current_position_accepted_ = false;
			      current_allow_block0_can_ = true;
			      current_force_pre_align_yaw_ = false;
		      current_use_goal_yaw_during_move_ = false;
		      straight_lateral_lock_active_ = false;
	      setState(State::IDLE);

      RCLCPP_WARN(get_logger(), "New R2 path received. Active goal canceled and controller reset to IDLE.");
    }

    for (size_t i = 0; i < msg->data.size(); ++i) {
      const int block_id = msg->data[i];

      double x = 0.0;
      double y = 0.0;

      if (!getBlockCenter(block_id, x, y)) {
        RCLCPP_WARN(get_logger(), "Invalid block id in R2 path: %d. Skip this block.", block_id);
        continue;
      }

      bool first_block_is_current = false;

	if (i == 0) {
	  const double first_dist = std::hypot(x - current_pose_.x, y - current_pose_.y);

	  if (first_dist < 0.20) {
	    first_block_is_current = true;
	    RCLCPP_WARN(
	      get_logger(),
	      "Robot is already near first R2 path block %d, dist=%.3f. "
	      "Keep first waypoint to process target yaw. "
	      "Do not send block0 CAN 0x01 for this near-current first waypoint.",
	      block_id,
	      first_dist);
	  }
	}


      if (i > 0) {
        const int prev_block = msg->data[i - 1];

        if (isEnteringStairBlock(prev_block, block_id)) {
          double prev_x = 0.0;
          double prev_y = 0.0;

          if (getBlockCenter(prev_block, prev_x, prev_y)) {
            Waypoint turn_wp;
            turn_wp.pose.x = prev_x;
            turn_wp.pose.y = prev_y;
            turn_wp.pose.yaw = r2_back_to_stair_yaw_;
            turn_wp.mode = MoveMode::FREE_2D;
            turn_wp.source =
              "turn_back_to_stair_before_" + std::to_string(prev_block) +
              "_to_" + std::to_string(block_id);
            turn_wp.force_pre_align_yaw = false;
            turn_wp.use_goal_yaw_during_move = false;
            turn_wp.camera_look_backward = true;

            bool turn_success = false;
            std::string turn_message;

            enqueueOrStartWaypoint(turn_wp, turn_success, turn_message);

            RCLCPP_WARN(
              get_logger(),
              "Down-stair detected: %d -> %d. Insert turn-back waypoint at block %d center "
              "(x=%.3f, y=%.3f, yaw=%.3f). result=%s",
              prev_block,
              block_id,
              prev_block,
              prev_x,
              prev_y,
              r2_back_to_stair_yaw_,
              turn_message.c_str());
          }
        }
      }

      Waypoint wp;
      wp.pose.x = x;
      wp.pose.y = y;
      wp.pose.yaw = r2_default_yaw_;
      wp.mode = MoveMode::FREE_2D;
      wp.source = "r2_path_block_" + std::to_string(block_id);
	      wp.path_block = block_id;
	      wp.force_pre_align_yaw = false;
	      wp.use_goal_yaw_during_move = false;
	      wp.is_pickup = false;
	      wp.pickup_block = -1;
	      wp.camera_look_backward = false;
	      wp.allow_block0_can = !(first_block_is_current && block_id == 0);

	      if (i > 0) {
	        const int prev_block = msg->data[i - 1];

	        double prev_h = 0.0;
	        double curr_h = 0.0;

	        if (getBlockHeight(prev_block, prev_h) && getBlockHeight(block_id, curr_h)) {
	          const double dh = curr_h - prev_h;

	          if (dh < -0.05) {
	            wp.camera_look_backward = true;

	            RCLCPP_WARN(
	              get_logger(),
	              "Down-stair/backward move detected while moving %d -> %d, height %.2f -> %.2f, "
	              "camera_look_backward=1, gimbal_can_id=0x77, gimbal_cmd=0x03",
	              prev_block,
	              block_id,
	              prev_h,
	              curr_h);
	          } else if (dh > 0.05) {
	            RCLCPP_WARN(
	              get_logger(),
	              "Up-stair detected while moving %d -> %d, height %.2f -> %.2f",
	              prev_block,
	              block_id,
	              prev_h,
	              curr_h);
	          }
	        }
	      }

	      bool target_yaw_set_by_pickup = false;
	      if (enable_pickup_action_) {
	        for (const auto & task : pending_tasks) {
	          if (!task.inserted && task.support == block_id) {
	            wp.pose.yaw = normalizeAngle(degToRad(task.yaw_deg));
	            target_yaw_set_by_pickup = true;

	            RCLCPP_WARN(
	              get_logger(),
	              "Set target yaw after arrival at support block %d by pickup_sequence: "
	              "target=%d yaw_deg=%d yaw=%.3f",
	              block_id,
	              task.target,
	              task.yaw_deg,
	              wp.pose.yaw);
	            break;
	          }
	        }
	      }

	      if (!target_yaw_set_by_pickup && i + 1 < msg->data.size()) {
	        const int next_block = msg->data[i + 1];
	        wp.pose.yaw = targetYawForLeavingBlock(block_id, next_block);

	        RCLCPP_WARN(
	          get_logger(),
	          "Set target yaw after arrival by next path edge: %d -> %d, yaw=%.3f",
	          block_id,
	          next_block,
	          wp.pose.yaw);
	      }

      bool success = false;
      std::string message;

      enqueueOrStartWaypoint(wp, success, message);

      RCLCPP_INFO(
        get_logger(),
	        "R2 waypoint added: block=%d, x=%.3f, y=%.3f, yaw=%.3f, "
	        "force_pre_align=0, use_goal_yaw_during_move=0, result=%s",
        block_id,
        x,
        y,
        wp.pose.yaw,
        message.c_str());

      insertPickupTasksAfterSupportBlock(block_id, x, y, pending_tasks);
    }

    if (enable_pickup_action_) {
      bool has_uninserted = false;
      std::ostringstream oss;
      oss << "Warning: some pickup_sequence tasks were not inserted: ";

      for (const auto & task : pending_tasks) {
        if (!task.inserted) {
          has_uninserted = true;
          oss << "[target=" << task.target
              << ", support=" << task.support
              << ", yaw=" << task.yaw_deg << "] ";
        }
      }

      if (has_uninserted) {
        RCLCPP_WARN(get_logger(), "%s", oss.str().c_str());
      }
    }
  }
  void r2ActionCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(
      get_logger(),
      "Received R2 action sequence:\n%s",
      msg->data.c_str());

    /*
      新格式：
        pickup_sequence:
          - target=9 support=6 yaw=0
          - target=8 support=9 yaw=90

      旧格式 pickup_block_x 只保留兼容日志，不再用于插入 pickup。
    */
    r2_pickup_tasks_ = parsePickupSequenceFromActionText(msg->data);
    has_pickup_sequence_ = !r2_pickup_tasks_.empty();

    r2_pickup_targets_ = parsePickupTargetsFromActionText(msg->data);
    has_r2_action_sequence_ = true;

    if (has_pickup_sequence_) {
      RCLCPP_WARN(get_logger(), "Parsed pickup_sequence tasks:");
      for (const auto & p : r2_pickup_tasks_) {
        RCLCPP_WARN(
          get_logger(),
          "  target=%d support=%d yaw=%d",
          p.target,
          p.support,
          p.yaw_deg);
      }
    } else {
      std::ostringstream oss;
      oss << "No pickup_sequence parsed. Legacy pickup targets parsed: ";
      for (int id : r2_pickup_targets_) {
        oss << id << " ";
      }

      RCLCPP_WARN(get_logger(), "%s", oss.str().c_str());
      RCLCPP_WARN(
        get_logger(),
        "Optimized controller requires pickup_sequence. "
        "Planner should output: target=.. support=.. yaw=..");
    }

    /*
      path 和 action 可能有先后顺序：
      - 初始启动时，如果 path 已经到了、action 后到，且控制器空闲，才用 cached path 建队列。
      - 重规划运行中 action 更新时，不用旧 cached path 自动重建；等待新的 path callback。
      - 如果 action 先到，后续 path callback 会直接使用 r2_pickup_tasks_。
    */
    if (enable_pickup_action_ && has_latest_r2_path_ && has_r2_action_sequence_) {
      if (rebuilding_r2_queue_) {
        return;
      }

      const bool controller_idle =
        !active_goal_ && goal_queue_.empty() && state_ == State::IDLE;

      if (!controller_idle) {
        RCLCPP_WARN(
          get_logger(),
          "Action sequence arrived while controller is active. "
          "Do not rebuild queue from cached path; wait for next R2 path.");
        return;
      }

      rebuilding_r2_queue_ = true;

      RCLCPP_WARN(
        get_logger(),
        "Action sequence arrived. Build R2 queue from cached path.");

      if (cancel_active_goal_when_new_r2_path_) {
        publishHardStop();

        goal_queue_.clear();
	        active_goal_ = false;
	        current_waypoint_is_pickup_ = false;
	        current_pickup_block_ = -1;
		        current_path_block_ = -1;
		        current_waypoint_source_ = "unknown";
		        current_position_accepted_ = false;
			        current_allow_block0_can_ = true;
			        current_force_pre_align_yaw_ = false;
		        current_use_goal_yaw_during_move_ = false;
		        straight_lateral_lock_active_ = false;
	        arm_grab_done_received_ = false;
	        pickup_step_off_sent_ = false;
	        pickup_step_on_sent_ = false;

        setState(State::IDLE);
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Controller is idle. Build queued R2 waypoints from cached path.");
      }

      auto path_msg = std::make_shared<std_msgs::msg::Int32MultiArray>(latest_r2_path_);
      r2PathCallback(path_msg);

      rebuilding_r2_queue_ = false;
    }
  }

  void handleAbsoluteGoal(
    const std::shared_ptr<lite_task_controller::srv::SetAbsoluteGoal::Request> req,
    std::shared_ptr<lite_task_controller::srv::SetAbsoluteGoal::Response> res)
  {
    if (!odom_received_) {
      res->success = false;
      res->message = "No odometry yet.";
      return;
    }

    if (!std::isfinite(req->x) ||
        !std::isfinite(req->y) ||
        !std::isfinite(req->yaw)) {
      res->success = false;
      res->message = "Goal contains non-finite x/y/yaw.";
      return;
    }

    Waypoint wp;
    wp.pose.x = req->x;
    wp.pose.y = req->y;
    wp.pose.yaw = req->yaw;
    wp.mode = MoveMode::FREE_2D;
    wp.source = "absolute";
    wp.is_pickup = false;
    wp.pickup_block = -1;
    if (req->use_custom_tolerance) {
      if (!std::isfinite(req->pos_tolerance) ||
          !std::isfinite(req->pos_release_tolerance) ||
          req->pos_tolerance <= 0.0 ||
          req->pos_release_tolerance <= 0.0) {
        res->success = false;
        res->message = "Custom tolerance must be positive finite values.";
        return;
      }

      wp.use_custom_pos_tolerance = true;
      wp.custom_pos_tolerance = req->pos_tolerance;
      wp.custom_pos_release_tolerance = req->pos_release_tolerance;
    }

    RCLCPP_INFO(
      get_logger(),
      "Receive absolute goal: current=(%.3f, %.3f, %.3f rad) -> goal=(%.3f, %.3f, %.3f rad), mode=%s",
      current_pose_.x, current_pose_.y, current_pose_.yaw,
      wp.pose.x, wp.pose.y, wp.pose.yaw,
      moveModeToString(wp.mode));

    enqueueOrStartWaypoint(wp, res->success, res->message);
  }

  void handleRelativeGoal(
    const std::shared_ptr<lite_task_controller::srv::SetRelativeGoal::Request> req,
    std::shared_ptr<lite_task_controller::srv::SetRelativeGoal::Response> res)
  {
    if (!odom_received_) {
      res->success = false;
      res->message = "No odometry yet.";
      return;
    }

    if (!std::isfinite(req->dx) ||
        !std::isfinite(req->dy) ||
        !std::isfinite(req->dyaw)) {
      res->success = false;
      res->message = "Relative goal contains non-finite dx/dy/dyaw.";
      return;
    }

    const double cy = std::cos(current_pose_.yaw);
    const double sy = std::sin(current_pose_.yaw);

    Waypoint wp;
    wp.pose.x = current_pose_.x + cy * req->dx - sy * req->dy;
    wp.pose.y = current_pose_.y + sy * req->dx + cy * req->dy;
    wp.pose.yaw = normalizeAngle(current_pose_.yaw + req->dyaw);
    wp.mode = MoveMode::FREE_2D;
    wp.source = "relative";
    wp.is_pickup = false;
    wp.pickup_block = -1;

    RCLCPP_INFO(
      get_logger(),
      "Receive relative goal: current=(%.3f, %.3f, %.3f rad), cmd=(dx=%.3f, dy=%.3f, dyaw=%.3f rad) "
      "-> abs_goal=(%.3f, %.3f, %.3f rad), mode=%s",
      current_pose_.x, current_pose_.y, current_pose_.yaw,
      req->dx, req->dy, req->dyaw,
      wp.pose.x, wp.pose.y, wp.pose.yaw,
      moveModeToString(wp.mode));

    enqueueOrStartWaypoint(wp, res->success, res->message);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const bool first_odom = !odom_received_;
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const double yaw = quatToYaw(msg->pose.pose.orientation);

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Ignore invalid odometry pose: x=%.3f y=%.3f yaw=%.3f",
        x,
        y,
        yaw);
      return;
    }

    current_pose_.x = x;
    current_pose_.y = y;
    current_pose_.yaw = yaw;
    odom_received_ = true;

    if (first_odom && pending_r2_path_waiting_for_odom_ && has_latest_r2_path_) {
      if (!enable_pickup_action_ || has_r2_action_sequence_) {
        pending_r2_path_waiting_for_odom_ = false;
        auto path_msg = std::make_shared<std_msgs::msg::Int32MultiArray>(latest_r2_path_);

        RCLCPP_WARN(
          get_logger(),
          "Odometry is ready. Build R2 queue from cached path.");

        r2PathCallback(path_msg);
      }
    }
  }

	  void acceptPositionAndGoAlign(double dist, bool strict)
	  {
	    RCLCPP_WARN(
	      get_logger(),
	      "Position accepted: source=%s block=%d pickup=%d dist=%.4f "
	      "pos_tol=%.4f release_tol=%.4f strict=%d "
	      "cur=(%.3f, %.3f, %.3f) goal=(%.3f, %.3f, %.3f)",
	      current_waypoint_source_.c_str(),
	      current_path_block_,
	      current_waypoint_is_pickup_ ? 1 : 0,
	      dist,
      current_pos_tolerance_,
      current_pos_release_tolerance_,
      strict ? 1 : 0,
	      current_pose_.x,
	      current_pose_.y,
	      current_pose_.yaw,
	      goal_pose_.x,
	      goal_pose_.y,
	      goal_pose_.yaw);

	    current_position_accepted_ = true;

    setState(State::ALIGN_YAW);
  }

  void controlLoop()
  {
    if (!odom_received_) return;

    /*
      R1_KFS 等待最高优先级：
      - 不清空队列
      - 不取消 active goal
      - 持续发 0 速度
      - 等 /r2_r1_kfs_cleared 后恢复当前状态机
    */
    if (waiting_for_r1_kfs_) {
      publishHardStop();

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Waiting for R1_KFS block=%d cleared. Robot stopped. Queue kept=%zu, active_goal=%d, state=%s",
        waiting_r1_block_,
        goal_queue_.size(),
        active_goal_ ? 1 : 0,
        stateToString(state_));

      return;
    }

    const auto now_t = now();

    const bool in_pickup_state =
      state_ == State::PICKUP_FORWARD ||
      state_ == State::PICKUP_WAIT_ARM_DONE ||
      state_ == State::PICKUP_REVERSE;

    if (active_goal_ && enable_goal_timeout_ && !in_pickup_state) {
      const double elapsed = (now_t - active_goal_start_time_).seconds();

      if (elapsed > goal_timeout_sec_) {
        RCLCPP_WARN(get_logger(), "Goal timeout: %.2f s > %.2f s", elapsed, goal_timeout_sec_);

        publishHardStop();
        goal_queue_.clear();
	        active_goal_ = false;
	        current_waypoint_is_pickup_ = false;
	        current_pickup_block_ = -1;
		        current_path_block_ = -1;
		        current_waypoint_source_ = "unknown";
		        current_position_accepted_ = false;
			        current_allow_block0_can_ = true;
			        current_force_pre_align_yaw_ = false;
		        current_use_goal_yaw_during_move_ = false;
		        straight_lateral_lock_active_ = false;
	        setState(State::IDLE);
	        return;
      }
    }

    const double ex = goal_pose_.x - current_pose_.x;
    const double ey = goal_pose_.y - current_pose_.y;
    const double dist = std::hypot(ex, ey);
    const double eyaw = angleDiff(goal_pose_.yaw, current_pose_.yaw);

    const double cy = std::cos(current_pose_.yaw);
    const double sy = std::sin(current_pose_.yaw);

    const double e_body_x = cy * ex + sy * ey;
    const double e_body_y = -sy * ex + cy * ey;

    const double control_forward_err = control_forward_sign_ * e_body_x;
    const double control_lateral_err = control_lateral_sign_ * e_body_y;

    const double display_forward_err = display_forward_sign_ * e_body_x;
    const double display_lateral_err = display_lateral_sign_ * e_body_y;

    const bool in_x_slow_zone = updateAndGetXSlowZone();
    const bool raw_x_boost_zone = updateAndGetXBoostZone();
    const bool in_x_boost_zone = raw_x_boost_zone && !in_x_slow_zone;

    switch (state_) {
      case State::IDLE:
      {
        publishStop();
        break;
      }

      case State::MOVE_TO_POS:
      {
        publishTargetYaw(move_phase_target_yaw_);

        double dt = (now_t - last_control_time_).seconds();
        last_control_time_ = now_t;

        if (dt <= 1e-4 || dt > 0.5) {
          dt = 1.0 / std::max(1.0, control_frequency_);
        }

	        const double lane_forward_err = goalYawForwardError();
	        const double lane_lateral_err = goalYawLateralError();
	        const double lane_lateral_tol =
          std::max(current_pos_tolerance_, current_pos_release_tolerance_);
	        const double motion_error =
	          current_r2_lane_drive_ ? std::abs(lane_forward_err) : dist;

	        if (motion_error + progress_epsilon_ < best_dist_) {
	          best_dist_ = motion_error;
	          last_progress_time_ = now_t;
	        }

	        const bool straight_lock_reached =
	          straight_lateral_lock_active_ &&
	          std::abs(control_forward_err) <= current_pos_tolerance_ &&
	          std::abs(control_lateral_err) <= straight_lateral_finish_tolerance_;
	        const bool straight_lock_release_reached =
	          straight_lateral_lock_active_ &&
	          std::abs(control_forward_err) <=
            std::max(current_pos_tolerance_, current_pos_release_tolerance_) &&
	          std::abs(control_lateral_err) <= straight_lateral_finish_tolerance_;
	        const bool pos_reached = isPositionReached(dist) || straight_lock_reached;
	        const bool pos_release_reached =
	          isPositionReleaseReached(dist) || straight_lock_release_reached;
	        const bool lane_forward_reached =
	          current_r2_lane_drive_ &&
	          std::abs(lane_forward_err) <=
            std::max(current_pos_tolerance_, current_pos_release_tolerance_);
	        const bool lane_target_passed =
	          current_r2_lane_drive_ &&
	          (current_r2_lane_drive_sign_ * lane_forward_err <= 0.0);
	        const bool lane_lateral_reached =
	          current_r2_lane_drive_ &&
	          std::abs(lane_lateral_err) <= lane_lateral_tol;

	        if (current_r2_lane_drive_ &&
	            lane_target_passed &&
	            !lane_lateral_reached &&
	            !pos_release_reached) {
	          publishHardStop();

	          RCLCPP_WARN_THROTTLE(
	            get_logger(),
	            *get_clock(),
	            500,
	            "R2 lane-drive reached/passed target line but block=%d is not accepted yet: "
	            "lane_forward_err=%.3f lane_lateral_err=%.3f dist=%.3f. "
	            "Hold yaw and stop; no pre-arrival rotation is allowed.",
	            current_path_block_,
	            lane_forward_err,
	            lane_lateral_err,
	            dist);

	          break;
	        }

	        if (pos_reached ||
	            ((lane_forward_reached || lane_target_passed) && lane_lateral_reached)) {
	          if (current_r2_lane_drive_) {
	            current_r2_lane_goal_accepted_ = true;

	            RCLCPP_WARN(
	              get_logger(),
	              "R2 lane-drive position accepted: block=%d lane_forward_err=%.3f "
	              "lane_lateral_err=%.3f drive_sign=%.0f forward_reached=%d passed=%d dist=%.3f",
	              current_path_block_,
	              lane_forward_err,
	              lane_lateral_err,
	              current_r2_lane_drive_sign_,
	              lane_forward_reached ? 1 : 0,
	              lane_target_passed ? 1 : 0,
	              dist);
	          }

	          publishHardStop();

	          if ((now_t - pos_reached_since_).seconds() >= pos_stable_time_) {
	            acceptPositionAndGoAlign(
	              current_r2_lane_drive_ ? std::abs(lane_forward_err) : dist,
	              !lane_target_passed);
	          }

	          break;
	        }

		        const auto axisSlowScale = [this](double abs_error) {
		          if (abs_error >= slow_down_radius_) {
		            return 1.0;
		          }

		          const double r = abs_error / std::max(1e-6, slow_down_radius_);
		          return std::clamp(r, std::abs(min_slow_scale_), 1.0);
		        };

	        const double limited_forward_err = std::clamp(
	          current_r2_lane_drive_ ? lane_forward_err : control_forward_err,
	          -std::abs(max_forward_error_for_control_),
	          std::abs(max_forward_error_for_control_));

	        double lateral_err_for_control = control_lateral_err;

	        if (current_r2_lane_drive_) {
	          lateral_err_for_control = 0.0;
	        } else if (straight_lateral_lock_active_) {
	          const double straight_lateral_limit =
	            std::max(
	              std::abs(straight_lateral_lock_goal_tolerance_),
	              std::abs(limited_forward_err) *
	              std::abs(straight_lateral_lock_max_lateral_ratio_));
	          lateral_err_for_control =
	            clampAbs(control_lateral_err, straight_lateral_limit);
	        }

	        const double limited_lateral_err = std::clamp(
	          lateral_err_for_control,
	          -std::abs(max_lateral_error_for_control_),
	          std::abs(max_lateral_error_for_control_));
        double lateral_control_err = limited_lateral_err;

        if (!current_r2_lane_drive_) {
          const double lateral_deadband = current_lateral_stop_tolerance_;
          const double abs_lateral_err = std::abs(limited_lateral_err);

          if (abs_lateral_err <= lateral_deadband) {
            lateral_control_err = 0.0;
          } else {
            lateral_control_err =
              std::copysign(abs_lateral_err - lateral_deadband, limited_lateral_err);
          }
        }
        const double forward_scale =
  axisSlowScale(std::abs(limited_forward_err));

/*
  横向不要像前进一样在末端被压得太狠。
  原逻辑 lateral_scale = axisSlowScale(abs(lateral_err))，
  会导致横向剩 10cm 左右时 scale 只有 0.2 左右，vy 很小，横移非常慢。

  这里保证横向 scale 最低 0.55：
  - 前进仍然正常减速
  - 横移末端不会慢到不动
  - 视觉小距离纠偏也更容易执行
*/
double lateral_scale = 1.0;

if (!straight_lateral_lock_active_) {
  lateral_scale = axisSlowScale(std::abs(lateral_control_err));
  lateral_scale = std::max(lateral_scale, 0.55);
}


        double d_forward = 0.0;
        double d_lateral = 0.0;

        if (last_err_valid_) {
          d_forward = (limited_forward_err - last_forward_err_) / dt;
          d_lateral = (lateral_control_err - last_lateral_err_) / dt;
        }

        last_forward_err_ = limited_forward_err;
        last_lateral_err_ = lateral_control_err;
        last_err_valid_ = true;

        d_forward = clampAbs(d_forward, 5.0);
        d_lateral = clampAbs(d_lateral, 5.0);

        double forward_cmd =
          k_forward_ * limited_forward_err * forward_scale +
          kd_forward_ * d_forward;

        double lateral_cmd =
          k_lateral_ * lateral_control_err * lateral_scale +
          kd_lateral_ * d_lateral;

        double yaw_lateral_ff = 0.0;
        if (enable_yaw_lateral_feedforward_ &&
            straight_lateral_lock_active_ &&
            !current_r2_lane_drive_) {
          const double move_eyaw = angleDiff(move_phase_target_yaw_, current_pose_.yaw);
          yaw_lateral_ff =
            yaw_lateral_feedforward_gain_ * forward_cmd * std::sin(move_eyaw);
          yaw_lateral_ff =
            clampAbs(yaw_lateral_ff, std::abs(yaw_lateral_feedforward_max_));
          lateral_cmd += yaw_lateral_ff;
        }

        if (std::abs(forward_cmd) < forward_min_) {
          forward_cmd = 0.0;
        }

        if (std::abs(lateral_cmd) < lateral_min_) {
          lateral_cmd = 0.0;
        }

        forward_cmd = clampAbs(forward_cmd, std::abs(forward_max_));
        lateral_cmd = clampAbs(lateral_cmd, std::abs(lateral_max_));

	        if (enable_min_approach_speed_ &&
	            motion_error < slow_down_radius_ &&
	            !pos_release_reached) {
	          applyMinPlanarSpeed(
	            forward_cmd,
            lateral_cmd,
            min_approach_linear_speed_,
            std::abs(forward_max_),
            std::abs(lateral_max_),
            min_approach_boost_lateral_);
        }

	        if (enable_stuck_release_ && pos_release_reached && current_path_block_ < 0) {
          const double no_progress_time = (now_t - last_progress_time_).seconds();

          if (no_progress_time >= stuck_release_time_) {
            if ((now_t - pos_reached_since_).seconds() >= pos_stable_time_) {
              publishHardStop();
              acceptPositionAndGoAlign(dist, false);
              break;
            }
          }
        }

        if (!pos_release_reached) {
          pos_reached_since_ = now_t;
        }

        double wz_cmd = 0.0;

        if (publish_angular_z_) {
          const double move_eyaw = angleDiff(move_phase_target_yaw_, current_pose_.yaw);
          wz_cmd = clampAbs(kw_ * move_eyaw, std::abs(wz_max_));
        }

        if (debug_force_zero_angular_z_) {
          wz_cmd = 0.0;
        }

        if (debug_safe_mode_) {
          forward_cmd = clampAbs(forward_cmd, debug_safe_forward_max_);
          lateral_cmd = clampAbs(lateral_cmd, debug_safe_lateral_max_);
          wz_cmd = clampAbs(wz_cmd, debug_safe_wz_max_);
        }

        geometry_msgs::msg::Twist target_cmd;
        target_cmd.linear.x = cmd_x_sign_ * forward_cmd;
        target_cmd.linear.y = cmd_y_sign_ * lateral_cmd;
        target_cmd.angular.z = wz_cmd;

        applyXZoneScaleToFinalCmd(target_cmd, in_x_slow_zone, in_x_boost_zone);

	        if (enable_min_approach_speed_ &&
	            motion_error < slow_down_radius_ &&
	            !pos_release_reached) {
	          double vx = target_cmd.linear.x;
	          double vy = target_cmd.linear.y;

          applyMinPlanarSpeed(
            vx,
            vy,
            min_approach_linear_speed_,
            std::abs(forward_max_),
            std::abs(lateral_max_),
            min_approach_boost_lateral_);

          target_cmd.linear.x = vx;
          target_cmd.linear.y = vy;
        }

        geometry_msgs::msg::Twist cmd = applyAccelerationLimit(target_cmd);
        cmd_vel_pub_->publish(cmd);

        if (debug_log_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), debug_print_every_ms_,
            "[MOVE] cur=(%.3f, %.3f, %.3f) goal=(%.3f, %.3f, %.3f) "
            "dist=%.4f best=%.4f f_err=%.3f l_err=%.3f straight_lock=%d "
            "scale=(%.2f, %.2f) yaw_ff=%.3f cmd=(%.3f, %.3f, %.3f) "
            "x_slow=%d x_boost=%d queue=%zu pickup=%d block=%d",
            current_pose_.x, current_pose_.y, current_pose_.yaw,
            goal_pose_.x, goal_pose_.y, goal_pose_.yaw,
            dist, best_dist_,
            control_forward_err,
            control_lateral_err,
            straight_lateral_lock_active_ ? 1 : 0,
            forward_scale,
            lateral_scale,
            yaw_lateral_ff,
            cmd.linear.x, cmd.linear.y, cmd.angular.z,
            in_x_slow_zone ? 1 : 0,
            in_x_boost_zone ? 1 : 0,
            goal_queue_.size(),
            current_waypoint_is_pickup_ ? 1 : 0,
            current_pickup_block_);
        }

        (void)display_forward_err;
        (void)display_lateral_err;

        break;
      }

      case State::ALIGN_YAW:
      {
        publishTargetYaw(goal_pose_.yaw);

        double wz_cmd = 0.0;

        if (publish_angular_z_) {
          wz_cmd = clampAbs(kw_ * eyaw, std::abs(wz_max_));
        }

        if (debug_force_zero_angular_z_) {
          wz_cmd = 0.0;
        }

        geometry_msgs::msg::Twist target_cmd;
        target_cmd.angular.z = isYawReached(eyaw) ? 0.0 : wz_cmd;

        applyXZoneScaleToFinalCmd(target_cmd, in_x_slow_zone, in_x_boost_zone);

        geometry_msgs::msg::Twist cmd = applyAccelerationLimit(target_cmd);
        cmd_vel_pub_->publish(cmd);

        if (debug_log_) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), debug_print_every_ms_,
            "[ALIGN] pre_align=%d cur_yaw=%.3f goal_yaw=%.3f eyaw=%.3f "
            "cmd_wz=%.3f queue=%zu pickup=%d block=%d",
            pre_align_mode_ ? 1 : 0,
            current_pose_.yaw,
            goal_pose_.yaw,
            eyaw,
            cmd.angular.z,
            goal_queue_.size(),
            current_waypoint_is_pickup_ ? 1 : 0,
            current_pickup_block_);
        }

		        if (isYawReached(eyaw)) {
		          if ((now_t - yaw_reached_since_).seconds() >= yaw_stable_time_) {
		            if (pre_align_mode_) {
		              pre_align_mode_ = false;
		              setState(State::MOVE_TO_POS);
		            } else {
		              setState(State::HOLD);
		            }
		          }
		        } else {
		          yaw_reached_since_ = now_t;
	        }

        break;
      }

      case State::HOLD:
	      {
	        publishTargetYaw(goal_pose_.yaw);
	        publishStop();

        if (debug_log_) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), debug_print_every_ms_,
            "[HOLD] yaw_only=%d cur=(%.3f, %.3f, %.3f) goal=(%.3f, %.3f, %.3f) "
            "dist=%.4f eyaw=%.3f queue=%zu pickup=%d block=%d",
            yaw_only_mode_ ? 1 : 0,
            current_pose_.x, current_pose_.y, current_pose_.yaw,
            goal_pose_.x, goal_pose_.y, goal_pose_.yaw,
            dist, eyaw,
            goal_queue_.size(),
            current_waypoint_is_pickup_ ? 1 : 0,
            current_pickup_block_);
        }

if ((now_t - state_enter_time_).seconds() >= hold_time_) {
	  if (current_path_block_ == 0 && current_allow_block0_can_ && !infrared_open_sent_at_block0_) {
    publishInfraredOpenOnceAtBlock0(
      current_path_block_,
      "waypoint hold completed, stop vehicle and send CAN 0x66 data 0x01 before next waypoint");

    RCLCPP_WARN(
      get_logger(),
      "Block 0 reached. Vehicle stopped. CAN 0x66 data 0x01 sent. "
      "Wait %.2f sec before next waypoint.",
      block0_after_open_wait_sec_);

    setState(State::BLOCK0_WAIT_AFTER_OPEN);
    break;
  }

  if (current_waypoint_is_pickup_ && enable_pickup_action_) {
    RCLCPP_WARN(
      get_logger(),
      "Pickup waypoint reached. Start pickup flow: block=%d",
      current_pickup_block_);

    startPickupForward();
  } else {
    setState(State::DONE);
  }
}


        break;
      }
      
      case State::BLOCK0_WAIT_AFTER_OPEN:
{
  publishTargetYaw(goal_pose_.yaw);
  publishHardStop();

  const double waited = (now_t - state_enter_time_).seconds();

  RCLCPP_WARN_THROTTLE(
    get_logger(),
    *get_clock(),
    300,
    "[BLOCK0_WAIT_AFTER_OPEN] vehicle stopped after CAN 0x01, waited=%.2f / %.2f sec",
    waited,
    block0_after_open_wait_sec_);

  if (waited >= block0_after_open_wait_sec_) {
    RCLCPP_WARN(
      get_logger(),
      "Block 0 wait after CAN 0x01 finished. Continue to next waypoint.");

    setState(State::DONE);
  }

  break;
}


      case State::PICKUP_FORWARD:
      {
        /*
          抓取流程第 1 步：
          到达 pickup waypoint 后，先前进 pickup_forward_distance_。
          前进完成后，不再立刻后退，而是启动视觉横向校准。
        */
        publishPickupLinearCommand(std::abs(pickup_forward_speed_));

        const double duration = pickupMotionDuration(
          pickup_forward_distance_,
          pickup_forward_speed_);

        if ((now_t - pickup_forward_start_time_).seconds() >= duration) {
          publishHardStop();

          pickup_waiting_vision_lateral_ = true;
          pickup_vision_lateral_done_ = false;
          pickup_reset_kfs_sent_ = false;
          pickup_vision_lateral_block_ = current_pickup_block_;


          RCLCPP_WARN(
            get_logger(),
            "Pickup block %d: forward %.3f m finished. Start vision lateral correction before grasp.",
            current_pickup_block_,
            pickup_forward_distance_);

          /*
            当前先默认左臂。
            如果你要右臂，改成 ActiveArm::RIGHT。
            后续也可以根据 current_pickup_block_ 自动选择左右臂。
          */
          startVisionLateralCorrection(ActiveArm::LEFT);
        }

        break;
      }


            case State::PICKUP_REVERSE:
      {
        /*
          新流程最后一步：
          机械臂抓取完成后，底盘后退 pickup_forward_distance_，
          然后发送 0x01，结束 pickup，继续下一个 waypoint。
        */
        publishPickupLinearCommand(-std::abs(pickup_reverse_speed_));

        const double duration = pickupMotionDuration(
          pickup_forward_distance_,
          pickup_reverse_speed_);

        if ((now_t - pickup_reverse_start_time_).seconds() >= duration) {
          publishHardStop();

          if (!pickup_step_on_sent_) {
            publishStepCommand(
              0x01,
              "after arm done and reverse 20cm finished for pickup block " +
              std::to_string(current_pickup_block_) +
              ", open infrared / disable stair mode");

            pickup_step_on_sent_ = true;
          }

          RCLCPP_WARN(
            get_logger(),
            "Pickup block %d: reverse %.3f m finished and 0x01 sent. Pickup flow done.",
            current_pickup_block_,
            pickup_forward_distance_);

	          current_waypoint_is_pickup_ = false;
	          current_pickup_block_ = -1;
		          current_path_block_ = -1;
		          current_waypoint_source_ = "unknown";
		          current_position_accepted_ = false;
			          current_allow_block0_can_ = true;
			          current_force_pre_align_yaw_ = false;
		          current_use_goal_yaw_during_move_ = false;
		          straight_lateral_lock_active_ = false;
	          arm_grab_done_received_ = false;

          pickup_waiting_vision_lateral_ = false;
          pickup_vision_lateral_done_ = false;
          pickup_reset_kfs_sent_ = false;

          setState(State::DONE);
        }

        break;
      }


            case State::PICKUP_WAIT_ARM_DONE:
      {
        /*
          新流程：
          前进 0.2m -> 视觉识别 -> 横向校准 -> reset_KFS -> 等机械臂抓取完成。
          收到 arm_grab_done=true 后，再后退 0.2m。
        */
        publishHardStop();

        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "[PICKUP_WAIT_ARM_DONE] waiting arm_grab_done=true. block=%d vision_done=%d reset_kfs_sent=%d",
          current_pickup_block_,
          pickup_vision_lateral_done_ ? 1 : 0,
          pickup_reset_kfs_sent_ ? 1 : 0);

        if (arm_grab_done_received_) {
          RCLCPP_WARN(
            get_logger(),
            "Pickup block %d: arm_done received. Start reverse %.3f m.",
            current_pickup_block_,
            pickup_forward_distance_);

          arm_grab_done_received_ = false;
          pickup_reverse_start_time_ = now();

          setState(State::PICKUP_REVERSE);
        }

        break;
      }

           case State::VISION_LATERAL_ALIGN:
      {
        /*
          注意：
          snapshot_offset_avg 是相机固化后的单次 offset，不是连续反馈。
          所以这里不能用 cmd_vel 闭环一直调。
          正确做法：
            1. 等待 offset
            2. 根据 offset 生成一次相对横移目标
            3. 复用 MOVE_TO_POS 去横移
            4. 横移完成后在 DONE 里回到 pickup 流程
        */
        publishHardStop();
        publishTargetYaw(current_pose_.yaw);

        double offset = 0.0;
        rclcpp::Time offset_stamp = now();

        const bool has_offset = getActiveVisionOffset(offset, offset_stamp);

        if (!has_offset) {
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "[VISION_LATERAL] Waiting snapshot offset. arm=%s left_received=%d right_received=%d",
            activeArmToString(active_vision_arm_),
            left_offset_received_ ? 1 : 0,
            right_offset_received_ ? 1 : 0);

          break;
        }

        const double age = (now_t - offset_stamp).seconds();

        if (age > vision_offset_timeout_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "[VISION_LATERAL] Snapshot offset timeout. arm=%s offset=%.6f age=%.3f timeout=%.3f",
            activeArmToString(active_vision_arm_),
            offset,
            age,
            vision_offset_timeout_);

          break;
        }

        /*
          如果 offset 已经很小，不需要横移。
          直接发布 reset_KFS，然后等机械臂完成。
        */
        if (std::abs(offset) <= vision_lateral_tolerance_) {
          RCLCPP_WARN(
            get_logger(),
            "[VISION_LATERAL] Offset already within tolerance. arm=%s offset=%.6f tol=%.6f",
            activeArmToString(active_vision_arm_),
            offset,
            vision_lateral_tolerance_);

          lateral_correction_active_ = false;
          active_vision_arm_ = ActiveArm::NONE;

	        if (pickup_waiting_vision_lateral_) {
            pickup_waiting_vision_lateral_ = false;
            pickup_vision_lateral_done_ = true;

            current_waypoint_is_pickup_ = true;
            current_pickup_block_ = pickup_vision_lateral_block_;

            if (auto_publish_reset_kfs_after_lateral_done_ && !pickup_reset_kfs_sent_) {
              publishResetKfs();
              pickup_reset_kfs_sent_ = true;
            }

            pickup_state_start_time_ = now();
            arm_grab_done_received_ = false;

            RCLCPP_WARN(
              get_logger(),
              "[PICKUP] Offset small. reset_KFS sent. Wait arm_grab_done. block=%d",
              current_pickup_block_);

            setState(State::PICKUP_WAIT_ARM_DONE);
	          } else {
	            active_goal_ = false;
	            current_force_pre_align_yaw_ = false;
	            current_use_goal_yaw_during_move_ = false;
	            setState(State::IDLE);
	          }

          break;
        }

        /*
          把 offset 转成一次相对横移距离。
          如果方向反了，改参数 vision_lateral_sign = -1.0。
        */
 double relative_dy =
  vision_lateral_sign_ * vision_lateral_k_ * offset;

relative_dy = clampAbs(relative_dy, vision_lateral_max_);

if (std::abs(relative_dy) > vision_lateral_tolerance_ &&
    vision_lateral_min_ > 1e-6 &&
    std::abs(relative_dy) < vision_lateral_min_) {
  relative_dy = std::copysign(vision_lateral_min_, relative_dy);
}

const double cyaw = std::cos(current_pose_.yaw);
const double syaw = std::sin(current_pose_.yaw);

Waypoint wp;
wp.pose.x = current_pose_.x - syaw * relative_dy;
wp.pose.y = current_pose_.y + cyaw * relative_dy;
wp.pose.yaw = current_pose_.yaw;
wp.mode = MoveMode::FREE_2D;
wp.source =
  "vision_lateral_correction_" +
  std::string(activeArmToString(active_vision_arm_)) +
  "_offset_" + std::to_string(offset);

/*
  注意：
  这里不能设置 is_pickup=true。
  否则横移完成后 HOLD 会再次进入 startPickupForward，形成循环。
  所以用 pickup_vision_lateral_block_ 单独保存 block。
*/
wp.is_pickup = false;
wp.pickup_block = -1;
wp.path_block = -1;
wp.camera_look_backward = false;

/*
  关键：
  视觉横向纠偏通常只有几厘米，不能使用普通路径点 7cm 容差。
  否则 initial_dist <= pos_tolerance 时会直接 yaw_only_mode=true，
  导致根本不横移。

  这里单独给视觉横移设置更小容差：
    - custom_pos_tolerance 约等于视觉容差，最低 8mm
    - custom_pos_release_tolerance 稍大，最低 15mm

  同时 startWaypoint() 里会自动把 current_lateral_stop_tolerance_
  压到 current_pos_tolerance_ * 0.5，
  避免 2~4cm 视觉横移被 lateral_stop_tolerance 吃掉。
*/
wp.use_custom_pos_tolerance = true;
wp.custom_pos_tolerance =
  std::max(0.008, std::abs(vision_lateral_tolerance_));
wp.custom_pos_release_tolerance =
  std::max(0.015, std::abs(vision_lateral_tolerance_) * 1.5);


        RCLCPP_WARN(
          get_logger(),
          "[VISION_LATERAL] Create one-shot lateral move. arm=%s offset=%.6f dy=%.6f "
          "current=(%.3f, %.3f, %.3f) goal=(%.3f, %.3f, %.3f) pickup_mode=%d pickup_block=%d",
          activeArmToString(active_vision_arm_),
          offset,
          relative_dy,
          current_pose_.x,
          current_pose_.y,
          current_pose_.yaw,
          wp.pose.x,
          wp.pose.y,
          wp.pose.yaw,
          pickup_waiting_vision_lateral_ ? 1 : 0,
          pickup_vision_lateral_block_);

        /*
          关键：
          startWaypoint 会把 current_pickup_block_ 改成 -1。
          没关系，我们已经用 pickup_vision_lateral_block_ 保存了。
        */
        startWaypoint(wp);

        break;
      }



            case State::DONE:
      {
        publishStop();

        RCLCPP_INFO(get_logger(), "Waypoint completed.");

        /*
          如果刚刚完成的是 pickup 视觉横向校准产生的相对横移动作，
          不要继续取下一个 waypoint，而是回到 pickup 流程：
          1. 恢复 pickup block
          2. 发布 reset_KFS
          3. 等机械臂 arm_grab_done
        */
        if (pickup_waiting_vision_lateral_) {
          pickup_waiting_vision_lateral_ = false;
          pickup_vision_lateral_done_ = true;

          current_waypoint_is_pickup_ = true;
          current_pickup_block_ = pickup_vision_lateral_block_;

          RCLCPP_WARN(
            get_logger(),
            "[PICKUP] Vision lateral correction move completed for block %d.",
            current_pickup_block_);

          if (auto_publish_reset_kfs_after_lateral_done_ && !pickup_reset_kfs_sent_) {
            publishResetKfs();
            pickup_reset_kfs_sent_ = true;
          }

          active_vision_arm_ = ActiveArm::NONE;
          lateral_correction_active_ = false;

          pickup_state_start_time_ = now();
          arm_grab_done_received_ = false;

          RCLCPP_WARN(
            get_logger(),
            "[PICKUP] reset_KFS sent. Now wait /arm_grab_done=true before reverse. block=%d",
            current_pickup_block_);

          setState(State::PICKUP_WAIT_ARM_DONE);
	          break;
	        }

	        if (has_deferred_r2_path_) {
	          auto path_msg =
	            std::make_shared<std_msgs::msg::Int32MultiArray>(deferred_r2_path_);
	          has_deferred_r2_path_ = false;
	          active_goal_ = false;

	          RCLCPP_WARN(
	            get_logger(),
	            "Current waypoint completed. Start deferred R2 path rebuild now.");

	          setState(State::IDLE);
	          r2PathCallback(path_msg);
	          break;
	        }
	
	        startNextWaypointFromQueue();
        break;
      }


    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LiteTaskController>());
  rclcpp::shutdown();
  return 0;
}
