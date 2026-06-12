#include <chrono>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"

using namespace std::chrono_literals;

class OmniNavController : public rclcpp::Node
{
public:
  OmniNavController() : Node("omni_nav_controller")
  {
    // ---------------- Topics ----------------
    declare_parameter("cmd_vel_topic", "/cmd_vel");
    declare_parameter("target_yaw_topic", "/target_yaw");
    declare_parameter("odom_topic", "/odometry/filtered");
    declare_parameter("scan_topic", "/scan");
    declare_parameter("motor_velocity_topic", "/motor_velocity");
    declare_parameter("rotate_cmd_topic", "/rotate_cmd");

    // ---------------- TF yaw source ----------------
    // 重定位模式推荐：
    // use_tf_yaw=true
    // yaw_frame=map
    // base_frame=base_link
    declare_parameter("use_tf_yaw", true);
    declare_parameter("use_yaw_topic", true);
    declare_parameter("yaw_topic", "/yaw");
    declare_parameter("yaw_in_degrees", true);
    declare_parameter("yaw_frame", "map");
    declare_parameter("base_frame", "base_link");
    declare_parameter("tf_lookup_timeout", 0.03);
    declare_parameter("stop_when_yaw_lost", true);

    // ---------------- Velocity mapping ----------------
    declare_parameter("scale_vx", 2000.0);
    declare_parameter("scale_vy", 2000.0);
    declare_parameter("sign_vx", 1.0);
    declare_parameter("sign_vy", 1.0);
    declare_parameter("max_vx", 200.0);
    declare_parameter("max_vy", 200.0);
    declare_parameter("min_start_vx", 30.0);
    declare_parameter("min_start_vy", 30.0);
    declare_parameter("deadband_vx", 8.0);
    declare_parameter("deadband_vy", 8.0);
    declare_parameter("alpha_vx", 0.5);
    declare_parameter("alpha_vy", 0.5);

    // ---------------- Axis remap ----------------
    declare_parameter("swap_xy", false);
    declare_parameter("invert_x", false);
    declare_parameter("invert_y", false);

    // ---------------- Timeout ----------------
    declare_parameter("cmd_timeout", 0.5);
    declare_parameter("target_yaw_timeout", 0.5);
    declare_parameter("stop_on_cmd_timeout", true);
    declare_parameter("hold_last_target_yaw", false);

    // ---------------- Target yaw input unit ----------------
    // false: /target_yaw 输入 rad，内部转 deg
    // true : /target_yaw 输入 deg
    declare_parameter("target_yaw_in_degrees", false);

    // ---------------- Obstacle slowdown ----------------
    declare_parameter("enable_scan_obstacle", false);
    declare_parameter("safe_distance", 0.3);
    declare_parameter("obstacle_slowdown_ratio", 0.3);
    declare_parameter("obstacle_detect_count", 3);
    declare_parameter("obstacle_free_count", 5);
    declare_parameter("obstacle_front_ratio", 0.25);
    declare_parameter("slowdown_vx_only", true);

    // ---------------- Yaw-motion gate ----------------
    declare_parameter("enable_yaw_motion_gate", true);
    declare_parameter("yaw_stop_threshold_deg", 15.0);
    declare_parameter("yaw_slow_threshold_deg", 5.0);
    declare_parameter("yaw_slow_min_scale", 0.15);

    // ---------------- Debug ----------------
    declare_parameter("debug_log", true);
    declare_parameter("debug_print_every_ms", 1000);

    loadParameters();

    // ---------------- TF ----------------
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---------------- Subscribers ----------------
    auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_,
      cmd_qos,
      std::bind(&OmniNavController::cmdVelCb, this, std::placeholders::_1));

    target_yaw_sub_ = create_subscription<std_msgs::msg::Float64>(
      target_yaw_topic_,
      rclcpp::QoS(20),
      std::bind(&OmniNavController::targetYawCb, this, std::placeholders::_1));

    current_yaw_sub_ = create_subscription<std_msgs::msg::Float64>(
      yaw_topic_,
      rclcpp::QoS(20),
      std::bind(&OmniNavController::currentYawCb, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::QoS(20),
      std::bind(&OmniNavController::odomCb, this, std::placeholders::_1));

    updateScanSubscription();

    // ---------------- Publishers ----------------
    motor_pub_ = create_publisher<std_msgs::msg::Int16MultiArray>(
      motor_velocity_topic_,
      rclcpp::QoS(20));

    rotate_pub_ = create_publisher<std_msgs::msg::Float64>(
      rotate_cmd_topic_,
      rclcpp::QoS(20));

    // ---------------- Timer ----------------
    timer_ = create_wall_timer(
      20ms,
      std::bind(&OmniNavController::controlLoop, this));

    // ---------------- Dynamic parameters ----------------
    param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&OmniNavController::onParamChange, this, std::placeholders::_1));

    last_control_time_ = this->now();

    RCLCPP_INFO(
      get_logger(),
      "OmniNavController started. TF map yaw + old deg rotate_cmd logic.");

    printConfig();
  }

private:
  // ---------------- ROS ----------------
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_yaw_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr current_yaw_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr motor_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rotate_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // ---------------- TF ----------------
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ---------------- Parameters ----------------
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string target_yaw_topic_{"/target_yaw"};
  std::string odom_topic_{"/odometry/filtered"};
  std::string scan_topic_{"/scan"};
  std::string motor_velocity_topic_{"/motor_velocity"};
  std::string rotate_cmd_topic_{"/rotate_cmd"};

  bool use_tf_yaw_{true};
  bool use_yaw_topic_{true};
  std::string yaw_topic_{"/yaw"};
  bool yaw_in_degrees_{true};
  std::string yaw_frame_{"map"};
  std::string base_frame_{"base_link"};
  double tf_lookup_timeout_{0.03};
  bool stop_when_yaw_lost_{true};

  double scale_vx_{2000.0};
  double scale_vy_{2000.0};
  double sign_vx_{1.0};
  double sign_vy_{1.0};
  double max_vx_{200.0};
  double max_vy_{200.0};
  double min_start_vx_{30.0};
  double min_start_vy_{30.0};
  double deadband_vx_{8.0};
  double deadband_vy_{8.0};
  double alpha_vx_{0.5};
  double alpha_vy_{0.5};

  bool swap_xy_{false};
  bool invert_x_{false};
  bool invert_y_{false};

  double cmd_timeout_{0.5};
  double target_yaw_timeout_{0.5};
  bool stop_on_cmd_timeout_{true};
  bool hold_last_target_yaw_{false};
  bool target_yaw_in_degrees_{false};

  bool enable_scan_obstacle_{false};
  double safe_distance_{0.3};
  double obstacle_slowdown_ratio_{0.3};
  int obstacle_detect_count_threshold_{3};
  int obstacle_free_count_threshold_{5};
  double obstacle_front_ratio_{0.25};
  bool slowdown_vx_only_{true};

  bool enable_yaw_motion_gate_{true};
  double yaw_stop_threshold_deg_{15.0};
  double yaw_slow_threshold_deg_{5.0};
  double yaw_slow_min_scale_{0.15};

  bool debug_log_{true};
  int debug_print_every_ms_{1000};

  // ---------------- State ----------------
  double cmd_vx_{0.0};
  double cmd_vy_{0.0};
  double cmd_wz_{0.0};
  bool received_cmd_{false};
  rclcpp::Time last_cmd_time_;

  bool received_yaw_{false};
  bool received_tf_yaw_{false};
  bool received_odom_yaw_{false};

  // 保持旧版逻辑：内部 yaw 使用 deg
  double current_yaw_deg_{0.0};
  double odom_yaw_deg_{0.0};

  bool received_target_yaw_{false};
  double target_yaw_deg_{0.0};
  rclcpp::Time last_target_yaw_time_;

  bool obstacle_{false};
  int obstacle_count_{0};
  int free_count_{0};

  double vx_filtered_{0.0};
  double vy_filtered_{0.0};
  rclcpp::Time last_control_time_;

  // 用于全向避障：记录底盘当前的期望运动方向
  double chassis_vx_target_{0.0};
  double chassis_vy_target_{0.0};

  // ---------------- Parameter utils ----------------
  void loadParameters()
  {
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    target_yaw_topic_ = get_parameter("target_yaw_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    scan_topic_ = get_parameter("scan_topic").as_string();
    motor_velocity_topic_ = get_parameter("motor_velocity_topic").as_string();
    rotate_cmd_topic_ = get_parameter("rotate_cmd_topic").as_string();

    use_tf_yaw_ = get_parameter("use_tf_yaw").as_bool();
    use_yaw_topic_ = get_parameter("use_yaw_topic").as_bool();
    yaw_topic_ = get_parameter("yaw_topic").as_string();
    yaw_in_degrees_ = get_parameter("yaw_in_degrees").as_bool();
    yaw_frame_ = get_parameter("yaw_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    tf_lookup_timeout_ = std::max(0.001, get_parameter("tf_lookup_timeout").as_double());
    stop_when_yaw_lost_ = get_parameter("stop_when_yaw_lost").as_bool();

    scale_vx_ = get_parameter("scale_vx").as_double();
    scale_vy_ = get_parameter("scale_vy").as_double();
    sign_vx_ = get_parameter("sign_vx").as_double();
    sign_vy_ = get_parameter("sign_vy").as_double();

    max_vx_ = std::max(0.0, get_parameter("max_vx").as_double());
    max_vy_ = std::max(0.0, get_parameter("max_vy").as_double());

    min_start_vx_ = std::max(0.0, get_parameter("min_start_vx").as_double());
    min_start_vy_ = std::max(0.0, get_parameter("min_start_vy").as_double());

    deadband_vx_ = std::max(0.0, get_parameter("deadband_vx").as_double());
    deadband_vy_ = std::max(0.0, get_parameter("deadband_vy").as_double());

    alpha_vx_ = std::clamp(get_parameter("alpha_vx").as_double(), 0.0, 1.0);
    alpha_vy_ = std::clamp(get_parameter("alpha_vy").as_double(), 0.0, 1.0);

    swap_xy_ = get_parameter("swap_xy").as_bool();
    invert_x_ = get_parameter("invert_x").as_bool();
    invert_y_ = get_parameter("invert_y").as_bool();

    cmd_timeout_ = std::max(0.01, get_parameter("cmd_timeout").as_double());
    target_yaw_timeout_ = std::max(0.01, get_parameter("target_yaw_timeout").as_double());

    stop_on_cmd_timeout_ = get_parameter("stop_on_cmd_timeout").as_bool();
    hold_last_target_yaw_ = get_parameter("hold_last_target_yaw").as_bool();
    target_yaw_in_degrees_ = get_parameter("target_yaw_in_degrees").as_bool();

    enable_scan_obstacle_ = get_parameter("enable_scan_obstacle").as_bool();
    safe_distance_ = std::max(0.01, get_parameter("safe_distance").as_double());

    obstacle_slowdown_ratio_ =
      std::clamp(get_parameter("obstacle_slowdown_ratio").as_double(), 0.0, 1.0);

    obstacle_detect_count_threshold_ =
      std::max(1, static_cast<int>(get_parameter("obstacle_detect_count").as_int()));

    obstacle_free_count_threshold_ =
      std::max(1, static_cast<int>(get_parameter("obstacle_free_count").as_int()));

    obstacle_front_ratio_ =
      std::clamp(get_parameter("obstacle_front_ratio").as_double(), 0.01, 1.0);

    slowdown_vx_only_ = get_parameter("slowdown_vx_only").as_bool();

    enable_yaw_motion_gate_ = get_parameter("enable_yaw_motion_gate").as_bool();

    yaw_stop_threshold_deg_ =
      std::max(0.1, get_parameter("yaw_stop_threshold_deg").as_double());

    yaw_slow_threshold_deg_ =
      std::max(0.0, get_parameter("yaw_slow_threshold_deg").as_double());

    yaw_slow_min_scale_ =
      std::clamp(get_parameter("yaw_slow_min_scale").as_double(), 0.0, 1.0);

    if (yaw_slow_threshold_deg_ > yaw_stop_threshold_deg_) {
      yaw_slow_threshold_deg_ = yaw_stop_threshold_deg_;
    }

    debug_log_ = get_parameter("debug_log").as_bool();
    debug_print_every_ms_ =
      std::max(100, static_cast<int>(get_parameter("debug_print_every_ms").as_int()));
  }

  void printConfig()
  {
    RCLCPP_INFO(
      get_logger(),
      "Config: cmd_vel=%s target_yaw=%s odom=%s motor=%s rotate=%s "
      "use_tf_yaw=%d use_yaw_topic=%d yaw_topic=%s yaw_input=%s "
      "yaw_frame=%s base_frame=%s tf_timeout=%.3f stop_when_yaw_lost=%d "
      "scale=(%.1f, %.1f) sign=(%.1f, %.1f) max=(%.1f, %.1f) "
      "min_start=(%.1f, %.1f) deadband=(%.1f, %.1f) alpha=(%.2f, %.2f) "
      "swap_xy=%d invert_x=%d invert_y=%d "
      "cmd_timeout=%.2f target_yaw_timeout=%.2f hold_last_yaw=%d "
      "target_yaw_input=%s rotate_cmd_output=deg "
      "yaw_gate=%d stop_th=%.1fdeg slow_th=%.1fdeg slow_min=%.2f "
      "scan_obstacle=%d safe_dist=%.2f slowdown_ratio=%.2f debug=%d debug_period_ms=%d",
      cmd_vel_topic_.c_str(),
      target_yaw_topic_.c_str(),
      odom_topic_.c_str(),
      motor_velocity_topic_.c_str(),
      rotate_cmd_topic_.c_str(),
      use_tf_yaw_,
      use_yaw_topic_,
      yaw_topic_.c_str(),
      yaw_in_degrees_ ? "deg" : "rad",
      yaw_frame_.c_str(),
      base_frame_.c_str(),
      tf_lookup_timeout_,
      stop_when_yaw_lost_,
      scale_vx_,
      scale_vy_,
      sign_vx_,
      sign_vy_,
      max_vx_,
      max_vy_,
      min_start_vx_,
      min_start_vy_,
      deadband_vx_,
      deadband_vy_,
      alpha_vx_,
      alpha_vy_,
      swap_xy_,
      invert_x_,
      invert_y_,
      cmd_timeout_,
      target_yaw_timeout_,
      hold_last_target_yaw_,
      target_yaw_in_degrees_ ? "deg" : "rad",
      enable_yaw_motion_gate_,
      yaw_stop_threshold_deg_,
      yaw_slow_threshold_deg_,
      yaw_slow_min_scale_,
      enable_scan_obstacle_,
      safe_distance_,
      obstacle_slowdown_ratio_,
      debug_log_,
      debug_print_every_ms_);
  }

  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    try {
      for (const auto & p : params) {
        const auto & name = p.get_name();

        if (name == "use_tf_yaw") {
          use_tf_yaw_ = p.as_bool();
        } else if (name == "use_yaw_topic") {
          use_yaw_topic_ = p.as_bool();
        } else if (name == "yaw_topic") {
          yaw_topic_ = p.as_string();
        } else if (name == "yaw_in_degrees") {
          yaw_in_degrees_ = p.as_bool();
        } else if (name == "yaw_frame") {
          yaw_frame_ = p.as_string();
        } else if (name == "base_frame") {
          base_frame_ = p.as_string();
        } else if (name == "tf_lookup_timeout") {
          tf_lookup_timeout_ = std::max(0.001, p.as_double());
        } else if (name == "stop_when_yaw_lost") {
          stop_when_yaw_lost_ = p.as_bool();

        } else if (name == "scale_vx") {
          scale_vx_ = p.as_double();
        } else if (name == "scale_vy") {
          scale_vy_ = p.as_double();
        } else if (name == "sign_vx") {
          sign_vx_ = p.as_double();
        } else if (name == "sign_vy") {
          sign_vy_ = p.as_double();

        } else if (name == "max_vx") {
          max_vx_ = std::max(0.0, p.as_double());
        } else if (name == "max_vy") {
          max_vy_ = std::max(0.0, p.as_double());
        } else if (name == "min_start_vx") {
          min_start_vx_ = std::max(0.0, p.as_double());
        } else if (name == "min_start_vy") {
          min_start_vy_ = std::max(0.0, p.as_double());
        } else if (name == "deadband_vx") {
          deadband_vx_ = std::max(0.0, p.as_double());
        } else if (name == "deadband_vy") {
          deadband_vy_ = std::max(0.0, p.as_double());

        } else if (name == "alpha_vx") {
          alpha_vx_ = std::clamp(p.as_double(), 0.0, 1.0);
        } else if (name == "alpha_vy") {
          alpha_vy_ = std::clamp(p.as_double(), 0.0, 1.0);

        } else if (name == "swap_xy") {
          swap_xy_ = p.as_bool();
        } else if (name == "invert_x") {
          invert_x_ = p.as_bool();
        } else if (name == "invert_y") {
          invert_y_ = p.as_bool();

        } else if (name == "cmd_timeout") {
          cmd_timeout_ = std::max(0.01, p.as_double());
        } else if (name == "target_yaw_timeout") {
          target_yaw_timeout_ = std::max(0.01, p.as_double());
        } else if (name == "stop_on_cmd_timeout") {
          stop_on_cmd_timeout_ = p.as_bool();
        } else if (name == "hold_last_target_yaw") {
          hold_last_target_yaw_ = p.as_bool();
        } else if (name == "target_yaw_in_degrees") {
          target_yaw_in_degrees_ = p.as_bool();

        } else if (name == "enable_scan_obstacle") {
          enable_scan_obstacle_ = p.as_bool();
        } else if (name == "safe_distance") {
          safe_distance_ = std::max(0.01, p.as_double());
        } else if (name == "obstacle_slowdown_ratio") {
          obstacle_slowdown_ratio_ = std::clamp(p.as_double(), 0.0, 1.0);
        } else if (name == "obstacle_detect_count") {
          obstacle_detect_count_threshold_ =
            std::max(1, static_cast<int>(p.as_int()));
        } else if (name == "obstacle_free_count") {
          obstacle_free_count_threshold_ =
            std::max(1, static_cast<int>(p.as_int()));
        } else if (name == "obstacle_front_ratio") {
          obstacle_front_ratio_ = std::clamp(p.as_double(), 0.01, 1.0);
        } else if (name == "slowdown_vx_only") {
          slowdown_vx_only_ = p.as_bool();

        } else if (name == "enable_yaw_motion_gate") {
          enable_yaw_motion_gate_ = p.as_bool();
        } else if (name == "yaw_stop_threshold_deg") {
          yaw_stop_threshold_deg_ = std::max(0.1, p.as_double());
        } else if (name == "yaw_slow_threshold_deg") {
          yaw_slow_threshold_deg_ = std::max(0.0, p.as_double());
        } else if (name == "yaw_slow_min_scale") {
          yaw_slow_min_scale_ = std::clamp(p.as_double(), 0.0, 1.0);

        } else if (name == "debug_log") {
          debug_log_ = p.as_bool();
        } else if (name == "debug_print_every_ms") {
          debug_print_every_ms_ = std::max(100, static_cast<int>(p.as_int()));
        }
      }

      if (yaw_slow_threshold_deg_ > yaw_stop_threshold_deg_) {
        yaw_slow_threshold_deg_ = yaw_stop_threshold_deg_;
      }

      updateScanSubscription();

    } catch (const std::exception & e) {
      result.successful = false;
      result.reason = std::string("Parameter update failed: ") + e.what();
    }

    return result;
  }

  // ---------------- Callbacks ----------------
  void cmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    cmd_vx_ = msg->linear.x;
    cmd_vy_ = msg->linear.y;
    cmd_wz_ = msg->angular.z;

    last_cmd_time_ = this->now();
    received_cmd_ = true;
  }

  void targetYawCb(const std_msgs::msg::Float64::SharedPtr msg)
  {
    const double input_value = msg->data;

    // 保持旧版逻辑：
    // /target_yaw 默认 rad，内部转 deg。
    if (target_yaw_in_degrees_) {
      target_yaw_deg_ = normalizeAngle180(input_value);
    } else {
      target_yaw_deg_ = normalizeAngle180(input_value * 180.0 / M_PI);
    }

    received_target_yaw_ = true;
    last_target_yaw_time_ = this->now();
  }

  void currentYawCb(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!use_yaw_topic_) {
      return;
    }

    const double input_value = msg->data;

    if (!std::isfinite(input_value)) {
      return;
    }

    if (yaw_in_degrees_) {
      current_yaw_deg_ = normalizeAngle180(input_value);
    } else {
      current_yaw_deg_ = normalizeAngle180(input_value * 180.0 / M_PI);
    }

    received_yaw_ = true;
  }

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & q = msg->pose.pose.orientation;

    if (!std::isfinite(q.x) || !std::isfinite(q.y) ||
        !std::isfinite(q.z) || !std::isfinite(q.w)) {
      return;
    }

    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(quat);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    odom_yaw_deg_ = normalizeAngle180(yaw * 180.0 / M_PI);
    received_odom_yaw_ = true;

    // 如果不用 TF yaw 且不用 /yaw topic，回退到旧版逻辑：用 /odometry/filtered 的 yaw。
    if (!use_tf_yaw_ && !use_yaw_topic_) {
      current_yaw_deg_ = odom_yaw_deg_;
      received_yaw_ = true;
    }
  }

  void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const int n = static_cast<int>(msg->ranges.size());

    if (n <= 0) {
      return;
    }

    // 根据底盘当前期望运动方向，动态计算雷达检测中心
    const double speed = std::hypot(chassis_vx_target_, chassis_vy_target_);
    double move_angle_rad = 0.0;

    if (speed > 1.0) {
      move_angle_rad = std::atan2(chassis_vy_target_, chassis_vx_target_);
    }

    int center_idx = n / 2;

    if (msg->angle_increment > 0.0) {
      double angle_diff = move_angle_rad - msg->angle_min;

      while (angle_diff < 0.0) {
        angle_diff += 2.0 * M_PI;
      }

      while (angle_diff >= 2.0 * M_PI) {
        angle_diff -= 2.0 * M_PI;
      }

      center_idx = static_cast<int>(std::round(angle_diff / msg->angle_increment));
      center_idx = std::clamp(center_idx, 0, n - 1);
    }

    const int half_window =
      std::max(1, static_cast<int>(n * obstacle_front_ratio_ * 0.5));

    bool detected = false;

    for (int i = -half_window; i <= half_window; ++i) {
      int idx = (center_idx + i) % n;

      if (idx < 0) {
        idx += n;
      }

      const float r = msg->ranges[idx];

      if (std::isfinite(r) &&
          r >= msg->range_min &&
          r <= msg->range_max &&
          r < safe_distance_) {
        detected = true;
        break;
      }
    }

    if (detected) {
      obstacle_count_++;
      free_count_ = 0;
    } else {
      free_count_++;
      obstacle_count_ = 0;
    }

    if (obstacle_count_ >= obstacle_detect_count_threshold_) {
      obstacle_ = true;
    }

    if (free_count_ >= obstacle_free_count_threshold_) {
      obstacle_ = false;
    }
  }

  void updateScanSubscription()
  {
    if (enable_scan_obstacle_) {
      if (!scan_sub_) {
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
          scan_topic_,
          rclcpp::SensorDataQoS(),
          std::bind(&OmniNavController::scanCb, this, std::placeholders::_1));
      }
    } else {
      scan_sub_.reset();
      obstacle_ = false;
      obstacle_count_ = 0;
      free_count_ = 0;
    }
  }

  // ---------------- Utils ----------------
  double normalizeAngle180(double deg) const
  {
    double x = std::fmod(deg + 180.0, 360.0);

    if (x < 0.0) {
      x += 360.0;
    }

    return x - 180.0;
  }

  bool updateYawFromTF()
  {
    if (use_yaw_topic_) {
      return received_yaw_;
    }

    if (!use_tf_yaw_) {
      return received_yaw_;
    }

    try {
      if (!tf_buffer_->canTransform(
            yaw_frame_,
            base_frame_,
            tf2::TimePointZero,
            tf2::durationFromSec(tf_lookup_timeout_))) {
        received_tf_yaw_ = false;

        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          debug_print_every_ms_,
          "TF %s -> %s is not available for yaw",
          yaw_frame_.c_str(),
          base_frame_.c_str());

        return false;
      }

      const auto tf = tf_buffer_->lookupTransform(
        yaw_frame_,
        base_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(tf_lookup_timeout_));

      const auto & q = tf.transform.rotation;

      tf2::Quaternion quat(q.x, q.y, q.z, q.w);
      tf2::Matrix3x3 m(quat);

      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      current_yaw_deg_ = normalizeAngle180(yaw * 180.0 / M_PI);

      received_tf_yaw_ = true;
      received_yaw_ = true;

      return true;

    } catch (const tf2::TransformException & ex) {
      received_tf_yaw_ = false;

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        debug_print_every_ms_,
        "Failed to lookup TF %s -> %s for yaw: %s",
        yaw_frame_.c_str(),
        base_frame_.c_str(),
        ex.what());

      return false;
    }
  }

  double applyMinStart(double value, double min_start) const
  {
    if (std::abs(value) < 1e-6) {
      return 0.0;
    }

    if (std::abs(value) < min_start) {
      return std::copysign(min_start, value);
    }

    return value;
  }

  double applyDeadband(double value, double deadband) const
  {
    if (std::abs(value) < deadband) {
      return 0.0;
    }

    return value;
  }

  double lowPass(double input, double prev, double alpha, double dt) const
  {
    constexpr double nominal_dt = 0.05;

    double adjusted_alpha = alpha * (dt / nominal_dt);
    adjusted_alpha = std::clamp(adjusted_alpha, 0.0, 1.0);

    return adjusted_alpha * input + (1.0 - adjusted_alpha) * prev;
  }

  double clampAbs(double value, double max_abs) const
  {
    return std::clamp(value, -max_abs, max_abs);
  }

  int16_t toInt16(double value) const
  {
    value = std::clamp(value, -32768.0, 32767.0);
    return static_cast<int16_t>(std::lround(value));
  }

  double computeMotionScaleFromYawError(double angle_err_deg) const
  {
    if (!enable_yaw_motion_gate_) {
      return 1.0;
    }

    const double abs_err = std::abs(angle_err_deg);

    if (abs_err >= yaw_stop_threshold_deg_) {
      return 0.0;
    }

    if (abs_err <= yaw_slow_threshold_deg_) {
      return 1.0;
    }

    const double denom =
      std::max(1e-6, yaw_stop_threshold_deg_ - yaw_slow_threshold_deg_);

    const double t = (abs_err - yaw_slow_threshold_deg_) / denom;
    const double scale = 1.0 - t * (1.0 - yaw_slow_min_scale_);

    return std::clamp(scale, yaw_slow_min_scale_, 1.0);
  }

  void publishMotor(double vx, double vy)
  {
    std_msgs::msg::Int16MultiArray motor_msg;
    motor_msg.data = {
      0,
      toInt16(vx),
      toInt16(vy),
      0
    };

    motor_pub_->publish(motor_msg);
  }

  void publishRotateDeg(double yaw_deg)
  {
    std_msgs::msg::Float64 rotate_msg;
    rotate_msg.data = normalizeAngle180(yaw_deg);
    rotate_pub_->publish(rotate_msg);
  }

  // ---------------- Main loop ----------------
  void controlLoop()
  {
    const auto now_t = this->now();

    double dt = (now_t - last_control_time_).seconds();
    last_control_time_ = now_t;

    if (dt <= 0.0 || dt > 0.2) {
      dt = 0.05;
    }

    // 0) 更新 yaw
    // 默认从 TF map -> base_link 获取 map 下 yaw。
    const bool yaw_ok = updateYawFromTF();

    if (!yaw_ok && stop_when_yaw_lost_) {
      vx_filtered_ = lowPass(0.0, vx_filtered_, alpha_vx_, dt);
      vy_filtered_ = lowPass(0.0, vy_filtered_, alpha_vy_, dt);

      publishMotor(vx_filtered_, vy_filtered_);

      if (debug_log_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          debug_print_every_ms_,
          "Yaw lost. stop_when_yaw_lost=true. motor=[0,%d,%d,0]",
          static_cast<int>(toInt16(vx_filtered_)),
          static_cast<int>(toInt16(vy_filtered_)));
      }

      return;
    }

    // 1) cmd_vel validity
    bool cmd_valid = false;
    double ros_vx_cmd = 0.0;
    double ros_vy_cmd = 0.0;
    double ros_wz_cmd = 0.0;

    if (received_cmd_) {
      const double cmd_age = (now_t - last_cmd_time_).seconds();

      if (cmd_age <= cmd_timeout_) {
        cmd_valid = true;
        ros_vx_cmd = cmd_vx_;
        ros_vy_cmd = cmd_vy_;
        ros_wz_cmd = cmd_wz_;
      }
    }

    // 2) target_yaw validity
    bool target_yaw_valid = false;
    double rotate_target_deg = 0.0;

    if (received_target_yaw_) {
      const double yaw_age = (now_t - last_target_yaw_time_).seconds();

      if (yaw_age <= target_yaw_timeout_) {
        target_yaw_valid = true;
        rotate_target_deg = target_yaw_deg_;
      }
    }

    // 3) yaw error
    double angle_err_deg = 0.0;

    if (received_yaw_ && target_yaw_valid) {
      angle_err_deg = normalizeAngle180(target_yaw_deg_ - current_yaw_deg_);
    }

    // 4) ROS axes -> chassis axes remap
    double mapped_x_cmd = ros_vx_cmd;
    double mapped_y_cmd = ros_vy_cmd;

    if (swap_xy_) {
      mapped_x_cmd = ros_vy_cmd;
      mapped_y_cmd = ros_vx_cmd;
    }

    if (invert_x_) {
      mapped_x_cmd = -mapped_x_cmd;
    }

    if (invert_y_) {
      mapped_y_cmd = -mapped_y_cmd;
    }

    // 5) scale, deadband, min_start
    double vx_target = 0.0;
    double vy_target = 0.0;

    if (cmd_valid) {
      vx_target = sign_vx_ * scale_vx_ * mapped_x_cmd;
      vy_target = sign_vy_ * scale_vy_ * mapped_y_cmd;

      vx_target = applyDeadband(vx_target, deadband_vx_);
      vy_target = applyDeadband(vy_target, deadband_vy_);

      vx_target = applyMinStart(vx_target, min_start_vx_);
      vy_target = applyMinStart(vy_target, min_start_vy_);
    } else if (stop_on_cmd_timeout_) {
      vx_target = 0.0;
      vy_target = 0.0;
    }

    chassis_vx_target_ = vx_target;
    chassis_vy_target_ = vy_target;

    // 6) low-pass filter
    const bool hard_stop_xy =
      (!cmd_valid && stop_on_cmd_timeout_) ||
      (std::abs(vx_target) < 1e-9 && std::abs(vy_target) < 1e-9);

    if (hard_stop_xy) {
      vx_filtered_ = 0.0;
      vy_filtered_ = 0.0;
    } else {
      vx_filtered_ = lowPass(vx_target, vx_filtered_, alpha_vx_, dt);
      vy_filtered_ = lowPass(vy_target, vy_filtered_, alpha_vy_, dt);
    }

    // 7) obstacle slowdown
    if (enable_scan_obstacle_ && obstacle_) {
      if (slowdown_vx_only_) {
        if (vx_filtered_ > 0.0) {
          vx_filtered_ *= obstacle_slowdown_ratio_;
        }
      } else {
        vx_filtered_ *= obstacle_slowdown_ratio_;
        vy_filtered_ *= obstacle_slowdown_ratio_;
      }
    }

    // 8) yaw-motion gate
    double motion_scale = 1.0;

    if (received_yaw_ && target_yaw_valid) {
      motion_scale = computeMotionScaleFromYawError(angle_err_deg);

      vx_filtered_ *= motion_scale;
      vy_filtered_ *= motion_scale;
    }

    // 9) clamp
    vx_filtered_ = clampAbs(vx_filtered_, max_vx_);
    vy_filtered_ = clampAbs(vy_filtered_, max_vy_);

    // 10) publish motor_velocity
    publishMotor(vx_filtered_, vy_filtered_);

    // 11) publish rotate_cmd
    // 保持旧版逻辑：/rotate_cmd 输出 deg
    if (target_yaw_valid) {
      publishRotateDeg(rotate_target_deg);
    } else if (hold_last_target_yaw_ && received_target_yaw_) {
      publishRotateDeg(target_yaw_deg_);
    }

    // 12) debug log
    if (debug_log_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        debug_print_every_ms_,
        "yaw_src=%s yaw_ok=%d cmd_valid=%d ros_cmd(vx=%.3f vy=%.3f wz=%.3f) "
        "mapped_cmd(x=%.3f y=%.3f) swap_xy=%d invert_x=%d invert_y=%d "
        "-> motor=[0,%d,%d,0] "
        "yaw_cur=%.2fdeg yaw_tgt=%.2fdeg yaw_valid=%d err=%.2fdeg motion_scale=%.2f "
        "obs=%d target_input_unit=%s odom_yaw=%.2fdeg rotate_cmd_out=deg",
        use_tf_yaw_ ? "tf_map" : "odom",
        yaw_ok,
        cmd_valid,
        ros_vx_cmd,
        ros_vy_cmd,
        ros_wz_cmd,
        mapped_x_cmd,
        mapped_y_cmd,
        swap_xy_,
        invert_x_,
        invert_y_,
        static_cast<int>(toInt16(vx_filtered_)),
        static_cast<int>(toInt16(vy_filtered_)),
        current_yaw_deg_,
        target_yaw_deg_,
        target_yaw_valid,
        angle_err_deg,
        motion_scale,
        obstacle_,
        target_yaw_in_degrees_ ? "deg" : "rad",
        odom_yaw_deg_);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OmniNavController>());
  rclcpp::shutdown();
  return 0;
}
