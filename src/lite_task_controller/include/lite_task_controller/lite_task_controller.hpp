#pragma once

#include <chrono>
#include <cmath>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"

#include "lite_task_controller/srv/set_relative_goal.hpp"
#include "lite_task_controller/srv/set_absolute_goal.hpp"

class LiteTaskController : public rclcpp::Node
{
public:
  LiteTaskController();

private:
  enum class State
  {
    IDLE = 0,
    MOVE_TO_POS = 1,
    ALIGN_YAW = 2,
    HOLD = 3,
    DONE = 4
  };

  struct Pose2D
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_yaw_pub_;
  rclcpp::Service<lite_task_controller::srv::SetRelativeGoal>::SharedPtr relative_goal_srv_;
  rclcpp::Service<lite_task_controller::srv::SetAbsoluteGoal>::SharedPtr absolute_goal_srv_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Current pose
  Pose2D current_pose_;
  bool odom_received_{false};

  // Goal pose
  Pose2D goal_pose_;
  bool active_goal_{false};

  // State machine
  State state_{State::IDLE};
  rclcpp::Time state_enter_time_;
  rclcpp::Time pos_reached_since_;
  rclcpp::Time yaw_reached_since_;

  // Parameters
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string target_yaw_topic_;

  double control_frequency_;

  double kx_;
  double ky_;

  double vx_max_;
  double vy_max_;
  double vx_min_;
  double vy_min_;

  double slow_down_radius_;
  double pos_tolerance_;
  double pos_tolerance_hysteresis_;
  double yaw_tolerance_;
  double yaw_tolerance_hysteresis_;

  double pos_stable_time_;
  double yaw_stable_time_;
  double hold_time_;

  bool publish_angular_z_;
  double kw_;
  double wz_max_;

  bool keep_yaw_during_move_;
  double fixed_move_yaw_;
  bool use_initial_yaw_for_move_{true};

  // Functions
  void declareAndLoadParameters();
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlLoop();

  void handleRelativeGoal(
    const std::shared_ptr<lite_task_controller::srv::SetRelativeGoal::Request> request,
    std::shared_ptr<lite_task_controller::srv::SetRelativeGoal::Response> response);

  void handleAbsoluteGoal(
    const std::shared_ptr<lite_task_controller::srv::SetAbsoluteGoal::Request> request,
    std::shared_ptr<lite_task_controller::srv::SetAbsoluteGoal::Response> response);

  void setState(State new_state);
  std::string stateToString(State s) const;

  void publishStop();
  void publishTargetYaw(double yaw);
  void publishCmdVel(double vx, double vy, double wz);

  double getYawFromOdom(const nav_msgs::msg::Odometry::SharedPtr msg) const;
  double normalizeAngle(double angle) const;
  double angleDiff(double target, double current) const;
  double clamp(double val, double min_val, double max_val) const;
  double sign(double v) const;

  bool isPositionReached(double ex, double ey) const;
  bool isYawReached(double eyaw) const;

  Pose2D makeAbsoluteGoalFromRelative(double dx, double dy, double dyaw) const;
};
