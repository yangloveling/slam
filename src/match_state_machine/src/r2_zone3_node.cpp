#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "lite_task_controller/srv/set_absolute_goal.hpp"

using namespace std::chrono_literals;

enum class R2Zone3State
{
  IDLE = 0,
  GOTO_RAMP_UP_POINT,
  FACE_GRID_POINT,
  SEND_CAN_01_OBSERVE_BOTTOM,
  WAIT_BOTTOM_GRID_RESULT,
  SELECT_MID_GRID_BY_BOTTOM_KFS,
  GOTO_SELECTED_MID_PREP_POINT,
  VISION_ALIGN_MID,
  PLACE_MID_KFS,
  MID_PLACE_DONE,
  GOTO_R1_WAIT_POINT,
  VISION_ALIGN_LIFT,
  SEND_CAN_03,
  WAIT_R2_ON_R1,
  WAIT_R1_MOVE_DONE,
  VISION_ALIGN_HIGH,
  PLACE_HIGH_KFS,
  TASK_DONE,
  ERROR
};

class R2Zone3Node : public rclcpp::Node
{
public:
  R2Zone3Node()
  : Node("r2_zone3_node")
  {
    declareParameters();
    loadParameters();

    if (!validateWaypoints())
    {
      state_ = R2Zone3State::ERROR;
      RCLCPP_ERROR(get_logger(), "路点配置错误，节点进入 ERROR 状态");
    }
    else
    {
      loadTaskPointsFromWaypoints();
      printWaypointsInfo();
    }

    createPublishers();
    createSubscribers();
    createClients();

    state_start_time_ = now();

    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&R2Zone3Node::controlLoop, this)
    );

    RCLCPP_INFO(
      get_logger(),
      "R2 三区节点已启动：Odometry + lite /set_absolute_goal + UInt32MultiArray CAN"
    );
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("start_signal_topic", "/r2/zone3_start");
    declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    declare_parameter<std::string>("goal_pose_topic", "/r2/goal_pose");

    declare_parameter<std::string>("absolute_goal_service", "/set_absolute_goal");
    declare_parameter<bool>("use_lite_absolute_goal", true);

    declare_parameter<std::string>("vision_priority_topic", "/r2/vision_priority");
    declare_parameter<std::string>("vision_offset_topic", "/r2/vision_offset");

    declare_parameter<std::string>("bottom_grid_result_topic", "/r2/bottom_grid_result");
    declare_parameter<std::string>("target_mid_grid_index_topic", "/r2/target_mid_grid_index");

    declare_parameter<std::string>("mid_place_cmd_topic", "/r2/mid_place_cmd");
    declare_parameter<std::string>("mid_place_done_topic", "/r2/mid_place_done");

    declare_parameter<std::string>("high_place_cmd_topic", "/r2/high_place_cmd");
    declare_parameter<std::string>("high_place_done_topic", "/r2/high_place_done");

    declare_parameter<std::string>("r2_on_r1_topic", "/r2/on_r1");
    declare_parameter<std::string>("r1_move_done_topic", "/r1/move_done");

    declare_parameter<std::string>("can_tx_topic", "/can_tx");

    declare_parameter<double>("control_rate", 50.0);

    declare_parameter<double>("nav_pos_tolerance", 0.08);
    declare_parameter<double>("nav_yaw_tolerance", 0.08);
    declare_parameter<double>("nav_execution_timeout", 30.0);

    declare_parameter<std::vector<double>>("waypoints_x", std::vector<double>{});
    declare_parameter<std::vector<double>>("waypoints_y", std::vector<double>{});
    declare_parameter<std::vector<double>>("waypoints_yaw", std::vector<double>{});
    declare_parameter<std::vector<std::string>>("waypoint_names", std::vector<std::string>{});

    declare_parameter<int>("ramp_up_waypoint_index", 0);
    declare_parameter<int>("face_grid_waypoint_index", 1);
    declare_parameter<int>("mid_left_prep_waypoint_index", 2);
    declare_parameter<int>("mid_right_prep_waypoint_index", 3);
    declare_parameter<int>("r1_wait_waypoint_index", 4);
    declare_parameter<int>("high_place_waypoint_index", 5);

    declare_parameter<std::string>("self_color", "r");
    declare_parameter<std::string>("bottom_kfs_select_strategy", "center_first");
    declare_parameter<double>("wait_bottom_grid_result_timeout_sec", 2.0);

    declare_parameter<double>("vision_mid_dx_threshold", 0.025);
    declare_parameter<double>("vision_mid_dy_threshold", 0.025);
    declare_parameter<double>("vision_mid_yaw_threshold", 0.05);

    declare_parameter<double>("vision_lift_dx_threshold", 0.020);
    declare_parameter<double>("vision_lift_dy_threshold", 0.030);
    declare_parameter<double>("vision_lift_yaw_threshold", 0.04);

    declare_parameter<double>("vision_high_dx_threshold", 0.025);
    declare_parameter<double>("vision_high_dy_threshold", 0.025);
    declare_parameter<double>("vision_high_yaw_threshold", 0.05);

    declare_parameter<int>("vision_stable_count_required", 5);

    declare_parameter<double>("max_vx", 0.25);
    declare_parameter<double>("max_vy", 0.25);
    declare_parameter<double>("max_wz", 0.6);

    declare_parameter<double>("vision_kp_x", 1.0);
    declare_parameter<double>("vision_kp_y", 1.0);
    declare_parameter<double>("vision_kp_yaw", 1.5);

    declare_parameter<int>("can_id_observe_bottom", 0x88);
    declare_parameter<int>("can_cmd_observe_bottom", 0x02);

    declare_parameter<int>("can_id_climb", 0x66);
    declare_parameter<int>("can_cmd_climb", 0x03);

    declare_parameter<double>("wait_r2_on_r1_timeout_sec", 10.0);
    declare_parameter<double>("wait_r1_move_done_timeout_sec", 20.0);
    declare_parameter<double>("place_action_timeout_sec", 10.0);
  }

  void loadParameters()
  {
    start_signal_topic_ = get_parameter("start_signal_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    goal_pose_topic_ = get_parameter("goal_pose_topic").as_string();

    absolute_goal_service_ = get_parameter("absolute_goal_service").as_string();
    use_lite_absolute_goal_ = get_parameter("use_lite_absolute_goal").as_bool();

    vision_priority_topic_ = get_parameter("vision_priority_topic").as_string();
    vision_offset_topic_ = get_parameter("vision_offset_topic").as_string();

    bottom_grid_result_topic_ = get_parameter("bottom_grid_result_topic").as_string();
    target_mid_grid_index_topic_ = get_parameter("target_mid_grid_index_topic").as_string();

    mid_place_cmd_topic_ = get_parameter("mid_place_cmd_topic").as_string();
    mid_place_done_topic_ = get_parameter("mid_place_done_topic").as_string();

    high_place_cmd_topic_ = get_parameter("high_place_cmd_topic").as_string();
    high_place_done_topic_ = get_parameter("high_place_done_topic").as_string();

    r2_on_r1_topic_ = get_parameter("r2_on_r1_topic").as_string();
    r1_move_done_topic_ = get_parameter("r1_move_done_topic").as_string();

    can_tx_topic_ = get_parameter("can_tx_topic").as_string();

    control_rate_ = get_parameter("control_rate").as_double();

    arrive_xy_threshold_ = get_parameter("nav_pos_tolerance").as_double();
    arrive_yaw_threshold_ = get_parameter("nav_yaw_tolerance").as_double();
    nav_execution_timeout_ = get_parameter("nav_execution_timeout").as_double();

    waypoints_x_ = get_parameter("waypoints_x").as_double_array();
    waypoints_y_ = get_parameter("waypoints_y").as_double_array();
    waypoints_yaw_ = get_parameter("waypoints_yaw").as_double_array();
    waypoint_names_ = get_parameter("waypoint_names").as_string_array();

    ramp_up_waypoint_index_ = get_parameter("ramp_up_waypoint_index").as_int();
    face_grid_waypoint_index_ = get_parameter("face_grid_waypoint_index").as_int();
    mid_left_prep_waypoint_index_ = get_parameter("mid_left_prep_waypoint_index").as_int();
    mid_right_prep_waypoint_index_ = get_parameter("mid_right_prep_waypoint_index").as_int();
    r1_wait_waypoint_index_ = get_parameter("r1_wait_waypoint_index").as_int();
    high_place_waypoint_index_ = get_parameter("high_place_waypoint_index").as_int();

    self_color_ = get_parameter("self_color").as_string();
    normalizeSelfColor();

    bottom_kfs_select_strategy_ = get_parameter("bottom_kfs_select_strategy").as_string();

    wait_bottom_grid_result_timeout_sec_ =
      get_parameter("wait_bottom_grid_result_timeout_sec").as_double();

    vision_mid_dx_threshold_ = get_parameter("vision_mid_dx_threshold").as_double();
    vision_mid_dy_threshold_ = get_parameter("vision_mid_dy_threshold").as_double();
    vision_mid_yaw_threshold_ = get_parameter("vision_mid_yaw_threshold").as_double();

    vision_lift_dx_threshold_ = get_parameter("vision_lift_dx_threshold").as_double();
    vision_lift_dy_threshold_ = get_parameter("vision_lift_dy_threshold").as_double();
    vision_lift_yaw_threshold_ = get_parameter("vision_lift_yaw_threshold").as_double();

    vision_high_dx_threshold_ = get_parameter("vision_high_dx_threshold").as_double();
    vision_high_dy_threshold_ = get_parameter("vision_high_dy_threshold").as_double();
    vision_high_yaw_threshold_ = get_parameter("vision_high_yaw_threshold").as_double();

    vision_stable_count_required_ = get_parameter("vision_stable_count_required").as_int();

    max_vx_ = get_parameter("max_vx").as_double();
    max_vy_ = get_parameter("max_vy").as_double();
    max_wz_ = get_parameter("max_wz").as_double();

    vision_kp_x_ = get_parameter("vision_kp_x").as_double();
    vision_kp_y_ = get_parameter("vision_kp_y").as_double();
    vision_kp_yaw_ = get_parameter("vision_kp_yaw").as_double();

    can_id_observe_bottom_ =
      static_cast<uint32_t>(get_parameter("can_id_observe_bottom").as_int());
    can_cmd_observe_bottom_ =
      static_cast<uint8_t>(get_parameter("can_cmd_observe_bottom").as_int());

    can_id_climb_ =
      static_cast<uint32_t>(get_parameter("can_id_climb").as_int());
    can_cmd_climb_ =
      static_cast<uint8_t>(get_parameter("can_cmd_climb").as_int());

    wait_r2_on_r1_timeout_sec_ = get_parameter("wait_r2_on_r1_timeout_sec").as_double();
    wait_r1_move_done_timeout_sec_ = get_parameter("wait_r1_move_done_timeout_sec").as_double();
    place_action_timeout_sec_ = get_parameter("place_action_timeout_sec").as_double();
  }

  void createPublishers()
  {
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_pose_topic_, 10);

    vision_priority_pub_ =
      create_publisher<std_msgs::msg::UInt8>(vision_priority_topic_, 10);

    target_mid_grid_index_pub_ =
      create_publisher<std_msgs::msg::UInt8>(target_mid_grid_index_topic_, 10);

    mid_place_cmd_pub_ =
      create_publisher<std_msgs::msg::UInt8>(mid_place_cmd_topic_, 10);

    high_place_cmd_pub_ =
      create_publisher<std_msgs::msg::UInt8>(high_place_cmd_topic_, 10);

    can_tx_pub_ =
      create_publisher<std_msgs::msg::UInt32MultiArray>(can_tx_topic_, 10);
  }

  void createSubscribers()
  {
    start_sub_ = create_subscription<std_msgs::msg::Bool>(
      start_signal_topic_,
      10,
      std::bind(&R2Zone3Node::startCallback, this, std::placeholders::_1)
    );

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      20,
      std::bind(&R2Zone3Node::odomCallback, this, std::placeholders::_1)
    );

    vision_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      vision_offset_topic_,
      10,
      std::bind(&R2Zone3Node::visionCallback, this, std::placeholders::_1)
    );

    bottom_grid_result_sub_ = create_subscription<std_msgs::msg::String>(
      bottom_grid_result_topic_,
      10,
      std::bind(&R2Zone3Node::bottomGridResultCallback, this, std::placeholders::_1)
    );

    mid_place_done_sub_ = create_subscription<std_msgs::msg::Bool>(
      mid_place_done_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        mid_place_done_ = msg->data;
      }
    );

    high_place_done_sub_ = create_subscription<std_msgs::msg::Bool>(
      high_place_done_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        high_place_done_ = msg->data;
      }
    );

    r2_on_r1_sub_ = create_subscription<std_msgs::msg::Bool>(
      r2_on_r1_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        r2_on_r1_ = msg->data;
      }
    );

    r1_move_done_sub_ = create_subscription<std_msgs::msg::Bool>(
      r1_move_done_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        r1_move_done_ = msg->data;
      }
    );
  }

  void createClients()
  {
    absolute_goal_client_ =
      create_client<lite_task_controller::srv::SetAbsoluteGoal>(
        absolute_goal_service_
      );

    RCLCPP_INFO(
      get_logger(),
      "Lite absolute goal client created: service=%s enabled=%d",
      absolute_goal_service_.c_str(),
      use_lite_absolute_goal_ ? 1 : 0
    );
  }

  void startCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data)
    {
      return;
    }

    if (state_ == R2Zone3State::ERROR)
    {
      RCLCPP_ERROR(get_logger(), "当前为 ERROR 状态，无法启动");
      return;
    }

    if (state_ == R2Zone3State::IDLE || state_ == R2Zone3State::TASK_DONE)
    {
      RCLCPP_INFO(get_logger(), "收到 R2 三区启动信号");
      resetRuntimeFlags();
      changeState(R2Zone3State::GOTO_RAMP_UP_POINT);
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    const auto & q = msg->pose.pose.orientation;
    current_yaw_ = quaternionToYaw(q.x, q.y, q.z, q.w);

    pose_valid_ = true;
  }

  void visionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4)
    {
      vision_valid_ = false;
      return;
    }

    vision_valid_ = msg->data[0] > 0.5f;
    vision_dx_ = static_cast<double>(msg->data[1]);
    vision_dy_ = static_cast<double>(msg->data[2]);
    vision_yaw_ = static_cast<double>(msg->data[3]);
  }

  void bottomGridResultCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    bottom_grid_result_valid_ = parseBottomGridResult(msg->data);

    if (bottom_grid_result_valid_)
    {
      RCLCPP_INFO(get_logger(), "收到下层三个格子识别结果: %s", msg->data.c_str());
    }
    else
    {
      RCLCPP_WARN(get_logger(), "下层格子结果解析失败: %s", msg->data.c_str());
    }
  }

  void controlLoop()
  {
    switch (state_)
    {
      case R2Zone3State::IDLE:
        stopChassis();
        break;

      case R2Zone3State::GOTO_RAMP_UP_POINT:
        gotoRampUpPointState();
        break;

      case R2Zone3State::FACE_GRID_POINT:
        faceGridPointState();
        break;

      case R2Zone3State::SEND_CAN_01_OBSERVE_BOTTOM:
        sendCan01ObserveBottomState();
        break;

      case R2Zone3State::WAIT_BOTTOM_GRID_RESULT:
        waitBottomGridResultState();
        break;

      case R2Zone3State::SELECT_MID_GRID_BY_BOTTOM_KFS:
        selectMidGridByBottomKfsState();
        break;

      case R2Zone3State::GOTO_SELECTED_MID_PREP_POINT:
        gotoSelectedMidPrepPointState();
        break;

      case R2Zone3State::VISION_ALIGN_MID:
        visionAlignMidState();
        break;

      case R2Zone3State::PLACE_MID_KFS:
        placeMidKfsState();
        break;

      case R2Zone3State::MID_PLACE_DONE:
        midPlaceDoneState();
        break;

      case R2Zone3State::GOTO_R1_WAIT_POINT:
        gotoR1WaitPointState();
        break;

      case R2Zone3State::VISION_ALIGN_LIFT:
        visionAlignLiftState();
        break;

      case R2Zone3State::SEND_CAN_03:
        sendCan03State();
        break;

      case R2Zone3State::WAIT_R2_ON_R1:
        waitR2OnR1State();
        break;

      case R2Zone3State::WAIT_R1_MOVE_DONE:
        waitR1MoveDoneState();
        break;

      case R2Zone3State::VISION_ALIGN_HIGH:
        visionAlignHighState();
        break;

      case R2Zone3State::PLACE_HIGH_KFS:
        placeHighKfsState();
        break;

      case R2Zone3State::TASK_DONE:
        taskDoneState();
        break;

      case R2Zone3State::ERROR:
      default:
        errorState();
        break;
    }
  }

  void gotoRampUpPointState()
  {
    setVisionPriority(false);

    sendNavigationGoalOnce(ramp_x_, ramp_y_, ramp_yaw_, "ramp_up_point");

    if (checkStateTimeout(nav_execution_timeout_, "上坡进入三区超时"))
    {
      return;
    }

    if (isArrived(ramp_x_, ramp_y_, ramp_yaw_))
    {
      RCLCPP_INFO(get_logger(), "已上坡进入三区，准备转向朝向九宫格");
      stopChassis();
      changeState(R2Zone3State::FACE_GRID_POINT);
    }
  }

  void faceGridPointState()
  {
    setVisionPriority(false);

    sendNavigationGoalOnce(face_grid_x_, face_grid_y_, face_grid_yaw_, "face_grid_point");

    if (checkStateTimeout(nav_execution_timeout_, "转向朝向九宫格超时"))
    {
      return;
    }

    if (isArrived(face_grid_x_, face_grid_y_, face_grid_yaw_))
    {
      RCLCPP_INFO(get_logger(), "已到达九宫格观察位，准备发送 CAN 0x88 / 0x02 观察下层");
      stopChassis();
      changeState(R2Zone3State::SEND_CAN_01_OBSERVE_BOTTOM);
    }
  }

  void sendCan01ObserveBottomState()
  {
    stopChassis();

    if (!can_01_sent_)
    {
      sendCanFrame(can_id_observe_bottom_, can_cmd_observe_bottom_);
      can_01_sent_ = true;

      RCLCPP_INFO(
        get_logger(),
        "已发送观察下层 CAN: ID=0x%X DATA[0]=0x%02X",
        can_id_observe_bottom_,
        can_cmd_observe_bottom_
      );
    }

    setVisionPriority(true);

    bottom_grid_result_valid_ = false;
    selected_bottom_grid_index_ = -1;
    selected_mid_grid_index_ = -1;

    changeState(R2Zone3State::WAIT_BOTTOM_GRID_RESULT);
  }

  void waitBottomGridResultState()
  {
    stopChassis();
    setVisionPriority(true);

    if (bottom_grid_result_valid_)
    {
      RCLCPP_INFO(get_logger(), "收到下层三个格子结果，准备选择中层放置格");
      changeState(R2Zone3State::SELECT_MID_GRID_BY_BOTTOM_KFS);
      return;
    }

    if (checkStateTimeout(
        wait_bottom_grid_result_timeout_sec_,
        "等待九宫格下层识别结果超时"))
    {
      return;
    }
  }

  void selectMidGridByBottomKfsState()
  {
    stopChassis();
    setVisionPriority(true);

    if (!selectBottomKfsForMiddlePlace())
    {
      RCLCPP_ERROR(get_logger(), "未找到本方下层 R1 KFS，无法选择中层放置格");
      changeState(R2Zone3State::ERROR);
      return;
    }

    publishSelectedMidGridIndex();

    RCLCPP_INFO(
      get_logger(),
      "选择下层 grid=%d 的正上方中层 grid=%d 放置 R2 KFS",
      selected_bottom_grid_index_,
      selected_mid_grid_index_
    );

    if (!selectMidPrepPointByGrid())
    {
      changeState(R2Zone3State::ERROR);
      return;
    }

    vision_stable_count_ = 0;
    changeState(R2Zone3State::GOTO_SELECTED_MID_PREP_POINT);
  }

  bool selectMidPrepPointByGrid()
  {
    // selected_mid_grid_index_:
    // 0 = 左格
    // 1 = 中格
    // 2 = 右格
    //
    // 你的策略：
    // 左格 -> 左预备点
    // 中格 -> 右预备点
    // 右格 -> 右预备点

    if (selected_mid_grid_index_ == 0)
    {
      selected_mid_prep_x_ = mid_left_prep_x_;
      selected_mid_prep_y_ = mid_left_prep_y_;
      selected_mid_prep_yaw_ = mid_left_prep_yaw_;
      selected_mid_prep_name_ = "mid_left_prep_point";

      RCLCPP_INFO(
        get_logger(),
        "中层目标格=左格(%d)，选择左预备点: x=%.3f y=%.3f yaw=%.3f",
        selected_mid_grid_index_,
        selected_mid_prep_x_,
        selected_mid_prep_y_,
        selected_mid_prep_yaw_
      );

      return true;
    }

    if (selected_mid_grid_index_ == 1 || selected_mid_grid_index_ == 2)
    {
      selected_mid_prep_x_ = mid_right_prep_x_;
      selected_mid_prep_y_ = mid_right_prep_y_;
      selected_mid_prep_yaw_ = mid_right_prep_yaw_;
      selected_mid_prep_name_ = "mid_right_prep_point";

      RCLCPP_INFO(
        get_logger(),
        "中层目标格=%s(%d)，选择右预备点: x=%.3f y=%.3f yaw=%.3f",
        selected_mid_grid_index_ == 1 ? "中格" : "右格",
        selected_mid_grid_index_,
        selected_mid_prep_x_,
        selected_mid_prep_y_,
        selected_mid_prep_yaw_
      );

      return true;
    }

    RCLCPP_ERROR(
      get_logger(),
      "selected_mid_grid_index_ 非法: %d，无法选择中层预备点",
      selected_mid_grid_index_
    );

    return false;
  }

  void gotoSelectedMidPrepPointState()
  {
    setVisionPriority(false);

    sendNavigationGoalOnce(
      selected_mid_prep_x_,
      selected_mid_prep_y_,
      selected_mid_prep_yaw_,
      selected_mid_prep_name_
    );

    if (checkStateTimeout(nav_execution_timeout_, "前往选定中层放置预备点超时"))
    {
      return;
    }

    if (isArrived(
        selected_mid_prep_x_,
        selected_mid_prep_y_,
        selected_mid_prep_yaw_))
    {
      RCLCPP_INFO(
        get_logger(),
        "到达选定中层放置预备点: %s，准备进行中层视觉微调",
        selected_mid_prep_name_.c_str()
      );

      stopChassis();
      setVisionPriority(true);
      vision_stable_count_ = 0;

      changeState(R2Zone3State::VISION_ALIGN_MID);
    }
  }

  void visionAlignMidState()
  {
    if (visionFineAdjust(
        vision_mid_dx_threshold_,
        vision_mid_dy_threshold_,
        vision_mid_yaw_threshold_))
    {
      RCLCPP_INFO(get_logger(), "中层视觉微调完成，放置中层 KFS");
      stopChassis();
      changeState(R2Zone3State::PLACE_MID_KFS);
    }
  }

  void placeMidKfsState()
  {
    stopChassis();
    setVisionPriority(false);

    if (!mid_place_cmd_sent_)
    {
      std_msgs::msg::UInt8 cmd;
      cmd.data = 1;
      mid_place_cmd_pub_->publish(cmd);
      mid_place_cmd_sent_ = true;

      RCLCPP_INFO(get_logger(), "已发送中层 KFS 放置命令");
    }

    if (mid_place_done_)
    {
      RCLCPP_INFO(get_logger(), "中层 KFS 放置完成，前往 R1 等待点");
      changeState(R2Zone3State::MID_PLACE_DONE);
      return;
    }

    if (checkStateTimeout(place_action_timeout_sec_, "等待中层放置完成超时"))
    {
      return;
    }
  }

  void midPlaceDoneState()
  {
    mid_place_done_ = false;
    changeState(R2Zone3State::GOTO_R1_WAIT_POINT);
  }

  void gotoR1WaitPointState()
  {
    setVisionPriority(false);

    sendNavigationGoalOnce(r1_wait_x_, r1_wait_y_, r1_wait_yaw_, "r1_wait_point");

    if (checkStateTimeout(nav_execution_timeout_, "前往 R1 等待点超时"))
    {
      return;
    }

    if (isArrived(r1_wait_x_, r1_wait_y_, r1_wait_yaw_))
    {
      RCLCPP_INFO(get_logger(), "到达 R1 等待点，开启上 R1 前视觉微调");
      stopChassis();
      vision_stable_count_ = 0;
      setVisionPriority(true);
      changeState(R2Zone3State::VISION_ALIGN_LIFT);
    }
  }

  void visionAlignLiftState()
  {
    if (visionFineAdjust(
        vision_lift_dx_threshold_,
        vision_lift_dy_threshold_,
        vision_lift_yaw_threshold_))
    {
      RCLCPP_INFO(get_logger(), "上 R1 前视觉微调完成，准备发送 CAN 0x66 / 0x03");
      stopChassis();
      changeState(R2Zone3State::SEND_CAN_03);
    }
  }

  void sendCan03State()
  {
    stopChassis();

    if (!can_03_sent_)
    {
      sendCanFrame(can_id_climb_, can_cmd_climb_);
      can_03_sent_ = true;

      RCLCPP_INFO(
        get_logger(),
        "已发送上 R1 CAN: ID=0x%X DATA[0]=0x%02X",
        can_id_climb_,
        can_cmd_climb_
      );
    }

    changeState(R2Zone3State::WAIT_R2_ON_R1);
  }

  void waitR2OnR1State()
  {
    stopChassis();

    if (r2_on_r1_)
    {
      RCLCPP_INFO(get_logger(), "R2 已上到 R1，等待 R1 移动完成");
      setVisionPriority(false);
      changeState(R2Zone3State::WAIT_R1_MOVE_DONE);
      return;
    }

    if (checkStateTimeout(wait_r2_on_r1_timeout_sec_, "等待 R2 上 R1 超时"))
    {
      return;
    }
  }

  void waitR1MoveDoneState()
  {
    stopChassis();

    if (r1_move_done_)
    {
      RCLCPP_INFO(get_logger(), "R1 移动完成，开启高层视觉微调");
      vision_stable_count_ = 0;
      setVisionPriority(true);
      changeState(R2Zone3State::VISION_ALIGN_HIGH);
      return;
    }

    if (checkStateTimeout(wait_r1_move_done_timeout_sec_, "等待 R1 移动完成超时"))
    {
      return;
    }
  }

  void visionAlignHighState()
  {
    if (visionFineAdjust(
        vision_high_dx_threshold_,
        vision_high_dy_threshold_,
        vision_high_yaw_threshold_))
    {
      RCLCPP_INFO(get_logger(), "高层视觉微调完成，放置高层 KFS");
      stopChassis();
      changeState(R2Zone3State::PLACE_HIGH_KFS);
    }
  }

  void placeHighKfsState()
  {
    stopChassis();
    setVisionPriority(false);

    if (!high_place_cmd_sent_)
    {
      std_msgs::msg::UInt8 cmd;
      cmd.data = 1;
      high_place_cmd_pub_->publish(cmd);
      high_place_cmd_sent_ = true;

      RCLCPP_INFO(get_logger(), "已发送高层 KFS 放置命令");
    }

    if (high_place_done_)
    {
      RCLCPP_INFO(get_logger(), "高层 KFS 放置完成，三区任务结束");
      changeState(R2Zone3State::TASK_DONE);
      return;
    }

    if (checkStateTimeout(place_action_timeout_sec_, "等待高层放置完成超时"))
    {
      return;
    }
  }

  void taskDoneState()
  {
    stopChassis();
    setVisionPriority(false);
  }

  void errorState()
  {
    stopChassis();
    setVisionPriority(false);
  }

  void changeState(R2Zone3State new_state)
  {
    if (state_ == new_state)
    {
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "R2Zone3 状态切换: %d -> %d",
      static_cast<int>(state_),
      static_cast<int>(new_state)
    );

    state_ = new_state;
    state_start_time_ = now();

    nav_goal_sent_in_state_ = false;
  }

  void sendNavigationGoalOnce(double x, double y, double yaw, const std::string & name)
  {
    if (nav_goal_sent_in_state_)
    {
      return;
    }

    publishGoal(x, y, yaw);

    bool sent_ok = true;

    if (use_lite_absolute_goal_)
    {
      sent_ok = sendLiteAbsoluteGoal(x, y, yaw);
    }

    if (!sent_ok)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "导航目标暂未发送成功，稍后重试: %s x=%.3f y=%.3f yaw=%.3f",
        name.c_str(),
        x,
        y,
        yaw
      );
      return;
    }

    nav_goal_sent_in_state_ = true;

    RCLCPP_INFO(
      get_logger(),
      "导航目标已发送: %s x=%.3f y=%.3f yaw=%.3f",
      name.c_str(),
      x,
      y,
      yaw
    );
  }

  bool sendLiteAbsoluteGoal(double x, double y, double yaw)
  {
    if (!absolute_goal_client_)
    {
      RCLCPP_ERROR(get_logger(), "absolute_goal_client_ 未初始化");
      return false;
    }

    if (!absolute_goal_client_->wait_for_service(100ms))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "lite 服务 %s 不可用，无法发送目标",
        absolute_goal_service_.c_str()
      );
      return false;
    }

    auto req =
      std::make_shared<lite_task_controller::srv::SetAbsoluteGoal::Request>();

    req->x = x;
    req->y = y;
    req->yaw = yaw;

    using ServiceResponseFuture =
      rclcpp::Client<lite_task_controller::srv::SetAbsoluteGoal>::SharedFuture;

    auto response_callback =
      [this, x, y, yaw](ServiceResponseFuture future)
      {
        try
        {
          auto res = future.get();

          RCLCPP_INFO(
            get_logger(),
            "lite响应: success=%d goal=(%.3f, %.3f, %.3f)",
            res->success ? 1 : 0,
            x,
            y,
            yaw
          );
        }
        catch (const std::exception & e)
        {
          RCLCPP_ERROR(
            get_logger(),
            "lite service 回调异常: %s",
            e.what()
          );
        }
      };

    absolute_goal_client_->async_send_request(req, response_callback);

    RCLCPP_INFO(
      get_logger(),
      "已调用 lite /set_absolute_goal: x=%.3f y=%.3f yaw=%.3f",
      x,
      y,
      yaw
    );

    return true;
  }

  void publishGoal(double x, double y, double yaw)
  {
    geometry_msgs::msg::PoseStamped goal;

    goal.header.stamp = now();
    goal.header.frame_id = "map";

    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;

    double qx, qy, qz, qw;
    yawToQuaternion(yaw, qx, qy, qz, qw);

    goal.pose.orientation.x = qx;
    goal.pose.orientation.y = qy;
    goal.pose.orientation.z = qz;
    goal.pose.orientation.w = qw;

    goal_pub_->publish(goal);
  }

  void sendCanFrame(uint32_t can_id, uint8_t data0)
  {
    std_msgs::msg::UInt32MultiArray msg;

    msg.data.resize(3);
    msg.data[0] = can_id;
    msg.data[1] = 1;
    msg.data[2] = static_cast<uint32_t>(data0);

    can_tx_pub_->publish(msg);

    RCLCPP_INFO(
      get_logger(),
      "已发布 CAN 数组消息: topic=%s ID=0x%X DLC=1 DATA[0]=0x%02X",
      can_tx_topic_.c_str(),
      can_id,
      data0
    );
  }

  void stopChassis()
  {
    geometry_msgs::msg::Twist cmd;
    cmd_vel_pub_->publish(cmd);
  }

  void setVisionPriority(bool enable)
  {
    std_msgs::msg::UInt8 msg;
    msg.data = enable ? 1 : 0;
    vision_priority_pub_->publish(msg);
  }

  bool checkStateTimeout(double timeout_sec, const std::string & msg)
  {
    if ((now() - state_start_time_).seconds() > timeout_sec)
    {
      RCLCPP_ERROR(get_logger(), "%s", msg.c_str());
      changeState(R2Zone3State::ERROR);
      return true;
    }

    return false;
  }

  bool isArrived(double tx, double ty, double tyaw)
  {
    if (!pose_valid_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "尚未收到 odom: %s",
        odom_topic_.c_str()
      );
      return false;
    }

    const double dist = std::hypot(tx - current_x_, ty - current_y_);
    const double eyaw = normalizeAngle(tyaw - current_yaw_);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "导航误差: target=(%.3f, %.3f, %.3f), current=(%.3f, %.3f, %.3f), dist=%.3f, eyaw=%.3f",
      tx,
      ty,
      tyaw,
      current_x_,
      current_y_,
      current_yaw_,
      dist,
      eyaw
    );

    return dist < arrive_xy_threshold_ &&
           std::abs(eyaw) < arrive_yaw_threshold_;
  }

  bool visionFineAdjust(double dx_th, double dy_th, double yaw_th)
  {
    if (!vision_valid_)
    {
      vision_stable_count_ = 0;
      stopChassis();
      return false;
    }

    const bool ok =
      std::abs(vision_dx_) < dx_th &&
      std::abs(vision_dy_) < dy_th &&
      std::abs(vision_yaw_) < yaw_th;

    if (ok)
    {
      stopChassis();
      vision_stable_count_++;
      return vision_stable_count_ >= vision_stable_count_required_;
    }

    vision_stable_count_ = 0;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = clamp(-vision_kp_x_ * vision_dx_, max_vx_);
    cmd.linear.y = clamp(-vision_kp_y_ * vision_dy_, max_vy_);
    cmd.angular.z = clamp(-vision_kp_yaw_ * vision_yaw_, max_wz_);

    cmd_vel_pub_->publish(cmd);

    return false;
  }

  bool parseBottomGridResult(const std::string & data)
  {
    std::vector<int> parsed(3, 0);

    if (data.find(':') != std::string::npos)
    {
      std::stringstream ss(data);
      std::string item;

      while (std::getline(ss, item, ','))
      {
        auto pos = item.find(':');
        if (pos == std::string::npos)
        {
          continue;
        }

        try
        {
          int idx = std::stoi(trim(item.substr(0, pos)));
          if (idx >= 0 && idx < 3)
          {
            parsed[idx] = colorStringToValue(trim(item.substr(pos + 1)));
          }
        }
        catch (...)
        {
        }
      }

      bottom_grid_state_ = parsed;
      return true;
    }

    std::stringstream ss(data);
    std::string tok;
    std::vector<std::string> tokens;

    while (ss >> tok)
    {
      tokens.push_back(tok);
    }

    if (tokens.size() != 3)
    {
      return false;
    }

    for (int i = 0; i < 3; ++i)
    {
      parsed[i] = colorStringToValue(tokens[i]);
    }

    bottom_grid_state_ = parsed;
    return true;
  }

  bool selectBottomKfsForMiddlePlace()
  {
    const int target = self_color_ == "r" ? 1 : 2;

    std::vector<int> candidates;

    for (int i = 0; i < 3; ++i)
    {
      if (bottom_grid_state_[i] == target)
      {
        candidates.push_back(i);
      }
    }

    if (candidates.empty())
    {
      return false;
    }

    if (bottom_kfs_select_strategy_ == "center_first")
    {
      for (int p : {1, 0, 2})
      {
        if (std::find(candidates.begin(), candidates.end(), p) != candidates.end())
        {
          selected_bottom_grid_index_ = p;
          selected_mid_grid_index_ = p;
          return true;
        }
      }
    }

    selected_bottom_grid_index_ = candidates.front();
    selected_mid_grid_index_ = selected_bottom_grid_index_;
    return true;
  }

  void publishSelectedMidGridIndex()
  {
    if (selected_mid_grid_index_ < 0 || selected_mid_grid_index_ > 2)
    {
      return;
    }

    std_msgs::msg::UInt8 msg;
    msg.data = static_cast<uint8_t>(selected_mid_grid_index_);
    target_mid_grid_index_pub_->publish(msg);
  }

  bool validateWaypoints()
  {
    if (waypoints_x_.empty() || waypoints_y_.empty() || waypoints_yaw_.empty())
    {
      RCLCPP_ERROR(get_logger(), "waypoints_x/y/yaw 不能为空");
      return false;
    }

    if (waypoints_x_.size() != waypoints_y_.size() ||
        waypoints_x_.size() != waypoints_yaw_.size())
    {
      RCLCPP_ERROR(
        get_logger(),
        "waypoints_x/y/yaw 长度不一致: x=%zu y=%zu yaw=%zu",
        waypoints_x_.size(),
        waypoints_y_.size(),
        waypoints_yaw_.size()
      );
      return false;
    }

    const int n = static_cast<int>(waypoints_x_.size());

    return validIndex(ramp_up_waypoint_index_, n) &&
           validIndex(face_grid_waypoint_index_, n) &&
           validIndex(mid_left_prep_waypoint_index_, n) &&
           validIndex(mid_right_prep_waypoint_index_, n) &&
           validIndex(r1_wait_waypoint_index_, n) &&
           validIndex(high_place_waypoint_index_, n);
  }

  bool validIndex(int i, int n)
  {
    if (i < 0 || i >= n)
    {
      RCLCPP_ERROR(get_logger(), "路点索引越界: index=%d size=%d", i, n);
      return false;
    }

    return true;
  }

  void loadTaskPointsFromWaypoints()
  {
    ramp_x_ = waypoints_x_[ramp_up_waypoint_index_];
    ramp_y_ = waypoints_y_[ramp_up_waypoint_index_];
    ramp_yaw_ = waypoints_yaw_[ramp_up_waypoint_index_];

    face_grid_x_ = waypoints_x_[face_grid_waypoint_index_];
    face_grid_y_ = waypoints_y_[face_grid_waypoint_index_];
    face_grid_yaw_ = waypoints_yaw_[face_grid_waypoint_index_];

    mid_left_prep_x_ = waypoints_x_[mid_left_prep_waypoint_index_];
    mid_left_prep_y_ = waypoints_y_[mid_left_prep_waypoint_index_];
    mid_left_prep_yaw_ = waypoints_yaw_[mid_left_prep_waypoint_index_];

    mid_right_prep_x_ = waypoints_x_[mid_right_prep_waypoint_index_];
    mid_right_prep_y_ = waypoints_y_[mid_right_prep_waypoint_index_];
    mid_right_prep_yaw_ = waypoints_yaw_[mid_right_prep_waypoint_index_];

    r1_wait_x_ = waypoints_x_[r1_wait_waypoint_index_];
    r1_wait_y_ = waypoints_y_[r1_wait_waypoint_index_];
    r1_wait_yaw_ = waypoints_yaw_[r1_wait_waypoint_index_];

    high_x_ = waypoints_x_[high_place_waypoint_index_];
    high_y_ = waypoints_y_[high_place_waypoint_index_];
    high_yaw_ = waypoints_yaw_[high_place_waypoint_index_];
  }

  void printWaypointsInfo()
  {
    RCLCPP_INFO(get_logger(), "========== R2 Zone3 路点 ==========");

    for (size_t i = 0; i < waypoints_x_.size(); ++i)
    {
      std::string name = "wp" + std::to_string(i);
      if (i < waypoint_names_.size())
      {
        name = waypoint_names_[i];
      }

      RCLCPP_INFO(
        get_logger(),
        "[%zu] %s: x=%.3f y=%.3f yaw=%.3f",
        i,
        name.c_str(),
        waypoints_x_[i],
        waypoints_y_[i],
        waypoints_yaw_[i]
      );
    }

    RCLCPP_INFO(
      get_logger(),
      "ramp=%d face_grid=%d mid_left_prep=%d mid_right_prep=%d r1_wait=%d high=%d",
      ramp_up_waypoint_index_,
      face_grid_waypoint_index_,
      mid_left_prep_waypoint_index_,
      mid_right_prep_waypoint_index_,
      r1_wait_waypoint_index_,
      high_place_waypoint_index_
    );

    RCLCPP_INFO(get_logger(), "==================================");
  }

  void resetRuntimeFlags()
  {
    vision_valid_ = false;
    vision_dx_ = 0.0;
    vision_dy_ = 0.0;
    vision_yaw_ = 0.0;
    vision_stable_count_ = 0;

    bottom_grid_result_valid_ = false;
    bottom_grid_state_ = {0, 0, 0};
    selected_bottom_grid_index_ = -1;
    selected_mid_grid_index_ = -1;

    selected_mid_prep_x_ = 0.0;
    selected_mid_prep_y_ = 0.0;
    selected_mid_prep_yaw_ = 0.0;
    selected_mid_prep_name_ = "mid_unknown_prep_point";

    mid_place_done_ = false;
    high_place_done_ = false;
    r2_on_r1_ = false;
    r1_move_done_ = false;

    can_01_sent_ = false;
    can_03_sent_ = false;

    mid_place_cmd_sent_ = false;
    high_place_cmd_sent_ = false;

    nav_goal_sent_in_state_ = false;

    setVisionPriority(false);
    stopChassis();
  }

  static double clamp(double v, double lim)
  {
    return std::clamp(v, -std::abs(lim), std::abs(lim));
  }

  static double normalizeAngle(double a)
  {
    return std::atan2(std::sin(a), std::cos(a));
  }

  static double quaternionToYaw(double x, double y, double z, double w)
  {
    return std::atan2(
      2.0 * (w * z + x * y),
      1.0 - 2.0 * (y * y + z * z)
    );
  }

  static void yawToQuaternion(
    double yaw,
    double & x,
    double & y,
    double & z,
    double & w)
  {
    x = 0.0;
    y = 0.0;
    z = std::sin(yaw / 2.0);
    w = std::cos(yaw / 2.0);
  }

  static std::string trim(const std::string & s)
  {
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos)
    {
      return "";
    }

    const auto e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
  }

  static int colorStringToValue(std::string s)
  {
    s = trim(s);
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);

    if (s == "r" || s == "red")
    {
      return 1;
    }

    if (s == "b" || s == "blue")
    {
      return 2;
    }

    return 0;
  }

  void normalizeSelfColor()
  {
    std::transform(self_color_.begin(), self_color_.end(), self_color_.begin(), ::tolower);

    if (self_color_ == "red")
    {
      self_color_ = "r";
    }

    if (self_color_ == "blue")
    {
      self_color_ = "b";
    }

    if (self_color_ != "r" && self_color_ != "b")
    {
      RCLCPP_WARN(get_logger(), "self_color 非法，默认使用 r");
      self_color_ = "r";
    }
  }

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mid_place_done_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr high_place_done_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr r2_on_r1_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr r1_move_done_sub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr vision_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr bottom_grid_result_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr vision_priority_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr target_mid_grid_index_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mid_place_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr high_place_cmd_pub_;

  rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr can_tx_pub_;

  rclcpp::Client<lite_task_controller::srv::SetAbsoluteGoal>::SharedPtr absolute_goal_client_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string start_signal_topic_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string goal_pose_topic_;
  std::string absolute_goal_service_;

  std::string vision_priority_topic_;
  std::string vision_offset_topic_;

  std::string bottom_grid_result_topic_;
  std::string target_mid_grid_index_topic_;

  std::string mid_place_cmd_topic_;
  std::string mid_place_done_topic_;

  std::string high_place_cmd_topic_;
  std::string high_place_done_topic_;

  std::string r2_on_r1_topic_;
  std::string r1_move_done_topic_;

  std::string can_tx_topic_;

  std::string self_color_;
  std::string bottom_kfs_select_strategy_;

  bool use_lite_absolute_goal_{true};
  bool nav_goal_sent_in_state_{false};

  double control_rate_{50.0};

  double arrive_xy_threshold_{0.08};
  double arrive_yaw_threshold_{0.08};
  double nav_execution_timeout_{30.0};

  std::vector<double> waypoints_x_;
  std::vector<double> waypoints_y_;
  std::vector<double> waypoints_yaw_;
  std::vector<std::string> waypoint_names_;

  int ramp_up_waypoint_index_{0};
  int face_grid_waypoint_index_{1};
  int mid_left_prep_waypoint_index_{2};
  int mid_right_prep_waypoint_index_{3};
  int r1_wait_waypoint_index_{4};
  int high_place_waypoint_index_{5};

  double wait_bottom_grid_result_timeout_sec_{2.0};

  double vision_mid_dx_threshold_{0.025};
  double vision_mid_dy_threshold_{0.025};
  double vision_mid_yaw_threshold_{0.05};

  double vision_lift_dx_threshold_{0.020};
  double vision_lift_dy_threshold_{0.030};
  double vision_lift_yaw_threshold_{0.04};

  double vision_high_dx_threshold_{0.025};
  double vision_high_dy_threshold_{0.025};
  double vision_high_yaw_threshold_{0.05};

  int vision_stable_count_required_{5};

  double max_vx_{0.25};
  double max_vy_{0.25};
  double max_wz_{0.6};

  double vision_kp_x_{1.0};
  double vision_kp_y_{1.0};
  double vision_kp_yaw_{1.5};

  uint32_t can_id_observe_bottom_{0x88};
  uint32_t can_id_climb_{0x66};

  uint8_t can_cmd_observe_bottom_{0x02};
  uint8_t can_cmd_climb_{0x03};

  double wait_r2_on_r1_timeout_sec_{10.0};
  double wait_r1_move_done_timeout_sec_{20.0};
  double place_action_timeout_sec_{10.0};

  double current_x_{0.0};
  double current_y_{0.0};
  double current_yaw_{0.0};
  bool pose_valid_{false};

  double ramp_x_{0.0};
  double ramp_y_{0.0};
  double ramp_yaw_{0.0};

  double face_grid_x_{0.0};
  double face_grid_y_{0.0};
  double face_grid_yaw_{0.0};

  double mid_left_prep_x_{0.0};
  double mid_left_prep_y_{0.0};
  double mid_left_prep_yaw_{0.0};

  double mid_right_prep_x_{0.0};
  double mid_right_prep_y_{0.0};
  double mid_right_prep_yaw_{0.0};

  double selected_mid_prep_x_{0.0};
  double selected_mid_prep_y_{0.0};
  double selected_mid_prep_yaw_{0.0};
  std::string selected_mid_prep_name_{"mid_unknown_prep_point"};

  double r1_wait_x_{0.0};
  double r1_wait_y_{0.0};
  double r1_wait_yaw_{0.0};

  double high_x_{0.0};
  double high_y_{0.0};
  double high_yaw_{0.0};

  bool vision_valid_{false};
  double vision_dx_{0.0};
  double vision_dy_{0.0};
  double vision_yaw_{0.0};
  int vision_stable_count_{0};

  bool bottom_grid_result_valid_{false};
  std::vector<int> bottom_grid_state_{0, 0, 0};

  int selected_bottom_grid_index_{-1};
  int selected_mid_grid_index_{-1};

  bool mid_place_done_{false};
  bool high_place_done_{false};
  bool r2_on_r1_{false};
  bool r1_move_done_{false};

  bool can_01_sent_{false};
  bool can_03_sent_{false};

  bool mid_place_cmd_sent_{false};
  bool high_place_cmd_sent_{false};

  R2Zone3State state_{R2Zone3State::IDLE};
  rclcpp::Time state_start_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<R2Zone3Node>());
  rclcpp::shutdown();
  return 0;
}

