#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <unordered_map>
#include <set>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <chrono>


class R2DecisionNode : public rclcpp::Node
{
public:
  R2DecisionNode()
  : Node("r2_decision_node")
  {
    // =========================
    // 参数
    // =========================

    declare_parameter<std::string>("detected_id_type_topic", "/detected_id_type");

    declare_parameter<std::string>("planner_input_id_type_topic", "/id_type");

    declare_parameter<std::string>("planner_path_topic", "/r2_planned_path_raw");
    declare_parameter<std::string>("planner_action_topic", "/r2_action_sequence_raw");

    declare_parameter<std::string>("controller_path_topic", "/r2_planned_path");
    declare_parameter<std::string>("controller_action_topic", "/r2_action_sequence");

    declare_parameter<int>("total_blocks", 12);
    declare_parameter<int>("target_count", 2);

    declare_parameter<int>("type_empty", 0);
    declare_parameter<int>("type_r1", 1);
    declare_parameter<int>("type_r2", 2);
    declare_parameter<int>("type_fake", 3);

    declare_parameter<bool>("wait_all_blocks_before_plan", true);
    declare_parameter<bool>("auto_send_path_to_controller", true);
    declare_parameter<bool>("send_path_once", true);

    declare_parameter<bool>("debug_log", true);

    loadParameters();

    // =========================
    // 订阅识别结果
    // =========================

    detected_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      detected_id_type_topic_,
      10,
      std::bind(&R2DecisionNode::detectedCallback, this, std::placeholders::_1)
    );

    // =========================
    // 订阅规划器结果
    // =========================

    planner_path_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      planner_path_topic_,
      10,
      std::bind(&R2DecisionNode::plannerPathCallback, this, std::placeholders::_1)
    );

    planner_action_sub_ = create_subscription<std_msgs::msg::String>(
      planner_action_topic_,
      10,
      std::bind(&R2DecisionNode::plannerActionCallback, this, std::placeholders::_1)
    );

    // =========================
    // 发布给规划器
    // =========================

    planner_id_type_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>(
      planner_input_id_type_topic_,
      10
    );

    // =========================
    // 发布给控制器
    // =========================

    controller_path_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
      controller_path_topic_,
      10
    );

    controller_action_pub_ = create_publisher<std_msgs::msg::String>(
      controller_action_topic_,
      10
    );

    RCLCPP_INFO(get_logger(), "R2 decision node started.");
    RCLCPP_INFO(get_logger(), "Detected input topic: %s", detected_id_type_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Planner input topic: %s", planner_input_id_type_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Planner path topic: %s", planner_path_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Controller path topic: %s", controller_path_topic_.c_str());
  }

private:
  void loadParameters()
  {
    detected_id_type_topic_ = get_parameter("detected_id_type_topic").as_string();

    planner_input_id_type_topic_ = get_parameter("planner_input_id_type_topic").as_string();

    planner_path_topic_ = get_parameter("planner_path_topic").as_string();
    planner_action_topic_ = get_parameter("planner_action_topic").as_string();

    controller_path_topic_ = get_parameter("controller_path_topic").as_string();
    controller_action_topic_ = get_parameter("controller_action_topic").as_string();

    total_blocks_ = get_parameter("total_blocks").as_int();
    target_count_ = get_parameter("target_count").as_int();

    type_empty_ = get_parameter("type_empty").as_int();
    type_r1_ = get_parameter("type_r1").as_int();
    type_r2_ = get_parameter("type_r2").as_int();
    type_fake_ = get_parameter("type_fake").as_int();

    wait_all_blocks_before_plan_ = get_parameter("wait_all_blocks_before_plan").as_bool();
    auto_send_path_to_controller_ = get_parameter("auto_send_path_to_controller").as_bool();
    send_path_once_ = get_parameter("send_path_once").as_bool();

    debug_log_ = get_parameter("debug_log").as_bool();
  }

  // ==========================================================
  // 接收识别节点输出
  //
  // 输入格式：
  //   msg.data[0] = block_id
  //   msg.data[1] = type
  //
  // type:
  //   0 = empty
  //   1 = R1 KFS
  //   2 = R2 KFS
  //   3 = fake
  // ==========================================================
  void detectedCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) {
      RCLCPP_WARN(get_logger(), "Invalid detected msg, size < 2");
      return;
    }

    int block_id = static_cast<int>(msg->data[0]);
    int type = static_cast<int>(msg->data[1]);

    if (block_id < 1 || block_id > total_blocks_) {
      RCLCPP_WARN(get_logger(), "Invalid block id: %d", block_id);
      return;
    }

    if (!isValidType(type)) {
      RCLCPP_WARN(get_logger(), "Invalid block type: block=%d, type=%d", block_id, type);
      return;
    }

    block_types_[block_id] = type;
    received_blocks_.insert(block_id);

    if (debug_log_) {
      RCLCPP_INFO(
        get_logger(),
        "Decision received detection: block=%d, type=%d, received=%ld/%d",
        block_id,
        type,
        received_blocks_.size(),
        total_blocks_
      );
    }

    if (shouldSendToPlanner()) {
      publishIdTypeToPlanner(block_id, type);
    } else {
      if (debug_log_) {
        RCLCPP_INFO(
          get_logger(),
          "Wait for more blocks before planning: received=%ld/%d",
          received_blocks_.size(),
          total_blocks_
        );
      }
    }
  }

  bool isValidType(int type) const
  {
    return type == type_empty_ ||
           type == type_r1_ ||
           type == type_r2_ ||
           type == type_fake_;
  }

  bool shouldSendToPlanner() const
  {
    if (!wait_all_blocks_before_plan_) {
      return true;
    }

    return static_cast<int>(received_blocks_.size()) >= total_blocks_;
  }

  // ==========================================================
  // 发布识别结果给 Dijkstra 规划器
  //
  // 这里有两种策略：
  // 1. 不等待全部方块：
  //    来一个发一个，规划器会不断更新路径
  //
  // 2. 等待全部方块：
  //    收齐 12 个后，把所有方块类型一次性全部发给规划器
  // ==========================================================
  void publishIdTypeToPlanner(int latest_block_id, int latest_type)
{
  if (!wait_all_blocks_before_plan_) {
    std_msgs::msg::UInt8MultiArray out;
    out.data.push_back(static_cast<uint8_t>(latest_block_id));
    out.data.push_back(static_cast<uint8_t>(latest_type));

    planner_id_type_pub_->publish(out);

    if (debug_log_) {
      RCLCPP_INFO(
        get_logger(),
        "Send one id_type to planner: block=%d, type=%d",
        latest_block_id,
        latest_type
      );
    }

    return;
  }

  // 如果等待全部方块，则收齐后只发送一次整套数据
  if (sent_all_id_type_to_planner_) {
    return;
  }

  RCLCPP_WARN(get_logger(), "All block types received. Send all id_type to planner.");

  for (int repeat = 0; repeat < 3; ++repeat) {
    for (int block_id = 1; block_id <= total_blocks_; ++block_id) {
      auto it = block_types_.find(block_id);

      int type = type_empty_;
      if (it != block_types_.end()) {
        type = it->second;
      }

      std_msgs::msg::UInt8MultiArray out;
      out.data.push_back(static_cast<uint8_t>(block_id));
      out.data.push_back(static_cast<uint8_t>(type));

      planner_id_type_pub_->publish(out);

      RCLCPP_INFO(
        get_logger(),
        "Send id_type to planner: block=%d, type=%d, repeat=%d",
        block_id,
        type,
        repeat + 1
      );

      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }

    rclcpp::sleep_for(std::chrono::milliseconds(50));
  }

  sent_all_id_type_to_planner_ = true;
}

  // ==========================================================
  // 接收规划器路径
  //
  // 输入：
  //   /r2_planned_path_raw
  //
  // 输出：
  //   /r2_planned_path
  //
  // LiteTaskController 会订阅 /r2_planned_path 并自动执行。
  // ==========================================================
  void plannerPathCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    if (msg->data.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty planned path from planner.");
      return;
    }

    latest_path_ = *msg;
    has_path_ = true;

    std::ostringstream oss;
    oss << "Decision received planned path: ";
    for (int b : msg->data) {
      oss << b << " ";
    }

    RCLCPP_WARN(get_logger(), "%s", oss.str().c_str());

    if (!auto_send_path_to_controller_) {
      RCLCPP_INFO(get_logger(), "auto_send_path_to_controller=false, path not sent.");
      return;
    }

    if (send_path_once_ && path_sent_to_controller_) {
      RCLCPP_WARN(get_logger(), "Path already sent once. Ignore new path because send_path_once=true.");
      return;
    }

    sendPathToController();
  }

  // ==========================================================
  // 接收规划器动作序列
  // ==========================================================
  void plannerActionCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    latest_action_ = *msg;
    has_action_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Decision received action sequence:\n%s",
      msg->data.c_str()
    );

    if (auto_send_path_to_controller_) {
      controller_action_pub_->publish(*msg);
    }
  }

  // ==========================================================
  // 发送路径给任务控制器
  // ==========================================================
  void sendPathToController()
  {
    if (!has_path_) {
      RCLCPP_WARN(get_logger(), "No path to send.");
      return;
    }

    controller_path_pub_->publish(latest_path_);
    path_sent_to_controller_ = true;

    std::ostringstream oss;
    oss << "Send path to controller: ";
    for (int b : latest_path_.data) {
      oss << b << " ";
    }

    RCLCPP_WARN(get_logger(), "%s", oss.str().c_str());
  }

private:
  // Topic
  std::string detected_id_type_topic_;

  std::string planner_input_id_type_topic_;

  std::string planner_path_topic_;
  std::string planner_action_topic_;

  std::string controller_path_topic_;
  std::string controller_action_topic_;

  // 参数
  int total_blocks_{12};
  int target_count_{2};

  int type_empty_{0};
  int type_r1_{1};
  int type_r2_{2};
  int type_fake_{3};

  bool wait_all_blocks_before_plan_{true};
  bool auto_send_path_to_controller_{true};
  bool send_path_once_{true};

  bool debug_log_{true};

  // 状态
  std::unordered_map<int, int> block_types_;
  std::set<int> received_blocks_;

  bool sent_all_id_type_to_planner_{false};

  bool has_path_{false};
  bool has_action_{false};
  bool path_sent_to_controller_{false};

  std_msgs::msg::Int32MultiArray latest_path_;
  std_msgs::msg::String latest_action_;

  // ROS
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr detected_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr planner_path_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_action_sub_;

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr planner_id_type_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr controller_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_action_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<R2DecisionNode>());
  rclcpp::shutdown();
  return 0;
}

