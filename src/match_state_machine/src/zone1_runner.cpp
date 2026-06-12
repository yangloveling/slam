#include <chrono>
#include <string>
#include <memory>
#include <cmath>
#include <vector>
#include <functional>
#include <regex>
#include <stdexcept>
#include <cstring>
#include <cerrno>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "lite_task_controller/srv/set_absolute_goal.hpp"

using namespace std::chrono_literals;

class Zone1Runner : public rclcpp::Node
{
public:
    Zone1Runner()
        : Node("zone1_runner"),
          state_enter_time_(0, 0, RCL_SYSTEM_TIME),
          run_start_time_(0, 0, RCL_SYSTEM_TIME),
          nav_retry_delay_start_(0, 0, RCL_SYSTEM_TIME),
          pre_vision_can03_sent_time_(0, 0, RCL_SYSTEM_TIME),
          post_vision_wait_start_time_(0, 0, RCL_SYSTEM_TIME),
          weapon_done_wait_start_time_(0, 0, RCL_SYSTEM_TIME)
    {
        declareAndLoadParameters();
        initRosInterfaces();
        initCanInterface();

        control_timer_ = create_wall_timer(
            100ms,
            std::bind(&Zone1Runner::controlLoop, this));

        current_state_ = State::WAIT_FOR_SYSTEM_READY;
        state_enter_time_ = now();

        RCLCPP_WARN(
            get_logger(),
            "Zone1Runner started. waypoint_count=%zu, grasp_index=%d, assemble_index=%d, camera_return_index=%d",
            waypoints_.size(),
            weapon_grasp_vision_waypoint_index_,
            weapon_assemble_waypoint_index_,
            camera_return_waypoint_index_);
    }

    ~Zone1Runner() override
    {
        if (can_socket_ >= 0) {
            close(can_socket_);
            can_socket_ = -1;
        }
    }

private:
    enum class State
    {
        WAIT_FOR_SYSTEM_READY,
        IDLE,
        NAVIGATING,
        VISION,
        WAIT_WEAPON_GRASP_DONE,
        WAIT_WEAPON_ASSEMBLE_DONE,
        DONE,
        ERROR
    };

    enum class ServiceCallState
    {
        IDLE,
        WAITING_FOR_RESPONSE,
        SUCCEEDED,
        FAILED,
        RETRY_DELAY
    };

    struct Waypoint
    {
        geometry_msgs::msg::Pose pose;
        std::string name;
    };

private:
    // =========================
    // ROS interfaces
    // =========================
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_signal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vision_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr weapon_done_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_start_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_priority_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vision_reset_cmd_pub_;

    rclcpp::Client<lite_task_controller::srv::SetAbsoluteGoal>::SharedPtr absolute_goal_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    // =========================
    // Parameters
    // =========================
    std::string start_signal_topic_;
    std::string odom_topic_;
    std::string cmd_vel_topic_;
    std::string set_absolute_goal_service_;

    std::string vision_start_topic_;
    std::string vision_priority_topic_;
    std::string vision_cmd_topic_;

    std::string vision_reset_cmd_topic_;
    std::string vision_reset_cmd_value_;

    std::string weapon_done_topic_;

    std::string can_interface_;
    bool enable_can_weapon_cmd_{true};

    double can_command_gap_sec_{1.00};
    double pre_vision_reset_delay_sec_{5.0};

    bool enable_camera_return_at_waypoint_{true};
    int camera_return_waypoint_index_{4};

    double nav_pos_tolerance_{0.20};
    double nav_yaw_tolerance_{1.0};
    double vision_fine_tune_pos_tolerance_{0.015};
    double vision_fine_tune_release_tolerance_{0.025};

    double arm_base_x_m_{0.0};
    double arm_base_y_m_{0.0};

    double post_vision_wait_sec_{1.0};

    int max_nav_retries_{3};

    int weapon_grasp_vision_waypoint_index_{1};
    int weapon_assemble_waypoint_index_{3};

    std::vector<double> waypoints_x_;
    std::vector<double> waypoints_y_;
    std::vector<double> waypoints_yaw_;
    std::vector<std::string> waypoint_names_;

    std::vector<Waypoint> waypoints_;

    // =========================
    // Runtime
    // =========================
    State current_state_{State::WAIT_FOR_SYSTEM_READY};
    ServiceCallState service_call_state_{ServiceCallState::IDLE};

    nav_msgs::msg::Odometry current_odom_;

    bool odom_received_{false};
    bool start_signal_received_{false};
    bool system_ready_{false};

    bool goal_sent_{false};
    int nav_retry_count_{0};
    size_t current_waypoint_index_{0};

    uint64_t current_req_id_{0};

    rclcpp::Time state_enter_time_;
    rclcpp::Time run_start_time_;
    rclcpp::Time nav_retry_delay_start_;
    rclcpp::Time pre_vision_can03_sent_time_;

    bool done_logged_{false};
    bool error_logged_{false};

    // =========================
    // Vision runtime
    // =========================
    bool vision_started_{false};
    bool vision_cmd_received_{false};

    // 两次 reset / 两次识别
    int vision_reset_count_{0};
    int vision_cmd_count_{0};

    double vision_first_x_{0.0};
    double vision_second_z_{0.0};

    double vision_cmd_x_{0.0};
    double vision_cmd_z_{0.0};

    bool vision_x_move_done_{false};

    bool vision_goal_sent_{false};
    bool vision_goal_accepted_{false};
    bool vision_goal_reached_{false};
    geometry_msgs::msg::Pose vision_goal_pose_;

    bool post_vision_waiting_{false};
    rclcpp::Time post_vision_wait_start_time_;

    // =========================
    // Weapon runtime
    // =========================
    bool weapon_done_received_{false};
    rclcpp::Time weapon_done_wait_start_time_;

    // =========================
    // CAN runtime
    // =========================
    int can_socket_{-1};

private:
    // =========================
    // Parameters
    // =========================
    void declareAndLoadParameters()
    {
        declare_parameter<std::string>("start_signal_topic", "/start_signal");
        declare_parameter<std::string>("odom_topic", "/odometry/filtered");
        declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        declare_parameter<std::string>("set_absolute_goal_service", "/set_absolute_goal");

        declare_parameter<std::string>("vision_start_topic", "/vision_start");
        declare_parameter<std::string>("vision_priority_topic", "/r1/vision_priority");
        declare_parameter<std::string>("vision_cmd_topic", "/vision/Weaponhead_Info");

        declare_parameter<std::string>("vision_reset_cmd_topic", "/vision/recognition_reset_cmd");
        declare_parameter<std::string>("vision_reset_cmd_value", "reset_Weaponhead");

        declare_parameter<std::string>("weapon_done_topic", "/r1/weapon_done");

        declare_parameter<std::string>("can_interface", "can0");
        declare_parameter<bool>("enable_can_weapon_cmd", true);

        declare_parameter<double>("can_command_gap_sec", 1.00);
        declare_parameter<double>("pre_vision_reset_delay_sec", 5.0);

        declare_parameter<bool>("enable_camera_return_at_waypoint", true);
        declare_parameter<int>("camera_return_waypoint_index", 4);

        declare_parameter<double>("nav_pos_tolerance", 0.20);
        declare_parameter<double>("nav_yaw_tolerance", 1.0);
        declare_parameter<double>("vision_fine_tune_pos_tolerance", 0.015);
        declare_parameter<double>("vision_fine_tune_release_tolerance", 0.025);

        declare_parameter<double>("arm_base_x_m", 0.0);
        declare_parameter<double>("arm_base_y_m", 0.0);

        declare_parameter<double>("post_vision_wait_sec", 1.0);

        declare_parameter<int>("max_nav_retries", 3);

        declare_parameter<int>("weapon_grasp_vision_waypoint_index", 1);
        declare_parameter<int>("weapon_assemble_waypoint_index", 3);

        declare_parameter<std::vector<double>>("waypoints_x", std::vector<double>{});
        declare_parameter<std::vector<double>>("waypoints_y", std::vector<double>{});
        declare_parameter<std::vector<double>>("waypoints_yaw", std::vector<double>{});
        declare_parameter<std::vector<std::string>>("waypoint_names", std::vector<std::string>{});

        get_parameter("start_signal_topic", start_signal_topic_);
        get_parameter("odom_topic", odom_topic_);
        get_parameter("cmd_vel_topic", cmd_vel_topic_);
        get_parameter("set_absolute_goal_service", set_absolute_goal_service_);

        get_parameter("vision_start_topic", vision_start_topic_);
        get_parameter("vision_priority_topic", vision_priority_topic_);
        get_parameter("vision_cmd_topic", vision_cmd_topic_);

        get_parameter("vision_reset_cmd_topic", vision_reset_cmd_topic_);
        get_parameter("vision_reset_cmd_value", vision_reset_cmd_value_);

        get_parameter("weapon_done_topic", weapon_done_topic_);

        get_parameter("can_interface", can_interface_);
        get_parameter("enable_can_weapon_cmd", enable_can_weapon_cmd_);

        get_parameter("can_command_gap_sec", can_command_gap_sec_);
        get_parameter("pre_vision_reset_delay_sec", pre_vision_reset_delay_sec_);

        get_parameter("enable_camera_return_at_waypoint", enable_camera_return_at_waypoint_);
        get_parameter("camera_return_waypoint_index", camera_return_waypoint_index_);

        get_parameter("nav_pos_tolerance", nav_pos_tolerance_);
        get_parameter("nav_yaw_tolerance", nav_yaw_tolerance_);
        get_parameter("vision_fine_tune_pos_tolerance", vision_fine_tune_pos_tolerance_);
        get_parameter("vision_fine_tune_release_tolerance", vision_fine_tune_release_tolerance_);

        get_parameter("arm_base_x_m", arm_base_x_m_);
        get_parameter("arm_base_y_m", arm_base_y_m_);

        get_parameter("post_vision_wait_sec", post_vision_wait_sec_);

        get_parameter("max_nav_retries", max_nav_retries_);

        get_parameter("weapon_grasp_vision_waypoint_index", weapon_grasp_vision_waypoint_index_);
        get_parameter("weapon_assemble_waypoint_index", weapon_assemble_waypoint_index_);

        get_parameter("waypoints_x", waypoints_x_);
        get_parameter("waypoints_y", waypoints_y_);
        get_parameter("waypoints_yaw", waypoints_yaw_);
        get_parameter("waypoint_names", waypoint_names_);

        if (waypoints_x_.empty() || waypoints_y_.empty() || waypoints_yaw_.empty()) {
            throw std::runtime_error("Waypoint arrays cannot be empty");
        }

        if (waypoints_x_.size() != waypoints_y_.size() ||
            waypoints_x_.size() != waypoints_yaw_.size()) {
            throw std::runtime_error("Waypoint array sizes mismatch");
        }

        if (weapon_grasp_vision_waypoint_index_ < 0 ||
            static_cast<size_t>(weapon_grasp_vision_waypoint_index_) >= waypoints_x_.size()) {
            throw std::runtime_error("weapon_grasp_vision_waypoint_index out of range");
        }

        if (weapon_assemble_waypoint_index_ < 0 ||
            static_cast<size_t>(weapon_assemble_waypoint_index_) >= waypoints_x_.size()) {
            throw std::runtime_error("weapon_assemble_waypoint_index out of range");
        }

        if (enable_camera_return_at_waypoint_) {
            if (camera_return_waypoint_index_ < 0 ||
                static_cast<size_t>(camera_return_waypoint_index_) >= waypoints_x_.size()) {
                throw std::runtime_error("camera_return_waypoint_index out of range");
            }
        }

        if (can_command_gap_sec_ < 0.0) {
            can_command_gap_sec_ = 0.0;
        }

        if (pre_vision_reset_delay_sec_ < 0.0) {
            pre_vision_reset_delay_sec_ = 0.0;
        }
        vision_fine_tune_pos_tolerance_ =
            std::max(0.001, vision_fine_tune_pos_tolerance_);
        vision_fine_tune_release_tolerance_ =
            std::max(vision_fine_tune_pos_tolerance_, vision_fine_tune_release_tolerance_);

        waypoints_.clear();
        waypoints_.reserve(waypoints_x_.size());

        for (size_t i = 0; i < waypoints_x_.size(); ++i) {
            Waypoint wp;

            wp.pose.position.x = waypoints_x_[i];
            wp.pose.position.y = waypoints_y_[i];
            wp.pose.position.z = 0.0;

            setPoseYaw(wp.pose, waypoints_yaw_[i]);

            if (i < waypoint_names_.size()) {
                wp.name = waypoint_names_[i];
            } else {
                wp.name = "wp_" + std::to_string(i);
            }

            waypoints_.push_back(wp);
        }
    }

    // =========================
    // ROS init
    // =========================
    void initRosInterfaces()
    {
        start_signal_sub_ = create_subscription<std_msgs::msg::Bool>(
            start_signal_topic_,
            10,
            std::bind(&Zone1Runner::startSignalCallback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_,
            20,
            std::bind(&Zone1Runner::odomCallback, this, std::placeholders::_1));

        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            cmd_vel_topic_,
            10);

        absolute_goal_client_ = create_client<lite_task_controller::srv::SetAbsoluteGoal>(
            set_absolute_goal_service_);

        vision_start_pub_ = create_publisher<std_msgs::msg::Bool>(
            vision_start_topic_,
            10);

        vision_priority_pub_ = create_publisher<std_msgs::msg::Bool>(
            vision_priority_topic_,
            10);

        vision_reset_cmd_pub_ = create_publisher<std_msgs::msg::String>(
            vision_reset_cmd_topic_,
            10);

        rclcpp::QoS vision_qos =
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

        vision_cmd_sub_ = create_subscription<std_msgs::msg::String>(
            vision_cmd_topic_,
            vision_qos,
            std::bind(&Zone1Runner::visionCmdCallback, this, std::placeholders::_1));

        weapon_done_sub_ = create_subscription<std_msgs::msg::Bool>(
            weapon_done_topic_,
            10,
            std::bind(&Zone1Runner::weaponDoneCallback, this, std::placeholders::_1));

        reset_service_ = create_service<std_srvs::srv::Trigger>(
            "~/reset",
            std::bind(
                &Zone1Runner::resetCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
    }

    // =========================
    // CAN
    // =========================
    void initCanInterface()
    {
        if (!enable_can_weapon_cmd_) {
            RCLCPP_WARN(get_logger(), "CAN command disabled by parameter.");
            return;
        }

        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

        if (can_socket_ < 0) {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to create CAN socket for interface %s: %s",
                can_interface_.c_str(),
                std::strerror(errno));
            return;
        }

        struct ifreq ifr;
        std::memset(&ifr, 0, sizeof(ifr));
        std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", can_interface_.c_str());

        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to get CAN interface index for %s: %s",
                can_interface_.c_str(),
                std::strerror(errno));

            close(can_socket_);
            can_socket_ = -1;
            return;
        }

        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to bind CAN socket to %s: %s",
                can_interface_.c_str(),
                std::strerror(errno));

            close(can_socket_);
            can_socket_ = -1;
            return;
        }

        RCLCPP_WARN(
            get_logger(),
            "CAN command enabled: interface=%s",
            can_interface_.c_str());
    }

    bool sendCanCommand(uint32_t can_id, uint8_t cmd)
    {
        if (!enable_can_weapon_cmd_) {
            RCLCPP_WARN(
                get_logger(),
                "CAN command disabled. Skip CAN 0x%X cmd=0x%02X",
                can_id,
                cmd);
            return true;
        }

        if (can_socket_ < 0) {
            RCLCPP_ERROR(
                get_logger(),
                "CAN socket not available. Failed to send CAN 0x%X cmd=0x%02X on %s",
                can_id,
                cmd,
                can_interface_.c_str());
            return false;
        }

        struct can_frame frame;
        std::memset(&frame, 0, sizeof(frame));

        frame.can_id = can_id;
        frame.can_dlc = 1;
        frame.data[0] = cmd;

        const ssize_t nbytes = write(can_socket_, &frame, sizeof(frame));

        if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to send CAN frame: interface=%s, id=0x%X, data=0x%02X, nbytes=%zd, error=%s",
                can_interface_.c_str(),
                can_id,
                cmd,
                nbytes,
                std::strerror(errno));
            return false;
        }

        RCLCPP_WARN(
            get_logger(),
            "CAN CMD sent: interface=%s, id=0x%X, data=0x%02X",
            can_interface_.c_str(),
            can_id,
            cmd);

        return true;
    }

    bool sendCanCommand0x66(uint8_t cmd)
    {
        return sendCanCommand(0x66, cmd);
    }

    bool sendCanCommand0x77(uint8_t cmd)
    {
        return sendCanCommand(0x77, cmd);
    }

    bool sleepBetweenCanCommands()
    {
        if (can_command_gap_sec_ <= 0.0) {
            return true;
        }

        rclcpp::sleep_for(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(can_command_gap_sec_)));

        return rclcpp::ok();
    }

    // =========================
    // Callbacks
    // =========================
    void startSignalCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data) {
            return;
        }

        if (current_state_ != State::IDLE &&
            current_state_ != State::DONE &&
            current_state_ != State::ERROR) {
            RCLCPP_WARN(
                get_logger(),
                "Start signal ignored because current state is %s",
                stateToString(current_state_).c_str());
            return;
        }

        start_signal_received_ = true;
        run_start_time_ = now();

        RCLCPP_WARN(get_logger(), "Start signal received.");
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = *msg;
        odom_received_ = true;
    }

    void weaponDoneCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data) {
            return;
        }

        if (current_state_ != State::WAIT_WEAPON_GRASP_DONE &&
            current_state_ != State::WAIT_WEAPON_ASSEMBLE_DONE) {
            RCLCPP_WARN(
                get_logger(),
                "Weapon done true ignored because current state is %s",
                stateToString(current_state_).c_str());
            return;
        }

        weapon_done_received_ = true;

        RCLCPP_WARN(
            get_logger(),
            "Weapon done true accepted in state %s",
            stateToString(current_state_).c_str());
    }

    void visionCmdCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (current_state_ != State::VISION) {
            return;
        }

        if (static_cast<int>(current_waypoint_index_) != weapon_grasp_vision_waypoint_index_) {
            RCLCPP_WARN(
                get_logger(),
                "Vision message ignored at waypoint index %zu. Only waypoint index %d is allowed.",
                current_waypoint_index_,
                weapon_grasp_vision_waypoint_index_);
            return;
        }

        if (msg->data.empty()) {
            RCLCPP_WARN(get_logger(), "Received empty vision command, ignored.");
            return;
        }

        const std::string data = msg->data;
        std::smatch match;

        // 支持视觉程序格式：
        // "x:+0.1234,z:+0.5678"
        // "x:0.123,z:0.567"
        // "x=0.123,z=0.567"
        std::regex pattern_xz(
            R"([xX]\s*[:=]\s*([-+]?\d*\.?\d+)\s*,?\s*[zZ]\s*[:=]\s*([-+]?\d*\.?\d+))",
            std::regex::icase);

        // 兼容 x,y 格式
        std::regex pattern_xy(
            R"([xX]\s*[:=]\s*([-+]?\d*\.?\d+).*?[yY]\s*[:=]\s*([-+]?\d*\.?\d+))",
            std::regex::icase);

        // 兼容两个数字格式
        std::regex pattern_two_numbers(
            R"(^\s*([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\s*$)",
            std::regex::icase);

        // 兼容旧格式：Z JX 0.123 0.456
        std::regex pattern_zjx(
            R"(Z\s+[JX]\s+([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+))",
            std::regex::icase);

        bool parsed = false;

        if (std::regex_search(data, match, pattern_xz)) {
            vision_cmd_x_ = std::stod(match[1]);
            vision_cmd_z_ = std::stod(match[2]);
            parsed = true;
        } else if (std::regex_search(data, match, pattern_xy)) {
            vision_cmd_x_ = std::stod(match[1]);
            vision_cmd_z_ = std::stod(match[2]);
            parsed = true;
        } else if (std::regex_search(data, match, pattern_two_numbers)) {
            vision_cmd_x_ = std::stod(match[1]);
            vision_cmd_z_ = std::stod(match[2]);
            parsed = true;
        } else if (std::regex_search(data, match, pattern_zjx)) {
            vision_cmd_x_ = std::stod(match[1]);
            vision_cmd_z_ = std::stod(match[2]);
            parsed = true;
        }

        if (parsed) {
            if (vision_reset_count_ == 1 && vision_cmd_count_ == 0) {
                // 第一次 reset 后的结果：只使用 x，先完成 x 方向微调。
                vision_first_x_ = vision_cmd_x_;
                const double ignored_z = vision_cmd_z_;
                vision_cmd_x_ = vision_first_x_;
                vision_cmd_z_ = 0.0;
                vision_cmd_count_ = 1;
                vision_cmd_received_ = true;

                RCLCPP_WARN(
                    get_logger(),
                    "Vision command #1 parsed: raw='%s', use x=%.4f only for first move, ignore z=%.4f",
                    data.c_str(),
                    vision_first_x_,
                    ignored_z);
            } else if (vision_reset_count_ == 2 && vision_cmd_count_ == 1) {
                // 第二次 reset 后的结果：x 已经移动完成，这里只使用 z。
                vision_second_z_ = vision_cmd_z_;

                // 最终用于微调的数据
                vision_cmd_x_ = 0.0;
                vision_cmd_z_ = vision_second_z_;
                vision_cmd_count_ = 2;
                vision_cmd_received_ = true;

                RCLCPP_WARN(
                    get_logger(),
                    "Vision command #2 parsed: raw='%s', use z=%.4f only. Second move offset: x=%.4f, z=%.4f",
                    data.c_str(),
                    vision_second_z_,
                    vision_cmd_x_,
                    vision_cmd_z_);
            } else {
                RCLCPP_WARN(
                    get_logger(),
                    "Vision command ignored outside expected reset phase: reset_count=%d, cmd_count=%d, raw='%s'",
                    vision_reset_count_,
                    vision_cmd_count_,
                    data.c_str());
            }
        } else {
            RCLCPP_WARN(
                get_logger(),
                "Failed to parse vision cmd: '%s'. Expected: 'x:+0.123,z:+0.456' or 'x y'",
                data.c_str());
        }
    }

    void resetCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        RCLCPP_WARN(get_logger(), "Reset requested.");

        publishStop();
        setVisionPriority(false);

        current_waypoint_index_ = 0;
        start_signal_received_ = false;
        goal_sent_ = false;
        service_call_state_ = ServiceCallState::IDLE;
        nav_retry_count_ = 0;
        pre_vision_can03_sent_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);

        resetVisionFlags();

        weapon_done_received_ = false;

        done_logged_ = false;
        error_logged_ = false;

        current_req_id_++;

        switchState(State::IDLE);

        response->success = true;
        response->message = "Reset completed.";
    }

    // =========================
    // Main loop
    // =========================
    void controlLoop()
    {
        if (!system_ready_) {
            if (odom_received_ && absolute_goal_client_->service_is_ready()) {
                system_ready_ = true;

                if (current_state_ == State::WAIT_FOR_SYSTEM_READY) {
                    switchState(State::IDLE);
                }

                RCLCPP_WARN(get_logger(), "System ready.");
            } else {
                RCLCPP_INFO_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    3000,
                    "Waiting for odom and absolute goal service...");
            }

            return;
        }

        switch (current_state_) {
            case State::WAIT_FOR_SYSTEM_READY:
                break;

            case State::IDLE:
                handleIdleState();
                break;

            case State::NAVIGATING:
                handleNavigatingState();
                break;

            case State::VISION:
                handleVisionState();
                break;

            case State::WAIT_WEAPON_GRASP_DONE:
                handleWaitWeaponGraspDoneState();
                break;

            case State::WAIT_WEAPON_ASSEMBLE_DONE:
                handleWaitWeaponAssembleDoneState();
                break;

            case State::DONE:
                handleDoneState();
                break;

            case State::ERROR:
                handleErrorState();
                break;
        }
    }

    void handleIdleState()
    {
        if (start_signal_received_) {
            current_waypoint_index_ = 0;
            nav_retry_count_ = 0;
            goal_sent_ = false;
            service_call_state_ = ServiceCallState::IDLE;
            pre_vision_can03_sent_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);

            resetVisionFlags();
            weapon_done_received_ = false;

            done_logged_ = false;
            error_logged_ = false;

            switchState(State::NAVIGATING);
        }
    }

    void handleNavigatingState()
    {
        if (current_waypoint_index_ >= waypoints_.size()) {
            switchState(State::DONE);
            return;
        }

        const auto &wp = waypoints_[current_waypoint_index_];

        if (service_call_state_ == ServiceCallState::RETRY_DELAY) {
            if (getDurationSeconds(now() - nav_retry_delay_start_) > 0.5) {
                goal_sent_ = false;
                service_call_state_ = ServiceCallState::IDLE;
            }

            return;
        }

        if (!goal_sent_) {
            sendCurrentWaypointGoal();
            return;
        }

        if (service_call_state_ == ServiceCallState::FAILED) {
            RCLCPP_WARN(
                get_logger(),
                "Nav to %s failed, retry %d/%d",
                wp.name.c_str(),
                nav_retry_count_,
                max_nav_retries_);

            if (nav_retry_count_ < max_nav_retries_) {
                nav_retry_count_++;
                service_call_state_ = ServiceCallState::RETRY_DELAY;
                nav_retry_delay_start_ = now();
            } else {
                RCLCPP_ERROR(
                    get_logger(),
                    "Max nav retries exceeded for %s",
                    wp.name.c_str());

                switchState(State::ERROR);
            }

            return;
        }

        if (service_call_state_ == ServiceCallState::SUCCEEDED) {
            if (hasReachedGoal(wp.pose)) {
                RCLCPP_WARN(
                    get_logger(),
                    "Reached waypoint [%zu/%zu] %s",
                    current_waypoint_index_ + 1,
                    waypoints_.size(),
                    wp.name.c_str());

                nav_retry_count_ = 0;

                // ============================================================
                // 第 2 个点：先发送 0x66/0x03 和 0x77/0x06，再进入视觉识别和微调
                // ============================================================
                if (static_cast<int>(current_waypoint_index_) == weapon_grasp_vision_waypoint_index_) {
                    RCLCPP_WARN(
                        get_logger(),
                        "Reached weapon grasp vision waypoint index=%zu. Send CAN 0x77/0x03 and CAN 0x77/0x06 before vision fine-tune.",
                        current_waypoint_index_);

                    const bool camera_back_ok = sendCanCommand0x77(0x03);

                    if (!camera_back_ok) {
                        RCLCPP_ERROR(
                            get_logger(),
                            "Failed to send pre-vision CAN 0x77/0x03 at waypoint index %zu",
                            current_waypoint_index_);

                        switchState(State::ERROR);
                        return;
                    }

                    pre_vision_can03_sent_time_ = now();

                    if (!sleepBetweenCanCommands()) {
                        RCLCPP_ERROR(
                            get_logger(),
                            "Interrupted while waiting between pre-vision CAN 0x77/0x03 and 0x77/0x06.");

                        switchState(State::ERROR);
                        return;
                    }

                    const bool grasp_ok = sendCanCommand0x77(0x06);

                    if (!grasp_ok) {
                        RCLCPP_ERROR(
                            get_logger(),
                            "Failed to send pre-vision CAN 0x77/0x06 at waypoint index %zu",
                            current_waypoint_index_);

                        switchState(State::ERROR);
                        return;
                    }

                    setVisionPriority(true);
                    resetVisionFlags();
                    switchState(State::VISION);
                    return;
                }

                // ============================================================
                // 第 4 个点：等待拼接完成 true，收到 true 后发送 0x07
                // ============================================================
                if (static_cast<int>(current_waypoint_index_) == weapon_assemble_waypoint_index_) {
                    RCLCPP_WARN(
                        get_logger(),
                        "Reached weapon assemble waypoint index=%zu. Waiting /r1/weapon_done true.",
                        current_waypoint_index_);

                    weapon_done_received_ = false;
                    weapon_done_wait_start_time_ = now();

                    switchState(State::WAIT_WEAPON_ASSEMBLE_DONE);
                    return;
                }

                // ============================================================
                // 第 6 个点：相机转回前方，发送 0x04
                // ============================================================
                if (enable_camera_return_at_waypoint_ &&
                    static_cast<int>(current_waypoint_index_) == camera_return_waypoint_index_) {
                    RCLCPP_WARN(
                        get_logger(),
                        "Reached camera return waypoint index=%zu. Send CAN 0x77/0x04 to turn camera front.",
                        current_waypoint_index_);

                    const bool camera_front_ok = sendCanCommand0x77(0x04);

                    if (!camera_front_ok) {
                        RCLCPP_ERROR(
                            get_logger(),
                            "Failed to send camera front CAN 0x77/0x04 at waypoint index %zu",
                            current_waypoint_index_);

                        switchState(State::ERROR);
                        return;
                    }

                    current_waypoint_index_++;
                    goal_sent_ = false;
                    service_call_state_ = ServiceCallState::IDLE;

                    switchState(State::NAVIGATING);
                    return;
                }

                current_waypoint_index_++;
                goal_sent_ = false;
                service_call_state_ = ServiceCallState::IDLE;
            }
        }
    }

    void handleVisionState()
    {
        publishStop();

        if (static_cast<int>(current_waypoint_index_) != weapon_grasp_vision_waypoint_index_) {
            RCLCPP_WARN(
                get_logger(),
                "VISION state entered at waypoint index %zu, but only index %d should use vision. Skip.",
                current_waypoint_index_,
                weapon_grasp_vision_waypoint_index_);

            resetVisionFlags();
            setVisionPriority(false);

            current_waypoint_index_++;
            goal_sent_ = false;
            service_call_state_ = ServiceCallState::IDLE;

            switchState(State::NAVIGATING);
            return;
        }

        if (!vision_started_) {
            std_msgs::msg::Bool start_msg;
            start_msg.data = true;
            vision_start_pub_->publish(start_msg);

            setVisionPriority(true);

            vision_started_ = true;

            RCLCPP_WARN(
                get_logger(),
                "Vision start sent on %s.",
                vision_start_topic_.c_str());

            return;
        }

        // 第一次 reset：在 0x66/0x03 发送 5 秒后识别 x。
        if (vision_reset_count_ == 0) {
            const rclcpp::Time delay_start =
                pre_vision_can03_sent_time_.nanoseconds() > 0 ?
                pre_vision_can03_sent_time_ :
                state_enter_time_;
            const double wait_elapsed = getDurationSeconds(now() - delay_start);

            if (wait_elapsed < pre_vision_reset_delay_sec_) {
                RCLCPP_INFO_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Waiting %.2f s after CAN 0x77/0x03 before vision reset. elapsed=%.2f s",
                    pre_vision_reset_delay_sec_,
                    wait_elapsed);
                return;
            }

            publishVisionResetCommand();

            vision_reset_count_ = 1;

            RCLCPP_WARN(
                get_logger(),
                "Vision reset #1 sent after CAN 0x77/0x03 delay. Waiting first vision command, use x only. Topic: %s",
                vision_cmd_topic_.c_str());

            return;
        }

        // 第一次结果收到后，先完成 x 方向微调；到位后再发送第二次 reset 识别 z。
        if (vision_reset_count_ == 1 && vision_cmd_count_ == 1 && !vision_x_move_done_) {
            if (!vision_cmd_received_) {
                RCLCPP_INFO_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    3000,
                    "Waiting first vision command at waypoint index %zu from topic: %s",
                    current_waypoint_index_,
                    vision_cmd_topic_.c_str());
                return;
            }

            if (!updateVisionFineTuneMotion("first x-direction")) {
                return;
            }

            vision_x_move_done_ = true;
            resetVisionGoalRuntime();
            vision_cmd_received_ = false;

            publishVisionResetCommand();

            vision_reset_count_ = 2;

            RCLCPP_WARN(
                get_logger(),
                "First x-direction fine-tune reached. Vision reset #2 sent. Waiting second vision command, use z only. Topic: %s",
                vision_cmd_topic_.c_str());

            return;
        }

        if (!vision_cmd_received_) {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                3000,
                "Waiting for vision command at waypoint index %zu from topic: %s",
                current_waypoint_index_,
                vision_cmd_topic_.c_str());

            return;
        }

        if (!updateVisionFineTuneMotion("second z-direction")) {
            return;
        }

        if (!post_vision_waiting_) {
            post_vision_waiting_ = true;
            post_vision_wait_start_time_ = now();

            RCLCPP_WARN(
                get_logger(),
                "Vision fine-tune finished. Waiting %.2f s before waiting weapon grasp done true.",
                post_vision_wait_sec_);

            return;
        }

        const double wait_elapsed = getDurationSeconds(now() - post_vision_wait_start_time_);

        if (wait_elapsed >= post_vision_wait_sec_) {
            RCLCPP_WARN(
                get_logger(),
                "Post-vision wait finished. Waiting first true = weapon grasp done.");

            resetVisionFlags();

            weapon_done_received_ = false;
            weapon_done_wait_start_time_ = now();

            switchState(State::WAIT_WEAPON_GRASP_DONE);
            return;
        }
    }

    void handleWaitWeaponGraspDoneState()
    {
        publishStop();
        setVisionPriority(true);

        if (weapon_done_received_) {
            RCLCPP_WARN(
                get_logger(),
                "First true received: weapon grasp done. Send CAN 0x77/0x07 then CAN 0x77/0x20 before next waypoint.");

            weapon_done_received_ = false;
            setVisionPriority(false);

            const bool grasp_done_ok = sendCanCommand0x77(0x07);

            if (!grasp_done_ok) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to send grasp-done CAN 0x77/0x07 at waypoint index %zu",
                    current_waypoint_index_);

                switchState(State::ERROR);
                return;
            }

            if (!sleepBetweenCanCommands()) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Interrupted while waiting between grasp-done CAN 0x07 and 0x20.");

                switchState(State::ERROR);
                return;
            }

            const bool next_step_ok = sendCanCommand0x77(0x20);

            if (!next_step_ok) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to send grasp-done CAN 0x77/0x20 at waypoint index %zu",
                    current_waypoint_index_);

                switchState(State::ERROR);
                return;
            }

            current_waypoint_index_++;
            goal_sent_ = false;
            service_call_state_ = ServiceCallState::IDLE;

            switchState(State::NAVIGATING);
            return;
        }

        const double elapsed = getDurationSeconds(now() - weapon_done_wait_start_time_);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            3000,
            "Waiting first true = weapon grasp done... %.1f s",
            elapsed);
    }

    void handleWaitWeaponAssembleDoneState()
    {
        publishStop();

        if (weapon_done_received_) {
            RCLCPP_WARN(
                get_logger(),
                "Second true received: weapon assemble done. Send CAN 0x77/0x07, then go to next waypoint.");

            weapon_done_received_ = false;

            const bool can_ok = sendCanCommand0x77(0x07);

            if (!can_ok) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to send CAN 0x77/0x07 at waypoint index %zu",
                    current_waypoint_index_);

                switchState(State::ERROR);
                return;
            }

            current_waypoint_index_++;
            goal_sent_ = false;
            service_call_state_ = ServiceCallState::IDLE;

            switchState(State::NAVIGATING);
            return;
        }

        const double elapsed = getDurationSeconds(now() - weapon_done_wait_start_time_);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            3000,
            "Waiting second true = weapon assemble done... %.1f s",
            elapsed);
    }

    void handleDoneState()
    {
        publishStop();
        setVisionPriority(false);

        if (!done_logged_) {
            const double total = getDurationSeconds(now() - run_start_time_);

            RCLCPP_WARN(
                get_logger(),
                "All waypoints completed! Total time: %.2f s",
                total);

            done_logged_ = true;
        }
    }

    void handleErrorState()
    {
        publishStop();
        setVisionPriority(false);

        if (!error_logged_) {
            RCLCPP_ERROR(get_logger(), "Error state - stopping.");
            error_logged_ = true;
        }
    }

    // =========================
    // Goal send
    // =========================
    void sendCurrentWaypointGoal()
    {
        const auto &wp = waypoints_[current_waypoint_index_];

        auto request = std::make_shared<lite_task_controller::srv::SetAbsoluteGoal::Request>();

        request->x = wp.pose.position.x;
        request->y = wp.pose.position.y;
        request->yaw = getPoseYaw(wp.pose);

        RCLCPP_WARN(
            get_logger(),
            "Sending waypoint [%zu/%zu] %s: x=%.3f, y=%.3f, yaw=%.3f",
            current_waypoint_index_ + 1,
            waypoints_.size(),
            wp.name.c_str(),
            request->x,
            request->y,
            request->yaw);

        const uint64_t req_id = ++current_req_id_;

        service_call_state_ = ServiceCallState::WAITING_FOR_RESPONSE;

        absolute_goal_client_->async_send_request(
            request,
            [this, req_id, wp_name = wp.name](
                rclcpp::Client<lite_task_controller::srv::SetAbsoluteGoal>::SharedFuture future) {
                if (req_id != current_req_id_) {
                    RCLCPP_DEBUG(
                        get_logger(),
                        "Ignoring stale response for %s",
                        wp_name.c_str());
                    return;
                }

                try {
                    auto resp = future.get();
                    (void)resp;

                    service_call_state_ = ServiceCallState::SUCCEEDED;

                    RCLCPP_WARN(
                        get_logger(),
                        "Goal to %s accepted.",
                        wp_name.c_str());
                } catch (const std::exception &e) {
                    service_call_state_ = ServiceCallState::FAILED;

                    RCLCPP_ERROR(
                        get_logger(),
                        "Goal to %s failed: %s",
                        wp_name.c_str(),
                        e.what());
                }
            });

        goal_sent_ = true;
    }

    bool updateVisionFineTuneMotion(const char *stage_name)
    {
        if (!vision_goal_sent_) {
            RCLCPP_WARN(
                get_logger(),
                "Vision %s command received. Start fine-tune at waypoint index %zu.",
                stage_name,
                current_waypoint_index_);

            fineTuneMoveToTarget();
            return false;
        }

        if (!vision_goal_reached_) {
            if (!vision_goal_accepted_) {
                RCLCPP_INFO_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Waiting for %s fine-tune goal service acceptance...",
                    stage_name);
                return false;
            }

            if (hasReachedGoal(
                    vision_goal_pose_,
                    vision_fine_tune_pos_tolerance_,
                    nav_yaw_tolerance_)) {
                vision_goal_reached_ = true;

                RCLCPP_WARN(
                    get_logger(),
                    "Vision %s fine-tune target reached at waypoint index %zu.",
                    stage_name,
                    current_waypoint_index_);
                return true;
            }

            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Moving to vision %s fine-tune target...",
                stage_name);
            return false;
        }

        return true;
    }

    void fineTuneMoveToTarget()
    {
        if (!odom_received_) {
            RCLCPP_ERROR(get_logger(), "No odometry for fine-tune.");
            switchState(State::ERROR);
            return;
        }

        const double robot_x = current_odom_.pose.pose.position.x;
        const double robot_y = current_odom_.pose.pose.position.y;

        // 视觉程序输出 x,z。
        // 这里沿用原来的二维微调接口：
        //   x -> 机器人前后方向偏移
        //   z -> 机器人左右方向/第二轴偏移
        //
        // 如果你现场发现方向反了，只需要改下面两个符号：
        //   base_x = arm_base_x_m_ + vision_cmd_x_;
        //   base_y = arm_base_y_m_ + vision_cmd_z_;
        // 视觉微调直接作用在 odom/map 坐标系：
        //   goal_odom_x = 当前机器人 x + 机械臂 x 补偿 - 视觉 x
        //   goal_odom_y = 当前机器人 y + 机械臂 y 补偿 + 视觉 z
        const double goal_odom_x = robot_x + arm_base_x_m_ - vision_cmd_z_;
        const double goal_odom_y = robot_y + arm_base_y_m_ + vision_cmd_x_;

        const double goal_yaw = getPoseYaw(waypoints_[current_waypoint_index_].pose);


        auto request = std::make_shared<lite_task_controller::srv::SetAbsoluteGoal::Request>();

        request->x = goal_odom_x;
        request->y = goal_odom_y;
        request->yaw = goal_yaw;
        request->use_custom_tolerance = true;
        request->pos_tolerance = vision_fine_tune_pos_tolerance_;
        request->pos_release_tolerance = vision_fine_tune_release_tolerance_;

        vision_goal_pose_.position.x = goal_odom_x;
        vision_goal_pose_.position.y = goal_odom_y;
        vision_goal_pose_.position.z = 0.0;
        setPoseYaw(vision_goal_pose_, goal_yaw);

        RCLCPP_WARN(
            get_logger(),
            "Fine-tune goal: x=%.3f, y=%.3f, yaw=%.3f, vision_offset=(x=%.4f, z=%.4f), tol=%.3f/%.3f",
            goal_odom_x,
            goal_odom_y,
            goal_yaw,
            vision_cmd_x_,
            vision_cmd_z_,
            vision_fine_tune_pos_tolerance_,
            vision_fine_tune_release_tolerance_);

        absolute_goal_client_->async_send_request(
            request,
            [this](rclcpp::Client<lite_task_controller::srv::SetAbsoluteGoal>::SharedFuture future) {
                try {
                    auto resp = future.get();
                    (void)resp;

                    vision_goal_accepted_ = true;
                    RCLCPP_WARN(get_logger(), "Fine-tune goal accepted, waiting for odom target reach.");
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Fine-tune goal exception: %s",
                        e.what());

                    setVisionPriority(false);
                    switchState(State::ERROR);
                }
            });

        vision_goal_sent_ = true;
    }

    // =========================
    // Vision helper
    // =========================
    void publishVisionResetCommand()
    {
        if (!vision_reset_cmd_pub_) {
            RCLCPP_ERROR(get_logger(), "vision_reset_cmd_pub_ is null.");
            return;
        }

        std_msgs::msg::String msg;
        msg.data = vision_reset_cmd_value_;
        vision_reset_cmd_pub_->publish(msg);

        RCLCPP_WARN(
            get_logger(),
            "Published vision reset command: topic=%s, data='%s'",
            vision_reset_cmd_topic_.c_str(),
            vision_reset_cmd_value_.c_str());
    }

    // =========================
    // State switch
    // =========================
    void switchState(State new_state)
    {
        if (current_state_ == new_state) {
            return;
        }

        RCLCPP_WARN(
            get_logger(),
            "State: %s -> %s",
            stateToString(current_state_).c_str(),
            stateToString(new_state).c_str());

        current_state_ = new_state;
        state_enter_time_ = now();

        if (new_state == State::VISION) {
            resetVisionFlags();
        }
    }

    // =========================
    // Helpers
    // =========================
    bool hasReachedGoal(const geometry_msgs::msg::Pose &goal) const
    {
        return hasReachedGoal(goal, nav_pos_tolerance_, nav_yaw_tolerance_);
    }

    bool hasReachedGoal(
        const geometry_msgs::msg::Pose &goal,
        double pos_tolerance,
        double yaw_tolerance) const
    {
        if (!odom_received_) {
            return false;
        }

        const double dx = goal.position.x - current_odom_.pose.pose.position.x;
        const double dy = goal.position.y - current_odom_.pose.pose.position.y;
        const double dist = std::hypot(dx, dy);

        const double yaw_err = std::abs(
            normalizeAngle(getPoseYaw(goal) - getPoseYaw(current_odom_.pose.pose)));

        return dist < pos_tolerance && yaw_err < yaw_tolerance;
    }

    void publishStop()
    {
        geometry_msgs::msg::Twist twist;
        cmd_vel_pub_->publish(twist);
    }

    void setVisionPriority(bool enable)
    {
        if (!vision_priority_pub_) {
            return;
        }

        std_msgs::msg::Bool msg;
        msg.data = enable;
        vision_priority_pub_->publish(msg);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Vision priority: %s",
            enable ? "true" : "false");
    }

    void resetVisionFlags()
    {
        vision_started_ = false;
        vision_cmd_received_ = false;

        vision_reset_count_ = 0;
        vision_cmd_count_ = 0;

        vision_first_x_ = 0.0;
        vision_second_z_ = 0.0;

        vision_cmd_x_ = 0.0;
        vision_cmd_z_ = 0.0;

        vision_x_move_done_ = false;
        resetVisionGoalRuntime();

        post_vision_waiting_ = false;
    }

    void resetVisionGoalRuntime()
    {
        vision_goal_sent_ = false;
        vision_goal_accepted_ = false;
        vision_goal_reached_ = false;
        vision_goal_pose_ = geometry_msgs::msg::Pose();
    }


    double getDurationSeconds(const rclcpp::Duration &duration) const
    {
        return duration.seconds();
    }

    void setPoseYaw(geometry_msgs::msg::Pose &pose, double yaw)
    {
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = std::sin(yaw / 2.0);
        pose.orientation.w = std::cos(yaw / 2.0);
    }

    double getPoseYaw(const geometry_msgs::msg::Pose &pose) const
    {
        const auto &q = pose.orientation;

        return std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    double normalizeAngle(double angle) const
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    std::string stateToString(State s) const
    {
        switch (s) {
            case State::WAIT_FOR_SYSTEM_READY:
                return "WAIT_FOR_SYSTEM_READY";

            case State::IDLE:
                return "IDLE";

            case State::NAVIGATING:
                return "NAVIGATING";

            case State::VISION:
                return "VISION";

            case State::WAIT_WEAPON_GRASP_DONE:
                return "WAIT_WEAPON_GRASP_DONE";

            case State::WAIT_WEAPON_ASSEMBLE_DONE:
                return "WAIT_WEAPON_ASSEMBLE_DONE";

            case State::DONE:
                return "DONE";

            case State::ERROR:
                return "ERROR";

            default:
                return "UNKNOWN";
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<Zone1Runner>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(
            rclcpp::get_logger("zone1_runner"),
            "Exception: %s",
            e.what());
    }

    rclcpp::shutdown();
    return 0;
}
