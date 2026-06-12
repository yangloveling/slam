#include <chrono>
#include <string>
#include <memory>
#include <cmath>
#include <vector>
#include <functional>
#include <atomic>
#include <stdexcept>
#include <unordered_set>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <cerrno>

#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lite_task_controller/srv/set_absolute_goal.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class ForestRunner : public rclcpp::Node
{
public:
    ForestRunner()
        : Node("forest_runner"),
          state_enter_time_(0, 0, RCL_SYSTEM_TIME),
          waypoint_start_time_(0, 0, RCL_SYSTEM_TIME),
          run_start_time_(0, 0, RCL_SYSTEM_TIME),
          service_call_start_time_(0, 0, RCL_SYSTEM_TIME),
          nav_retry_delay_start_(0, 0, RCL_SYSTEM_TIME)
    {
        declareAndLoadParameters();
        initCanSocket();
        initRosInterfaces();

        control_timer_ = create_wall_timer(
            100ms,
            std::bind(&ForestRunner::controlLoop, this));

        current_state_ = State::WAIT_FOR_SYSTEM_READY;
        state_enter_time_ = now();

        RCLCPP_INFO(
            get_logger(),
            "ForestRunner started. Waypoints: %zu",
            waypoints_.size());

        RCLCPP_INFO(
            get_logger(),
            "CAN: enable=%s, interface=%s, id=0x%X",
            enable_can_send_ ? "true" : "false",
            can_interface_.c_str(),
            infrared_can_id_);

        RCLCPP_INFO(
            get_logger(),
            "Infrared cmd: ON=0x01, OFF=0x02, ROS topic=%s",
            infrared_cmd_topic_.c_str());
    }

    ~ForestRunner() override
    {
        closeCanSocket();
    }

private:
    enum class State
    {
        WAIT_FOR_SYSTEM_READY,
        IDLE,
        NAVIGATING,
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
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_signal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lite_task_status_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr infrared_cmd_pub_;

    rclcpp::Client<lite_task_controller::srv::SetAbsoluteGoal>::SharedPtr absolute_goal_client_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

    std::string start_signal_topic_;
    std::string odom_topic_;
    std::string cmd_vel_topic_;
    std::string set_absolute_goal_service_;
    std::string lite_task_status_topic_;

    double nav_pos_tolerance_{0.08};
    double nav_yaw_tolerance_{0.35};
    int max_nav_retries_{3};
    double nav_execution_timeout_{90.0};
    double service_call_timeout_{5.0};

    bool use_odom_reach_fallback_{true};
    double min_task_running_time_before_done_{0.3};

    std::vector<double> waypoints_x_;
    std::vector<double> waypoints_y_;
    std::vector<double> waypoints_yaw_;
    std::vector<std::string> waypoint_names_;
    std::vector<Waypoint> waypoints_;

    std::string infrared_cmd_topic_;
    int infrared_can_id_{0x66};

    std::string can_interface_{"can0"};
    bool enable_can_send_{true};
    int can_socket_fd_{-1};

    std::vector<int64_t> infrared_on_indices_param_;
    std::vector<int64_t> infrared_off_indices_param_;

    std::unordered_set<size_t> infrared_on_indices_;
    std::unordered_set<size_t> infrared_off_indices_;

    std::unordered_set<size_t> infrared_on_processed_;
    std::unordered_set<size_t> infrared_off_processed_;

    bool infrared_enabled_{false};
    bool allow_repeat_infrared_cmd_{false};

    State current_state_{State::WAIT_FOR_SYSTEM_READY};
    ServiceCallState service_call_state_{ServiceCallState::IDLE};

    nav_msgs::msg::Odometry current_odom_;

    bool odom_received_{false};
    bool start_signal_received_{false};
    bool system_ready_{false};
    bool goal_sent_{false};

    bool task_done_received_{false};
    bool task_running_seen_{false};
    std::string latest_task_status_;

    int nav_retry_count_{0};
    size_t current_waypoint_index_{0};

    std::atomic<uint64_t> current_req_id_{0};

    rclcpp::Time state_enter_time_;
    rclcpp::Time waypoint_start_time_;
    rclcpp::Time run_start_time_;
    rclcpp::Time service_call_start_time_;
    rclcpp::Time nav_retry_delay_start_;

    bool done_logged_{false};
    bool error_logged_{false};

private:
    void declareAndLoadParameters()
    {
        declare_parameter<std::string>("start_signal_topic", "/start_signal");
        declare_parameter<std::string>("odom_topic", "/odometry/filtered");
        declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        declare_parameter<std::string>("set_absolute_goal_service", "/set_absolute_goal");
        declare_parameter<std::string>("lite_task_status_topic", "/lite_task_status");

        declare_parameter<double>("nav_pos_tolerance", 0.08);
        declare_parameter<double>("nav_yaw_tolerance", 0.35);
        declare_parameter<int>("max_nav_retries", 3);
        declare_parameter<double>("nav_execution_timeout", 90.0);
        declare_parameter<double>("service_call_timeout", 5.0);

        declare_parameter<bool>("use_odom_reach_fallback", true);
        declare_parameter<double>("min_task_running_time_before_done", 0.3);

        declare_parameter<std::vector<double>>("waypoints_x", std::vector<double>{0.5});
        declare_parameter<std::vector<double>>("waypoints_y", std::vector<double>{0.0});
        declare_parameter<std::vector<double>>("waypoints_yaw", std::vector<double>{0.0});
        declare_parameter<std::vector<std::string>>("waypoint_names", std::vector<std::string>{});

        declare_parameter<std::string>("infrared_cmd_topic", "/step_cmd");
        declare_parameter<int>("infrared_can_id", 0x66);

        declare_parameter<std::string>("can_interface", "can0");
        declare_parameter<bool>("enable_can_send", true);

        declare_parameter<std::vector<int64_t>>("infrared_on_indices", std::vector<int64_t>{4});
        declare_parameter<std::vector<int64_t>>("infrared_off_indices", std::vector<int64_t>{13});
        declare_parameter<bool>("allow_repeat_infrared_cmd", false);

        get_parameter("start_signal_topic", start_signal_topic_);
        get_parameter("odom_topic", odom_topic_);
        get_parameter("cmd_vel_topic", cmd_vel_topic_);
        get_parameter("set_absolute_goal_service", set_absolute_goal_service_);
        get_parameter("lite_task_status_topic", lite_task_status_topic_);

        get_parameter("nav_pos_tolerance", nav_pos_tolerance_);
        get_parameter("nav_yaw_tolerance", nav_yaw_tolerance_);
        get_parameter("max_nav_retries", max_nav_retries_);
        get_parameter("nav_execution_timeout", nav_execution_timeout_);
        get_parameter("service_call_timeout", service_call_timeout_);

        get_parameter("use_odom_reach_fallback", use_odom_reach_fallback_);
        get_parameter("min_task_running_time_before_done", min_task_running_time_before_done_);

        get_parameter("waypoints_x", waypoints_x_);
        get_parameter("waypoints_y", waypoints_y_);
        get_parameter("waypoints_yaw", waypoints_yaw_);
        get_parameter("waypoint_names", waypoint_names_);

        get_parameter("infrared_cmd_topic", infrared_cmd_topic_);
        get_parameter("infrared_can_id", infrared_can_id_);
        get_parameter("can_interface", can_interface_);
        get_parameter("enable_can_send", enable_can_send_);

        get_parameter("infrared_on_indices", infrared_on_indices_param_);
        get_parameter("infrared_off_indices", infrared_off_indices_param_);
        get_parameter("allow_repeat_infrared_cmd", allow_repeat_infrared_cmd_);

        if (waypoints_x_.empty() || waypoints_y_.empty() || waypoints_yaw_.empty()) {
            throw std::runtime_error("Waypoint arrays cannot be empty");
        }

        if (waypoints_x_.size() != waypoints_y_.size() ||
            waypoints_x_.size() != waypoints_yaw_.size()) {
            throw std::runtime_error("Waypoint array sizes mismatch");
        }

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
                wp.name = "wp_" + std::to_string(i + 1);
            }

            waypoints_.push_back(wp);
        }

        loadInfraredIndices();

        RCLCPP_INFO(get_logger(), "Loaded %zu waypoints", waypoints_.size());

        for (size_t i = 0; i < waypoints_.size(); ++i) {
            RCLCPP_INFO(
                get_logger(),
                "[%zu] %s: x=%.3f, y=%.3f, yaw=%.3f",
                i + 1,
                waypoints_[i].name.c_str(),
                waypoints_[i].pose.position.x,
                waypoints_[i].pose.position.y,
                getPoseYaw(waypoints_[i].pose));
        }

        printIndexSet("infrared_on_indices", infrared_on_indices_);
        printIndexSet("infrared_off_indices", infrared_off_indices_);
    }

    void loadInfraredIndices()
    {
        infrared_on_indices_.clear();
        infrared_off_indices_.clear();

        for (auto idx : infrared_on_indices_param_) {
            if (idx <= 0) {
                RCLCPP_WARN(get_logger(), "Invalid infrared_on_indices item: %ld", static_cast<long>(idx));
                continue;
            }

            if (static_cast<size_t>(idx) > waypoints_.size()) {
                RCLCPP_WARN(
                    get_logger(),
                    "infrared_on_indices item %ld > waypoint count %zu",
                    static_cast<long>(idx),
                    waypoints_.size());
            }

            infrared_on_indices_.insert(static_cast<size_t>(idx));
        }

        for (auto idx : infrared_off_indices_param_) {
            if (idx <= 0) {
                RCLCPP_WARN(get_logger(), "Invalid infrared_off_indices item: %ld", static_cast<long>(idx));
                continue;
            }

            if (static_cast<size_t>(idx) > waypoints_.size()) {
                RCLCPP_WARN(
                    get_logger(),
                    "infrared_off_indices item %ld > waypoint count %zu",
                    static_cast<long>(idx),
                    waypoints_.size());
            }

            infrared_off_indices_.insert(static_cast<size_t>(idx));
        }
    }

    void printIndexSet(const std::string &name, const std::unordered_set<size_t> &indices) const
    {
        std::vector<size_t> sorted(indices.begin(), indices.end());
        std::sort(sorted.begin(), sorted.end());

        std::string text;
        for (size_t i = 0; i < sorted.size(); ++i) {
            text += std::to_string(sorted[i]);
            if (i + 1 < sorted.size()) {
                text += ", ";
            }
        }

        RCLCPP_INFO(get_logger(), "%s: [%s]", name.c_str(), text.c_str());
    }

    void initCanSocket()
    {
        if (!enable_can_send_) {
            RCLCPP_WARN(get_logger(), "CAN sending disabled.");
            return;
        }

        can_socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_fd_ < 0) {
            RCLCPP_ERROR(get_logger(), "Create CAN socket failed: %s", std::strerror(errno));
            return;
        }

        struct ifreq ifr;
        std::memset(&ifr, 0, sizeof(ifr));
        std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", can_interface_.c_str());

        if (ioctl(can_socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(
                get_logger(),
                "Get CAN interface index failed: interface=%s, error=%s",
                can_interface_.c_str(),
                std::strerror(errno));
            closeCanSocket();
            return;
        }

        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
            RCLCPP_ERROR(
                get_logger(),
                "Bind CAN socket failed: interface=%s, error=%s",
                can_interface_.c_str(),
                std::strerror(errno));
            closeCanSocket();
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "CAN socket initialized: interface=%s, fd=%d",
            can_interface_.c_str(),
            can_socket_fd_);
    }

    void closeCanSocket()
    {
        if (can_socket_fd_ >= 0) {
            close(can_socket_fd_);
            can_socket_fd_ = -1;
        }
    }

    bool sendCanCommand(uint8_t cmd_value, const std::string &reason)
    {
        if (!enable_can_send_) {
            return false;
        }

        if (can_socket_fd_ < 0) {
            RCLCPP_ERROR(
                get_logger(),
                "CAN socket not initialized. Cannot send id=0x%X data=0x%02X",
                infrared_can_id_,
                cmd_value);
            return false;
        }

        struct can_frame frame;
        std::memset(&frame, 0, sizeof(frame));

        frame.can_id = static_cast<canid_t>(infrared_can_id_);
        frame.can_dlc = 1;
        frame.data[0] = cmd_value;

        const ssize_t nbytes = write(can_socket_fd_, &frame, sizeof(frame));
        if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
            RCLCPP_ERROR(
                get_logger(),
                "CAN send failed: interface=%s, id=0x%X, data=0x%02X, nbytes=%zd, error=%s",
                can_interface_.c_str(),
                infrared_can_id_,
                cmd_value,
                nbytes,
                std::strerror(errno));
            return false;
        }

        RCLCPP_INFO(
            get_logger(),
            "CAN sent: interface=%s, id=0x%X, data=0x%02X, reason=%s",
            can_interface_.c_str(),
            infrared_can_id_,
            cmd_value,
            reason.c_str());

        return true;
    }

    void initRosInterfaces()
    {
        start_signal_sub_ = create_subscription<std_msgs::msg::Bool>(
            start_signal_topic_,
            10,
            std::bind(&ForestRunner::startSignalCallback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_,
            20,
            std::bind(&ForestRunner::odomCallback, this, std::placeholders::_1));

        lite_task_status_sub_ = create_subscription<std_msgs::msg::String>(
            lite_task_status_topic_,
            10,
            std::bind(&ForestRunner::liteTaskStatusCallback, this, std::placeholders::_1));

        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

        infrared_cmd_pub_ = create_publisher<std_msgs::msg::UInt8>(infrared_cmd_topic_, 10);

        absolute_goal_client_ =
            create_client<lite_task_controller::srv::SetAbsoluteGoal>(set_absolute_goal_service_);

        reset_service_ = create_service<std_srvs::srv::Trigger>(
            "~/reset",
            std::bind(
                &ForestRunner::resetCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
    }

    void startSignalCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !start_signal_received_) {
            start_signal_received_ = true;
            run_start_time_ = now();
            RCLCPP_INFO(get_logger(), "Start signal received");
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = *msg;
        odom_received_ = true;
    }

    void liteTaskStatusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        latest_task_status_ = msg->data;
        const std::string status_upper = toUpper(msg->data);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "lite_task_status: %s",
            msg->data.c_str());

        if (contains(status_upper, "MOVE") ||
            contains(status_upper, "ALIGN") ||
            contains(status_upper, "HOLD") ||
            contains(status_upper, "RUNNING") ||
            contains(status_upper, "ACTIVE") ||
            contains(status_upper, "BUSY")) {
            task_running_seen_ = true;
        }

        if (contains(status_upper, "DONE") ||
            contains(status_upper, "IDLE") ||
            contains(status_upper, "FINISH") ||
            contains(status_upper, "FINISHED") ||
            contains(status_upper, "COMPLETED")) {
            task_done_received_ = true;
        }
    }

    void resetCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        RCLCPP_INFO(get_logger(), "Reset requested");

        publishStop();

        current_waypoint_index_ = 0;
        start_signal_received_ = false;
        goal_sent_ = false;
        nav_retry_count_ = 0;
        service_call_state_ = ServiceCallState::IDLE;
        done_logged_ = false;
        error_logged_ = false;

        task_done_received_ = false;
        task_running_seen_ = false;
        latest_task_status_.clear();

        infrared_on_processed_.clear();
        infrared_off_processed_.clear();

        current_req_id_++;

        publishInfraredCommand(0x02, "reset safety off");

        switchState(State::IDLE);

        response->success = true;
        response->message = "ForestRunner reset completed";
    }

    void controlLoop()
    {
        if (!system_ready_) {
            if (odom_received_ && absolute_goal_client_->service_is_ready()) {
                system_ready_ = true;

                if (current_state_ == State::WAIT_FOR_SYSTEM_READY) {
                    switchState(State::IDLE);
                }

                RCLCPP_INFO(get_logger(), "System ready");
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
        publishStop();

        if (!start_signal_received_) {
            return;
        }

        current_waypoint_index_ = 0;
        nav_retry_count_ = 0;
        goal_sent_ = false;
        service_call_state_ = ServiceCallState::IDLE;
        done_logged_ = false;
        error_logged_ = false;

        task_done_received_ = false;
        task_running_seen_ = false;
        latest_task_status_.clear();

        infrared_on_processed_.clear();
        infrared_off_processed_.clear();

        switchState(State::NAVIGATING);
    }

    void handleNavigatingState()
    {
        if (current_waypoint_index_ >= waypoints_.size()) {
            switchState(State::DONE);
            return;
        }

        const auto &wp = waypoints_[current_waypoint_index_];

        if (service_call_state_ == ServiceCallState::RETRY_DELAY) {
            double retry_delay = getDurationSeconds(now() - nav_retry_delay_start_);
            if (retry_delay > 0.5) {
                goal_sent_ = false;
                service_call_state_ = ServiceCallState::IDLE;
            }
            return;
        }

        if (!goal_sent_) {
            sendCurrentWaypointGoal();
            waypoint_start_time_ = now();

            task_done_received_ = false;
            task_running_seen_ = false;
            latest_task_status_.clear();

            return;
        }

        if (service_call_state_ == ServiceCallState::WAITING_FOR_RESPONSE) {
            double call_elapsed = getDurationSeconds(now() - service_call_start_time_);
            if (call_elapsed > service_call_timeout_) {
                RCLCPP_WARN(get_logger(), "Service call timeout for %s", wp.name.c_str());
                service_call_state_ = ServiceCallState::FAILED;
            }
            return;
        }

        if (service_call_state_ == ServiceCallState::FAILED) {
            RCLCPP_WARN(
                get_logger(),
                "Goal request failed for %s, retry %d/%d",
                wp.name.c_str(),
                nav_retry_count_ + 1,
                max_nav_retries_);

            if (nav_retry_count_ < max_nav_retries_) {
                nav_retry_count_++;
                service_call_state_ = ServiceCallState::RETRY_DELAY;
                nav_retry_delay_start_ = now();
            } else {
                RCLCPP_ERROR(get_logger(), "Max navigation retries exceeded for %s", wp.name.c_str());
                publishInfraredCommand(0x02, "navigation error safety off");
                switchState(State::ERROR);
            }

            return;
        }

        if (service_call_state_ != ServiceCallState::SUCCEEDED) {
            return;
        }

        double execution_elapsed = getDurationSeconds(now() - waypoint_start_time_);

        if (execution_elapsed > nav_execution_timeout_) {
            RCLCPP_WARN(
                get_logger(),
                "Navigation execution timeout for %s. latest_task_status=%s",
                wp.name.c_str(),
                latest_task_status_.c_str());

            service_call_state_ = ServiceCallState::FAILED;
            return;
        }

        bool reached = false;

        const bool allow_status_done =
            execution_elapsed >= min_task_running_time_before_done_;

        if (allow_status_done && task_done_received_) {
            reached = true;

            RCLCPP_INFO(
                get_logger(),
                "Waypoint %s completed by lite_task_status='%s'",
                wp.name.c_str(),
                latest_task_status_.c_str());
        }

        if (!reached && use_odom_reach_fallback_ && hasReachedGoal(wp.pose)) {
            reached = true;
            RCLCPP_INFO(get_logger(), "Waypoint %s completed by odom fallback", wp.name.c_str());
        }

        if (!reached) {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Waiting waypoint [%zu/%zu] %s complete. elapsed=%.2f, status='%s', done=%d",
                current_waypoint_index_ + 1,
                waypoints_.size(),
                wp.name.c_str(),
                execution_elapsed,
                latest_task_status_.c_str(),
                task_done_received_ ? 1 : 0);
            return;
        }

        onCurrentWaypointReached(execution_elapsed);
    }

    void onCurrentWaypointReached(double execution_elapsed)
    {
        const auto &wp = waypoints_[current_waypoint_index_];
        const size_t reached_index_1based = current_waypoint_index_ + 1;

        RCLCPP_INFO(
            get_logger(),
            "Reached waypoint [%zu/%zu] %s in %.2f s",
            reached_index_1based,
            waypoints_.size(),
            wp.name.c_str(),
            execution_elapsed);

        handleInfraredOnWaypointReached(reached_index_1based, wp.name);

        nav_retry_count_ = 0;
        current_waypoint_index_++;
        goal_sent_ = false;
        service_call_state_ = ServiceCallState::IDLE;

        task_done_received_ = false;
        task_running_seen_ = false;
        latest_task_status_.clear();
    }

    void handleDoneState()
    {
        publishStop();

        if (done_logged_) {
            return;
        }

        double total = getDurationSeconds(now() - run_start_time_);

        RCLCPP_INFO(get_logger(), "All waypoints completed! Total time: %.2f s", total);

        publishInfraredCommand(0x02, "mission completed safety off");

        done_logged_ = true;
    }

    void handleErrorState()
    {
        publishStop();

        if (error_logged_) {
            return;
        }

        publishInfraredCommand(0x02, "error state safety off");

        RCLCPP_ERROR(get_logger(), "Entered ERROR state");
        error_logged_ = true;
    }

    void handleInfraredOnWaypointReached(size_t waypoint_index_1based, const std::string &waypoint_name)
    {
        bool should_send_on = infrared_on_indices_.count(waypoint_index_1based) > 0;
        bool should_send_off = infrared_off_indices_.count(waypoint_index_1based) > 0;

        if (!allow_repeat_infrared_cmd_) {
            if (should_send_on && infrared_on_processed_.count(waypoint_index_1based) > 0) {
                should_send_on = false;
            }

            if (should_send_off && infrared_off_processed_.count(waypoint_index_1based) > 0) {
                should_send_off = false;
            }
        }

        if (should_send_on) {
            infrared_on_processed_.insert(waypoint_index_1based);

            publishInfraredCommand(
                0x01,
                "arrived waypoint #" + std::to_string(waypoint_index_1based) +
                " (" + waypoint_name + "), infrared ON");
        }

        if (should_send_off) {
            infrared_off_processed_.insert(waypoint_index_1based);

            publishInfraredCommand(
                0x02,
                "arrived waypoint #" + std::to_string(waypoint_index_1based) +
                " (" + waypoint_name + "), infrared OFF");
        }
    }

    void publishInfraredCommand(uint8_t cmd_value, const std::string &reason)
    {
        if (cmd_value != 0x01 && cmd_value != 0x02) {
            RCLCPP_WARN(get_logger(), "Invalid infrared command: 0x%02X", cmd_value);
            return;
        }

        const bool target_enabled = (cmd_value == 0x01);

        if (!allow_repeat_infrared_cmd_ && infrared_enabled_ == target_enabled) {
            RCLCPP_INFO(
                get_logger(),
                "Infrared command skipped because already %s. reason=%s",
                target_enabled ? "ON" : "OFF",
                reason.c_str());
            return;
        }

        if (infrared_cmd_pub_) {
            std_msgs::msg::UInt8 msg;
            msg.data = cmd_value;
            infrared_cmd_pub_->publish(msg);

            RCLCPP_INFO(
                get_logger(),
                "ROS infrared command published: topic=%s, data=0x%02X",
                infrared_cmd_topic_.c_str(),
                cmd_value);
        }

        const bool can_ok = sendCanCommand(cmd_value, reason);

        if (can_ok || !enable_can_send_) {
            infrared_enabled_ = target_enabled;
        }

        RCLCPP_INFO(
            get_logger(),
            "Infrared command handled: id=0x%X, cmd=0x%02X, can=%s, reason=%s",
            infrared_can_id_,
            cmd_value,
            can_ok ? "OK" : "FAILED_OR_DISABLED",
            reason.c_str());
    }

    void sendCurrentWaypointGoal()
    {
        const auto &wp = waypoints_[current_waypoint_index_];

        auto request = std::make_shared<lite_task_controller::srv::SetAbsoluteGoal::Request>();
        request->x = wp.pose.position.x;
        request->y = wp.pose.position.y;
        request->yaw = getPoseYaw(wp.pose);

        RCLCPP_INFO(
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
        service_call_start_time_ = now();
        goal_sent_ = true;

        absolute_goal_client_->async_send_request(
            request,
            [this, req_id, wp_name = wp.name](
                rclcpp::Client<lite_task_controller::srv::SetAbsoluteGoal>::SharedFuture future)
            {
                if (req_id != current_req_id_.load()) {
                    return;
                }

                try {
                    auto resp = future.get();

                    if (resp && resp->success) {
                        service_call_state_ = ServiceCallState::SUCCEEDED;

                        RCLCPP_INFO(
                            get_logger(),
                            "Goal to %s accepted: %s",
                            wp_name.c_str(),
                            resp->message.c_str());
                    } else {
                        service_call_state_ = ServiceCallState::FAILED;
                        RCLCPP_WARN(get_logger(), "Goal to %s rejected", wp_name.c_str());
                    }
                } catch (const std::exception &e) {
                    service_call_state_ = ServiceCallState::FAILED;

                    RCLCPP_ERROR(
                        get_logger(),
                        "Goal to %s failed: %s",
                        wp_name.c_str(),
                        e.what());
                }
            });
    }

    void switchState(State new_state)
    {
        if (current_state_ == new_state) {
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "State: %s -> %s",
            stateToString(current_state_).c_str(),
            stateToString(new_state).c_str());

        current_state_ = new_state;
        state_enter_time_ = now();
    }

    bool hasReachedGoal(const geometry_msgs::msg::Pose &goal) const
    {
        if (!odom_received_) {
            return false;
        }

        const double dx = goal.position.x - current_odom_.pose.pose.position.x;
        const double dy = goal.position.y - current_odom_.pose.pose.position.y;
        const double dist = std::hypot(dx, dy);

        const double goal_yaw = getPoseYaw(goal);
        const double current_yaw = getPoseYaw(current_odom_.pose.pose);
        const double yaw_err = std::abs(normalizeAngle(goal_yaw - current_yaw));

        return dist < nav_pos_tolerance_ && yaw_err < nav_yaw_tolerance_;
    }

    void publishStop()
    {
        geometry_msgs::msg::Twist twist;
        cmd_vel_pub_->publish(twist);
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
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }

        while (angle < -M_PI) {
            angle += 2.0 * M_PI;
        }

        return angle;
    }

    std::string stateToString(State s) const
    {
        switch (s) {
            case State::WAIT_FOR_SYSTEM_READY:
                return "READY_WAIT";
            case State::IDLE:
                return "IDLE";
            case State::NAVIGATING:
                return "NAVIGATING";
            case State::DONE:
                return "DONE";
            case State::ERROR:
                return "ERROR";
            default:
                return "UNKNOWN";
        }
    }

    std::string toUpper(const std::string &input) const
    {
        std::string out = input;
        std::transform(
            out.begin(),
            out.end(),
            out.begin(),
            [](unsigned char c) {
                return static_cast<char>(std::toupper(c));
            });
        return out;
    }

    bool contains(const std::string &text, const std::string &pattern) const
    {
        return text.find(pattern) != std::string::npos;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<ForestRunner>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(
            rclcpp::get_logger("forest_runner"),
            "Exception: %s",
            e.what());
    }

    rclcpp::shutdown();
    return 0;
}

