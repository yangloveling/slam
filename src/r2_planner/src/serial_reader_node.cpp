#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <vector>
#include <cstdint>
#include <cstring>
#include <functional>
#include <cerrno>
#include <algorithm>

// ==================== 分隔符帧解析器 ====================
class FrameParser {
public:
    using DataCallback = std::function<void(const std::vector<uint8_t>&)>;

    FrameParser(DataCallback cb, size_t max_buffer_size = 2048)
        : callback_(std::move(cb)), max_buffer_size_(max_buffer_size) {}

    void add_bytes(const uint8_t* data, size_t len) {
        buffer_.insert(buffer_.end(), data, data + len);
        if (buffer_.size() > max_buffer_size_) {
            buffer_.erase(buffer_.begin(), buffer_.begin() + (buffer_.size() - max_buffer_size_));
        }
        process();
    }

private:
    void process() {
        while (buffer_.size() >= 2) {
            if (buffer_[0] == 0x0A && buffer_[1] == 0x0D) {
                size_t tail_pos = 0;
                for (size_t i = 2; i <= buffer_.size() - 2; ++i) {
                    if (buffer_[i] == 0x0D && buffer_[i + 1] == 0x0A) {
                        tail_pos = i;
                        break;
                    }
                }
                if (tail_pos > 0) {
                    std::vector<uint8_t> payload(buffer_.begin() + 2, buffer_.begin() + tail_pos);
                    callback_(payload);
                    buffer_.erase(buffer_.begin(), buffer_.begin() + tail_pos + 2);
                    continue;
                } else {
                    break;
                }
            } else {
                buffer_.erase(buffer_.begin());
            }
        }
    }

    DataCallback callback_;
    std::vector<uint8_t> buffer_;
    size_t max_buffer_size_;
};

// ==================== ROS2 节点 ====================
class SerialReaderNode : public rclcpp::Node {
public:
    SerialReaderNode() : Node("serial_reader_node") {
        declare_parameter("serial_port", "/dev/ttyACM0");
        declare_parameter("baud_rate", 115200);
        declare_parameter("log_period_ms", 1000);
        declare_parameter("raw_data_topic", "/serial/raw_data");
        declare_parameter("id_type_topic", "/detected_id_type");

        std::string port = get_parameter("serial_port").as_string();
        int baud_rate = get_parameter("baud_rate").as_int();
        log_period_ms_ = std::max(100, static_cast<int>(get_parameter("log_period_ms").as_int()));
        raw_data_topic_ = get_parameter("raw_data_topic").as_string();
        id_type_topic_ = get_parameter("id_type_topic").as_string();

        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ == -1) {
            RCLCPP_ERROR(get_logger(), "Failed to open %s", port.c_str());
            rclcpp::shutdown();
            return;
        }

        struct termios opts;
        if (tcgetattr(fd_, &opts) != 0) {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to get termios for %s: %s",
                port.c_str(),
                std::strerror(errno)
            );
            close(fd_);
            fd_ = -1;
            rclcpp::shutdown();
            return;
        }

        speed_t baud_const = baudToConstant(baud_rate);
        if (baud_const == 0) {
            RCLCPP_WARN(get_logger(), "Unsupported baud_rate=%d, fallback to 115200", baud_rate);
            baud_const = B115200;
            baud_rate = 115200;
        }

        cfsetispeed(&opts, baud_const);
        cfsetospeed(&opts, baud_const);
        opts.c_cflag |= (CLOCAL | CREAD);
        opts.c_cflag &= ~PARENB;
        opts.c_cflag &= ~CSTOPB;
        opts.c_cflag &= ~CSIZE;
        opts.c_cflag |= CS8;
        opts.c_cflag &= ~CRTSCTS;
        opts.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        opts.c_iflag &= ~(IXON | IXOFF | IXANY);
        opts.c_oflag &= ~OPOST;
        opts.c_cc[VMIN] = 0;
        opts.c_cc[VTIME] = 1;

        if (tcsetattr(fd_, TCSANOW, &opts) != 0) {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to set termios for %s: %s",
                port.c_str(),
                std::strerror(errno)
            );
            close(fd_);
            fd_ = -1;
            rclcpp::shutdown();
            return;
        }

        // 发布者
        raw_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>(raw_data_topic_, 10);
        id_type_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>(id_type_topic_, 10);

        parser_ = std::make_unique<FrameParser>(
            [this](const std::vector<uint8_t>& data) { on_frame_received(data); }
        );

        keep_running_ = true;
        read_thread_ = std::thread(&SerialReaderNode::read_loop, this);
        RCLCPP_INFO(
            get_logger(),
            "Serial reader started on %s@%d, raw_topic=%s, id_type_topic=%s",
            port.c_str(),
            baud_rate,
            raw_data_topic_.c_str(),
            id_type_topic_.c_str());
    }

    ~SerialReaderNode() {
        keep_running_ = false;
        if (read_thread_.joinable()) read_thread_.join();
        if (fd_ != -1) close(fd_);
    }

private:
    speed_t baudToConstant(int baud_rate) const {
        switch (baud_rate) {
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            case 230400: return B230400;
            case 460800: return B460800;
            case 921600: return B921600;
            default: return 0;
        }
    }

    void read_loop() {
        uint8_t buf[256];
        while (keep_running_ && rclcpp::ok()) {
            int n = read(fd_, buf, sizeof(buf));
            if (n > 0) {
                parser_->add_bytes(buf, n);
            } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Serial read error: %s",
                    std::strerror(errno)
                );
            }
        }
    }

    void on_frame_received(const std::vector<uint8_t>& payload) {
        // 发布原始字节（调试用）
        auto raw_msg = std_msgs::msg::UInt8MultiArray();
        raw_msg.data = payload;
        raw_pub_->publish(raw_msg);

        if (payload.size() >= 2) {
            uint8_t type   = payload[0];  // 第一个字节：type，原样发布
            uint8_t raw_id = payload[1];  // 第二个字节：原始 id，需映射

            uint8_t id = 0;
            bool valid = true;
            
            // 映射表（保持不变，只是数据来源改为 payload[1]）
            switch (raw_id) {
                case 97:  id = 1;  break;
                case 98:  id = 2;  break;
                case 99:  id = 3;  break;
                case 112: id = 4;  break;
                case 120: id = 5;  break;
                case 102: id = 6;  break;
                case 111: id = 7;  break;
                case 121: id = 8;  break;
                case 103: id = 9;  break;
                case 108: id = 10; break;
                case 107: id = 11; break;
                case 106: id = 12; break;
                case 238: 
                    id = 13; 
                    break;
                default:  valid = false; break;
            }

            if (valid) {
                auto id_type_msg = std_msgs::msg::UInt8MultiArray();
                id_type_msg.data = {id, type};
                id_type_pub_->publish(id_type_msg);

                RCLCPP_INFO_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    log_period_ms_,
                    "ID=%d, Type=%d",
                    id,
                    type
                );
            } else {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    log_period_ms_,
                    "Unknown raw_id=%d, Type=%d",
                    raw_id,
                    type
                );
            }
        } else {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                log_period_ms_,
                "Invalid payload length: %zu",
                payload.size()
            );
        }
    }

    int fd_ = -1;
    std::thread read_thread_;
    std::atomic<bool> keep_running_;
    std::unique_ptr<FrameParser> parser_;
    int log_period_ms_{1000};
    std::string raw_data_topic_{"/serial/raw_data"};
    std::string id_type_topic_{"/detected_id_type"};

    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr id_type_pub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialReaderNode>();
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
