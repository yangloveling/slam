#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class OdomToYaw : public rclcpp::Node
{
public:
    OdomToYaw() : Node("odom_to_yaw")
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&OdomToYaw::odom_callback, this, std::placeholders::_1));

        yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("/yaw", 10);

        RCLCPP_INFO(this->get_logger(), "odom_to_yaw node started");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto q = msg->pose.pose.orientation;

        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(quat);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double yaw_deg = yaw * 180.0 / M_PI;

        if (yaw_deg < 0)
            yaw_deg += 360.0;

        std_msgs::msg::Float64 yaw_msg;
        yaw_msg.data = yaw_deg;
        yaw_pub_->publish(yaw_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomToYaw>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
