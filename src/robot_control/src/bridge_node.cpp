/**
 * @file imu_odom_bridge.cpp
 * @brief Convert odometry from IMU/body frame to base_link frame.
 *
 * 输入:
 *   nav_msgs/msg/Odometry
 *   pose 表示 odom -> imu/body
 *
 * 输出:
 *   nav_msgs/msg/Odometry
 *   pose 表示 odom -> base_link
 *
 * 需要提供的外参:
 *   base_link -> imu/body
 *
 * 注意:
 *   这里的 imu/body 是 /Odometry.child_frame_id 里的那个 body。
 *   不要再填雷达到 base_link 的外参，除非 body 就是雷达。
 */

#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "Eigen/Core"
#include "Eigen/Geometry"

class ImuOdomBridge : public rclcpp::Node {
public:
    ImuOdomBridge() : Node("imu_odom_bridge"), first_frame_(true) {
        // =========================
        // 参数声明
        // =========================
        this->declare_parameter<std::string>("input_topic", "/Odometry");
        this->declare_parameter<std::string>("output_topic", "/odom");

        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>("imu_frame", "body");

        /*
         * 这里填写 base_link -> imu/body 外参
         *
         * tx, ty, tz:
         *   IMU 原点在 base_link 坐标系下的位置
         *
         * roll, pitch, yaw:
         *   IMU 坐标系相对于 base_link 坐标系的旋转
         *
         * 如果 IMU 的坐标轴和 base_link 基本平行:
         *   roll = 0
         *   pitch = 0
         *   yaw = 0
         *
         * 注意:
         *   当前雷达/IMU 安装 pitch 固定为 +0.7854 rad。
         */
        this->declare_parameter<double>("tx",  0.126960);
        this->declare_parameter<double>("ty", 0.350815);
        this->declare_parameter<double>("tz", 0.57);
        this->declare_parameter<double>("roll", 0.0);
        this->declare_parameter<double>("pitch", 0.7854);
        this->declare_parameter<double>("yaw", 0.0);

        this->declare_parameter<bool>("zero_first_frame", true);

        // =========================
        // 读取参数
        // =========================
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();

        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        imu_frame_ = this->get_parameter("imu_frame").as_string();

        double tx = this->get_parameter("tx").as_double();
        double ty = this->get_parameter("ty").as_double();
        double tz = this->get_parameter("tz").as_double();

        double roll = this->get_parameter("roll").as_double();
        double pitch = this->get_parameter("pitch").as_double();
        double yaw = this->get_parameter("yaw").as_double();

        zero_first_frame_ = this->get_parameter("zero_first_frame").as_bool();

        // =========================
        // 构建外参 base_link -> imu/body
        // =========================
        Eigen::Matrix3d R_base_imu = rpyToRot(roll, pitch, yaw);
        Eigen::Vector3d t_base_imu(tx, ty, tz);

        T_base_imu_ = makeTransform(R_base_imu, t_base_imu);
        T_imu_base_ = inverseTransform(T_base_imu_);

        // =========================
        // ROS 通信
        // =========================
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            input_topic_,
            rclcpp::QoS(100),
            std::bind(&ImuOdomBridge::odomCallback, this, std::placeholders::_1)
        );

        pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            output_topic_,
            rclcpp::QoS(100)
        );

        // =========================
        // 打印信息
        // =========================
        RCLCPP_INFO(this->get_logger(), "ImuOdomBridge started.");
        RCLCPP_INFO(this->get_logger(), "Input topic : %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "odom frame  : %s", odom_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "base frame  : %s", base_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "imu frame   : %s", imu_frame_.c_str());

        RCLCPP_INFO(
            this->get_logger(),
            "Extrinsic base_link -> imu/body: xyz=(%.6f, %.6f, %.6f), rpy=(%.6f, %.6f, %.6f)",
            tx, ty, tz, roll, pitch, yaw
        );

        RCLCPP_WARN(
            this->get_logger(),
            "Make sure this extrinsic is base_link -> imu/body, not base_link -> lidar."
        );
    }

private:
    Eigen::Matrix3d rpyToRot(double roll, double pitch, double yaw) {
        Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;
        q.normalize();

        return q.toRotationMatrix();
    }

    Eigen::Matrix3d quatToRot(double qx, double qy, double qz, double qw) {
        Eigen::Quaterniond q(qw, qx, qy, qz);

        if (q.norm() < 1e-12) {
            return Eigen::Matrix3d::Identity();
        }

        q.normalize();
        return q.toRotationMatrix();
    }

    Eigen::Vector4d rotToQuat(const Eigen::Matrix3d& R) {
        Eigen::Quaterniond q(R);
        q.normalize();

        return Eigen::Vector4d(
            q.x(),
            q.y(),
            q.z(),
            q.w()
        );
    }

    Eigen::Matrix4d makeTransform(
        const Eigen::Matrix3d& R,
        const Eigen::Vector3d& t
    ) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = R;
        T.block<3, 1>(0, 3) = t;
        return T;
    }

    Eigen::Matrix4d inverseTransform(const Eigen::Matrix4d& T) {
        Eigen::Matrix3d R = T.block<3, 3>(0, 0);
        Eigen::Vector3d t = T.block<3, 1>(0, 3);

        Eigen::Matrix4d T_inv = Eigen::Matrix4d::Identity();
        T_inv.block<3, 3>(0, 0) = R.transpose();
        T_inv.block<3, 1>(0, 3) = -R.transpose() * t;

        return T_inv;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // =========================
        // 1. 读取输入 odom -> imu/body
        // =========================
        Eigen::Matrix3d R_odom_imu = quatToRot(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );

        Eigen::Vector3d t_odom_imu(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        );

        Eigen::Matrix4d T_odom_imu = makeTransform(R_odom_imu, t_odom_imu);

        // =========================
        // 2. 转成 odom -> base_link
        //
        // T_odom_base = T_odom_imu * T_imu_base
        // =========================
        Eigen::Matrix4d T_odom_base = T_odom_imu * T_imu_base_;

        // =========================
        // 3. 第一帧归零，可选
        // =========================
        Eigen::Matrix4d T_out;

        if (zero_first_frame_) {
            if (first_frame_) {
                T_odom_base_0_ = T_odom_base;
                T_base0_odom_ = inverseTransform(T_odom_base_0_);
                first_frame_ = false;

                RCLCPP_INFO(
                    this->get_logger(),
                    "Captured first base_link pose as zero reference."
                );
            }

            T_out = T_base0_odom_ * T_odom_base;
        } else {
            T_out = T_odom_base;
        }

        Eigen::Matrix3d R_out = T_out.block<3, 3>(0, 0);
        Eigen::Vector3d t_out = T_out.block<3, 1>(0, 3);
        Eigen::Vector4d q_out = rotToQuat(R_out);

        // =========================
        // 4. 处理 twist
        //
        // 输入 twist 默认认为是在 imu/body frame 下，
        // 输出 twist 转成 base_link frame 下。
        //
        // 已知:
        //   v_imu: IMU 原点速度，表达在 IMU 坐标系
        //   w_imu: IMU 角速度，表达在 IMU 坐标系
        //
        // 转换:
        //   w_base = R_base_imu * w_imu
        //   v_base = R_base_imu * v_imu - w_base x t_base_imu
        // =========================
        Eigen::Matrix3d R_base_imu = T_base_imu_.block<3, 3>(0, 0);
        Eigen::Vector3d t_base_imu = T_base_imu_.block<3, 1>(0, 3);

        Eigen::Vector3d v_imu(
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z
        );

        Eigen::Vector3d w_imu(
            msg->twist.twist.angular.x,
            msg->twist.twist.angular.y,
            msg->twist.twist.angular.z
        );

        Eigen::Vector3d w_base = R_base_imu * w_imu;
        Eigen::Vector3d v_base = R_base_imu * v_imu - w_base.cross(t_base_imu);

        // =========================
        // 5. 发布结果
        // =========================
        nav_msgs::msg::Odometry odom_out;

        odom_out.header.stamp = msg->header.stamp;
        odom_out.header.frame_id = odom_frame_;
        odom_out.child_frame_id = base_frame_;

        odom_out.pose.pose.position.x = t_out.x();
        odom_out.pose.pose.position.y = t_out.y();
        odom_out.pose.pose.position.z = t_out.z();

        odom_out.pose.pose.orientation.x = q_out.x();
        odom_out.pose.pose.orientation.y = q_out.y();
        odom_out.pose.pose.orientation.z = q_out.z();
        odom_out.pose.pose.orientation.w = q_out.w();

        odom_out.pose.covariance = msg->pose.covariance;

        odom_out.twist.twist.linear.x = v_base.x();
        odom_out.twist.twist.linear.y = v_base.y();
        odom_out.twist.twist.linear.z = v_base.z();

        odom_out.twist.twist.angular.x = w_base.x();
        odom_out.twist.twist.angular.y = w_base.y();
        odom_out.twist.twist.angular.z = w_base.z();

        odom_out.twist.covariance = msg->twist.covariance;

        pub_->publish(odom_out);
    }

private:
    std::string input_topic_;
    std::string output_topic_;

    std::string odom_frame_;
    std::string base_frame_;
    std::string imu_frame_;

    bool zero_first_frame_;

    Eigen::Matrix4d T_base_imu_;
    Eigen::Matrix4d T_imu_base_;

    Eigen::Matrix4d T_odom_base_0_;
    Eigen::Matrix4d T_base0_odom_;

    bool first_frame_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuOdomBridge>());
    rclcpp::shutdown();
    return 0;
}
