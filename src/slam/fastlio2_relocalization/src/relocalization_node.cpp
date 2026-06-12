#include <memory>
#include <mutex>
#include <string>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <small_gicp/registration/registration_helper.hpp>

using namespace std::chrono_literals;

class FastLioRelocalizationNode : public rclcpp::Node {
public:
  FastLioRelocalizationNode()
  : Node("fastlio2_relocalization_node") {
    // =========================
    // 基础参数
    // =========================
    this->declare_parameter<std::string>("map_pcd", "");
    this->declare_parameter<std::string>("cloud_topic", "/cloud_registered_body");
    this->declare_parameter<std::string>("odom_topic", "/odometry/filtered");

    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");

    this->declare_parameter<double>("downsampling_resolution", 0.5);
    this->declare_parameter<double>("max_correspondence_distance", 2.0);
    this->declare_parameter<int>("num_threads", 4);
    this->declare_parameter<double>("align_period", 0.2);
    this->declare_parameter<double>("max_cloud_odom_dt", 0.25);
    this->declare_parameter<bool>("skip_repeated_cloud", true);
    this->declare_parameter<int>("min_raw_source_points", 100);
    this->declare_parameter<int>("min_preprocessed_source_points", 50);

    // =========================
    // body 点云 -> base_link 点云转换参数
    //
    // /cloud_registered_body 的点云坐标系是 body
    // /odometry/filtered 的 child_frame_id 是 base_link
    //
    // 这里的 base_to_body_* 表示：
    //   body 原点和姿态在 base_link 坐标系下的位姿
    // 即：
    //   T_base_body
    //
    // 点云转换：
    //   p_base = T_base_body * p_body
    // =========================
    this->declare_parameter<bool>("cloud_to_base_enable", true);

    this->declare_parameter<double>("base_to_body_tx", 0.126960);
    this->declare_parameter<double>("base_to_body_ty",  0.350815);
    this->declare_parameter<double>("base_to_body_tz",  0.57);

    this->declare_parameter<double>("base_to_body_roll",  0.0);
    this->declare_parameter<double>("base_to_body_pitch", 0.7854);
    this->declare_parameter<double>("base_to_body_yaw",   0.0);

    // =========================
    // 快速运动稳定性参数
    // =========================
    this->declare_parameter<int>("min_inliers", 300);
    this->declare_parameter<double>("min_inlier_ratio", 0.25);

    // 相邻两次成功重定位 T_map_base 允许的最大跳变
    // 过大则认为是错误匹配，拒绝发布
    this->declare_parameter<double>("max_translation_jump", 1.5);
    this->declare_parameter<double>("max_rotation_jump_deg", 30.0);

    // GICP 失败或结果被拒绝时，是否继续发布上一帧 map->odom
    // 建议 true，避免 TF 短时间断掉
    this->declare_parameter<bool>("keep_last_tf_on_fail", true);

    // 启动后第一帧是否跳过 jump gate
    // 建议 true，因为第一帧没有历史参考
    this->declare_parameter<bool>("accept_first_reloc", true);

    // =========================
    // 读取基础参数
    // =========================
    map_pcd_ = this->get_parameter("map_pcd").as_string();
    cloud_topic_ = this->get_parameter("cloud_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();

    map_frame_ = this->get_parameter("map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    downsampling_resolution_ =
      std::max(0.01, this->get_parameter("downsampling_resolution").as_double());
    max_correspondence_distance_ =
      std::max(0.05, this->get_parameter("max_correspondence_distance").as_double());
    num_threads_ = std::max(1, static_cast<int>(this->get_parameter("num_threads").as_int()));
    align_period_ = std::max(0.05, this->get_parameter("align_period").as_double());
    max_cloud_odom_dt_ = std::max(0.0, this->get_parameter("max_cloud_odom_dt").as_double());
    skip_repeated_cloud_ = this->get_parameter("skip_repeated_cloud").as_bool();
    min_raw_source_points_ =
      std::max(1, static_cast<int>(this->get_parameter("min_raw_source_points").as_int()));
    min_preprocessed_source_points_ =
      std::max(
        1,
        static_cast<int>(this->get_parameter("min_preprocessed_source_points").as_int()));

    cloud_to_base_enable_ = this->get_parameter("cloud_to_base_enable").as_bool();

    min_inliers_ = std::max(1, static_cast<int>(this->get_parameter("min_inliers").as_int()));
    min_inlier_ratio_ =
      std::clamp(this->get_parameter("min_inlier_ratio").as_double(), 0.0, 1.0);
    max_translation_jump_ =
      std::max(0.01, this->get_parameter("max_translation_jump").as_double());
    max_rotation_jump_deg_ =
      std::max(0.1, this->get_parameter("max_rotation_jump_deg").as_double());
    keep_last_tf_on_fail_ = this->get_parameter("keep_last_tf_on_fail").as_bool();
    accept_first_reloc_ = this->get_parameter("accept_first_reloc").as_bool();

    // =========================
    // 读取 base_link -> body 外参
    // =========================
    const double base_to_body_tx =
      this->get_parameter("base_to_body_tx").as_double();
    const double base_to_body_ty =
      this->get_parameter("base_to_body_ty").as_double();
    const double base_to_body_tz =
      this->get_parameter("base_to_body_tz").as_double();

    const double base_to_body_roll =
      this->get_parameter("base_to_body_roll").as_double();
    const double base_to_body_pitch =
      this->get_parameter("base_to_body_pitch").as_double();
    const double base_to_body_yaw =
      this->get_parameter("base_to_body_yaw").as_double();

    Eigen::Matrix3d R_base_body = rpyToRot(
      base_to_body_roll,
      base_to_body_pitch,
      base_to_body_yaw
    );

    Eigen::Vector3d t_base_body(
      base_to_body_tx,
      base_to_body_ty,
      base_to_body_tz
    );

    T_base_body_ = Eigen::Isometry3d::Identity();
    T_base_body_.linear() = R_base_body;
    T_base_body_.translation() = t_base_body;

    // 初始 map->odom 先设为单位阵
    last_T_map_odom_ = Eigen::Isometry3d::Identity();
    has_last_map_odom_ = false;

    last_T_map_base_ = Eigen::Isometry3d::Identity();
    has_last_map_base_ = false;
    has_last_processed_cloud_stamp_ = false;

    // =========================
    // 打印参数
    // =========================
    RCLCPP_INFO(this->get_logger(), "===== fastlio2_relocalization parameters =====");
    RCLCPP_INFO(this->get_logger(), "map_pcd     : %s", map_pcd_.c_str());
    RCLCPP_INFO(this->get_logger(), "cloud_topic : %s", cloud_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "odom_topic  : %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "map_frame   : %s", map_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "odom_frame  : %s", odom_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "base_frame  : %s", base_frame_.c_str());

    RCLCPP_INFO(this->get_logger(), "downsampling_resolution     : %.3f", downsampling_resolution_);
    RCLCPP_INFO(this->get_logger(), "max_correspondence_distance : %.3f", max_correspondence_distance_);
    RCLCPP_INFO(this->get_logger(), "num_threads                 : %d", num_threads_);
    RCLCPP_INFO(this->get_logger(), "align_period                : %.3f", align_period_);
    RCLCPP_INFO(this->get_logger(), "max_cloud_odom_dt           : %.3f", max_cloud_odom_dt_);
    RCLCPP_INFO(
      this->get_logger(),
      "skip_repeated_cloud=%s, min_raw_source_points=%d, min_preprocessed_source_points=%d",
      skip_repeated_cloud_ ? "true" : "false",
      min_raw_source_points_,
      min_preprocessed_source_points_
    );

    RCLCPP_INFO(
      this->get_logger(),
      "cloud_to_base_enable: %s",
      cloud_to_base_enable_ ? "true" : "false"
    );

    RCLCPP_INFO(
      this->get_logger(),
      "T_base_body xyz=(%.6f, %.6f, %.6f), rpy=(%.6f, %.6f, %.6f)",
      base_to_body_tx,
      base_to_body_ty,
      base_to_body_tz,
      base_to_body_roll,
      base_to_body_pitch,
      base_to_body_yaw
    );

    RCLCPP_INFO(
      this->get_logger(),
      "gating: min_inliers=%d, min_inlier_ratio=%.3f, max_translation_jump=%.3f, max_rotation_jump_deg=%.3f",
      min_inliers_,
      min_inlier_ratio_,
      max_translation_jump_,
      max_rotation_jump_deg_
    );

    RCLCPP_INFO(
      this->get_logger(),
      "keep_last_tf_on_fail=%s, accept_first_reloc=%s",
      keep_last_tf_on_fail_ ? "true" : "false",
      accept_first_reloc_ ? "true" : "false"
    );

    if (cloud_to_base_enable_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Input cloud is assumed to be in body frame. It will be transformed to base_link frame before GICP."
      );
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "cloud_to_base_enable is false. Input cloud will be used directly. This assumes body == base_link."
      );
    }

    // =========================
    // 加载地图
    // =========================
    if (!loadMap(map_pcd_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load map: %s", map_pcd_.c_str());
      throw std::runtime_error("failed to load map");
    }

    // =========================
    // ROS pub/sub/timer
    // =========================
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/relocalization_pose", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&FastLioRelocalizationNode::cloudCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      100,
      std::bind(&FastLioRelocalizationNode::odomCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(align_period_),
      std::bind(&FastLioRelocalizationNode::alignTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "fastlio2_relocalization_node started");
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

  double rotationAngleDeg(const Eigen::Matrix3d& R_a, const Eigen::Matrix3d& R_b) {
    Eigen::Matrix3d R_diff = R_a.transpose() * R_b;
    Eigen::AngleAxisd aa(R_diff);
    double angle_rad = std::abs(aa.angle());
    return angle_rad * 180.0 / M_PI;
  }

  bool loadMap(const std::string& path) {
    if (path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "map_pcd is empty");
      return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *map_cloud) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Could not read PCD: %s", path.c_str());
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded map: %zu points", map_cloud->size());

    map_points_.clear();
    map_points_.reserve(map_cloud->size());

    for (const auto& p : map_cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }

      map_points_.emplace_back(
        static_cast<double>(p.x),
        static_cast<double>(p.y),
        static_cast<double>(p.z));
    }

    RCLCPP_INFO(this->get_logger(), "Valid map points: %zu", map_points_.size());

    auto preprocessed = small_gicp::preprocess_points(
      map_points_,
      downsampling_resolution_,
      20,
      num_threads_);

    target_ = preprocessed.first;
    target_tree_ = preprocessed.second;

    if (!target_ || target_->empty()) {
      RCLCPP_ERROR(this->get_logger(), "Preprocessed map is empty");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Preprocessed map points: %zu", target_->size());
    return true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_ = msg;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_cloud_ = msg;
  }

  Eigen::Isometry3d odomMsgToEigen(const nav_msgs::msg::Odometry& odom) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    T.translation() = Eigen::Vector3d(
      odom.pose.pose.position.x,
      odom.pose.pose.position.y,
      odom.pose.pose.position.z);

    Eigen::Quaterniond q(
      odom.pose.pose.orientation.w,
      odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z);

    if (q.norm() < 1e-12) {
      T.linear() = Eigen::Matrix3d::Identity();
    } else {
      T.linear() = q.normalized().toRotationMatrix();
    }

    return T;
  }

  geometry_msgs::msg::PoseStamped eigenToPoseMsg(
    const Eigen::Isometry3d& T,
    const rclcpp::Time& stamp) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_;

    pose.pose.position.x = T.translation().x();
    pose.pose.position.y = T.translation().y();
    pose.pose.position.z = T.translation().z();

    Eigen::Quaterniond q(T.linear());
    q.normalize();

    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    return pose;
  }

  void sendMapToOdomTf(
    const Eigen::Isometry3d& T_map_odom,
    const rclcpp::Time& stamp) {
    Eigen::Quaterniond q(T_map_odom.linear());
    q.normalize();

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = odom_frame_;

    tf_msg.transform.translation.x = T_map_odom.translation().x();
    tf_msg.transform.translation.y = T_map_odom.translation().y();
    tf_msg.transform.translation.z = T_map_odom.translation().z();

    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
  }

  void publishLastMapToOdomIfAvailable(const rclcpp::Time& stamp) {
    if (!keep_last_tf_on_fail_) {
      return;
    }

    if (!has_last_map_odom_) {
      return;
    }

    sendMapToOdomTf(last_T_map_odom_, stamp);
  }

  void alignTimerCallback() {
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg;
    nav_msgs::msg::Odometry::SharedPtr odom_msg;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      cloud_msg = latest_cloud_;
      odom_msg = latest_odom_;
    }

    if (!cloud_msg || !odom_msg) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Waiting for cloud and odom...");
      return;
    }

    const rclcpp::Time cloud_stamp(cloud_msg->header.stamp);
    const rclcpp::Time odom_stamp(odom_msg->header.stamp);
    const bool cloud_stamp_valid = cloud_stamp.nanoseconds() != 0;

    if (max_cloud_odom_dt_ > 0.0) {
      const double cloud_odom_dt = std::abs((cloud_stamp - odom_stamp).seconds());
      if (cloud_odom_dt > max_cloud_odom_dt_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Skip reloc: cloud/odom timestamp delta %.3f s exceeds %.3f s",
          cloud_odom_dt,
          max_cloud_odom_dt_);
        publishLastMapToOdomIfAvailable(this->now());
        return;
      }
    }

    if (skip_repeated_cloud_ &&
        cloud_stamp_valid &&
        has_last_processed_cloud_stamp_ &&
        cloud_stamp == last_processed_cloud_stamp_) {
      publishLastMapToOdomIfAvailable(this->now());
      return;
    }

    if (cloud_stamp_valid) {
      last_processed_cloud_stamp_ = cloud_stamp;
      has_last_processed_cloud_stamp_ = true;
    }

    // =========================
    // 1. 读取当前帧点云
    // =========================
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*cloud_msg, pcl_cloud);

    std::vector<Eigen::Vector3d> source_points;
    source_points.reserve(pcl_cloud.size());

    // =========================
    // 2. 点云 body -> base_link
    // =========================
    for (const auto& p : pcl_cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }

      Eigen::Vector3d p_body(
        static_cast<double>(p.x),
        static_cast<double>(p.y),
        static_cast<double>(p.z)
      );

      Eigen::Vector3d p_base;
      if (cloud_to_base_enable_) {
        p_base = T_base_body_ * p_body;
      } else {
        p_base = p_body;
      }

      if (!std::isfinite(p_base.x()) || !std::isfinite(p_base.y()) || !std::isfinite(p_base.z())) {
        continue;
      }

      source_points.emplace_back(p_base);
    }

    if (source_points.size() < static_cast<size_t>(min_raw_source_points_)) {
      RCLCPP_WARN(this->get_logger(), "Too few raw source points: %zu", source_points.size());
      publishLastMapToOdomIfAvailable(cloud_msg->header.stamp);
      return;
    }

    // =========================
    // 3. odom -> base_link
    // =========================
    Eigen::Isometry3d T_odom_base = odomMsgToEigen(*odom_msg);

    // =========================
    // 4. 使用上一帧 map->odom 作为初值修正
    //
    // 之前：
    //   init_T_map_base = T_odom_base
    //
    // 优化后：
    //   init_T_map_base = last_T_map_odom * T_odom_base
    //
    // 这对快速移动非常重要
    // =========================
    Eigen::Isometry3d init_T_map_base = Eigen::Isometry3d::Identity();

    if (has_last_map_odom_) {
      init_T_map_base = last_T_map_odom_ * T_odom_base;
    } else {
      init_T_map_base = T_odom_base;
    }

    // =========================
    // 5. small_gicp 配准
    // =========================
    small_gicp::RegistrationSetting setting;
    setting.num_threads = num_threads_;
    setting.downsampling_resolution = downsampling_resolution_;
    setting.max_correspondence_distance = max_correspondence_distance_;

    auto source_preprocessed = small_gicp::preprocess_points(
      source_points,
      downsampling_resolution_,
      20,
      num_threads_);

    auto source = source_preprocessed.first;
    const size_t source_size = source->size();

    if (source_size < static_cast<size_t>(min_preprocessed_source_points_)) {
      RCLCPP_WARN(this->get_logger(), "Too few preprocessed source points: %zu", source_size);
      publishLastMapToOdomIfAvailable(cloud_msg->header.stamp);
      return;
    }

    auto result = small_gicp::align(
      *target_,
      *source,
      *target_tree_,
      init_T_map_base,
      setting);

    if (!result.converged) {
      RCLCPP_WARN(this->get_logger(), "small_gicp not converged");
      publishLastMapToOdomIfAvailable(cloud_msg->header.stamp);
      return;
    }

    Eigen::Isometry3d T_map_base = result.T_target_source;

    const double inlier_ratio =
      source_size > 0 ?
      static_cast<double>(result.num_inliers) / static_cast<double>(source_size) :
      0.0;

    // =========================
    // 6. inliers / ratio 门控
    // =========================
    if (static_cast<int>(result.num_inliers) < min_inliers_ ||
        inlier_ratio < min_inlier_ratio_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Reject reloc by inlier gate. inliers=%zu, source=%zu, ratio=%.3f, min_inliers=%d, min_ratio=%.3f",
        result.num_inliers,
        source_size,
        inlier_ratio,
        min_inliers_,
        min_inlier_ratio_
      );

      publishLastMapToOdomIfAvailable(cloud_msg->header.stamp);
      return;
    }

    // =========================
    // 7. 位姿跳变门控
    //
    // 第一帧没有历史，默认接受。
    // 后续如果 T_map_base 相比上次成功结果跳太大，则拒绝。
    // =========================
    if (has_last_map_base_) {
      const double trans_jump =
        (T_map_base.translation() - last_T_map_base_.translation()).norm();

      const double rot_jump_deg =
        rotationAngleDeg(last_T_map_base_.linear(), T_map_base.linear());

      if (trans_jump > max_translation_jump_ ||
          rot_jump_deg > max_rotation_jump_deg_) {
        RCLCPP_WARN(
          this->get_logger(),
          "Reject reloc by jump gate. trans_jump=%.3f m, rot_jump=%.3f deg, limit=(%.3f m, %.3f deg), inliers=%zu, ratio=%.3f",
          trans_jump,
          rot_jump_deg,
          max_translation_jump_,
          max_rotation_jump_deg_,
          result.num_inliers,
          inlier_ratio
        );

        publishLastMapToOdomIfAvailable(cloud_msg->header.stamp);
        return;
      }
    } else if (!accept_first_reloc_) {
      RCLCPP_WARN(this->get_logger(), "Reject first reloc because accept_first_reloc=false");
      publishLastMapToOdomIfAvailable(cloud_msg->header.stamp);
      return;
    }

    // =========================
    // 8. 计算并保存 map->odom
    //
    // T_map_odom = T_map_base * inverse(T_odom_base)
    // =========================
    Eigen::Isometry3d T_map_odom = T_map_base * T_odom_base.inverse();

    last_T_map_base_ = T_map_base;
    has_last_map_base_ = true;

    last_T_map_odom_ = T_map_odom;
    has_last_map_odom_ = true;

    // =========================
    // 9. 发布 /relocalization_pose
    // =========================
    auto pose_msg = eigenToPoseMsg(T_map_base, cloud_msg->header.stamp);
    pose_pub_->publish(pose_msg);

    // =========================
    // 10. 发布 map -> odom
    // =========================
    sendMapToOdomTf(T_map_odom, cloud_msg->header.stamp);

    RCLCPP_INFO(
      this->get_logger(),
      "Reloc ok. inliers=%zu, source=%zu, ratio=%.3f",
      result.num_inliers,
      source_size,
      inlier_ratio
    );
  }

private:
  std::string map_pcd_;
  std::string cloud_topic_;
  std::string odom_topic_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;

  double downsampling_resolution_;
  double max_correspondence_distance_;
  int num_threads_;
  double align_period_;
  double max_cloud_odom_dt_;
  bool skip_repeated_cloud_;
  int min_raw_source_points_;
  int min_preprocessed_source_points_;

  bool cloud_to_base_enable_;
  Eigen::Isometry3d T_base_body_;

  int min_inliers_;
  double min_inlier_ratio_;
  double max_translation_jump_;
  double max_rotation_jump_deg_;
  bool keep_last_tf_on_fail_;
  bool accept_first_reloc_;

  Eigen::Isometry3d last_T_map_odom_;
  bool has_last_map_odom_;

  Eigen::Isometry3d last_T_map_base_;
  bool has_last_map_base_;

  rclcpp::Time last_processed_cloud_stamp_;
  bool has_last_processed_cloud_stamp_;

  std::mutex mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<Eigen::Vector3d> map_points_;

  small_gicp::PointCloud::Ptr target_;
  small_gicp::KdTree<small_gicp::PointCloud>::Ptr target_tree_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FastLioRelocalizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
