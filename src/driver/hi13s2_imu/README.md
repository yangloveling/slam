# hi13s2_imu

ROS 2 驱动包 for HiPNUC HI13S2 IMU。

## 节点
- `hi13s2_node`：IMU 数据读取，发布 `/imu/data`
- `yaw_publisher`：计算并发布偏航角（度），支持低通滤波和变化阈值
- `heading_controller`：PID 朝向控制，订阅 `/yaw` 输出 `/cmd_vel`

## 参数
详见各节点对应的 YAML 配置文件。

## 使用方法
```bash
ros2 launch hi13s2_imu imu_only.launch.py