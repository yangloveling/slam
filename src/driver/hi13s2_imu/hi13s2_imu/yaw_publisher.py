#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math

class YawPublisher(Node):
    def __init__(self):
        super().__init__('yaw_publisher')

        # 声明参数
        self.declare_parameter('alpha', 0.1)               # 低通滤波系数
        self.declare_parameter('change_threshold_deg', 1.0) # 变化阈值（度）

        self.alpha = self.get_parameter('alpha').value
        self.threshold = self.get_parameter('change_threshold_deg').value

        self.sub = self.create_subscription(Imu, '/livox/imu', self.imu_callback, 10)
        self.pub = self.create_publisher(Float64, '/yaw', 10)

        self.filtered_yaw = None          # 滤波后的弧度值
        self.last_published_yaw_deg = None # 上次发布的度数

    def imu_callback(self, msg):
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # 计算原始偏航角（弧度）
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        raw_yaw = math.atan2(siny_cosp, cosy_cosp)

        # 低通滤波（处理角度跳变）
        if self.filtered_yaw is None:
            self.filtered_yaw = raw_yaw
        else:
            diff = raw_yaw - self.filtered_yaw
            if diff > math.pi:
                raw_yaw -= 2 * math.pi
            elif diff < -math.pi:
                raw_yaw += 2 * math.pi
            self.filtered_yaw = self.alpha * raw_yaw + (1 - self.alpha) * self.filtered_yaw

        # 转换为度
        yaw_deg = self.filtered_yaw * 180.0 / math.pi

        # 判断变化是否超过阈值
        if (self.last_published_yaw_deg is None or
            abs(yaw_deg - self.last_published_yaw_deg) >= self.threshold):
            self.pub.publish(Float64(data=yaw_deg))
            self.last_published_yaw_deg = yaw_deg

def main(args=None):
    rclpy.init(args=args)
    node = YawPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
