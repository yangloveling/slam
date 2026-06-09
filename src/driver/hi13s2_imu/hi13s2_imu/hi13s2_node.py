#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import struct
import math
import time

FRAME_HEAD = b'\x5A\xA5'


class Hi13s2Node(Node):
    def __init__(self):
        super().__init__('hi13s2_node')

        # 声明参数
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('mode', 'auto')          # auto / hi91 / hi92
        self.declare_parameter('debug', False)
        self.declare_parameter('normalize_quat', False) # 强制归一化四元数

        port = self.get_parameter('port').value
        self.get_logger().info(f"Using port: {port}")  
        baud = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.mode = self.get_parameter('mode').value
        self.debug = self.get_parameter('debug').value
        self.normalize_quat = self.get_parameter('normalize_quat').value

        # 初始化串口
        self.ser = serial.Serial(port, baud, timeout=0.001)
        self.pub = self.create_publisher(Imu, '/imu/data', 50)
        self.buffer = bytearray()
        self.timer = self.create_timer(0.001, self.read_serial)

        self.frame_count = 0
        self.error_count = 0
        self.last_print = time.time()

        self.get_logger().info(f"HI13S2 node started on {port}@{baud}")
        self.get_logger().info("Waiting for data... Press Ctrl+C to stop")

    def read_serial(self):
        try:
            data = self.ser.read(2048)
            if data:
                self.buffer.extend(data)
                if len(self.buffer) > 8192:
                    self.buffer = self.buffer[-4096:]
                self.process_buffer()
        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")

        now = time.time()
        if now - self.last_print > 3.0:
            total = self.frame_count + self.error_count
            rate = self.frame_count / 3.0 if total > 0 else 0
            self.get_logger().info(f"Good: {self.frame_count}, Bad: {self.error_count}, Rate: {rate:.1f}Hz, Buf: {len(self.buffer)}")
            self.frame_count = 0
            self.error_count = 0
            self.last_print = now

    def process_buffer(self):
        while len(self.buffer) >= 6:
            idx = self.buffer.find(FRAME_HEAD)
            if idx < 0:
                self.buffer.clear()
                return

            if idx > 0:
                self.get_logger().warn(f"Discarding {idx} bytes before frame head")
                self.buffer = self.buffer[idx:]

            if len(self.buffer) < 6:
                return

            payload_len = struct.unpack('<H', self.buffer[2:4])[0]
            frame_len = 6 + payload_len

            if payload_len > 256 or payload_len < 4:
                self.get_logger().warn(f"Invalid payload_len: {payload_len}, skipping head")
                self.buffer = self.buffer[2:]
                self.error_count += 1
                continue

            if len(self.buffer) < frame_len:
                return

            frame = self.buffer[:frame_len]
            self.buffer = self.buffer[frame_len:]

            payload = frame[6:]
            tag = payload[0]

            # 根据模式选择处理函数
            if self.mode == 'hi91' or (self.mode == 'auto' and tag == 0x91 and payload_len == 76):
                self.parse_hi91(payload)
            elif self.mode == 'hi92' or (self.mode == 'auto' and tag == 0x92 and payload_len == 48):
                self.parse_hi92(payload)
            else:
                self.get_logger().warn(f"Unknown frame: tag={tag:#x}, len={payload_len}")
                if self.debug:
                    self.get_logger().info(f"Raw hex: {payload[:20].hex()}")
                self.error_count += 1

    def parse_hi91(self, payload):
        """
        解析 HI91 帧（浮点输出），结构依据官方 hipnuc_dec.h：
        tag(1) + main_status(2) + temp(1) + air_pressure(4) + system_time(4) +
        acc(3x4) + gyr(3x4) + mag(3x4) + roll(4) + pitch(4) + yaw(4) + quat(4x4) = 76 字节
        """
        try:
            if len(payload) < 76:
                self.get_logger().warn(f"HI91 payload too short: {len(payload)} < 76")
                self.error_count += 1
                return

            offset = 0
            tag = payload[offset]; offset += 1
            main_status = struct.unpack('<H', payload[offset:offset+2])[0]; offset += 2
            temp = struct.unpack('<b', payload[offset:offset+1])[0]; offset += 1          # int8_t
            air_pressure = struct.unpack('<f', payload[offset:offset+4])[0]; offset += 4  # float, Pa
            system_time = struct.unpack('<I', payload[offset:offset+4])[0]; offset += 4   # uint32, ms

            # 加速度 (3 float) 单位：g
            acc = struct.unpack('<fff', payload[offset:offset+12]); offset += 12
            # 角速度 (3 float) 单位：°/s
            gyr = struct.unpack('<fff', payload[offset:offset+12]); offset += 12
            # 磁力计 (3 float) 单位：uT
            mag = struct.unpack('<fff', payload[offset:offset+12]); offset += 12
            # 欧拉角 (3 float) 单位：°
            roll, pitch, yaw = struct.unpack('<fff', payload[offset:offset+12]); offset += 12
            # 四元数 (4 float) 顺序：w, x, y, z
            quat = struct.unpack('<ffff', payload[offset:offset+16]); offset += 16
            qw, qx, qy, qz = quat

            # 单位转换（与官方C++节点一致）
            ax, ay, az = [a * 9.80665 for a in acc]          # g → m/s²
            gx, gy, gz = [math.radians(g) for g in gyr]      # °/s → rad/s

            # 数值有效性检查
            if not all(math.isfinite(v) for v in [ax, ay, az, gx, gy, gz, qx, qy, qz, qw]):
                self.get_logger().warn("HI91: Non-finite value detected")
                self.error_count += 1
                return

            # 物理范围检查（根据实际传感器性能调整阈值）
            if max(abs(ax), abs(ay), abs(az)) > 100:   # 超过10g
                self.get_logger().warn(f"HI91: Acceleration out of range: ({ax:.2f}, {ay:.2f}, {az:.2f})")
                self.error_count += 1
                return
            if max(abs(gx), abs(gy), abs(gz)) > 100:    # 超过100 rad/s (~5700°/s)
                self.get_logger().warn(f"HI91: Angular velocity out of range: ({gx:.2f}, {gy:.2f}, {gz:.2f})")
                self.error_count += 1
                return

            # 四元数归一化检查
            norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
            if abs(norm - 1.0) > 0.1:
                self.get_logger().warn(f"HI91: Quaternion norm {norm} too far from 1")
                if self.normalize_quat and norm > 1e-12:
                    inv_norm = 1.0 / norm
                    qw *= inv_norm
                    qx *= inv_norm
                    qy *= inv_norm
                    qz *= inv_norm
                    self.get_logger().info("Quaternion normalized")
                else:
                    if self.debug:
                        self.get_logger().info(f"  qw={qw:.6f}, qx={qx:.6f}, qy={qy:.6f}, qz={qz:.6f}")
                    self.error_count += 1
                    return

            # 调试输出
            if self.debug:
                self.get_logger().info(f"HI91 parsed: acc=({ax:.2f},{ay:.2f},{az:.2f}), "
                                       f"gyro=({gx:.2f},{gy:.2f},{gz:.2f}), "
                                       f"quat=({qw:.3f},{qx:.3f},{qy:.3f},{qz:.3f})")

            self.publish_imu(ax, ay, az, gx, gy, gz, qx, qy, qz, qw)
            self.frame_count += 1

        except Exception as e:
            self.get_logger().warn(f"HI91 parse error: {e}")
            self.error_count += 1

    def parse_hi92(self, payload):
        """
        解析 HI92 帧（原始 int16 输出），结构依据原始Python代码和官方hi81_t（类似）
        tag(1) + unknown(2) + acc(3x2) + gyr(3x2) + mag(3x2) + unknown(3x2) + quat(4x2) = 48 字节
        """
        try:
            if len(payload) < 48:
                self.get_logger().warn(f"HI92 payload too short: {len(payload)} < 48")
                self.error_count += 1
                return

            offset = 1 + 2        # tag + 未知2字节
            ax_raw, ay_raw, az_raw = struct.unpack('<hhh', payload[offset:offset+6]); offset += 6
            gx_raw, gy_raw, gz_raw = struct.unpack('<hhh', payload[offset:offset+6]); offset += 6
            offset += 6            # 跳过磁力计
            offset += 6            # 跳过其他
            qw_raw, qx_raw, qy_raw, qz_raw = struct.unpack('<hhhh', payload[offset:offset+8])

            # 缩放因子（根据原始代码）
            ax = ax_raw * 0.001 * 9.80665
            ay = ay_raw * 0.001 * 9.80665
            az = az_raw * 0.001 * 9.80665

            gx = gx_raw * 0.01 * math.pi / 180.0
            gy = gy_raw * 0.01 * math.pi / 180.0
            gz = gz_raw * 0.01 * math.pi / 180.0

            qw = qw_raw / 32768.0
            qx = qx_raw / 32768.0
            qy = qy_raw / 32768.0
            qz = qz_raw / 32768.0

            # 数值检查
            if not all(math.isfinite(v) for v in [ax, ay, az, gx, gy, gz, qw, qx, qy, qz]):
                self.get_logger().warn("HI92: Non-finite value detected")
                self.error_count += 1
                return

            if max(abs(ax), abs(ay), abs(az)) > 100:
                self.error_count += 1
                return
            if max(abs(gx), abs(gy), abs(gz)) > 100:
                self.error_count += 1
                return

            norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
            if abs(norm - 1.0) > 0.1:
                self.get_logger().warn(f"HI92: Quaternion norm {norm} too far from 1")
                if self.normalize_quat and norm > 1e-12:
                    inv_norm = 1.0 / norm
                    qw *= inv_norm
                    qx *= inv_norm
                    qy *= inv_norm
                    qz *= inv_norm
                    self.get_logger().info("Quaternion normalized")
                else:
                    self.error_count += 1
                    return

            if self.debug:
                self.get_logger().info(f"HI92 parsed: acc=({ax:.2f},{ay:.2f},{az:.2f}), "
                                       f"gyro=({gx:.2f},{gy:.2f},{gz:.2f}), "
                                       f"quat=({qw:.3f},{qx:.3f},{qy:.3f},{qz:.3f})")

            self.publish_imu(ax, ay, az, gx, gy, gz, qx, qy, qz, qw)
            self.frame_count += 1

        except Exception as e:
            self.get_logger().warn(f"HI92 parse error: {e}")
            self.error_count += 1

    def publish_imu(self, ax, ay, az, gx, gy, gz, qx, qy, qz, qw):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        # 协方差矩阵（可调节）
        msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
        msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Hi13s2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()