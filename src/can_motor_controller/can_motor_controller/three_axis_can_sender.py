#!/usr/bin/env python3
"""Send velocity and yaw commands to a three-axis CAN controller."""

import time
import math
import struct

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16MultiArray, Float64
from nav_msgs.msg import Odometry

import can


class ThreeAxisCanSender(Node):
    """
    Send target angle and velocity payloads to the CAN controller.

    CAN payload default mode:
        int16 target_angle_deg
        int16 vx
        int16 vy
        int16 yaw_deg

    Optional payload mode:
        int16 target_angle_deg
        int16 vx
        int16 vy
        int16 wz_deg_s
    """

    def __init__(self):
        super().__init__('three_axis_can_sender')

        # ================= 参数声明 =================
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_id', 0x51)
        self.declare_parameter('auto_reconnect_can', True)
        self.declare_parameter('can_reconnect_interval', 1.0)
        self.declare_parameter('fail_fast_on_can_init', False)

        self.declare_parameter('send_rate_hz', 50.0)

        self.declare_parameter('rotate_cmd_topic', '/rotate_cmd')
        self.declare_parameter('motor_velocity_topic', '/motor_velocity')
        self.declare_parameter('angular_velocity_topic', '/angular_velocity_cmd')
        self.declare_parameter('odom_topic', '/odometry/filtered')

        self.declare_parameter('velocity_timeout', 0.8)
        self.declare_parameter('rotate_timeout', 1.0)
        self.declare_parameter('wz_timeout', 0.8)
        self.declare_parameter('odom_timeout', 1.0)

        self.declare_parameter('log_interval_sec', 0.2)
        self.declare_parameter('warn_interval_sec', 1.0)

        self.declare_parameter('zero_on_velocity_timeout', True)
        self.declare_parameter('zero_on_wz_timeout', True)
        self.declare_parameter('require_odom_before_send', False)
        self.declare_parameter('init_target_from_odom_if_no_rotate_cmd', True)

        # rotate_cmd_mode:
        #   absolute: /rotate_cmd 是绝对目标角
        #   relative: /rotate_cmd 是相对当前 yaw 的增量角
        self.declare_parameter('rotate_cmd_mode', 'absolute')

        # target_timeout_behavior:
        #   hold_last: 超时后继续保持最后目标角，兼容原逻辑
        #   snap_to_yaw: 超时后目标角变成当前 yaw，不再追旧目标
        #   zero: 超时后目标角变成 0，不太推荐
        self.declare_parameter('target_timeout_behavior', 'hold_last')

        # payload_mode:
        #   target_vx_vy_yaw: 发送 target, vx, vy, yaw，兼容原单片机协议
        #   target_vx_vy_wz: 发送 target, vx, vy, wz
        self.declare_parameter('payload_mode', 'target_vx_vy_yaw')

        # wz_source:
        #   cmd: 使用 /angular_velocity_cmd
        #   odom: 使用 odom.twist.twist.angular.z
        #   zero: 始终发送 0
        self.declare_parameter('wz_source', 'cmd')

        # angular_velocity_cmd_unit:
        #   deg_s: /angular_velocity_cmd 单位为 度/秒
        #   rad_s: /angular_velocity_cmd 单位为 弧度/秒
        self.declare_parameter('angular_velocity_cmd_unit', 'deg_s')

        # /motor_velocity 数据索引
        self.declare_parameter('vx_index', 1)
        self.declare_parameter('vy_index', 2)

        # 目标附近锁定参数
        self.declare_parameter('enable_target_hold', True)
        self.declare_parameter('reach_hold_deg', 2.0)
        self.declare_parameter('release_hold_deg', 5.0)
        self.declare_parameter('reach_hold_count', 3)

        # ================= 读取参数 =================
        self.can_interface = self.get_parameter('can_interface').value
        self.can_id = int(self.get_parameter('can_id').value)
        self.auto_reconnect_can = bool(self.get_parameter('auto_reconnect_can').value)
        self.can_reconnect_interval = float(self.get_parameter('can_reconnect_interval').value)
        self.fail_fast_on_can_init = bool(self.get_parameter('fail_fast_on_can_init').value)

        self.send_rate_hz = float(self.get_parameter('send_rate_hz').value)

        self.rotate_cmd_topic = self.get_parameter('rotate_cmd_topic').value
        self.motor_velocity_topic = self.get_parameter('motor_velocity_topic').value
        self.angular_velocity_topic = self.get_parameter('angular_velocity_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value

        self.velocity_timeout = float(self.get_parameter('velocity_timeout').value)
        self.rotate_timeout = float(self.get_parameter('rotate_timeout').value)
        self.wz_timeout = float(self.get_parameter('wz_timeout').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout').value)

        self.log_interval_sec = float(self.get_parameter('log_interval_sec').value)
        self.warn_interval_sec = float(self.get_parameter('warn_interval_sec').value)

        self.zero_on_velocity_timeout = bool(self.get_parameter('zero_on_velocity_timeout').value)
        self.zero_on_wz_timeout = bool(self.get_parameter('zero_on_wz_timeout').value)
        self.require_odom_before_send = bool(self.get_parameter('require_odom_before_send').value)
        self.init_target_from_odom_if_no_rotate_cmd = bool(
            self.get_parameter('init_target_from_odom_if_no_rotate_cmd').value
        )

        self.rotate_cmd_mode = str(self.get_parameter('rotate_cmd_mode').value).lower()
        self.target_timeout_behavior = str(
            self.get_parameter('target_timeout_behavior').value
        ).lower()
        self.payload_mode = str(self.get_parameter('payload_mode').value).lower()
        self.wz_source = str(self.get_parameter('wz_source').value).lower()
        self.angular_velocity_cmd_unit = str(
            self.get_parameter('angular_velocity_cmd_unit').value
        ).lower()

        self.vx_index = int(self.get_parameter('vx_index').value)
        self.vy_index = int(self.get_parameter('vy_index').value)

        self.enable_target_hold = bool(self.get_parameter('enable_target_hold').value)
        self.reach_hold_deg = float(self.get_parameter('reach_hold_deg').value)
        self.release_hold_deg = float(self.get_parameter('release_hold_deg').value)
        self.reach_hold_count = int(self.get_parameter('reach_hold_count').value)

        # ================= 参数保护 =================
        self.send_rate_hz = max(1.0, self.send_rate_hz)
        self.velocity_timeout = max(0.01, self.velocity_timeout)
        self.rotate_timeout = max(0.01, self.rotate_timeout)
        self.wz_timeout = max(0.01, self.wz_timeout)
        self.odom_timeout = max(0.01, self.odom_timeout)
        self.log_interval_sec = max(0.05, self.log_interval_sec)
        self.warn_interval_sec = max(0.1, self.warn_interval_sec)
        self.can_reconnect_interval = max(0.2, self.can_reconnect_interval)

        self.vx_index = max(0, self.vx_index)
        self.vy_index = max(0, self.vy_index)

        self.reach_hold_deg = max(0.1, self.reach_hold_deg)
        self.release_hold_deg = max(self.reach_hold_deg, self.release_hold_deg)
        self.reach_hold_count = max(1, self.reach_hold_count)

        self.validate_string_params()

        # ================= 当前状态 =================
        self.current_target_angle_deg = 0.0
        self.current_vx = 0
        self.current_vy = 0
        self.current_yaw_deg = 0.0
        self.current_wz_cmd_deg_s = 0.0
        self.current_wz_odom_deg_s = 0.0

        self.received_rotate = False
        self.received_velocity = False
        self.received_wz_cmd = False
        self.received_odom = False

        now = time.monotonic()
        self.last_rotate_time = now
        self.last_velocity_time = now
        self.last_wz_cmd_time = now
        self.last_odom_time = now

        self.last_log_time = 0.0
        self.last_warn_time = 0.0
        self.last_can_reconnect_time = 0.0

        # 目标附近锁定状态
        self.target_hold_active = False
        self.reach_counter = 0

        # ================= 初始化 CAN =================
        self.can_bus = None
        self.init_can_bus()

        # ================= 订阅 =================
        self.sub_angle = self.create_subscription(
            Float64,
            self.rotate_cmd_topic,
            self.angle_callback,
            10
        )

        self.sub_vel = self.create_subscription(
            Int16MultiArray,
            self.motor_velocity_topic,
            self.velocity_callback,
            10
        )

        self.sub_wz = self.create_subscription(
            Float64,
            self.angular_velocity_topic,
            self.angular_velocity_callback,
            10
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            20
        )

        # ================= 定时器 =================
        timer_period = 1.0 / self.send_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f"ThreeAxisCanSender started: "
            f"can={self.can_interface}, can_id=0x{self.can_id:X}, "
            f"rate={self.send_rate_hz:.1f}Hz, "
            f"payload_mode={self.payload_mode}, "
            f"rotate_cmd_mode={self.rotate_cmd_mode}, "
            f"target_timeout_behavior={self.target_timeout_behavior}, "
            f"vx_index={self.vx_index}, vy_index={self.vy_index}, "
            f"hold(enable={self.enable_target_hold}, "
            f"reach={self.reach_hold_deg:.1f}deg, "
            f"release={self.release_hold_deg:.1f}deg, "
            f"count={self.reach_hold_count})"
        )

    # ================= 参数检查 =================
    def validate_string_params(self):
        if self.rotate_cmd_mode not in ['absolute', 'relative']:
            self.get_logger().warn(
                f"Invalid rotate_cmd_mode={self.rotate_cmd_mode}, fallback to absolute"
            )
            self.rotate_cmd_mode = 'absolute'

        if self.target_timeout_behavior not in ['hold_last', 'snap_to_yaw', 'zero']:
            self.get_logger().warn(
                f"Invalid target_timeout_behavior={self.target_timeout_behavior}, "
                f"fallback to hold_last"
            )
            self.target_timeout_behavior = 'hold_last'

        if self.payload_mode not in ['target_vx_vy_yaw', 'target_vx_vy_wz']:
            self.get_logger().warn(
                f"Invalid payload_mode={self.payload_mode}, fallback to target_vx_vy_yaw"
            )
            self.payload_mode = 'target_vx_vy_yaw'

        if self.wz_source not in ['cmd', 'odom', 'zero']:
            self.get_logger().warn(
                f"Invalid wz_source={self.wz_source}, fallback to cmd"
            )
            self.wz_source = 'cmd'

        if self.angular_velocity_cmd_unit not in ['deg_s', 'rad_s']:
            self.get_logger().warn(
                f"Invalid angular_velocity_cmd_unit={self.angular_velocity_cmd_unit}, "
                f"fallback to deg_s"
            )
            self.angular_velocity_cmd_unit = 'deg_s'

    # ================= CAN =================
    def init_can_bus(self):
        try:
            try:
                self.can_bus = can.interface.Bus(
                    channel=self.can_interface,
                    interface='socketcan'
                )
            except TypeError:
                self.can_bus = can.interface.Bus(
                    channel=self.can_interface,
                    bustype='socketcan'
                )

            self.get_logger().info(
                f"CAN bus initialized on {self.can_interface}, can_id=0x{self.can_id:X}"
            )
            return True

        except Exception as e:
            self.can_bus = None

            if self.fail_fast_on_can_init:
                self.get_logger().error(f"Failed to initialize CAN: {e}")
                raise

            self.get_logger().warn(
                f"CAN interface {self.can_interface} is unavailable: {e}. "
                "Disable CAN output until reconnect succeeds."
            )
            return False

    def try_reconnect_can(self):
        if not self.auto_reconnect_can:
            return

        now = time.monotonic()
        if (now - self.last_can_reconnect_time) < self.can_reconnect_interval:
            return

        self.last_can_reconnect_time = now
        self.throttled_warn("CAN bus unavailable, trying to reconnect...")
        self.init_can_bus()

    # ================= 工具函数 =================
    def normalize_angle_360(self, angle_deg: float) -> float:
        a = math.fmod(angle_deg, 360.0)
        if a < 0.0:
            a += 360.0
        return a

    def normalize_angle_180(self, angle_deg: float) -> float:
        a = math.fmod(angle_deg + 180.0, 360.0)
        if a < 0.0:
            a += 360.0
        return a - 180.0

    def shortest_signed_angle_diff_deg(self, target: float, current: float) -> float:
        return self.normalize_angle_180(target - current)

    def clamp_int16(self, value) -> int:
        return max(-32768, min(32767, int(value)))

    def angle_to_int16_deg180(self, angle_deg: float) -> int:
        return int(round(self.normalize_angle_180(angle_deg)))

    def quaternion_to_yaw_deg(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        return self.normalize_angle_180(math.degrees(yaw_rad))

    def is_finite(self, value) -> bool:
        try:
            return math.isfinite(float(value))
        except Exception:
            return False

    def throttled_info(self, msg: str):
        now = time.monotonic()
        if (now - self.last_log_time) >= self.log_interval_sec:
            self.get_logger().info(msg)
            self.last_log_time = now

    def throttled_warn(self, msg: str):
        now = time.monotonic()
        if (now - self.last_warn_time) >= self.warn_interval_sec:
            self.get_logger().warn(msg)
            self.last_warn_time = now

    # ================= 回调 =================
    def angle_callback(self, msg: Float64):
        if not self.is_finite(msg.data):
            self.throttled_warn(f"Invalid /rotate_cmd: {msg.data}")
            return

        cmd_deg = float(msg.data)

        if self.rotate_cmd_mode == 'absolute':
            target_deg = cmd_deg
        else:
            # relative 模式下，优先基于当前 odom yaw 做相对转角
            if self.received_odom:
                target_deg = self.current_yaw_deg + cmd_deg
            else:
                # 没有 odom 时，只能基于上一次目标角累加
                target_deg = self.current_target_angle_deg + cmd_deg
                self.throttled_warn(
                    "rotate_cmd_mode=relative but odometry not received, "
                    "using previous target as base"
                )

        self.current_target_angle_deg = self.normalize_angle_180(target_deg)
        self.received_rotate = True
        self.last_rotate_time = time.monotonic()

        # 新目标到来时解除锁定
        self.target_hold_active = False
        self.reach_counter = 0

        self.throttled_info(
            f"Rotate cmd received: mode={self.rotate_cmd_mode}, "
            f"cmd={cmd_deg:.2f}, target={self.current_target_angle_deg:.2f}"
        )

    def velocity_callback(self, msg: Int16MultiArray):
        max_index = max(self.vx_index, self.vy_index)
        if len(msg.data) <= max_index:
            self.throttled_warn(
                f"{self.motor_velocity_topic} length too short, "
                f"need index {max_index}, got length={len(msg.data)}"
            )
            return

        self.current_vx = self.clamp_int16(msg.data[self.vx_index])
        self.current_vy = self.clamp_int16(msg.data[self.vy_index])

        self.received_velocity = True
        self.last_velocity_time = time.monotonic()

    def angular_velocity_callback(self, msg: Float64):
        if not self.is_finite(msg.data):
            self.throttled_warn(f"Invalid angular velocity cmd: {msg.data}")
            return

        value = float(msg.data)

        if self.angular_velocity_cmd_unit == 'rad_s':
            value = math.degrees(value)

        self.current_wz_cmd_deg_s = value
        self.received_wz_cmd = True
        self.last_wz_cmd_time = time.monotonic()

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.current_yaw_deg = self.quaternion_to_yaw_deg(q.x, q.y, q.z, q.w)

        # odom.twist.twist.angular.z 默认单位是 rad/s
        self.current_wz_odom_deg_s = math.degrees(msg.twist.twist.angular.z)

        self.received_odom = True
        self.last_odom_time = time.monotonic()

    # ================= 状态生成 =================
    def get_effective_target_angle(self, now: float) -> float:
        yaw_deg = self.current_yaw_deg
        target_deg = self.current_target_angle_deg

        rotate_timed_out = (
            self.received_rotate and
            ((now - self.last_rotate_time) > self.rotate_timeout)
        )

        if not self.received_rotate:
            if self.init_target_from_odom_if_no_rotate_cmd and self.received_odom:
                target_deg = yaw_deg
        else:
            if rotate_timed_out:
                if self.target_timeout_behavior == 'hold_last':
                    # 兼容原始逻辑：保持最后一次目标角
                    pass
                elif self.target_timeout_behavior == 'snap_to_yaw':
                    if self.received_odom:
                        target_deg = yaw_deg
                elif self.target_timeout_behavior == 'zero':
                    target_deg = 0.0

        should_apply_target_hold = True
        if rotate_timed_out and self.target_timeout_behavior != 'hold_last':
            should_apply_target_hold = False

        if self.enable_target_hold and self.received_odom and self.received_rotate:
            if should_apply_target_hold:
                angle_err = self.shortest_signed_angle_diff_deg(target_deg, yaw_deg)
                abs_err = abs(angle_err)

                if self.target_hold_active:
                    if abs_err > self.release_hold_deg:
                        self.target_hold_active = False
                        self.reach_counter = 0
                    else:
                        # 到达目标附近后，让 target 跟随当前 yaw，减小抖动
                        target_deg = yaw_deg
                else:
                    if abs_err <= self.reach_hold_deg:
                        self.reach_counter += 1
                        if self.reach_counter >= self.reach_hold_count:
                            self.target_hold_active = True
                            target_deg = yaw_deg
                    else:
                        self.reach_counter = 0

        return self.normalize_angle_180(target_deg)

    def get_effective_velocity(self, now: float):
        vx = self.current_vx
        vy = self.current_vy

        if self.zero_on_velocity_timeout:
            if (
                not self.received_velocity or
                ((now - self.last_velocity_time) > self.velocity_timeout)
            ):
                vx = 0
                vy = 0

        return self.clamp_int16(vx), self.clamp_int16(vy)

    def get_effective_wz_deg_s(self, now: float) -> float:
        if self.wz_source == 'zero':
            return 0.0

        if self.wz_source == 'odom':
            if not self.received_odom:
                return 0.0
            return self.current_wz_odom_deg_s

        # self.wz_source == 'cmd'
        wz = self.current_wz_cmd_deg_s

        if self.zero_on_wz_timeout:
            if (
                not self.received_wz_cmd or
                ((now - self.last_wz_cmd_time) > self.wz_timeout)
            ):
                wz = 0.0

        return wz

    def get_effective_state(self):
        now = time.monotonic()

        target_angle_deg = self.get_effective_target_angle(now)
        vx, vy = self.get_effective_velocity(now)
        yaw_deg = self.normalize_angle_180(self.current_yaw_deg)
        wz_deg_s = self.get_effective_wz_deg_s(now)

        return target_angle_deg, vx, vy, yaw_deg, wz_deg_s

    # ================= 发送逻辑 =================
    def timer_callback(self):
        if self.can_bus is None:
            self.try_reconnect_can()
            return

        now = time.monotonic()

        if self.require_odom_before_send and not self.received_odom:
            self.throttled_warn(
                "require_odom_before_send=True but odometry not received yet"
            )
            return

        if self.received_odom and (now - self.last_odom_time) > self.odom_timeout:
            self.throttled_warn(
                f"odometry timeout: age={now - self.last_odom_time:.2f}s"
            )

        target_angle_deg, vx, vy, yaw_deg, wz_deg_s = self.get_effective_state()
        angle_err = self.shortest_signed_angle_diff_deg(target_angle_deg, yaw_deg)

        if self.send_can_frame(target_angle_deg, vx, vy, yaw_deg, wz_deg_s):
            if self.payload_mode == 'target_vx_vy_yaw':
                fourth_name = 'yaw'
                fourth_value = self.angle_to_int16_deg180(yaw_deg)
            else:
                fourth_name = 'wz'
                fourth_value = self.clamp_int16(round(wz_deg_s))

            self.throttled_info(
                f"CAN sent: target={self.angle_to_int16_deg180(target_angle_deg)}, "
                f"yaw={self.angle_to_int16_deg180(yaw_deg)}, "
                f"err={angle_err:.2f}, "
                f"vx={vx}, vy={vy}, "
                f"{fourth_name}={fourth_value}, "
                f"hold={int(self.target_hold_active)}"
            )

    def send_can_frame(
        self,
        target_angle_deg: float,
        vx: int,
        vy: int,
        yaw_deg: float,
        wz_deg_s: float
    ) -> bool:
        angle_raw = self.clamp_int16(self.angle_to_int16_deg180(target_angle_deg))
        vx_raw = self.clamp_int16(vx)
        vy_raw = self.clamp_int16(vy)

        if self.payload_mode == 'target_vx_vy_yaw':
            fourth_raw = self.clamp_int16(self.angle_to_int16_deg180(yaw_deg))
        else:
            fourth_raw = self.clamp_int16(round(wz_deg_s))

        # 大端 int16，兼容原来的高字节在前
        data = struct.pack('>hhhh', angle_raw, vx_raw, vy_raw, fourth_raw)

        can_msg = can.Message(
            arbitration_id=self.can_id,
            data=data,
            is_extended_id=False,
            dlc=8
        )

        try:
            self.can_bus.send(can_msg)
            return True

        except can.CanError as e:
            self.throttled_warn(f"CAN send error: {e}")
            self.close_can_bus()
            return False

        except Exception as e:
            self.throttled_warn(f"Unexpected CAN send exception: {e}")
            self.close_can_bus()
            return False

    # ================= 销毁 =================
    def close_can_bus(self):
        try:
            if self.can_bus is not None:
                self.can_bus.shutdown()
        except Exception as e:
            self.throttled_warn(f"Error during CAN shutdown: {e}")
        finally:
            self.can_bus = None

    def destroy_node(self):
        try:
            self.close_can_bus()
            self.get_logger().info("CAN bus shutdown complete")
        except Exception as e:
            self.get_logger().warn(f"Error during node destroy: {e}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ThreeAxisCanSender()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        if node is not None:
            node.get_logger().error(f"Node exception: {e}")
        else:
            print(f"[three_axis_can_sender] init exception: {e}")

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
