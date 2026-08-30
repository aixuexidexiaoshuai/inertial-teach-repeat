#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
里程计监控 / 校准诊断工具。

订阅 EKF 融合里程计 /odom_combined，把开机第一帧锁为原点（起始角度 0°、x=0、y=0），
每秒打印相对原点的：方向角(度)、x、y（单位 cm）。

归零方式：第一帧位姿 (x0,y0,yaw0) 作为新坐标系，做平移 + 旋转到起始朝向，使
起始恒为 (0°, 0, 0)。因此：
  - 车不动 → 一直 ~0（正好看里程计漂移/IMU 零偏）
  - 直走   → x 增大（看距离标度准不准）
  - 原地左转 → 方向角增大（看角度标度准不准）

随时重新归零（不必重启）：
    ros2 topic pub --once /odom_monitor/reset std_msgs/msg/Empty "{}"

用法（惯导链路已在跑时）：
    ros2 run ahpu_odom_nav odom_monitor

典型用法：
  · 静置几分钟看方向角变化 = 直接量【航向漂移率】（本方案实测 ~2°/6min）
  · 推着车直走 5m 看 x 读数 = 校轮径/里程计标度
  · 原地转 90° 看方向角 = 校陀螺标度与符号（左转应【增大】，反了设 gyro_z_sign:=-1.0）
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class OdomMonitor(Node):
    def __init__(self):
        super().__init__("odom_monitor")
        self.declare_parameter("odom_topic", "/odom_combined")
        self.declare_parameter("rate", 1.0)  # 打印频率 Hz

        topic = self.get_parameter("odom_topic").value
        rate = float(self.get_parameter("rate").value)

        self.origin = None   # (x0, y0, yaw0) 锁定的原点
        self.cur = None      # 最新原始位姿 (x, y, yaw)

        self.create_subscription(Odometry, topic, self.on_odom, 20)
        self.create_subscription(Empty, "/odom_monitor/reset", self.on_reset, 1)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        self.get_logger().info(
            f"里程计监控启动，订阅 {topic}；"
            f"发 /odom_monitor/reset 可重新归零")

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.cur = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        if self.origin is None:
            self.origin = self.cur
            self.get_logger().info("已归零：原点锁定（起始 0°, x=0, y=0）")

    def on_reset(self, _msg):
        if self.cur is not None:
            self.origin = self.cur
            self.get_logger().info("已重新归零")

    def on_timer(self):
        if self.cur is None or self.origin is None:
            self.get_logger().info("等待 /odom_combined ...")
            return
        x, y, yaw = self.cur
        x0, y0, yaw0 = self.origin
        dx, dy = x - x0, y - y0
        c, s = math.cos(yaw0), math.sin(yaw0)
        xr = dx * c + dy * s        # 旋转到起始朝向系
        yr = -dx * s + dy * c
        yaw_deg = math.degrees(norm_angle(yaw - yaw0))
        self.get_logger().info(
            f"方向角= {yaw_deg:+7.2f}°   x= {xr * 100:+8.1f} cm   y= {yr * 100:+8.1f} cm")


def main():
    rclpy.init()
    node = OdomMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
