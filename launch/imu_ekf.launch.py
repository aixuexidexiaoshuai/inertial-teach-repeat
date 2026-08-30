#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
惯导定位底座：ICM-20948(纯陀螺) + madgwick(关磁) + EKF → /odom_combined

链路：
  icm20948_driver ──/icm/data_raw──> imu_filter_madgwick(use_mag=false) ──/imu/data──>
    robot_localization ekf_node(只融 vyaw，航向从 0 积分) ──> /odom_combined

⚠️ 本 launch【不启动底盘】。请先用你自己底盘的 launch 把 /odom（轮式里程计）和
   /cmd_vel 跑起来，再起本文件。EKF 订阅的 odom0 就是 /odom。

⚠️ 开机后 3 秒内车必须【静止】（陀螺零偏标定），且每次都在【同一发车点、同一朝向】开机
   —— 航向从 0 积分，开机车头向就是 odom_combined 的 +X 轴。

参数：
  imu_addr:=104        ICM 的 I²C 地址（十进制）：104=0x68，105=0x69。先 i2cdetect 确认。
  i2c_bus:=5           I²C 总线号（/dev/i2c-N 的 N）
  gyro_z_sign:=1.0     陀螺 yaw 符号；实车左转时 yaw 应增大，反了改 -1.0
  ekf_config:=<file>   覆盖默认的 config/ekf_odom.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("ahpu_odom_nav")
    default_ekf = os.path.join(pkg_share, "config", "ekf_odom.yaml")

    # IMU 安装 TF 用【单位阵】：恒定的安装倾斜只是把整个 odom 帧旋转一个固定角，
    # teach-and-repeat 录/放同帧、自动抵消，无需标定安装角（详见 README）。
    base_to_imu = Node(
        package="tf2_ros", executable="static_transform_publisher", name="base_to_imu",
        arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "imu_link"],
    )

    icm_driver = Node(
        package="ahpu_odom_nav", executable="icm20948_driver",
        name="icm20948_driver", output="screen",
        parameters=[{
            "i2c_bus": ParameterValue(LaunchConfiguration("i2c_bus"), value_type=int),
            "addr": ParameterValue(LaunchConfiguration("imu_addr"), value_type=int),
            "gyro_z_sign": ParameterValue(LaunchConfiguration("gyro_z_sign"), value_type=float),
        }],
    )

    # madgwick：陀螺(驱动已扣零偏)积分出 yaw + 加速度只纠 roll/pitch。
    #   ⚠️ use_mag 必须 false —— 这是整套方案的前提（磁畸变会让航向跳数十度）。
    #   代价：绝对 yaw 是随机初值、不可复现 → 所以 EKF 不融它，只融 vyaw（见 ekf_odom.yaml）。
    imu_filter = Node(
        package="imu_filter_madgwick", executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick", output="screen",
        parameters=[{
            "use_mag": False,
            "gain": 0.05,
            "world_frame": "enu",
            "publish_tf": False,
        }],
        remappings=[
            ("imu/data_raw", "/icm/data_raw"),
            ("imu/data", "/imu/data"),
        ],
    )

    ekf = Node(
        package="robot_localization", executable="ekf_node", name="ekf_filter_node",
        output="screen",
        parameters=[LaunchConfiguration("ekf_config")],
        remappings=[("odometry/filtered", "odom_combined")],
    )

    return LaunchDescription([
        DeclareLaunchArgument("i2c_bus", default_value="5",
                              description="ICM 所在 I²C 总线号（/dev/i2c-N）"),
        DeclareLaunchArgument("imu_addr", default_value="104",
                              description="ICM I²C 地址(十进制)：104=0x68，105=0x69"),
        DeclareLaunchArgument("gyro_z_sign", default_value="1.0",
                              description="陀螺 yaw 符号：左转 yaw 应增大；装反了设 -1.0"),
        DeclareLaunchArgument("ekf_config", default_value=default_ekf,
                              description="EKF 参数文件"),
        base_to_imu,
        icm_driver,
        imu_filter,
        ekf,
    ])
