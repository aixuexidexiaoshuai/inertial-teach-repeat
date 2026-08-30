#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一键起全链路：底盘 + 惯导 + EKF（发车前跑这一个就够）。

  origincar_base(串口, akmcar=false) ──> /odom, 收 /cmd_vel
  ICM-20948 ──> madgwick(关磁) ──> ekf_node ──> /odom_combined

⚠️⚠️ 开机后 3 秒内车必须【静止】（陀螺零偏标定），且每次都在【同一发车点、同一朝向】开机。

⚠️ 转向链路只支持 akmcar:=false —— STM32 把 cmd_vel.angular.z 当【角速度 ω】读，
   按 R=Vx/Vz 反解阿克曼转角，follower 的 _omega_for_delta 就是按这个模型反解的。
   akmcar:=true 会启用厂商的 cmd_vel_to_ackermann_drive（通用自行车模型，且
   origincar_base 还会把转角再除以 2），与固件模型不等价 —— 本 launch 直接拒绝。

⚠️ 下位机固件需要是【已改版】：舵机转向对称化 + 串口上报 50Hz（见 README「下位机固件」一节）。
   用原厂固件也能跑，但左右转向不对称会损害复现性，且上报 20Hz 时链路跑不满 50Hz。

用法：
    ros2 launch ahpu_odom_nav bringup.launch.py
    # 底盘已经在别处起好了 → 只起惯导部分：
    ros2 launch ahpu_odom_nav imu_ekf.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _reject_generic_ackermann(context, *args, **kwargs):
    """akmcar:=true 会静默切到与本车固件不等价的转向链，直接拦下。"""
    if LaunchConfiguration("akmcar").perform(context).strip().lower() in {"true", "1", "yes", "on"}:
        raise RuntimeError(
            "akmcar:=true 与本车 STM32 的 omega 协议不兼容（详见本文件顶部说明）。"
            "请用 akmcar:=false。")
    return []


def generate_launch_description():
    base_launch = os.path.join(get_package_share_directory("origincar_base"), "launch")
    pkg_launch = os.path.join(get_package_share_directory("ahpu_odom_nav"), "launch")

    # 底盘串口（发 /odom、收 /cmd_vel）
    base_serial = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(base_launch, "base_serial.launch.py")),
        launch_arguments={"akmcar": LaunchConfiguration("akmcar")}.items(),
    )
    # 机器人模型（robot_state_publisher）
    robot_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(base_launch, "robot_mode_description.launch.py")),
    )
    joint_state = Node(
        package="joint_state_publisher", executable="joint_state_publisher",
        name="joint_state_publisher",
    )
    # base_footprint → base_link 用单位阵（base_footprint 本就在 base_link 正下方）
    base_to_link = Node(
        package="tf2_ros", executable="static_transform_publisher", name="base_to_link",
        arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"],
    )
    # 惯导 + EKF
    imu_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_launch, "imu_ekf.launch.py")),
        launch_arguments={
            "i2c_bus": LaunchConfiguration("i2c_bus"),
            "imu_addr": LaunchConfiguration("imu_addr"),
            "gyro_z_sign": LaunchConfiguration("gyro_z_sign"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("akmcar", default_value="false",
                              description="必须 false：固件 omega 协议是唯一受支持的转向链"),
        DeclareLaunchArgument("i2c_bus", default_value="5",
                              description="ICM 所在 I²C 总线号（/dev/i2c-N）"),
        DeclareLaunchArgument("imu_addr", default_value="104",
                              description="ICM I²C 地址(十进制)：104=0x68，105=0x69"),
        DeclareLaunchArgument("gyro_z_sign", default_value="1.0",
                              description="陀螺 yaw 符号：左转 yaw 应增大；装反了设 -1.0"),
        OpaqueFunction(function=_reject_generic_ackermann),
        base_serial,
        robot_desc,
        joint_state,
        base_to_link,
        imu_ekf,
    ])
