#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轨迹跟踪（回放）launch：只起 odom_waypoint_follower。

前提：底盘（发 /odom、收 /cmd_vel）+ imu_ekf.launch.py（发 /odom_combined）已在跑。

参数全部来自 params_file（一个 path_xxx_smooth.yaml）：改 yaml 重启即生效，无需重新编译。
  ros2 launch ahpu_odom_nav odom_nav.launch.py params_file:=/abs/path/path_my_route_smooth.yaml

便捷覆盖（留空则完全不注入，yaml 原值生效）：
  speed:=0.4          临时改巡航线速度 linear_speed（m/s），调速不用改 yaml
  wait_start:=true    启动后待命，等 /odom_nav/start 收到 True 再发车
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    speed = LaunchConfiguration("speed").perform(context).strip()
    wait_start = LaunchConfiguration("wait_start").perform(context).strip().lower()

    overrides = {}
    if speed:
        # 必须转 float，否则整数会被当 INTEGER 类型、与已声明的 double 冲突而崩
        overrides["linear_speed"] = float(speed)
    if wait_start:
        overrides["wait_start"] = wait_start in ("true", "1", "yes", "on")

    return [Node(
        package="ahpu_odom_nav",
        executable="odom_waypoint_follower",
        name="odom_waypoint_follower",
        output="screen",
        parameters=[params_file] + ([overrides] if overrides else []),
    )]


def generate_launch_description():
    pkg_share = get_package_share_directory("ahpu_odom_nav")
    default_params = os.path.join(pkg_share, "config", "waypoints.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="轨迹 + 控制参数文件（path_xxx_smooth.yaml）"),
        DeclareLaunchArgument("speed", default_value="",
                              description="临时覆盖巡航线速度 m/s；空=用 yaml 里的值"),
        DeclareLaunchArgument("wait_start", default_value="",
                              description="true=待命等发车信号；空=用 yaml 里的值"),
        OpaqueFunction(function=_launch_setup),
    ])
