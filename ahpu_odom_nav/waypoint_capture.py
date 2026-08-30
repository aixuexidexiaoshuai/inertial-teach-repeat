#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轨迹录制工具（teach-and-repeat 的 "teach" 一步，配合 odom_waypoint_follower）。

两种采法：
  - 【连续录制】遥控顺着路线开一遍，按【行程】每 sample_dist 米自动记一点（默认 2cm），
    把整条路线录成密集折线。每点同时记录当时的车头 yaw —— 回放时作航向参考，
    也是 follower 判断"这一点当时是前进还是倒车"的依据。
  - 【手动单点】敲回车在当前位姿补采一个点。

结束(q)时把录到的点写成一个【可直接跑的参数文件】 <config_dir>/path_<name>.yaml
（以模板 yaml 为骨架，只替换 waypoints_x/y/yaw 三个数组，其余控制参数沿用模板值），
同时在终端打印数组作备份。录完建议再跑一遍 smooth_path 平滑 + 均匀重采样：
    ros2 run ahpu_odom_nav smooth_path <config_dir>/path_<name>.yaml 0.02
然后用生成的 _smooth.yaml 回放。

⚠️ 录制与回放必须在【同一发车点、同一朝向开机】：航点是 odom_combined 系的，
   而该坐标系的原点/朝向 = 开机时刻的车位姿。中途重启底盘 → odom 原点漂走 → 坐标作废。
⚠️ 开机头 3 秒静止别碰车（陀螺零偏标定期，见 icm20948_driver）。

用法：
    ros2 run ahpu_odom_nav waypoint_capture --ros-args -p path_name:=my_route
    # 交互：s=开始连续录制  p=暂停  回车=手动补一点  u=撤销  q=结束并写文件
"""

import math
import os
import threading

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    """从四元数提取 yaw（绕 z），单位 rad。"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class WaypointCapture(Node):
    def __init__(self):
        super().__init__("waypoint_capture")
        self.declare_parameter("odom_topic", "/odom_combined")
        self.declare_parameter("sample_dist", 0.02)   # 连续录制：每行进多少米记一点（默认 2cm）
        self.declare_parameter("path_name", "path")   # 输出文件名 path_<name>.yaml
        # 输出目录：默认当前工作目录（留空即可）。
        self.declare_parameter("config_dir", "")
        # 参数模板：只替换其中的 waypoints_x/y/yaw，其余控制参数原样继承。
        #   留空 = 用装在 share/ahpu_odom_nav/config/ 里的 waypoints.yaml。
        self.declare_parameter("template", "")

        self.sample_dist = float(self.get_parameter("sample_dist").value)
        self.path_name = self.get_parameter("path_name").value
        self.config_dir = str(self.get_parameter("config_dir").value) or os.getcwd()
        self.template = self._resolve_template(str(self.get_parameter("template").value))

        self.lock = threading.Lock()
        self.latest = None        # (x, y, yaw_deg)
        self.points = []          # [(x, y, yaw_deg), ...] 按记录顺序
        self.recording = False
        self.last_rec_xy = None   # 上一记录点 (x,y)，用于按行程采样
        self.path_len = 0.0       # 累计路径长度 m

        topic = self.get_parameter("odom_topic").value
        self.create_subscription(Odometry, topic, self.on_odom, 50)
        self.create_timer(2.0, self.on_status_timer)
        self.get_logger().info(
            f"采集启动：订阅 {topic}，连续录制间隔 {self.sample_dist} m，"
            f"输出 path_{self.path_name}.yaml（等待第一帧里程计…）")

    def _resolve_template(self, template):
        """模板 yaml 的查找顺序：绝对路径 → config_dir/ → 包内 share/config/waypoints.yaml。"""
        if template and os.path.isabs(template):
            return template
        if template:
            cand = os.path.join(self.config_dir, template)
            if os.path.isfile(cand):
                return cand
        name = template or "waypoints.yaml"
        try:
            from ament_index_python.packages import get_package_share_directory
            return os.path.join(get_package_share_directory("ahpu_odom_nav"), "config", name)
        except Exception:
            return os.path.join(self.config_dir, name)

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        cur = (p.position.x, p.position.y, math.degrees(yaw_from_quat(p.orientation)))
        with self.lock:
            self.latest = cur
            if self.recording and self.last_rec_xy is not None:
                d = math.hypot(cur[0] - self.last_rec_xy[0], cur[1] - self.last_rec_xy[1])
                if d >= self.sample_dist:
                    self.points.append(cur)
                    self.path_len += d
                    self.last_rec_xy = (cur[0], cur[1])

    def on_status_timer(self):
        with self.lock:
            rec, n, length = self.recording, len(self.points), self.path_len
            latest = self.latest
        if latest is None:
            return  # 还没收到里程计
        x, y, yawd = latest
        if rec:
            self.get_logger().info(
                f"[录制中] 当前 x={x:.3f} y={y:.3f} yaw={yawd:.1f}° | 已记 {n} 点，行程 {length:.2f} m")
        else:
            self.get_logger().info(f"[待命] 当前 x={x:.3f} y={y:.3f} yaw={yawd:.1f}°")

    # ---- 记录控制（输入线程调用）----
    def start_recording(self):
        with self.lock:
            if self.latest is None:
                return False
            self.recording = True
            self.last_rec_xy = (self.latest[0], self.latest[1])
            self.points.append(self.latest)   # 起锚点
        return True

    def pause_recording(self):
        with self.lock:
            self.recording = False

    def manual_point(self):
        with self.lock:
            if self.latest is None:
                return None
            self.points.append(self.latest)
            self.last_rec_xy = (self.latest[0], self.latest[1])
            return (len(self.points) - 1, self.latest)

    def undo(self):
        with self.lock:
            if self.points:
                p = self.points.pop()
                if self.points:
                    self.last_rec_xy = (self.points[-1][0], self.points[-1][1])
                return p
        return None

    # ---- 写出可跑的参数文件 ----
    def write_path_file(self):
        with self.lock:
            pts = list(self.points)
        if not pts:
            return None, "没采到点"
        xs = ", ".join(f"{p[0]:.3f}" for p in pts)
        ys = ", ".join(f"{p[1]:.3f}" for p in pts)
        yaws = ", ".join(f"{p[2]:.2f}" for p in pts)
        x_line = f"    waypoints_x:   [{xs}]\n"
        y_line = f"    waypoints_y:   [{ys}]\n"
        # 每点同步记录 yaw（度，odom 系）→ 回放位姿跟踪用作航向参考（“方向由 yaw 决定”）
        yaw_line = f"    waypoints_yaw: [{yaws}]\n"

        out_path = os.path.join(self.config_dir, f"path_{self.path_name}.yaml")
        tmpl_path = self.template
        try:
            with open(tmpl_path, "r", encoding="utf-8") as f:
                lines = f.readlines()
            replaced = {"x": False, "y": False, "yaw": False}
            out_lines = []
            for ln in lines:
                s = ln.lstrip()
                if s.startswith("waypoints_x:"):
                    out_lines.append(x_line); replaced["x"] = True
                elif s.startswith("waypoints_y:"):
                    out_lines.append(y_line); replaced["y"] = True
                elif s.startswith("waypoints_yaw:"):
                    out_lines.append(yaw_line); replaced["yaw"] = True
                else:
                    out_lines.append(ln)
            if not all(replaced.values()):
                return None, f"模板缺少待替换字段 {replaced}（{tmpl_path}）"
            with open(out_path, "w", encoding="utf-8") as f:
                f.writelines(out_lines)
            return out_path, None
        except FileNotFoundError:
            return None, f"模板不存在：{tmpl_path}"
        except Exception as e:
            return None, str(e)


def input_loop(node: WaypointCapture):
    print("\n=== 密集路径采集 ===")
    print("  s  = 开始连续录制（每 {:.2f}m 自动记一点）".format(node.sample_dist))
    print("  p  = 暂停连续录制")
    print("  回车 = 在当前位姿手动补采一个点")
    print("  u  = 撤销最后一个点")
    print("  q  = 结束并写出 path_{}.yaml\n".format(node.path_name))
    while rclpy.ok():
        try:
            with node.lock:
                rec, n = node.recording, len(node.points)
            tag = "REC" if rec else "暂停"
            line = input(f"[{tag} | 已记 {n} 点] s/p/回车/u/q > ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if line == "q":
            break
        elif line == "s":
            if node.start_recording():
                print("  ▶ 开始连续录制，遥控顺着路线开")
            else:
                print("  ⚠️ 还没收到 /odom_combined，确认底盘+EKF 已启动")
        elif line == "p":
            node.pause_recording()
            print("  ⏸ 已暂停")
        elif line == "u":
            p = node.undo()
            print(f"  ↩ 撤销 {('x=%.3f y=%.3f' % (p[0], p[1])) if p else '(空)'}")
        elif line == "":
            r = node.manual_point()
            if r is None:
                print("  ⚠️ 还没收到 /odom_combined")
            else:
                i, p = r
                print(f"  ✓ 手动点{i}: x={p[0]:.3f} y={p[1]:.3f} yaw={p[2]:.1f}°")
        else:
            print("  ? 未知命令（s/p/回车/u/q）")

    # ---- 结束：写文件 + 打印备份 ----
    node.pause_recording()
    out_path, err = node.write_path_file()
    with node.lock:
        pts = list(node.points)
        length = node.path_len
    if pts:
        xs = ", ".join(f"{p[0]:.3f}" for p in pts)
        ys = ", ".join(f"{p[1]:.3f}" for p in pts)
        print(f"\n===== 采集结束：{len(pts)} 点，路径长 {length:.2f} m =====")
        if out_path:
            print(f"✓ 已写出可跑参数文件：{out_path}")
            print(f"  跑它： ros2 launch ahpu_odom_nav odom_nav.launch.py params_file:={out_path}")
        else:
            print(f"✗ 写文件失败：{err}")
        print("\n--- 备份数组（必要时手动用）---")
        print(f"    waypoints_x:   [{xs}]")
        print(f"    waypoints_y:   [{ys}]")
        print("===============================\n")
    else:
        print("\n没采到点。\n")
    rclpy.shutdown()


def main():
    rclpy.init()
    node = WaypointCapture()
    t = threading.Thread(target=input_loop, args=(node,), daemon=True)
    t.start()
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
