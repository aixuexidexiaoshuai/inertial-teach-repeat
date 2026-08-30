#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
里程计 + 惯导 teach-and-repeat 轨迹跟踪节点（不依赖 GPS / SLAM / 地图 / Nav2）。

输入：EKF 融合里程计 /odom_combined（轮速管位置 + 陀螺管航向，见 config/ekf_odom.yaml）
输出：/cmd_vel（linear.x = v，angular.z = ω）

原理：对 teach-and-repeat 录的密集折线（每 ~2cm 一点，每点带录制时车头 yaw）做 Stanley 位姿跟踪：
  ① 单调前向【小窗口】找最近点 —— 杜绝在自交/折返处"空间最近点"跨腿跳点。
     （全局最近点会在三点掉头处从去程腿跳到回程腿，甩掉整段，且跳点时机随噪声变 →
       每次回放都不一样。progress_window 必须远小于折返处的索引跨度。）
  ② 两个误差：横向误差 e（车到最近点的位移投影到路径法线，左正右负）
             航向误差 he（该点录制车头朝向 − 当前 yaw）
  ③ Stanley 控制律 + 曲率前馈：
        δ = heading_gain·he + atan2(−k·e, ks+v) + curv_ff·atan(L·κ)
            └─ 车头掰正 ─┘   └── 拉回线上 ──┘   └─ 弯道提前打舵 ─┘
     前两项是反馈（等误差出现才纠，弯道必然滞后）；曲率前馈是前馈——录制路径这一点该弯
     多少是已知的，直接先把舵打出来，he/e 退化为微调。仅前进段加（倒车段是掉头尖点，κ 无定义）。
  ④ δ → ω 换算后限幅/滤波 → /cmd_vel；速度按过弯/末端/倒车规划。
  ⑤ 终点：进度到末点且离末点 < final_tolerance（默认 4cm）停车，可选加航向门。

【忠实倒车】录制里有后退段（如三点掉头）→ 回放那段就【倒着走】贴录制轨迹。
  每点是否倒车 = 录制车头 yaw 与运动切线相反(cos<0)。前↔后换向先停稳 dir_switch_sec 再走
  （阿克曼换向必须先停）。ω 用 |v|·tanδ/L 的幅值，前进/后退符号自洽。

⚠️ 坐标系：航点都在 odom_combined 系 —— 【开机点=原点，开机车头向=+X 轴】。
   所以录制与回放必须在【同一发车点、同一朝向开机】，两次坐标系才自动重合。
⚠️ 纯航位推算随距离累积漂移，适合数米~数十米量级的短程复现。

用法：
    ros2 launch ahpu_odom_nav odom_nav.launch.py params_file:=<path_xxx_smooth.yaml>
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32MultiArray

from .path_geometry import path_spacing, path_curvature


def yaw_from_quat(q):
    """从四元数提取 yaw（绕 z）。"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def norm_angle(a):
    """归一化到 [-pi, pi]。"""
    return math.atan2(math.sin(a), math.cos(a))


class OdomWaypointFollower(Node):
    def __init__(self):
        super().__init__("odom_waypoint_follower")

        # ================= 参数 =================
        # ---- 路径（由 waypoint_capture + smooth_path 生成的 yaml 提供）----
        self.declare_parameter("waypoints_x", [0.0, 1.0])
        self.declare_parameter("waypoints_y", [0.0, 0.0])
        # 每点的录制车头 yaw（度，odom_combined 系，与 x/y 一一对应）。
        #   作 Stanley 的【航向参考】；缺省哨兵 999 的点用相邻点切线推算。
        self.declare_parameter("waypoints_yaw", [999.0])
        self.declare_parameter("no_heading_value", 999.0)     # 朝向哨兵值：等于此=无 yaw

        # ---- 车辆 / 下位机转向契约 ----
        # ⚠️ 本节点发出的 angular.z 是【角速度 ω】，由下位机按 R = Vx/Vz 反解阿克曼转角。
        #    若你的底盘把 angular.z 当【前轮转角】读，需自行改 _omega_for_delta（见 README）。
        self.declare_parameter("wheelbase", 0.143)            # 轴距 m
        # 前轮转角上限【左右可以不对称】（本车实测：左 0.320rad=18.3°，右 0.4461rad=25.6°，
        # 右边更大是因为固件限幅 + 舵机 PWM 上限共同作用）。按自己的车实测填。
        self.declare_parameter("steer_max_left_rad", 0.3200)
        self.declare_parameter("steer_max_right_rad", 0.4461)
        # 下位机 ω→转角 换算里用到的固件常数（用于精确反解，见 _omega_for_delta）
        self.declare_parameter("fw_axle_spacing", 0.144)      # 固件轴距常数
        self.declare_parameter("fw_half_wheelspacing", 0.081) # 固件 0.5×轮距
        self.declare_parameter("fw_min_turn_radius", 0.350)   # 固件最小转弯半径钳位

        # ---- Stanley 控制律 ----
        self.declare_parameter("stanley_k", 1.2)              # 横向误差增益（大→纠偏狠，过大发抖）
        self.declare_parameter("stanley_ks", 0.4)             # 软化项 m/s（低速防 atan2 过冲）
        self.declare_parameter("heading_gain", 1.0)           # 航向误差权重
        # 曲率前馈：δ += gain·atan(L·κ)。0=关（退回纯反馈）。只在前进段生效。
        #   gain 拆两部分：1.0=纯运动学(atan(L/R)，与速度无关)；超出部分补偿舵机/控制延迟，
        #   而延迟距离 = v·τ 正比于速度 → 换巡航速度可用 adaptive 自动缩放，免重标。
        self.declare_parameter("curv_ff_gain", 1.4)           # 定值增益（adaptive=false 时用）
        self.declare_parameter("curv_ff_adaptive", False)     # true: gain = 1.0 + speed_k·linear_speed
        self.declare_parameter("curv_ff_speed_k", 0.667)      # 0.667 使 0.6m/s → 1.4，与定值等价
        self.declare_parameter("curv_kappa_baseline", 0.15)   # 算 κ 的三点外接圆弧长基线 m
        self.declare_parameter("progress_window", 8)          # 单调前向搜索窗口(点数)，须 < 折返索引跨度

        # ---- 转角平滑 / 限幅 ----
        # δ 斜率限幅 rad/s，0=关。治高速极限环：限制转角【每秒】变化量，只砍突变、
        #   不引入相位滞后（区别于下面的 EMA 低通——那个会加滞后，反而喂养振荡）。
        #   推荐 2.0~4.0 rad/s（≈115~230°/s），舵机物理上也就这么快。
        self.declare_parameter("delta_rate_max", 0.0)
        # δ 一阶低通(EMA)系数 0<α≤1：新舵角权重。1.0=不滤波。高速直线摆动可调 0.3~0.5
        #   （越小越稳，弯道响应越钝）。仅前进段滤波，换向/停车自动复位。
        self.declare_parameter("alpha_delta", 1.0)

        # ---- 速度规划 ----
        self.declare_parameter("linear_speed", 0.20)          # 巡航标称线速度 m/s
        self.declare_parameter("min_speed", 0.05)             # 速度下限 m/s（各种减速的地板）
        self.declare_parameter("turn_slowdown", 0.6)          # 过弯减速强度 0~1（越大转弯越慢）
        self.declare_parameter("end_slow_dist", 0.30)         # 末端减速起始距离 m（贴准终点）
        self.declare_parameter("end_slow_speed", 0.40)        # 末端减速【封顶】速度 m/s（高速刹不住终点用）
        # 线加速度斜率限幅 m/s²，0=关。【只限加速、不限减速】：要慢下来立刻慢（刹车越晚越危险），
        #   提速则摊成多帧平滑爬升 —— 突跳的 linear.x 会让车身俯仰/打滑、位姿抖、δ 跟着抖。
        self.declare_parameter("accel_max", 1.5)
        self.declare_parameter("control_rate", 50.0)          # 控制频率 Hz（须 ≤ EKF 频率）

        # ---- 倒车 ----
        self.declare_parameter("dir_switch_sec", 0.5)         # 前↔后换向先停稳的时长 s
        self.declare_parameter("reverse_max_speed", 0.12)     # 倒车段线速度上限 m/s（盲区，慢且稳）
        # 倒车转向符号：Stanley 的 δ 是前进语义，倒车直接用会打反。本车实测 = -1.0；换底盘需实车验证。
        self.declare_parameter("reverse_steer_sign", -1.0)

        # ---- 终点判定 ----
        self.declare_parameter("final_tolerance", 0.04)       # 终点到达半径 m（默认 4cm）
        # 终点航向容差(度)：>0 时位置到达【且】|车头−末点录制朝向|<此 才算到达；-1=不约束。
        self.declare_parameter("final_yaw_tol_deg", -1.0)
        # 航向没满足时最多再前冲多少 m（边走边纠航向），超过则兜底停 + 告警（防无限前冲）。
        self.declare_parameter("final_overshoot_max", 0.20)
        # 终点区(<10cm)卡住超过此秒数 → 强制停（防撞板空转烧电机）。
        self.declare_parameter("final_stuck_sec", 2.0)

        # ---- 发车门控 / 话题 ----
        # wait_start=true：启动后【待命】，等 start_topic 收到 True 才发车（给传感器预热的时间）。
        self.declare_parameter("wait_start", False)
        self.declare_parameter("start_topic", "/odom_nav/start")
        self.declare_parameter("odom_topic", "/odom_combined")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        # 调参用调试话题（Float32MultiArray），字段见 _publish_debug 的注释。空字符串=关。
        self.declare_parameter("debug_topic", "/odom_nav/debug")

        gp = self.get_parameter
        wx0 = list(gp("waypoints_x").value)
        wy0 = list(gp("waypoints_y").value)
        wyaw0 = list(gp("waypoints_yaw").value) if gp("waypoints_yaw").value else []
        self.no_heading = float(gp("no_heading_value").value)

        self.wheelbase = float(gp("wheelbase").value)
        self.delta_max_l = float(gp("steer_max_left_rad").value)
        self.delta_max_r = float(gp("steer_max_right_rad").value)
        self.fw_axle = float(gp("fw_axle_spacing").value)
        self.fw_hw = float(gp("fw_half_wheelspacing").value)
        self.fw_rmin = float(gp("fw_min_turn_radius").value)
        # 由转角上限反推真实最小转弯半径（自行车模型，δ 即等效中心转角）
        self.min_turn_l = self.wheelbase / math.tan(self.delta_max_l)
        self.min_turn_r = self.wheelbase / math.tan(self.delta_max_r)

        self.stanley_k = float(gp("stanley_k").value)
        self.stanley_ks = float(gp("stanley_ks").value)
        self.heading_gain = float(gp("heading_gain").value)
        self.curv_ff_gain = float(gp("curv_ff_gain").value)
        self.curv_ff_adaptive = bool(gp("curv_ff_adaptive").value)
        self.curv_ff_speed_k = float(gp("curv_ff_speed_k").value)
        self.kappa_baseline = float(gp("curv_kappa_baseline").value)
        self.progress_window = int(gp("progress_window").value)

        self.delta_rate_max = float(gp("delta_rate_max").value)
        self.alpha_delta = min(1.0, max(0.0, float(gp("alpha_delta").value)))
        self.delta_prev = 0.0     # δ 斜率限幅状态
        self.delta_filt = 0.0     # δ 低通状态
        self.v_prev = 0.0         # 加速度限幅状态（上周期发出的线速度幅值 ≥0）

        self.v_nom = float(gp("linear_speed").value)
        self.v_min = float(gp("min_speed").value)
        self.turn_slowdown = float(gp("turn_slowdown").value)
        self.end_slow_dist = float(gp("end_slow_dist").value)
        self.end_slow_speed = float(gp("end_slow_speed").value)
        self.accel_max = float(gp("accel_max").value)
        rate = float(gp("control_rate").value)
        self.control_rate = rate

        self.dir_switch_sec = float(gp("dir_switch_sec").value)
        self.reverse_max_speed = float(gp("reverse_max_speed").value)
        self.reverse_steer_sign = float(gp("reverse_steer_sign").value)

        self.final_tol = float(gp("final_tolerance").value)
        self.final_yaw_tol_deg = float(gp("final_yaw_tol_deg").value)
        self.final_yaw_tol = math.radians(self.final_yaw_tol_deg)   # ≤0 = 不约束
        self.final_overshoot_max = float(gp("final_overshoot_max").value)
        self.final_stuck_sec = float(gp("final_stuck_sec").value)

        self.wait_start = bool(gp("wait_start").value)
        self.started = not self.wait_start

        # 曲率前馈生效增益一次性算好（都是常数，不随控制周期变）。
        # ⚠️ 用【标称】linear_speed 而非运行时实际速度：过弯/末端的临时降速不扰动前馈，行为可预测。
        self.curv_ff_eff = (1.0 + self.curv_ff_speed_k * self.v_nom
                            if self.curv_ff_adaptive else self.curv_ff_gain)

        # ================= 状态 =================
        self.pose = None            # (x, y, yaw)
        self.near_final_t0 = None   # 终点超时保护计时起点

        # ================= 通信 =================
        # 里程计回调放并行组、控制定时器独占组：即使将来在同进程里挂上耗时的回调，
        # 控制循环也不会被它阻塞（单线程 executor 下，一个慢回调能把 50Hz 控制饿到 1Hz）。
        self._grp_io = ReentrantCallbackGroup()
        self._grp_ctrl = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Odometry, gp("odom_topic").value, self.on_odom, 20,
                                 callback_group=self._grp_io)
        if self.wait_start:
            self.create_subscription(Bool, gp("start_topic").value, self.on_start, 1,
                                     callback_group=self._grp_io)
            self.get_logger().info(
                f"⏸  待命中 —— 等 {gp('start_topic').value} 发 True 才发车")
        self.pub_cmd = self.create_publisher(Twist, gp("cmd_vel_topic").value, 10)
        self.pub_done = self.create_publisher(String, "/odom_nav/finished", 1)
        dbg = str(gp("debug_topic").value).strip()
        self.dbg_pub = (self.create_publisher(Float32MultiArray, dbg, 10) if dbg else None)

        self._apply_waypoints(wx0, wy0, wyaw0)

        self.timer = self.create_timer(1.0 / rate, self.on_timer,
                                       callback_group=self._grp_ctrl)

        n_rev = sum(1 for r in self.rev if r)
        self.get_logger().info(
            f"转向上限：左 δ≤{math.degrees(self.delta_max_l):.1f}°(R≥{self.min_turn_l:.3f}m)｜"
            f"右 δ≤{math.degrees(self.delta_max_r):.1f}°(R≥{self.min_turn_r:.3f}m)")
        self.get_logger().info(
            f"曲率前馈：{'自适应' if self.curv_ff_adaptive else '定值'} → 生效增益 "
            f"{self.curv_ff_eff:.2f}{'（关）' if self.curv_ff_eff == 0.0 else ''}")
        self.get_logger().info(
            f"轨迹跟踪启动：{len(self.waypoints)} 点（间距 {self.ds*100:.1f}cm），"
            f"Stanley k={self.stanley_k} 窗口={self.progress_window}，v={self.v_nom} m/s，"
            f"终点容差={self.final_tol}m"
            + (f"｜终点航向≤{self.final_yaw_tol_deg:.0f}°" if self.final_yaw_tol > 0 else "")
            + (f"｜含 {n_rev} 个倒车点" if n_rev else ""))

    # ---------------- 回调 ----------------
    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def on_start(self, msg: Bool):
        """发车/停止门控。True=发车；False=急停并复位待命。"""
        if msg.data:
            if not self.started:
                self.started = True
                self.get_logger().info("▶️  收到发车信号 —— 出发！")
        elif self.started:
            self.started = False
            self.stop()
            self.get_logger().warn("⏹  收到停止信号 —— 已停车（再发 True 可继续）")

    # ---------------- 路径加载 ----------------
    def _apply_waypoints(self, wx, wy, wyaw_deg):
        """加载一条路径（航点 + 每点录制 yaw）并重置跟踪状态。"""
        if len(wx) != len(wy) or len(wx) < 2:
            raise ValueError("waypoints_x / waypoints_y 长度必须相等且至少 2 点")
        self.waypoints = list(zip(wx, wy))
        wyaw = list(wyaw_deg) if wyaw_deg else []
        self.wp_yaw = []
        for i in range(len(self.waypoints)):
            if i < len(wyaw) and abs(float(wyaw[i]) - self.no_heading) > 1e-6:
                self.wp_yaw.append(math.radians(float(wyaw[i])))
            else:
                self.wp_yaw.append(None)
        # path_yaw = 每点【录制车头朝向】（有 yaw 用 yaw，缺省用相邻点切线）→ Stanley 航向参考
        self.path_yaw = self._build_path_yaw()
        # path_tan = 每点【运动切线】（点序几何方向）→ 横向误差的路径法线基准
        self.path_tan = self._build_path_tan()
        # rev = 每点是否【倒车】：录制车头朝向与运动切线相反(cos<0) = 当时在倒车
        self.rev = [math.cos(self.path_yaw[i] - self.path_tan[i]) < 0.0
                    for i in range(len(self.waypoints))]
        # 曲率前馈用的 κ（左正右负）
        self.ds = path_spacing(self.waypoints)
        self.kappa = path_curvature(self.waypoints, self.ds, self.kappa_baseline)
        if len(self.waypoints) < 15:
            self.get_logger().warn(
                f"只有 {len(self.waypoints)} 个航点 —— 本节点是【密集路径】跟踪器"
                "（teach-and-repeat 录制通常几百点）。点太稀会导致横向误差/切线估计粗糙。")
        # 重置跟踪状态
        self.progress_idx = 0
        self.end_idx = len(self.waypoints) - 1
        self.finished = False
        self.near_final_t0 = None
        # 当前行进方向（+1 前 / -1 后），起点按首点方向
        self.cur_dir = -1.0 if self.rev[0] else 1.0
        self.switch_t0 = None

    def _build_path_yaw(self):
        """每点航向参考（rad）：优先用录制 yaw，缺省用相邻点切线。"""
        n = len(self.waypoints)
        out = []
        for i in range(n):
            if self.wp_yaw[i] is not None:
                out.append(self.wp_yaw[i])
            else:
                if i < n - 1:
                    ax, ay = self.waypoints[i]; bx, by = self.waypoints[i + 1]
                else:
                    ax, ay = self.waypoints[i - 1]; bx, by = self.waypoints[i]
                out.append(math.atan2(by - ay, bx - ax))
        return out

    def _build_path_tan(self):
        """每点【运动切线】(rad)：由点序几何方向算（末点用前一段）。"""
        n = len(self.waypoints)
        out = []
        for i in range(n):
            if i < n - 1:
                ax, ay = self.waypoints[i]; bx, by = self.waypoints[i + 1]
            else:
                ax, ay = self.waypoints[i - 1]; bx, by = self.waypoints[i]
            out.append(math.atan2(by - ay, bx - ax))
        return out

    # ---------------- 主控制循环 ----------------
    def on_timer(self):
        if self.finished:
            return
        if not self.started:
            self.stop()        # 待命：持续发零速（很多底盘没有失联保护）
            return
        if self.pose is None:
            return             # 还没收到里程计

        x, y, yaw = self.pose
        n = len(self.waypoints)

        # ---- ① 单调前向【小窗口】找最近点 ----
        best_i, best_d = self.progress_idx, float("inf")
        for i in range(self.progress_idx, min(n, self.progress_idx + self.progress_window)):
            wx, wy = self.waypoints[i]
            d = math.hypot(wx - x, wy - y)
            if d < best_d:
                best_d, best_i = d, i
        self.progress_idx = best_i

        # ---- ② 终点判定 ----
        ei = self.end_idx
        lx, ly = self.waypoints[ei]
        dist_last = math.hypot(lx - x, ly - y)
        tan_last = self.path_tan[ei]
        s_beyond = (x - lx) * math.cos(tan_last) + (y - ly) * math.sin(tan_last)

        # 终点超时保护：进度到末点且距离 <10cm → 计时，卡住超时强制停（防撞板空转烧电机）
        if best_i >= ei and dist_last < 0.10:
            if self.near_final_t0 is None:
                self.near_final_t0 = self.get_clock().now()
            else:
                elapsed = (self.get_clock().now() - self.near_final_t0).nanoseconds * 1e-9
                if elapsed > self.final_stuck_sec:
                    self._finish(f"终点超时（剩 {dist_last*100:.1f}cm，卡住 {elapsed:.1f}s）→ 强制停")
                    return
        else:
            self.near_final_t0 = None

        if best_i >= ei:
            pos_ok = (dist_last < self.final_tol) or (s_beyond > 0.0)
            he_final = abs(norm_angle(self.path_yaw[ei] - yaw))
            yaw_gate = self.final_yaw_tol > 0.0
            if pos_ok and (not yaw_gate or he_final <= self.final_yaw_tol):
                self._finish(f"到达（剩 {dist_last*100:.1f}cm，航向差 {math.degrees(he_final):.0f}°）")
                return
            if yaw_gate and pos_ok and s_beyond > self.final_overshoot_max:
                # 位置到了但航向门迟迟不满足 → 兜底停，防无限前冲
                self._finish(f"越过末点 {s_beyond:.2f}m｜航向差 "
                             f"{math.degrees(he_final):.0f}°>{self.final_yaw_tol_deg:.0f}°（兜底停）")
                return
            # 位置未到 / 航向未达标 → 落到下面继续 Stanley（边前进边纠航向）

        # ---- ③ 行进方向：前↔后换向先停稳（阿克曼换向必须先停）----
        sdir = -1.0 if self.rev[best_i] else 1.0
        if sdir != self.cur_dir:
            if self.switch_t0 is None:
                self.switch_t0 = self.get_clock().now()
                self.get_logger().info(
                    f"[换向] {'前进→后退' if sdir < 0 else '后退→前进'}（idx{best_i}）"
                    f"停稳 {self.dir_switch_sec:.1f}s")
            self.stop()
            if (self.get_clock().now() - self.switch_t0).nanoseconds * 1e-9 >= self.dir_switch_sec:
                self.cur_dir = sdir
                self.switch_t0 = None
            return

        # ---- ④ 两个误差 ----
        # 横向误差 e 用【运动切线】的法线；航向误差 he 用【录制车头朝向】。
        # 倒车时录制 yaw 已是 tan+180°，he 仍相对车头 → ω 用 |v| 幅值，前/后符号自洽。
        tan = self.path_tan[best_i]
        face = self.path_yaw[best_i]
        px, py = self.waypoints[best_i]
        e = (x - px) * (-math.sin(tan)) + (y - py) * (math.cos(tan))
        he = norm_angle(face - yaw)

        # ---- ⑤ Stanley + 曲率前馈 → 前轮转角 δ ----
        delta = (self.heading_gain * he
                 + math.atan2(-self.stanley_k * e, self.stanley_ks + self.v_nom))
        kappa_here = self.kappa[best_i] if best_i < len(self.kappa) else 0.0
        if sdir > 0 and self.curv_ff_eff != 0.0:
            # 仅前进段：倒车段是三点掉头尖点，κ 无定义，交给航向反馈
            delta += self.curv_ff_eff * math.atan(self.wheelbase * kappa_here)
        delta = self._clamp_delta(delta)

        # δ 一阶低通：掐掉高速直线的高频抖动。仅前进段滤波；倒车（尖点掉头需要突变的大打角）
        #   不滤，只让状态跟随，下次转回前进时从当前 δ 平滑接续、不甩头。
        if self.alpha_delta < 1.0 and sdir > 0:
            self.delta_filt = self.alpha_delta * delta + (1.0 - self.alpha_delta) * self.delta_filt
            delta = self.delta_filt
        else:
            self.delta_filt = delta

        # δ 斜率限幅：只砍【突变】，对缓慢变化零影响、不引入滞后（这才是对症极限环的手段）。
        if self.delta_rate_max > 0.0 and sdir > 0:
            max_step = self.delta_rate_max / max(self.control_rate, 1e-6)
            diff = delta - self.delta_prev
            if abs(diff) > max_step:
                delta = self.delta_prev + math.copysign(max_step, diff)
        self.delta_prev = delta

        # ---- ⑥ 速度规划 ----
        dmax_here = self.delta_max_l if delta >= 0.0 else self.delta_max_r
        base = self.reverse_max_speed if sdir < 0 else self.v_nom
        if dist_last < self.end_slow_dist:
            # 末端减速：进入区间即把峰值封到 end_slow_speed（高速刹得住），再线性 ramp 到下限；
            #   end_slow_speed 无效(≤0 或 ≥v_nom)时退化为从 v_nom 线性降。
            cap = self.end_slow_speed if 0.0 < self.end_slow_speed < self.v_nom else self.v_nom
            base = min(base, cap * (dist_last / self.end_slow_dist))
        speed = base * (1.0 - self.turn_slowdown * min(abs(delta) / dmax_here, 1.0))
        v_mag = max(self.v_min, speed)

        # 线加速度斜率限幅：【只限加速、不限减速】。停车走 stop()，其中 v_prev 复位 0。
        if self.accel_max > 0.0:
            dv_up = self.accel_max / max(self.control_rate, 1e-6)
            if v_mag > self.v_prev + dv_up:
                v_mag = self.v_prev + dv_up
        self.v_prev = v_mag

        # ---- ⑦ δ → ω，发布 ----
        w = self._omega_for_delta(sdir * v_mag, delta)
        if sdir < 0:
            # 倒车段翻向：Stanley 的 δ 是前进语义；带符号 v 会让倒车 ω 符号翻转，
            # 导致掉头时方向盘打反（实车实测）。本车 reverse_steer_sign = -1.0。
            w *= self.reverse_steer_sign
        cmd = Twist()
        cmd.linear.x = sdir * v_mag
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)

        self._publish_debug(best_i, x, y, yaw, e, he, delta, sdir * v_mag, kappa_here, dist_last)

    # ---------------- 工具 ----------------
    def _publish_debug(self, idx, x, y, yaw, e, he, delta, v, kappa, dist_last):
        """调参用。data = [idx, x, y, yaw, e, he, delta, delta_max, v, kappa, dist_to_end]"""
        if self.dbg_pub is None:
            return
        self.dbg_pub.publish(Float32MultiArray(data=[
            float(idx), x, y, yaw, e, he, delta,
            (self.delta_max_l if delta >= 0.0 else self.delta_max_r),
            v, kappa, dist_last]))

    def _omega_for_delta(self, v_signed, delta):
        """
        把期望前轮转角 δ 换算成下位机需要的 angular.z(=ω)。

        ⚠️ 不是教科书的 ω = v·tanδ/L。本车固件收到 ω 之后做的是
              R = Vx/Vz ;  AngleR = atan(Axle/(R + 0.5*Wheel))
        所以要让固件算出的 AngleR == δ，必须按同一模型反解：
              R = Axle/tanδ − 0.5*Wheel ,  ω = Vx/R
        换底盘时请按你自己固件的换算改这里（或直接返回 v_signed*tan(δ)/wheelbase）。
        """
        # δ 接近 ±90° 时 tan(δ)→∞ 会让 R 异常；v≈0 时 ω 无意义
        if abs(delta) < 1e-4 or abs(v_signed) < 1e-6 or abs(abs(delta) - math.pi / 2) < 1e-2:
            return 0.0
        R = self.fw_axle / math.tan(delta) - self.fw_hw
        if abs(R) < self.fw_rmin:      # 固件会这么钳，提前对齐，避免"发了却没执行"
            R = math.copysign(self.fw_rmin, R)
        return v_signed / R

    def _clamp_delta(self, delta):
        """左右转角上限可以不同（本车 +0.320 / −0.4461 rad）。"""
        return max(-self.delta_max_r, min(self.delta_max_l, delta))

    def _finish(self, where):
        self.get_logger().info(f"[完成] {where} ✓ 巡航结束")
        self.stop()
        self.finished = True
        self.pub_done.publish(String(data="done"))

    def stop(self):
        self.delta_filt = 0.0      # 清 δ 低通状态，起步从 0 平滑爬升不甩头
        self.delta_prev = 0.0      # δ 斜率限幅状态复位
        self.v_prev = 0.0          # 加速度限幅状态复位
        self.pub_cmd.publish(Twist())


def main():
    rclpy.init()
    node = OdomWaypointFollower()
    # 多线程执行器：控制定时器独占一组，不会被 IO 或将来扩展的其它回调阻塞。
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # ⚠️⚠️ 退出必须停车！很多底盘固件【没有 cmd_vel 失联保护】，速度是全局变量、
        #   只在收到串口数据时更新 ⇒ 本节点一退出，底盘会保持最后的速度继续跑（实测：
        #   Ctrl-C 打断巡航，车一直冲）。发多次是因为单次 publish 可能还在 DDS 队列里
        #   就 destroy_node 了。强烈建议同时在固件侧加看门狗。
        try:
            import time as _t
            for _ in range(5):
                node.stop()
                _t.sleep(0.02)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
