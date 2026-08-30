# 惯导定点巡航 · Inertial Teach-and-Repeat

> **没有 GPS、不建图、不巡线 —— 用一颗陀螺让阿克曼小车把人开过的路线厘米级重走。**

ROS 2 实现的 teach-and-repeat（示教-复现）定点巡航：遥控开一遍录下轨迹，之后车自己精确重走。
定位靠 **轮式里程计（管位置）+ 纯陀螺惯导（管航向）**，跟踪靠 **Stanley + 曲率前馈**。

| 指标 | 实测 |
|---|---|
| 终点定位精度 | **< 4 cm** |
| 航向漂移 | **~2° / 6 min**（纯陀螺 + ZUPT） |
| 控制频率 | 50 Hz |
| 路点间隔 | 2 cm |
| 已移植 | 3 台车（2×RDK X5 + 1×RDK S100） |

---

## 核心思想

定点巡航的命门是**航向**，不是位置：位置误差线性累积，航向误差却被路程放大 ——
**航向错 1°，走 5m 就横偏 8.7cm**。

三个候选航向源里，磁力计在金属场地会跳数十度，轮速一打滑就带偏，**只有陀螺可信**。
于是本方案做了一个反直觉的决定：

> **主动放弃磁力计给的"绝对航向"，换一个每次开机都从 0 开始的"相对航向"。**

纯陀螺有累积漂移这个致命缺陷，但它**每次开机都以完全相同的方式漂**。
teach-and-repeat 的本质是"重复"而不是"定位"—— 录制和回放都在"开机→静止标定→航向归零"
之后进行，两次的坐标系天然重合，**不需要初始对准、不需要找绝对北**。

**用"相对复现性"替代"绝对定位"**，就是这套方案的全部哲学。

陀螺的三个病，三个治法：

| 病 | 症状 | 治法 | 在哪 |
|---|---|---|---|
| ① 开机零偏 | 静止时也在"转"，实测 1.9°/s，积分 3s 就假偏 5.6° | 开机静止 3s 标定均值，之后逐帧扣除 | `icm20948_driver.py` |
| ② 绝对航向不可复现 | 关磁后 madgwick 的绝对 yaw 是随机初值（实测 -127°） | EKF **只融 vyaw**、航向从 0 积分 | `config/ekf_odom.yaml` |
| ③ 零偏温漂 | 芯片升温 bias 漂移，实测 -1.3°/min | ZUPT：只在**确认静止**时把残留零偏缓慢吸收 | `icm20948_driver.py` |

---

## 数据流

```
                    ┌─ 轮式里程计 /odom ─────────────┐ (只用 x + vx：管位置)
                    │                                ▼
ICM-20948 ──► icm20948_driver ──► imu_filter_madgwick ──► ekf_node ──► /odom_combined
 (6轴用法)      ①开机标零偏          (use_mag=FALSE)      (只融 vyaw)    (x, y, yaw)
                ②ZUPT治温漂       陀螺积分yaw+加速度纠roll/pitch  航向从0积分      │
                                                                                  ▼
      录制轨迹 path_xxx_smooth.yaml ────► odom_waypoint_follower ────► /cmd_vel ──► 底盘
      (每2cm一点 + 每点录制车头yaw)        Stanley + 曲率前馈          (v, ω)
```

---

## 硬件 / 依赖

本仓库**按下面这套硬件写死**（参数、转向契约、I²C 地址都是实测值），照着装就能跑：

| 部件 | 型号 |
|---|---|
| 底盘 | OriginCar 阿克曼底盘（STM32F407ZG 下位机，ROS 包 `origincar_base`） |
| 主控 | 地平线 RDK S100 / RDK X5（ROS 2 Humble） |
| 惯导 | ICM-20948，挂 40Pin 的 **i2c-5**，地址 0x68 或 0x69（模块 AD0 电平决定） |
| 下位机固件 | **建议用改过的版本**：舵机转向对称化 + 串口上报 50Hz（见下面「下位机固件」一节） |

> 硬件不同（换底盘/换 IMU）也不是不能用，但至少要重新确认**轴距、转角上限、
> 以及下位机对 `angular.z` 的解释**这三件事，见「上车前的一次性确认」。

**软件依赖**

- ROS 2 Humble
- `origincar_base`（底盘官方包，装在车上，发 `/odom`、收 `/cmd_vel`）
- `robot_localization`（提供 `ekf_node`）
- `imu_filter_madgwick`（**必须 `use_mag:=false`**）
- Python：`numpy`、`scipy`、`matplotlib`（仅 `smooth_path` 平滑工具需要）

```bash
sudo apt install ros-$ROS_DISTRO-robot-localization ros-$ROS_DISTRO-imu-filter-madgwick
pip3 install numpy scipy matplotlib
```

## 安装

```bash
cd ~/your_ws/src
git clone <本仓库地址> ahpu_odom_nav
cd ~/your_ws
colcon build --packages-select ahpu_odom_nav
source install/setup.bash
```

---

## 快速开始

### 0. 先确认硬件

```bash
i2cdetect -y 5          # 找到 ICM 的地址：0x68(=104) 或 0x69(=105)
```

### 1. 起全链路（底盘 + 惯导 + EKF）

```bash
# ⚠️ 车放在发车点，朝向摆正，开机后 3 秒内不要碰车（陀螺零偏标定期）
ros2 launch ahpu_odom_nav bringup.launch.py imu_addr:=104
# 底盘已在别处起好 → 只起惯导部分：ros2 launch ahpu_odom_nav imu_ekf.launch.py
```

看到 `陀螺零偏标定完成(300 样本)：bias ... → 已扣除` 就算就绪。

**先做一次符号验证**（只做一次，之后不用管）：

```bash
ros2 run ahpu_odom_nav odom_monitor     # 另开一个终端
```
把车**原地左转**，方向角读数应当**增大**。如果反了，加 `gyro_z_sign:=-1.0` 重起。

### 2. 录制轨迹

```bash
ros2 run ahpu_odom_nav waypoint_capture --ros-args -p path_name:=my_route
#   s = 开始录制 → 遥控顺着路线开一遍 → q = 结束写文件
```
生成 `path_my_route.yaml`（默认写在当前目录）。

### 3. 平滑 + 均匀重采样

```bash
ros2 run ahpu_odom_nav smooth_path path_my_route.yaml 0.02
```
生成 `path_my_route_smooth.yaml`（可直接回放）+ `path_my_route.png` 路线图
（蓝=前进 / 红=倒车 / 品红十字=掉头尖点），**先看图确认路线对**再上车。

### 4. 回放

```bash
# ⚠️ 把车放回【同一发车点、同一朝向】，重新静止开机，重跑第 1 步
ros2 launch ahpu_odom_nav odom_nav.launch.py \
     params_file:=$PWD/path_my_route_smooth.yaml
```

**只想先验证链路通不通**？包里带了一条合成的直角弯演示轨迹（3×3m 空地即可）：

```bash
ros2 launch ahpu_odom_nav odom_nav.launch.py \
     params_file:=$(ros2 pkg prefix ahpu_odom_nav)/share/ahpu_odom_nav/config/path_demo_corner.yaml
```

---

## 上车前的一次性确认

同款硬件下只有这几项需要实车过一遍，确认完就不用再管：

| # | 确认什么 | 怎么做 |
|---|---|---|
| 1 | **IMU 的 I²C 地址** | `i2cdetect -y 5` 看是 0x68(=104) 还是 0x69(=105)，写进 `imu_addr` |
| 2 | **yaw 符号** | 起 `odom_monitor`，把车**原地左转**，方向角应当**增大**；反了加 `gyro_z_sign:=-1.0` |
| 3 | **倒车转向** | 录一段带倒车的轨迹回放，倒车打反就把 `reverse_steer_sign` 从 −1.0 翻成 +1.0 |
| 4 | **速度调参** | 从 `linear_speed: 0.20` 起步：先把直线跑稳（`stanley_k` 调到不发抖），再调弯道（`curv_ff_gain`） |

### `angular.z` 的契约（改底盘时唯一要动的地方）

本车 STM32 把 `angular.z` 当**角速度 ω** 读，并按 `R = Vx/Vz` 反解阿克曼转角。
`_omega_for_delta()` 就是**按这个固件模型精确反解**的（所以要填 `fw_axle_spacing` 等三个固件常数），
保证固件最终算出来的转角就是 Stanley 想要的 δ。

换成别的底盘时，只改这一个函数：底盘直接读转角就 `return delta`；
底盘用教科书自行车模型就 `return v_signed * math.tan(delta) / self.wheelbase`。
**搞错这一环车会以很诡异的方式转圈，请先低速验证。**

## 参数速查（主调钮）

在 `config/waypoints.yaml`（或录出来的 `path_*_smooth.yaml`）里改，**重启即生效，不用重新编译**。

| 参数 | 默认 | 说明 |
|---|---|---|
| `stanley_k` | 1.2 | 横向纠偏增益。**大→纠偏狠/收敛快；过大→发抖画龙** |
| `stanley_ks` | 0.4 | 软化项，低速防过冲。一般不动 |
| `heading_gain` | 1.0 | 车头对齐路径的力度 |
| `curv_ff_gain` | 1.4 | 曲率前馈。弯道滞后/走大弯就调大（@0.6m/s 标定值） |
| `progress_window` | 8 | 前向搜索窗口。**必须 < 折返处的索引跨度**，否则掉头时会跨腿跳点 |
| `delta_rate_max` | 3.0 | 转角斜率限幅，治高速极限环（只砍突变，不引入滞后） |
| `linear_speed` | 0.20 | 巡航速度。也可 `speed:=0.4` 临时覆盖 |
| `final_tolerance` | 0.04 | 终点判定半径 |
| `final_yaw_tol_deg` | -1 | 终点航向门；>0 时车头没对齐就不算到达（边走边纠） |

调参时订 `/odom_nav/debug`（`Float32MultiArray`）：
`[idx, x, y, yaw, e, he, delta, delta_max, v, kappa, dist_to_end]`

---

## 工程铁律（血泪换的）

1. **发车点严格同朝向开机** —— 航向从 0 起算，朝向错了整条轨迹偏。
2. **开机头 3 秒静止别碰车** —— 陀螺零偏标定期，动了标定就废，**而且不报错**。
3. **IMU 平放固定** —— 安装偏航角**不用标定**（恒定偏差只是把整个 odom 帧转一个固定角，
   录制和回放同帧、自动抵消），但**松动**会毁掉一切。
4. **I²C 地址锁死在配置里** —— AD0 接触不良会让地址在 0x68/0x69 之间随机跳。
5. **串口按序列号绑 udev 别名** —— 底盘和另一个 USB-CDC 设备抢 `/dev/ttyACM0`，谁先枚举谁赢。
6. **退出必须停车** —— 很多底盘固件没有 `cmd_vel` 失联保护，上位机一退，
   车会**保持最后的速度继续跑**。本节点退出时连发 5 次零速兜底，但强烈建议
   **在固件侧加看门狗**：超过 N ms 没收到指令自动刹停。

---

## 目录结构

```
ahpu_odom_nav/
├── ahpu_odom_nav/
│   ├── icm20948_driver.py         # IMU 驱动：开机零偏标定 + ZUPT 温漂跟踪（纯 stdlib i2c-dev）
│   ├── waypoint_capture.py        # 录制：每 2cm 记一点 + 车头 yaw
│   ├── smooth_path.py             # 平滑 + 均匀重采样 + 路线图（按掉头尖点分段）
│   ├── path_geometry.py           # 路径几何：点间距 + 有符号曲率 κ
│   ├── odom_waypoint_follower.py  # 跟踪主节点：Stanley + 曲率前馈 + 忠实倒车
│   └── odom_monitor.py            # 漂移/标度诊断：静置量漂移、直走校轮径、原地转校陀螺
├── config/
│   ├── ekf_odom.yaml              # ⭐ EKF 配置：轮速管位置、陀螺管航向（注释很全，可直接抄）
│   ├── waypoints.yaml             # 参数模板（录制时作骨架）
│   └── path_demo_corner.yaml      # 合成演示轨迹：直角弯，用于链路自检
└── launch/
    ├── bringup.launch.py           # 一键起全链路：底盘 + 惯导 + EKF
    ├── imu_ekf.launch.py           # 只起惯导定位底座（底盘已在别处启动时用）
    └── odom_nav.launch.py          # 轨迹跟踪（回放）
```

---

## 下位机固件

上位机算得再准，最终把转角变成舵机 PWM 的是 STM32。这一层做了两处改动，都建议照做：

**① 舵机转向对称化** —— 原厂映射公式里的偶次项让车**左转打得少、右转打得多**，
而 Stanley 内部假设左右转向对称（同一套增益处理左右偏差），不对称等于左右等效增益不同，
直接损害复现性。改法是只保留奇次项：

```c
// BALANCE/balance.c（阿克曼分支）
// 改前：Angle_Servo = -0.628f*pow(AngleR,3) + 1.269f*pow(AngleR,2) - 1.772f*AngleR + 1.573f;
//       Servo = SERVO_INIT + (Angle_Servo - 1.572f)*Ratio;
Angle_Servo = -0.628f*pow(AngleR,3) - 1.772f*AngleR;
Servo = SERVO_INIT + Angle_Servo*Ratio;
```

改后 ±0.32 rad 处左右偏离中位严格相等（旧公式差 167 个 PWM），且 0 时精确回中位。

**② 串口上报 20Hz → 50Hz** —— `HARDWARE/usartx.c` 的 `data_task` 把
`F2T(RATE_20_HZ)` 换成 `F2T(RATE_50_HZ)`（波特率 115200 不用动）。
整条链是「下位机上报 → EKF → follower」，而 follower 直接用最近收到的位姿、
**不做时间戳外推**，所以三者必须一起是 50Hz：下位机上报 50、EKF `frequency: 50.0`、
follower `control_rate: 50.0`。少改一个，另外两个的收益就被卡死。

### ⚠️ 还没做、但强烈建议你先做的：cmd_vel 看门狗

原厂固件里的 `command_lost_count` 失联保护**被注释掉了**，速度是全局变量、只在收到串口
数据时更新 —— **上位机一退出，底盘会保持最后的速度继续跑**（实测 Ctrl-C 打断巡航，车一直冲）。

本仓库的兜底是节点退出时连发 5 次零速，但**进程被 `kill -9` 时它一样不执行**。
真正的解法是在固件里加看门狗：超过 N ms 没收到 `cmd_vel` 自动零速刹停。

其余可继续优化的固件项（转向模型统一、里程计标定、控制周期加时间戳、硬件级急停、
倒车符号统一）见 [docs/技术详解.md 第 8 节](docs/技术详解.md)。

## 文档

- [docs/技术详解.md](docs/技术详解.md) —— 完整技术文章：三代方案的演进、每个决策的取舍、
  Stanley 各项的作用、实测数据。想搞懂"为什么这么做"的看这篇。

## License

MIT
