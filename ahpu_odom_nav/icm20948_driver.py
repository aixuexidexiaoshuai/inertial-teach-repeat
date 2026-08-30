#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ICM-20948 IMU 驱动（**当 6 轴用**：只读陀螺 + 加速度，不读磁力计）。

设计取舍（本方案的核心）：
  ICM-20948 是 9 轴（含 AK09916 磁力计），磁力计能给【绝对航向】，理论上漂移有界、
  冷启动即用。但在金属结构密集的场地里磁畸变会让航向跳数十度，同一朝向换个位置读数就变。
  本方案主动放弃"绝对"，只用陀螺积分 + 开机归零，换取【每次开机完全一致】的相对航向。
  → 磁力计整段不读（省 I²C 带宽，也少一个 Remote I/O 出错源）；
  → 下游 imu_filter_madgwick 必须 use_mag:=false。

本节点只做【读取 + 零偏处理 + 发布】，姿态解算交给 imu_filter_madgwick：
  - 读 ICM-20948 加速度 + 陀螺（纯 stdlib i2c-dev ioctl，零外部库）
  - ① 开机静止 gyro_calib_sec 秒标定陀螺零偏，之后逐帧扣除
  - ② ZUPT：运行中检测到"真静止"时把残留零偏缓慢吸收进 bias → 跟踪温漂
  - 发布 /icm/data_raw (sensor_msgs/Imu)：加速度(m/s²) + 角速度(rad/s)，无 orientation

下游链路：
  icm20948_driver → /icm/data_raw
    → imu_filter_madgwick(use_mag=false) → /imu/data
      → robot_localization ekf_node(只融 vyaw) → /odom_combined

⚠️ 铁律：开机后 gyro_calib_sec(默认 3s) 内车必须【静止】，碰一下标定就废（且不会报错）。
⚠️ IMU 平放固定；安装偏航角无需标定（见 README：录/放同帧自动抵消），
   唯一需实车确认的是 yaw 符号（左转 yaw 应增大，反了就设 gyro_z_sign:=-1.0）。
"""

import fcntl
import math
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

I2C_SLAVE = 0x0703

# —— ICM-20948 寄存器（bank0 除非注明）——
REG_BANK_SEL = 0x7F
WHO_AM_I = 0x00
USER_CTRL = 0x03
PWR_MGMT_1 = 0x06
PWR_MGMT_2 = 0x07
INT_PIN_CFG = 0x0F
ACCEL_XOUT_H = 0x2D
GYRO_XOUT_H = 0x33
# bank2
GYRO_SMPLRT_DIV = 0x00
GYRO_CONFIG_1 = 0x01
ACCEL_SMPLRT_DIV_1 = 0x10
ACCEL_SMPLRT_DIV_2 = 0x11
ACCEL_CONFIG = 0x14

# —— 量程标度（与下面 _init_icm 写入的配置一致）——
ACCEL_LSB_PER_G = 16384.0   # ±2g
GYRO_LSB_PER_DPS = 65.5     # ±500dps
G = 9.80665
DEG2RAD = math.pi / 180.0


def _s16_be(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v >= 32768 else v


class I2CDev:
    """极简 i2c-dev 封装：set_addr 选从机，写 reg 指针再读（地址自增）。"""

    def __init__(self, bus):
        self.fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
        self.addr = None

    def set_addr(self, addr):
        if addr != self.addr:
            fcntl.ioctl(self.fd, I2C_SLAVE, addr)
            self.addr = addr

    def wr(self, addr, reg, val):
        self.set_addr(addr)
        os.write(self.fd, bytes([reg, val]))

    def rd(self, addr, reg, n=1, retries=3):
        # 偶发 [Errno 121] Remote I/O(总线 NAK) 重试：每次微延时让总线复位，
        # 掩盖偶发 NAK，丢帧几乎归零；全部失败才抛，由上层跳帧。
        last = None
        for _ in range(max(1, retries)):
            try:
                self.set_addr(addr)
                os.write(self.fd, bytes([reg]))
                return os.read(self.fd, n)
            except OSError as e:
                last = e
                self.addr = None          # 强制下次重选从机(总线可能已错乱)
                time.sleep(0.0005)        # 0.5ms 让总线安顿
        raise last

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass


class Icm20948Driver(Node):
    def __init__(self):
        super().__init__("icm20948_driver")
        self.declare_parameter("i2c_bus", 5)
        # ⚠️ I²C 地址随模块 AD0 电平而变：0x68(104) 或 0x69(105)。
        #    先用 `i2cdetect -y <bus>` 确认；接触不良会让地址随机跳变 → 锁死配置 + 修排线。
        self.declare_parameter("addr", 0x68)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("imu_topic", "/icm/data_raw")
        # 陀螺 z 符号：IMU 装反(绕竖直轴翻 180°)时把 yaw 方向翻回来。
        #   实车验证：左转 /odom_combined 的 yaw 应【增大】；反了就设 -1.0。
        self.declare_parameter("gyro_z_sign", 1.0)
        # 开机静止标零偏：测 gyro_calib_sec 秒均值当零偏，之后扣掉 → 积分干净。
        #   纯陀螺方案必须；否则零偏直接积分成航向漂移（1.9°/s 的零偏，静止 3s 就假偏 5.6°）。
        self.declare_parameter("gyro_calib_enable", True)
        self.declare_parameter("gyro_calib_sec", 3.0)
        # ZUPT 静止零偏跟踪（治温漂）：车静止时(三轴都<阈值持续一小段)把残留零偏慢慢吸进
        #   bias → 跟着温度走；跑动时立即冻结。阈值取噪声峰(~0.6°/s)之上、真转动(>5°/s)之下。
        self.declare_parameter("zupt_enable", True)
        self.declare_parameter("zupt_gyro_thresh_dps", 1.5)   # 三轴|扣偏后角速度|都<此(°/s)算静止
        self.declare_parameter("zupt_window_sec", 0.5)        # 需连续静止此时长才更新
        self.declare_parameter("zupt_ema", 0.01)              # 静止时零偏跟踪率(每帧,时间常数≈1s)

        self.bus = int(self.get_parameter("i2c_bus").value)
        self.addr = int(self.get_parameter("addr").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        rate = float(self.get_parameter("rate_hz").value)
        self.gyro_z_sign = float(self.get_parameter("gyro_z_sign").value)
        # 陀螺零偏标定状态
        gcal_en = bool(self.get_parameter("gyro_calib_enable").value)
        self.gcal_n = max(1, int(float(self.get_parameter("gyro_calib_sec").value) * rate))
        self.gbias = [0.0, 0.0, 0.0]
        self.gsum = [0.0, 0.0, 0.0]
        self.gcnt = 0
        self.gcal_done = (not gcal_en)
        # ZUPT 状态
        self.zupt_enable = bool(self.get_parameter("zupt_enable").value)
        self.zupt_thresh = float(self.get_parameter("zupt_gyro_thresh_dps").value) * DEG2RAD
        self.zupt_win = max(1, int(float(self.get_parameter("zupt_window_sec").value) * rate))
        self.zupt_ema = float(self.get_parameter("zupt_ema").value)
        self._still_cnt = 0     # 连续静止帧计数

        self.dev = I2CDev(self.bus)
        self._init_icm()

        self.pub_imu = self.create_publisher(
            Imu, self.get_parameter("imu_topic").value, 20)
        self.create_timer(1.0 / rate, self.on_timer)

        self.get_logger().info(
            f"ICM-20948 驱动启动：i2c-{self.bus} @0x{self.addr:02x}，{rate:.0f}Hz，"
            f"发 {self.get_parameter('imu_topic').value}（frame={self.frame_id}，"
            f"gyro_z_sign={self.gyro_z_sign:+.0f}）"
            f"｜⚠️ 开机 {float(self.get_parameter('gyro_calib_sec').value):.0f}s 内请勿碰车（标零偏）")

    # ---- 初始化 ----
    def _bank(self, b):
        self.dev.wr(self.addr, REG_BANK_SEL, (b & 0x03) << 4)

    def _init_icm(self):
        who = self.dev.rd(self.addr, WHO_AM_I)[0]
        if who != 0xEA:
            self.get_logger().warn(f"WHO_AM_I=0x{who:02x}（期望 0xEA），检查接线/地址")
        self._bank(0)
        self.dev.wr(self.addr, PWR_MGMT_1, 0x80)   # 软复位
        time.sleep(0.1)
        self.dev.wr(self.addr, PWR_MGMT_1, 0x01)   # 清睡眠、自动时钟
        time.sleep(0.01)
        self.dev.wr(self.addr, PWR_MGMT_2, 0x00)   # 使能加速度+陀螺
        self.dev.wr(self.addr, USER_CTRL, 0x00)    # 关内部 I2C master
        self.dev.wr(self.addr, INT_PIN_CFG, 0x02)
        time.sleep(0.01)
        self._bank(2)
        # 陀螺 ±500dps + DLPF 开（DLPFCFG=0）：(FS_SEL=1<<1)|FCHOICE=1 = 0x03
        self.dev.wr(self.addr, GYRO_CONFIG_1, 0x03)
        self.dev.wr(self.addr, GYRO_SMPLRT_DIV, 0x0A)     # ODR≈1100/(1+10)=100Hz
        # 加速度 ±2g + DLPF 开：FS_SEL=0, FCHOICE=1 = 0x01
        self.dev.wr(self.addr, ACCEL_CONFIG, 0x01)
        self.dev.wr(self.addr, ACCEL_SMPLRT_DIV_1, 0x00)
        self.dev.wr(self.addr, ACCEL_SMPLRT_DIV_2, 0x0A)  # ODR≈1125/(1+10)≈102Hz
        self._bank(0)
        time.sleep(0.01)

    # ---- 周期读取 + 发布 ----
    def on_timer(self):
        now = self.get_clock().now().to_msg()
        try:
            a = self.dev.rd(self.addr, ACCEL_XOUT_H, 6)
            g = self.dev.rd(self.addr, GYRO_XOUT_H, 6)
        except OSError as e:
            self.get_logger().warn(f"读 ICM 失败：{e}", throttle_duration_sec=2.0)
            return

        ax = _s16_be(a[0], a[1]) / ACCEL_LSB_PER_G * G
        ay = _s16_be(a[2], a[3]) / ACCEL_LSB_PER_G * G
        az = _s16_be(a[4], a[5]) / ACCEL_LSB_PER_G * G
        gx = _s16_be(g[0], g[1]) / GYRO_LSB_PER_DPS * DEG2RAD
        gy = _s16_be(g[2], g[3]) / GYRO_LSB_PER_DPS * DEG2RAD
        gz = _s16_be(g[4], g[5]) / GYRO_LSB_PER_DPS * DEG2RAD

        # ① 开机零偏标定：先累计取均值当零偏，期间对外发 0（此刻车必须静止，
        #    发 0 比发未标定的脏数据安全），完成后逐帧扣掉零偏 → 积分干净。
        if not self.gcal_done:
            self.gsum[0] += gx; self.gsum[1] += gy; self.gsum[2] += gz; self.gcnt += 1
            gx = gy = gz = 0.0
            if self.gcnt >= self.gcal_n:
                self.gbias = [self.gsum[i] / self.gcnt for i in range(3)]
                self.gcal_done = True
                self.get_logger().info(
                    "陀螺零偏标定完成(%d 样本)：bias %.2f/%.2f/%.2f °/s → 已扣除，开始正常发布" % (
                        self.gcnt, self.gbias[0] / DEG2RAD, self.gbias[1] / DEG2RAD,
                        self.gbias[2] / DEG2RAD))
        else:
            gx -= self.gbias[0]; gy -= self.gbias[1]; gz -= self.gbias[2]
            # ② ZUPT：静止(三轴扣偏后角速度都<阈值)持续 zupt_win 帧 → 把残留慢慢吸进 gbias。
            #    跑动(任一轴超阈)立即复位、不更新 —— 否则会把转弯当成零偏吸进去，越校越错。
            if self.zupt_enable:
                if abs(gx) < self.zupt_thresh and abs(gy) < self.zupt_thresh and abs(gz) < self.zupt_thresh:
                    self._still_cnt += 1
                    if self._still_cnt >= self.zupt_win:
                        self.gbias[0] += self.zupt_ema * gx
                        self.gbias[1] += self.zupt_ema * gy
                        self.gbias[2] += self.zupt_ema * gz
                        self.get_logger().info(
                            "[ZUPT] 静止校准零偏 z=%.4f°/s" % (self.gbias[2] / DEG2RAD),
                            throttle_duration_sec=10.0)
                else:
                    self._still_cnt = 0

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self.frame_id
        # 无 orientation 估计（交给 madgwick）：协方差[0]=-1 按 REP-145 约定
        imu.orientation_covariance[0] = -1.0
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz * self.gyro_z_sign     # 装反时翻 yaw 方向
        imu.linear_acceleration_covariance[0] = 0.05
        imu.linear_acceleration_covariance[4] = 0.05
        imu.linear_acceleration_covariance[8] = 0.05
        imu.angular_velocity_covariance[0] = 0.005
        imu.angular_velocity_covariance[4] = 0.005
        imu.angular_velocity_covariance[8] = 0.005
        self.pub_imu.publish(imu)

    def destroy_node(self):
        self.dev.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = Icm20948Driver()
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
