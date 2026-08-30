#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
路径几何：点间距 ds + 每点有符号曲率 κ（供 Stanley 的曲率前馈用）。

曲率前馈需要"录制路径这一点该弯多少"。κ 由【固定弧长基线】的三点外接圆算：
  基线太短 → 贴住录点噪声，κ 抖；基线太长 → 把圆角抹平，弯道前馈不足。
0.15m 是实车调定值（2cm 点距 → 前后各取 7~8 个点）。

符号约定：左转为正（与 Stanley 的横向误差 e 左正右负一致）。
"""

import math


def path_spacing(waypoints, default_ds=0.02):
    """量出路径平均点间距 ds（m）。smooth_path 的重采样间距是命令行参数，2cm/10cm 都可能。"""
    n = len(waypoints)
    if n < 2:
        return default_ds
    tot = sum(math.hypot(waypoints[i + 1][0] - waypoints[i][0],
                         waypoints[i + 1][1] - waypoints[i][1]) for i in range(n - 1))
    return max(tot / (n - 1), 1e-3)


def path_curvature(waypoints, ds, baseline=0.15):
    """
    每点有符号曲率 κ（1/m，左正右负）。首尾各 k 个点无法取三点外接圆，保持 0。

    κ = 2·cross / (a·b·c)   —— 三点外接圆半径的标准公式的倒数形式
      a,b,c = 三点两两距离，cross = 叉积（定符号：左转为正）
    """
    n = len(waypoints)
    kappa = [0.0] * n
    if n < 3:
        return kappa
    k = max(1, int(round(baseline / max(ds, 1e-6))))
    for i in range(k, n - k):
        ax, ay = waypoints[i - k]
        bx, by = waypoints[i]
        cx, cy = waypoints[i + k]
        a = math.hypot(bx - ax, by - ay)
        b = math.hypot(cx - bx, cy - by)
        c = math.hypot(cx - ax, cy - ay)
        cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        # 分别检查 a,b,c：单个极小值就会让除法不稳定（共线时 cross≈0 直接跳过）
        if abs(cross) < 1e-12 or min(a, b, c) < 1e-6:
            continue
        kappa[i] = 2.0 * cross / (a * b * c)
    return kappa
