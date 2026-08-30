#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
录制轨迹的【平滑拟合 + 均匀弧长重采样 + 路线图】工具（teach-and-repeat 的中间一步）。

waypoint_capture 录的原始点带遥控抖动，且在掉头处含后退段（前进→后退→前进，
方向反转处是一个尖点 cusp，样条不能直接跨过去拟合）。本工具：
  1. 先按【运动方向反转】把路径切成若干【单向段】（每段全前进或全后退），尖点处断开；
  2. 每段按记录顺序拟合三次平滑样条 + 均匀弧长重采样（默认 2cm）；
  3. 输出每点 yaw = 【车头朝向】（前进段 = 运动切线；后退段 = 运动切线 + 180°）——
     保留前进/后退信息，follower 据此判定该点是否倒车，忠实倒着回放；
  4. 末尾 end_taper 弧长内把 yaw 平滑拉回【录制末点朝向】（样条会把末段转弯圆滑掉，
     导致平滑后终点朝向与录制不一致；带 taper 而非只钉末点，是为了让航向前馈不在末点突跳）；
  5. 写出 <name>_smooth.yaml（可直接回放）+ <name>.png 路线图（前进蓝/后退红/尖点标记）。

用法：
  ros2 run ahpu_odom_nav smooth_path <path_xxx.yaml> [ds=0.02] [smooth_s=0.0015] [end_taper=0.50]
  # 或直接 python3 smooth_path.py <path_xxx.yaml> [ds] [s] [end_taper]

平滑度 s：太小 = 贴住噪声不平滑；太大 = 切角/抄近道。0.0015~0.004 之间调。
依赖：numpy、scipy、matplotlib。
"""

import sys
import os
import math
import yaml
import numpy as np

CUSP_DEG = 120.0   # 相邻运动方向变化 > 此角度 = 判定为掉头尖点（纯转弯在密点下每步只变几度）
END_TAPER_M = 0.50 # 终点航向约束：末尾此弧长内把 yaw 平滑 blend 到录制末点朝向（治平滑后终点朝向漂移）


def _norm(a):
    return math.atan2(math.sin(a), math.cos(a))


def _resample_segment(sx, sy, ds, s_fac):
    """对一段(单向)点做样条平滑 + 均匀弧长重采样，返回 (rx, ry, tangent_rad)。"""
    from scipy.interpolate import splprep, splev
    m = len(sx)
    if m < 3:
        # 点太少直接线性
        rx = np.array(sx, float); ry = np.array(sy, float)
        tang = math.atan2(ry[-1] - ry[0], rx[-1] - rx[0])
        return rx, ry, np.full(len(rx), tang)
    s = s_fac * m
    tck, u = splprep([sx, sy], s=s, k=min(3, m - 1))
    uu = np.linspace(0, 1, max(2000, m * 40))
    xx, yy = splev(uu, tck)
    seg = np.hypot(np.diff(xx), np.diff(yy))
    arc = np.concatenate([[0], np.cumsum(seg)])
    L = arc[-1]
    n_new = max(2, int(round(L / ds)) + 1)
    targ = np.linspace(0, L, n_new)
    ui = np.interp(targ, arc, uu)
    rx, ry = splev(ui, tck)
    dx, dy = splev(ui, tck, der=1)
    tang = np.arctan2(dy, dx)
    return np.asarray(rx), np.asarray(ry), np.asarray(tang)


def smooth_one(f, ds=0.02, s_fac=0.0015, end_taper=END_TAPER_M):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    doc = yaml.safe_load(open(f))
    P = doc["odom_waypoint_follower"]["ros__parameters"]
    x = np.array(P["waypoints_x"], float)
    y = np.array(P["waypoints_y"], float)
    yaw_raw = P.get("waypoints_yaw", None)
    n0 = len(x)
    has_yaw = isinstance(yaw_raw, list) and len(yaw_raw) == n0   # 全量 yaw 才用来判前进/后退
    yaw_raw = np.array(yaw_raw, float) if has_yaw else None

    # 去掉相邻重复点（splprep 要求参数严格递增），yaw 同步保留
    keep = [0]
    for i in range(1, n0):
        if math.hypot(x[i] - x[keep[-1]], y[i] - y[keep[-1]]) > 1e-3:
            keep.append(i)
    x, y = x[keep], y[keep]
    if has_yaw:
        yaw_raw = yaw_raw[keep]
    m = len(x)
    if m < 2:
        raise ValueError(f"有效点太少（{m}）")

    # 运动方向 md[i]（点 i→i+1），检测掉头尖点
    md = np.arctan2(np.diff(y), np.diff(x))          # len m-1
    cusps = [i for i in range(1, len(md))
             if abs(_norm(md[i] - md[i - 1])) > math.radians(CUSP_DEG)]
    bounds = [0] + cusps + [m - 1]
    segs = [(bounds[k], bounds[k + 1]) for k in range(len(bounds) - 1)
            if bounds[k + 1] > bounds[k]]

    RX, RY, RFACE, REV = [], [], [], []
    seg_info = []
    end_dpsi = None                                    # 终点航向被校正的角度（None=未约束）
    for si, (a, b) in enumerate(segs):
        sx, sy = x[a:b + 1], y[a:b + 1]
        # 前进/后退：录制车头 yaw 与运动方向同向=前进
        if has_yaw:
            c = np.mean([math.cos(math.radians(yaw_raw[i]) - md[i]) for i in range(a, b)])
            fwd = c >= 0.0
        else:
            fwd = True
        rx, ry, tang = _resample_segment(sx, sy, ds, s_fac)
        face = tang if fwd else tang + math.pi        # 车头朝向：后退段=运动切线+180°
        face = np.arctan2(np.sin(face), np.cos(face))
        # ── 终点航向约束 ────────────────────────────────────────────────────
        #   样条重采样后末点 yaw = 样条端点切线，会把录制末尾那段转弯【圆滑掉】，
        #   导致平滑后终点朝向每次都和录制不一致（终点航向门会因此判不过）。
        #   修法：仅【最后一段前进段】的末尾 END_TAPER_M 弧长内，
        #   把 face 从样条切线平滑 blend 到【录制末点朝向】yaw_raw[-1]，末点精确落到录制值。
        #   带 taper（而非只钉末点）是为了让航向前馈不在末点突跳。
        if (has_yaw and si == len(segs) - 1 and fwd and len(face) >= 2
                and end_taper > 1e-6):
            end_face = math.radians(float(yaw_raw[-1]))     # 录制末点车头朝向（目标）
            seg_len = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(rx), np.diff(ry)))])
            s_from_end = seg_len[-1] - seg_len              # 每点到末点的剩余弧长
            w = np.clip(1.0 - s_from_end / max(end_taper, 1e-6), 0.0, 1.0)  # 末点=1，taper 起点=0
            # 沿最短弧插值：face + w·wrap(end_face − face)
            dpsi = np.arctan2(np.sin(end_face - face), np.cos(end_face - face))
            end_dpsi = float(dpsi[-1])                  # 末点处样条切线→录制朝向的校正量
            face = np.arctan2(np.sin(face + w * dpsi), np.cos(face + w * dpsi))
        start = 1 if si > 0 else 0                     # 跳过与上段共享的尖点，避免重复/零切线
        RX.extend(rx[start:]); RY.extend(ry[start:])
        RFACE.extend(np.degrees(face[start:])); REV.extend([0 if fwd else 1] * len(rx[start:]))
        seg_info.append((len(rx[start:]), fwd))

    RX, RY, RFACE, REV = map(np.array, (RX, RY, RFACE, REV))
    n_new = len(RX)
    L = float(np.sum(np.hypot(np.diff(RX), np.diff(RY))))
    n_rev = int(np.sum(REV))

    # 写出 smooth yaml（复制模板，替换三数组）
    def arr(a, p=4):
        return "[" + ", ".join(f"{v:.{p}f}" for v in a) + "]"
    base = os.path.basename(f)[:-5]
    out_yaml = os.path.join(os.path.dirname(f), base + "_smooth.yaml")
    out = []
    for ln in open(f).read().splitlines(keepends=True):
        st = ln.lstrip()
        ind = ln[:len(ln) - len(st)]
        if st.startswith("waypoints_x:"):
            out.append(f"{ind}waypoints_x:   {arr(RX)}\n")
        elif st.startswith("waypoints_y:"):
            out.append(f"{ind}waypoints_y:   {arr(RY)}\n")
        elif st.startswith("waypoints_yaw:"):
            out.append(f"{ind}waypoints_yaw: {arr(RFACE, 2)}\n")
        else:
            out.append(ln)
    open(out_yaml, "w").writelines(out)

    # 路线图
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.plot(x, y, "o", ms=3, color="#cccccc", label=f"raw ({m} pts)")
    fwd_mask = REV == 0
    ax.plot(RX[fwd_mask], RY[fwd_mask], ".", ms=6, color="#1f77b4", label="forward")
    if n_rev:
        ax.plot(RX[REV == 1], RY[REV == 1], ".", ms=7, color="#d62728", label=f"reverse ({n_rev})")
    q = max(1, n_new // 40)
    ax.quiver(RX[::q], RY[::q], np.cos(np.radians(RFACE[::q])), np.sin(np.radians(RFACE[::q])),
              color="#ff7f0e", scale=25, width=0.004, label="car heading (yaw)")
    for c in cusps:
        ax.plot([x[c]], [y[c]], "P", ms=13, color="magenta")
    if cusps:
        ax.plot([], [], "P", color="magenta", label=f"turnaround cusp ({len(cusps)})")
    ax.plot(RX[0], RY[0], "s", ms=12, color="green", label="start")
    ax.plot(RX[-1], RY[-1], "*", ms=18, color="black", label="end")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_title(f"{base}  L={L:.2f}m  segs={len(segs)}  rev_pts={n_rev}")
    ax.set_xlabel("x (m, odom)")
    ax.set_ylabel("y (m)")
    ax.legend(loc="best", fontsize=8)
    png = os.path.join(os.path.dirname(f), base + ".png")
    fig.savefig(png, dpi=110, bbox_inches="tight")
    return out_yaml, png, n0, m, n_new, L, len(segs), n_rev, end_dpsi


def main():
    if len(sys.argv) < 2:
        print("用法: ros2 run ahpu_odom_nav smooth_path <path_xxx.yaml> [ds=0.02] [s=0.0015] [end_taper=0.50]")
        sys.exit(1)
    f = sys.argv[1]
    ds = float(sys.argv[2]) if len(sys.argv) > 2 else 0.02
    s_fac = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0015
    end_taper = float(sys.argv[4]) if len(sys.argv) > 4 else END_TAPER_M
    out_yaml, png, n0, m, n_new, L, nseg, n_rev, end_dpsi = smooth_one(f, ds, s_fac, end_taper)
    print(f"smoothed yaml -> {out_yaml}")
    print(f"route map     -> {png}")
    print(f"raw {n0}pts -> dedup {m} -> smoothed {n_new}pts, arclen {L:.2f}m, {nseg} 段, {n_rev} 倒车点")
    if end_dpsi is not None:
        print(f"终点航向约束: 末尾 {end_taper:.2f}m blend 到录制末点朝向, "
              f"较样条切线校正 {math.degrees(end_dpsi):+.1f}°")


if __name__ == "__main__":
    main()
