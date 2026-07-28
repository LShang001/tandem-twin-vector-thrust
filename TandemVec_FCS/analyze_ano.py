#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AnoCom 姿态数据统计分析（Python 版，numpy + matplotlib 升级）

用法:
    python analyze_ano.py [ano_raw.bin]            # 仅打印文本报告
    python analyze_ano.py ano_raw.bin --plot       # 额外弹出图表
    python analyze_ano.py ano_raw.bin --save fig.png --csv out.csv

解码逻辑与 lib/AnoComProtocol 严格对齐（已与固件逐字节核对）：
    帧结构 : AB | src | dst | func | len(2B小端) | data | SC | AC
    双校验 : SC = 逐字节和 & 0xFF ; AC = "和的和" & 0xFF（uint8 逐步截断）
    0x03   : roll/pitch/yaw = int16 小端 ×100 (度) , fusion @ data[6]
    采样率 : 姿态帧实际间隔 20ms（handleAnoCom@200Hz，4 包分组 → 0x03 为 50Hz）

升级点（相对 JS 版）：
    - numpy 向量化统计 / 校验
    - 漂移率改用最小二乘线性拟合（np.polyfit）+ R²，比首尾差更稳健
    - matplotlib 画 roll/pitch/yaw 时间序列与 yaw 解缠漂移趋势
    - CSV 导出，便于外部工具二次分析
    - 解码 fusionSta：bit7=DETA100在线 / bit6=数据源=DETA100 / 低6位=EKF状态
"""
import sys
import csv
import argparse

try:
    import numpy as np
except ImportError:
    sys.exit("需要 numpy：pip install numpy")

# Windows 控制台默认可能是 GBK，重配 UTF-8 以正常输出中文 / emoji
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

RANGE_180 = 18000  # ±180° × 100，固件 yaw 约定上限


# ----------------------------- 解码 -----------------------------
def _sum_check(b):
    """SC：逐字节累加 & 0xFF（与固件 uint8 行为一致）。"""
    return int(np.sum(b, dtype=np.int64)) & 0xFF


def _add_check(b):
    """AC：累加和的累加 & 0xFF。cumsum 不逐步截断，但最终 & 0xFF 等价。"""
    return int(np.sum(np.cumsum(b.astype(np.int64)))) & 0xFF


def decode(buf):
    """逐帧解码整段缓冲。CRC 失败逐字节重新同步，而非盲跳整帧。"""
    data = np.frombuffer(buf, dtype=np.uint8)
    n = len(data)
    i = buf.find(b"\xab")
    if i < 0:
        i = n

    total = crc_ok = crc_fail = att_fail = 0
    func_tally = {}
    idx, roll_raw, pitch_raw, yaw_raw, fusion = [], [], [], [], []

    while i + 8 <= n:
        if data[i] != 0xAB:
            i += 1
            continue
        func = int(data[i + 3])
        length = int(data[i + 4]) | (int(data[i + 5]) << 8)
        if length > 256 or i + 8 + length > n:
            i += 1
            continue

        frame = data[i:i + 6 + length]
        ok = (_sum_check(frame) == data[i + 6 + length] and
              _add_check(frame) == data[i + 7 + length])
        if not ok:
            crc_fail += 1
            if func == 0x03:
                att_fail += 1
            i += 1            # 不信任 len，逐字节重新同步
            continue

        # —— CRC 通过，本帧可信 ——
        crc_ok += 1
        total += 1
        func_tally[func] = func_tally.get(func, 0) + 1
        if func == 0x03 and length >= 7:
            p = data[i + 6:i + 6 + length]
            r, pi, y = np.frombuffer(p[:6].tobytes(), dtype="<i2")  # 3×int16 小端
            idx.append(total)
            roll_raw.append(int(r))
            pitch_raw.append(int(pi))
            yaw_raw.append(int(y))
            fusion.append(int(p[6]))
        i += 8 + length       # 仅 CRC 通过才信任 len 跳进整帧

    atts = {
        "idx": np.array(idx, dtype=int),
        "roll_raw": np.array(roll_raw, dtype=int),
        "pitch_raw": np.array(pitch_raw, dtype=int),
        "yaw_raw": np.array(yaw_raw, dtype=int),
        "fusion": np.array(fusion, dtype=int),
    }
    stats = dict(total=total, crc_ok=crc_ok, crc_fail=crc_fail,
                 att_fail=att_fail, func_tally=func_tally)
    return stats, atts


def _unwrap_deg(yaw_deg):
    """对度制 yaw 解缠 ±180° wrap（转弧度借 np.unwrap，兼容所有 numpy 版本）。"""
    return np.degrees(np.unwrap(np.radians(yaw_deg)))


# ----------------------------- 报告 -----------------------------
def report(stats, atts, dt):
    print("\n" + "=" * 30)
    print("  AnoCom 数据统计报告 (Python)")
    print("=" * 30)
    print(f"有效帧: {stats['total']} (CRC通过 {stats['crc_ok']}, CRC失败 {stats['crc_fail']})")
    na = len(atts["idx"])
    extra = f"，另有 {stats['att_fail']} 帧 CRC 失败被丢弃" if stats["att_fail"] else ""
    print(f"姿态帧(0x03): {na}{extra}")
    ft = ", ".join(f"0x{k:02x}={v}" for k, v in sorted(stats["func_tally"].items()))
    print(f"功能码分布: {ft}")

    if na < 2:
        print("\n姿态帧不足，无法分析")
        return

    roll = atts["roll_raw"] / 100.0
    pitch = atts["pitch_raw"] / 100.0
    yaw = atts["yaw_raw"] / 100.0
    yaw_un = _unwrap_deg(yaw)

    # 2. 数据范围健康检查（针对当前固件 ±180° 约定）
    print("\n--- 数据范围健康检查 ---")
    any_out = False
    for name, raw in (("roll ", atts["roll_raw"]),
                      ("pitch", atts["pitch_raw"]),
                      ("yaw  ", atts["yaw_raw"])):
        out = int(np.sum(np.abs(raw) > RANGE_180))
        lo, hi = int(raw.min()), int(raw.max())
        any_out = any_out or out > 0
        flag = f"⚠️  {out} 帧超出 ±180°" if out else "✅"
        print(f"  {name} raw: [{lo}, {hi}]  ({lo/100:.2f}° ~ {hi/100:.2f}°)  {flag}")
    print("  ⚠️  存在超出 ±180° 的样本：固件约定应为 ±180°，请检查 AHRS 输出或 int16 回绕"
          if any_out else
          "  ✅ 全部落在 ±180° 预期范围内（int16 限幅 ±327.67° 远未触及）")

    # 3. 姿态稳定性
    print("\n--- 姿态稳定性 ---")

    def s(arr, name):
        print(f"  {name}: 均值={arr.mean():.3f}° 标准差={arr.std():.4f}° "
              f"范围=[{arr.min():.2f}, {arr.max():.2f}] 波动={arr.max()-arr.min():.3f}°")
    s(roll, "roll ")
    s(pitch, "pitch")
    s(yaw_un, "yaw  ")

    # 4. yaw 漂移率（最小二乘线性拟合 + R²）
    t = np.arange(na) * dt
    slope, intercept = np.polyfit(t, yaw_un, 1)
    fit = slope * t + intercept
    ss_res = float(np.sum((yaw_un - fit) ** 2))
    ss_tot = float(np.sum((yaw_un - yaw_un.mean()) ** 2))
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 1.0
    endpoint = float(yaw_un[-1] - yaw_un[0])
    print("\n--- yaw 漂移率 ---")
    print(f"  时长: {t[-1]:.1f}s ({na} 帧 × {dt*1000:.0f}ms 估算)")
    print(f"  线性拟合漂移率: {slope:.4f}°/s  (R²={r2:.3f})")
    print(f"  首尾差参考: {endpoint:.3f}° → {endpoint/t[-1]:.4f}°/s")
    if stats["att_fail"] > 0:
        print(f"  ⚠️  期间 {stats['att_fail']} 个姿态帧 CRC 失败被丢弃，真实时长可能更长，漂移率为偏大估计")
    a = abs(slope)
    print("  ✅ 漂移率极低 (无磁力计静止场景可接受)" if a < 0.05 else
          "  ⚠️  轻微漂移 (无磁力计正常范围)" if a < 0.5 else
          "  ❌ 显著漂移，零偏估计可能未收敛")

    # 5. yaw 突变检测（区分 ±180° wrap 与真实异常）
    print("\n--- yaw 突变检测 ---")
    d = np.diff(yaw)
    big = np.abs(d) > 30
    is_wrap = np.abs(np.abs(d) - 360) < 30
    anomaly = np.where(big & ~is_wrap)[0]
    wraps = int(np.sum(big & is_wrap))
    for j in anomaly:
        print(f"  帧 {j+1}: yaw 从 {yaw[j]:.2f}° 跳到 {yaw[j+1]:.2f}° (Δ={d[j]:.2f}°)")
    if wraps:
        print(f"  ℹ️  {wraps} 次 ±180° 边界 wrap（正常，已在漂移统计中解缠）")
    print("  ✅ 无异常突变" if len(anomaly) == 0 else
          f"  ⚠️  共 {len(anomaly)} 次异常突变（非 wrap），检查 AHRS 输出或丢帧")

    # 6. 融合状态解码（固件 communication.cpp 中 fusionSta 编码）
    print("\n--- 融合状态 (fusionSta 解码) ---")
    fus = atts["fusion"]
    print(f"  DETA100 在线   (bit7): {int(np.sum((fus & 0x80) != 0))}/{na} 帧")
    print(f"  数据源=DETA100 (bit6): {int(np.sum((fus & 0x40) != 0))}/{na} 帧")
    low = fus & 0x3F
    vals, counts = np.unique(low, return_counts=True)
    dist = ", ".join(f"{int(v)}×{int(c)}" for v, c in zip(vals, counts))
    print(f"  EKF 初始化/融合状态 (低6位) 分布: {dist}")

    # 7. 姿态趋势（首尾各 5 帧）
    print("\n--- 姿态趋势 (首5帧) ---")
    for k in range(min(5, na)):
        print(f"  t{atts['idx'][k]:>3}: roll={roll[k]:.2f} pitch={pitch[k]:.2f} "
              f"yaw={yaw[k]:.2f} (raw: {atts['yaw_raw'][k]})")
    print("--- 姿态趋势 (末5帧) ---")
    for k in range(max(0, na - 5), na):
        print(f"  t{atts['idx'][k]:>3}: roll={roll[k]:.2f} pitch={pitch[k]:.2f} "
              f"yaw={yaw[k]:.2f} (raw: {atts['yaw_raw'][k]})")


# ----------------------------- 导出 / 绘图 -----------------------------
def export_csv(atts, dt, path):
    na = len(atts["idx"])
    yaw = atts["yaw_raw"] / 100.0
    yaw_un = _unwrap_deg(yaw) if na >= 2 else yaw
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["idx", "t_s", "roll_deg", "pitch_deg", "yaw_deg",
                    "yaw_unwrap_deg", "roll_raw", "pitch_raw", "yaw_raw", "fusion"])
        for k in range(na):
            w.writerow([int(atts["idx"][k]), round(k * dt, 4),
                        atts["roll_raw"][k] / 100, atts["pitch_raw"][k] / 100,
                        atts["yaw_raw"][k] / 100, round(float(yaw_un[k]), 4),
                        int(atts["roll_raw"][k]), int(atts["pitch_raw"][k]),
                        int(atts["yaw_raw"][k]), int(atts["fusion"][k])])
    print(f"\n已导出 CSV: {path}  ({na} 行)")


def plot(atts, dt, save=None, show=False):
    na = len(atts["idx"])
    if na < 2:
        print("  姿态帧不足，跳过绘图")
        return
    try:
        import matplotlib
        if save and not show:
            matplotlib.use("Agg")     # 无 GUI 也能保存
        import matplotlib.pyplot as plt
    except ImportError:
        print("  未安装 matplotlib，跳过绘图：pip install matplotlib")
        return

    roll = atts["roll_raw"] / 100.0
    pitch = atts["pitch_raw"] / 100.0
    yaw = atts["yaw_raw"] / 100.0
    yaw_un = _unwrap_deg(yaw)
    t = np.arange(na) * dt
    slope, intercept = np.polyfit(t, yaw_un, 1)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    ax1.plot(t, roll, lw=1, label="roll")
    ax1.plot(t, pitch, lw=1, label="pitch")
    ax1.plot(t, yaw, lw=1, label="yaw")
    ax1.set_ylabel("angle (deg)")
    ax1.set_title("AnoCom attitude time series")
    ax1.legend(loc="upper right")
    ax1.grid(alpha=.3)

    ax2.plot(t, yaw_un, lw=1, color="tab:red", label="yaw (unwrapped)")
    ax2.plot(t, slope * t + intercept, "--", color="k", lw=1,
             label=f"fit {slope:.4f} deg/s")
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("yaw unwrapped (deg)")
    ax2.set_title("yaw drift trend")
    ax2.legend(loc="upper left")
    ax2.grid(alpha=.3)

    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=120)
        print(f"  图表已保存: {save}")
    if show:
        plt.show()


# ----------------------------- 入口 -----------------------------
def main():
    ap = argparse.ArgumentParser(description="AnoCom 姿态数据统计分析 (Python 版)")
    ap.add_argument("file", nargs="?", default="ano_raw.bin", help="二进制 dump 文件")
    ap.add_argument("--dt", type=float, default=0.02,
                    help="姿态帧间隔(秒)，默认 0.02 (50Hz)")
    ap.add_argument("--plot", "-p", action="store_true", help="弹出图表窗口")
    ap.add_argument("--save", metavar="PNG", help="保存图表到 PNG 文件")
    ap.add_argument("--csv", metavar="CSV", help="导出姿态数据到 CSV")
    args = ap.parse_args()

    try:
        with open(args.file, "rb") as f:
            buf = f.read()
    except OSError as e:
        sys.exit(f'无法读取文件 "{args.file}": {e}')
    print(f"读取 {len(buf)} 字节 ({args.file})")

    stats, atts = decode(buf)
    report(stats, atts, args.dt)

    if args.csv:
        export_csv(atts, args.dt, args.csv)
    if args.plot or args.save:
        plot(atts, args.dt, save=args.save, show=args.plot)


if __name__ == "__main__":
    main()
