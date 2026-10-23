#!/usr/bin/env python3
"""@file gen_mag_reference.py
@brief 生成固件磁参考场站点常量（一次性离线工具）
@module 固件/导航（磁参考场生成；不属于固件运行时）
@details 输入：站点纬度/经度/椭球高/日期（命令行）；处理：调用 ppigrf 计算
         IGRF 总场，按 NED（D 向下为正）换算并输出 C++ 初始化常量；
         输出：stdout 文本（供手工粘贴到 include/mag_reference.h）。
         用法（需要 ppigrf；可装到临时目录后用 PYTHONPATH 指向）：
         py -3.12 -m pip install --target D:/tmp/tv_ppigrf ppigrf
         PYTHONPATH=D:/tmp/tv_ppigrf py -3.12 tools/gen_mag_reference.py \
             --lat 28.2 --lon 112.9 --alt-km 0.05 --date 2026-01-01
@theory  IGRF-14 球谐模型；N/E/D、H=hypot(N,E)、F=norm(N,E,D)、
         I=atan2(D,H)、dec=atan2(E,N) @ref REF-120 §9.9.1 p.323–324
@frame   NED（x 北/y 东/z 下）；D 向下为正
@units   经纬度 deg、高度 km、磁场 µT、角度 rad/deg
@note    ★ 离线工具，固件运行时不依赖 ppigrf/pandas；换站点/日期必须重新生成，
         并检查 D>0、dec 符号后再写入 `include/mag_reference.h`。
@see     TandemVec_FCS/include/mag_reference.h、src/navigation_task.cpp
"""

from __future__ import annotations

import argparse
import math

import pandas as pd
import ppigrf


def main() -> int:
    """解析站点参数并打印 NED 参考场 C++ 初始化常量。"""
    ap = argparse.ArgumentParser()
    ap.add_argument("--lat", type=float, default=28.2, help="geodetic latitude [deg]")
    ap.add_argument("--lon", type=float, default=112.9, help="longitude [deg]")
    ap.add_argument("--alt-km", type=float, default=0.05, help="ellipsoid height [km]")
    ap.add_argument("--date", default="2026-01-01", help="IGRF epoch date")
    args = ap.parse_args()

    date = pd.Timestamp(args.date)
    be, bn, bu = ppigrf.igrf(args.lon, args.lat, args.alt_km, date)
    e_ut, n_ut, up_ut = float(be[0]) / 1000.0, float(bn[0]) / 1000.0, float(bu[0]) / 1000.0
    d_ut = -up_ut  # NED 向下为正
    h_ut = math.hypot(n_ut, e_ut)
    f_ut = math.hypot(h_ut, d_ut)
    dec_rad = math.atan2(e_ut, n_ut)
    dip_rad = math.atan2(d_ut, h_ut)

    print(f"site: lat={args.lat} lon={args.lon} alt_km={args.alt_km} date={date.date()}")
    print(f"N/E/D = {n_ut:.3f} / {e_ut:.3f} / {d_ut:.3f} uT")
    print(f"H/F   = {h_ut:.3f} / {f_ut:.3f} uT")
    print(f"dip   = {math.degrees(dip_rad):.3f} deg = {dip_rad:.6f} rad")
    print(f"dec   = {math.degrees(dec_rad):.3f} deg = {dec_rad:.6f} rad")
    print()
    print("C++ initializer:")
    print("static constexpr MagReferenceNed kDefaultMagReference = {")
    print(f"    {{{n_ut:.3f}f, {e_ut:.3f}f, {d_ut:.3f}f}},")
    print(f"    {h_ut:.3f}f, {f_ut:.3f}f,")
    print(f"    {dip_rad:.6f}f, {dec_rad:.6f}f,")
    print(f"    {args.lat:.1f}f, {args.lon:.1f}f, true}};")
    print()
    print("校验：D 必须为正（北半球磁力线下指）；dec<0 表示磁北在真北以西。")
    return 0 if d_ut > 0.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
