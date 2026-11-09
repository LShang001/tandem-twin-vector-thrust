#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    prefetch_tiles.py
@module  工具 / 地图瓦片预下载
@brief   按作业区域预下载地图瓦片到本地缓存，供野外离线使用
@details 为什么需要预下载：地图页的瓦片走后端代理，用过的区域会自动落盘，
         但第一次去某个场地仍然需要网络。飞行现场经常没网，所以要在出发前
         把作业区域按需要的缩放级别一次性抓下来。
         瓦片源与后端一致（CartoCDN 深色底图），坐标系是标准 Web Mercator /
         WGS-84，没有 GCJ-02 偏移。

         数量估算：一个区域每升一个级别瓦片数 ×4。
           z13 覆盖 5 km 见方 ≈ 几十张
           z16 覆盖 2 km 见方 ≈ 一两百张
           z18 覆盖 500 m 见方 ≈ 一两百张
         脚本会先打印计划张数与预计耗时，加 --yes 才真正开始下。

@theory  N/A（Web Mercator 标准瓦片编号换算）
@frame   WGS-84 经纬度 → Web Mercator 瓦片坐标 z/x/y
@units   经纬度 deg、瓦片张数无量纲
@note    按 OSM 使用条款，批量下载要控制速率并带明确 UA；本脚本默认 0.05 s/张。
@test    T1 --dry-run 能给出正确的瓦片张数与覆盖范围；
         T2 实跑后目标目录生成的 png 文件数量与计划一致
@see     TandemVec_FCS/GCS/server/main.py 的 /tiles/{z}/{x}/{y}.png
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TILE_DIR = ROOT / 'output' / 'tiles'
# 瓦片源必须与后端 /tiles 接口一致（server/main.py 的 GCS_TILE_SOURCE）。
# 2026-11-09 实测：OSM 官源 / OpenTopoMap / openstreetmap.fr / bzh 在国内全部
# 超时或 404，wikimedia 403，CartoCDN 返回 "API KEY REQUIRED" 占位图——
# 境外免费源在国内基本不可用。默认走天地图（需 key），也可给自定义模板。
TIANDITU_TMPL = ('https://t{s}.tianditu.gov.cn/vec_w/wmts?SERVICE=WMTS'
                 '&REQUEST=GetTile&VERSION=1.0.0&LAYER=vec&STYLE=default'
                 '&TILEMATRIXSET=w&FORMAT=tiles'
                 '&TILEMATRIX={z}&TILEROW={y}&TILECOL={x}&tk={key}')

UA = 'UniGNC-GCS-prefetch/0.3 (+local flight test tool)'


def build_url(source: str, key: str, z: int, x: int, y: int) -> str:
    """按源配置生成瓦片 URL（与后端 _tile_url 保持同一套规则）"""
    if source == 'tianditu':
        return (TIANDITU_TMPL
                .replace('{s}', str((x + y) % 8))
                .replace('{z}', str(z)).replace('{x}', str(x)).replace('{y}', str(y))
                .replace('{key}', key))
    return (source.replace('{z}', str(z))
            .replace('{x}', str(x)).replace('{y}', str(y))
            .replace('{key}', key))


def deg2tile(lat: float, lon: float, z: int) -> tuple[int, int]:
    """经纬度 → 该缩放级别下的瓦片 x/y（Web Mercator）"""
    lat_rad = math.radians(lat)
    n = 2.0 ** z
    x = int((lon + 180.0) / 360.0 * n)
    y = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    x = max(0, min(int(n) - 1, x))
    y = max(0, min(int(n) - 1, y))
    return x, y


def tile_range(lat0, lon0, lat1, lon1, z):
    """给出一块矩形区域在该级别覆盖的瓦片坐标范围"""
    x0, y0 = deg2tile(lat0, lon0, z)
    x1, y1 = deg2tile(lat1, lon1, z)
    return min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)


def main() -> int:
    """命令行入口。"""
    ap = argparse.ArgumentParser(description='按区域预下载地图瓦片')
    ap.add_argument('--center', required=True,
                    help='中心点 "纬度,经度"，如 28.2300,112.9400')
    ap.add_argument('--radius-km', type=float, default=2.0,
                    help='覆盖半径（km），默认 2')
    ap.add_argument('--zoom', type=int, nargs='+', default=[14, 16, 18],
                    help='缩放级别，可给多个，默认 14 16 18')
    ap.add_argument('--rate', type=float, default=0.05,
                    help='每张间隔秒数，默认 0.05（约 20 张/s）')
    ap.add_argument('--yes', action='store_true', help='确认执行（不加只打印计划）')
    ap.add_argument('--source', default=os.environ.get('GCS_TILE_SOURCE', 'tianditu'),
                    help='瓦片源：tianditu（默认，需 key）或含 {z}/{x}/{y} 的 URL 模板')
    ap.add_argument('--key', default=os.environ.get('GCS_TILE_KEY', ''),
                    help='瓦片源密钥（天地图必需；也可用 GCS_TILE_KEY 环境变量）')
    args = ap.parse_args()

    try:
        lat_c, lon_c = [float(v) for v in args.center.split(',')]
    except Exception:
        print('[FAIL] --center 格式应为 "纬度,经度"', file=sys.stderr)
        return 1

    # 半径 → 经纬度跨度（小范围平面近似，预下载够用）
    dlat = args.radius_km / 111.0
    dlon = args.radius_km / (111.0 * max(0.2, math.cos(math.radians(lat_c))))

    lat0, lat1 = lat_c - dlat, lat_c + dlat
    lon0, lon1 = lon_c - dlon, lon_c + dlon

    plan = []
    total = 0
    print(f'中心 {lat_c:.6f},{lon_c:.6f}  半径 {args.radius_km} km')
    for z in sorted(set(args.zoom)):
        x0, y0, x1, y1 = tile_range(lat0, lon0, lat1, lon1, z)
        n = (x1 - x0 + 1) * (y1 - y0 + 1)
        total += n
        plan.append((z, x0, y0, x1, y1, n))
        print(f'  z{z}: x {x0}..{x1}  y {y0}..{y1}  → {n} 张')

    est = total * max(0.0, args.rate)
    print(f'\n合计 {total} 张，预计 {est:.0f} s（{est/60:.1f} min）')
    print(f'缓存目录：{TILE_DIR}')

    if not args.yes:
        print('\n这是计划预览。确认后加 --yes 执行。')
        return 0

    if total == 0:
        print('[FAIL] 计划为空', file=sys.stderr)
        return 1
    if total > 20000:
        print(f'[FAIL] {total} 张过多（>20000），请缩小半径或减少级别', file=sys.stderr)
        return 1

    if args.source == 'tianditu' and not args.key:
        print('[FAIL] 天地图需要 key：--key <key> 或设 GCS_TILE_KEY 环境变量\n'
              '      免费申请：https://console.tianditu.gov.cn/', file=sys.stderr)
        return 1

    ok = 0
    skip = 0
    fail = 0
    t0 = time.time()
    for z, x0, y0, x1, y1, n in plan:
        for x in range(x0, x1 + 1):
            for y in range(y0, y1 + 1):
                dest = TILE_DIR / str(z) / str(x) / f'{y}.png'
                if dest.is_file() and dest.stat().st_size > 0:
                    skip += 1
                    continue
                try:
                    req = urllib.request.Request(
                        build_url(args.source, args.key, z, x, y),
                        headers={'User-Agent': UA})
                    with urllib.request.urlopen(req, timeout=10) as resp:
                        data = resp.read()
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    dest.write_bytes(data)
                    ok += 1
                except Exception as exc:
                    fail += 1
                    if fail <= 5:
                        print(f'  [warn] {z}/{x}/{y}: {exc}')
                time.sleep(max(0.0, args.rate))
        print(f'  z{z} 完成：新增 {ok}，跳过 {skip}，失败 {fail}')

    print(f'\n完成：新增 {ok}，跳过（已缓存）{skip}，失败 {fail}，'
          f'耗时 {time.time()-t0:.0f} s')
    return 0 if fail == 0 else 2


if __name__ == '__main__':
    sys.exit(main())
