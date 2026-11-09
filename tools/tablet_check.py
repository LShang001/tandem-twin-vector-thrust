#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    tablet_check.py
@module  工具/GCS 平板响应式验收
@brief   用 Playwright 以真实平板视口打开 GCS，检查横向溢出并截图

@details 为什么必须用真实视口：CSS 的 @media 断点只有浏览器真的按那个宽度布局
         才会触发。把桌面截图缩小（screenshot 的 maxWidth）只是缩放像素，
         媒体查询根本不生效——那样"验证"出来的结论是假的。

         检查项：scrollWidth > clientWidth 表示有横向溢出，平板上会左右拖动，
         这是响应式最常见的硬伤。
@test    本脚本即自检：无溢出 + 截图生成
@see     TandemVec_FCS/GCS/web/css/style.css（@media 断点）
"""
from __future__ import annotations

import sys
from pathlib import Path

from playwright.sync_api import sync_playwright

OUT = Path('tmp')
URL = 'http://127.0.0.1:8091'

SIZES = [
    ('ipad_1024x768', 1024, 768),
    ('android_1280x800', 1280, 800),
]


def main() -> int:
    OUT.mkdir(exist_ok=True)
    bad = 0
    with sync_playwright() as p:
        b = p.chromium.launch()
        for name, w, h in SIZES:
            pg = b.new_page(viewport={'width': w, 'height': h})
            pg.goto(URL, wait_until='networkidle')
            pg.wait_for_timeout(3500)
            m = pg.evaluate(
                "() => ({sw: document.documentElement.scrollWidth,"
                " cw: document.documentElement.clientWidth,"
                " navOverflow: (() => { const n = document.querySelector('.nav');"
                "   return n ? n.scrollWidth > n.clientWidth + 1 : null; })()})")
            over = m['sw'] > m['cw']
            print(f"[{name}] viewport {w}x{h}  scrollW={m['sw']} clientW={m['cw']}  "
                  f"横向溢出={over}  导航溢出={m['navOverflow']}")
            pg.screenshot(path=str(OUT / f'tablet_{name}.png'))
            pg.close()
            if over:
                bad += 1
        b.close()
    print(f'截图完成，溢出 {bad}/{len(SIZES)}')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
