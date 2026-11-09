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

# 人机工程判据：触控目标最小 44×44 CSS px（iOS HIG）／48×48 dp（Material）。
# 取 40 作判据下限——低于它误触率明显上升，而 40–43 在平板上还能接受。
MIN_TOUCH = 40

# 关键交互元素：漏了哪一个都可能在平板上点不准
TOUCH_SELECTORS = [
    ('.nav-btn', '顶部导航'),
    ('.btn', '通用按钮'),
    ('#connectBtn', '连接按钮'),
    ('#vehSelect', '机号选择'),
    ('#scopeWrap canvas', '曲线画布'),
    ('#specRun', '频谱按钮'),
]


def main() -> int:
    OUT.mkdir(exist_ok=True)
    bad = 0
    with sync_playwright() as p:
        b = p.chromium.launch()
        for name, w, h in SIZES:
            # has_touch=True 让浏览器按「粗指针」设备处理，
            # 否则 @media (pointer: coarse) 的触控放大部分根本不生效，
            # 测出来的仍是桌面尺寸（那样验证等于没验证）。
            pg = b.new_page(viewport={'width': w, 'height': h}, has_touch=True,
                            is_mobile=False)

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
            # 触控尺寸：逐选择器量第一个可见元素的渲染尺寸
            small = pg.evaluate(
                '''(sel) => {
                    const out = [];
                    for (const [css, label] of sel) {
                      const el = document.querySelector(css);
                      if (!el) continue;
                      const r = el.getBoundingClientRect();
                      if (r.width < 1 && r.height < 1) continue;
                      out.push({label, css, w: Math.round(r.width), h: Math.round(r.height)});
                    }
                    return out;
                }''',
                [[c, l] for c, l in TOUCH_SELECTORS])
            under = [x for x in small if x['w'] < MIN_TOUCH or x['h'] < MIN_TOUCH]
            print(f'  触控尺寸（判据 {MIN_TOUCH}px）：')
            for x in small:
                mark = '  <-- 偏小' if x in under else ''
                print(f"    {x['label']:<10} {x['w']:>4}x{x['h']:<4}{mark}")
            if under:
                bad += 1
            pg.screenshot(path=str(OUT / f'tablet_{name}.png'))
            pg.close()
            if over:
                bad += 1
        b.close()
    print(f'截图完成，溢出 {bad}/{len(SIZES)}')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
