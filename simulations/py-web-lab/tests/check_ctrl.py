"""check_ctrl.py — 浏览器实测：控制律切换按钮（sas→indi→lqr→adrc）。

@file    check_ctrl.py
@module  analysis/py-web-lab 测试（浏览器）
@brief   用 Playwright 点选控制律按钮并截图，验证前端切换链路。
@details 输入：本地 8090 服务 + headless Chromium；处理：打开页面 → 依次点击模式按钮
         → 读回 S.ctrl；输出：控制台断言 + 截图（人工核对）。
@theory  N/A（前端交互链路测试）
@frame  N/A
@units   N/A
@note    依赖本机 Playwright 浏览器绝对路径 EXE；服务未启动时脚本会直接失败。
@see     server/controllers.py、tests/pw_verify.py
"""
import asyncio

from playwright.async_api import async_playwright

EXE = r'C:\Users\12631\AppData\Local\ms-playwright\chromium_headless_shell-1228\chrome-headless-shell-win64\chrome-headless-shell.exe'


async def main():
    async with async_playwright() as p:
        b = await p.chromium.launch(executable_path=EXE, args=['--use-angle=swiftshader'])
        pg = await b.new_page(viewport={'width': 1440, 'height': 900})
        errs = []
        pg.on('console', lambda m: errs.append(f'[{m.type}] {m.text}') if m.type in ('error', 'warning') else None)
        await pg.goto('http://127.0.0.1:8090/', wait_until='networkidle')
        await pg.wait_for_selector('#loader.done', timeout=15000)
        await pg.wait_for_timeout(1500)

        # 巡航：sas → indi
        await pg.click('#b-ctrl')
        await pg.wait_for_timeout(1000)
        print('[巡航] 控制律 =', await pg.text_content('#b-ctrl'))
        await pg.wait_for_timeout(2500)
        print('   INDI 巡航 2.5s: att =', await pg.text_content('#d-att'))
        await pg.screenshot(path='pwl-ctrl-indi.png')

        # 悬停：sas → lqr → adrc
        await pg.click('#b-vtol')
        await pg.wait_for_timeout(2000)
        await pg.click('#b-ctrl')
        await pg.wait_for_timeout(1000)
        print('[悬停] 控制律 =', await pg.text_content('#b-ctrl'), ' B_true 可见 =',
              await pg.locator('#b-btrue').is_visible())
        await pg.wait_for_timeout(2500)
        print('   LQR 悬停 2.5s: att =', await pg.text_content('#d-att'))
        await pg.screenshot(path='pwl-ctrl-lqr.png')
        await pg.click('#b-ctrl')
        await pg.wait_for_timeout(1000)
        print('[悬停] 控制律 =', await pg.text_content('#b-ctrl'))
        await pg.wait_for_timeout(2500)
        print('   ADRC 悬停 2.5s: att =', await pg.text_content('#d-att'), ' h =', await pg.text_content('#d-h'))
        await pg.screenshot(path='pwl-ctrl-adrc.png')

        # 切回 sas
        await pg.click('#b-ctrl')
        await pg.wait_for_timeout(1000)
        print('[悬停] 控制律 =', await pg.text_content('#b-ctrl'))
        print('[console]', ' | '.join(errs[:8]) if errs else '无错误/警告')
        await b.close()


asyncio.run(main())
