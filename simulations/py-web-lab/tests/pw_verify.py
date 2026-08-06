# py-web-lab 浏览器实测：连接 → 巡航 → 悬停 → 定高 → 截图
import asyncio
from pathlib import Path

from playwright.async_api import async_playwright

OUT = Path(__file__).resolve().parent
URL = 'http://127.0.0.1:8090/'


async def main():
    async with async_playwright() as p:
        b = await p.chromium.launch(
            executable_path=r'C:\Users\12631\AppData\Local\ms-playwright\chromium_headless_shell-1228\chrome-headless-shell-win64\chrome-headless-shell.exe',
            args=['--use-angle=swiftshader'])
        pg = await b.new_page(viewport={'width': 1440, 'height': 900})
        errs = []
        pg.on('console', lambda m: errs.append(f'[{m.type}] {m.text}') if m.type in ('error', 'warning') else None)
        pg.on('pageerror', lambda e: errs.append(f'[pageerror] {e}'))
        await pg.goto(URL, wait_until='networkidle')
        # 等待 loader 消失（WS init 完成）
        await pg.wait_for_selector('#loader.done', timeout=15000)
        print('[1] 连接成功（loader done）')
        print('    conn:', await pg.text_content('#conn'))
        await pg.wait_for_timeout(3000)
        print('    巡航遥测: thr=', await pg.text_content('#v-thr'),
              ' att=', await pg.text_content('#d-att'))
        await pg.screenshot(path=str(OUT / 'pwl-01-cruise.png'))

        # 悬停模式
        await pg.click('#b-vtol')
        await pg.wait_for_timeout(3000)
        print('[2] 悬停模式: btn=', await pg.text_content('#b-vtol'),
              ' sas=', await pg.text_content('#b-sas'),
              ' aero=', await pg.text_content('#b-aero'))
        print('    att=', await pg.text_content('#d-att'),
              ' thr=', await pg.text_content('#v-thr'),
              ' h=', await pg.text_content('#d-h'))
        await pg.screenshot(path=str(OUT / 'pwl-02-hover.png'))

        # 定高 5m
        await pg.click('#b-alt')
        await pg.wait_for_timeout(4000)
        print('[3] 定高开: btn=', await pg.text_content('#b-alt'),
              ' h=', await pg.text_content('#d-h'),
              ' altRef slider=', await pg.input_value('#s-alt'))
        await pg.screenshot(path=str(OUT / 'pwl-03-alt.png'))

        # 倾斜指令 dt=12
        await pg.fill('#s-dt', '12')
        await pg.dispatch_event('#s-dt', 'input')
        await pg.wait_for_timeout(3000)
        print('[4] 倾斜 12°: att=', await pg.text_content('#d-att'),
              ' h=', await pg.text_content('#d-h'))
        await pg.screenshot(path=str(OUT / 'pwl-04-tilt.png'))

        # 航向角速度 dw=40°/s
        await pg.fill('#s-dw', '40')
        await pg.dispatch_event('#s-dw', 'input')
        await pg.wait_for_timeout(2500)
        print('[5] 航向 40°/s: att=', await pg.text_content('#d-att'),
              ' sas=', await pg.text_content('#b-sas'))
        await pg.screenshot(path=str(OUT / 'pwl-05-yawrate.png'))

        # 暂停/继续
        await pg.click('#b-pause')
        await pg.wait_for_timeout(1200)
        h1 = await pg.text_content('#d-h')
        att1 = await pg.text_content('#d-att')
        await pg.wait_for_timeout(1200)
        h2 = await pg.text_content('#d-h')
        att2 = await pg.text_content('#d-att')
        print(f'[P] 暂停: btn=', await pg.text_content('#b-pause'),
              f' h={h1}→{h2} att={att1}→{att2}')
        await pg.click('#b-pause')
        await pg.wait_for_timeout(1500)
        print(f'[P] 继续: btn=', await pg.text_content('#b-pause'),
              ' h=', await pg.text_content('#d-h'))
        await pg.screenshot(path=str(OUT / 'pwl-05b-pause.png'))

        # 退出悬停 → 巡航
        await pg.click('#b-vtol')
        await pg.wait_for_timeout(2500)
        print('[6] 退出悬停: btn=', await pg.text_content('#b-vtol'),
              ' att=', await pg.text_content('#d-att'),
              ' thr=', await pg.text_content('#v-thr'))
        await pg.screenshot(path=str(OUT / 'pwl-06-back-cruise.png'))

        print('[console]', ' | '.join(errs[:8]) if errs else '无错误/警告')
        await b.close()


asyncio.run(main())
