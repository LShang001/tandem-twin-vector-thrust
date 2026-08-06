# 验证：回中开关（从 thumb 按下拖拽，模拟真实用户）
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
        await pg.click('#b-vtol')
        await pg.wait_for_timeout(2000)

        async def drag(offset_px):
            """从当前 thumb 按下，拖 offset_px 后松开"""
            box = await pg.locator('#s-dw').bounding_box()
            v = float(await pg.input_value('#s-dw'))
            cy = box['y'] + box['height'] / 2
            thumb_x = box['x'] + (v + 80) / 160 * box['width']
            await pg.mouse.move(thumb_x, cy)
            await pg.mouse.down()
            await pg.mouse.move(thumb_x + offset_px, cy, steps=6)
            await pg.mouse.up()

        # [关] 回中关闭：松手停留
        await pg.click('#b-spring')
        print('[开关]', await pg.text_content('#b-spring'))
        await drag(100)
        await pg.wait_for_timeout(1500)
        v = await pg.input_value('#s-dw')
        print(f'[关] 松手 1.5s dw={v} （应停留非 0）')

        # [开] 回中开启：松手回 0
        await pg.click('#b-spring')
        await drag(60)
        await pg.wait_for_timeout(1500)
        v = await pg.input_value('#s-dw')
        print(f'[开] 松手 1.5s dw={v} （应回 0）')
        assert v == '0', '回中未生效'

        # 悬停 dt 滑块也回中
        await drag(80)
        await pg.wait_for_timeout(1500)
        v = await pg.input_value('#s-dt')
        print(f'[dt] 松手 1.5s dt={v} （应回 0）')

        print('[console]', ' | '.join(errs[:8]) if errs else '无错误/警告')
        await b.close()


asyncio.run(main())
