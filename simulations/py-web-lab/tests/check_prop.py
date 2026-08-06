# 数值验证：螺旋桨旋转（__pwl 钩子读 prop.rotation 两帧对比）
import asyncio
import json

from playwright.async_api import async_playwright

EXE = r'C:\Users\12631\AppData\Local\ms-playwright\chromium_headless_shell-1228\chrome-headless-shell-win64\chrome-headless-shell.exe'


async def main():
    async with async_playwright() as p:
        b = await p.chromium.launch(executable_path=EXE, args=['--use-angle=swiftshader'])
        pg = await b.new_page(viewport={'width': 1440, 'height': 900})
        await pg.goto('http://127.0.0.1:8090/', wait_until='networkidle')
        await pg.wait_for_selector('#loader.done', timeout=15000)
        await pg.wait_for_timeout(2000)

        async def props():
            return await pg.evaluate('window.__pwl.getProps()')

        async def wf():
            return await pg.evaluate('window.__pwl.getState().wf')

        p0 = await props()
        await pg.wait_for_timeout(500)
        p1 = await props()
        await pg.wait_for_timeout(500)
        p2 = await props()
        print(f'前桨 rotation.x: {p0["fr"]:.4f} → {p1["fr"]:.4f} → {p2["fr"]:.4f}')
        print(f'尾桨 rotation.x: {p0["tr"]:.4f} → {p1["tr"]:.4f} → {p2["tr"]:.4f}')
        d1 = (p1['fr'] - p0['fr']) % (2 * 3.14159)
        d2 = (p2['fr'] - p1['fr']) % (2 * 3.14159)
        print(f'前桨 0.5s 增量: {d1:.4f} rad / {d2:.4f} rad （>0.3 即明显在转）')
        print(f'wf = {await wf():.0f} rad/s')
        # 暂停后应停转
        await pg.click('#b-pause')
        await pg.wait_for_timeout(600)
        p3 = await props()
        await pg.wait_for_timeout(500)
        p4 = await props()
        print(f'暂停后前桨: {p3["fr"]:.4f} → {p4["fr"]:.4f} （应几乎不变）')
        await b.close()


asyncio.run(main())
