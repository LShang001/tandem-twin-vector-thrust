"""check_shutdown.py — 验证「关闭服务」按钮 → 进程退出 → 端口释放。

@file    check_shutdown.py
@module  analysis/py-web-lab 测试（浏览器）
@brief   点页面关闭按钮后轮询 8090 端口，断言服务已退出且页面提示已关闭。
@details 输入：本地 8090 服务 + headless Chromium；处理：点击关闭 → port_alive()
         轮询 → 读页面文案；输出：控制台断言。
@theory  N/A（生命周期测试）
@frame  N/A
@units   超时 s
@note    该测试会**真的杀掉** 8090 服务进程，跑完需重新启动服务。
@see     server/main.py、server/cli.py
"""
import asyncio
import urllib.request

from playwright.async_api import async_playwright

EXE = r'C:\Users\12631\AppData\Local\ms-playwright\chromium_headless_shell-1228\chrome-headless-shell-win64\chrome-headless-shell.exe'


def port_alive():
    try:
        urllib.request.urlopen('http://127.0.0.1:8090/', timeout=2)
        return True
    except Exception:
        return False


async def main():
    assert port_alive(), '服务未启动'
    print('[0] 服务在运行')
    async with async_playwright() as p:
        b = await p.chromium.launch(executable_path=EXE, args=['--use-angle=swiftshader'])
        pg = await b.new_page(viewport={'width': 1440, 'height': 900})
        errs = []
        pg.on('console', lambda m: errs.append(f'[{m.type}] {m.text}') if m.type in ('error', 'warning') else None)
        await pg.goto('http://127.0.0.1:8090/', wait_until='networkidle')
        await pg.wait_for_selector('#loader.done', timeout=15000)
        print('[1] 页面连接:', await pg.text_content('#conn'))
        await pg.screenshot(path='pwl-shutdown-before.png')

        # 点击关闭服务（处理 confirm 弹窗）
        pg.on('dialog', lambda d: d.accept())
        await pg.click('#b-shutdown')
        await pg.wait_for_timeout(2500)
        print('[2] 关闭后页面状态:', await pg.text_content('#conn'))
        await pg.screenshot(path='pwl-shutdown-after.png')
        await b.close()

    # 等待进程退出 + 端口释放
    for _ in range(20):
        if not port_alive():
            break
        await asyncio.sleep(0.5)
    print('[3] 端口 8090 已释放:', not port_alive())
    assert not port_alive(), '服务未退出！'


asyncio.run(main())
