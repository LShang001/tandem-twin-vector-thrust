"""把 GCS 的 server/ 与 web/ 同步进 APK 构建目录。

为什么需要这一步：Chaquopy 只打包 `src/main/python` 下的内容，而真正的代码
在 `TandemVec_FCS/GCS/` 下。与其维护两份副本，不如每次构建前同步一次——
副本不进 git（.gitignore 掉），源头永远只有一份。

用法（由 build.gradle 的 syncSources 任务自动调用）：
    py -3.12 android/gcs-app/sync-src.py
"""
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent            # .../android/gcs-app
GCS = HERE.parent.parent                          # .../GCS
DST = HERE / 'app' / 'src' / 'main' / 'python'

# server 是 Python 包；web 是前端静态资源（被 server.main 以相对路径引用）
SRC_ITEMS = ['server', 'web']

# 不必要也不应该进 APK 的东西（缓存/输出/测试数据）
IGNORE = shutil.ignore_patterns('__pycache__', '*.pyc', '.pytest_cache',
                                'test*', 'output', '*.log')


def main() -> int:
    if not (GCS / 'server' / 'main.py').exists():
        print(f'!! 找不到后端源码：{GCS / "server" / "main.py"}', file=sys.stderr)
        return 1
    DST.mkdir(parents=True, exist_ok=True)
    for name in SRC_ITEMS:
        src = GCS / name
        if not src.exists():
            print(f'!! 缺少 {name}', file=sys.stderr)
            return 1
        dst = DST / name
        if dst.exists():
            shutil.rmtree(dst)
        shutil.copytree(src, dst, ignore=IGNORE)
        n = sum(1 for _ in dst.rglob('*') if _.is_file())
        print(f'  同步 {name}: {n} 个文件')
    total = sum(1 for _ in DST.rglob('*') if _.is_file())
    print(f'同步完成，共 {total} 个文件 → {DST}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
