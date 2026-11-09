"""把 GCS 的 server/ 与 web/ 同步进 APK 构建目录，附带 pymavlink 精简副本。

为什么需要这一步：Chaquopy 只打包 `src/main/python` 下的内容，而真正的代码
在 `TandemVec_FCS/GCS/` 下。与其维护两份副本，不如每次构建前同步一次——
副本不进 git（.gitignore 掉），源头永远只有一份。

为什么 pymavlink 不走 pip：它的 setup.py 把 fastcrc / lxml 写进 install_requires，
两者都是 C 扩展，而 Chaquopy 禁止编译原生代码，pip 必然失败。
但 pymavlink 源码里 fastcrc 是**可选**的（`mavgen_python.py`：
"falling back to a pure Python implementation if fastcrc is not available"），
lxml 同理。所以把纯 Python 部分直接复制进来即可，CRC 自动走纯 Python 实现。

用法（由 build.gradle 的 syncSources 任务自动调用）：
    py -3.12 android/gcs-app/sync-src.py
"""
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent            # .../android/gcs-app
GCS = HERE.parent.parent                          # .../GCS
DST = HERE / 'app' / 'src' / 'main' / 'python'

SRC_ITEMS = ['server', 'web']

IGNORE = shutil.ignore_patterns('__pycache__', '*.pyc', '.pytest_cache',
                                'test*', 'output', '*.log')

# pymavlink 只取运行时必需的纯 Python 文件：
#   顶层 16 个 .py（mavutil/mavextra/quaternion/...）
#   generator/mavcrc.py —— mavutil 直接 `from pymavlink.generator.mavcrc import x25crc`
#   dialects/v20/common.py —— GCS 实际用的 dialect（1.6 MB，其余 45 MB 用不到）
PYMAVLINK_KEEP = [
    '*.py',
    'generator/mavcrc.py',
    'generator/__init__.py',
    'dialects/__init__.py',
    'dialects/v20/__init__.py',
    'dialects/v20/common.py',
]


def sync_pymavlink() -> int:
    """@brief 复制 pymavlink 的纯 Python 子集
    @return 复制的文件数
    """
    try:
        import pymavlink
    except ImportError:
        print('!! 本机没有 pymavlink，APK 里将没有 MAVLink 支持', file=sys.stderr)
        return 0
    src_root = Path(pymavlink.__file__).parent
    dst_root = DST / 'pymavlink'
    if dst_root.exists():
        shutil.rmtree(dst_root)
    n = 0
    for pat in PYMAVLINK_KEEP:
        for f in src_root.glob(pat):
            if not f.is_file():
                continue
            rel = f.relative_to(src_root)
            out = dst_root / rel
            out.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(f, out)
            n += 1
    size = sum(f.stat().st_size for f in dst_root.rglob('*') if f.is_file())
    print(f'  同步 pymavlink: {n} 个文件（{size/1024/1024:.1f} MB，已排除其余 dialects）')
    return n


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
    sync_pymavlink()
    total = sum(1 for _ in DST.rglob('*') if _.is_file())
    print(f'同步完成，共 {total} 个文件 → {DST}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
