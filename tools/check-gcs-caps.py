"""
@file    check-gcs-caps.py
@module  工具/守卫/GCS 能力清单一致性
@brief   校验 server/caps.py 声明的能力与 server/main.py 的实际实现是否一致
@details 这个清单不是文档——它是调用方（人/脚本/agent）发现能力的唯一入口，
         一旦与实现漂移，就会出现"清单说有、调了没有"或"实现了、没人知道"。
         两种都会让自动化失效，所以必须进守卫。

判据（任一不满足即 FAIL）：
  ① 清单里每条命令在 main.py 都有分派分支（写了没实现）
  ② main.py 每个分派分支在清单里都登记过（实现了没登记）
  ③ 清单里每条参数都真的被该分支读取（声明了但没用）
"""
from __future__ import annotations

import sys
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'TandemVec_FCS' / 'GCS'))

from server import caps  # noqa: E402


def main() -> int:
    d = caps.as_dict()
    print(f'能力清单：{d["n_modules"]} 个模块 / {d["n_cmds"]} 条命令')
    per_module = '、'.join(f'{m["id"]}({len(m["cmds"])})' for m in d['modules'])
    print(f'  模块分布：{per_module}')

    problems = caps.verify_against_source()
    if problems:
        print(f'\n[FAIL] 清单与实现不一致（{len(problems)} 处）：')
        for p in problems:
            print(f'  - {p}')
        print('\n修法：清单与 main.py 分派必须同进同出——'
              '加命令时先登记 caps.py 再实现分派，或删掉不再使用的声明。')
        return 1

    # 附加检查：模块 id 唯一、命令名全局唯一（重复会让 /api/cmd 分派歧义）
    ids = [m['id'] for m in d['modules']]
    if len(ids) != len(set(ids)):
        print(f'\n[FAIL] 模块 id 重复：{ids}')
        return 1
    seen, dup = set(), []
    for m in d['modules']:
        for c in m['cmds']:
            if c['cmd'] in seen:
                dup.append(c['cmd'])
            seen.add(c['cmd'])
    if dup:
        print(f'\n[FAIL] 命令名重复登记：{dup}')
        return 1

    print('[PASS] GCS 能力清单与实现一致（命令/参数/模块归属均对得上）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
