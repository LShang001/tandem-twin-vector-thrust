#!/usr/bin/env python3
"""sync-params.py — 由 models/aircraft-model.json 生成仿真参数模块。

用法:
    python tools/sync-params.py            重新生成 parameters.mjs
    python tools/sync-params.py --check    校验生成物与 JSON 一致（漂移检测）

生成目标: simulations/vector-thrust-lab/src/core/parameters.mjs（禁止手改）
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MODEL_JSON = ROOT / "models" / "aircraft-model.json"
TARGET = ROOT / "simulations" / "vector-thrust-lab" / "src" / "core" / "parameters.mjs"

HEADER = """// ============================================================
//  物理参数（小型 UAV 量级）
//  单一事实源：仓库根目录 models/aircraft-model.json
//  本文件由 tools/sync-params.py 生成 —— 请勿手工编辑
// ============================================================
"""


def js_number(v: float) -> str:
    """输出与 JS 双精度一致的数值字面量（整数值不带小数点）。"""
    if isinstance(v, int) or (isinstance(v, float) and v.is_integer() and abs(v) < 1e15):
        return str(int(v))
    return repr(float(v))


def render(model: dict) -> str:
    lines = [HEADER, "export const P = Object.freeze({\n"]
    for section in model["sections"]:
        lines.append(f"  // ---- {section['title']} ----\n")
        for p in section["parameters"]:
            name = p["name"]
            if not name.isidentifier():
                raise ValueError(f"非法参数名: {name}")
            comment = f"{p['description']} [{p['unit']}]"
            lines.append(f"  {name}: {js_number(p['value'])},  // {comment}\n")
        lines.append("\n")
    lines.append("});\n")
    return "".join(lines)


def main() -> int:
    model = json.loads(MODEL_JSON.read_text(encoding="utf-8"))
    content = render(model)
    if "--check" in sys.argv:
        if not TARGET.exists():
            print(f"[FAIL] 生成物不存在: {TARGET}", file=sys.stderr)
            return 1
        current = TARGET.read_text(encoding="utf-8")
        if current == content:
            print(f"[OK] {TARGET.relative_to(ROOT)} 与 {MODEL_JSON.relative_to(ROOT)} 一致")
            return 0
        print("[FAIL] 参数漂移：parameters.mjs 与 aircraft-model.json 不一致", file=sys.stderr)
        print("       运行 python tools/sync-params.py 重新生成", file=sys.stderr)
        return 1
    TARGET.write_text(content, encoding="utf-8", newline="\n")
    n = sum(len(s["parameters"]) for s in model["sections"])
    print(f"[OK] 已生成 {TARGET.relative_to(ROOT)}（{n} 个参数，模型 {model['id']} v{model['version']}）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
