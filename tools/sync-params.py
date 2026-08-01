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
SCHEMA_JSON = ROOT / "models" / "aircraft-model.schema.json"
TARGET = ROOT / "simulations" / "vector-thrust-lab" / "src" / "core" / "parameters.mjs"

HEADER = """// ============================================================
//  物理参数（小型 UAV 量级）
//  单一事实源：仓库根目录 models/aircraft-model.json
//  本文件由 tools/sync-params.py 生成 —— 请勿手工编辑
// ============================================================
"""


def validate_model(model: dict) -> None:
    """对 aircraft-model.json 做轻量结构校验（约束对齐 aircraft-model.schema.json，
    不引入外部 jsonschema 依赖）。违反约束即抛 ValueError。"""
    if not isinstance(model, dict):
        raise ValueError("模型根节点必须是 JSON 对象")
    for key in ("id", "name", "version", "status", "sections"):
        if key not in model:
            raise ValueError(f"缺少顶层必填字段: {key}")

    schema = json.loads(SCHEMA_JSON.read_text(encoding="utf-8"))
    source_enum = schema["properties"]["status"]["enum"]
    param_props = schema["properties"]["sections"]["items"]["properties"]["parameters"]["items"]["properties"]
    p_source_enum = param_props["source"]["enum"]
    p_conf_enum = param_props["confidence"]["enum"]

    if model["status"] not in source_enum:
        raise ValueError(f"status 非法: {model['status']!r}（允许: {source_enum}）")

    seen: set[str] = set()
    total = 0
    for section in model["sections"]:
        if not isinstance(section, dict) or "title" not in section or "parameters" not in section:
            raise ValueError("section 必须包含 title 与 parameters")
        for p in section["parameters"]:
            total += 1
            for key in ("name", "value", "unit", "description", "source", "confidence"):
                if key not in p:
                    raise ValueError(f"参数缺少必填字段 {key}: {p.get('name', '<unnamed>')}")
            name = p["name"]
            if not name.isidentifier():
                raise ValueError(f"非法参数名: {name}")
            if name in seen:
                raise ValueError(f"参数名重复: {name}（schema additionalProperties 要求全局唯一）")
            seen.add(name)
            if not isinstance(p["value"], (int, float)) or isinstance(p["value"], bool):
                raise ValueError(f"参数 {name} 的 value 必须是数字")
            if p["source"] not in p_source_enum:
                raise ValueError(f"参数 {name} 的 source 非法: {p['source']!r}（允许: {p_source_enum}）")
            if p["confidence"] not in p_conf_enum:
                raise ValueError(f"参数 {name} 的 confidence 非法: {p['confidence']!r}（允许: {p_conf_enum}）")


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


def _reject_constant(name: str) -> None:
    raise ValueError(f"非法 JSON 常量: {name}（NaN/Infinity 禁止进入参数源）")


def main() -> int:
    model = json.loads(MODEL_JSON.read_text(encoding="utf-8"), parse_constant=_reject_constant)
    validate_model(model)
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
