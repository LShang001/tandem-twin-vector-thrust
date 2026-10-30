#!/usr/bin/env python3
"""sync-params.py — 由 models/aircraft-model.json 生成仿真参数模块。

@file    sync-params.py
@module  analysis/工具（参数同步）
@brief   把唯一参数源 JSON 渲染为 simulations/vector-thrust-lab/src/core/parameters.mjs。
@details 输入：models/aircraft-model.json + aircraft-model.schema.json；
         处理：校验 → 按字段顺序渲染 ESM 常量与来源/置信度注释；
         输出：parameters.mjs（生成物，禁止手改）；--check 时只比对不写盘。
         用法：python tools/sync-params.py / python tools/sync-params.py --check
@theory  N/A（参数搬运，不含物理模型）
@frame  N/A
@units   沿用 JSON 内的单位字段（N·s²、kg·m²、rad/s 等）
@note    本文件是参数一致性的唯一通道：改参数必须改 JSON 再同步，
         手改 parameters.mjs 会被 --check 判为漂移（@ref AGENTS.md 边界）。
@see     models/aircraft-model.json、@ref AGENTS.md（参数同步命令）
"""
from __future__ import annotations

import json
import math
import re
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
    """把参数 JSON 渲染为 parameters.mjs 源码文本（含来源/单位/置信度注释）。

    @param[in] model aircraft-model.json 的解析结果
    @return 生成的 ESM 源码（写盘前可先与现有文件比对）
    """
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


FW_CONFIG = ROOT / "TandemVec_FCS" / "include" / "TandemVec_Config.h"
FW_DMAX_RE = re.compile(r"/\*\s*dMax\s*\*/\s*([0-9.]+)f")


def _warn_dmax_calibers(model: dict) -> None:
    """--check 附加提示：模型 dMax 与固件 dMax 两套口径并存时打印 warning。

    @note DEC-002 A-04（2026-10-23）决定不改模型 25°/固件 15°，两套口径显式分开；
          本函数只提示、不改变退出码，避免文档/分析混写两套口径。
    """
    if not FW_CONFIG.exists():
        return
    match = FW_DMAX_RE.search(FW_CONFIG.read_text(encoding="utf-8", errors="ignore"))
    model_dmax = next(
        (p["value"] for s in model["sections"] for p in s["parameters"] if p["name"] == "dMax"),
        None,
    )
    if match is None or model_dmax is None:
        return
    fw_deg = math.degrees(float(match.group(1)))
    model_deg = math.degrees(float(model_dmax))
    if abs(model_deg - fw_deg) > 0.5:
        print(
            f"[WARN] dMax 双口径并存：模型概念窗口 {model_deg:.1f}°（仿真域）"
            f" ≠ 固件实机包线 {fw_deg:.1f}°（TandemVec_Config.h），"
            "两套口径不可互相外推；见 DEC-002 A-04 与 ASM-017"
        )


def main() -> int:
    """CLI 入口：默认写盘生成 parameters.mjs；--check 只比对并返回退出码。"""
    model = json.loads(MODEL_JSON.read_text(encoding="utf-8"), parse_constant=_reject_constant)
    validate_model(model)
    content = render(model)
    if "--check" in sys.argv:
        if not TARGET.exists():
            print(f"[FAIL] 生成物不存在: {TARGET}", file=sys.stderr)
            return 1
        current = TARGET.read_text(encoding="utf-8")
        ok = current == content
        if ok:
            print(f"[OK] {TARGET.relative_to(ROOT)} 与 {MODEL_JSON.relative_to(ROOT)} 一致")
        else:
            print("[FAIL] 参数漂移：parameters.mjs 与 aircraft-model.json 不一致", file=sys.stderr)
            print("       运行 python tools/sync-params.py 重新生成", file=sys.stderr)
        _warn_dmax_calibers(model)
        return 0 if ok else 1
    TARGET.write_text(content, encoding="utf-8", newline="\n")
    n = sum(len(s["parameters"]) for s in model["sections"])
    print(f"[OK] 已生成 {TARGET.relative_to(ROOT)}（{n} 个参数，模型 {model['id']} v{model['version']}）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
