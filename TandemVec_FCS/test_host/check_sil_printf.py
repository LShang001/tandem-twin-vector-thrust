#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SIL printf 格式串/实参数量一致性检查。
@file    check_sil_printf.py
@brief   扫描 fw_sil_env_main.cpp 里的 printf/fprintf 调用，比对格式符与实参个数。
@module  TandemVec_FCS/测试设施（宿主 SIL 静态门禁）
@details 背景：给 SIL_REAL_JSON 增加字段时漏了实参，导致其后所有字段右移一位、
         末位 %s 读越界输出 (null) 使 JSON 不可解析（2026-10-27 实测）。
         g++ 默认不对 std::printf 报 -Wformat 参数缺失，故用本脚本收口。
@frame   N/A
@units   N/A

用法：py -3.12 test_host/check_sil_printf.py [--strict]
退出码非 0 即存在数量不匹配的 printf。
"""
import argparse
import os
import re
import sys

TARGET = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      "fw_sil", "fw_sil_env_main.cpp")
SPEC_RE = re.compile(
    r"%(?:[-+ #0]*[\d.]*)(?:hh|h|ll|l|z|j|t|L)?[diuoxXeEfgGaAcspFDDUU]")
STR_RE = re.compile(r'"(?:[^"\\]|\\.)*"')


def find_calls(src, names):
    """返回 (函数名, 起始括号位置) 列表。"""
    out = []
    for m in re.finditer(r"\b(?:std::)?(" + "|".join(names) + r")\s*\(", src):
        out.append((m.group(1), m.end() - 1))
    return out


def scan_call(src, open_idx):
    """从参数左括号起做引号/转义感知的深度扫描，返回调用体文本。"""
    depth = 0
    i = open_idx
    instr = False
    esc = False
    while i < len(src):
        ch = src[i]
        if esc:
            esc = False
        elif ch == "\\":
            esc = True
        elif ch == '"':
            instr = not instr
        elif not instr:
            if ch == "(":
                depth += 1
            elif ch == ")":
                depth -= 1
                if depth == 0:
                    return src[open_idx:i + 1]
        i += 1
    return ""


def split_top(args_text):
    parts, buf, depth, instr, esc = [], "", 0, False, False
    for ch in args_text:
        if esc:
            buf += ch
            esc = False
            continue
        if ch == "\\":
            buf += ch
            esc = True
            continue
        if ch == '"':
            instr = not instr
            buf += ch
            continue
        if not instr:
            if ch in "([{":
                depth += 1
            elif ch in ")]}":
                depth -= 1
            elif ch == "," and depth == 0:
                parts.append(buf)
                buf = ""
                continue
        buf += ch
    if buf.strip():
        parts.append(buf)
    return [p for p in parts if p.strip()]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", default=TARGET)
    ap.add_argument("--strict", action="store_true",
                    help="把无法静态判定的调用也计为问题")
    args = ap.parse_args()

    src = open(args.file, encoding="utf-8").read()
    bad = 0
    checked = 0
    for name, pos in find_calls(src, ("printf", "fprintf", "snprintf")):
        call = scan_call(src, pos)
        if not call:
            continue
        inner = call[1:-1]
        parts = split_top(inner)
        fmt_idx = 1 if name == "fprintf" else 0
        if name == "snprintf":
            fmt_idx = 2
        if len(parts) <= fmt_idx:
            continue
        fmt_lit = parts[fmt_idx].strip()
        if not fmt_lit.startswith('"'):
            continue  # 格式串是变量，静态不可判定
        fmt = "".join(x[1:-1] for x in re.findall(r'"(?:[^"\\]|\\.)*"', fmt_lit))
        if "%" not in fmt:
            continue
        specs = SPEC_RE.findall(fmt)
        arg_parts = parts[fmt_idx + 1:]
        checked += 1
        if len(arg_parts) != len(specs):
            bad += 1
            ln = src.count("\n", 0, pos) + 1
            print(f"[FAIL] {os.path.basename(args.file)}:{ln} {name}: "
                  f"格式符 {len(specs)} 个，实参 {len(arg_parts)} 个")
            for i, p in enumerate(arg_parts):
                if i >= len(specs):
                    print(f"        多余实参 #{i}: {p.strip()[:70]}")
            for i, s in enumerate(specs):
                if i >= len(arg_parts):
                    print(f"        缺实参的格式符 #{i}: {s}")
    print(f"检查 printf 调用 {checked} 处，不匹配 {bad} 处")
    if bad:
        return 1
    print("OK: SIL printf 格式符与实参数量一致")
    return 0


if __name__ == "__main__":
    sys.exit(main())
