#!/usr/bin/env python3
"""代码注释与理论对照规范检查器（GOV-004 的机检实现）。

@file    check-code-comments.py
@module  工具/质量门禁
@brief   静态扫描自研代码，按 GOV-004 §9 规则输出注释质量报告并作为门禁。
@details 输入：git 跟踪的自研代码文件；处理：解析文件头/文档块/@ref/TODO/
         注释块；输出：人读摘要 + 可选 JSON 报告，--check 时有 error 返回 1。
@theory  N/A（工程规则实现）
@frame   N/A
@units   N/A

用法（仓库根目录）：
    py -3.12 tools/check-code-comments.py                 # 扫描全量，打印摘要
    py -3.12 tools/check-code-comments.py --check         # 门禁：有 error 即退出码 1
    py -3.12 tools/check-code-comments.py --area fw-nav   # 只查一个区域
    py -3.12 tools/check-code-comments.py --paths a.cpp b.py
    py -3.12 tools/check-code-comments.py --changed       # 只查 git 工作区改动文件
    py -3.12 tools/check-code-comments.py --json report.json

规则（与 GOV-004 §9 一一对应）：
    R01 error   缺 L0 文件头，或缺少 @file / @brief / @module
    R02 error   @ref 目标格式非法
    R03 error   TODO/FIXME 缺少 (owner, YYYY-MM-DD)
    R04 warning 连续 ≥3 行注释掉的代码
    R05 warning 注释行超过 120 字符
    R06 warning 纯英文注释块（≥6 个英文词、无中文）
    R07 error   理论关键文件没有任何 REF-/MOD-/CTL-/THY- 引用
    R08 warning 公共符号缺 L1 文档块（JS/Python 启发式）
    R09 warning 文件头缺 @details / @frame / @units
    R10 warning 模糊引用（“见书/见文档/参考教材”无编号）

说明：
- 只检查 git 跟踪的自研代码（范围与 GOV-004 §2 一致），第三方库/生成物自动跳过；
- 文件头含 @generated 的文件整文件跳过；
- 规则命中按“文件内去重”计数，评分 = 100 − 20×error − 5×warning。
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

EXTS = {".mjs", ".js", ".py", ".cpp", ".h", ".hpp", ".cc"}

# 纳入检查的路径前缀（GOV-004 §2.1）
INCLUDE_PREFIXES = (
    "simulations/vector-thrust-lab/",
    "simulations/high-fidelity-analysis/",
    "simulations/py-web-lab/",
    "TandemVec-EKF-Report/sim/",
    "TandemVec_FCS/src/",
    "TandemVec_FCS/include/",
    "TandemVec_FCS/lib/navigation-main/src/",
    "TandemVec_FCS/test_host/",
    "TandemVec_FCS/GCS/",
    "TandemVec_FCS/tools/",
    "TandemVec_FCS/analyze_ano.py",
    "TandemVec_FCS/analyze_ano.js",
    "scripts/",
    "tools/",
    "blender-test/",
)

# 区域划分（供 --area 使用），按前缀归属；一个文件可命中多个区域
AREAS = {
    "sim": ("simulations/vector-thrust-lab/",),
    "fw-ekf": ("TandemVec_FCS/lib/navigation-main/src/",),
    "fw-nav": (
        "TandemVec_FCS/src/navigation_task.",
        "TandemVec_FCS/src/sensor_imu.",
        "TandemVec_FCS/src/math_utils.h",
        "TandemVec_FCS/include/ins_",
        "TandemVec_FCS/include/QuaternionMath.h",
        "TandemVec_FCS/include/Quat4f.h",
        "TandemVec_FCS/include/VerticalKF",
        "TandemVec_FCS/include/HorizontalKF.h",
        "TandemVec_FCS/include/ComplementaryFilter.h",
        "TandemVec_FCS/include/IMU_AutoCalibrator.h",
        "TandemVec_FCS/include/GeoDisplacement.h",
    ),
    "fw-control": (
        "TandemVec_FCS/src/flight_control.",
        "TandemVec_FCS/include/FlightCtrlParams.h",
        "TandemVec_FCS/include/ControlUnits.h",
        "TandemVec_FCS/include/IncrementalPID.h",
        "TandemVec_FCS/include/PositionPID.h",
        "TandemVec_FCS/include/RcCurve.h",
        "TandemVec_FCS/include/InertiaDecoupling.h",
        "TandemVec_FCS/include/AirframeModel.h",
        "TandemVec_FCS/include/TandemVec_",
        "TandemVec_FCS/include/TVC_Control_",
        "TandemVec_FCS/include/ActuatorFaultMonitor.h",
    ),
    "fw-infra": (
        "TandemVec_FCS/src/main.",
        "TandemVec_FCS/src/communication.",
        "TandemVec_FCS/src/can_bus.",
        "TandemVec_FCS/src/mavlink_bridge.",
        "TandemVec_FCS/src/ubx_config.",
        "TandemVec_FCS/src/task_scheduler",
        "TandemVec_FCS/src/ano_params.",
        "TandemVec_FCS/src/ano_vars.",
        "TandemVec_FCS/src/sensor_peripheral.",
        "TandemVec_FCS/src/state_data.",
        "TandemVec_FCS/src/deta100_types.h",
        "TandemVec_FCS/src/icm45686_",
        "TandemVec_FCS/include/AnoVars.h",
        "TandemVec_FCS/include/StatusLED.h",
        "TandemVec_FCS/include/Ws2812AppStatus.h",
        "TandemVec_FCS/include/MedianFilter.h",
        "TandemVec_FCS/include/DETA100_module.h",
        "TandemVec_FCS/analyze_ano.",
    ),
    "fw-tests": ("TandemVec_FCS/test_host/",),
    "gcs": ("TandemVec_FCS/GCS/", "TandemVec_FCS/tools/"),
    "analysis": (
        "simulations/high-fidelity-analysis/",
        "simulations/py-web-lab/",
        "TandemVec-EKF-Report/sim/",
        "scripts/",
        "blender-test/",
        "tools/",
    ),
}

# 理论关键文件：必须出现 REF-/MOD-/CTL-/THY- 引用（GOV-004 §9 R07）
THEORY_GLOBS = (
    "simulations/vector-thrust-lab/src/core/",
    "simulations/high-fidelity-analysis/core.py",
    "simulations/high-fidelity-analysis/controllers.py",
    "simulations/high-fidelity-analysis/estimators.py",
    "TandemVec-EKF-Report/sim/ekf_core.py",
    "TandemVec-EKF-Report/sim/hifi/hifi_core.py",
    "TandemVec-EKF-Report/sim/opt/opt_ekf.py",
    "TandemVec_FCS/src/flight_control.cpp",
    "TandemVec_FCS/src/navigation_task.cpp",
    "TandemVec_FCS/src/sensor_imu.cpp",
    "TandemVec_FCS/src/math_utils.h",
    "TandemVec_FCS/include/AirframeModel.h",
    "TandemVec_FCS/include/QuaternionMath.h",
    "TandemVec_FCS/include/Quat4f.h",
    "TandemVec_FCS/include/PositionPID.h",
    "TandemVec_FCS/include/VerticalKF.h",
    "TandemVec_FCS/include/VerticalKF_2State.h",
    "TandemVec_FCS/include/HorizontalKF.h",
    "TandemVec_FCS/include/InertiaDecoupling.h",
    "TandemVec_FCS/include/TVC_Control_",
    "TandemVec_FCS/include/TandemVec_ADRC.h",
    "TandemVec_FCS/include/TandemVec_ControlAllocation.h",
    "TandemVec_FCS/include/TandemVec_OnlineID.h",
    "TandemVec_FCS/include/TandemVec_Propulsion.h",
    "TandemVec_FCS/include/RcCurve.h",
    "TandemVec_FCS/lib/navigation-main/src/ekf_15_state.h",
    "TandemVec_FCS/lib/navigation-main/src/transforms.h",
    "TandemVec_FCS/lib/navigation-main/src/earth_model.h",
    "TandemVec_FCS/lib/navigation-main/src/tilt_compass.h",
    "TandemVec_FCS/lib/navigation-main/src/navigation.h",
)

DOC_REF_TOKEN = re.compile(
    r"@ref\s+("
    r"(?:REF|MOD|CTL|THY|GOV|REQ|VER|COMM|PROTOCOL|GCS|SAFE|CFG|PROP|CONOPS)-\d{3}"
    r"|PAPER"
    r"|TandemVec-Paper/[\w./-]+"
    r"|AGENTS\.md"
    r"|docs/[\w./-]+"
    r"|[\w./-]+\.(?:md|json|h|hpp|cpp|py|mjs|js|tex)"
    r")"
)
REF_ANY = re.compile(r"@ref\s+(\S+)")
CJK = re.compile(r"[\u4e00-\u9fff]")
EN_WORD = re.compile(r"[A-Za-z]{2,}")
TODO = re.compile(r"\b(TODO|FIXME|XXX)\b")
TODO_OK = re.compile(r"\b(?:TODO|FIXME|XXX)\s*\(\s*[\w.@-]+\s*,\s*\d{4}-\d{2}-\d{2}\s*\)")
VAGUE_REF = re.compile(r"见\s*(?:书|文献|文档|教材|论文)|参考\s*(?:书|教材|文档)")
GENERATED = re.compile(r"@generated\b")

COMMENT_LINE = {
    ".py": ("#",),
    ".mjs": ("//", "/*", "*"),
    ".js": ("//", "/*", "*"),
    ".cpp": ("//", "/*", "*"),
    ".h": ("//", "/*", "*"),
    ".hpp": ("//", "/*", "*"),
    ".cc": ("//", "/*", "*"),
}

CODE_HINT = re.compile(
    r"(?:[;{}]\s*$)"
    r"|^\s*(?:if|for|while|switch|return|break|continue|const|let|var|def|class|struct|"
    r"void|int|float|double|bool|auto|static|public|private|std::|#include)\b"
)


@dataclass
class Finding:
    rule: str
    level: str
    line: int
    message: str


@dataclass
class FileResult:
    path: str
    score: int = 100
    errors: int = 0
    warnings: int = 0
    findings: list[Finding] = field(default_factory=list)


def git_files() -> list[str]:
    out = subprocess.check_output(
        ["git", "-C", str(ROOT), "ls-files", "-z"], stderr=subprocess.DEVNULL
    )
    return [p for p in out.decode("utf-8", "replace").split("\0") if p]


def changed_files() -> set[str]:
    try:
        out = subprocess.check_output(
            ["git", "-C", str(ROOT), "diff", "--name-only", "HEAD"],
            stderr=subprocess.DEVNULL,
        )
        names = set(out.decode("utf-8", "replace").splitlines())
        out2 = subprocess.check_output(
            ["git", "-C", str(ROOT), "ls-files", "--others", "--exclude-standard", "-z"],
            stderr=subprocess.DEVNULL,
        )
        names.update(p for p in out2.decode("utf-8", "replace").split("\0") if p)
        return names
    except subprocess.CalledProcessError:
        return set()


def excluded(rel: str) -> bool:
    parts = rel.split("/")
    if any(p in {"vendor", "node_modules", "__pycache__", ".git"} for p in parts):
        return True
    # 隐藏工具目录（.dsh-ops/.reasonix 等）不属于工程代码
    if any(p.startswith(".") for p in parts[:-1]) or parts[0].startswith("."):
        return True
    if "TandemVec_FCS/lib/" in rel and "navigation-main/src/" not in rel:
        return True
    if rel.endswith("parameters.mjs") or rel.endswith("standalone.html"):
        return True
    if rel.startswith("TandemVec_FCS/test_host/bin/"):
        return True
    return False


def in_scope(rel: str) -> bool:
    if Path(rel).suffix.lower() not in EXTS:
        return False
    if excluded(rel):
        return False
    return any(rel.startswith(p) for p in INCLUDE_PREFIXES)


def in_area(rel: str, area: str) -> bool:
    return any(rel.startswith(p) for p in AREAS[area])


def is_theory(rel: str) -> bool:
    return any(rel.startswith(g) for g in THEORY_GLOBS)


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        return path.read_text(encoding="utf-8", errors="replace")


def first_code_index(lines: list[str], ext: str) -> int:
    """返回第一条非空、非 shebang、非 coding 声明的行号（0 基）。"""
    for i, raw in enumerate(lines):
        s = raw.strip()
        if not s:
            continue
        if i == 0 and s.startswith("#!"):
            continue
        if re.match(r"#.*coding[:=]", s):
            continue
        return i
    return 0


def extract_header(lines: list[str], ext: str) -> tuple[bool, str]:
    """判断文件顶部是否有注释头，并返回其文本。"""
    start = first_code_index(lines, ext)
    if start >= len(lines):
        return False, ""
    first = lines[start].lstrip()
    if ext == ".py":
        m = re.match(r'([ru]?)("""|\'\'\')', first)
        if not m:
            return False, ""
        quote = m.group(2)
        chunks = [first]
        # 单行 docstring
        if chunks[0].count(quote) >= 2:
            return True, chunks[0]
        for raw in lines[start + 1 : start + 80]:
            chunks.append(raw)
            if quote in raw:
                break
        return True, "\n".join(chunks)
    if first.startswith("/*"):
        chunks = [first]
        if "*/" in first:
            return True, first
        for raw in lines[start + 1 : start + 80]:
            chunks.append(raw)
            if "*/" in raw:
                break
        return True, "\n".join(chunks)
    return False, ""


def header_tags(header: str) -> set[str]:
    return set(re.findall(r"@(\w+)", header))


def comment_run(lines: list[str], ext: str) -> list[tuple[int, int, list[str]]]:
    """返回连续注释块 (start, end, lines)（0 基，含端点）。"""
    prefixes = COMMENT_LINE.get(ext, ("//",))
    runs: list[tuple[int, int, list[str]]] = []
    cur: list[int] = []
    for i, raw in enumerate(lines):
        s = raw.strip()
        is_comment = bool(s) and any(s.startswith(p) for p in prefixes)
        if is_comment:
            cur.append(i)
        else:
            if len(cur) >= 2:
                runs.append((cur[0], cur[-1], [lines[j] for j in cur]))
            cur = []
    if len(cur) >= 2:
        runs.append((cur[0], cur[-1], [lines[j] for j in cur]))
    return runs


def has_doc_above(lines: list[str], idx: int, lookback: int = 6) -> bool:
    for j in range(max(0, idx - lookback), idx):
        s = lines[j].strip()
        if s.startswith("/**") or s.startswith('"""') or s.startswith("'''"):
            return True
        if s.startswith("///") or s.startswith("//!"):
            return True
        if s.startswith("//") or s.startswith("*"):
            continue
        if not s:
            continue
        return False
    return False


def check_file(rel: str, text: str) -> FileResult:
    ext = Path(rel).suffix.lower()
    lines = text.splitlines()
    findings: list[Finding] = []

    if GENERATED.search("\n".join(lines[:40])):
        return FileResult(path=rel)

    ok, header = extract_header(lines, ext)
    if not ok:
        findings.append(Finding("R01", "error", 1, "缺少文件顶部注释头（L0）"))
    else:
        tags = header_tags(header)
        missing = [t for t in ("file", "brief", "module") if t not in tags]
        if missing:
            findings.append(
                Finding("R01", "error", 1, "文件头缺少必填字段 " + " / ".join("@" + t for t in missing))
            )
        optional = [t for t in ("details", "frame", "units") if t not in tags]
        if optional:
            findings.append(
                Finding("R09", "warning", 1, "文件头缺少字段 " + " / ".join("@" + t for t in optional))
            )

    # R02 @ref 目标格式
    for i, raw in enumerate(lines, 1):
        if "@ref" not in raw:
            continue
        for m in REF_ANY.finditer(raw):
            token = m.group(1).rstrip("）)，,;。")
            if not DOC_REF_TOKEN.match("@ref " + token):
                # 允许纯代码标识符引用（如 @ref eulerFromQuat）
                if re.fullmatch(r"[A-Za-z_][\w.:()<>-]*", token):
                    continue
                findings.append(Finding("R02", "error", i, f"@ref 目标格式非法：{token}"))

    # R03 TODO 规范
    for i, raw in enumerate(lines, 1):
        if TODO.search(raw) and not TODO_OK.search(raw):
            findings.append(
                Finding("R03", "error", i, "TODO/FIXME 缺少 (owner, YYYY-MM-DD)")
            )

    # R04 注释掉的代码
    for start, end, block in comment_run(lines, ext):
        code_like = 0
        for raw in block:
            body = re.sub(r"^\s*(//+|#+|\*+|/\*+)+", "", raw).strip()
            if not body or body.startswith("@"):
                continue
            if CODE_HINT.search(body):
                code_like += 1
        if end - start + 1 >= 3 and code_like >= 2:
            findings.append(
                Finding("R04", "warning", start + 1, f"疑似注释掉的代码（{end - start + 1} 行）")
            )

    # R05 注释行宽
    prefixes = COMMENT_LINE.get(ext, ("//",))
    for i, raw in enumerate(lines, 1):
        s = raw.strip()
        if s and any(s.startswith(p) for p in prefixes) and len(raw) > 120:
            findings.append(Finding("R05", "warning", i, f"注释行长度 {len(raw)} > 120"))

    # R06 纯英文注释块
    for start, _end, block in comment_run(lines, ext):
        text_block = "\n".join(block)
        if CJK.search(text_block):
            continue
        words = EN_WORD.findall(text_block)
        if len(words) >= 6 and not re.search(r"@|http|SPDX|coding", text_block):
            findings.append(Finding("R06", "warning", start + 1, "纯英文注释块（应使用中文）"))

    # R07 理论文件引用
    if is_theory(rel) and not re.search(r"@ref\s+(REF|MOD|CTL|THY)-\d{3}", text):
        findings.append(Finding("R07", "error", 1, "理论关键文件缺少 REF-/MOD-/CTL-/THY- 引用"))

    # R08 公共符号文档块（JS/Python 启发式）；测试文件豁免（测试名即语义）
    is_test_file = (
        "/tests/" in rel
        or Path(rel).name.startswith("test_")
        or rel.endswith(".test.mjs")
        or rel.startswith("TandemVec_FCS/test_host/")
    )
    if not is_test_file:
        if ext in {".mjs", ".js"}:
            for i, raw in enumerate(lines):
                if re.match(r"export\s+(async\s+)?function\s+\w+", raw) or re.match(
                    r"export\s+(?:const|let)\s+\w+\s*=\s*(?:async\s*)?\(", raw
                ):
                    if not has_doc_above(lines, i):
                        findings.append(
                            Finding("R08", "warning", i + 1, "导出函数缺少 L1 文档块")
                        )
        elif ext == ".py":
            for i, raw in enumerate(lines):
                m = re.match(r"(?:async\s+)?def\s+(\w+)|class\s+(\w+)", raw)
                if not m:
                    continue
                name = m.group(1) or m.group(2)
                if name.startswith("_"):
                    continue
                # docstring 检查：本行或后续 1–2 行出现三引号
                window = "\n".join(lines[i : i + 3])
                if '"""' not in window and "'''" not in window:
                    findings.append(
                        Finding("R08", "warning", i + 1, f"公共符号 {name} 缺少 docstring")
                    )

    # R10 模糊引用
    for i, raw in enumerate(lines, 1):
        if VAGUE_REF.search(raw) and not re.search(r"(§|@ref|\d{3}|p\.\d+)", raw):
            findings.append(Finding("R10", "warning", i, "模糊引用：缺少章节/式号/页码"))

    for f in findings:
        if f.level == "error":
            f.message = f"[{f.rule}] {f.message}"

    res = FileResult(path=rel)
    # 同一规则同一行只保留一条，避免重复噪声
    seen: set[tuple[str, int]] = set()
    for f in findings:
        key = (f.rule, f.line)
        if key in seen:
            continue
        seen.add(key)
        res.findings.append(f)
    res.errors = sum(1 for f in res.findings if f.level == "error")
    res.warnings = sum(1 for f in res.findings if f.level == "warning")
    res.score = max(0, 100 - 20 * res.errors - 5 * res.warnings)
    return res


def score_file(rel: str) -> FileResult:
    return check_file(rel, read_text(ROOT / rel))


def select_files(args) -> list[str]:
    if args.paths:
        files = []
        for p in args.paths:
            rel = Path(p).as_posix().lstrip("./")
            if (ROOT / rel).is_file() and in_scope(rel):
                files.append(rel)
        return sorted(set(files))
    candidates = set(git_files()) | changed_files()
    files = [p for p in candidates if in_scope(p)]
    if args.area:
        files = [p for p in files if in_area(p, args.area)]
    if args.changed:
        changed = changed_files()
        files = [p for p in files if p in changed]
    return sorted(files)


def main() -> int:
    ap = argparse.ArgumentParser(description="GOV-004 代码注释规范检查")
    ap.add_argument("--area", choices=sorted(AREAS), help="只检查某个区域")
    ap.add_argument("--paths", nargs="+", help="只检查指定文件（相对仓库根）")
    ap.add_argument("--changed", action="store_true", help="只检查 git 工作区改动文件")
    ap.add_argument("--check", action="store_true", help="有 error 时返回退出码 1")
    ap.add_argument("--json", metavar="PATH", help="输出 JSON 报告")
    ap.add_argument("--max-examples", type=int, default=12, help="每种规则最多打印的样例数")
    ap.add_argument("--score", action="store_true", help="按分数从低到高排序输出")
    args = ap.parse_args()

    files = select_files(args)
    if not files:
        print("未选中任何文件（检查 --area/--paths/--changed 与 GOV-004 §2 范围）。")
        return 0

    results = [score_file(p) for p in files]
    total_err = sum(r.errors for r in results)
    total_warn = sum(r.warnings for r in results)
    file_err = sum(1 for r in results if r.errors)
    file_warn = sum(1 for r in results if r.warnings and not r.errors)
    clean = sum(1 for r in results if not r.errors and not r.warnings)
    avg = sum(r.score for r in results) / len(results)

    by_rule: dict[str, list[tuple[str, Finding]]] = {}
    for r in results:
        for f in r.findings:
            by_rule.setdefault(f.rule, []).append((r.path, f))
    by_rule_count = {k: len(v) for k, v in sorted(by_rule.items())}

    print("=" * 78)
    print("GOV-004 代码注释与理论对照规范检查")
    print(f"范围: {len(results)} 文件 | 平均分: {avg:.1f} | error: {total_err} | warning: {total_warn}")
    print(f"文件: 有 error {file_err} | 仅 warning {file_warn} | 全绿 {clean}")
    print("规则命中: " + ("  ".join(f"{k}={v}" for k, v in by_rule_count.items()) or "无"))
    print("=" * 78)

    for rule in sorted(by_rule):
        items = by_rule[rule]
        print(f"\n[{rule}] 命中 {len(items)} 处（显示前 {min(len(items), args.max_examples)} 处）")
        for path, f in items[: args.max_examples]:
            print(f"  {path}:{f.line}  {f.message}")

    order = sorted(results, key=lambda r: (r.score, r.path))
    print("\n" + "-" * 78)
    print("低分文件（<= 80 分）:")
    shown = 0
    for r in order:
        if r.score > 80:
            break
        print(f"  {r.score:3d}  E{r.errors:<2d} W{r.warnings:<2d}  {r.path}")
        shown += 1
    if shown == 0:
        print("  无")

    if args.json:
        payload = {
            "generated_at": _dt.datetime.now().astimezone().isoformat(timespec="seconds"),
            "scope": len(results),
            "summary": {
                "errors": total_err,
                "warnings": total_warn,
                "files_with_errors": file_err,
                "files_clean": clean,
                "avg_score": round(avg, 2),
                "by_rule": by_rule_count,
            },
            "files": [
                {
                    "path": r.path,
                    "score": r.score,
                    "errors": r.errors,
                    "warnings": r.warnings,
                    "findings": [
                        {"rule": f.rule, "level": f.level, "line": f.line, "message": f.message}
                        for f in r.findings
                    ],
                }
                for r in order
            ],
        }
        Path(args.json).write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        print(f"\nJSON 报告已写入: {args.json}")

    if args.check and total_err:
        print(f"\n门禁未通过：{total_err} 个 error。")
        return 1
    print("\n检查完成。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
