#!/usr/bin/env python3
"""build-standalone.py — 单文件版构建：把 ESM 工程内联成 standalone.html。

@file    build-standalone.py
@module  analysis/工具（构建）
@brief   收集模块依赖闭包，转换为 tiny-AMD 形式并与 CSS 一起内联成单个 HTML。
@details 用法（仓库根目录）：
           py -3.12 tools/build-standalone.py          生成 standalone.html
           py -3.12 tools/build-standalone.py --check  漂移检测（重建并字节比对）
         原理：file:// 下浏览器禁止加载 ES 模块（CORS），故把全部模块按依赖闭包
         转成 tiny-AMD（__def/__req），消除全部文件请求。
         仅支持本项目实际使用的语法子集：命名导入、命名空间导入（import * as）、
         export function/const/let/class、export { ... }（含 as 别名、多行）。
         遇到不支持的形式（default 导入/导出、re-export、动态 import）直接报错。
@theory  N/A（构建工具，不含物理模型）
@frame  N/A
@units   N/A（输出字节数）
@note    standalone.html 是构建产物，禁止手改；改源码后必须重跑本工具
         （@ref AGENTS.md 边界）。--check 供 CI 做产物漂移门禁。
@see     simulations/vector-thrust-lab/standalone.html、@ref AGENTS.md
"""
from __future__ import annotations

import argparse
import re
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SIM = ROOT / "simulations" / "vector-thrust-lab"
VENDOR = SIM / "vendor" / "three-r170"
OUT = SIM / "standalone.html"
ENTRY = "src/main.js"

IMPORT_RE = re.compile(r'^[ \t]*import\s+(.+?)\s+from\s+[\'"]([^\'"]+)[\'"]\s*;?[ \t]*\n?', re.M | re.S)
EXPORT_LIST_RE = re.compile(r'^[ \t]*export\s*\{([^}]*)\}\s*;?[ \t]*\n?', re.M | re.S)
EXPORT_DECL_RE = re.compile(r'^export\s+(async function|function|class|const|let|var)\s+(\w+)', re.M)
UNSUPPORTED_RE = re.compile(r'^[ \t]*(export\s+default|export\s*\*\s*from|import\s+[\'"])|[^\'"`]\bimport\s*\(', re.M)


def resolve(spec: str, importer: Path) -> tuple[str, Path]:
    """导入说明符 → (规范模块 id, 绝对路径)。"""
    if spec == "three":
        return "three", VENDOR / "three.module.js"
    if spec.startswith("three/addons/"):
        return spec, VENDOR / "addons" / spec[len("three/addons/"):]
    path = (importer.parent / spec).resolve()
    return path.relative_to(SIM).as_posix(), path


def parse_names(clause: str) -> list[tuple[str, str]]:
    """'{ A, B as C }' → [(本地名, 导出名)]；'A' → [(A, A)]。"""
    out = []
    for item in clause.strip().strip("{}").split(","):
        item = item.strip()
        if not item:
            continue
        if " as " in item:
            a, b = (s.strip() for s in item.split(" as "))
            out.append((b, a))
        else:
            out.append((item, item))
    return out


def transform(path: Path) -> tuple[str, list[tuple[str, Path]], list[str]]:
    """单模块转换 → (tiny-AMD 工厂函数体, 依赖 (id, 绝对路径) 列表, 导出局部名列表)。"""
    text = path.read_text(encoding="utf-8")
    if m := UNSUPPORTED_RE.search(text):
        line = text[: m.start()].count("\n") + 1
        raise SyntaxError(f"{path.relative_to(SIM)}:{line} 含不支持的形式：{m.group(0).strip()!r}")

    deps: list[tuple[str, Path]] = []
    exports: list[str] = []

    def import_repl(m: re.Match[str]) -> str:
        clause, spec = m.group(1).strip(), m.group(2)
        mod_id, dep_path = resolve(spec, path)
        deps.append((mod_id, dep_path))
        if clause.startswith("*"):
            ns = re.fullmatch(r"\*\s+as\s+(\w+)", clause)
            if not ns:
                raise SyntaxError(f"{path}: 无法解析导入 {m.group(0)!r}")
            return f"const {ns.group(1)} = __req({mod_id!r});\n"
        if clause.startswith("{"):
            pairs = parse_names(clause)
            destructure = ", ".join(f"{src}: {dst}" if src != dst else dst for dst, src in pairs)
            return f"const {{ {destructure} }} = __req({mod_id!r});\n"
        raise SyntaxError(f"{path}: default 导入不受支持：{m.group(0)!r}")

    body = IMPORT_RE.sub(import_repl, text)

    def export_list_repl(m: re.Match[str]) -> str:
        for dst, src in parse_names(m.group(1)):
            exports.append(dst if dst == src else f"{dst}: {src}")
        return ""

    body = EXPORT_LIST_RE.sub(export_list_repl, body)

    def export_decl_repl(m: re.Match[str]) -> str:
        exports.append(m.group(2))
        return f"{m.group(1)} {m.group(2)}"

    body = EXPORT_DECL_RE.sub(export_decl_repl, body)
    return body, deps, exports


def collect() -> dict[str, tuple[Path, str, list[str]]]:
    """从入口出发收集依赖闭包 → {模块 id: (路径, 工厂体, 导出列表)}。"""
    modules: dict[str, tuple[Path, str, list[str]]] = {}
    queue = [(ENTRY, SIM / ENTRY)]
    while queue:
        mod_id, path = queue.pop(0)
        if mod_id in modules:
            continue
        body, deps, exports = transform(path)
        modules[mod_id] = (path, body, exports)
        queue.extend(d for d in deps if d[0] not in modules)
    return modules


RUNTIME = """'use strict';
const __mods = Object.create(null), __exports = Object.create(null);
function __def(id, factory) { __mods[id] = factory; }
function __req(id) {
  if (id in __exports) return __exports[id];
  if (!__mods[id]) throw new Error('standalone: 模块缺失 ' + id);
  return __exports[id] = __mods[id]();
}
"""


def build_bundle() -> str:
    """从入口模块收集依赖闭包并转换为 tiny-AMD 定义表（返回拼接后的 JS 文本）。

    @note   遇到不支持的 ESM 形式（default 导入/导出、re-export、动态 import）直接报错，
             不做静默降级——否则产物会在浏览器里运行期失败。
    """
    modules = collect()
    parts = [RUNTIME]
    for mod_id, (_path, body, exports) in modules.items():
        ret = ", ".join(exports)
        parts.append(f"__def({mod_id!r}, function() {{\n{body}\nreturn {{ {ret} }};\n}});\n")
    parts.append(f"__req({ENTRY!r});\n")
    return "".join(parts)


def build_html(bundle: str) -> str:
    """把模块包、CSS 与启动脚本拼成单文件 HTML（file:// 可直接打开）。"""
    html = (SIM / "index.html").read_text(encoding="utf-8")
    css = (SIM / "css" / "style.css").read_text(encoding="utf-8")
    css_tag = '<link rel="stylesheet" href="./css/style.css">'
    if css_tag not in html:
        raise RuntimeError(f"index.html 中未找到待替换片段：{css_tag}")
    html = html.replace(css_tag, "<style>\n" + css + "\n</style>")
    for tag in ('<script async src="./vendor/three-r170/es-module-shims.min.js"></script>\n',
                '<script type="importmap-shim">\n'
                '{"imports":{"three":"./vendor/three-r170/three.module.js","three/addons/":"./vendor/three-r170/addons/"}}\n'
                '</script>\n',
                '<script type="module-shim" src="./src/main.js"></script>'):
        if tag not in html:
            raise RuntimeError(f"index.html 中未找到待替换片段：{tag[:60]}...")
        html = html.replace(tag, "")
    banner = ("<!-- 本文件由 tools/build-standalone.py 生成，勿手改；"
              "源：index.html + css/style.css + src/ + vendor/three-r170（Three.js r170, MIT） -->\n")
    html = html.replace("<!DOCTYPE html>\n", "<!DOCTYPE html>\n" + banner, 1)
    html = html.replace("</body>", "<script>\n" + bundle + "</script>\n</body>", 1)
    return html


def main() -> int:
    """CLI 入口：默认生成 standalone.html；--check 重建并字节比对。"""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="漂移检测：重建并比对，不写入")
    args = ap.parse_args()

    html = build_html(build_bundle())
    if args.check:
        if not OUT.is_file():
            print(f"standalone.html 不存在，请先运行 build-standalone.py")
            return 1
        if OUT.read_text(encoding="utf-8") != html:
            print("standalone.html 与源存在漂移，请重新运行 tools/build-standalone.py")
            return 1
        print("standalone.html 与源一致。")
        return 0
    OUT.write_text(html, encoding="utf-8", newline="\n")
    print(f"生成 {OUT.relative_to(ROOT)}（{len(html) / 1024:.0f} KB），双击即可运行，无需服务器。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
