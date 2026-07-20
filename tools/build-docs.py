#!/usr/bin/env python3
"""文档站点可复现构建：docs/ 下 Markdown → simulations/vector-thrust-lab/docs/ HTML 站。

用法（仓库根目录）：
    py -3.12 tools/build-docs.py            构建并写入站点
    py -3.12 tools/build-docs.py --check    漂移检测：临时目录重建并与现有站点比对

约定：
- 固定 pandoc 命令（gfm → html5，独立页面，docstyle.css）；
- 每页注入返回导航（← 返回文档中心）与页脚溯源（源文件、生成时间、工具版本）；
- index.html 文档中心由本脚本从页面清单生成，不手工维护；
- --check 比对前剥离页脚溯源行（含时间戳），其余字节级一致。
"""
from __future__ import annotations

import argparse
import datetime
import posixpath
import re
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SITE = ROOT / "simulations" / "vector-thrust-lab" / "docs"

# (分区标题, [(源 MD（相对仓库根）, 输出 HTML（相对站点根）, 页面标题, 卡片描述)])
SECTIONS: list[tuple[str, list[tuple[str, str, str, str]]]] = [
    ("项目治理", [
        ("docs/00-项目治理/GOV-001-项目范围与术语.md", "governance/GOV-001.html",
         "GOV-001 项目范围与术语", "项目定位、范围边界、子系统清单、文档体系地图与术语表"),
        ("docs/00-项目治理/GOV-002-信息与配置管理.md", "governance/GOV-002.html",
         "GOV-002 信息与配置管理", "文档编号规则、参数治理工作流与状态分类、Git 配置管理、行为保持红线"),
    ]),
    ("方案设计", [
        ("docs/01-方案设计/CFG-000-概念构型基线C0.md", "design/CFG-000.html",
         "CFG-000 概念构型基线 C0", "纵列双发矢量推力布局：总体参数、推进构型、三通道控制机理、已知限制"),
        ("docs/01-方案设计/CONOPS-001-运行概念.md", "design/CONOPS-001.html",
         "CONOPS-001 运行概念", "平台定位、用户角色、运行模式与典型使用场景"),
    ]),
    ("需求与验证", [
        ("docs/02-需求与验证/REQ-001-需求基线.md", "requirements/REQ-001.html",
         "REQ-001 需求基线", "需求编号规则、功能/性能/接口需求表、验证方法与状态"),
    ]),
    ("理论推导", [
        ("docs/03-理论推导/THY-001-旋转与姿态的数学基础.md", "theory/01-attitude-kinematics.html",
         "纯理论推导（一）：旋转与姿态的数学基础", "DCM、轴角与罗德里格斯公式、SO(3) 与角速度、四元数代数与姿态运动学"),
        ("docs/03-理论推导/THY-002-推进系统的物理建模.md", "theory/02-propulsion-modeling.html",
         "纯理论推导（二）：推进系统的物理建模", "桨盘动量理论、k_T/k_Q 相似律来源、反扭矩角动量守恒、电机动态与陀螺力矩"),
        ("docs/03-理论推导/THY-003-刚体动力学控制分配与模态分析.md", "theory/03-dynamics-allocation-modes.html",
         "纯理论推导（三）：刚体动力学、控制分配与模态分析", "牛顿-欧拉方程、控制效能矩阵、权限退化、模态结构与增稳机理"),
    ]),
    ("数学建模", [
        ("docs/04-数学建模/MOD-001-六自由度仿真模型规范.md", "modeling.html",
         "理论分析与建模文档", "从单电机矢量座到整机六自由度：推力映射、气动模型、配平分析、SAS 律与演示性数值检查"),
        ("docs/04-数学建模/MOD-002-坐标系与符号约定.md", "modeling/MOD-002.html",
         "MOD-002 坐标系与符号约定", "代码↔文档接口契约：DCM/四元数/欧拉角实现约定、符号表与代码变量对照"),
        ("docs/04-数学建模/MOD-003-配平与稳定性分析.md", "modeling/MOD-003.html",
         "MOD-003 配平与稳定性分析", "纵向配平推导与数值核对、C_m0 补偿案例、小扰动模态定性分析"),
    ]),
    ("控制与分配", [
        ("docs/05-控制与分配/CTL-001-控制架构与SAS.md", "control/CTL-001.html",
         "CTL-001 控制架构与 SAS", "控制链总览、三通道 SAS 律、反馈极性整定、差速开方分配"),
        ("docs/05-控制与分配/CTL-002-可控性与失效模式.md", "control/CTL-002.html",
         "CTL-002 可控性与失效模式", "控制效能矩阵、可控性包线、失效模式登记与改进方向"),
    ]),
    ("推进与执行机构", [
        ("docs/06-推进与执行机构/PROP-001-推进与摆座模型.md", "propulsion/PROP-001.html",
         "PROP-001 推进与摆座模型", "推力/反扭矩集总模型、一阶滞后、摆座运动学、待标定项与台架计划"),
    ]),
    ("验证与确认", [
        ("docs/07-验证与确认/VER-001-验证确认与模型可信度.md", "verification/VER-001.html",
         "VER-001 验证确认与模型可信度", "V 模型框架、证据清单、模型可信度分级与证据规则"),
        ("docs/07-验证与确认/SAFE-001-初步安全计划.md", "verification/SAFE-001.html",
         "SAFE-001 初步安全计划", "纯软件仿真范围声明、软件层面风险与缓解、硬件化前瞻议题登记"),
    ]),
    ("决策记录", [
        ("docs/09-决策记录/DEC-001-参数单一事实源.md", "decisions/DEC-001.html",
         "DEC-001 参数单一事实源", "ADR：aircraft-model.json + 代码生成 + 漂移检测的决策与备选方案"),
        ("docs/09-决策记录/模板-决策记录.md", "decisions/adr-template.html",
         "决策记录模板（ADR）", "新增决策记录使用的空白模板"),
    ]),
    ("参考资料", [
        ("docs/08-参考资料/参考资料注册表.md", "references/reference-register.html",
         "参考资料注册表", "第三方依赖与工具链登记（Three.js、es-module-shims、Pandoc 等）"),
    ]),
    ("注册表", [
        ("docs/registers/参数数据手册.md", "registers/parameter-handbook.html",
         "参数数据手册", "46 项参数全量手册：值、单位、来源、置信度与代码字段"),
        ("docs/registers/假设与局限日志.md", "registers/assumptions-log.html",
         "假设与局限日志", "ASM-001~014：模型假设、影响评估与解除条件"),
        ("docs/registers/模型-代码追溯矩阵.md", "registers/traceability-matrix.html",
         "模型-代码追溯矩阵", "方程 ↔ 代码位置 ↔ 单元测试 ↔ 回归基线场景"),
        ("docs/registers/需求注册表.md", "registers/requirements-register.html",
         "需求注册表", "REQ-ID、来源、验证方法、状态与证据"),
        ("docs/registers/验证覆盖矩阵.md", "registers/verification-coverage.html",
         "验证覆盖矩阵", "需求 ↔ 验证方法 ↔ 测试证据 ↔ 状态"),
        ("docs/registers/危害与失效模式日志.md", "registers/hazard-log.html",
         "危害与失效模式日志", "HAZ-001~008：失效模式、严重度、检测与缓解"),
        ("docs/registers/templates/工况与试验卡.md", "registers/test-case-card.html",
         "工况与试验卡模板", "仿真工况与试验记录的空白模板"),
    ]),
]

PROVENANCE_STYLE = (
    "<style>.provenance{margin-top:36px;padding-top:10px;border-top:1px solid var(--line,#333);"
    "font-size:11px;color:var(--dim,#888);letter-spacing:.05em}</style>"
)

HUB_HEAD = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>文档中心 · 纵列双发矢量推力飞行器</title>
<link rel="stylesheet" href="docstyle.css">
<style>
  .hub-head{margin:0 0 26px}
  .hub-head h1{border-bottom:none;margin-bottom:6px;padding-bottom:0}
  .hub-head p{color:var(--dim);font-size:13px;letter-spacing:.12em}
  .card{display:block;text-decoration:none;background:var(--panel);border:1px solid var(--line);
    border-radius:14px;padding:20px 24px;margin:14px 0;transition:all .2s;
    box-shadow:0 8px 30px rgba(0,0,0,.35), inset 0 1px 0 rgba(255,255,255,.04)}
  .card:hover{border-color:rgba(56,189,248,.55);transform:translateY(-2px);box-shadow:0 12px 36px rgba(0,0,0,.45),0 0 18px rgba(56,189,248,.12)}
  .card .tag{display:inline-block;font-size:10.5px;letter-spacing:.15em;color:var(--cyan);
    border:1px solid rgba(56,189,248,.4);border-radius:6px;padding:2px 8px;margin-bottom:8px}
  .card .tag.pure{color:#fbbf24;border-color:rgba(251,191,36,.4)}
  .card h2{margin:0 0 6px;font-size:16px;color:#e0f2fe;letter-spacing:.04em}
  .card p{margin:0;font-size:12.5px;color:var(--dim);line-height:1.7}
  .sec{font-size:11px;letter-spacing:.25em;color:var(--dim);margin:30px 0 4px}
</style>
</head>
<body>
<article>
  <a class="back" href="../index.html">← 返回仿真实验室</a>
  <div class="hub-head">
    <h1>纵列双发矢量推力飞行器 · 文档中心</h1>
    <p>TANDEM TWIN VECTOR-THRUST AIRCRAFT — DOCUMENTATION</p>
  </div>
"""

HUB_FOOT = "</article>\n</body>\n</html>\n"


def pandoc_version() -> str:
    out = subprocess.run(["pandoc", "--version"], capture_output=True, text=True,
                         encoding="utf-8", check=True).stdout
    return out.splitlines()[0].strip()


def md_to_html_map() -> dict[str, str]:
    """仓库根相对 MD 路径 → 站点根相对 HTML 路径（跨文档链接改写用）。"""
    return {src: out for _s, pages in SECTIONS for src, out, _t, _d in pages}


MD_HREF = re.compile(r'href="([^"]+\.md)(#[^"]*)?"')


def rewrite_md_links(html: str, src_rel: str, out_rel: str) -> str:
    """将 pandoc 输出中指向 .md 的链接改写为站内对应 HTML 页面。

    源 MD 间的相对链接（GitHub/编辑器中有效）在 HTML 站中会 404，
    按页面清单映射为 HTML 路径；清单未覆盖的目标保持原样（由 check-links.py 兜底）。
    """
    mapping = md_to_html_map()
    out_dir = posixpath.dirname(out_rel) or "."

    def repl(m: re.Match[str]) -> str:
        resolved = posixpath.normpath(posixpath.join(posixpath.dirname(src_rel), m.group(1)))
        target = mapping.get(resolved)
        if target is None:
            return m.group(0)
        frag = m.group(2) or ""
        return f'href="{posixpath.relpath(target, out_dir)}{frag}"'

    return MD_HREF.sub(repl, html)


def build_page(src: Path, out: Path, title: str, depth: int, src_rel: str, out_rel: str) -> None:
    """单页构建：pandoc + 返回导航 + 页脚溯源。

    depth 为输出页面相对站点根的目录深度（0 = 站点根），
    用于计算 CSS 与返回链接的相对前缀。
    """
    prefix = "../" * depth
    css = f"{prefix}docstyle.css"
    back = f"{prefix}index.html" if depth else "./index.html"
    full_title = f"{title} · 纵列双发矢量推力飞行器"

    result = subprocess.run(
        ["pandoc", "-f", "gfm", "-t", "html5", "-s",
         "--metadata", f"pagetitle={full_title}", "--css", css, str(src)],
        capture_output=True, text=True, encoding="utf-8")
    if result.returncode != 0:
        raise RuntimeError(f"pandoc 失败 {src}:\n{result.stderr}")
    html = result.stdout

    nav = f'<body><article><a class="back" href="{back}">← 返回文档中心</a>'
    if "<body>" not in html:
        raise RuntimeError(f"pandoc 输出缺少 <body>：{src}")
    html = html.replace("<body>", nav, 1)

    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    rel_src = src.relative_to(ROOT).as_posix()
    footer = (f'<footer class="provenance">源文件 <code>{rel_src}</code> · '
              f'由 <code>tools/build-docs.py</code> 生成 · {PANDOC_VER} · {stamp}</footer>')
    html = html.replace("</body>", f"{footer}</article></body>", 1)
    html = html.replace("</head>", f"{PROVENANCE_STYLE}</head>", 1)
    html = rewrite_md_links(html, src_rel, out_rel)

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(html, encoding="utf-8", newline="\n")


def build_hub(site: Path) -> None:
    """文档中心 index.html 由页面清单生成。"""
    parts = [HUB_HEAD]
    for section, pages in SECTIONS:
        parts.append(f'  <div class="sec">{section}</div>\n')
        for _src, out, title, desc in pages:
            tag = "pure" if title.startswith("纯理论推导") else ""
            tag_cls = f' pure' if tag else ""
            tag_text = "THEORY" if tag else title.split()[0]
            parts.append(
                f'  <a class="card" href="./{out}">\n'
                f'    <span class="tag{tag_cls}">{tag_text}</span>\n'
                f'    <h2>{title}</h2>\n'
                f'    <p>{desc}</p>\n'
                f'  </a>\n')
    parts.append(HUB_FOOT)
    (site / "index.html").write_text("".join(parts), encoding="utf-8", newline="\n")


def build_all(site: Path) -> int:
    count = 0
    for _section, pages in SECTIONS:
        for src_rel, out_rel, title, _desc in pages:
            src = ROOT / src_rel
            if not src.is_file():
                raise FileNotFoundError(f"清单源文件不存在：{src_rel}")
            build_page(src, site / out_rel, title, len(Path(out_rel).parent.parts), src_rel, out_rel)
            count += 1
    build_hub(site)
    return count + 1


PROVENANCE_RE = re.compile(r'<footer class="provenance">.*?</footer>', re.S)


def scrub(text: str) -> str:
    """剥离含时间戳的页脚，供 --check 比对。"""
    return PROVENANCE_RE.sub("<footer/>", text)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="漂移检测：重建并比对，不写入站点")
    args = ap.parse_args()

    global PANDOC_VER
    PANDOC_VER = pandoc_version()

    if not args.check:
        n = build_all(SITE)
        print(f"构建完成：{n} 页 → {SITE.relative_to(ROOT)}")
        return 0

    with tempfile.TemporaryDirectory() as tmp:
        tmp_site = Path(tmp) / "site"
        build_all(tmp_site)
        drift = []
        for rebuilt in sorted(tmp_site.rglob("*.html")):
            rel = rebuilt.relative_to(tmp_site)
            existing = SITE / rel
            if not existing.is_file():
                drift.append(f"新增（站点缺失）: {rel}")
            elif scrub(existing.read_text(encoding="utf-8")) != scrub(rebuilt.read_text(encoding="utf-8")):
                drift.append(f"内容漂移: {rel}")
        for existing in sorted(SITE.rglob("*.html")):
            rel = existing.relative_to(SITE)
            if not (tmp_site / rel).is_file():
                drift.append(f"多余（清单未覆盖）: {rel}")
    if drift:
        print("站点与 Markdown 源存在漂移：")
        for d in drift:
            print(f"  {d}")
        return 1
    print("站点与 Markdown 源一致（页脚时间戳除外）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
