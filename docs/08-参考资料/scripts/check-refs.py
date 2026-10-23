#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check-refs.py — REF 文献库一致性校验 / catalog 生成器

校验范围（errors，任一不满足即失败）：
  1. 注册表（主注册表 + index/registry_pending.md）与 papers/ books/ 目录的双向一致：
     每个文献目录必须在注册表或 pending 表登记；每个登记了本地路径的 REF 必须有对应目录。
  2. REF 编号唯一（注册表、pending、目录三处合并去重后仍唯一）。
  3. papers/REF-*：paper.pdf 存在、大小合理、文件头 = "%PDF-"（禁止登录页 HTML 冒充 PDF）。
  4. papers/REF-*：paper.md 存在且字符数 ≥ MIN_MD_CHARS（非空/非占位）。
  5. papers/REF-*：notes.md 存在且含必备节（题录 / 摘要 / 与本项目的关系 / 可借鉴 / 不适用）。
     历史遗留条目（REF < STRICT_MIN_REF）降级为 warning，不阻断。
  6. books/REF-*：book.pdf 头校验；book.json 必须存在且含必备字段；book.json 里
     volumes 指向的输出目录必须存在且 full.md 非空；pdf.sha256 必须与实际文件一致。
  7. index/catalog.json（若存在）：可解析、REF 集合与磁盘一致、pdf_sha256 / md_chars 与磁盘一致。

warning（不阻断）：
  - 历史遗留论文缺 notes.md 或 notes 结构不完整。
  - MinerU quality_report.json 的 status != "ok"（含 HTML 表格等保真度告警）。
  - 注册表状态与磁盘实际不一致中"可解释"的情形（如"仅题录"但本地有目录）。

用法：
  py -3.12 docs/08-参考资料/scripts/check-refs.py
  py -3.12 docs/08-参考资料/scripts/check-refs.py --update-catalog     # 重新生成 index/catalog.json 后再校验
  py -3.12 docs/08-参考资料/scripts/check-refs.py --json              # 机器可读输出
  py -3.12 docs/08-参考资料/scripts/check-refs.py --quiet             # 只输出结论与错误

退出码：0 = 无 error；1 = 存在 error；2 = 脚本自身异常。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

# ---------------------------------------------------------------- 常量

SCRIPT = Path(__file__).resolve()
REF_ROOT = SCRIPT.parent.parent              # docs/08-参考资料
PAPERS_DIR = REF_ROOT / "papers"
BOOKS_DIR = REF_ROOT / "books"
INDEX_DIR = REF_ROOT / "index"
CATALOG = INDEX_DIR / "catalog.json"
MAIN_REGISTRY = REF_ROOT / "参考资料注册表.md"
PENDING_REGISTRY = INDEX_DIR / "registry_pending.md"

REF_DIR_RE = re.compile(r"^REF-(\d{3})-([A-Za-z][A-Za-z\-']*)-(\d{4})-(.+)$")
MIN_PDF_BYTES = 1024
MIN_MD_CHARS = 2000
STRICT_MIN_REF = 122          # >=122：本轮新增文献库，notes 必备节为硬性要求；<122 为历史遗留
BOOK_REQUIRED_FIELDS = ("ref", "title", "authors", "isbn", "pdf", "volumes", "quality_status")

REQUIRED_NOTE_SECTIONS = [
    ("题录", r"题录"),
    ("摘要", r"摘要"),
    ("与本项目的关系", r"与(本)?项目的关系"),
    ("可借鉴", r"可借鉴"),
    ("不适用", r"不适用"),
]

# ---------------------------------------------------------------- 数据结构


class Report:
    def __init__(self) -> None:
        self.errors: list[str] = []
        self.warnings: list[str] = []
        self.info: dict = {}

    def err(self, msg: str) -> None:
        self.errors.append(msg)

    def warn(self, msg: str) -> None:
        self.warnings.append(msg)


def sha256_file(path: Path, cache: dict) -> str:
    key = str(path)
    if key in cache:
        return cache[key]
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    digest = h.hexdigest()
    cache[key] = digest
    return digest


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def pdf_header_ok(path: Path) -> bool:
    try:
        with open(path, "rb") as f:
            return f.read(5) == b"%PDF-"
    except OSError:
        return False


def count_chars(path: Path) -> int:
    try:
        return len(read_text(path))
    except OSError:
        return 0


# ---------------------------------------------------------------- 注册表解析

def parse_registry_table(path: Path) -> dict[str, dict]:
    """解析 Markdown 注册表表格（表头：编号 | 标题 | 类型 | 位置/链接 | 用途 | 状态）。

    只解析表头以「编号」「标题」开头的表格——索引文件里还有别的表（完成表/失败项表）也以
    REF-xxx 开头，必须排除，否则会覆盖正确的注册表行。同一 REF 以首次出现为准。
    """
    rows: dict[str, dict] = {}
    if not path.exists():
        return rows
    in_table = False
    header: list[str] = []
    for line in read_text(path).splitlines():
        line = line.strip()
        if not line.startswith("|"):
            in_table = False
            header = []
            continue
        cells = [c.strip() for c in line.strip("|").split("|")]
        if cells and cells[0] in ("编号", "REF", "Ref"):
            in_table = True
            header = cells
            continue
        if not in_table:
            continue
        if len(cells) < 4 or not cells[0].startswith("REF-"):
            continue
        if not re.fullmatch(r"REF-\d{3}", cells[0].strip()):
            continue
        ref = cells[0].strip()
        if ref in rows:
            continue
        title = cells[1] if len(cells) > 1 else ""
        ctype = cells[2] if len(cells) > 2 else ""
        location = cells[3] if len(cells) > 3 else ""
        status = cells[-1] if len(cells) >= 5 else ""
        m = re.search(r"`([^`]+)`", location)
        local = m.group(1).strip() if m else ""
        rows[ref] = {
            "ref": ref,
            "title": re.sub(r"<[^>]+>", "", title).strip(),
            "type": ctype,
            "location": local,
            "status": status,
            "source_file": path.name,
        }
    return rows


def resolve_local_dir(location: str) -> Path | None:
    """把注册表里的本地路径（如 papers/REF-xxx/ 或 books/REF-xxx/）解析成绝对目录。"""
    if not location:
        return None
    loc = location.strip().rstrip("/")
    if not (loc.startswith("papers/") or loc.startswith("books/")):
        return None
    p = REF_ROOT / loc
    return p if p.exists() else p  # 返回期望路径（可能不存在，由调用方判定）


# ---------------------------------------------------------------- 目录发现

def discover_papers(report: Report) -> dict[str, Path]:
    found: dict[str, Path] = {}
    if not PAPERS_DIR.exists():
        report.err(f"论文目录不存在：{PAPERS_DIR}")
        return found
    for d in sorted(PAPERS_DIR.iterdir()):
        if not d.is_dir() or d.name.startswith("."):
            continue
        m = REF_DIR_RE.match(d.name)
        if not m:
            report.err(f"论文目录命名不符合 REF-<nnn>-<作者>-<年份>-<关键词>：{d.name}")
            continue
        ref = f"REF-{m.group(1)}"
        if ref in found:
            report.err(f"REF 编号重复目录：{ref}（{found[ref].name} 与 {d.name}）")
            continue
        found[ref] = d
    return found


def discover_books(report: Report) -> dict[str, Path]:
    found: dict[str, Path] = {}
    if not BOOKS_DIR.exists():
        report.warn(f"书籍目录不存在（可忽略）：{BOOKS_DIR}")
        return found
    for d in sorted(BOOKS_DIR.iterdir()):
        if not d.is_dir() or d.name.startswith("."):
            continue
        m = REF_DIR_RE.match(d.name)
        if not m:
            report.err(f"书籍目录命名不符合规范：{d.name}")
            continue
        ref = f"REF-{m.group(1)}"
        if ref in found:
            report.err(f"REF 编号重复目录：{ref}（{found[ref].name} 与 {d.name}）")
            continue
        found[ref] = d
    return found


# ---------------------------------------------------------------- 论文校验

def required_sections_missing(text: str) -> list[str]:
    miss = []
    for name, pattern in REQUIRED_NOTE_SECTIONS:
        if not re.search(pattern, text):
            miss.append(name)
    return miss


def check_paper(ref: str, d: Path, report: Report, sha_cache: dict) -> dict:
    entry: dict = {
        "ref": ref,
        "type": "paper",
        "path": f"papers/{d.name}",
        "dir_name": d.name,
        "status": "未就位",
        "pdf_sha256": None,
        "pdf_bytes": None,
        "pdf_pages": None,
        "md_chars": 0,
        "notes_ok": False,
        "quality": None,
        "title": None,
        "authors": None,
        "year": None,
    }
    m = REF_DIR_RE.match(d.name)
    if m:
        entry["authors"] = [m.group(2)]
        entry["year"] = int(m.group(3))

    pdf = d / "paper.pdf"
    md = d / "paper.md"
    notes = d / "notes.md"

    # --- PDF
    if not pdf.exists():
        report.err(f"{ref}: 缺少 paper.pdf（{entry['path']}）")
    else:
        size = pdf.stat().st_size
        entry["pdf_bytes"] = size
        if size < MIN_PDF_BYTES:
            report.err(f"{ref}: paper.pdf 过小（{size} B），疑似占位/坏文件")
        elif not pdf_header_ok(pdf):
            head = pdf.read_bytes()[:8].decode("latin-1", "replace")
            report.err(f"{ref}: paper.pdf 文件头不是 %PDF-（实际 {head!r}）——可能是登录页 HTML")
        else:
            entry["pdf_sha256"] = sha256_file(pdf, sha_cache)

    # --- Markdown
    if not md.exists():
        report.err(f"{ref}: 缺少 paper.md（MinerU 转换产物未就位）")
    else:
        chars = count_chars(md)
        entry["md_chars"] = chars
        if chars < MIN_MD_CHARS:
            report.err(f"{ref}: paper.md 内容过短（{chars} 字符 < {MIN_MD_CHARS}），疑似空转换")

    # --- notes
    strict = int(ref.split("-")[1]) >= STRICT_MIN_REF
    if not notes.exists():
        if strict:
            report.err(f"{ref}: 缺少 notes.md（必备：题录 + 摘要 + 项目关系 + 可借鉴/不适用）")
        else:
            report.warn(f"{ref}: 历史遗留条目缺 notes.md（不阻断，建议补写）")
    else:
        text = read_text(notes)
        miss = required_sections_missing(text)
        entry["notes_ok"] = not miss
        if miss:
            if strict:
                report.err(f"{ref}: notes.md 缺少必备节：{'、'.join(miss)}")
            else:
                report.warn(f"{ref}: 历史遗留 notes.md 结构不完整（缺 {'、'.join(miss)}），不阻断")

    # --- quality report
    q = d / "quality_report.json"
    if q.exists():
        try:
            qd = json.loads(read_text(q))
            entry["quality"] = qd.get("status")
            mdq = qd.get("markdown") or {}
            entry["pdf_pages"] = None
            if entry["quality"] not in (None, "ok"):
                report.warn(f"{ref}: MinerU 质量状态 = {entry['quality']}（{len(qd.get('warnings') or [])} 条告警）")
        except (json.JSONDecodeError, OSError) as exc:
            report.warn(f"{ref}: quality_report.json 无法解析（{exc}）")

    # --- title / authors：优先 paper.md 一级标题 + notes 题录表
    if md.exists():
        for line in read_text(md).splitlines()[:40]:
            if line.startswith("# "):
                title = re.sub(r"<[^>]+>", "", line[2:]).strip()
                title = re.sub(r"\s+", " ", title)
                entry["title"] = title or None
                break
    if notes.exists():
        text = read_text(notes)
        am = re.search(r"\|\s*作者\s*\|\s*([^|\n]+)\|", text)
        if am:
            authors = [a.strip() for a in re.split(r"[,，、;；]", am.group(1)) if a.strip()]
            if authors:
                entry["authors"] = authors
        ym = re.search(r"\|\s*年份\s*\|\s*([^|\n]+)\|", text)
        if ym:
            y = re.search(r"(19|20)\d{2}", ym.group(1))
            if y:
                entry["year"] = int(y.group(0))

    if entry["pdf_sha256"] and entry["md_chars"] >= MIN_MD_CHARS:
        entry["status"] = "已就位"
    elif entry["pdf_sha256"]:
        entry["status"] = "缺 Markdown"
    elif entry["md_chars"]:
        entry["status"] = "缺 PDF"
    return entry


# ---------------------------------------------------------------- 书籍校验

def check_book(ref: str, d: Path, report: Report, sha_cache: dict) -> dict:
    entry: dict = {
        "ref": ref,
        "type": "book",
        "path": f"books/{d.name}",
        "status": "未就位",
        "pdf_sha256": None,
        "pdf_bytes": None,
        "pdf_pages": None,
        "md_chars": 0,
        "notes_ok": False,
        "quality": None,
        "title": None,
        "authors": None,
        "year": None,
        "volumes": [],
    }
    m = REF_DIR_RE.match(d.name)
    if m:
        entry["authors"] = [m.group(2)]
        entry["year"] = int(m.group(3))

    pdf = d / "book.pdf"
    book_json = d / "book.json"

    if not pdf.exists():
        report.err(f"{ref}: 缺少 book.pdf")
    else:
        entry["pdf_bytes"] = pdf.stat().st_size
        if not pdf_header_ok(pdf):
            report.err(f"{ref}: book.pdf 文件头不是 %PDF-")
        else:
            entry["pdf_sha256"] = sha256_file(pdf, sha_cache)
        if pdf.stat().st_size < 10 * MIN_PDF_BYTES:
            report.warn(f"{ref}: book.pdf 体积偏小（{pdf.stat().st_size} B），请确认是否为完整书")

    if not book_json.exists():
        report.err(f"{ref}: 缺少 book.json（题录/ISBN/页数/SHA256/分卷页范围/质量状态）")
        return entry

    try:
        meta = json.loads(read_text(book_json))
    except json.JSONDecodeError as exc:
        report.err(f"{ref}: book.json 不是合法 JSON（{exc}）")
        return entry

    missing = [f for f in BOOK_REQUIRED_FIELDS if f not in meta]
    if missing:
        report.err(f"{ref}: book.json 缺少必备字段：{'、'.join(missing)}")

    entry["title"] = meta.get("title")
    entry["authors"] = meta.get("authors") or entry["authors"]
    entry["year"] = meta.get("year") or entry["year"]
    entry["quality"] = meta.get("quality_status")
    entry["isbn"] = meta.get("isbn")
    entry["notes_ok"] = True  # book.json 即书籍的元数据笔记

    pdf_meta = meta.get("pdf") or {}
    if entry["pdf_sha256"] and pdf_meta.get("sha256") and pdf_meta["sha256"] != entry["pdf_sha256"]:
        report.err(f"{ref}: book.json 记录的 SHA256 与实际 book.pdf 不一致")
    if pdf_meta.get("bytes") and entry["pdf_bytes"] and pdf_meta["bytes"] != entry["pdf_bytes"]:
        report.warn(f"{ref}: book.json 记录的字节数与实际不一致（{pdf_meta['bytes']} vs {entry['pdf_bytes']}）")
    entry["pdf_pages"] = pdf_meta.get("pdf_pages")
    if pdf_meta.get("header_ok") is not True:
        report.warn(f"{ref}: book.json 未标注 header_ok=true")

    volumes = meta.get("volumes") or []
    if not volumes:
        report.err(f"{ref}: book.json 的 volumes 为空（必须给出分卷页范围与转换产物）")
    for vol in volumes:
        name = vol.get("name") or "?"
        out = vol.get("output_dir")
        md_rel = vol.get("markdown")
        chars = vol.get("md_chars")
        if not out:
            report.err(f"{ref}: volumes[{name}] 缺 output_dir")
            continue
        vdir = d / out
        if not vdir.exists():
            report.err(f"{ref}: volumes[{name}] 输出目录不存在：{out}/")
            continue
        md_path = d / (md_rel or f"{out}/full.md")
        if not md_path.exists():
            report.err(f"{ref}: volumes[{name}] 缺 Markdown：{md_path.relative_to(d)}")
            continue
        actual = count_chars(md_path)
        if actual < MIN_MD_CHARS:
            report.err(f"{ref}: volumes[{name}] Markdown 过短（{actual} 字符）")
        if isinstance(chars, int) and abs(actual - chars) > 5:
            report.warn(f"{ref}: volumes[{name}] md_chars 记录 {chars} 与实际 {actual} 不一致")
        entry["volumes"].append({
            "name": name,
            "page_range": vol.get("page_range"),
            "md_chars": actual,
            "quality": vol.get("quality_status"),
        })

    merged = meta.get("merged_artifacts") or []
    for art in merged:
        rel = art.get("path")
        if rel and rel.endswith(".md"):
            p = d / rel
            if p.exists():
                entry["md_chars"] += count_chars(p)
            else:
                report.warn(f"{ref}: merged_artifacts 指向的 {rel} 不存在")
    if entry["md_chars"] == 0:
        for vol in entry["volumes"]:
            entry["md_chars"] += vol.get("md_chars") or 0

    if entry["pdf_sha256"] and entry["volumes"]:
        entry["status"] = "已入库"
    elif entry["pdf_sha256"]:
        entry["status"] = "缺转换产物"
    if entry["quality"] and entry["quality"] != "ok":
        report.warn(f"{ref}: 书籍质量状态 = {entry['quality']}（HTML 表格等保真度告警，见 book.json）")
    return entry


# ---------------------------------------------------------------- 注册表交叉校验

def cross_check_registry(
    report: Report,
    papers: dict[str, Path],
    books: dict[str, Path],
    main_rows: dict[str, dict],
    pending_rows: dict[str, dict],
) -> None:
    all_rows: dict[str, dict] = {}
    for source, rows in (("主注册表", main_rows), ("pending", pending_rows)):
        for ref, row in rows.items():
            if ref in all_rows:
                report.warn(f"{ref}: 同时出现在主注册表与 registry_pending.md（合并后请删除 pending 行）")
                continue
            all_rows[ref] = row

    local_dirs = {**papers, **books}
    for ref, row in all_rows.items():
        loc = row.get("location") or ""
        if not loc:
            continue
        target = resolve_local_dir(loc)
        if target is None:
            continue
        if not target.exists():
            report.err(f"{ref}: 注册表登记了本地路径但目录不存在：{loc}")
            continue
        if ref in books and target != books[ref]:
            report.warn(f"{ref}: 注册表路径 {loc} 与目录名 {books[ref].name} 不一致")
        if ref in papers and target != papers[ref]:
            report.warn(f"{ref}: 注册表路径 {loc} 与目录名 {papers[ref].name} 不一致")

    for ref, d in local_dirs.items():
        if ref not in all_rows:
            report.err(f"{ref}: 目录 {d.name} 未在注册表或 registry_pending.md 登记")

    literature_types = ("论文", "专著", "会议论文", "期刊论文", "预印本", "技术报告", "理论参考")
    for ref, row in all_rows.items():
        if ref in local_dirs:
            continue
        ctype = row.get("type") or ""
        if not any(t in ctype for t in literature_types):
            continue  # 工具/第三方库/协议文档等不在本脚本校验范围
        status = row.get("status") or ""
        if "仅题录" in status or "待补" in status or "计划" in status:
            continue  # 约定内状态：只有题录、无本地 PDF
        if status:
            report.warn(f"{ref}: 状态标注为「{status}」但本地无对应原始文件（若非仅题录/待补，请修正状态）")
        else:
            report.warn(f"{ref}: 未解析到状态列（表格列数不足），请检查该行格式")


# ---------------------------------------------------------------- catalog

def build_catalog(papers_entries: list[dict], books_entries: list[dict]) -> dict:
    entries = sorted(papers_entries + books_entries, key=lambda e: int(e["ref"].split("-")[1]))
    counts = {
        "papers_total": len(papers_entries),
        "papers_ready": sum(1 for e in papers_entries if e["status"] == "已就位"),
        "books_total": len(books_entries),
        "books_ready": sum(1 for e in books_entries if e["status"] == "已入库"),
    }
    return {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds"),
        "generated_by": "docs/08-参考资料/scripts/check-refs.py --update-catalog",
        "root": "docs/08-参考资料",
        "conventions": {
            "papers": "papers/REF-<nnn>-<第一作者>-<年份>-<关键词>/{paper.pdf,paper.md,notes.md,quality_report.json}",
            "books": "books/REF-<nnn>-<作者>-<年份>-<关键词>/{book.pdf,book.json,<vol>/full.md}",
            "md_chars": "paper.md（书籍为 book.md）字符数；缺失为 0",
            "notes_ok": "notes.md 含 题录/摘要/项目关系/可借鉴/不适用 全部必备节（书籍以 book.json 视为元数据笔记）",
            "status": "已就位 = PDF + Markdown 齐备；仅题录条目不在本 catalog（见 index/registry_pending.md）",
        },
        "counts": counts,
        "entries": entries,
    }


def check_catalog(report: Report, catalog: dict | None, papers_entries: list[dict], books_entries: list[dict]) -> None:
    if not CATALOG.exists():
        report.warn("index/catalog.json 不存在（可用 --update-catalog 生成）")
        return
    try:
        data = json.loads(read_text(CATALOG))
    except json.JSONDecodeError as exc:
        report.err(f"index/catalog.json 不是合法 JSON（{exc}）")
        return
    if catalog is not None and data.get("counts") != catalog.get("counts"):
        report.warn(f"catalog.json 的 counts 与当前磁盘不一致：{data.get('counts')} vs {catalog.get('counts')}（建议 --update-catalog）")
    entries = {e["ref"]: e for e in data.get("entries") or []}
    expected = {e["ref"]: e for e in papers_entries + books_entries}
    for ref in sorted(set(entries) - set(expected)):
        report.warn(f"{ref}: catalog.json 有记录但磁盘已无对应目录（建议 --update-catalog）")
    for ref, exp in sorted(expected.items()):
        got = entries.get(ref)
        if not got:
            report.err(f"{ref}: catalog.json 缺少该条目（建议 --update-catalog）")
            continue
        if exp.get("pdf_sha256") and got.get("pdf_sha256") and got["pdf_sha256"] != exp["pdf_sha256"]:
            report.err(f"{ref}: catalog.json 的 pdf_sha256 与磁盘不一致")
        if isinstance(got.get("md_chars"), int) and got["md_chars"] != exp.get("md_chars"):
            report.warn(f"{ref}: catalog.json 的 md_chars 与磁盘不一致（{got['md_chars']} vs {exp.get('md_chars')}）")


# ---------------------------------------------------------------- main

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="REF 文献库一致性校验 / catalog 生成")
    ap.add_argument("--update-catalog", action="store_true", help="重新生成 index/catalog.json 后再校验")
    ap.add_argument("--json", action="store_true", help="以 JSON 输出结果")
    ap.add_argument("--quiet", action="store_true", help="只输出结论与错误")
    args = ap.parse_args(argv)

    report = Report()
    sha_cache: dict[str, str] = {}

    papers = discover_papers(report)
    books = discover_books(report)

    main_rows = parse_registry_table(MAIN_REGISTRY)
    pending_rows = parse_registry_table(PENDING_REGISTRY)
    if not MAIN_REGISTRY.exists():
        report.err(f"主注册表不存在：{MAIN_REGISTRY.name}")
    if not MAIN_REGISTRY.exists() and not pending_rows:
        report.err("既无主注册表也无 registry_pending.md，无法校验登记关系")

    papers_entries = [check_paper(ref, d, report, sha_cache) for ref, d in sorted(papers.items())]
    books_entries = [check_book(ref, d, report, sha_cache) for ref, d in sorted(books.items())]

    cross_check_registry(report, papers, books, main_rows, pending_rows)

    catalog = build_catalog(papers_entries, books_entries)
    if args.update_catalog:
        INDEX_DIR.mkdir(parents=True, exist_ok=True)
        CATALOG.write_text(json.dumps(catalog, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    check_catalog(report, catalog, papers_entries, books_entries)

    report.info = {
        "papers_dirs": len(papers),
        "books_dirs": len(books),
        "main_registry_rows": len(main_rows),
        "pending_registry_rows": len(pending_rows),
        "papers_ready": catalog["counts"]["papers_ready"],
        "books_ready": catalog["counts"]["books_ready"],
        "catalog_written": bool(args.update_catalog),
    }

    if args.json:
        print(json.dumps({
            "ok": not report.errors,
            "info": report.info,
            "errors": report.errors,
            "warnings": report.warnings,
        }, ensure_ascii=False, indent=2))
        return 0 if not report.errors else 1

    if not args.quiet:
        print("=" * 72)
        print("REF 文献库校验  |  根目录：docs/08-参考资料")
        print("=" * 72)
        print(f"扫描目录：papers {len(papers)} 个、books {len(books)} 个")
        print(f"注册表行：主注册表 {len(main_rows)} 行、registry_pending {len(pending_rows)} 行")
        print(f"就位统计：论文 {catalog['counts']['papers_ready']}/{len(papers)}，"
              f"书籍 {catalog['counts']['books_ready']}/{len(books)}")
        print("-" * 72)
        if papers_entries or books_entries:
            print(f"{'REF':8s} {'类型':4s} {'状态':10s} {'md_chars':>9s} {'notes':5s} {'quality':7s} 路径")
            for e in papers_entries + books_entries:
                print(f"{e['ref']:8s} {e['type']:4s} {e['status']:10s} {e['md_chars']:>9d} "
                      f"{'ok' if e['notes_ok'] else 'MISS':5s} {str(e['quality'] or '-'):7s} {e['path']}")
        print("-" * 72)
        if args.update_catalog:
            print(f"已写入 catalog：{CATALOG.relative_to(REF_ROOT.parent.parent)}"
                  if CATALOG.is_relative_to(REF_ROOT.parent.parent) else f"已写入 catalog：{CATALOG}")

    if report.errors:
        print(f"\n[ERROR] 共 {len(report.errors)} 条：")
        for msg in report.errors:
            print(f"  - {msg}")
    if report.warnings:
        print(f"\n[WARN] 共 {len(report.warnings)} 条（不阻断）：")
        for msg in report.warnings:
            print(f"  - {msg}")

    ok = not report.errors
    print("\n" + ("结果：PASS（0 error）" if ok else f"结果：FAIL（{len(report.errors)} error）")
          + f"；warning {len(report.warnings)} 条")
    return 0 if ok else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:  # noqa: BLE001
        print(f"脚本异常：{exc}", file=sys.stderr)
        raise
