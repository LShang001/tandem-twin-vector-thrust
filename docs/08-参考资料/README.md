# 文献资料管理

> 外部参考文献（专著、论文、技术报告、标准文档）的**单一事实源**：原始 PDF、MinerU
> 转换文本、阅读笔记、注册表与索引都集中在本目录。
> 文献条目在 `../参考资料注册表.md` 中登记；本目录只展开存放/转换/阅读细节。
> 本体系自 2026-10-21 起按“书籍 + 论文 + 主题笔记 + 机读目录 + 自动校验”组织。

---

## 1. 目录结构

```
docs/08-参考资料/
├── 参考资料注册表.md            # REF 编号唯一登记（只追加，不改已有行）
├── README.md                    # 本文件
├── books/                       # 专著/教材（本地研究副本，PDF 不入 git）
│   └── REF-<nnn>-<作者>-<年份>-<关键词>/
│       ├── book.pdf             # 原始 PDF（本地；.gitignore 不跟踪）
│       ├── book_pXXX-XXX.pdf    # >200 页时拆卷（MinerU 上限）
│       ├── book.json            # 题录/ISBN/页数/SHA256/分卷/质量状态
│       ├── toc_probe/full.md    # 目录/前置页 OCR
│       ├── part1/full.md        # 分卷转换（按页范围）
│       ├── part2/full.md
│       ├── book.md              # （后续）按顺序合并的全文
│       ├── chapters/            # （后续）按章节切分，便于精读
│       └── notes.md             # 书籍级阅读笔记/章节导览
├── papers/                      # 论文/技术报告
│   └── REF-<nnn>-<作者>-<年份>-<关键词>/
│       ├── paper.pdf
│       ├── paper.md             # MinerU full.md 重命名
│       ├── notes.md             # 元数据 + 摘要 + 与本项目关系
│       ├── quality_report.json
│       └── manifest.json
├── notes/                       # 跨文献的主题精读/综合笔记（团队产出）
│   ├── 00-文献地图与结论.md
│   ├── 01-捷联惯导理论.md
│   ├── 02-卡尔曼滤波理论.md
│   ├── 03-组合导航与鲁棒性.md
│   ├── 04-航向与磁辅助.md
│   └── 05-仿真验证.md
├── index/                       # 机读/检索层
│   ├── bibliography.md          # 题录地图（按主题分组、标本地状态）
│   ├── catalog.json             # 由脚本扫描生成的机读目录
│   ├── TEAM-BRIEF.md            # 研究组基线/分工/写作规范
│   └── registry_pending.md      # 待合并进主注册表的行
└── scripts/
    ├── convert-paper.ps1        # 旧辅助脚本（保留兼容）
    └── check-refs.py            # 注册表↔目录↔PDF↔Markdown 一致性校验
```

### 书籍与论文的差异

| 维度 | 论文 | 书籍 |
|---|---|---|
| PDF 文件名 | `paper.pdf` | `book.pdf` |
| 转换产物 | `paper.md` | `book.md`（合并）/`chapters/*.md` |
| 页数限制 | 一般 <200 页 | >200 页必须拆卷（MinerU v4 硬上限 200 页） |
| 阅读方式 | 全文 | 按章节 + 页码定位 |
| git | `papers/**/*.pdf` 例外跟踪 | `books/**/*.pdf` 不跟踪（体积/版权），只跟踪 `book.json`/`book.md`/笔记 |

---

## 2. 工作流（论文）

### 第一步：获取 PDF

```bash
# arXiv
curl -L "https://arxiv.org/pdf/<arxiv-id>.pdf" -o paper.pdf
# DOI（开放获取；付费墙不要下载）
curl -L "https://doi.org/<DOI>" -o paper.pdf
```

**必须验证**：`head -c 5 paper.pdf` 输出 `%PDF-`；若是 `<!DOC`/`<html`，说明拿到登录页，
换 arXiv/作者主页/出版社开放 PDF。不要反复重试同一 URL。

### 第二步：建目录

`papers/REF-<nnn>-<第一作者姓>-<年份>-<3-5 个关键词>/`，编号取注册表最大 REF+1；
不得跳号/复用（REF-106 是历史预留，不占用）。

### 第三步：MinerU 转换

```bash
SKILL="C:/Users/12631/.agents/skills/mineru-pdf-markdown/scripts"
py -3.12 "$SKILL/mineru_convert.py" "papers/REF-xxx/paper.pdf" \
  -o "papers/REF-xxx" --model-version vlm --language en
# 扫描件加 --ocr；≤200 页；超过则先用 pypdf 拆卷
```

- Token 已配置（`mineru_convert.py --doctor` 可自检）。
- 输出 `full.md` 重命名为 `paper.md`；保留 `quality_report.json`/`manifest.json`。
- 质量审计：`py -3.12 "$SKILL/mineru_quality.py" "papers/REF-xxx"`。
- 公式/表格多、质量 `warn` 时优先看 `quality_report.json` 的 `latex_block_count` 与
  `html_table_count`；不要因为 `warn` 就重复转换。

### 第四步：写 notes.md

模板见 §4。每篇必须有“与项目的关系”，否则不算完成。

### 第五步：登记

在 `../参考资料注册表.md` 的 `## 说明` 前追加一行；同时更新 `index/catalog.json`
（由 `check-refs.py --update-catalog` 或手工）。**主注册表只由 Lead/管理员改**；
团队成员写 `index/registry_pending.md`，由 Lead 合并。

---

## 3. 工作流（书籍）

1. 复制 PDF 到 `books/REF-xxx/book.pdf`；计算 SHA256，写 `book.json`。
2. 页数 >200 时用 `pypdf` 拆成 ≤200 页的分卷（建议 140-190 页/卷），记录每卷原页范围。
3. 先转目录/前置页（`--pages 1-24`）确认版本/ISBN/章节页码，再转正文分卷。
4. 扫描件必须 `--ocr --language ch`；VLM 模型对中文公式/上下标更稳。
5. 合并：按分卷顺序拼 `full.md` → `book.md`；按章节标题切 `chapters/`。记录
   “印刷页 → PDF 页”偏移（如 REF-120：+8）。
6. 写 `notes.md`：版本/ISBN/目录、与项目相关的章节、页码映射、阅读优先级。
7. 书籍 PDF 默认不纳入 git；`book.json`、`book.md`、`chapters/`、`notes.md` 纳入。

---

## 4. notes.md 模板

```markdown
# 文献笔记：<标题>

- **编号**：REF-xxx
- **作者/版本**：<作者；版次；出版社/期刊；年份>
- **DOI/ISBN**：<...>
- **本地**：`paper.pdf` / `paper.md`（字符数）/ `quality_report.json` 状态
- **关联项目模块**：<§3 推进建模 / §5 控制分配 / EKF 导航 ...>

## 核心内容（3-5 条）

## 与项目的关系

| 维度 | 本文 | 本项目 |
|---|---|---|
| 平台/传感器 | | |
| 方法/假设 | | |
| 可借鉴 | | |
| 不适用 | | |

## 关键摘录（2-3 段，带页码/公式号）

## 引用建议
```

主题笔记（`notes/0X-*.md`）用 `index/TEAM-BRIEF.md` §5 的格式，必须含“理论要点→
代码对照→升级项→不建议项→开放问题”。

---

## 5. 编号、状态与引用

- REF 编号：`REF-001~004` 工具；`REF-101~103` 理论预留；`REF-104+` 论文/专著；
  `REF-120/121` 两本秦永元教材；新条目从当前最大号 +1。
- 状态：`已就位`（PDF+MD+notes 齐）、`已入库`（PDF/元数据在，转换/笔记进行中）、
  `仅题录`（付费墙/未获取）、`待补充`。
- 正文引用：`[REF-120 §9.3 p.267]`、`[REF-148]`；代码引用：`ekf_15_state.h:2261
  PropagateStateFromIncrements()`。
- 文献结论 vs 项目证据 vs 推断必须分开写；不得把文献平台结论直接外推到本机。

---

## 6. 目录/校验工具

```bash
# 校验注册表↔目录↔PDF 头↔Markdown↔notes 必备节
py -3.12 docs/08-参考资料/scripts/check-refs.py

# 重新生成机读目录（如脚本支持）
py -3.12 docs/08-参考资料/scripts/check-refs.py --update-catalog
```

`check-refs.py` 的验收标准：0 error；warning 只允许“仅题录/转换中”等显式状态。
`catalog.json` 字段：`ref/title/authors/year/type/path/status/pdf_sha256/md_chars/notes_ok`。

---

## 7. 当前状态（2026-10-21）

| 类别 | 内容 | 状态 |
|---|---|---|
| 专著 | REF-120 秦永元《惯性导航（第三版）》 | ✅ 全文 388 页；`ocr-errata.md`：前置页/章标题/高频混淆字/关键公式已对照 PDF 修订；正文其余页待分批复核 |
| 专著 | REF-121 秦永元等《卡尔曼滤波与组合导航原理（第3版）》 | ✅ 全文 408 页 OCR；`ocr-errata.md`：p84/255/291 抽查+全库字符扫描，未发现系统性语义错误 |
| 论文 | 既有 8 篇控制类文献（REF-104~119） | 已就位 |
| 论文 | 导航/EKF 方向开放获取论文（REF-136/138/140/144/145/154/163/169） | ✅ 8 篇已入库（PDF+MD+notes） |
| 主题笔记 | `notes/00`–`07` | ✅ 全部完成（00 地图/01 SINS/02 KF/03 组合导航/04 航向磁/05 仿真/06 对准与可观测性/07 一致性与 FDI） |
| 主报告 | `TandemVec-EKF-Report/导航算法升级评估-专业文献调研-2026-10.md` | ✅ 已出（含证据冲突/负面结果） |
| 索引 | `index/bibliography.md` / `index/TEAM-BRIEF.md` / `index/catalog.json` / `index/registry_pending.md` | ✅ 已建立 |
| 校验 | `scripts/check-refs.py` | ✅ PASS（0 error / 15 warning，2026-10-21） |
| 仿真 | `sim/opt/upgrade_candidates.py` + JSON | ✅ A/B/C/P0 四套件（5 种子）；含 H 索引修正说明 |

---

## 8. 版权与仓库体积

- 书籍 PDF 为用户提供的个人研究副本，仅本地使用，**不提交到公开仓库/不再分发**；
  `books/**/*.pdf` 被 `.gitignore` 忽略。
- 论文只下载开放获取版本；付费墙只登记题录。
- >5 MB 的 PDF 不轻易入库；确需入库时用 Git LFS 或只保留转换文本/题录。
- 转换后的 Markdown 可能含原文大段内容，默认仅项目内部研究使用。

---

## 9. 与既有文献的关系

本目录同时服务于两条线：
1. **控制/执行器线**（REF-104~119）：推力矢量、INDI、控制分配；
2. **导航/估计线**（REF-120 起）：SINS、KF/ESKF、组合导航、鲁棒/容错、磁/光流辅助。

两条线共用注册表与转换工具，但主题笔记分开，避免“控制论文的结论”污染导航判断。
