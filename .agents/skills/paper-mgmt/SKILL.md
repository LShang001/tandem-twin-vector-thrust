---
name: paper-mgmt
description: 文献资料管理全流程：搜索→下载→MinerU转换→质量审计→分析笔记→注册表更新。处理论文PDF、arxiv、DOI。触发：加论文、查文献、读paper、文献管理、下论文。
compatibility: Requires Python 3.8+, PowerShell, MinerU API (v1 free or v4 with token)
metadata:
  author: LShang
  version: 1.0.0
---

# 文献资料管理全流程

覆盖外部参考文献（论文、技术报告、标准文档）的搜索、获取、转换、阅读分析、注册登记全流程。

## 触发场景

- "加一篇论文"、"下载这篇 paper"、"把这个 PDF 转成 Markdown"
- "查一下 XXX 方向的文献"、"有没有关于 XXX 的论文"
- "读一下这篇论文"、"分析这篇文献"
- 任何涉及外部参考文献的操作

## 目录结构

```
docs/08-参考资料/
├── 参考资料注册表.md          ← REF 编号登记（只增不改已有）
├── README.md                   ← 管理工作流文档
├── papers/                     ← 所有文献
│   └── REF-<nnn>-<第一作者>-<年份>-<3-5个关键词>/
│       ├── paper.pdf           ← 原始 PDF（权威副本）
│       ├── paper.md            ← MinerU 转换的 Markdown
│       ├── notes.md            ← 阅读分析笔记
│       └── quality_report.json ← 质量审计（可选，MinerU 自动生成）
└── scripts/
    └── convert-paper.ps1       ← 辅助脚本
```

## 工作流（严格按顺序）

### 步骤 1：搜索论文

用 `search_semantic`（Semantic Scholar）按关键词搜索。也可按标题精确匹配、按作者搜索。

返回结果时展示：标题、作者、年份、引用数、摘要前两句。让用户确认要下载哪篇。

### 步骤 2：获取 PDF

按来源分：

| 来源 | 方法 |
|------|------|
| **arxiv** | `download_arxiv` 拿 PDF URL → 用 `Invoke-WebRequest` 下载 |
| **DOI（开放获取）** | `download_by_doi` 拿 PDF URL → 下载 |
| **Semantic Scholar（开放获取）** | `get_semantic_paper_detail` 查 `openAccessPdf.url` |
| **用户直接给 PDF 路径** | 跳过下载 |

下载命令模板：
```powershell
Invoke-WebRequest -Uri "<pdf-url>" -OutFile "<目标路径>/paper.pdf"
```

### 步骤 3：建目录 + 验证 PDF

**目录命名**：`REF-<编号>-<第一作者姓氏>-<年份>-<3-5个关键词用连字符连接>`

示例：`REF-104-Kumar-2020-quaternion-thrust-vectoring`

编号规则：查 `docs/08-参考资料/参考资料注册表.md`，取最大 REF 编号 + 1。

**验证 PDF 真伪**（必须执行）：
```powershell
$header = Get-Content -Path "<路径>/paper.pdf" -Encoding Byte -TotalCount 5
$magic = [System.Text.Encoding]::ASCII.GetString($header)
# 必须输出 "%PDF-" 开头，如果是 "HTML" 说明是登录页重定向，需换下载方式
```

假 PDF 处理：用 `search_semantic_paper_match` 找 arxiv 版本，或尝试出版社 CDN 地址。不要反复重试同一 URL。

### 步骤 4：MinerU 转换

**选型决策**：
- 学术论文（含公式/表格）→ v4 + `--model-version vlm`（需 `MINERU_TOKEN`）
- ≤10MB、≤20 页、纯文本为主 → v1 Agent API（免 Token）
- 超过 200 页 → 分卷转换

**执行**（两种方式）：

方式一 —— 用项目辅助脚本：
```powershell
cd docs/08-参考资料
.\scripts\convert-paper.ps1 -PaperPath "papers/REF-xxx"
```

方式二 —— 直接调 MinerU：
```powershell
# v1（免 Token）
$MINERU = "$env:USERPROFILE\.agents\skills\mineru-pdf-markdown\scripts"
py -3.12 "$MINERU\agent_parse.py" "<pdf路径>" -o "<输出目录>" --language en

# v4（需 Token）
py -3.12 "$MINERU\mineru_convert.py" "<pdf路径>" -o "<输出目录>" --language en --model-version vlm
```

转换完成后：`full.md` → `paper.md`（重命名）。

### 步骤 5：质量审计

```powershell
$MINERU = "$env:USERPROFILE\.agents\skills\mineru-pdf-markdown\scripts"
py -3.12 "$MINERU\mineru_quality.py" "<输出目录>"
```

检查要点：
- `paper.md` 不为空，字符数合理（通常 >5KB）
- 公式渲染正确（`$$` 块完整）
- 标题层级清晰
- 表格保留（如有）

如有严重质量问题（大量乱码、公式全部丢失），换 v4+vlm 重试。

### 步骤 6：阅读 & 写笔记

通读 `paper.md`，在 `notes.md` 中按以下模板记录：

```markdown
# 文献笔记：<标题>

- **编号**：REF-xxx
- **作者**：<作者列表>
- **年份**：<年份>
- **来源**：<会议/期刊名>，<页数>
- **arxiv/DOI**：<链接>
- **关联项目模块**：<如：§3 推力矢量映射 / §5 控制分配 / §7 SAS 设计>

## 核心内容

<3-5 条要点，每条 1-2 句>

## 与项目的关系

<对照表或对比段落，覆盖：平台差异、执行器差异、控制分配方法差异、可借鉴/不适用之处>

| 维度 | 本文 | 本项目 |
|------|------|--------|
| ... | ... | ... |

### 可借鉴的方法
### 关键差异（不适用之处）

## 关键摘录

<2-3 段重要原文或公式，附简要说明>

## 引用建议

<在项目文档/论文中的引用场景>
```

**笔记原则**：
- 必须包含"与项目的关系"对照——这是最重要的节
- 不要复述论文全部内容，只提炼与项目相关的
- 公式可以从 paper.md 中复制（MinerU 已转成 LaTeX）

### 步骤 7：更新注册表

在 `docs/08-参考资料/参考资料注册表.md` 末尾（`## 说明` 之前）追加一行：

```markdown
| REF-<nnn> | <标题> (<作者>, <会议/期刊> <年份>) | 论文 | `papers/REF-<nnn>-<目录名>/` | <与项目模块的关联，20 字以内> | 已就位 |
```

不修改已有行，只追加。

---

## 踩坑记录

这些是从实际使用中积累的——出现两次就加一条：

- **DOI 下载陷阱**：`curl -L https://doi.org/xxx` 经常被重定向到期刊登录页（HTML），不是真 PDF。始终用 `Get-Content -Encoding Byte -TotalCount 5` 验证 `%PDF-` 文件头。假 PDF → 换 arxiv 或出版社 CDN 地址
- **MinerU v1 公式丢失**：v1 Agent API 用 pipeline 轻量模型，学术论文的复杂公式可能丢失或乱码。看到大量 `egin{array}...` 断裂 → 换 v4 + `--model-version vlm`
- **MinerU 图片不下载**：转换产物的 `images/` 目录已在 `.gitignore` 中。如需保留图片，手动从 MinerU 输出目录复制
- **REF 编号断裂**：忘了更新注册表会导致下一个论文 REF 编号重复。必须在步骤 7 完成后用 `grep REF- papers/` 确认编号无冲突
- **大 PDF 炸 Git**：>1MB 的 PDF 提交后会让 `git clone` 越来越慢。超过 5MB 的论文考虑 Git LFS 或只存元数据

## 注意事项

- **版权**：仅下载开放获取论文。付费墙论文只登记元数据（DOI、标题、摘要），PDF 不放入仓库
- **MinerU Token**：v4 需要 `MINERU_TOKEN` 环境变量。如果没有，降级到 v1 或用 `web_fetch` 抓 arxiv HTML 摘要
- **笔记质量**：每篇笔记的核心价值在于"与项目的关系"节——没有这节的笔记不值
- **注册表同步**：加论文后必须更新注册表，否则 REF 编号会断裂
