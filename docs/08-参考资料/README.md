# 文献资料管理

> 外部参考文献（论文、技术报告、标准文档）的存放、转换与阅读工作流。
> 文献条目在 `../参考资料注册表.md` 中登记，本目录存储原始 PDF 和转换后的 Markdown。

## 目录结构

```
papers/
└── REF-<nnn>-<第一作者>-<年份>-<标题缩写>/
    ├── paper.pdf              # 原始 PDF（权威副本）
    ├── paper.md               # MinerU 转换的 Markdown
    ├── notes.md               # 阅读分析笔记
    ├── manifest.json          # MinerU 转换元数据（自动生成）
    └── quality_report.json    # 质量审计报告（自动生成）

scripts/
    └── convert-paper.ps1      # 辅助脚本：PDF → Markdown 一键转换
```

## 工作流程

### 第一步：获取 PDF

```powershell
# 从 arxiv 下载
curl -L "https://arxiv.org/pdf/<arxiv-id>.pdf" -o paper.pdf

# 或从 DOI 下载（注意：期刊可能重定向到登录页，用 file 命令验证真伪）
curl -L "https://doi.org/<DOI>" -o paper.pdf
```

### 第二步：验证 PDF 真伪

```powershell
file paper.pdf
# 必须输出 "PDF document, version 1.x" —— 显示 "HTML document" 说明是登录页重定向，需换下载方式
```

### 第三步：创建文献目录

按命名规范 `REF-<编号>-<第一作者>-<年份>-<关键词>` 创建子目录，放入 PDF：

```powershell
New-Item -ItemType Directory -Path "papers/REF-104-Kumar-2020-quaternion-thrust-vectoring"
Move-Item paper.pdf "papers/REF-104-Kumar-2020-quaternion-thrust-vectoring/"
```

### 第四步：MinerU 转换

学术/工程论文（含公式、表格）→ 使用 v4 + vlm：

```powershell
# 方式一：辅助脚本（推荐）
.\scripts\convert-paper.ps1 -PaperPath "papers/REF-104-Kumar-2020-quaternion-thrust-vectoring"

# 方式二：手动调用 MinerU
python "C:\Users\12631\.agents\skills\mineru-pdf-markdown\scripts\mineru_convert.py" `
    "papers/REF-xxx/paper.pdf" `
    -o "papers/REF-xxx" `
    --language en `
    --model-version vlm
```

- 需先配置 `MINERU_TOKEN` 环境变量（从 https://mineru.net 申请）
- 短小论文（≤10MB、≤20 页、无复杂公式）可用 v1 Agent API（免 Token）
- 超过 200 页的文档需分卷转换

### 第五步：质量审计

```powershell
python "C:\Users\12631\.agents\skills\mineru-pdf-markdown\scripts\mineru_quality.py" `
    "papers/REF-xxx"
```

检查 `quality_report.json`：
- MD 字符数是否合理（不应为空）
- 标题数、表格行数、公式块数
- 缺失图片数

### 第六步：阅读 & 写笔记

在 `notes.md` 中记录：

```markdown
# 文献笔记：<标题>

- **编号**：REF-xxx
- **作者**：<作者列表>
- **年份**：<年份>
- **DOI**：<DOI 或 arxiv ID>
- **关联项目模块**：<如：§3 推进建模 / §5 控制分配 / §7 SAS 设计>

## 核心内容

<3-5 条要点>

## 与项目的关系

<如何引用、哪些公式/方法可借鉴、差异点>

## 关键摘录

<重要段落或公式>
```

### 第七步：更新注册表

在 `../参考资料注册表.md` 中按顺序递增 REF 编号登记：

```markdown
| REF-104 | <标题> | 论文 | `papers/REF-104-.../` | <用途> | 已就位 |
```

---

## 命名规范

| 要素 | 格式 | 示例 |
|------|------|------|
| 目录名 | `REF-<nnn>-<第一作者>-<年份>-<3-5个关键词>` | `REF-104-Kumar-2020-quaternion-thrust-vectoring` |
| PDF 文件 | 统一命名为 `paper.pdf` | — |
| 转换产物 | `paper.md`（MinerU 产出 `full.md` 后重命名） | — |
| 笔记 | 统一命名为 `notes.md` | — |

## 已就位的文献

| REF | 目录 | 状态 |
|-----|------|------|
| *(暂无——等待添加第一篇文献)* | | |

---

## 工具依赖

| 工具 | 用途 | 安装 |
|------|------|------|
| MinerU v4 API | PDF → Markdown（公式/表格） | `MINERU_TOKEN` 环境变量 |
| Python 3.8+ | 运行 MinerU 脚本 | 已有（`py -3.12`） |
| `file` (Git Bash / WSL) | PDF 真伪验证 | Git for Windows 自带 |
