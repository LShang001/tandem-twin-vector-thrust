---
name: agents-doc-opt
description: 审查优化本项目 AGENTS.md/CLAUDE.md——6 异味检测、精简外移、修复双文档漂移与跨项目污染。触发：AGENTS.md 臃肿、指令不生效、CLAUDE.md 与 AGENTS.md 不同步、审查/优化 agent 文档、新 Agent 入场.
license: MIT
compatibility: Requires Python 3.12+（可选 validate_skill.py）、git、PowerShell
metadata:
  author: LShang
  version: 1.0.0
---

# Agent 指令文件优化（本项目）

审查并精简本项目两套 AGENTS.md（根仿真项目 + TandemVec_FCS 固件子项目），消除配置异味、双文档漂移和跨项目污染。只做文档优化，不评估技能质量（那是 skill-creator-plus 的职责）。

## When to Use

- "审查 AGENTS.md"、"优化 agent 文档"、"指令不生效"
- "CLAUDE.md 和 AGENTS.md 不一致"、"精简 CLAUDE.md"
- 新 Agent 入场前检查文档，或季度例行审计

## Prerequisites

- 双项目结构认知：**根 AGENTS.md**（仿真，106 行达标，一般不动）+ **TandemVec_FCS/AGENTS.md**（固件，重点优化对象）
- 引用文件 `.agents/docs/hardware-reference.md` 已存在（串口/硬件事实表，勿重复内联回 AGENTS.md）
- 校验脚本（可选）：`python C:\Users\12631\.agents\skills\learn-skill\scripts\validate_skill.py`

## How to Run

当用户触发"审查/优化 agent 文档"类请求时：先按 Quick Reference 三条命令建立现状基线，再按 Procedure 第 1-7 步执行。纯检查请求只需完成第 1-3、6 步并汇报，不做任何修改。

## Quick Reference

- 行数统计：`(Get-Content <file>).Count`
- 残留化石检查：`Select-String -Path TandemVec_FCS\AGENTS.md -Pattern 'vtvl_electricdualrotor|parameters\.mjs|pio_projects'`
- 异味标准：Lint 泄漏 / 上下文膨胀 / 技能泄漏 / 指令冲突 / 初始化化石 / 盲引用
- 长度闸门：≤80 优秀 · 81-120 审查 · 121-160 精简 · **>160 必须砍**（遵循率崩溃线）

## Procedure

1. **侦察现状**：读取根 `AGENTS.md`、根 `CLAUDE.md`、`TandemVec_FCS/AGENTS.md`、`TandemVec_FCS/CLAUDE.md`，统计行数；检查是否有 `.cursorrules`/`copilot-instructions.md` 等其他指令文件
2. **逐行评分**：每行问"删掉它 Agent 会做不同的/错误的决定吗？"——不会则删。linter 已强制的规则、文件列举/`platformio.ini`/`package.json` 能自行发现的都删
3. **6 异味检测**：
   - Lint 泄漏（linter 已强制规则）→ 删
   - 上下文膨胀（>120 行、长表格）→ 表格外移到 `.agents/docs/hardware-reference.md`，AGENTS.md 留一句带触发条件的引用
   - 技能泄漏（低频操作流程）→ 移入 Skill
   - 指令冲突（互相矛盾规则）→ 删冲突方
   - 初始化化石（旧仓库路径、跨项目引用）→ 验证并清除
   - 盲引用（引用文件无触发条件）→ 补"何时/为何读"
4. **提方案**：展示保留/删除/新增/外移摘要。**结构变更（CLAUDE.md 改导入、大段删除）先经用户确认再动**；明显冗余直接执行
5. **实施**：
   - 表格外移目标：`TandemVec_FCS/.agents/docs/hardware-reference.md`（文件头注明触发条件）
   - `TandemVec_FCS/CLAUDE.md` 必须是单行 `@AGENTS.md`——**绝不维护第二份独立文档**
   - 抢救 CLAUDE.md 独有且有文件实证的信息（glob 验证文件存在）并入 AGENTS.md，再删 CLAUDE.md
   - 安全硬约束**必须保留**：坐标钉死约定、执行器映射、硬性代码边界、安全关键修改规则、实机分阶段验证、COM 口独占规则
6. **验证**：
   - 残留检查（见快速参考）输出为空
   - 目标行数 ≤160（安全关键项目可接受 130-140）
   - 引用路径全部 glob 实证存在（`TandemVec_ControlAllocation.h`、`电路拓扑参考.md`、`run_all.sh` 等）
   - `git diff --stat` 确认删除量符合预期；`git status --short` 检查新文件未被忽略
7. **提交**：中文 commit，尾部 `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`；PowerShell 用 here-string 变量传消息（`@'...'@` 必须独立成行）

## Pitfalls

- **跨项目污染是本项目最高频错误**：`parameters.mjs` 属于根仿真项目（sync-params.py 生成），写进 TandemVec_FCS 文档就是污染；同类错误还包括把根项目命令抄进固件文档
- **初始化化石**：旧仓库路径 `d:/pio_projects/vtvl_electricdualrotor_fcs/`（迁移前的仓库名）曾残留在固件 AGENTS.md 串口清理命令里
- CLAUDE.md 与 AGENTS.md 内容重复必漂移——旧版模块表曾比 AGENTS.md 落后一个版本
- 安全关键项目别砍过头：通用行为规范可删，但坐标约定/边界/实机安全等"删了 Agent 就会犯错"的必须留
- 命令动词不要裸写 shell 命令（grep/cat/sed），用 agent 自己的读取/搜索工具
- 校验脚本退出码非 0 就是没完成，模型自检报告不算数

## Verification

```powershell
(Get-Content TandemVec_FCS\AGENTS.md).Count  # 期望 ≤160
Select-String -Path TandemVec_FCS\AGENTS.md -Pattern 'vtvl_electricdualrotor|parameters\.mjs|pio_projects'  # 期望无输出
Get-Content TandemVec_FCS\CLAUDE.md  # 期望仅 @AGENTS.md
```

以上三条全过 = 优化达标。技能本体质量检查可另跑 `python C:\Users\12631\.agents\skills\learn-skill\scripts\validate_skill.py <技能目录> --max-desc-length 1024`。
