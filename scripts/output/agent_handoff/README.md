# Agent 交接协议

> 多 Agent 并行研发的交接规范。每个 Agent 完成工作后在此写入交接记录，下一个 Agent 读此记录获取上下文。

## 交接记录格式

```json
{
  "from": "<agent-name>",
  "to": "<agent-name | broadcast>",
  "timestamp": "<ISO 8601>",
  "task_id": "<任务简述 slug>",
  "summary": "<1-3 句完成情况>",
  "artifacts": ["<修改的文件路径>"],
  "blockers": ["<阻塞项>"],
  "next_steps": ["<建议后续步骤>"]
}
```

## Agent 注册表

| Name | 类型 | 职责 | 定义文件 |
|------|------|------|----------|
| Codex | 主实现 | 代码生成、重构、仿真、测试 | `.claude/agents/codex.md` |
| Hermes | 文档理论 | 技术文档、理论推导、LaTeX | `.claude/agents/hermes.md` |
| Reasonix | 分析审计 | 交叉验证、错误检测、一致性 | `.claude/agents/reasonix.md` |
| Kuhn | 研究探索 | 文献调研、技术评估、外部检索 | `.claude/agents/kuhn.md` |

## 工作流模式

### 模式 1：文档-代码交叉验证

```
Hermes → (写文档/推导公式) → Reasonix → (对照代码审计) → Hermes → (修复)
```

### 模式 2：研究-设计-实现

```
Kuhn → (调研技术方案) → Codex → (原型实现) → Reasonix → (审查) → Codex → (修复)
```

### 模式 3：并行多维度审计

```
Reasonix(正确性) + Reasonix(安全性) + Reasonix(一致性) → Hermes/Codex → (汇总修复)
```

## 交接目录

交接记录 JSON 文件存放在 `scripts/output/agent_handoff/` 下，命名规则：
`<YYYYMMDD>-<HHMMSS>-<from>-to-<to>.json`
