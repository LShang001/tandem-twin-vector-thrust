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

完整角色定义与协作工作流见 `docs/00-项目治理/GOV-003-多Agent协作规范.md`。

| Name | 类型 | 职责 |
|------|------|------|
| Codex | 主实现 | 代码生成、重构、仿真、测试 |
| Hermes | 文档理论 | 技术文档、理论推导、LaTeX |
| Reasonix | 分析审计 | 交叉验证、错误检测、一致性 |
| Kuhn | 研究探索 | 文献调研、技术评估、外部检索 |

## 交接记录命名

`<YYYYMMDD>-<HHMMSS>-<from>-to-<to>.json`
