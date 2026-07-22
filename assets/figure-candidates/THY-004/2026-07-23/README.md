# THY-004 图 1 非 AI 候选归档（2026-07-23）

本目录保存图 1 在切换到 AI/Web 混合方案前的 Web 主体版本，便于回溯视觉取舍。

| 文件 | 说明 | SHA-256 | 结论 |
|---|---|---|---|
| `configuration-web-hybrid-source.html` | Web 三维主体 + HTML 标注源码快照 | `8D87F83381552F8372E62CD5C8DEE37DD1327AFCD1FB8D34F0E09ECC89BF3CEB` | 工程来源最直接，但原版缩印字号偏小，尾发引导线与 CG 锚点不够明确 |
| `configuration-web-hybrid-before-ai.png` | 上述源码对应的 2400 x 1500 PNG | `2F3D2EAF4D20B6462C07D5FCFA5990010A240B18373624CB9E5726055E4E33AE` | 作为被替换候选保留；不再作为 LaTeX 当前图 1 |

本快照之后曾短暂使用 `01-ai-selected-f.png` 作为视觉底图，但最终图 1 已改为
`docs/03-理论推导/THY-004/fig/01-web-orthogonal-deflection.png` 的 Web 程序化几何，
并通过 HTML/SVG 确定性标注锁定前后发、正交摆轴、非零摆角、旋向与控制通道。
工程语义仍以 `models/aircraft-model.json` 和 `simulations/vector-thrust-lab/src/core/` 为准。
