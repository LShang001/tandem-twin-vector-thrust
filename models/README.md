# models — 模型参数单一事实源

`aircraft-model.json` 是整机六自由度仿真全部物理参数的**唯一权威来源**。任何参数修改只改这里，然后运行：

```bash
python tools/sync-params.py            # 重新生成仿真参数文件
python tools/sync-params.py --check    # 校验仿真参数文件与 JSON 无漂移
```

生成物：`simulations/vector-thrust-lab/src/core/parameters.mjs`（**禁止手工编辑**）。

## 约定

- **单位**：内部统一 SI（m、kg、s、N、rad）；`unit` 字段为人读标注
- **角度**：一律弧度（如 `dMax = 0.4363 rad = 25°`）
- **坐标系**：NED 惯性系；机体系 x 前 / y 右 / z 下（详见 `docs/04-数学建模/MOD-002`）

## 参数来源状态（source / status）

| 状态 | 含义 |
|---|---|
| `MODEL-DEFAULT` | 模型默认值，无任何外部依据（当前全部参数） |
| `ASSUMED` | 有明确理由的工程假设，理由记录在假设日志 |
| `ESTIMATED` | 由计算/估算得出，计算过程可追溯 |
| `DATASHEET` | 来自供应商数据表，需登记参考资料编号 |
| `BENCH-TESTED` | 台架试验标定，需关联试验记录 |
| `FLIGHT-IDENTIFIED` | 飞行数据系统辨识，需关联数据处理记录 |

`confidence`（low/medium/high）为主观置信度，与来源状态互补。参数升级来源状态时，必须同步更新本文件与 `docs/registers/参数数据手册.md`。
