# sim-data 数据台账（三机型快速 MC）

> 全部为 **L1 降阶 MC**：不是固件 SIL，不是 HIL/台架/实飞。
> 完整 `samples.csv`（约 110 MB/套）不入库，可由 `run_mega.py` 复现；
> 入库的是 `summary/analysis/1% 样本/最坏案例/casewise`。

## 1 数据集

| 目录 | 内容 | 状态 |
|---|---|---|
| `mega-nominal-150k-2026-10-24/` | 名义范围 50k/机型，无故障 | 有效 |
| `mega-300k-2026-10-24/` | 宽范围 fixed 300k（早期） | **TandemVec 故障桶无效**，无故障桶有效 |
| `mega-fixed-300k-paired-2026-10-24/` | 与 osc 成对的 fixed 300k（修正前） | **TandemVec 故障桶无效**，无故障桶有效 |
| `mega-osc-300k-paired-2026-10-24/` | 默认 osc 300k + 逐 case 转移（修正前） | **TandemVec 故障桶无效**，无故障桶有效 |
| `mega-fixed-300k-v2-2026-10-24/` | **修正后** fixed 300k | 权威基线 |
| `mega-osc-default-300k-v2-2026-10-24/` | **修正后** 默认 osc 300k | 权威 A/B |
| `mega-osc-tuned-300k-v2-2026-10-24/` | **修正后** tuned osc 300k（0.15 s/4/0.85/1.01/0.45） | 权威 A/B |
| `boundary-grid-2026-10-24/` | TandemVec 延迟×姿态/速率增益×故障确定性网格 | 修正后 |

## 2 thrust_loss 建模勘误（2026-10-24）

修正前 `fast_mc.py` 把 `thrust_loss` 乘在**油门**上再过推力曲线：

- TandemVec 是二次曲线 `T = Tmax·u²`，`u×0.5` 后实际推力变成 **0.25 倍**，
  把“50% 推力损失”误算成 75%；
- 共轴/液体是线性油门，乘油门与乘推力等价，因此结果不受影响。

现在改为乘在**推力**上（推力效率因子）。影响：

- 无故障 case 的 `thrust_loss=1.0`，结果逐位不变；无故障结论全部沿用。
- TandemVec 故障桶必须用 v2 数据；旧目录仅保留作历史，不得再引用其故障数字。
- 修正后 TandemVec 在 50% 推力损失下仍可能落到地面（T/W=4→2），
  失败主要来自高延迟/高增益下的姿态发散；osc 能减少姿态失败，但不能恢复高度。
  共轴/液体 50% 损失后 T/W<1，仍以坠落为主。

## 3 复现

```bash
cd simulations/three_vehicle_mc
py -3.12 run_mega.py --cases 100000 --variant fixed --seed 20261024 --out <dir>
py -3.12 run_mega.py --cases 100000 --variant osc --seed 20261024 --out <dir>
py -3.12 run_mega.py --cases 100000 --variant osc --osc-window 0.15 \
  --osc-threshold 4 --osc-decay 0.85 --osc-recovery 1.01 --osc-floor 0.45 \
  --seed 20261024 --out <dir>
py -3.12 analyze_mega.py --dir <dir> --compare-dir <reference>
py -3.12 compare_casewise.py --baseline <fixed-dir> --candidate <osc-dir>
py -3.12 run_boundary_grid.py --out sim-data/boundary-grid-2026-10-24
```

`compare_casewise.py` 会先校验 17 个参数列逐 case 一致，防止把不同样本差异
当成策略收益。
