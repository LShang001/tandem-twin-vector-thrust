# coax-model-2026-10-31 — 共轴 rigorous 动力学模型与 RK4 统一积分

## 目的

按用户要求（2026-10-31）：

1. 共轴机型从 L1 集总 plant 升级为可选 L2 物理模型（`--coax-plant-model rigorous`）；
2. 所有仿真积分统一为经典四阶 Runge-Kutta，不再用欧拉积分。

模型与公式见 `docs/04-数学建模/MOD-005-共轴双桨VTVL动力学模型.md`；
SIL 开关说明见 `TandemVec_FCS/test_host/fw_sil/README.md` §11.2。

## 证据文件

| 文件 | 内容 |
|---|---|
| `acceptance_legacy_rk4.log` | legacy L1 口径 + RK4，3 profiles × 5 seeds |
| `acceptance_rigorous_rk4.log` | rigorous L2 口径 + RK4，同 profile/seed |
| `cvc_host_rk4.log` | `test_coaxial_vtv_control`（T8 6DOF 闭环改 RK4） |
| `cdyn_host_rk4.log` | `test_coaxial_dynamics_model` 36 断言 |

## 结果

| 项 | legacy L1 | rigorous L2 |
|---|---:|---:|
| 3×5 验收 | 8/9（唯一 FAIL `mag_nominal seed1` max_pitch **22.777°** > 20°） | 9/9 |
| 8×5（Euler 版归档） | 39/40 | 40/40 |
| 宿主 `T8` 6DOF（RK4） | `roll=0.001 pitch=0.001 pos=(-1.11,1.98,0.10)` | 同左（测试走同一控制链） |
| `cdyn` | 36/36 | 36/36 |

积分口径变化：legacy 的 `mag_nominal seed1` 由 Euler 基线 22.764° → RK4 22.777°
（+0.013°），失败集与其它场景结论不变；rigorous 仍全通过。

## 重要口径（不得误读）

- rigorous 下 `mag_nominal seed1` 俯仰峰值显著降低（Euler 版 22.764°→3.427°）**不是
  控制性能提升**：rigorous 推力 `T∝u²`，固件 `T_est∝u`，悬停时 `T_est` 高估约 30%，
  同一 α 指令在物理 plant 上产生的力矩更小、激励更弱；hold 反而略差。
- `legacy` 仍是验收基线；`rigorous` 用于研究/灵敏度/台架标定前检查。
- 惯量、力臂、kT/kQ/Jp 仍为 MODEL-DEFAULT 量级值，不是共轴实机辨识值。

## 复现

```bash
# 构建（需先 cd TandemVec_FCS）
cd TandemVec_FCS
BIN_DIR=T:/sil_rk4 bash test_host/fw_sil/build_and_run_coaxial.sh --build-only

# legacy / rigorous 3×5
py -3.12 TandemVec_FCS/test_host/fw_sil/run_real_acceptance.py \
  --bin T:/sil_rk4/fw_coax.exe \
  --profiles mag_clean,mag_nominal,nomag_degraded --seeds 1,2,3
py -3.12 TandemVec_FCS/test_host/fw_sil/run_real_acceptance.py \
  --bin T:/sil_rk4/fw_coax.exe \
  --profiles mag_clean,mag_nominal,nomag_degraded --seeds 1,2,3 \
  --append-args "--coax-plant-model rigorous"
```

二进制 `T:/sil_rk4/fw_coax.exe`，sha256 前缀 `e9531bf025ff48d8`；
构建源 = HEAD `c255d7a` + 本次未提交的模型/RK4 改动。
