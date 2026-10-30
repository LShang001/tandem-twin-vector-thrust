# 共轴 L1 plant 可切保真度证据（coax-plant-2026-10-30）

> 归属：team task-5（`coax-sil-plant`）。对象：`TandemVec_FCS/test_host/fw_sil/fw_sil_env_main.cpp`
> 的共轴 VTVL L1 plant。**默认开关数值逐位保持原行为**；全部保真度增强都是 CLI
> 开关（默认关闭/零值），不改变既有验收口径与固件源码。

- 源码指纹：`fw_sil_env_main.cpp` 3115 行，SHA-256
  `6b1c756cb41d2bfc1794aabc40d8c4d58a8baeadf846909d6446933ed63e4a26`
- 构建（工作区含 Lead 的 `MadgwickAHRS.cpp` 链接补丁，属未提交改动）：
  ```bash
  cd TandemVec_FCS
  BIN_DIR=/c/Users/12631/AppData/Local/Temp/tv_coax_sil_bin \
    bash test_host/fw_sil/build_and_run_coaxial.sh --build-only
  ```
- 验收（3 profiles × 5 seeds = 15 场景，40 s，真实固件 `BFS_COAXIAL_VTVL`）：
  ```bash
  py -3.12 TandemVec_FCS/test_host/fw_sil/run_real_acceptance.py \
    --bin /c/Users/12631/AppData/Local/Temp/tv_coax_sil_bin/fw_coax.exe \
    --profiles mag_clean,mag_nominal,nomag_degraded --seeds 1,2,3,4,5 \
    --append-args=--coax-realistic
  ```
  （`--append-args` 的值以 `=` 传入，否则 argparse 会把 `--coax-realistic`
  当成选项名而报错。）

## 1. 开关语义、默认值与 realistic 取值

| CLI 开关 | 物理含义 | 默认 | realistic 预设 | 取值依据 / L1 假设 |
| --- | --- | --- | --- | --- |
| `--coax-motor-tau <s>` | 两电机推力一阶跟踪时间常数（转速/推力建立滞后） | 0.28 s（沿用既有 L1 值；`0`=理想瞬时） | 0.28 s | 小桨电机+电调在 10–30 Hz 带宽量级，0.28 s 为原 plant 既有 L1 假设，量级与 100–300 g 级多旋翼动力组相符；本任务只把它变成可覆盖参数，不重新标定 |
| `--coax-servo-rate <deg/s>` | 舵机输出角速率上限（° /s，`0`=无限速率） | 300 °/s（沿用既有 L1 值） | 300 °/s | 9 g 数字舵机典型 0.1–0.15 s/60°，对应 400–600 °/s 空载；带矢量机构负载取 300 °/s 保守值，沿用既有假设 |
| `--coax-ground-mu <μ>` | 落地后水平库仑摩擦系数（地面水平速度消减） | 0（不启用） | 0.5 | 橡胶起落架/水泥地干摩擦 μ≈0.5–0.7，取 0.5；L1 假设：法向力≈重力、动/静摩擦不分、μ·g 线性消减到零且不反冲 |
| `--coax-realistic` | 预设：一次性写入 τ=0.28、300 °/s、μ=0.5 | 关闭 | — | 组合成"接近实机低速 VTVL"的可复现臂；按 argv 顺序生效，后续显式开关可覆盖 |
| `--coax-servojam roll\|pitch` | 在卡死时刻锁存该舵机**当时实际角**并强制保持（齿轮咬死/连杆卡滞） | none | — | 离散故障注入，不标定概率；卡死时刻缺省复用 `--fault-at`（8 s），可用 `--coax-servojam-at` 指定 |
| `--coax-servojam-at <s>` | 卡死发生时刻 | = `--fault-at` | — | — |
| `--coax-motor-loss1/2 <r>` | 电机 1/2 推力**持久**保留比例（`1`=无损失，`0`=完全失效） | 1.0 | — | 与 `--fault 3/5`（按子步重置的瞬时故障）相互独立；用于长时间推力衰减/单发失效对照 |

补充事实（避免误解）：
- **执行器动态原本就在**：`runRealNavLoop` 的 2 kHz 内环自 `fd6e692`（2026-10-24）
  起就含电机 τ=0.28 s、舵机一阶 τ=0.05 s + 300 °/s 速率限幅；控制帧
  （200 Hz）自那时起已做 0.1% PWM 量化（±15° / 1000 计数 ≈ 0.03°/计数）。
  因此 `--coax-motor-tau 0.28` / `--coax-servo-rate 300` 与默认**逐位一致**，
  这两个开关的默认值就是既有 L1 行为。
- 所以 `--coax-realistic` 相对默认基线**真正新增的物理只有地面水平摩擦**
  （μ=0.5）；执行器两项只是把既有硬编码值显式化。
- **气动阻尼按用户指示不在本 L1 plant 范围内**（VTVL 低速段气动阻尼相对
  控制力矩很小，且现有 `--wind` 直接力语义保持不变）。

## 2. 默认开关等价性（逐位）

`default_equivalence.txt`：15/15 场景的 stdout 非 JSON 行逐字节一致；
`SIL_REAL_JSON` 的全部既有字段数值完全一致；唯一差异是新增的 6 个
`coax_*` 追踪字段（`coax_motor_tau_s / coax_servo_rate_dps / coax_ground_mu /
coax_servojam / coax_loss1 / coax_loss2`）。默认开关验收仍为
**14 PASS + 1 FAIL**，失败项与改动前完全相同：

| 场景 | 结果 | 失败原因 |
| --- | --- | --- |
| mag_nominal seed1 | FAIL（既有） | `max_pitch=22.108 > 20.0`（alt=9.10, hold_end=0.96, roll_max=8.21） |
| 其余 14 场景 | PASS | — |

可复现性：同一源码重新构建二进制后再复跑两套 15 场景验收，
`acceptance_default.txt` / `acceptance_realistic.txt` 与复跑日志逐字节相同
（确定性 plant + 固定 seed）。默认验收日志亦与改动前基线
`baseline_acceptance.txt` 逐字节相同。

## 3. realistic 预设：15 场景结果（全 PASS）

`realistic_15.json`（含逐项 gate 判定与 metrics）、`realistic_15_raw.jsonl`
（原始 SIL_REAL_JSON 行）、`acceptance_realistic.txt`（验收脚本输出）。

| profile | seed | final_alt [m] | hold_end [m] | pos_rms [m] | max_roll [°] | max_pitch [°] | 结果 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| mag_clean | 1 | 9.857 | 0.884 | 0.89 | 1.194 | 4.705 | PASS |
| mag_clean | 2 | 9.840 | 0.268 | 0.48 | 0.992 | 2.032 | PASS |
| mag_clean | 3 | 9.937 | 0.479 | 0.64 | 0.943 | 2.862 | PASS |
| mag_clean | 4 | 9.870 | 0.295 | 0.76 | 0.751 | 2.005 | PASS |
| mag_clean | 5 | 9.928 | 0.365 | 0.59 | 0.957 | 2.187 | PASS |
| mag_nominal | 1 | 9.092 | 0.780 | 0.55 | 8.942 | 19.915 | PASS |
| mag_nominal | 2 | 9.071 | 0.297 | 1.02 | 8.186 | 5.386 | PASS |
| mag_nominal | 3 | 9.033 | 0.855 | 0.77 | 7.502 | 5.321 | PASS |
| mag_nominal | 4 | 9.117 | 0.519 | 0.44 | 12.649 | 9.362 | PASS |
| mag_nominal | 5 | 9.100 | 0.337 | 0.70 | 10.143 | 5.003 | PASS |
| nomag_degraded | 1 | 9.868 | 117.20（不判） | 0.45 | 0.962 | 1.728 | PASS |
| nomag_degraded | 2 | 9.875 | 109.30（不判） | 0.55 | 0.938 | 1.529 | PASS |
| nomag_degraded | 3 | 9.913 | 113.27（不判） | 0.53 | 1.059 | 1.532 | PASS |
| nomag_degraded | 4 | 9.833 | 116.73（不判） | 0.25 | 0.932 | 1.660 | PASS |
| nomag_degraded | 5 | 9.923 | 123.20（不判） | 0.38 | 1.015 | 1.720 | PASS |

**失败项：无（15/15 PASS）。** 但必须如实说明：`mag_nominal seed1` 在默认开关下
以 `max_pitch=22.108` 越 20° 门限失败，realistic 下变为 19.915。这个差异**完全来自
地面摩擦对起飞前地面段的微小扰动**（已验证：只加 `--coax-ground-mu 0.5` 即得到
同样的 22.108→19.915），属于混沌敏感性，**不是保真度提升带来的鲁棒性改善**；
不应据此放宽默认门限或删掉 seed1。该 seed 的余量只有 0.085°，属于临界通过。

## 4. 单开关冒烟（同二进制 A/B，mag_clean seed1 40 s）

| 臂 | max_pitch [°] | hold_end [m] | 观察 |
| --- | --- | --- | --- |
| default | 5.770 | 0.873 | 基线 |
| `--coax-motor-tau 2.0` | 21.389 | 4.315 | 电机滞后显著放大俯仰超调（同量级于 τ 增大） |
| `--coax-motor-tau 0` | 6.063 | 0.873 | 理想电机，闭环几乎无差（真实固件鲁棒） |
| `--coax-servo-rate 60` | 5.770 | 0.873 | 该工况指令斜率未触及限幅（舵机一阶 τ=50 ms 主导） |
| `--coax-servo-rate 5`（mag_nominal s1） | 89.166 | 708.70 | 速率严重受限后失控坠地，证明限幅确实生效 |
| `--coax-ground-mu 0.5` | 4.705 | 0.884 | 仅起飞前地面段扰动，飞行段无影响 |
| `--coax-servojam roll --coax-servojam-at 15` | 5.770（max_roll=179.985） | 478.06 | 滚转舵机卡死→横滚失控坠地（land_t=19.67 s） |
| `--coax-servojam pitch --coax-servojam-at 15` | 89.912 | 440.78 | 俯仰舵机卡死→俯仰失控坠地（land_t=18.26 s） |
| `--coax-motor-loss1 0.8` | 3.225 | 0.881 | 单发 20% 推力损失仍可控（差速力矩补偿） |
| `--fault 5 --fault-at 20 --loss 0.1`，μ=0 → 0.5 | — | — | 迫降后水平滑行终点 msE：−2.90 m → −0.40 m（μ 减小滑行 2.5 m） |

非法值均被拒绝并返回 2（`--coax-servojam bad`、`--coax-ground-mu 9`、
`--coax-motor-loss1 1.5`、`--coax-motor-tau -1`、`--coax-servo-rate -5`、
`--coax-servojam-at -2`）。`--coax-realistic` 与显式
`--coax-motor-tau 0.28 --coax-servo-rate 300 --coax-ground-mu 0.5` 输出完全一致；
`--coax-realistic` 之后显式开关可覆盖（已验证 `--coax-ground-mu 0`）。

## 5. 已知局限 / 未覆盖

- **无气动阻尼/风相对速度**（按用户指示取消）：仍沿用 `--wind` 直接力语义。
- 地面接触仍是点质量近似：无地面法向反力/接触力矩/姿态支撑，摩擦只在
  `pos[2] >= 0`（on_ground）时对水平速度做库仑消减，不打滑、不滚转、
  不模拟起落架刚度；μ 不影响飞行段。
- `--coax-servojam` 为确定性离散事件（锁存实际角），不建模卡滞前的摩擦/迟滞；
  与 `--fault 4` 同时启用时后者优先覆盖 roll（故障注入优先级更高）。
- `--coax-motor-loss1/2` 为持久比例，不含温升/电压跌落动态；`--fault 3/5`
  的瞬时故障仍在每个 2 kHz 子步重置覆盖。
- 执行器 τ/速率/量化沿用既有 L1 假设，本轮未做频响/台架标定；
  `--coax-servo-rate` 在正常定高工况因舵机一阶 τ=50 ms 主导而通常不饱和，
  只在剧烈机动（大指令误差）下起作用。
- 本证据是 SIL（真实固件+近似 6DOF plant），**不是台架/实机证据**；
  plant 质量/惯量/力臂/差速力矩仍为 L1 假设。
