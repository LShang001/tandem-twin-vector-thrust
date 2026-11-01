# REF-175 · u-blox NEO-F10N 数据手册（UBX-23002117 R05）

- 转换方式：MinerU 4.0.12（standard tier，`mineru parse paper.pdf --tier standard`）
- 来源：https://content.u-blox.com/sites/default/files/documents/NEO-F10N_DataSheet_UBX-23002117.pdf
- 器件：u-blox F10 双频（L1/L5）标准精度 GNSS 模组，22 页。
- 与本项目关系：仿真 `test_imu_nav_perf.cpp` 的 F10N GNSS 档参数依据；
  固件 UBX 解析对 F10N 的 PVT+EOE 场景见 `UBX协议要点-2026-11-01.md`。

## 关键性能（多星座默认 GPS+GAL+BDS）

| 参数 | 值 | 备注 |
| --- | --- | --- |
| 最大导航更新率 | 10 Hz | 单星座（GPS/BDS）可 20 Hz |
| 位置精度（CEP 50%） | 1 m | 单星座 1.5 m |
| 速度精度 | 0.05 m/s | 50% @30 m/s |
| 动态航向精度 | 0.3° | |
| 时间脉冲 | 30 ns RMS / 60 ns 99% | 1PPS 可配 0.25 Hz–10 MHz |
| 动态/高度/速度限 | 4 g / 80 km / 500 m/s | |
| TTFF | 冷 28 s / 热 2 s | |
| 灵敏度 | 跟踪 −167 dBm | 冷启 −148 dBm |

## 仿真口径换算

- CEP → 1σ：`σ ≈ CEP / 1.177`；
- 多星座 10 Hz：`pos_std≈0.85 m`、`vel_std=0.05 m/s`；
- 单星座 20 Hz：`pos_std≈1.27 m`（CEP 1.5）、`vel_std=0.05`；
- 接收机延迟：手册不提供；项目调研见 `GNSS接收机延迟调研-2026-11-01.md`（典型 10–30 ms，UBX 时间标签可补偿）。

## 与 M8N 对比

| | M8N（UBX-13003221，10 Hz） | F10N（10 Hz） |
| --- | --- | --- |
| 星座 | GPS/GLO/GAL/BDS 单频 | GPS/GAL/BDS 双频 L1/L5 |
| 位置 CEP | ~2.5 m | 1 m |
| 速度 | 0.05 m/s | 0.05 m/s |
| 更新率 | 10 Hz | 10 Hz（单星座 20 Hz） |

F10N 的 L5 抗多径是城市/低空场景的主要收益；本仿真只建模位置/速度噪声，
不建模多径时变（后续如需城市峡谷场景可加 Markov 多径模型）。
