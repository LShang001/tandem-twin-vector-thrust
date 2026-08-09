# tools/ — 调试与分析脚本索引

> 2026-08-10 归拢：串口调试脚本已并入 `GCS/server/cli.py`（统一入口），本目录只保留
> 离线分析工具与历史验证脚本。**要连串口调设备，先看 cli.py 帮助**：
> `py -3.12 TandemVec_FCS/GCS/server/cli.py --help`（在 `TandemVec_FCS/GCS` 目录执行）。

## 统一调试入口（GCS/server/cli.py，已取代散脚本）

| 旧散脚本 | cli.py 子命令 | 说明 |
|----------|---------------|------|
| scan_baud.py / probe_2m.py / probe_com10.py | `baudscan --port COM10` | 波特率扫描（0x00 帧头间隔规律性评分） |
| diag_stick.py | `stick -t 10` | 打杆诊断：模式/解锁 + 8 通道 + 控制输出实时 |
| sniff_rc.py | `sniff rc` | 0x06 模式 + 0x20 8 通道 |
| sniff_att.py | `sniff att` | 0x03 欧拉 + 0x21 控制输出 |
| sniff_axes.py | `sniff axes` | 0x01 IMU + 0x03 欧拉（轴方向测试） |
| sniff_euler.py | `sniff euler` | 欧拉角统计（均值/标准差/范围，欧拉奇异检验） |
| sniff_raw.py / parse_anocom.py | `sniff raw` | 原始字节统计（高频字节/帧头候选） |
| flash_export.py | `flash export` | 黑匣子导出解析（cli.py 版含分片重试，更可靠） |
| — | `param set/verify` | 写后读回验证（0x00 确认帧在 2M 间歇丢帧下不可靠，读回为准） |

## 保留的独立工具（离线分析，不依赖串口）

| 脚本 | 用途 |
|------|------|
| `plot_blackbox.py <csv> [-o dir]` | 黑匣子 CSV 可视化（列名自适应 S 帧通道表；依赖 matplotlib） |
| `audit_math.py` | 数学/约定一致性审计 |
| `dps_profile_parser.js` / `dps_profile_runner.js` | DBG 任务调度性能分析（`tasks` 命令输出） |
| `nmea-host-test/` | GNSS 双协议解析 host 回归测试（g++ 66 断言） |

## 历史验证脚本（verify_*.py — 一次性数学/符号验证，跑完即归档）

> 均为飞行控制语义验证脚本，依赖 `simulations/high-fidelity-analysis`。已完成的验证
> 结论沉淀在 AGENTS.md 踩坑记录与 docs/，脚本保留备查，**非调试工具**。

| 脚本 | 验证内容 |
|------|----------|
| verify_allocation_sign.py | B_true 分配器符号（My→dt、Mx→dw） |
| verify_axis_map.py / verify_mix_axes.py | mix 层轴置换与符号（以实机直通行为为锚点） |
| verify_control_sign.py / verify_torque_sign.py | 控制律符号 |
| verify_floor_consistency.py / verify_w0_floor.py | w0 下限一致性 |
| verify_frame_map.py | 帧映射 |
| verify_slope_direct.py | 斜率直测 |
| verify_yaw_authority.py / verify_yaw_gain_schedule.py / verify_yaw_vs_archive.py | yaw 通道权限/增益调度/与存档对比 |
