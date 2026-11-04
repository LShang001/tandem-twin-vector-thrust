# GCS MAVLink 双向接入与测试（2026-11-04）

## 1. 架构

- 新增 `GCS/server/mavlink_service.py`：基于 **pymavlink dialects.v20.common** 的
  服务，不直接开串口；`feed(bytes)` 解帧并维护 snapshot/params。
- GCS 连接时选择 `anocom | mavlink`；`GCS.mav` 持有服务，`mav_pump()` 把
  `drain_tx()` 的字节写回 `SerialLink`。
- 下行解析：HEARTBEAT/SYS_STATUS/ATTITUDE/GLOBAL_POSITION_INT/VFR_HUD/
  RAW_IMU/RC_CHANNELS_RAW/GPS_RAW_INT/HOME_POSITION/AUTOPILOT_VERSION/
  PARAM_VALUE/STATUSTEXT/COMMAND_ACK/MISSION_ACK/MISSION_COUNT。
- 上行：心跳、PARAM_REQUEST_LIST/READ/SET、COMMAND_LONG、PING、TIMESYNC、
  MISSION_REQUEST_LIST。
- 字段映射：MAVLink 的 `v_north_mps/rel_alt_m/gps_fix` 等同步为 GCS 现有
  `vel_n_ms/rel_h_m/gps_fix` 等，保证仪表与 `CsvRecorder` 复用。

## 2. 关键坑位

1. **必须用 v20 dialect**：`pymavlink.mavutil.mavlink` 默认是
   `dialects.v10.ardupilotmega`，发 v1 帧固件不认、表现为命令无 ACK/参数0。
   改用 `from pymavlink.dialects.v20 import common as mavlink2`，首字节 `0xFD`。
2. **TX 泵不能忘**：`MAVLink` 只把帧写入 sink；GCS 必须在连接/收帧/命令后调用
   `mav_pump()`，否则看起来“发了命令但没有 ACK”。
3. **MAVLink 分支提前 return 会绕过记录/WS**：`_on_telemetry_rx` 的 MAVLink
   分支必须补 `recorder.write` 和 `telemetry` 广播，否则 CSV 只有表头、前端空白。
4. **GCS 字段名兼容**：MAVLink 原生单位/命名与 AnoCom 不同，必须做别名映射，
   否则记录列为空。
5. **pymavlink 版本签名差异**：`command_ack_send`/`timesync_send` 参数随版本不同；
   GCS 代码按 v2.4.50 实际签名调用，测试覆盖。

## 3. CLI / 前端

```bash
py -3.12 server/cli.py --port COM9 --protocol mavlink mav status
py -3.12 server/cli.py --port COM9 --protocol mavlink mav params
py -3.12 server/cli.py --port COM9 --protocol mavlink mav param-set att_roll.kp 4.25
py -3.12 server/cli.py --port COM9 --protocol mavlink mav cmd --command 176
py -3.12 server/cli.py --port COM9 --protocol mavlink mav ping
py -3.12 server/cli.py --port COM9 --protocol mavlink mav timesync
py -3.12 server/cli.py --port COM9 --protocol mavlink mav mission
py -3.12 server/cli.py --port COM9 --protocol mavlink record start -o out.csv -d 10
```

前端连接栏新增协议选择；WS 支持 `mav_param_list/mav_param_set/mav_cmd/mav_status`。

## 4. 验证证据（2026-11-04 实机）

- 真实 MAVLink 状态：姿态/位置/卫星/Home/版本解析正常。
- 参数：PARAM_VALUE **148/148**；`param-set` 同名值确认 `confirmed=4.25`。
- 命令：SET_MODE(176) → `COMMAND_ACK(176,3)`（安全拒绝）。
- PING/TIMESYNC：均收到回复帧；MISSION_REQUEST_LIST → MISSION_COUNT=0。
- 记录：MAVLink模式10s CSV **316行**，`roll/heading/vel_n_ms/rel_h_m/rc1/
  gps_fix/unlocked` 均有值。
- 自动化：`GCS/tests` **52 passed**（服务、TX泵、GCS集成、字段映射）。

## 5. 边界

- 固件明确拒绝 MAVLink 解锁/改模式（RC CH5/CH6 硬判定），这是安全红线。
- 不假装支持航线上传：MISSION_COUNT=0 / MISSION_ACK UNSUPPORTED。
- 未做动态/室外空口长测；当前只验证静态 USB/数传链路。
- `pymavlink` 已加入 `requirements.txt`，无依赖时 AnoCom 路径仍可运行。
