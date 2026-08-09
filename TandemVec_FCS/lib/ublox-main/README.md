
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

![Bolder Flight Systems Logo](img/logo-words_75.png) &nbsp; &nbsp; ![Arduino Logo](img/arduino_logo_75.png)

# Ubx

本库通过 UBX 协议与 uBlox GNSS 接收机进行通信。本库兼容 Arduino 和 CMake 构建系统。

* [许可证 (License)](LICENSE.md)
* [变更日志 (Changelog)](CHANGELOG.md)
* [贡献指南 (Contributing guide)](CONTRIBUTING.md)

# 描述 (Description)

uBlox 生产标准和高精度的 GPS 接收机。这些接收机具有高灵敏度、极短的捕获时间以及小巧的外形尺寸。UBX 是一种 uBlox 二进制格式，用于高效地从接收机检索数据。

# 双协议支持 (Dual Protocol, 2026-08-09 本地扩展)

本库除 UBX 外还内置 NMEA 解析（minmea，MIT 许可，见 `LICENSE.minmea`），
支持在 `kUbx` / `kNmea` / `kAuto` 三种协议模式下独立切换：

```C++
ubx.SetProtocol(bfs::Ubx::GpsProtocol::kAuto);   // UBX 优先，失效后 NMEA 兜底
ubx.SwitchProtocol(bfs::Ubx::GpsProtocol::kNmea, 38400);  // 切换并重设串口波特率
```

- **kUbx**（默认）：纯 UBX，行为与旧版一致
- **kNmea**：纯 NMEA——解析 GGA/RMC/GSA，快照合成 `UbxEpoch` 入同一队列，
  导航层（`PopEpoch`）无感
- **kAuto**：双流并行，UBX epoch 优先；UBX 失效超过
  `SetNmeaUbxBackoffMs()`（默认 300ms）后自动用 NMEA 快照兜底合成

NMEA 合成语义（供上层理解精度边界）：
- 数据来源 GGA（位置/高程/HDOP/fix）+ RMC（对地速度/航向）+ GSA（PDOP/VDOP，
  GN 组合解优先）；`pvt_tow_ms == eoe_tow_ms` = UTC 时间-of-day 伪 tow
- 秒键去重 + 800ms 合成节流：GGA/RMC 双句 1Hz 流 → 1Hz epoch，位置最新、
  速度滞后 ≤1s（NMEA 流固有限制）；`spd_acc_mps` 取 3.0 使缺失的垂直速度
  只作弱约束
- `horz_acc_m` = HDOP × 2.5（C/A 码 UERE 保守估计）；`pvt_pdop` 来自 GSA
  （0 = 未提供，动态权重不缩放）
- 已知限制：RMC 尾部 `,,A`（u-blox 真实输出）会使官方 `minmea_parse_rmc`
  失败——本库自实现解析（`minmea_scan` 截断到 date 字段）

回归测试：`TandemVec_FCS/tools/nmea-host-test`（g++，66 项断言，覆盖三模式/
分片/串扰/混合流/跨日/缺省字段/坏校验/溢出恢复）。

# 安装 (Installation)

## Arduino

使用 Arduino 库管理器（Library Manager）安装本库，或将其克隆到您的 `Arduino/libraries` 文件夹中。本库通过以下方式引用：

```C++
#include "ubx.h"
```

Arduino 可执行示例位于：*examples/arduino/ublox_example/ublox_example.ino*。
Teensy 3.x、4.x 和 LC 设备已用于 Arduino 环境下的测试，本库也应兼容其他 Arduino 设备。

## CMake

本库使用 CMake 进行构建，并导出名为 *ubx* 的库目标。头文件引用方式如下：

```C++
#include "ubx.h"
```

本库也可以使用 CMake 的惯例进行独立编译：创建一个 *build* 目录，然后在该目录中执行以下命令：

```bash
cmake .. -DMCU=MK66FX1M0
make
```

这将构建库以及一个名为 *ublox_example* 的示例可执行文件。示例可执行文件的源文件位于 *examples/cmake/ublox_example.cc*。请注意，*cmake* 命令包含一个定义，用于指定代码编译的目标微控制器。这是正确配置代码、CPU 频率以及编译/链接器选项所必需的。可用的 MCU 包括：

* MK20DX128
* MK20DX256
* MK64FX512
* MK66FX1M0
* MKL26Z64
* IMXRT1062_T40
* IMXRT1062_T41
* IMXRT1062_MMOD

已知这些 MCU 配合 Teensy 产品中使用的相同封装可以正常工作。只要仅仅是封装变更，切换封装通常也能正常工作。

示例目标创建了一个使用 UBX 协议与 GNSS 接收机通信的可执行文件。每个目标还有一个 `*_hex` 用于创建上传到微控制器的 hex 文件，以及一个 `*_upload` 用于使用 [Teensy CLI Uploader](https://www.pjrc.com/teensy/loader_cli.html) 烧录 Teensy。有关设置构建环境的说明，请参阅我们的 [build-tools 仓库](https://github.com/bolderflight/build-tools)。

# 命名空间 (Namespace)

本库位于命名空间 *bfs* 中。

# 用法 (Usage)

## 接收机设置 (Receiver Setup)

本库解析来自以下消息的数据：

* UBX-NAV-DOP
* UBX-NAV-EOE
* UBX-NAV-POSECEF
* UBX-NAV-PVT
* UBX-NAV-VELECEF
* UBX-NAV-TIMEGPS

应使用 [u-center 软件](https://www.u-blox.com/en/product/u-center) 启用这些消息。

如果高精度定位数据可用，应启用以下消息，本库将使用它们：

* UBX-NAV-HPPOSECEF
* UBX-NAV-HPPOSLLH

如果相对定位数据可用（例如来自静止或移动基站），应启用以下消息，本库将使用它：

* UBX-NAV-RELPOSNED

最后，如果您连接到固定基线（fixed-baseline）并进行 Survey-in（定点勘测），应启用以下消息，本库将使用它来提供有关 Survey-in 状态的信息：

* UBX-NAV-SVIN

# Ubx 类详解

## 方法 (Methods)

**Ubx()** 默认构造函数，需要调用 `Config` 方法来设置串口。

**Ubx(HardwareSerial&ast; bus)** 创建一个 Ubx 对象。此构造函数用于串口通信接口，需将指向串口总线的指针传递给构造函数。

```C++
bfs::Ubx ubx(&Serial1);
```

**void Config(HardwareSerial&ast; bus)** 设置用于通信的串口。如果使用了默认构造函数，则必须调用此方法。

**bool Begin(const int32_t baud)** 建立与 GNSS 接收机的通信。成功接收到数据返回 true，否则返回 false。

```C++
bool status = ubx.Begin(921600);
```

### 数据采集 (Data Collection)

以下方法用于读取和解析串口数据。收到完整的历元（epoch）新数据时返回 true。

**bool Read()** 读取并解析串口数据。收到历元结束帧（End of Epoch frame）时返回 true，这表明所有数据已更新并可供使用。

```C++
if (ubx.Read()) {
  // 使用 GNSS 数据
}
```

### 数据检索 (Data Retrieval)

最新的有效数据包存储在 Ubx 对象中。可以使用以下函数检索数据字段。

#### 通用数据 (Common Data)

**Fix fix()** 返回 GNSS 定位状态。

| 枚举 (Enum)   | 描述 (Description)                                                        |
| ------------- | ------------------------------------------------------------------------- |
| FIX_NONE      | 无定位 (No fix)                                                           |
| FIX_2D        | 2D 定位 (2D fix)                                                          |
| FIX_3D        | 3D 定位 (3D fix)                                                          |
| FIX_DGNSS     | 应用差分修正的 3D 定位 (3D fix with differential corrections applied)     |
| FIX_RTK_FLOAT | 3D 定位，RTK 修正，浮点模糊度 (RTK corrections with floating ambiguities) |
| FIX_RTK_FIXED | 3D 定位，RTK 修正，固定模糊度 (RTK corrections with fixed ambiguities)    |

**int8_t num_sv()** 导航解算中使用的卫星车辆数量。

**int16_t utc_year()** UTC 年。

**int8_t utc_month()** UTC 月。

**int8_t utc_day()** UTC 日。

**int8_t utc_hour()** UTC 小时。

**int8_t utc_min()** UTC 分钟。

**int8_t utc_sec()** UTC 秒。

**int32_t utc_nano()** UTC 纳秒。

**double gps_tow_s()** GPS 周内秒 (Time of week)，单位：秒 (s)。

**int16_t week()** GPS 周数。

**int8_t leap_s()** 闰秒 (GPS-UTC)。

**uint32_t time_acc_ns()** 估计时间精度，单位：纳秒 (ns)。

**float north_vel_mps()** 北向速度，单位：米/秒 (m/s)。

**float east_vel_mps()** 东向速度，单位：米/秒 (m/s)。

**float down_vel_mps()** 天向（向下）速度，单位：米/秒 (m/s)。

**float gnd_spd_mps()** 地速 (2D)，单位：米/秒 (m/s)。

**float ecef_vel_x_mps()** ECEF X轴速度，单位：米/秒 (m/s)。

**float ecef_vel_y_mps()** ECEF Y轴速度，单位：米/秒 (m/s)。

**float ecef_vel_z_mps()** ECEF Z轴速度，单位：米/秒 (m/s)。

**float spd_acc_mps()** 估计速度精度，单位：米/秒 (m/s)。

**float track_deg()** 估计地面航迹 (2D 运动航向)，单位：度 (deg)。

**float track_rad()** 估计地面航迹 (2D 运动航向)，单位：弧度 (rad)。

**float track_acc_deg()** 估计地面航迹 (2D 运动航向) 精度，单位：度 (deg)。

**float track_acc_rad()** 估计地面航迹 (2D 运动航向) 精度，单位：弧度 (rad)。

**double lat_deg()** 纬度，单位：度 (deg)。

**double lat_rad()** 纬度，单位：弧度 (rad)。

**double lon_deg()** 经度，单位：度 (deg)。

**double lon_rad()** 经度，单位：弧度 (rad)。

**float alt_wgs84_m()** 基于 WGS84 椭球体的高度，单位：米 (m)。

**float alt_msl_m()** 平均海平面高度 (MSL)，单位：米 (m)。

**float horz_acc_m()** 估计水平位置精度，单位：米 (m)。

**float vert_acc_m()** 估计垂直位置精度，单位：米 (m)。

**double ecef_pos_x_m()** ECEF X轴位置，单位：米 (m)。

**double ecef_pos_y_m()** ECEF Y轴位置，单位：米 (m)。

**double ecef_pos_z_m()** ECEF Z轴位置，单位：米 (m)。

**float ecef_pos_acc_m()** 估计 ECEF 位置精度，单位：米 (m)。

**float gdop()** 几何精度衰减因子 (Geometric Dilution of Precision)。

**float pdop()** 位置精度衰减因子 (Position Dilution of Precision)。

**float tdop()** 时间精度衰减因子 (Time Dilution of Precision)。

**float vdop()** 垂直精度衰减因子 (Vertical Dilution of Precision)。

**float hdop()** 水平精度衰减因子 (Horizontal Dilution of Precision)。

**float ndop()** 北向精度衰减因子 (Northing Dilution of Precision)。

**float edop()** 东向精度衰减因子 (Easting Dilution of Precision)。

#### 相对定位数据 (Relative Position Data)

**bool rel_pos_avail()** 相对定位数据是否可用。

**bool rel_pos_moving_baseline()** 接收机是否在移动基站 (moving base) 模式下运行。

**bool rel_pos_ref_pos_miss()** 本历元计算移动基站解算时，是否使用了外推的参考位置。

**bool rel_pos_ref_obs_miss()** 本历元计算移动基站解算时，是否使用了外推的参考观测值。

**bool rel_pos_heading_valid()** 相对位置矢量的航向是否有效。

**bool rel_pos_normalized()** 相对位置矢量的分量（包括高精度部分）是否已归一化。

**double rel_pos_north_m()** 相对位置矢量的北向分量，单位：米 (m)。

**double rel_pos_east_m()** 相对位置矢量的东向分量，单位：米 (m)。

**double rel_pos_down_m()** 相对位置矢量的天向（向下）分量，单位：米 (m)。

**float rel_pos_acc_north_m()** 相对位置北向分量的精度，单位：米 (m)。

**float rel_pos_acc_east_m()** 相对位置东向分量的精度，单位：米 (m)。

**float rel_pos_acc_down_m()** 相对位置天向（向下）分量的精度，单位：米 (m)。

**double rel_pos_len_m()** 相对位置矢量的长度，单位：米 (m)。

**float rel_pos_len_acc_m()** 相对位置矢量长度的精度，单位：米 (m)。

**float rel_pos_heading_deg()** 相对位置矢量的航向，单位：度 (deg)。

**float rel_pos_heading_rad()** 相对位置矢量的航向，单位：弧度 (rad)。

**float rel_pos_heading_acc_deg()** 相对位置矢量航向的精度，单位：度 (deg)。

**float rel_pos_heading_acc_rad()** 相对位置矢量航向的精度，单位：弧度 (rad)。

#### Survey In 数据 (Survey In Data - 定点勘测数据)

**bool svin_valid()** Survey-in 位置有效性标志，true = 有效，否则为 false。

**bool svin_in_progress()** Survey-in 正在进行标志，true = 进行中，否则为 false。

**uint32_t svin_dur_s()** 已经过的 Survey-in 观测时间，单位：秒 (s)。

**double svin_ecef_pos_x_m()** 当前 Survey-in 平均位置 ECEF X 坐标，单位：米 (m)。

**double svin_ecef_pos_y_m()** 当前 Survey-in 平均位置 ECEF Y 坐标，单位：米 (m)。

**double svin_ecef_pos_z_m()** 当前 Survey-in 平均位置 ECEF Z 坐标，单位：米 (m)。

**float svin_ecef_pos_acc_m()** 当前 Survey-in 平均位置精度，单位：米 (m)。

**uint32_t svin_num_obs()** Survey-in 期间使用的位置观测次数。
