/**
 * @file    ImuDevice.h
 * @module  传感器库 / IMU 型号无关抽象接口（ImuAbstraction）
 * @brief   为多款 6 轴 IMU 提供统一读取接口，屏蔽寄存器架构差异
 * @details 本文件只定义接口与配置类型，**不包含任何具体芯片实现，也不 include
 *          任何芯片头文件**。具体后端分别位于各自芯片库内：
 *            - lib/ICM42688/src/ICM42688_ImuDevice.{h,cpp}
 *            - lib/ICM45686/src/ICM45686_ImuDevice.{h,cpp}
 *          装配（根据编译期宏选择后端）由固件的 src/imu_factory.cpp 完成。
 *
 * @theory  N/A（设备抽象层）。两款芯片的关键差异（均以官方手册为准）：
 *
 *          | 项目        | ICM-42688-P                        | ICM-45686                    |
 *          |-------------|------------------------------------|------------------------------|
 *          | WHO_AM_I    | reg 0x75, 值 0x47（需先切 bank0）  | reg 0x72, 值 0xE9（扁平）    |
 *          | 寄存器架构  | bank 切换（REG_BANK_SEL=0x76）     | 扁平 + IPREG 间接窗口        |
 *          | LPF         | GYRO/ACCEL_CONFIG0 直接写带宽档    | IPREG_SYS1_REG_172 / SYS2_131|
 *          | LPF 语义    | 固定标称 Hz（50/100/200/400）      | ODR/2^n 倍率（随 ODR 变化）  |
 *          | SPI 读      | Mode3, MSB first, bit7=1, 无 dummy | 同左（手册 §9.5 逐条一致）   |
 *
 *          手册依据：ICM-45686 见 lib/ICM45686/doc/DS-000489 ICM-45686 v1.1.pdf
 *          §9.5（SPI 时序/读写位）、§13（间接寄存器 4µs 间隔 + 单次 burst）、
 *          §14（端序：默认小端，寄存器表按大端描述）、§16/§20/§21（寄存器位域）。
 *
 * @ref     lib/ICM45686/doc/ICM45686_手册对照审查报告.md
 * @frame   N/A：本层只转发芯片原生轴向数据，坐标系变换（RUB→FRD）在
 *          sensor_imu.cpp 统一完成，避免同一变换分散到多个库。
 * @units   accX/Y/Z = g；gyrX/Y/Z = deg/s。
 * @note    ★ 引脚共用结论（2026-10-31）：两款芯片均为 LGA-14 2.5x3.0mm，且
 *          4-wire SPI 读时序逐条一致（见上表），因此 PA4(CS)/PA5(SCLK)/
 *          PA6(MISO)/PA7(MOSI) 可直接复用，换型只需改编译期宏。
 *          **未验证**：pin1..pin14 的供电/信号物理顺序尚未逐脚比对两份官方
 *          Signal Descriptions 表。换 PCB 前必须逐脚核对 VDD/VDDIO/GND/INT
 *          位置，不能凭"同为 LGA-14"就认定可直插替换。
 * @see     src/imu_factory.cpp（装配）、src/sensor_imu.cpp（唯一业务使用者）
 */
#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>

/**
 * @brief 支持的 IMU 型号。
 * @note  新增型号需同时实现：本接口的后端类 + 在 imu_factory.cpp 增加宏分支。
 * @note  ★ 枚举值刻意命名为 Icm45686 而非 ICM45686 —— lib/ICM45686/src/imu/
 *        inv_imu.h:44 有 `#define ICM45686`（空宏），会把该标识符整个抹掉，
 *        导致 `ImuModel::ICM45686` 退化成 `ImuModel::` 语法错误。
 */
enum class ImuModel : uint8_t
{
  Unknown = 0,
  ICM42688P, ///< TDK InvenSense ICM-42688-P（现役硬件）
  Icm45686,  ///< TDK InvenSense ICM-45686（升级候选）
};

/**
 * @brief 统一的低通滤波带宽档位。
 *
 * ★ 重要：本枚举表达的是"档位语义"，不是"绝对截止频率"。
 *   - ICM-42688-P：DLPF 为固定标称带宽，Bw100Hz 就是约 100Hz。
 *   - ICM-45686：UI LPF 是 ODR/2^n 倍率档，后端会按**实际 ODR**换算成最接近
 *     目标频率的 2^n 档（n 在 2..7）。同一 Bw100Hz 在 ODR=2000Hz 落到 Div16
 *     (125Hz)，在 ODR=3200Hz 落到 Div32 (100Hz)。
 *   因此**切换型号后滤波特性必须重新评估**，不能把一颗芯片的调参结论直接沿用。
 */
enum class ImuLpfBw : uint8_t
{
  Bw50Hz = 0,
  Bw100Hz,
  Bw200Hz,
  Bw400Hz,
};

/**
 * @brief 统一采样配置。
 *
 * ★ ODR 语义（重要）：这里的 Hz 是"意图目标值"，不是精确档位。
 *   两款芯片的 ODR 档位是两套不同的 2 倍序列，交集只有 50/100/200Hz：
 *     - ICM-42688-P: 50,100,200,500,1000,2000,4000,8000,16000,32000
 *     - ICM-45686  : 50,100,200,400,800,1600,3200,6400 （无 2000Hz）
 *   因此后端的选择规则是：取"不小于目标值的最接近档"；若更高档不存在则取最高档。
 *   这保证换型后采样率不会低于目标（避免读到大量重复样本、破坏 IMU 增量累积
 *   的 dt 一致性），并通过 actualAccelOdrHz()/actualGyroOdrHz() 暴露真实生效值，
 *   供上层遥测核对。
 *
 *   现役硬件（ICM42688 @ 2000Hz）行为不变；若换 ICM45686，同一配置会落到
 *   3200Hz（最小且不小于 2000 的可用档），需重新评估 EKF 的 IMU 降采样假设。
 */
struct ImuConfig
{
  /// 加速度计 ODR 目标值 (Hz)，见上方 ODR 语义说明。
  uint16_t accelOdrHz = 2000;
  /// 陀螺仪 ODR 目标值 (Hz)，见上方 ODR 语义说明。
  uint16_t gyroOdrHz = 2000;
  /// 加速度计量程 (g)：2/4/8/16（两款一致，精确支持）。
  uint8_t accelFsrG = 8;
  /// 陀螺仪量程 (dps)：125/250/500/1000/2000（两款一致，精确支持）。
  uint16_t gyroFsrDps = 2000;
  // ★2026-11-01 降延迟：陀螺 100→200Hz、加计 50→100Hz。
  //   一阶 LPF 群延迟 ≈ 1/(2π·fc)：100Hz→1.59ms、200Hz→0.80ms、
  //   50Hz→3.18ms、100Hz→1.59ms。姿态环 200Hz（5ms）下，陀螺延迟从
  //   0.32 拍降到 0.16 拍，加计从 0.64 拍降到 0.32 拍。
  //   代价：白噪声 RMS ∝ sqrt(带宽)，200Hz 陀螺噪声约为 100Hz 的 1.41 倍；
  //   仿真矩阵显示该带宽变化对姿态/位置 RMSE 影响 <5%（见 test_imu_nav_perf
  //   的 LPF A/B），因此用带宽换延迟是划算的。上板仍需复核姿态环相位裕度。
  ImuLpfBw gyroLpf = ImuLpfBw::Bw200Hz;
  ImuLpfBw accelLpf = ImuLpfBw::Bw100Hz;
};

/**
 * @brief IMU 型号无关驱动接口。
 *
 * 使用约定（与 sensor_imu.cpp 既有流程一致）：
 *   1. `begin(cfg)` 初始化并应用配置；返回 0 成功、负值失败（见各后端错误码）。
 *   2. `dataReady()` 返回 1 表示有新样本；随后用 accX()..gyrZ() 取值。
 *   3. 全部方法须在同一任务上下文调用，**不保证线程安全**（本工程为 2kHz 调度
 *      任务内调用，且与 200Hz 导航任务串行）。
 *
 * 设计约束：本层刻意不提供 FIFO/中断/APEX 等高级能力——飞控当前用的是
 * 轮询 UI 寄存器路径。若将来改用 FIFO，应扩展接口而不是让上层直接接触芯片库。
 *
 * @note ★ 关于 ICM45686 库里自带的 ICM45686FlightAdapter（2026-10-31 决策）：
 *       它同样是一层"飞控适配"，与本接口职责重叠，但**刻意没有采用**，原因：
 *        1. 它只有 45686 一侧，接了就会变成"42688 走 ImuDevice、45686 走
 *           FlightAdapter"的两套路径，丧失型号无关性；
 *        2. 它额外提供 readSample()（raw+物理量+饱和标志）、hasFreshSample()、
 *           stats()（读错计数/dt 统计）。这些对健康监测有价值，但**飞控当前
 *           没有任何消费方**（grep 全仓无引用），接入等于增加未使用的代码面；
 *        3. 飞控现有链路 readIMUData() 是 dataReady()→accX()..gyrZ()，与
 *           ICM42688 库原生用法对称，保持这条最小公共路径最不容易出错。
 *       若将来要做 IMU 健康监测（重复帧/全0/全1/饱和/连续错误），正确做法是
 *       在本接口上**同时**为两个型号扩展（例如加 stats()/isSaturated() 的
 *       型号无关版本），而不是只给 45686 接 FlightAdapter。
 */
class ImuDevice
{
public:
  virtual ~ImuDevice() = default;

  /// 芯片型号（遥测/日志区分用）。
  virtual ImuModel model() const = 0;

  /// 型号名称（静态字符串，无需释放）。
  virtual const char *modelName() const = 0;

  /// 初始化芯片并应用 cfg。返回 0 成功；负值见后端错误码定义。
  virtual int begin(const ImuConfig &cfg) = 0;

  /// 轮询数据就绪。返回 1 表示有新样本可取。
  virtual int dataReady() = 0;

  /// 加速度 X/Y/Z，单位 g（芯片原生轴向，未做坐标系变换）。
  virtual float accX() = 0;
  virtual float accY() = 0;
  virtual float accZ() = 0;

  /// 角速度 X/Y/Z，单位 deg/s（芯片原生轴向，未做坐标系变换）。
  virtual float gyrX() = 0;
  virtual float gyrY() = 0;
  virtual float gyrZ() = 0;

  /// 读取 WHO_AM_I 原始值用于健康检查/型号诊断。返回 0 表示读取成功。
  virtual int readWhoAmI(uint8_t &value) = 0;

  /// 实际生效的加速度计 ODR (Hz)。begin() 成功后有效；用于核对 ODR 意图是否
  /// 被精确满足（见 ImuConfig 的 ODR 语义说明）。
  virtual uint16_t actualAccelOdrHz() const = 0;

  /// 实际生效的陀螺仪 ODR (Hz)。
  virtual uint16_t actualGyroOdrHz() const = 0;

  /// 芯片温度 (°C)。未读到有效样本时返回 NAN（上层用 std::isfinite 过滤）。
  /// ★2026-11-01 为 TandemVec 的 1 Hz 芯片温度遥测（imu_die_temp_c）新增；
  ///   本抽象层其余部分保持与 VTVL 项目一致。
  virtual float dieTempC() = 0;

  /// 最近一次失败的可读描述（成功时为空串）。
  virtual const char *lastError() const { return ""; }
};
