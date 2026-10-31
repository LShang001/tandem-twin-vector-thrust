#ifndef ICM45686_FLIGHT_ADAPTER_H
#define ICM45686_FLIGHT_ADAPTER_H

#include "Arduino.h"
#include "ICM45686.h"

/*
 * ICM45686_FlightAdapter
 *
 * 这是面向飞控主循环的轻量适配层。底层 ICM456xx 类保留了官方 driver 风格：
 * 返回原始结构、需要调用者自己做量纲换算、错误统计和数据新鲜度判断。飞控代码如果直接依赖
 * 这些细节，会让导航和控制模块里散落大量传感器驱动知识。
 *
 * 本适配层只做确定性、低成本的事情：
 * - 初始化 accel/gyro 的 ODR 与量程。
 * - 把寄存器原始数据转换为 g、dps、degC。
 * - 记录样本间隔、读数错误、原始值接近饱和次数。
 * - 提供 fresh/stale 判断，便于上层触发降级或传感器故障处理。
 *
 * 本适配层不做坐标系旋转、安装方向补偿、滤波、积分、零偏在线估计或 EKF 融合；这些应由飞控
 * 的传感器标定层、姿态解算层和导航层负责。
 */

/*
 * 飞控采样配置。
 *
 * 这里仍使用整数配置，是为了给上层飞控参数表、EEPROM/Flash 参数保存和串口命令调参保留简单
 * 表达方式。底层会通过 ICM456xx 兼容 API 映射到芯片真实枚举；若需要精确表达 12.5 Hz、
 * 15.625 dps 等小数档位，底层驱动的强类型枚举 API 更合适。
 */
struct ICM45686FlightConfig {
  uint16_t accel_odr_hz = 200;       // 加速度计输出频率，单位 Hz；飞控主循环常用 200 Hz 或更高。
  uint16_t gyro_odr_hz = 200;        // 陀螺仪输出频率，单位 Hz；通常应与姿态环读取频率匹配。
  uint16_t accel_fsr_g = 16;         // 加速度计量程，单位 g；飞行器强机动/振动场景建议优先 16 g。
  uint16_t gyro_fsr_dps = 2000;      // 陀螺仪量程，单位 deg/s；小型飞行器快速旋转时优先 2000 dps。

  // 超过该时间没有成功样本时，readSample() 输出 fresh=false。
  uint32_t stale_after_us = 10000;
};

/*
 * 单次 IMU 采样输出。
 *
 * raw_* 字段保留原始寄存器值，便于做饱和、重复帧、全 0/全 1 和标定回放分析；物理量字段是按
 * 当前配置量程换算得到的直接可读值。这里不做坐标变换，数组顺序仍是芯片读出的 X/Y/Z。
 */
struct ICM45686FlightSample {
  uint32_t timestamp_us = 0;                 // 成功读数后的 micros() 时间戳，用于上层计算 dt。
  int16_t raw_accel[3] = {0, 0, 0};          // 原始加速度寄存器值，满量程约对应 int16 正负端。
  int16_t raw_gyro[3] = {0, 0, 0};           // 原始陀螺寄存器值，满量程约对应 int16 正负端。
  int16_t raw_temp = 0;                      // 原始温度寄存器值，公式见 rawTempToC()。
  float accel_g[3] = {0.0f, 0.0f, 0.0f};     // 换算后的加速度，单位 g。
  float gyro_dps[3] = {0.0f, 0.0f, 0.0f};    // 换算后的角速度，单位 deg/s。
  float temperature_c = 0.0f;                // 换算后的温度，单位摄氏度。
  float accel_norm_g = 0.0f;                 // 三轴加速度模长；静止且轴向正确时应接近 1 g。
  bool saturated = false;                    // 任一轴原始值接近满量程时置位，提示量程可能不够。
  bool fresh = false;                        // 本次 readSample() 成功后置位；失败样本保持 false。
};

/*
 * 运行统计。
 *
 * 这些统计值是飞控健康监测的基础输入：连续 read_errors 可触发传感器降级，dt_min/max 可判断调度
 * 抖动，saturated_samples 可提示量程不足或强振动。不要在中断里直接改这些字段。
 */
struct ICM45686FlightStats {
  uint32_t successful_samples = 0;   // 成功读取并完成换算的样本数。
  uint32_t read_errors = 0;          // 底层 getDataFromRegisters() 或未启动状态导致的失败次数。
  uint32_t saturated_samples = 0;    // 至少一轴接近原始满量程的样本数。
  int last_status = 0;               // 最近一次底层返回码，0 表示 INV_IMU_OK。
  uint32_t last_sample_us = 0;       // 最近一次成功样本时间戳。
  uint32_t dt_min_us = UINT32_MAX;   // 成功样本间最小间隔。
  uint32_t dt_max_us = 0;            // 成功样本间最大间隔。
  uint64_t dt_sum_us = 0;            // 成功样本间隔累计值，用于计算平均 dt。
};

/*
 * 飞控适配类。
 *
 * 生命周期建议：
 * 1. 构造时绑定一个已经用 SPI/I2C 构造好的 ICM456xx。
 * 2. setup() 中调用 begin()，确认返回 0。
 * 3. 主循环或定时任务中周期性调用 readSample()。
 * 4. 上层用 stats() 和 hasFreshSample() 做健康监测。
 */
class ICM45686FlightAdapter {
 public:
  // 只保存引用，不拥有底层对象；调用者必须保证 imu_ref 生命周期长于本适配器。
  explicit ICM45686FlightAdapter(ICM456xx &imu_ref);

  // 初始化底层芯片并启动 accel/gyro。该函数会清空统计值，失败时 started=false。
  int begin(const ICM45686FlightConfig &config = ICM45686FlightConfig());

  // 读取一帧寄存器数据并换算物理量。失败时 sample 会被清零，返回底层错误码。
  int readSample(ICM45686FlightSample &sample);

  // 关闭 accel/gyro 输出。用于测试切换模式或低功耗前停用传感器。
  int stop();

  // 返回当前配置，只读引用避免复制。
  const ICM45686FlightConfig &config() const;

  // 返回统计值，只读引用方便上层遥测输出。
  const ICM45686FlightStats &stats() const;

  // 清零统计值，不改变传感器启动状态。
  void resetStats();

  // 判断最近一次成功样本是否仍在 stale_after_us 时间窗内。
  bool hasFreshSample(uint32_t now_us) const;

 private:
  // UI 寄存器原始加速度为 int16 满量程编码，按当前 FSR 线性换算到 g。
  static float rawAccelToG(int16_t raw, uint16_t fsr_g);

  // UI 寄存器原始陀螺为 int16 满量程编码，按当前 FSR 线性换算到 deg/s。
  static float rawGyroToDps(int16_t raw, uint16_t fsr_dps);

  // ICM45686 UI 温度寄存器换算公式：25 + raw / 128。
  static float rawTempToC(int16_t raw);

  // 简单饱和判据：接近 int16 正负端即认为本轴量程可能不够。
  static bool nearRawLimit(int16_t raw);

  ICM456xx &imu;             // 底层 Arduino 驱动引用，不负责释放。
  ICM45686FlightConfig cfg;  // 最近一次 begin() 使用的配置。
  ICM45686FlightStats stat;  // 运行统计。
  bool started = false;      // begin() 成功启动 accel/gyro 后为 true。
};

#endif
