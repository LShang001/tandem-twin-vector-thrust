/**
 * @file    imu_range.h
 * @module  固件/传感器（IMU 量程契约）
 * @brief   ICM42688 实际配置的加速度/角速度量程常量，供传感器采集与
 *          IMU 健康监测共用，避免换量程时只改一处导致饱和误判
 * @details 传感器层用这些常量选择 `setAccelFS/setGyroFS`；导航层用同一组
 *          常量初始化 `ImuHealthConfig`。改量程时只改本文件。
 *          不变量：`kIcm42688AccelRangeMps2` 必须等于所选 FS 档的满量程。
 * @theory  量程/门限契约：健康监测的饱和门限必须由实际 FS 派生，否则
 *          16 g 档会被 8 g 门限误判为饱和
 * @frame   N/A（标量量程）
 * @units   g、m/s²、dps、rad/s
 * @note    当前生产配置为 ±8 g / ±2000 dps；78.45 与 34.9066 是既有
 *          健康门限的取整值，保持数值不变以免改变已验证行为。
 * @see     src/sensor_imu.cpp、src/navigation_task.cpp、include/ins_imu_health.h
 */

#ifndef TANDEMVEC_IMU_RANGE_H
#define TANDEMVEC_IMU_RANGE_H

namespace bfs
{

/* 量程对（FS + 满量程物理值）改为编译期宏，默认值与历史逐位相同。
   动机：16 g / 32 g 包线研究此前靠**手改本文件**产出二进制（报告 §4.15 的
   `tv_32g_*`），产物不可复现、也无法与 `--imu-fs-g` 档对齐；宏化后可用
   `EXTRA_CXXFLAGS="-DBFS_ICM42688_ACCEL_FS_G=32.0f -DBFS_ICM42688_ACCEL_RANGE_MPS2=313.8f"`
   一条命令重建，且下面的 static_assert 会挡住"只改 FS 不改 m/s²"的半改状态
   （量程三处必须同步：传感器 FS、健康门限、EKF 合理值门限）。 */
#ifndef BFS_ICM42688_ACCEL_FS_G
#define BFS_ICM42688_ACCEL_FS_G 8.0f
#define BFS_ICM42688_ACCEL_RANGE_MPS2 78.45f
#endif
#ifndef BFS_ICM42688_GYRO_FS_DPS
#define BFS_ICM42688_GYRO_FS_DPS 2000.0f
#define BFS_ICM42688_GYRO_RANGE_RADPS 34.9066f
#endif

constexpr float kIcm42688AccelFullScaleG = BFS_ICM42688_ACCEL_FS_G;
constexpr float kIcm42688AccelRangeMps2 = BFS_ICM42688_ACCEL_RANGE_MPS2;
constexpr float kIcm42688GyroFullScaleDps = BFS_ICM42688_GYRO_FS_DPS;
constexpr float kIcm42688GyroRangeRadps = BFS_ICM42688_GYRO_RANGE_RADPS;

// 成对一致性：m/s² 与 rad/s 必须等于 FS × 标准换算（容差 0.2%，覆盖 78.45/34.9066
// 这类历史取整值），否则说明只改了 FS 没改物理量程。
static_assert(kIcm42688AccelRangeMps2 > 0.998f * kIcm42688AccelFullScaleG * 9.80625f &&
                  kIcm42688AccelRangeMps2 < 1.002f * kIcm42688AccelFullScaleG * 9.80625f,
              "IMU 加计量程对不一致：BFS_ICM42688_ACCEL_RANGE_MPS2 必须随 FS 同步");
static_assert(kIcm42688GyroRangeRadps > 0.998f * kIcm42688GyroFullScaleDps * 0.01745329252f &&
                  kIcm42688GyroRangeRadps < 1.002f * kIcm42688GyroFullScaleDps * 0.01745329252f,
              "IMU 陀螺量程对不一致：BFS_ICM42688_GYRO_RANGE_RADPS 必须随 FS 同步");

} // namespace bfs

#endif // TANDEMVEC_IMU_RANGE_H
