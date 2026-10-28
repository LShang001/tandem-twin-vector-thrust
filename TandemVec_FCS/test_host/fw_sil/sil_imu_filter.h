/**
 * @file    sil_imu_filter.h
 * @module  固件/宿主 SIL（传感器路径：ICM 软件低通）
 * @brief   复现 `sensor_imu.cpp` 的 `ComplementaryFilter` 一阶低通（2 kHz）：
 *          生产在**传感器系**滤波（陀螺 α=0.15、加计 x/y α=0.02、z α=0.05），
 *          之后才映射到机体系；本模块输入已是机体系，故按映射后的系数：
 *          机体系 x（来自传感器 z）α=0.05、y（传感器 x）α=0.02、
 *          z（传感器 y）α=0.02；陀螺三轴 α=0.15。
 * @details 生产固件在 `sensor_imu.cpp` 对每个 2 kHz 样本先做一阶指数低通，
 *          再做 RUB→FRD 轴映射，然后送入导航任务；SIL 若直接注入
 *          `ImuModel::Step` 原始样本，会绕过这条滤波，使 120 Hz 振动/噪声被高估。
 *          轴系数不能照抄传感器系的 z=0.05：生产 `axBody=-azFilteredRaw`，
 *          纵向机体系 x 用的是传感器 z（α=0.05）。
 *          本模块只做标量通道滤波，不改变坐标/单位/量程。
 * @theory  一阶低通 y←αx+(1−α)y，α=dt/(τ+dt) @ref REF-121 §2.2.1 p.33
 * @frame   N/A（标量通道；输入输出同单位）
 * @units   加速度 m/s²、角速度 rad/s
 * @note    ICM 内部 100 Hz 陀螺/50 Hz 加计数字滤波未单独建模；本层是
 *          `sensor_imu.cpp` 的软件滤波，已足以对齐主要群延迟/衰减。
 */

#pragma once

#include "sil_sensor_model.h"

namespace sil {

struct ImuLpfState
{
  bool initialized = false;
  float accel[3] = {0.0f, 0.0f, 0.0f};
  float gyro[3] = {0.0f, 0.0f, 0.0f};
};

inline void ImuLpfInit(ImuLpfState &s, const ImuSample &first)
{
  for (int i = 0; i < 3; ++i)
  {
    s.accel[i] = first.accel[i];
    s.gyro[i] = first.gyro[i];
  }
  s.initialized = true;
}

inline ImuSample ImuLpfApply(ImuLpfState &s, const ImuSample &in)
{
  if (!s.initialized)
  {
    ImuLpfInit(s, in);
    return in;
  }
  // 生产滤波在传感器系：sensor x/y α=0.02、sensor z α=0.05；
  // 映射 axBody=-azSensor、ayBody=+axSensor、azBody=-aySensor，
  // 因此机体系系数为 x=0.05、y=0.02、z=0.02。
  const float a_body[3] = {0.05f, 0.02f, 0.02f};
  const float a_g = 0.15f;
  ImuSample out = in;
  for (int i = 0; i < 3; ++i)
  {
    s.accel[i] = a_body[i] * in.accel[i] + (1.0f - a_body[i]) * s.accel[i];
    s.gyro[i] = a_g * in.gyro[i] + (1.0f - a_g) * s.gyro[i];
    out.accel[i] = s.accel[i];
    out.gyro[i] = s.gyro[i];
  }
  return out;
}

}  // namespace sil
