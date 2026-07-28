/**
 * @file math_utils.h
 * @brief 纯数学/工具函数集合 (header-only, inline)
 *
 * 本文件包含不依赖任何全局状态的纯工具函数。
 * 所有函数标记为 inline，可被多个 .cpp 文件安全包含。
 */
#pragma once

#include <cmath>
#include <cstdint>
#include <Arduino.h>
#include "state_data.h"

/**
 * @brief 将角度归一化到 [-π, π] 范围
 * 使用fmod()函数替代while循环，提高大角度处理效率
 */
inline float wrapAnglePi(float angle_rad)
{
  // 使用fmod将角度归一化到[-2π, 2π]范围
  angle_rad = fmodf(angle_rad, 2.0f * M_PI);
  // 处理负角度，确保在[-π, π]范围
  if (angle_rad > M_PI)
    angle_rad -= 2.0f * M_PI;
  else if (angle_rad < -M_PI)
    angle_rad += 2.0f * M_PI;
  return angle_rad;
}

/**
 * @brief 将角度归一化到 [0, 2π) 范围
 * 使用fmod()函数替代while循环，提高大角度处理效率
 */
inline float wrapAngleTwoPi(float angle_rad)
{
  // 使用fmod将角度归一化到[-2π, 2π]范围
  angle_rad = fmodf(angle_rad, 2.0f * M_PI);
  // 处理负角度，确保在[0, 2π)范围
  if (angle_rad < 0.0f)
    angle_rad += 2.0f * M_PI;
  return angle_rad;
}

/**
 * @brief 线性插值函数
 */
inline float linearInterpolate(float x, float x0, float x1, float y0, float y1)
{
  if (x <= x0)
    return y0;
  if (x >= x1)
    return y1;
  return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

/**
 * @brief 浮点数映射函数（带限幅）
 *
 * 将输入值从 [in_min, in_max] 线性映射到 [out_min, out_max]。
 * 自动处理输出范围的正反向（out_min < out_max 或 out_min > out_max）。
 * 当输入范围为零时返回 out_min 以避免除零。
 *
 * @param x        输入值
 * @param in_min   输入范围下限
 * @param in_max   输入范围上限
 * @param out_min  输出范围下限
 * @param out_max  输出范围上限
 * @return 映射并限幅后的输出值
 */
inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  // 输入范围为零时返回 out_min，避免除零
  if (in_max == in_min)
  {
    return out_min;
  }
  // 线性映射公式: y = y0 + (x - x0) * (y1 - y0) / (x1 - x0)
  float mapped_value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  // 根据输出范围方向进行限幅
  if (out_min < out_max)
  {
    return std::min(std::max(mapped_value, out_min), out_max); // 正向范围限幅
  }
  else
  {
    return std::min(std::max(mapped_value, out_max), out_min); // 反向范围限幅
  }
}

/**
 * @brief 控制舵机位置 (支持扩展PWM范围)
 *
 * 将百分比位置转换为 PWM 脉宽并输出到指定引脚。
 * 映射关系: -50% -> 500us, 50% -> 1500us (中位), 150% -> 2500us
 * 使用 CUSTOM_PWM_FREQUENCY (333Hz) 和 CUSTOM_PWM_RESOLUTION (16位) 配置。
 *
 * @param percent 舵机目标位置 (-50% ~ 150%)，50%对应中位
 * @param pwmPin  PWM 输出引脚编号
 */
inline void SetServoPos(float percent, int pwmPin)
{
  // 舵机位置和脉宽范围常量
  const float MIN_PERCENT = -50.0f;           // 最小百分比位置
  const float MAX_PERCENT = 150.0f;           // 最大百分比位置
  const float MIN_PULSE_WIDTH_US = 500.0f;    // 最小脉宽 (微秒)
  const float MAX_PULSE_WIDTH_US = 2500.0f;   // 最大脉宽 (微秒)

  // 限幅百分比到有效范围
  percent = constrain(percent, MIN_PERCENT, MAX_PERCENT);

  // 将百分比线性映射为脉宽 (微秒)
  float pulse_width_us = MIN_PULSE_WIDTH_US +
                         (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US) *
                             ((percent - MIN_PERCENT) / (MAX_PERCENT - MIN_PERCENT));

  // 计算 PWM 周期和最大占空比值
  float pwm_period_us = 1000000.0f / CUSTOM_PWM_FREQUENCY;       // PWM周期 (微秒)
  float max_pwm_value = static_cast<float>((1 << CUSTOM_PWM_RESOLUTION) - 1); // 16位最大值=65535

  // 计算占空比并输出
  float dutyCycle = (pulse_width_us / pwm_period_us) * max_pwm_value;
  analogWrite(pwmPin, static_cast<uint32_t>(dutyCycle));
}

/**
 * @brief 直接设置PWM脉宽 (单位: 微秒)
 *
 * 将脉宽微秒值直接转换为 PWM 占空比并输出。
 * 脉宽范围: 500us ~ 2500us (标准舵机/电调范围)
 *
 * @param pulse_width_us 目标脉宽 (微秒, 500-2500)
 * @param pwmPin         PWM 输出引脚编号
 */
inline void setPulseWidth(float pulse_width_us, int pwmPin)
{
  // 限幅到标准脉宽范围
  pulse_width_us = constrain(pulse_width_us, 500.0f, 2500.0f);
  // 计算占空比
  float pwm_period_us = 1000000.0f / CUSTOM_PWM_FREQUENCY;       // PWM周期
  float max_pwm_value = static_cast<float>((1 << CUSTOM_PWM_RESOLUTION) - 1); // 最大计数值
  uint32_t dutyCycle = static_cast<uint32_t>((pulse_width_us / pwm_period_us) * max_pwm_value);
  analogWrite(pwmPin, dutyCycle);
}

/**
 * @brief 获取TVC舵机当前角度 (通过外置角度传感器)
 *
 * 读取 ADC 引脚的模拟值，转换为电压后映射为角度。
 * ADC 为 16位 (0-65535)，参考电压 3.3V。
 * 角度映射: 0V -> -22.5°, 3.3V -> +22.5° (总行程45°)
 *
 * @param servoPin ADC 引脚编号
 * @return TVC 舵机当前角度 (度, 范围 -22.5 ~ +22.5)
 */
inline float getServoAngle(int servoPin)
{
  int sensorValue = analogRead(servoPin);                              // 读取 ADC 原始值 (0-65535)
  float sensorVoltage = static_cast<float>(sensorValue) * (3.3f / 65535.0f); // 转换为电压 (0-3.3V)
  float sensorAngle = (sensorVoltage / 3.3f) * 45.0f - 22.5f;         // 映射为角度 (-22.5 ~ +22.5°)
  return sensorAngle;
}

/**
 * @brief 获取当前飞行控制模式
 *
 * 根据遥控器模式通道值和解锁状态确定飞行模式：
 * - 通道5 (raw_rc_values[4]) < 1500: 未解锁，强制手动模式
 * - 模式通道 < 1300: 手动模式 (MANUAL)
 * - 模式通道 1300-1650: 高度保持模式 (AUTO_ALTITUDE)
 * - 模式通道 1650-2100: 根据轨迹规划状态选择定点 (AUTO_POSITION) 或制导 (GUIDED)
 *
 * @param mode_channel_val 模式通道的 PWM 值 (988-2012)
 * @return ControlMode 当前飞行模式枚举值
 */
inline ControlMode getControlMode(float mode_channel_val)
{
  // 安全联锁：未解锁时强制手动模式
  if (raw_rc_values[4] < 1500)
  {
    return MANUAL;
  }
  // 根据模式通道 PWM 值分档
  if (mode_channel_val < 1300)
  {
    return MANUAL;           // 低档: 手动模式
  }
  else if (mode_channel_val < 1650)
  {
    return AUTO_ALTITUDE;    // 中档: 高度保持模式
  }
  else if (mode_channel_val < 2100)
  {
    // 高档: 根据轨迹规划状态选择定点或制导模式
    if (trajectoryPlanningStarted)
    {
      return GUIDED;         // 点火且模式通道>1750: 制导模式
    }
    else
    {
      return AUTO_POSITION;  // 否则: 定点模式
    }
  }
  else
  {
    return MANUAL;           // 超出范围: 回退手动模式
  }
}

/**
 * @brief NED坐标系到机体系的二维旋转
 */
inline void nedToBody(float ned_x, float ned_y, float yaw_rad, float &body_x, float &body_y)
{
  float cos_yaw = cosf(yaw_rad);
  float sin_yaw = sinf(yaw_rad);
  body_x = ned_x * cos_yaw + ned_y * sin_yaw;
  body_y = -ned_x * sin_yaw + ned_y * cos_yaw;
}

/**
 * @brief 机体系到NED坐标系的二维旋转
 */
inline void bodyToNed(float body_x, float body_y, float yaw_rad, float &ned_x, float &ned_y)
{
  float cos_yaw = cosf(yaw_rad);
  float sin_yaw = sinf(yaw_rad);
  ned_x = body_x * cos_yaw - body_y * sin_yaw;
  ned_y = body_x * sin_yaw + body_y * cos_yaw;
}

/**
 * @brief 应用死区处理
 *
 * 对输入值施加对称死区：绝对值小于死区阈值时输出零，
 * 超出死区时减去死区偏移，确保输出在死区边界处连续。
 *
 * @param value    输入值
 * @param deadzone 死区阈值 (正值)
 * @return 经死区处理后的值
 */
inline float applyDeadzone(float value, float deadzone)
{
  if (fabsf(value) < deadzone)
  {
    return 0.0f;                 // 在死区内: 输出零
  }
  else if (value > 0.0f)
  {
    return value - deadzone;     // 正向超出: 减去死区偏移
  }
  else
  {
    return value + deadzone;     // 负向超出: 加上死区偏移
  }
}

/**
 * @brief 将浮点数转换为大端字节数组
 */
inline void floatToBigEndianBytes(float value, uint8_t *bytes)
{
  uint8_t *floatBytes = reinterpret_cast<uint8_t *>(&value);
  bytes[0] = floatBytes[3];
  bytes[1] = floatBytes[2];
  bytes[2] = floatBytes[1];
  bytes[3] = floatBytes[0];
}
