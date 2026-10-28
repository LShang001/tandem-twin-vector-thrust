/**
 * @file    sil_vibration_isolator.h
 * @module  固件/宿主 SIL（弹载飞控减震座传递率）
 * @brief   单自由度基础激励隔振传递率，用于评估物理减震底座对 IMU 振动的抑制
 * @details 输入：激励频率 f、减震座固有频率 fn、阻尼比 ζ；处理：
 *          T(r)=sqrt((1+(2ζr)²)/((1−r²)²+(2ζr)²))，r=f/fn；
 *          输出：无量纲传递率（<1 隔振，>1 放大）。SIL 用它缩放注入 IMU 的
 *          振动幅值，模拟“空气框架 → 减震座 → 飞控板”的机械低通。
 * @theory  单自由度质量-弹簧-阻尼系统在基础简谐激励下的位移传递率；
 *          共振区 r≈1 放大，r>√2 开始隔振，高频按 1/r² 衰减。
 *          @ref S. S. Rao, Mechanical Vibrations, 6th ed., Ch. 3
 *          (Harmonically Excited Vibration, base excitation)
 * @frame   N/A（标量传递率，作用于三个平动/转动通道的振动分量）
 * @units   输入 Hz；输出无量纲
 * @note    这是**幅值**模型，不含相位；单频正弦下相位不影响 RMS。真实减震座
 *          还有转动传递率、非线性和温度/老化，需台架实测后再定参数。
 *          本模块只用于 SIL 研究，不进入固件。
 */
#pragma once

#include <algorithm>
#include <cmath>

namespace sil
{

inline float VibrationTransmissibility(float f_hz, float fn_hz, float zeta)
{
  if (!(f_hz > 0.0f) || !(fn_hz > 0.0f))
  {
    return 1.0f;
  }
  zeta = std::clamp(zeta, 0.001f, 2.0f);
  const float r = f_hz / fn_hz;
  const float r2 = r * r;
  const float c = 2.0f * zeta * r;
  const float c2 = c * c;
  const float den = (1.0f - r2) * (1.0f - r2) + c2;
  if (!(den > 0.0f))
  {
    return 1.0f;
  }
  return std::sqrt((1.0f + c2) / den);
}

}  // namespace sil
