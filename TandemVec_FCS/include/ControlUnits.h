#pragma once

// 控制器人机接口采用角度制，刚体动力学采用 SI 制。
// 所有角加速度只允许在控制器/物理层边界通过本文件转换。
namespace ControlUnits
{
static constexpr float kDegPerRad = 57.29577951308232f;
static constexpr float kRadPerDeg = 0.017453292519943295f;

constexpr float dps2ToRadps2(float alpha_dps2)
{
    return alpha_dps2 * kRadPerDeg;
}

constexpr float radps2ToDps2(float alpha_radps2)
{
    return alpha_radps2 * kDegPerRad;
}
} // namespace ControlUnits
