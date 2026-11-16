/**
 * @file    mag_cal_defaults.h
 * @module  固件/传感器（磁力计编译期默认标定参数）
 * @brief   把一次合格椭球标定的参数固化进固件，作为内部 Flash 标定的兜底
 * @details 上电加载顺序（initMagnetometer）：内部 Flash（magic/版本/CRC 校验）
 *          → 本文件编译期默认参数 → 未标定直通。Flash 无有效标定（换板、擦除、
 *          首次烧录）且本文件 kValid=true 时，应用默认参数并尝试写回 Flash；
 *          写回失败只影响持久化，不影响本次运行。
 *          强制改用本文件参数（Flash 里已有旧标定时）：DBG `magcal erase` → 重启。
 * @note    ★ 机型门控：共轴双桨分支（BFS_COAXIAL_VTVL）已启用，参数来自同一架
 *          飞行器的户外标定（2026-11-15，45686 配置实机；用户 2026-11-16 确认
 *          标定机体与本项目共轴 env 是同一架，仅运行代码不同）。纵列/单涵道
 *          保持 kValid=false，待各自 magcal guided 标定完成后填入，不得抄共轴值。
 *          磁参数取决于机体铁磁构件/电流回路/磁力计安装，与 IMU 型号无关，
 *          共轴全部 IMU 配置适用。
 *          ★ 与 MagMath.h 共轴轴对齐表是实测组合：软铁白化矩阵含不可观旋转
 *          （约 +90°），换参数或换轴表后必须用 DBG magraw 复核——机头朝北
 *          水平 yaw_est≈0°、机头朝东≈+90°（含长沙磁偏 −4.114° 补偿后）。
 *          ★ 灵敏度单位系差异：源固件 IST8310 驱动用 0.30303 uT/LSB（旧注释），
 *          本项目用 0.3（数据手册典型值，已修 +1%）。b/scale 按源单位系原值
 *          移植、不做换算：理论硬铁残差 ~0.15 uT（航向 ~0.2°），相对标定自身
 *          残差（拟合场强 41.8 vs 长沙参考 48.97，偏低 ~15%）是二阶小量；
 *          M 无量纲不受影响。要高精度应在本项目固件上重标（magcal guided）
 *          后更新本文件，移植值只是「免重标即可飞」的兜底。
 *          参数坐标系 = 磁力计芯片系（与 magcal::MagCalParams 一致，标定在
 *          芯片系完成、轴对齐在其后，见 MagSensor.h 标定管线）。
 * @see     lib/MagCal/MagCalStore.h（Flash 持久化）、lib/MagCal/MagCalMath.h
 *          （paramsSanity：scale 必须 5–200 µT）、src/sensor_imu.cpp initMagnetometer、
 *          lib/MagSensor/MagMath.h（共轴轴对齐实测表）
 */

#ifndef TANDEMVEC_MAG_CAL_DEFAULTS_H
#define TANDEMVEC_MAG_CAL_DEFAULTS_H

namespace magcal_defaults
{

#if defined(BFS_COAXIAL_VTVL)

// 共轴双桨实机（45686 配置）2026-11-15 户外重标固化值。
// 来源：同源 VTVL 固件包 VTVL_ElectricDualRotor_FCS_CNmodify_new 的
// include/mag_cal_defaults.h（同一架飞行器，仅运行代码不同）。
static constexpr bool kValid = true;

// 硬铁偏移 b (uT, 芯片系)
static constexpr float kHardIron[3] = {-3.6144f, 15.5312f, -4.6636f};

// 软铁矩阵 M：c = scale·M·(raw − b)
static constexpr float kSoftIron[3][3] = {
    {-0.0027f, 0.0218f, -0.0136f},
    {0.0237f, 0.0014f, -0.0023f},
    {0.0012f, 0.0122f, 0.0194f}};

// 拟合总场强 (uT)。比长沙 IGRF 参考 48.97 偏低 ~15%，反映本次标定的软铁残差；
// 模长偏差门参考取本值（mag_measurement_gate.h），不用固定站点总场。
static constexpr float kScale = 41.8343f;

#else

// 纵列/单涵道：尚未完成上板椭球标定，保持「Flash 无效则未标定直通」。
// 填写方法：DBG magcal guided（或 start/fit）成功后，按 magcal status 打印的
// bias/softIron/scale 逐项填入，置 kValid=true，重新编译烧录。不得抄共轴值。
static constexpr bool kValid = false;
static constexpr float kHardIron[3] = {0.0f, 0.0f, 0.0f};
static constexpr float kSoftIron[3][3] = {
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f}};
static constexpr float kScale = 0.0f;

#endif

} // namespace magcal_defaults

#endif // TANDEMVEC_MAG_CAL_DEFAULTS_H
