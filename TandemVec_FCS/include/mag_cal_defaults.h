/**
 * @file    mag_cal_defaults.h
 * @module  固件/传感器（磁力计编译期默认标定参数）
 * @brief   把一次合格椭球标定的参数固化进固件，作为内部 Flash 标定的兜底
 * @details 上电加载顺序（initMagnetometer）：内部 Flash（magic/版本/CRC 校验）
 *          → 本文件编译期默认参数 → 未标定直通。Flash 无有效标定（换板、擦除
 *          Flash、首次烧录）且本文件 kValid=true 时，应用默认参数并尝试写回
 *          Flash；写回失败只影响持久化，不影响本次运行。
 *          机制来源：同源共轴 VTVL 固件（2026-11-15 户外重标后固化，换板/重烧
 *          无需重新两圈标定）；本文件只移植机制，参数须用**本机型**标定结果填写。
 * @note    ★ 参数是机型+安装状态+磁环境专属：纵列机型的安装方向/硬铁环境与共轴
 *          不同，**不得抄用共轴 VTVL 的标定值**。更换磁力计模块、改变安装位置/
 *          方向、或到不同磁环境（不同城市）飞行时，必须重新标定并更新本文件。
 *          填写方法：DBG `magcal guided`（或 start/fit）成功后，按 `magcal status`
 *          打印的 bias/softIron/scale 逐项填入，置 kValid=true，重新编译烧录。
 *          参数坐标系 = 磁力计芯片系（与 magcal::MagCalParams 一致，标定在芯片系
 *          完成、轴对齐在其后，见 MagSensor.h 标定管线）。
 * @see     lib/MagCal/MagCalStore.h（Flash 持久化）、lib/MagCal/MagCalMath.h
 *          （paramsSanity：scale 必须 5–200 µT）、src/sensor_imu.cpp initMagnetometer
 */

#ifndef TANDEMVEC_MAG_CAL_DEFAULTS_H
#define TANDEMVEC_MAG_CAL_DEFAULTS_H

namespace magcal_defaults
{

// 是否启用编译期默认标定。完成一次合格的本机型标定后，填好下方参数并改为 true。
// 当前 false：纵列机型尚未完成上板椭球标定，保持「Flash 无效则未标定直通」行为。
static constexpr bool kValid = false;

// 硬铁偏移 b (uT, 芯片系) —— 待本机型标定后填写
static constexpr float kHardIron[3] = {0.0f, 0.0f, 0.0f};

// 软铁矩阵 M：c = scale·M·(raw − b) —— 待本机型标定后填写
static constexpr float kSoftIron[3][3] = {
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f}};

// 拟合出的总场强 (uT)，必须为物理量（约 5–200 uT，paramsSanity 会校验）
static constexpr float kScale = 0.0f;

} // namespace magcal_defaults

#endif // TANDEMVEC_MAG_CAL_DEFAULTS_H
