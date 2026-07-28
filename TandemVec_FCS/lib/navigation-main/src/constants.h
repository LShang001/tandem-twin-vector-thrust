/*
 * WGS-84 地球物理常量
 * ==================
 *
 * 定义 WGS-84 参考椭球模型的核心几何参数和物理常数，是整个导航库
 * 的数值基础：椭球半长轴、扁率、偏心率（用于卯酉/子午圈曲率半径和
 * 大地坐标换算）、地球自转角速度（用于捷联惯导旋转补偿和陀螺零偏估计）、
 * 地球引力常数（用于重力模型和轨道力学）。所有常量按 WGS-84 / ICD-GPS-200
 * 标准取值，在编译期以 static constexpr 确定，零运行时开销。
 */

#ifndef NAVIGATION_SRC_CONSTANTS_H_ // NOLINT
#define NAVIGATION_SRC_CONSTANTS_H_

namespace bfs
{

// ====================================================================================
// WGS-84 椭球几何参数
// ====================================================================================

/* 椭球半长轴（赤道半径），单位 m */
static constexpr double SEMI_MAJOR_AXIS_LENGTH_M = 6378137.0;

/* 扁率 (Flattening)：f = (a - b) / a */
static constexpr double FLATTENING = 1.0 / 298.257223563;

/* 半短轴（极半径），由 a 和 f 导出：b = a * (1 - f)，单位 m */
static constexpr double SEMI_MINOR_AXIS_LENGTH_M = 6356752.3142;

/* 第一偏心率 e，单位无量纲 */
static constexpr double ECC = 8.1819190842622e-2;

/* 第一偏心率平方 e²，广泛用于卯酉圈/子午圈曲率半径和坐标换算 */
static constexpr double ECC2 = 6.6943799901414e-3;

// ====================================================================================
// 地球自转与引力参数
// ====================================================================================

/* 地球自转角速度，单位 rad/s */
static constexpr double WE_RADPS = 7292115.0e-11;

/* ICD-GPS-200 规范取值的地球自转角速度，单位 rad/s */
static constexpr double WE_GPS_RADPS = 7292115.1467e-11;

/* 地球引力常数 (GM)，单位 m³/s² */
static constexpr double GM_M3PS2 = 3986004.418e8;

/* ICD-GPS-200 规范取值的地球引力常数，单位 m³/s² */
static constexpr double GM_GPS_M3PS2 = 3986005.0e8;

} // namespace bfs

#endif // NAVIGATION_SRC_CONSTANTS_H_ NOLINT
