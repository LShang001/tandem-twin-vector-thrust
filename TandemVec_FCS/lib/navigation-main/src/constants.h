/**
 * @file    constants.h
 * @module  固件/导航（WGS-84 椭球与地球物理常量）
 * @brief   定义 WGS-84 参考椭球的几何/自转/引力常量，编译期常量零运行时开销
 * @details 输入：无（编译期常量）；处理：无；输出：`bfs` 命名空间下的
 *          static constexpr 常量，供 earth_model.h / transforms.h / ekf_15_state.h
 *          计算曲率半径、正常重力、地球自转率与大地坐标换算。
 *          不变量：全部为 constexpr，不占 RAM；单位与符号约定见本文件逐项注释。
 * @theory  WGS-84 椭球参数与正常重力常数 @ref REF-120 §7.2.1 表 7.2.1（WGS-84 行）；
 *          正常重力 Somigliana 公式 (7.2.17) @ref REF-120 §7.2.4 (7.2.17) p.178；
 *          地球自转角速度与引力常数取 ICD-GPS-200 规范值（见各常量行注释）
 * @frame   N/A（标量常量；使用处的坐标系见各函数 @frame）
 * @units   SI：长度 m，角速度 rad/s，引力常数 m³/s²，偏心率无量纲
 * @note    同名常量有“WGS-84 实测值”与“ICD-GPS-200 规范值”两套：
 *          `WE_RADPS`/`GM_M3PS2` 为 WGS-84 值，`WE_GPS_RADPS`/`GM_GPS_M3PS2`
 *          为 GPS 接口规范值（两者相对差 <1e-7，导航积分用前者）。
 * @see     earth_model.h、transforms.h、docs/08-参考资料/notes/01-捷联惯导理论.md
 */

#ifndef NAVIGATION_SRC_CONSTANTS_H_ // NOLINT
#define NAVIGATION_SRC_CONSTANTS_H_

namespace bfs
{

// ====================================================================================
// WGS-84 椭球几何参数
// ====================================================================================
// 关系式：f=(a−b)/a，b=a(1−f)，e²=2f−f²，Rn=a/√(1−e²sin²φ)，
//         Rm=a(1−e²)/(1−e²sin²φ)^(3/2)。
// 来源：@ref REF-120 §7.2.1 表 7.2.1（WGS-84 行：a=6378137 m、1/f=298.257）
// 实现：earth_model.h::earthrad_transverse_m() / earthrad_meridonal_m()；transforms.h::lla2ecef()
// 等价变换：无（WGS-84 定义式，不涉及坐标系差异）
// 数值证据：@test test_host/test_ekf_15state.cpp T2/T3 静止传播 100/200 步
//           （lla 变化率经 Rn/Rm 积分，纬度/高度保持有界，量级与 WGS-84 一致）
// ------------------------------------------------------------------------------------

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
// 用途：ω_ie 参与比力方程的哥氏项与姿态传播的导航系旋转补偿（见 earth_model.h 的
//       earthrate()）；GM 参与正常重力与轨道类计算。两套取值来源不同：
//       − `WE_RADPS`/`GM_M3PS2`：WGS-84 椭球的实测/导出值，导航积分使用这一套；
//       − `WE_GPS_RADPS`/`GM_GPS_M3PS2`：ICD-GPS-200 规范发布值，与 GPS 接收机
//         的 ICD 接口对齐时使用。两套相对差 <1e-7，200 Hz 导航周期内累计差异
//         远小于 IMU 噪声，不可混用但可安全替换。
// 来源：@ref REF-120 §7.2.4 p.178（地球自转与重力模型上下文）；
//       ICD-GPS-200 表 20-IV（WGS-84 参数）
// 实现：earth_model.h::earthrate()、ComputeEarthModelTerms()
// 等价变换：无
// 数值证据：@test test_host/test_ekf_15state.cpp T1/T2（初始零偏扣除使用
//           earthrate+navrate 的物理角速度，静止时速度保持 <0.02 m/s）
// ------------------------------------------------------------------------------------

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
