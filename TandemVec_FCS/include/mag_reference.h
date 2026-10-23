/**
 * @file    mag_reference.h
 * @module  固件/导航（磁矢量参考场）
 * @brief   给出本机 home 站点 NED 真北参考磁场，供磁矢量门控/EKF 使用
 * @details 输入：无（编译期常量）；处理：不运行球谐模型，只提供离线生成的
 *          WMM/IGRF 站点值；输出：`MagReferenceNed`（N/E/D、水平/总场、倾角、
 *          磁偏角、站点坐标、valid）。
 *          不变量：D 分量按 NED **向下为正**；磁偏角按真北→磁北、东偏为正；
 *          站点常量离开 home 50 km 后由调用方判定失效（本文件只提供距离工具）。
 * @theory  地磁场矢量分解为水平分量 H 与垂直分量 D，倾角 I=atan2(D,H)；
 *          真北航向 = 磁北航向 + 磁偏角（东偏为正）
 *          @ref REF-120 §9.9.1 p.323–324（磁矢量定方位）、
 *          notes/04-航向与磁辅助（磁偏角/倾角与航向可观测性）
 * @frame   NED（x 北 / y 东 / z 下）；机体系 FRD 由 EKF 的 C^n_b 转换
 * @units   磁场 µT，角度 rad，站点 deg，距离 km
 * @note    ★ 这是**站点固定**参考场，不是全球 WMM 运行时模型。当前值由
 *          IGRF-14（2026.0，28.2°N、112.9°E、椭球高 0.05 km）离线计算；
 *          换场地/跨磁异常区必须用 `tools/gen_mag_reference.py` 重新生成，
 *          否则导航层会因 `valid=false` 或模长/倾角门控拒绝磁量测。
 * @see     `src/navigation_task.cpp` 磁量测段、`include/mag_measurement_gate.h`、
 *          `tools/gen_mag_reference.py`
 */

#ifndef TANDEMVEC_MAG_REFERENCE_H
#define TANDEMVEC_MAG_REFERENCE_H

#include <cmath>
#include <cstdint>

namespace bfs
{

struct MagReferenceNed
{
    float field_ut[3];       //< N/E/D，µT；D 向下为正
    float horizontal_ut;     //< 水平分量 H，µT
    float total_ut;          //< 总场 F，µT
    float dip_rad;           //< 倾角 I=atan2(D,H)，rad
    float declination_rad;   //< 磁偏角（真北→磁北，东偏为正），rad
    float site_lat_deg;      //< 生成站点纬度，deg
    float site_lon_deg;      //< 生成站点经度，deg
    bool valid;              //< 是否可作为参考场
};

/*
 * 2026.0 IGRF-14 站点值（长沙近似 home：28.2°N, 112.9°E, 0.05 km）。
 * 重新生成：PYTHONPATH=<ppigrf_target> py -3.12 tools/gen_mag_reference.py
 * 生成脚本会打印同格式常量；不要手改方向符号。
 */
static constexpr float kMagReferencePi = 3.14159265358979323846f;
static constexpr float kMagReferenceDegToRad =
    kMagReferencePi / 180.0f;

static constexpr MagReferenceNed kDefaultMagReference = {
    {35.075f, -2.523f, 34.079f}, // N/E/D µT
    35.166f,                     // H
    48.969f,                     // F
    0.769696f,                   // dip 44.100°（D 向下为正）
    -0.071801f,                  // declination -4.114°（东偏为正）
    28.2f,                       // site latitude
    112.9f,                      // site longitude
    true};

namespace magref
{

inline float degToRad(const float deg)
{
    return deg * kMagReferenceDegToRad;
}

inline float radToDeg(const float rad)
{
    return rad / kMagReferenceDegToRad;
}

/**
 * @brief 两个经纬度站点间的球面距离（haversine），km
 * @param[in] lat1_deg 纬度 1，deg
 * @param[in] lon1_deg 经度 1，deg
 * @param[in] lat2_deg 纬度 2，deg
 * @param[in] lon2_deg 经度 2，deg
 * @return 球面距离 km（地球平均半径 6371 km）
 * @note 只用于“参考场是否仍在 home 附近”的粗门控，不用于导航定位。
 */
inline float siteDistanceKm(const float lat1_deg, const float lon1_deg,
                            const float lat2_deg, const float lon2_deg)
{
    constexpr float kEarthRadiusKm = 6371.0f;
    const float dlat = degToRad(lat2_deg - lat1_deg);
    const float dlon = degToRad(lon2_deg - lon1_deg);
    const float a = std::sin(0.5f * dlat) * std::sin(0.5f * dlat) +
                    std::cos(degToRad(lat1_deg)) *
                        std::cos(degToRad(lat2_deg)) *
                        std::sin(0.5f * dlon) * std::sin(0.5f * dlon);
    const float clamped_a = std::fmax(0.0f, std::fmin(1.0f, a));
    const float c = 2.0f * std::atan2(std::sqrt(clamped_a),
                                      std::sqrt(1.0f - clamped_a));
    return kEarthRadiusKm * c;
}

} // namespace magref
} // namespace bfs

#endif // TANDEMVEC_MAG_REFERENCE_H
