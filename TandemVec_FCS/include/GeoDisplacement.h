// 用于根据WGS84经纬度计算高精度的北东位移。
// 此库提供了高精度的WGS84椭球坐标转换和ENU坐标计算功能，适用于需要精确位置测量的应用。
// 注意：本库不包含GPS数据处理功能，仅用于计算位置偏移。
#ifndef GEO_DISPLACEMENT_H
#define GEO_DISPLACEMENT_H

#include <math.h> // 提供sin、cos、sqrt等数学函数

// WGS84地球椭球常量参数（使用double提升精度）
const double WGS84_A = 6378137.0;                  // 长半轴（米）
const double WGS84_F = 1.0 / 298.257223563;        // 扁率
const double WGS84_E2 = WGS84_F * (2.0 - WGS84_F); // 第一偏心率平方

// ECEF坐标结构体
struct ECEF_Coord
{
    double X;
    double Y;
    double Z;
};

// ENU坐标结构体（包含东、北、天位移）
struct ENU_Coord
{
    double East;  // 东向位移（米）
    double North; // 北向位移（米）
    double Up;    // 天向位移（米）
};

/**
 * 将WGS84经纬度高（LLA）转换为ECEF坐标。
 *
 * 参数:
 *   lat: 纬度（度）
 *   lon: 经度（度）
 *   alt: 高度（米）
 *
 * 返回:
 *   ECEF_Coord: ECEF坐标 {X, Y, Z}（米）
 *
 * 注意: 输入度数自动转换为弧度；处理负纬度（南半球）。
 */
inline ECEF_Coord lla_to_ecef(double lat, double lon, double alt)
{
    double lat_rad = lat * M_PI / 180.0;                                     // 转换为弧度（M_PI来自math.h）
    double lon_rad = lon * M_PI / 180.0;                                     // 转换为弧度
    double N = WGS84_A / sqrt(1.0 - WGS84_E2 * sin(lat_rad) * sin(lat_rad)); // 主曲率半径N
    ECEF_Coord ecef;
    ecef.X = (N + alt) * cos(lat_rad) * cos(lon_rad);     // X分量
    ecef.Y = (N + alt) * cos(lat_rad) * sin(lon_rad);     // Y分量
    ecef.Z = (N * (1.0 - WGS84_E2) + alt) * sin(lat_rad); // Z分量
    return ecef;
}

/**
 * 将ECEF差值转换为ENU坐标，使用预计算的三角函数值。
 *
 * 参数:
 *   dx, dy, dz: ECEF坐标差值（米）
 *   sin_ref_lat, cos_ref_lat: 参考纬度的sin/cos值
 *   sin_ref_lon, cos_ref_lon: 参考经度的sin/cos值
 *
 * 返回:
 *   ENU_Coord: ENU坐标 {East, North, Up}（米）
 *
 * 注意: 直接展开旋转矩阵计算，避免数组开销，提高嵌入式效率。
 */
inline ENU_Coord ecef_to_enu(double dx, double dy, double dz,
                      double sin_ref_lat, double cos_ref_lat,
                      double sin_ref_lon, double cos_ref_lon)
{
    ENU_Coord enu;
    // 东向位移: -sin(λ₀)ΔX + cos(λ₀)ΔY
    enu.East = -sin_ref_lon * dx + cos_ref_lon * dy;
    // 北向位移: -sin(φ₀)cos(λ₀)ΔX - sin(φ₀)sin(λ₀)ΔY + cos(φ₀)ΔZ
    enu.North = -sin_ref_lat * cos_ref_lon * dx - sin_ref_lat * sin_ref_lon * dy + cos_ref_lat * dz;
    // 天向位移: cos(φ₀)cos(λ₀)ΔX + cos(φ₀)sin(λ₀)ΔY + sin(φ₀)ΔZ
    enu.Up = cos_ref_lat * cos_ref_lon * dx + cos_ref_lat * sin_ref_lon * dy + sin_ref_lat * dz;
    return enu;
}

/**
 * 计算相对于原点的北东位移。
 *
 * 参数:
 *   ref_lat, ref_lon, ref_alt: 原点（参考点）纬度、经度、高度（度，度，米）
 *   cur_lat, cur_lon, cur_alt: 当前点纬度、经度、高度（度，度，米）
 *
 * 返回:
 *   ENU_Coord: 包含东、北、天位移的结构体（米）
 *
 * 注意: 若高度未知，可设为0；函数处理负值和跨日期线（经度模360需外部处理）。
 *       在AVR平台（如UNO），double为32位精度；在ARM平台（如ESP32），为64位。
 */
inline ENU_Coord calculate_ne_displacement(double ref_lat, double ref_lon, double ref_alt,
                                    double cur_lat, double cur_lon, double cur_alt)
{
    // 计算参考和当前ECEF坐标
    ECEF_Coord ref_ecef = lla_to_ecef(ref_lat, ref_lon, ref_alt);
    ECEF_Coord cur_ecef = lla_to_ecef(cur_lat, cur_lon, cur_alt);

    // 计算ECEF差值
    double dx = cur_ecef.X - ref_ecef.X;
    double dy = cur_ecef.Y - ref_ecef.Y;
    double dz = cur_ecef.Z - ref_ecef.Z;

    // 预计算参考点弧度和三角函数值，避免重复计算
    double ref_lat_rad = ref_lat * M_PI / 180.0;
    double ref_lon_rad = ref_lon * M_PI / 180.0;
    double sin_ref_lat = sin(ref_lat_rad);
    double cos_ref_lat = cos(ref_lat_rad);
    double sin_ref_lon = sin(ref_lon_rad);
    double cos_ref_lon = cos(ref_lon_rad);

    // 转换为ENU
    return ecef_to_enu(dx, dy, dz, sin_ref_lat, cos_ref_lat, sin_ref_lon, cos_ref_lon);
}

#endif // GEO_DISPLACEMENT_H
