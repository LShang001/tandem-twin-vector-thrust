/**
 * @file MagMath.h
 * @brief 磁力计通用解析与轴对齐数学层（纯平台无关，宿主机可直接回归测试）
 *
 * 本头文件不依赖 Arduino / Wire / Eigen，只做三件事：
 *   1. 各芯片 16 位原始读数 -> 物理单位 uT（灵敏度来自各自数据手册）
 *   2. 芯片坐标系 -> 机体 FRD(前-右-下) 的整数旋转矩阵对齐
 *   3. 硬铁偏置（offset）扣除
 *
 * 设计意图：IST8310 / QMC5883L / 未来任何磁力计驱动都只负责"从芯片读出
 * 芯片坐标系下的 uT 向量"，坐标系对齐与偏置统一在本层完成，保证更换磁力计
 * 型号时只需要改一张旋转矩阵常量表。
 *
 * 轴对齐约定（与 AGENTS.md 钉死的 FRD 机体系一致）：
 *   body_x = 前(+x_b)   body_y = 右(+y_b)   body_z = 下(+z_b)
 * 地磁场在北半球指"北 + 下"，因此静止平放机头朝北时：
 *   FRD 磁向量 = (+|B|cos(incl), 0, +|B|sin(incl))，z 分量必须为正。
 * 这是判定任何新芯片轴映射正误的物理判据（见 test_mag_parse.cpp）。
 */

#ifndef MAG_SENSOR_MAG_MATH_H
#define MAG_SENSOR_MAG_MATH_H

#include <stdint.h>

namespace magmath {

/* ============================================================================
 * 1. 灵敏度换算（数据手册典型值）
 * ============================================================================ */

// IST8310：灵敏度 0.3 uT/LSB（数据手册典型值）。
// 旧注释 "330 LSB/Gauss -> 100/330 = 0.30303" 与实际灵敏度 1/0.3 = 333.33 LSB/G
// 不符，会引入 +1% 系统误差，已修正为 0.3（见 lib/IST8310/IST8310.h）。
inline float ist8310LsToUt(int16_t raw)
{
    return static_cast<float>(raw) * 0.3f;
}

// QMC5883L：灵敏度由量程(RNG)决定（数据手册）：
//   RNG=0 (±2G)  -> 12000 LSB/Gauss
//   RNG=1 (±8G)  ->  3000 LSB/Gauss
// 1 Gauss = 100 uT
constexpr float kQmc5883lLsbg[2] = {12000.0f, 3000.0f}; //< LSB/Gauss

inline float qmc5883lLsToUt(int16_t raw, uint8_t rangeIndex)
{
    return static_cast<float>(raw) * (100.0f / kQmc5883lLsbg[rangeIndex & 0x01u]);
}

/* ============================================================================
 * 2. 6 字节小端数据块解码
 * ============================================================================ */

/**
 * @brief QMC5883L 6 字节数据解码（buf 从 X_LSB(0x00) 开始：XL XH YL YH ZL ZH）
 * @param buf       6 字节原始数据
 * @param rangeIndex 量程索引 0=±2G, 1=±8G
 * @param out       输出三轴磁场 (uT)，芯片坐标系
 * @note 16 位二进制补码，小端；先经 uint16_t 组包再转 int16_t，
 *       避免左移进位到符号位的实现定义行为（对齐现有 IST8310 驱动风格）。
 */
inline void qmc5883lDecode(const uint8_t buf[6], uint8_t rangeIndex, float out[3])
{
    const int16_t rx = static_cast<int16_t>(static_cast<uint16_t>((buf[1] << 8) | buf[0]));
    const int16_t ry = static_cast<int16_t>(static_cast<uint16_t>((buf[3] << 8) | buf[2]));
    const int16_t rz = static_cast<int16_t>(static_cast<uint16_t>((buf[5] << 8) | buf[4]));
    out[0] = qmc5883lLsToUt(rx, rangeIndex);
    out[1] = qmc5883lLsToUt(ry, rangeIndex);
    out[2] = qmc5883lLsToUt(rz, rangeIndex);
}

/**
 * @brief IST8310 6 字节数据解码（buf 从 DATAXL(0x03) 开始：XL XH YL YH ZL ZH）
 */
inline void ist8310Decode(const uint8_t buf[6], float out[3])
{
    const int16_t rx = static_cast<int16_t>(static_cast<uint16_t>((buf[1] << 8) | buf[0]));
    const int16_t ry = static_cast<int16_t>(static_cast<uint16_t>((buf[3] << 8) | buf[2]));
    const int16_t rz = static_cast<int16_t>(static_cast<uint16_t>((buf[5] << 8) | buf[4]));
    out[0] = ist8310LsToUt(rx);
    out[1] = ist8310LsToUt(ry);
    out[2] = ist8310LsToUt(rz);
}

/* ============================================================================
 * 3. 芯片系 -> 机体 FRD 的整数旋转矩阵 + 偏置
 * ============================================================================ */

/**
 * @brief 通用轴对齐：body[i] = R[i][0]*chip0 + R[i][1]*chip1 + R[i][2]*chip2 - off[i]
 * @param chip 芯片坐标系三轴磁场 (uT)
 * @param rot  3x3 旋转矩阵（元素只取 +1/0/-1，覆盖任意坐标轴重排+反向）
 * @param off  机体 FRD 系下的硬铁偏置 (uT)，无标定时全 0
 * @param out  输出机体 FRD (前-右-下) 磁场 (uT)
 */
inline void magAlignToBody(const float chip[3], const int8_t rot[3][3],
                           const float off[3], float out[3])
{
    for (int i = 0; i < 3; i++)
    {
        out[i] = static_cast<float>(rot[i][0]) * chip[0] +
                 static_cast<float>(rot[i][1]) * chip[1] +
                 static_cast<float>(rot[i][2]) * chip[2] -
                 off[i];
    }
}

/*
 * ★ 芯片安装方向常量表（换磁力计型号/改安装方向时只改这里）
 *
 * IST8310（本机实测安装约定，与 sensor_imu.cpp 历史 FLU 映射 (-y,-x,+z) 严格一致）：
 *   芯片 X -> 机体右(+Y)；芯片 Y -> 机尾(-X)；芯片 Z -> 上(-Z)
 *   => body_x = -chip_y, body_y = +chip_x, body_z = -chip_z
 */
static constexpr int8_t kAxisIst8310ToFrd[3][3] = {
    {0, -1, 0},
    {1, 0, 0},
    {0, 0, -1},
};

/*
 * QMC5883L 默认（★ 待上板标定，硬件未接）：
 *   假设模块水平安装（芯片面朝上）、芯片 X 朝机头、芯片 Y 朝右、芯片 Z 朝上。
 *   => body_x = +chip_x, body_y = +chip_y, body_z = -chip_z
 *
 * 上板标定方法（拿到模块后按此操作，结果回填本表）：
 *   1. 机头朝北平放：body FRD 应为 (+|B|cos(incl), 0, +|B|sin(incl))，z 必为正；
 *   2. 机头朝东平放：x≈0、y≈-|B|cos(incl)（北在左侧），据此判定 x/y 及符号；
 *   3. 机头朝下（悬停基态）：z 分量应翻为接近 0（磁力线水平分量在 x-y 面）。
 */
static constexpr int8_t kAxisQmc5883lToFrd[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, -1},
};

} // namespace magmath

#endif // MAG_SENSOR_MAG_MATH_H
