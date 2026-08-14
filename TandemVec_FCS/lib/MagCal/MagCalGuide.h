/**
 * @file MagCalGuide.h
 * @brief 大疆式"两圈正交旋转"引导校准状态机（纯平台无关，宿主机可测）
 *
 * 操作流程（与大疆 DJI GO 指南针校准同构，参考 ArduPilot CompassCalibrator）：
 *   阶段1 HORIZONTAL：机体水平，绕竖直轴(≈z_b)匀速转一圈；
 *   阶段2 NOSE_DOWN ：机头朝下，绕竖直轴(≈x_b)再转一圈；
 *   两个正交平面上的磁矢量轨迹各画一个圆，两个圆解出三维硬铁偏移——
 *   两圈数据合并后由 MagCalMath 的 9 参数椭球拟合一次性解硬铁+软铁。
 *
 * 判定输入（机体 FRD，由调用方每 IMU 帧喂入）：
 *   accFrd  [m/s²]  加速度计（静止/匀速时 ≈ 重力反方向）
 *   gyroFrd [rad/s] 陀螺仪
 *   magFrd  [uT]    磁力计（含已生效标定；仅用方向做扇区覆盖判定）
 *
 * 完成判据：
 *   - 姿态门：|acc| ∈ [0.85g, 1.15g] 且主导轴分量 ≥ 0.7·|acc|
 *     （阶段1 主导轴 z_b：az≈+g；阶段2 主导轴 x_b：ax≈+g）
 *   - 旋转门：主导轴角速度 ∈ [0.3, 6.0] rad/s（~17-344°/s），
 *     次轴 |ω| < 0.45·主导轴（确实在绕竖直轴转而非乱晃）
 *   - 覆盖门：磁方位角按 45° 分 8 扇区，覆盖 ≥7 扇区（≥315°）判定一圈完成
 *
 * 门限均为宽松经验值：判定失败只是"暂停采集"，扇区不清零，
 * 用户拿不稳、停顿、换手都不影响，转满角度即通过。
 */

#ifndef MAG_CAL_GUIDE_H
#define MAG_CAL_GUIDE_H

#include <stdint.h>
#include <math.h>

namespace magcal {

enum class GuideStage : uint8_t
{
    IDLE = 0,    //< 未启动
    HORIZONTAL,  //< 阶段1：水平旋转（绕 z_b）
    NOSE_DOWN,   //< 阶段2：机头朝下旋转（绕 x_b）
    DONE,        //< 两圈完成（拟合由调用方执行）
    FAILED,      //< 拟合质量门未过（调用方设置）
};

struct GuideStatus
{
    GuideStage stage = GuideStage::IDLE;
    uint8_t sectorsDone = 0; //< 当前阶段已覆盖扇区数（0..8）
    uint8_t totalSectors = 8;
};

class MagCalGuide
{
public:
    static constexpr uint8_t kSectors = 8;         //< 45° 一扇区
    static constexpr uint8_t kSectorsRequired = 7; //< 覆盖 ≥315° 判定一圈完成

    // ---- 判定门限（SI；与 TandemVec_Config.h 的 g=9.79 一致，容差 ±15%）----
    static constexpr float kPi = 3.14159265358979f;
    static constexpr float kGravity = 9.79f;
    static constexpr float kAccelMin = 0.85f * kGravity;
    static constexpr float kAccelMax = 1.15f * kGravity;
    static constexpr float kAxisDominance = 0.70f; //< 主导轴分量/|acc| 下限
    static constexpr float kRotMin = 0.30f;        //< 主导轴角速度下限 (~17°/s)
    static constexpr float kRotMax = 6.0f;         //< 上限 (~344°/s)
    static constexpr float kSecondaryRatio = 0.45f; //< 次轴 |ω| 相对主导轴上限

    void start()
    {
        _stage = GuideStage::HORIZONTAL;
        _sectorMask = 0;
    }

    void reset() { _stage = GuideStage::IDLE; _sectorMask = 0; }

    GuideStage stage() const { return _stage; }
    uint8_t sectorsDone() const;
    void status(GuideStatus &out) const
    {
        out.stage = _stage;
        out.sectorsDone = sectorsDone();
        out.totalSectors = kSectors;
    }

    void setFailed() { _stage = GuideStage::FAILED; }

    /**
     * @brief 每 IMU 帧调用一次（建议 1kHz，200Hz 也可——扇区判定逐帧即可）
     * @param accFrd   机体 FRD 加速度 (m/s²)
     * @param gyroFrd  机体 FRD 角速度 (rad/s)
     * @param magFrd   机体 FRD 磁场 (uT)，方向参考
     * @param magFresh magFrd 是否为新样本（旧样本不重复计入扇区）
     * @return true  本帧姿态/旋转判定通过 → 调用方应把对应芯片系原始样本
     *               送入 MagCalCollector（其内部自带时间/间距抽稀）
     * @return false 判定未过（正常，继续转）/ 已完成 / 未启动
     */
    bool update(const float accFrd[3], const float gyroFrd[3],
                const float magFrd[3], bool magFresh);

private:
    GuideStage _stage = GuideStage::IDLE;
    uint8_t _sectorMask = 0;

    bool checkRotation(float wMain, float w1, float w2) const
    {
        const float a = fabsf(wMain);
        if (!(a >= kRotMin && a <= kRotMax))
            return false;
        return fabsf(w1) < kSecondaryRatio * a && fabsf(w2) < kSecondaryRatio * a;
    }

    void recordSector(float angle)
    {
        // 方位角 [-π, π) → 扇区 0..7
        int sec = (int)floorf((angle + kPi) / (2.0f * kPi) * (float)kSectors);
        if (sec < 0)
            sec = 0;
        if (sec >= kSectors)
            sec = kSectors - 1;
        _sectorMask |= (uint8_t)(1u << sec);
    }

    void advanceIfCovered()
    {
        if (sectorsDone() < kSectorsRequired)
            return;
        if (_stage == GuideStage::HORIZONTAL)
        {
            _stage = GuideStage::NOSE_DOWN;
            _sectorMask = 0;
        }
        else if (_stage == GuideStage::NOSE_DOWN)
        {
            _stage = GuideStage::DONE;
        }
    }
};

inline uint8_t MagCalGuide::sectorsDone() const
{
    uint8_t m = _sectorMask;
    uint8_t n = 0;
    while (m)
    {
        n += (uint8_t)(m & 1u);
        m >>= 1;
    }
    return n;
}

inline bool MagCalGuide::update(const float accFrd[3], const float gyroFrd[3],
                                const float magFrd[3], bool magFresh)
{
    if (_stage != GuideStage::HORIZONTAL && _stage != GuideStage::NOSE_DOWN)
        return false; // IDLE/DONE/FAILED：不动作

    // ---- 1. 姿态门：近似匀速/静止 ----
    const float accNorm = sqrtf(accFrd[0] * accFrd[0] + accFrd[1] * accFrd[1] +
                                accFrd[2] * accFrd[2]);
    if (!(accNorm >= kAccelMin && accNorm <= kAccelMax))
        return false;

    float wMain, w1, w2;
    if (_stage == GuideStage::HORIZONTAL)
    {
        // 水平：重力沿 +z_b（acc 测的是支撑力，z_b 指向下）
        if (accFrd[2] < kAxisDominance * accNorm)
            return false;
        wMain = gyroFrd[2];
        w1 = gyroFrd[0];
        w2 = gyroFrd[1];
    }
    else // NOSE_DOWN
    {
        // 机头朝下：重力沿 +x_b
        if (accFrd[0] < kAxisDominance * accNorm)
            return false;
        wMain = gyroFrd[0];
        w1 = gyroFrd[1];
        w2 = gyroFrd[2];
    }

    // ---- 2. 旋转门：绕竖直轴匀速转 ----
    if (!checkRotation(wMain, w1, w2))
        return false;

    // ---- 3. 扇区覆盖（用磁方位角；方向数据即可，不依赖标定精度）----
    if (magFresh)
    {
        if (_stage == GuideStage::HORIZONTAL)
        {
            recordSector(atan2f(magFrd[1], magFrd[0]));
        }
        else
        {
            recordSector(atan2f(magFrd[2], magFrd[1]));
        }
        advanceIfCovered();
    }

    return true;
}

} // namespace magcal

#endif // MAG_CAL_GUIDE_H
