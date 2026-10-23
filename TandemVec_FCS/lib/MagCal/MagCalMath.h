/**
 * @file MagCalMath.h
 * @brief 磁力计椭球标定数学层（纯平台无关，宿主机可直接回归测试）
 *
 * 算法：最小二乘椭球拟合（Li & Griffiths 形式的 9 参数二次型 + 正定约束），
 *   1. 采集芯片系原始样本（未做任何轴映射/标定的原始 uT 数据）；
 *   2. 拟合椭球  xᵀQx + uᵀx = 1：
 *        硬铁偏移  b = −½·Q⁻¹u
 *        半径平方  s = 1 + bᵀQb
 *   3. Jacobi 特征分解 Q = VΛVᵀ（Q 必须正定，否则拒绝）；
 *   4. 白化矩阵  M = Λ^{1/2}·Vᵀ / √s（满足 MᵀM = Q/s），
 *      使 c = M·(x−b) 在椭球面上恒有 |c| = 1（被拉伸的轴压回单位球）。
 *   5. 质量门通过后把 scale 设为平均原始场强（µT），因此对上层输出
 *      c = scale·M·(x−b) 是物理磁场（|c|≈50 µT），不是单位球。
 *
 * 标定在"芯片系"进行（与安装方向的轴映射解耦——轴映射表改了标定依然有效），
 * 应用顺序： raw(芯片) → 标定(芯片系) → 轴映射(机体FRD)，由 MagSensor 层保证。
 *
 * 质量门（全部通过才判定标定有效，阈值均为保守经验值，实际值随结果打印）：
 *   - 样本数 ≥ 30
 *   - 空间覆盖：样本散点矩阵 λmin/λmax ≥ 1e-3（拒绝共面/共线采集）
 *   - Q 正定且 λmax/λmin ≤ 25（拒绝退化椭球/病态数据）
 *   - 校正后残差 RMS（||c|−1| 均值）≤ 0.15
 *
 * 参考：Y. Petrov ellipsoid_fit（MATLAB 社区经典实现）的 9 参数二次型表述、
 * QGroundControl/ArduPilot 板载磁校准的椭球拟合思路（本实现为独立重写）。
 */

#ifndef MAG_CAL_MAG_CAL_MATH_H
#define MAG_CAL_MAG_CAL_MATH_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

namespace magcal {

/* ============================================================================
 * 标定参数（芯片系）
 * ============================================================================ */

struct MagCalParams
{
    float hardIron[3];   //< 硬铁偏移 b (uT, 芯片系)
    float softIron[3][3]; //< 软铁矩阵 M：c = M·(raw − b)，|c| = 1（单位球）
    float scale;         //< 输出场强缩放 (uT)；有效标定必须为物理场强（约 5–200 uT）
    bool valid;          //< 是否有效（身份参数时 false = 未标定）

    void identity()
    {
        hardIron[0] = hardIron[1] = hardIron[2] = 0.0f;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                softIron[i][j] = (i == j) ? 1.0f : 0.0f;
        scale = 1.0f;
        valid = false;
    }
};

/**
 * @brief 标定拟合质量报告（diagnostics）
 */
struct MagCalStats
{
    uint16_t count;       //< 参与拟合的样本数
    float residualRms;    //< 校正后 ||c|−1| 的均方根（越小越好，典型 <0.03）
    float eigRatio;       //< Q 的 λmax/λmin（越接近 1 椭球越圆）
    float coverageRatio;  //< 样本散点 λmin/λmax（3D 覆盖度）
    float fieldMagUt;     //< 拟合出的原始磁场模长 √s (uT)
};

/* ============================================================================
 * 应用与基础工具
 * ============================================================================ */

/**
 * @brief 应用标定：out = scale·M·(in − b)，芯片系
 * @note scale ≤ 0 时按 1.0 处理，仅用于 `fit()` 内部计算单位球残差；
 *       有效标定的 scale 必须是物理场强（µT），不能持久化为 0。
 */
inline void applyCal(const MagCalParams &p, const float in[3], float out[3])
{
    const float d[3] = {in[0] - p.hardIron[0], in[1] - p.hardIron[1], in[2] - p.hardIron[2]};
    const float s = (p.scale > 0.0f) ? p.scale : 1.0f;
    out[0] = s * (p.softIron[0][0] * d[0] + p.softIron[0][1] * d[1] + p.softIron[0][2] * d[2]);
    out[1] = s * (p.softIron[1][0] * d[0] + p.softIron[1][1] * d[1] + p.softIron[1][2] * d[2]);
    out[2] = s * (p.softIron[2][0] * d[0] + p.softIron[2][1] * d[1] + p.softIron[2][2] * d[2]);
}

/**
 * @brief 标定参数合理性校验（加载持久化参数时防御坏块/位翻转）
 * @note 有效标定的 scale 必须是物理场强；旧版本 `scale=0` 的单位球标定
 *       会被拒绝，必须重新 `magcal fit/save` 后才能用于导航。
 */
inline bool paramsSanity(const MagCalParams &p)
{
    if (!p.valid)
        return false;
    if (!isfinite(p.scale) || p.scale < 5.0f || p.scale > 200.0f)
        return false;
    for (int i = 0; i < 3; i++)
    {
        if (!isfinite(p.hardIron[i]) || fabsf(p.hardIron[i]) > 10000.0f)
            return false;
        for (int j = 0; j < 3; j++)
        {
            const float v = p.softIron[i][j];
            if (!isfinite(v) || fabsf(v) > 1000.0f)
                return false;
        }
    }
    return true;
}

/**
 * @brief IEEE 802.3 CRC32（反射多项式 0xEDB88320，初值/终值异或 0xFFFFFFFF）
 *        用于持久化记录完整性校验（标准测试向量 "123456789" → 0xCBF43926）。
 */
inline uint32_t crc32(const void *data, size_t len, uint32_t crc = 0xFFFFFFFFu)
{
    const uint8_t *p = (const uint8_t *)data;
    while (len--)
    {
        crc ^= *p++;
        for (int k = 0; k < 8; k++)
        {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

/* ============================================================================
 * 3×3 对称矩阵 Jacobi 特征分解（数值计算经典实现，自包含无外部依赖）
 * ============================================================================ */

/**
 * @brief 3×3 实对称矩阵特征分解 A = V·diag(w)·Vᵀ
 * @param a   输入矩阵（会被破坏），仅使用上三角
 * @param w   输出特征值，按降序排列
 * @param v   输出特征向量（列向量），与 w 一一对应
 */
inline void jacobiEigen3(const float aIn[3][3], float w[3], float v[3][3])
{
    float a[3][3];
    memcpy(a, aIn, sizeof(a));

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            v[i][j] = (i == j) ? 1.0f : 0.0f;

    for (int sweep = 0; sweep < 32; sweep++)
    {
        const float off = fabsf(a[0][1]) + fabsf(a[0][2]) + fabsf(a[1][2]);
        if (off < 1e-12f)
            break;

        for (int p = 0; p < 3; p++)
        {
            for (int q = p + 1; q < 3; q++)
            {
                const float apq = a[p][q];
                if (fabsf(apq) < 1e-30f)
                    continue;

                const float app = a[p][p];
                const float aqq = a[q][q];
                // 稳定的旋转角计算（避免 θ 溢出）
                const float theta = (aqq - app) / (2.0f * apq);
                float t = (theta >= 0.0f)
                              ? 1.0f / (theta + sqrtf(theta * theta + 1.0f))
                              : 1.0f / (theta - sqrtf(theta * theta + 1.0f));
                if (!isfinite(t))
                    t = 1.0f;
                const float c = 1.0f / sqrtf(t * t + 1.0f);
                const float s = t * c;

                a[p][p] = app - t * apq;
                a[q][q] = aqq + t * apq;
                a[p][q] = a[q][p] = 0.0f;

                for (int k = 0; k < 3; k++)
                {
                    if (k == p || k == q)
                        continue;
                    const float akp = a[k][p];
                    const float akq = a[k][q];
                    a[k][p] = a[p][k] = c * akp - s * akq;
                    a[k][q] = a[q][k] = s * akp + c * akq;
                }

                for (int k = 0; k < 3; k++)
                {
                    const float vkp = v[k][p];
                    const float vkq = v[k][q];
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
        }
    }

    w[0] = a[0][0];
    w[1] = a[1][1];
    w[2] = a[2][2];

    // 按特征值降序排列（交换列）
    for (int i = 0; i < 2; i++)
    {
        int m = i;
        for (int j = i + 1; j < 3; j++)
            if (w[j] > w[m])
                m = j;
        if (m != i)
        {
            const float tw = w[i];
            w[i] = w[m];
            w[m] = tw;
            for (int k = 0; k < 3; k++)
            {
                const float tv = v[k][i];
                v[k][i] = v[k][m];
                v[k][m] = tv;
            }
        }
    }
}

/**
 * @brief 3×3 矩阵求逆（高斯-若当，含主元判断；奇异返回 false）
 */
inline bool invert3(const float mIn[3][3], float mOut[3][3])
{
    float m[3][6];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
            m[i][j] = mIn[i][j];
        for (int j = 3; j < 6; j++)
            m[i][j] = (i == (j - 3)) ? 1.0f : 0.0f;
    }
    for (int col = 0; col < 3; col++)
    {
        int piv = col;
        for (int r = col + 1; r < 3; r++)
            if (fabsf(m[r][col]) > fabsf(m[piv][col]))
                piv = r;
        if (fabsf(m[piv][col]) < 1e-12f)
            return false;
        if (piv != col)
            for (int j = 0; j < 6; j++)
            {
                const float t = m[col][j];
                m[col][j] = m[piv][j];
                m[piv][j] = t;
            }
        const float inv = 1.0f / m[col][col];
        for (int j = 0; j < 6; j++)
            m[col][j] *= inv;
        for (int r = 0; r < 3; r++)
        {
            if (r == col)
                continue;
            const float f = m[r][col];
            if (f == 0.0f)
                continue;
            for (int j = 0; j < 6; j++)
                m[r][j] -= f * m[col][j];
        }
    }
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            mOut[i][j] = m[i][j + 3];
    return true;
}

/**
 * @brief 9×9 线性方程组求解（高斯-若当 + 部分主元；奇异返回 false）
 */
inline bool solve9(const float A[9][9], const float b[9], float x[9])
{
    float m[9][10];
    float scaleMax = 0.0f;
    for (int i = 0; i < 9; i++)
    {
        for (int j = 0; j < 9; j++)
        {
            m[i][j] = A[i][j];
            if (fabsf(m[i][j]) > scaleMax)
                scaleMax = fabsf(m[i][j]);
        }
        m[i][9] = b[i];
    }
    if (scaleMax < 1e-12f)
        return false;

    for (int col = 0; col < 9; col++)
    {
        int piv = col;
        for (int r = col + 1; r < 9; r++)
            if (fabsf(m[r][col]) > fabsf(m[piv][col]))
                piv = r;
        if (fabsf(m[piv][col]) < 1e-9f * scaleMax)
            return false;
        if (piv != col)
            for (int j = 0; j < 10; j++)
            {
                const float t = m[col][j];
                m[col][j] = m[piv][j];
                m[piv][j] = t;
            }
        const float inv = 1.0f / m[col][col];
        for (int j = 0; j < 10; j++)
            m[col][j] *= inv;
        for (int r = 0; r < 9; r++)
        {
            if (r == col)
                continue;
            const float f = m[r][col];
            if (f == 0.0f)
                continue;
            for (int j = 0; j < 10; j++)
                m[r][j] -= f * m[col][j];
        }
    }
    for (int i = 0; i < 9; i++)
        x[i] = m[i][9];
    return true;
}

/* ============================================================================
 * 采样收集器（带最小间距/最小间隔抽稀）
 * ============================================================================ */

class MagCalCollector
{
public:
    static constexpr uint16_t kMaxSamples = 400;   //< 样本上限（400×12B = 4.8KB RAM）
    static constexpr uint16_t kMinSamples = 30;    //< 拟合最小样本数
    static constexpr uint32_t kMinIntervalUs = 20000; //< 最小采样间隔 20ms（≤50Hz）
    static constexpr float kMinSpacingUt = 0.5f;   //< 与前一样本的最小欧氏距离 (uT)

    void reset()
    {
        _count = 0;
        _lastUs = 0;
        _sx = _sy = _sz = 0.0f;
        _sxx = _syy = _szz = 0.0f;
        _sxy = _sxz = _syz = 0.0f;
    }

    /**
     * @brief 添加芯片系原始样本（抽稀后可能被拒绝）
     * @return true 样本被接受
     */
    bool addSample(float x, float y, float z, uint32_t nowUs)
    {
        if (_count >= kMaxSamples)
            return false;
        if (_count > 0)
        {
            if ((uint32_t)(nowUs - _lastUs) < kMinIntervalUs)
                return false;
            const Pt &last = _pts[_count - 1];
            const float dx = x - last.x, dy = y - last.y, dz = z - last.z;
            if (dx * dx + dy * dy + dz * dz < kMinSpacingUt * kMinSpacingUt)
                return false;
        }
        _pts[_count++] = {x, y, z};
        _lastUs = nowUs;

        // 流式累积散点矩（覆盖度检测用）
        _sx += x;
        _sy += y;
        _sz += z;
        _sxx += x * x;
        _syy += y * y;
        _szz += z * z;
        _sxy += x * y;
        _sxz += x * z;
        _syz += y * z;
        return true;
    }

    uint16_t count() const { return _count; }

    void sampleMinMax(float mn[3], float mx[3]) const
    {
        mn[0] = mn[1] = mn[2] = 1e30f;
        mx[0] = mx[1] = mx[2] = -1e30f;
        for (uint16_t i = 0; i < _count; i++)
        {
            const Pt &p = _pts[i];
            if (p.x < mn[0]) mn[0] = p.x;
            if (p.y < mn[1]) mn[1] = p.y;
            if (p.z < mn[2]) mn[2] = p.z;
            if (p.x > mx[0]) mx[0] = p.x;
            if (p.y > mx[1]) mx[1] = p.y;
            if (p.z > mx[2]) mx[2] = p.z;
        }
        if (_count == 0)
        {
            mn[0] = mn[1] = mn[2] = 0.0f;
            mx[0] = mx[1] = mx[2] = 0.0f;
        }
    }

    /**
     * @brief 椭球拟合（质量门全过才返回 true）
     * @param out   拟合结果（芯片系；scale 置 0 = 单位球输出）
     * @param stats 质量报告
     */
    bool fit(MagCalParams &out, MagCalStats &stats) const;

private:
    struct Pt
    {
        float x, y, z;
    };

    Pt _pts[kMaxSamples];
    uint16_t _count = 0;
    uint32_t _lastUs = 0;

    // 流式散点矩
    float _sx = 0, _sy = 0, _sz = 0;
    float _sxx = 0, _syy = 0, _szz = 0;
    float _sxy = 0, _sxz = 0, _syz = 0;
};

/* ============================================================================
 * 拟合实现（header-only）
 * ============================================================================ */

inline bool MagCalCollector::fit(MagCalParams &out, MagCalStats &stats) const
{
    stats.count = _count;
    stats.residualRms = 0.0f;
    stats.eigRatio = 0.0f;
    stats.coverageRatio = 0.0f;
    stats.fieldMagUt = 0.0f;

    if (_count < kMinSamples)
        return false;

    // ---- 1. 空间覆盖度：样本散点矩阵 λmin/λmax ----
    {
        const float n = (float)_count;
        const float ux = _sx / n, uy = _sy / n, uz = _sz / n;
        float scat[3][3];
        scat[0][0] = _sxx / n - ux * ux;
        scat[1][1] = _syy / n - uy * uy;
        scat[2][2] = _szz / n - uz * uz;
        scat[0][1] = scat[1][0] = _sxy / n - ux * uy;
        scat[0][2] = scat[2][0] = _sxz / n - ux * uz;
        scat[1][2] = scat[2][1] = _syz / n - uy * uz;
        float w[3], v[3][3];
        jacobiEigen3(scat, w, v);
        if (w[0] <= 0.0f)
            return false;
        stats.coverageRatio = w[2] / w[0]; // w[2] = λmin（已降序）
        if (stats.coverageRatio < 1e-3f)
            return false; // 共面/共线采集：椭球不可辨识
    }

    // ---- 2. 法方程：xᵀQx + uᵀx = 1（以样本均值中心化）----
    // ★ 数据中心化：数学上与原问题严格等价（中心 = c − ½Q⁻¹w），但法方程
    //   条件数显著改善——零均值数据下奇次矩交叉项≈0，二次项子空间与线性项
    //   子空间解耦，硬铁偏置对测量噪声的敏感性降低约一个数量级
    //   （无中心化时 0.2uT 噪声可引入 ~3uT 偏置误差；中心化后 ≤1uT）。
    const float n = (float)_count;
    const float c[3] = {_sx / n, _sy / n, _sz / n};

    float A[9][9];
    float b[9];
    memset(A, 0, sizeof(A));
    memset(b, 0, sizeof(b));
    for (uint16_t i = 0; i < _count; i++)
    {
        const float x = _pts[i].x - c[0];
        const float y = _pts[i].y - c[1];
        const float z = _pts[i].z - c[2];
        const float row[9] = {x * x, y * y, z * z,
                              2.0f * x * y, 2.0f * x * z, 2.0f * y * z,
                              x, y, z};
        for (int r = 0; r < 9; r++)
        {
            b[r] += row[r]; // 目标向量 = 1
            for (int col = 0; col < 9; col++)
                A[r][col] += row[r] * row[col];
        }
    }

    float theta[9];
    if (!solve9(A, b, theta))
        return false;

    // ---- 3. 组装 Q / w，检查正定性 ----
    const float Q[3][3] = {{theta[0], theta[3], theta[4]},
                           {theta[3], theta[1], theta[5]},
                           {theta[4], theta[5], theta[2]}};
    const float w[3] = {theta[6], theta[7], theta[8]};

    float lambda[3], V[3][3];
    jacobiEigen3(Q, lambda, V);
    if (!(lambda[0] > 0.0f && lambda[1] > 0.0f && lambda[2] > 0.0f))
        return false; // Q 非正定：数据不自洽
    stats.eigRatio = lambda[0] / lambda[2];
    if (stats.eigRatio > 25.0f)
        return false; // 退化椭球（近圆柱/平面）

    // ---- 4. 中心（中心化坐标系）b' = −½·Q⁻¹w；原坐标系中心 = c + b' ----
    float Qinv[3][3];
    if (!invert3(Q, Qinv))
        return false;
    const float bP[3] = {-0.5f * (Qinv[0][0] * w[0] + Qinv[0][1] * w[1] + Qinv[0][2] * w[2]),
                         -0.5f * (Qinv[1][0] * w[0] + Qinv[1][1] * w[1] + Qinv[1][2] * w[2]),
                         -0.5f * (Qinv[2][0] * w[0] + Qinv[2][1] * w[1] + Qinv[2][2] * w[2])};

    // s = 1 + b'ᵀQb'（"=1"归一化半径；构造白化矩阵用）
    float qb[3];
    for (int i = 0; i < 3; i++)
        qb[i] = Q[i][0] * bP[0] + Q[i][1] * bP[1] + Q[i][2] * bP[2];
    const float s = 1.0f + bP[0] * qb[0] + bP[1] * qb[1] + bP[2] * qb[2];
    const float radiusUt = sqrtf(s);
    if (!(s > 0.0f) || !isfinite(radiusUt) || radiusUt > 1000.0f)
        return false;

    const float bFinal[3] = {c[0] + bP[0], c[1] + bP[1], c[2] + bP[2]};

    // ---- 5. 白化矩阵 M = Λ^{1/2}·Vᵀ / √s（满足 MᵀM = Q/s，
    //          使 c = M·(x−b) 在椭球面上恒有 |c| = 1）
    //          注意是 Λ 的正平方根：被拉伸的轴要"压"回单位球，反之亦然。 ----
    MagCalParams cand;
    cand.identity();
    for (int i = 0; i < 3; i++)
    {
        const float li = sqrtf(lambda[i]); // 降序特征值
        for (int j = 0; j < 3; j++)
        {
            // M[i][j] = (1/√s)·Λ^{1/2}[i][i]·Vᵀ[i][j] = (1/√s)·li·V[j][i]
            cand.softIron[i][j] = (1.0f / radiusUt) * li * V[j][i];
        }
    }
    cand.hardIron[0] = bFinal[0];
    cand.hardIron[1] = bFinal[1];
    cand.hardIron[2] = bFinal[2];
    // 先保留单位球缩放计算残差；拟合质量确认后再写入物理场强，
    // 使 MagSensor 对外输出 µT（与 MagSample/EKF 参考场同量纲）。
    cand.scale = 0.0f;
    cand.valid = true;

    // ---- 6. 校正残差：||c|−1| 的均方根；同时统计平均半径（物理场强诊断） ----
    double sumE2 = 0.0;
    double sumR = 0.0;
    for (uint16_t i = 0; i < _count; i++)
    {
        const float in[3] = {_pts[i].x, _pts[i].y, _pts[i].z};
        float c[3];
        applyCal(cand, in, c);
        const float e = sqrtf(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]) - 1.0f;
        sumE2 += (double)e * e;
        const float dx = in[0] - bFinal[0], dy = in[1] - bFinal[1], dz = in[2] - bFinal[2];
        sumR += (double)sqrtf(dx * dx + dy * dy + dz * dz);
    }
    stats.residualRms = (float)sqrt(sumE2 / (double)_count);
    if (stats.residualRms > 0.15f)
        return false;

    // 平均半径 = 原始场强估计（比 √s 更稳健，不受"=1"归一化偏移影响）
    stats.fieldMagUt = (float)(sumR / (double)_count);
    if (!(stats.fieldMagUt >= 5.0f && stats.fieldMagUt <= 200.0f))
        return false;
    // 对外输出物理场强（µT），不能把单位球参数直接交给 EKF/持久化层。
    cand.scale = stats.fieldMagUt;

    out = cand;
    return true;
}

} // namespace magcal

#endif // MAG_CAL_MAG_CAL_MATH_H
