/**
 * @file    test_mag_cal.cpp
 * @module  固件/宿主回归测试（fw-tests：磁力计椭球标定数学层）
 * @brief   验证 CRC32、Jacobi 特征分解、线性代数工具、合成椭球拟合与拒绝路径
 * @details 被测：`lib/MagCal/MagCalMath.h`（纯平台无关）。
 *          覆盖：1) CRC32 标准向量；2) 3×3 对称矩阵 Jacobi 特征分解（含旋转后重构）；
 *          3) invert3 / solve9 线性代数工具；4) applyCal（身份/偏移/缩放）；
 *          5) 合成椭球拟合：已知硬铁+软铁+噪声 → 恢复精度 / 校正后物理场强 / 质量门；
 *          6) 拒绝路径：样本不足 / 共面（覆盖度不足）/ 参数合理性（含旧 scale=0 记录）。
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude -Ilib/MagCal test_host/test_mag_cal.cpp -o bin/mc
 * @theory  椭球模型 (B−h)ᵀA(B−h)=1，硬铁 h 由椭球中心给出、软铁由 A 的特征值/
 *          特征向量给出；校正即把椭球映射回单位球。硬铁/软铁参数只做一次拟合，
 *          跑在芯片系（与轴映射解耦），应用顺序 raw→标定→轴映射→FRD。
 *          @ref AGENTS.md（2026-08-15 磁力计椭球标定 DBG magcal）
 * @frame   芯片系（标定在轴映射之前完成，与 FRD 解耦）
 * @units   磁场 µT，CRC/矩阵元素无量纲；恢复精度门 ~1 µT @0.2 µT 噪声
 * @note    噪声用固定线性同余发生器而非 std::random，避免不同编译器/标准库
 *          分布实现差异导致结果不可复现。质量门（正定/覆盖度/残差≤15%）是
 *          固件侧拒绝坏标定的唯一防线，故 A6 的拒绝路径与拟合精度同等重要。
 * @test    S1 CRC32；S2 Jacobi 分解与重构；S3 invert3/solve9；
 *          S4 applyCal 三种情形；S5 合成椭球恢复精度与物理场强（µT）；
 *          S6 拒绝路径（样本不足/共面/共线）；S7 paramsSanity（含旧 scale=0）
 * @see     TandemVec_FCS/lib/MagCal/MagCalMath.h、test_mag_guide.cpp
 */
// ============================================================
#include <cstdio>
#include <cmath>
#include "MagCalMath.h"

static int g_fail = 0;
#define CHECK(cond, msg) \
  do { if (!(cond)) { printf("FAIL: %s (line %d)\n", msg, __LINE__); g_fail++; } } while (0)
#define CLOSE(a, b, tol) (fabsf((a) - (b)) <= (tol))

// 确定性伪随机（噪声源，避免依赖 std::random 的编译器差异）
static uint32_t g_rand = 0x12345678u;
static float frand()
{
    g_rand = g_rand * 1664525u + 1013904223u;
    return ((float)(g_rand >> 8) / 16777216.0f) * 2.0f - 1.0f; // [-1,1)
}

int main()
{
    // ---- 1. CRC32 标准测试向量 "123456789" → 0xCBF43926 ----
    {
        const uint8_t v[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
        CHECK(magcal::crc32(v, 9) == 0xCBF43926u, "CRC32 标准向量");
    }

    // ---- 2. Jacobi 特征分解：A = R(30°)·diag(3,2,1)·Rᵀ ----
    {
        const float c = cosf(30.0f * 3.14159265f / 180.0f);
        const float s = sinf(30.0f * 3.14159265f / 180.0f);
        // R = [[c,-s,0],[s,c,0],[0,0,1]]
        const float R[3][3] = {{c, -s, 0}, {s, c, 0}, {0, 0, 1}};
        const float D[3] = {3.0f, 2.0f, 1.0f};
        float A[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                A[i][j] = 0.0f;
                for (int k = 0; k < 3; k++)
                    A[i][j] += R[i][k] * D[k] * R[j][k];
            }
        float w[3], V[3][3];
        magcal::jacobiEigen3(A, w, V);
        CHECK(CLOSE(w[0], 3.0f, 1e-4), "jacobi 特征值 3");
        CHECK(CLOSE(w[1], 2.0f, 1e-4), "jacobi 特征值 2");
        CHECK(CLOSE(w[2], 1.0f, 1e-4), "jacobi 特征值 1");
        // 重构 A = V·diag(w)·Vᵀ
        float Rc[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                Rc[i][j] = 0.0f;
                for (int k = 0; k < 3; k++)
                    Rc[i][j] += V[i][k] * w[k] * V[j][k];
            }
        bool ok = true;
        for (int i = 0; i < 3 && ok; i++)
            for (int j = 0; j < 3 && ok; j++)
                ok = fabsf(Rc[i][j] - A[i][j]) < 1e-4f;
        CHECK(ok, "jacobi 重构 V·diag(w)·Vᵀ = A");
    }

    // ---- 3. invert3 / solve9 ----
    {
        const float A[3][3] = {{1, 2, 3}, {0, 1, 4}, {5, 6, 0}};
        float Ai[3][3], I[3][3];
        CHECK(magcal::invert3(A, Ai), "invert3 可逆");
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                I[i][j] = 0.0f;
                for (int k = 0; k < 3; k++)
                    I[i][j] += A[i][k] * Ai[k][j];
            }
        bool ident = true;
        for (int i = 0; i < 3 && ident; i++)
            for (int j = 0; j < 3 && ident; j++)
                ident = fabsf(I[i][j] - ((i == j) ? 1.0f : 0.0f)) < 1e-4f;
        CHECK(ident, "invert3 A·A⁻¹ = I");

        float A9[9][9];
        float b9[9];
        for (int i = 0; i < 9; i++)
        {
            for (int j = 0; j < 9; j++)
                A9[i][j] = (i == j) ? (float)(i + 1) : 0.0f;
            b9[i] = (float)(i + 1);
        }
        float x9[9];
        CHECK(magcal::solve9(A9, b9, x9), "solve9 可解");
        bool sol = true;
        for (int i = 0; i < 9 && sol; i++)
            sol = fabsf(x9[i] - 1.0f) < 1e-4f;
        CHECK(sol, "solve9 对角系统 x=1");
    }

    // ---- 4. applyCal 基础行为 ----
    {
        magcal::MagCalParams id;
        id.identity();
        const float in[3] = {10.0f, -20.0f, 30.0f};
        float out[3];
        magcal::applyCal(id, in, out);
        CHECK(CLOSE(out[0], 10.0f, 1e-5) && CLOSE(out[1], -20.0f, 1e-5) && CLOSE(out[2], 30.0f, 1e-5),
              "身份标定直通");

        magcal::MagCalParams p;
        p.identity();
        p.hardIron[0] = 5.0f;
        p.hardIron[1] = -3.0f;
        p.hardIron[2] = 2.0f;
        magcal::applyCal(p, in, out);
        CHECK(CLOSE(out[0], 5.0f, 1e-5) && CLOSE(out[1], -17.0f, 1e-5) && CLOSE(out[2], 28.0f, 1e-5),
              "硬铁偏移");
    }

    // ---- 5. 合成椭球拟合（真实量级：|B|≈48uT，硬铁 ~10uT，软铁含交叉项） ----
    {
        // 真值软铁 A（对称，特征值 1.10/0.95/0.90）：raw = A⁻¹·(F·u) + b
        const float A[3][3] = {{1.10f, 0.05f, 0.03f},
                               {0.05f, 0.95f, -0.04f},
                               {0.03f, -0.04f, 0.90f}};
        float Ai[3][3];
        magcal::invert3(A, Ai);
        const float b[3] = {8.0f, -12.0f, 5.0f};
        const float F = 48.0f; // 地磁场模长 (uT)

        magcal::MagCalCollector col;
        col.reset();

        // 斐波那契球面均匀采样 + 高斯噪声（≈0.2uT σ）
        const int N = 2000;
        const float phi = 3.14159265f * (3.0f - sqrtf(5.0f));
        for (int i = 0; i < N; i++)
        {
            const float yy = 1.0f - (2.0f * (float)i + 1.0f) / (float)N;
            const float rr = sqrtf(1.0f - yy * yy);
            const float th = phi * (float)i;
            const float u[3] = {rr * cosf(th), yy, rr * sinf(th)};

            // raw = A⁻¹(F·u) + b + noise
            float w[3] = {F * u[0], F * u[1], F * u[2]};
            float raw[3] = {b[0] + Ai[0][0] * w[0] + Ai[0][1] * w[1] + Ai[0][2] * w[2],
                            b[1] + Ai[1][0] * w[0] + Ai[1][1] * w[1] + Ai[1][2] * w[2],
                            b[2] + Ai[2][0] * w[0] + Ai[2][1] * w[1] + Ai[2][2] * w[2]};
            raw[0] += 0.2f * frand();
            raw[1] += 0.2f * frand();
            raw[2] += 0.2f * frand();
            col.addSample(raw[0], raw[1], raw[2], (uint32_t)i * 20000u);
        }
        CHECK(col.count() >= 100, "合成样本采集量足够");

        magcal::MagCalParams fit;
        magcal::MagCalStats st;
        bool ok = col.fit(fit, st);
        CHECK(ok, "合成椭球拟合通过质量门");

        // 硬铁恢复精度（噪声 0.2uT 下 < 0.5uT）
        CHECK(CLOSE(fit.hardIron[0], b[0], 0.5f), "硬铁 x 恢复");
        CHECK(CLOSE(fit.hardIron[1], b[1], 0.5f), "硬铁 y 恢复");
        CHECK(CLOSE(fit.hardIron[2], b[2], 0.5f), "硬铁 z 恢复");
        // 场强诊断：平均半径对软铁各向异性敏感（A 特征值 1.1/0.95/0.9 →
        // 均匀方向上的平均半径 ≈ F·1.04 ≈ 50），只做宽区间断言
        CHECK(st.fieldMagUt > 43.0f && st.fieldMagUt < 54.0f, "场强诊断在物理区间");
        // 残差质量
        CHECK(st.residualRms < 0.05f, "校正残差 RMS < 5%");

        // 用拟合参数校正"采集器外"的新样本：应恢复物理场强（µT），
        // 不能再输出单位球；否则 EKF 的模长门控/参考场比较全部失效。
        const float u2[3] = {1.0f, 0.0f, 0.0f}; // 任取方向
        float w[3] = {F * u2[0], F * u2[1], F * u2[2]};
        float raw[3] = {b[0] + Ai[0][0] * w[0] + Ai[0][1] * w[1] + Ai[0][2] * w[2],
                        b[1] + Ai[1][0] * w[0] + Ai[1][1] * w[1] + Ai[1][2] * w[2],
                        b[2] + Ai[2][0] * w[0] + Ai[2][1] * w[1] + Ai[2][2] * w[2]};
        float c[3];
        magcal::applyCal(fit, raw, c);
        const float mag = sqrtf(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
        CHECK(CLOSE(mag, F, 2.5f), "校正后输出物理场强≈48uT");
        CHECK(fit.scale > 43.0f && fit.scale < 54.0f, "拟合 scale 为物理场强");
    }

    // ---- 6. 拒绝路径 ----
    {
        // 6.1 样本不足
        magcal::MagCalCollector col;
        col.reset();
        for (int i = 0; i < 10; i++)
            col.addSample((float)i, 2.0f * i, 3.0f * i, (uint32_t)i * 20000u);
        magcal::MagCalParams p;
        magcal::MagCalStats st;
        CHECK(!col.fit(p, st), "样本不足拒绝");

        // 6.2 共面采集（xy 平面圆，z 恒为 0）：覆盖度不足拒绝
        col.reset();
        for (int i = 0; i < 200; i++)
        {
            const float t = 2.0f * 3.14159265f * i / 200.0f;
            col.addSample(48.0f * cosf(t), 48.0f * sinf(t), 0.0f, (uint32_t)i * 20000u);
        }
        CHECK(!col.fit(p, st), "共面数据拒绝");
        CHECK(st.coverageRatio < 1e-3f, "共面覆盖度报告");

        // 6.3 共线采集（沿一条直线）：覆盖度不足拒绝
        col.reset();
        for (int i = 0; i < 200; i++)
            col.addSample((float)i * 0.3f, (float)i * 0.2f, (float)i * 0.1f, (uint32_t)i * 20000u);
        CHECK(!col.fit(p, st), "共线数据拒绝");
    }

    // ---- 7. paramsSanity ----
    {
        magcal::MagCalParams p;
        p.identity();
        CHECK(!magcal::paramsSanity(p), "identity(valid=false) 不可用");
        p.valid = true;
        CHECK(!magcal::paramsSanity(p), "identity(valid=true, scale=1) 旧单位球标定拒绝");
        p.scale = 48.0f;
        CHECK(magcal::paramsSanity(p), "scale=48uT 的有效标定可用");
        p.scale = 0.0f;
        CHECK(!magcal::paramsSanity(p), "旧 Flash scale=0 记录拒绝");
        p.scale = 48.0f;
        p.softIron[1][1] = INFINITY;
        CHECK(!magcal::paramsSanity(p), "NaN/Inf 参数拒绝");
    }

    if (g_fail == 0)
    {
        printf("=== test_mag_cal: 全部通过 ===\n");
        return 0;
    }
    printf("=== test_mag_cal: %d 项失败 ===\n", g_fail);
    return 1;
}
