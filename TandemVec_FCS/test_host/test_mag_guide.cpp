/**
 * @file    test_mag_guide.cpp
 * @module  固件/宿主回归测试（fw-tests：两圈引导校准状态机）
 * @brief   验证水平旋转→机头朝下旋转的两阶段流程、判定门与扇区跨边界
 * @details 被测：`lib/MagCal/MagCalGuide.h`（纯平台无关状态机）。
 *          覆盖：1) 阶段 1 水平旋转：姿态/旋转/扇区覆盖判定，≥315° 自动切阶段 2；
 *          2) 阶段 2 机头朝下旋转：完成 → DONE；3) 判定门：加速度模长越界 /
 *          次轴旋转过大 / 姿态不对 → 拒绝采集；4) DONE 后停止动作、reset 回 IDLE、
 *          扇区跨 ±π 边界。
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude -Ilib/MagCal test_host/test_mag_guide.cpp -o bin/mg
 * @theory  大疆式两圈校准：水平圈覆盖磁航向全周，机头朝下圈覆盖第三个自由度；
 *          每个阶段要求姿态门（|acc|≈g、主导轴 ≥0.7|acc|、主导转速 0.3–6 rad/s、
 *          次轴 <0.45×主导）成立才采信样本。
 *          @ref AGENTS.md（2026-08-15 两圈引导校准 DBG magcal guided）
 * @frame   机体系 FRD（加速度/角速率均机体系；磁方位在水平面内按 ±π 归一）
 * @units   加速度 m/s²，角速率 rad/s，角度 deg/rad，扇区 45°×8
 * @note    315° 覆盖阈值 = 7/8 圈，留 45° 余量避免操作者收尾时反复补转；
 *          扇区跨 ±π 的用例来自现场“从 -175° 起转”的真实操作习惯。
 * @test    S1 完整两圈流程；S2 加速度模长门；S3 姿态门；S4 次轴旋转门；
 *          S5 转速过快/过慢拒绝；S6 magFresh=false 语义；S7 扇区跨 ±π；
 *          S8 reset / FAILED 终态
 * @see     TandemVec_FCS/lib/MagCal/MagCalGuide.h、test_mag_cal.cpp
 */
// ============================================================
#include <cstdio>
#include <cmath>
#include "MagCalGuide.h"

static int g_fail = 0;
#define CHECK(cond, msg) \
  do { if (!(cond)) { printf("FAIL: %s (line %d)\n", msg, __LINE__); g_fail++; } } while (0)

int main()
{
    const float G = magcal::MagCalGuide::kGravity;
    const float accHoriz[3] = {0.0f, 0.0f, G};      // 水平：重力沿 +z_b
    const float accNoseDown[3] = {G, 0.0f, 0.0f};   // 机头朝下：重力沿 +x_b
    const float gyrZ[3] = {0.0f, 0.0f, 1.0f};       // 绕竖直轴 1 rad/s
    const float gyrX[3] = {1.0f, 0.0f, 0.0f};
    const float magN[3] = {30.0f, 0.0f, 0.0f};

    // ---- 1. 完整两圈流程 ----
    {
        magcal::MagCalGuide guide;
        guide.start();
        CHECK(guide.stage() == magcal::GuideStage::HORIZONTAL, "start 进入阶段1");

        // 阶段1：水平旋转，磁方位随旋转走一圈
        float theta = 0.0f;
        bool switched = false;
        float switchAngle = 0.0f;
        for (int i = 0; i < 1000 && guide.stage() == magcal::GuideStage::HORIZONTAL; i++)
        {
            const float mag[3] = {30.0f * cosf(theta), 30.0f * sinf(theta), 0.0f};
            CHECK(guide.update(accHoriz, gyrZ, mag, true), "阶段1 合格帧可采集");
            if (guide.stage() == magcal::GuideStage::NOSE_DOWN)
            {
                switched = true;
                switchAngle = theta;
                break;
            }
            theta += 0.02f;
        }
        CHECK(switched, "阶段1 覆盖 315° 后自动切阶段2");
        // 覆盖 7 个 45° 扇区：从扇区中心（0°）起转约 270° 即达，
        // 最坏（从扇区边缘起转）315°——断言窗口覆盖两种起点
        CHECK(switchAngle >= 4.4f && switchAngle <= 5.6f, "阶段1 切换发生在 252°~321°");
        {
            magcal::GuideStatus st;
            guide.status(st);
            CHECK(st.sectorsDone == 0, "切阶段后扇区清零");
        }

        // 阶段2：机头朝下，磁矢量在 y-z 平面转一圈
        float phi = 0.0f;
        for (int i = 0; i < 1000 && guide.stage() == magcal::GuideStage::NOSE_DOWN; i++)
        {
            const float mag[3] = {0.0f, 30.0f * cosf(phi), 30.0f * sinf(phi)};
            guide.update(accNoseDown, gyrX, mag, true);
            phi += 0.02f;
        }
        CHECK(guide.stage() == magcal::GuideStage::DONE, "阶段2 完成 → DONE");
        CHECK(!guide.update(accHoriz, gyrZ, magN, true), "DONE 后停止动作");
    }

    // ---- 2. 判定门：加速度模长越界 ----
    {
        magcal::MagCalGuide guide;
        guide.start();
        const float accLo[3] = {0, 0, 8.0f}; // 8.0 < 0.85g
        CHECK(!guide.update(accLo, gyrZ, magN, true), "加速度模长过小拒绝");
        const float accHi[3] = {0, 0, 12.0f}; // > 1.15g
        CHECK(!guide.update(accHi, gyrZ, magN, true), "加速度模长过大拒绝");
    }

    // ---- 3. 判定门：姿态不对（阶段1 要求 az 主导）----
    {
        magcal::MagCalGuide guide;
        guide.start();
        const float accTilt[3] = {0.72f * G, 0, 0.68f * G}; // 倾斜：az/|acc| < 0.7
        CHECK(!guide.update(accTilt, gyrZ, magN, true), "倾斜姿态拒绝（阶段1）");
    }

    // ---- 4. 判定门：次轴旋转过大（不是绕竖直轴）----
    {
        magcal::MagCalGuide guide;
        guide.start();
        const float gyrBad[3] = {0.0f, 0.5f, 1.0f}; // 次轴 0.5 ≥ 0.45×1.0
        CHECK(!guide.update(accHoriz, gyrBad, magN, true), "次轴旋转过大拒绝");
    }

    // ---- 5. 旋转过快/过慢拒绝 ----
    {
        magcal::MagCalGuide guide;
        guide.start();
        const float gyrSlow[3] = {0, 0, 0.1f};
        CHECK(!guide.update(accHoriz, gyrSlow, magN, true), "转速过慢拒绝");
        const float gyrFast[3] = {0, 0, 7.0f};
        CHECK(!guide.update(accHoriz, gyrFast, magN, true), "转速过快拒绝");
    }

    // ---- 6. magFresh=false 时不更新扇区但仍可采集 ----
    {
        magcal::MagCalGuide guide;
        guide.start();
        CHECK(guide.update(accHoriz, gyrZ, magN, false), "无新磁样本仍可采集");
        magcal::GuideStatus st;
        guide.status(st);
        CHECK(st.sectorsDone == 0, "无新磁样本不更新扇区");
    }

    // ---- 7. 扇区跨 ±π 边界：从 -175° 起转，覆盖所有扇区 ----
    {
        magcal::MagCalGuide guide;
        guide.start();
        for (int i = 0; i < 900 && guide.stage() == magcal::GuideStage::HORIZONTAL; i++)
        {
            const float th = -3.05f + 0.02f * (float)i; // -175° 起
            const float mag[3] = {30.0f * cosf(th), 30.0f * sinf(th), 0.0f};
            guide.update(accHoriz, gyrZ, mag, true);
        }
        CHECK(guide.stage() == magcal::GuideStage::NOSE_DOWN, "跨 ±π 边界覆盖正常切换");
    }

    // ---- 8. reset / FAILED ----
    {
        magcal::MagCalGuide guide;
        guide.start();
        guide.reset();
        CHECK(guide.stage() == magcal::GuideStage::IDLE, "reset 回 IDLE");
        guide.start();
        guide.setFailed();
        CHECK(guide.stage() == magcal::GuideStage::FAILED, "setFailed");
        CHECK(!guide.update(accHoriz, gyrZ, magN, true), "FAILED 后停止动作");
    }

    if (g_fail == 0)
    {
        printf("=== test_mag_guide: 全部通过 ===\n");
        return 0;
    }
    printf("=== test_mag_guide: %d 项失败 ===\n", g_fail);
    return 1;
}
