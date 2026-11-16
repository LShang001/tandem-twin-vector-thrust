/**
 * @file    test_mag_axis_coax.cpp
 * @module  固件/宿主回归测试（fw-tests：共轴机型磁轴对齐表 + 编译期标定参数）
 * @brief   锁定共轴双桨户外实测轴表与 mag_cal_defaults 参数转录（2026-11-15 户外重标移植）
 * @details 被测：`lib/MagSensor/MagMath.h`（BFS_COAXIAL_VTVL 分支）与
 *          `include/mag_cal_defaults.h`（共轴分支）。本文件在包含头文件前自行
 *          定义 BFS_COAXIAL_VTVL，与 run_all.sh 其它段同命令编译即可；纵列/
 *          单涵道的原表断言在 test_mag_parse.cpp（不带机型宏）。
 *          编译：g++ -std=c++17 -Iinclude -Ilib/MagSensor -Ilib/MagCal
 *                -Ilib/eigen/src -Ilib/navigation-main/src
 *                test_host/test_mag_axis_coax.cpp -o bin/mgcoax
 * @theory  轴对齐是整数旋转/反射表；椭球拟合的软铁白化矩阵含不可观旋转（约+90°），
 *          标定参数与轴表是户外实测组合（机头朝北→yaw≈0、朝东→+90°），换任一项
 *          必须上板 magraw 复核。参数来源：同源 VTVL 固件包 2026-11-15 户外重标
 *          （同一架共轴实机 45686 配置，用户 2026-11-16 确认机体同一）。
 *          @ref MOD-002 §0（FRD 机体系）、docs/registers/踩坑记录-AGENTS.md 2026-11-16 节
 * @frame   芯片系 → 机体 FRD（x 前/y 右/z 下）；参考场 NED（北/东/下）
 * @units   磁场 uT，角度 rad，加速度 m/s²
 * @test    C1 共轴轴表逐元素；C2 全链路机头朝北水平 FRD=(34,0,34) + TiltCompass
 *          yaw≈0；C3 defaults kValid/scale/hardIron/softIron 与源值逐项一致；
 *          C4 paramsSanity 通过
 * @see     test_mag_parse.cpp（原表/无机型宏）、lib/MagSensor/MagMath.h、
 *          include/mag_cal_defaults.h
 */
#define BFS_COAXIAL_VTVL 1
#include <cstdio>
#include <cmath>
#include "MagMath.h"
#include "mag_cal_defaults.h"
#include "MagCalMath.h"
#include "tilt_compass.h"

static int g_fail = 0;
#define CHECK(cond, msg) \
  do { if (!(cond)) { printf("FAIL: %s (line %d)\n", msg, __LINE__); g_fail++; } } while (0)
#define CLOSE(a, b, tol) (std::fabs((double)(a) - (double)(b)) <= (double)(tol))

int main()
{
    // ---- C1 共轴户外实测轴表：body_x=+chip_x, body_y=-chip_y, body_z=-chip_z ----
    {
        const float chip[3] = {10.0f, 20.0f, 30.0f};
        const float off[3] = {0, 0, 0};
        float out[3] = {0, 0, 0};
        magmath::magAlignToBody(chip, magmath::kAxisIst8310ToFrd, off, out);
        CHECK(CLOSE(out[0], 10.0f, 1e-4), "C1 共轴 IST 轴对齐 x=+chip_x");
        CHECK(CLOSE(out[1], -20.0f, 1e-4), "C1 共轴 IST 轴对齐 y=-chip_y");
        CHECK(CLOSE(out[2], -30.0f, 1e-4), "C1 共轴 IST 轴对齐 z=-chip_z");
    }

    // ---- C2 全链路物理判据：机头朝北平放，FRD=(34,0,34)，z 为正 ----
    // 共轴表反解：chip=(body_x, -body_y, -body_z)=(34,0,-34)
    {
        const float chip[3] = {34.0f, 0.0f, -34.0f};
        const float off[3] = {0, 0, 0};
        float frd[3] = {0, 0, 0};
        magmath::magAlignToBody(chip, magmath::kAxisIst8310ToFrd, off, frd);
        CHECK(CLOSE(frd[0], 34.0f, 1e-3), "C2 全链路 FRD x=+34");
        CHECK(CLOSE(frd[1], 0.0f, 1e-3), "C2 全链路 FRD y=0");
        CHECK(CLOSE(frd[2], 34.0f, 1e-3), "C2 全链路 FRD z=+34（北半球 z 必须为正）");

        // TiltCompass：平放（重力方向 +z），磁北在机头 -> yaw≈0
        const Eigen::Vector3f accel(0.0f, 0.0f, 9.79f);
        const Eigen::Vector3f mag(frd[0], frd[1], frd[2]);
        const Eigen::Vector3f ypr = bfs::TiltCompass(accel, mag);
        CHECK(CLOSE(ypr(0), 0.0f, 1e-3), "C2 机头朝北 yaw≈0");
        CHECK(CLOSE(ypr(1), 0.0f, 1e-3), "C2 平放 pitch≈0");
        CHECK(CLOSE(ypr(2), 0.0f, 1e-3), "C2 平放 roll≈0");
    }

    // ---- C3 编译期默认标定参数与源值逐项一致（防转录错） ----
    {
        CHECK(magcal_defaults::kValid, "C3 共轴 defaults kValid=true");
        CHECK(CLOSE(magcal_defaults::kScale, 41.8343f, 1e-4), "C3 scale=41.8343");
        CHECK(CLOSE(magcal_defaults::kHardIron[0], -3.6144f, 1e-4), "C3 b0=-3.6144");
        CHECK(CLOSE(magcal_defaults::kHardIron[1], 15.5312f, 1e-4), "C3 b1=15.5312");
        CHECK(CLOSE(magcal_defaults::kHardIron[2], -4.6636f, 1e-4), "C3 b2=-4.6636");
        const float srcM[3][3] = {
            {-0.0027f, 0.0218f, -0.0136f},
            {0.0237f, 0.0014f, -0.0023f},
            {0.0012f, 0.0122f, 0.0194f}};
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                CHECK(CLOSE(magcal_defaults::kSoftIron[r][c], srcM[r][c], 1e-6),
                      "C3 softIron 与源值一致");
    }

    // ---- C4 参数过 paramsSanity（scale 5–200 uT 等质量门） ----
    {
        magcal::MagCalParams p;
        p.identity();
        p.valid = true;
        p.scale = magcal_defaults::kScale;
        for (int i = 0; i < 3; ++i)
        {
            p.hardIron[i] = magcal_defaults::kHardIron[i];
            for (int j = 0; j < 3; ++j)
                p.softIron[i][j] = magcal_defaults::kSoftIron[i][j];
        }
        CHECK(magcal::paramsSanity(p), "C4 共轴 defaults 过 paramsSanity");
    }

    if (g_fail == 0)
    {
        printf("=== test_mag_axis_coax: 全部通过 ===\n");
        return 0;
    }
    printf("=== test_mag_axis_coax: %d 项失败 ===\n", g_fail);
    return 1;
}
