// ============================================================
//  test_mag_parse.cpp — 磁力计解析/轴对齐数学层 host 测试
//
//  纯平台无关：验证 lib/MagSensor/MagMath.h ——
//    1. IST8310 / QMC5883L 灵敏度换算（数据手册典型值）
//    2. 6 字节小端 + 二进制补码解码
//    3. 芯片系 -> 机体 FRD 的旋转矩阵对齐（含硬铁偏置）
//    4. 全链路：芯片原始数据 -> FRD -> TiltCompass 北向判定
//       （地磁场在北半球指"北+下"，FRD 的 z 分量必须为正，
//        这是任何新芯片轴映射正误的物理判据）
// ============================================================
#include <cstdio>
#include <cmath>
#include "MagMath.h"
#include "tilt_compass.h"

static int g_fail = 0;
#define CHECK(cond, msg) \
  do { if (!(cond)) { printf("FAIL: %s (line %d)\n", msg, __LINE__); g_fail++; } } while (0)
#define CLOSE(a, b, tol) (fabsf((a) - (b)) <= (tol))

int main()
{
    // ---- 1. IST8310 灵敏度：0.3 uT/LSB（数据手册典型值） ----
    CHECK(CLOSE(magmath::ist8310LsToUt(100), 30.0f, 1e-4), "IST8310 100LSB = 30uT");
    CHECK(CLOSE(magmath::ist8310LsToUt(-1000), -300.0f, 1e-3), "IST8310 负值");
    CHECK(CLOSE(magmath::ist8310LsToUt(330), 99.0f, 1e-3),
          "IST8310 330LSB = 99uT（旧 0.30303 系数会得 100，+1% 偏差已修正）");

    // ---- 2. QMC5883L 灵敏度：±2G=12000 LSB/G, ±8G=3000 LSB/G ----
    CHECK(CLOSE(magmath::qmc5883lLsToUt(12000, 0), 100.0f, 1e-4), "QMC ±2G 12000LSB=1G=100uT");
    CHECK(CLOSE(magmath::qmc5883lLsToUt(3000, 1), 100.0f, 1e-4), "QMC ±8G 3000LSB=1G=100uT");
    CHECK(CLOSE(magmath::qmc5883lLsToUt(12000, 1), 400.0f, 1e-3), "QMC ±8G 12000LSB=4G=400uT");
    CHECK(CLOSE(magmath::qmc5883lLsToUt(-3000, 1), -100.0f, 1e-4), "QMC ±8G 负值");

    // ---- 3. 小端组包 + 二进制补码解码 ----
    // buf = {0x34, 0x12, ...} -> 0x1234 = 4660
    {
        uint8_t buf[6] = {0x34, 0x12, 0x00, 0x00, 0x00, 0x00};
        float out[3] = {0, 0, 0};
        magmath::qmc5883lDecode(buf, 1, out);
        CHECK(CLOSE(out[0], 4660.0f * (100.0f / 3000.0f), 1e-3), "QMC 小端组包 0x1234");
    }
    // buf = {0x00, 0x80, ...} -> -32768（补码）
    {
        uint8_t buf[6] = {0x00, 0x80, 0xFF, 0xFF, 0x01, 0x00};
        float out[3] = {0, 0, 0};
        magmath::qmc5883lDecode(buf, 1, out);
        CHECK(CLOSE(out[0], -32768.0f * (100.0f / 3000.0f), 1e-2), "QMC 补码 -32768");
        CHECK(CLOSE(out[1], -1.0f * (100.0f / 3000.0f), 1e-4), "QMC 补码 -1");
        CHECK(CLOSE(out[2], 1.0f * (100.0f / 3000.0f), 1e-4), "QMC 补码 +1");
    }
    // IST8310 解码同小端路径，仅灵敏度不同
    {
        uint8_t buf[6] = {0x34, 0x12, 0x00, 0x00, 0x00, 0x00};
        float out[3] = {0, 0, 0};
        magmath::ist8310Decode(buf, out);
        CHECK(CLOSE(out[0], 4660.0f * 0.3f, 1e-3), "IST 小端组包 0x1234");
    }

    // ---- 4. 芯片系 -> 机体 FRD 轴对齐 ----
    // IST8310 本机安装：body_x=-chip_y, body_y=+chip_x, body_z=-chip_z
    {
        const float chip[3] = {10.0f, 20.0f, 30.0f};
        const float off[3] = {0, 0, 0};
        float out[3] = {0, 0, 0};
        magmath::magAlignToBody(chip, magmath::kAxisIst8310ToFrd, off, out);
        CHECK(CLOSE(out[0], -20.0f, 1e-4), "IST 轴对齐 x=-chip_y");
        CHECK(CLOSE(out[1], 10.0f, 1e-4), "IST 轴对齐 y=+chip_x");
        CHECK(CLOSE(out[2], -30.0f, 1e-4), "IST 轴对齐 z=-chip_z");
    }
    // QMC5883L 默认：body_x=+chip_x, body_y=+chip_y, body_z=-chip_z
    {
        const float chip[3] = {10.0f, 20.0f, 30.0f};
        const float off[3] = {0, 0, 0};
        float out[3] = {0, 0, 0};
        magmath::magAlignToBody(chip, magmath::kAxisQmc5883lToFrd, off, out);
        CHECK(CLOSE(out[0], 10.0f, 1e-4), "QMC 轴对齐 x=+chip_x");
        CHECK(CLOSE(out[1], 20.0f, 1e-4), "QMC 轴对齐 y=+chip_y");
        CHECK(CLOSE(out[2], -30.0f, 1e-4), "QMC 轴对齐 z=-chip_z（芯片Z朝上->机体Z朝下）");
    }
    // 硬铁偏置扣除
    {
        const float chip[3] = {10.0f, 20.0f, 30.0f};
        const float off[3] = {1.0f, 2.0f, 3.0f};
        float out[3] = {0, 0, 0};
        magmath::magAlignToBody(chip, magmath::kAxisQmc5883lToFrd, off, out);
        CHECK(CLOSE(out[0], 9.0f, 1e-4), "偏置扣除 x");
        CHECK(CLOSE(out[1], 18.0f, 1e-4), "偏置扣除 y");
        CHECK(CLOSE(out[2], -33.0f, 1e-4), "偏置扣除 z");
    }

    // ---- 5. 全链路物理判据：机头朝北平放 ----
    // 长沙磁倾角约 45°，总场约 48uT：FRD 磁向量 ≈ (34, 0, 34)，z 为正（指向下）。
    // 反解 IST8310 芯片读数：chip_x=body_y=0, chip_y=-body_x=-34, chip_z=-body_z=-34
    {
        const float chip[3] = {0.0f, -34.0f, -34.0f};
        const float off[3] = {0, 0, 0};
        float frd[3] = {0, 0, 0};
        magmath::magAlignToBody(chip, magmath::kAxisIst8310ToFrd, off, frd);
        CHECK(CLOSE(frd[0], 34.0f, 1e-3), "全链路 FRD x=+34");
        CHECK(CLOSE(frd[1], 0.0f, 1e-3), "全链路 FRD y=0");
        CHECK(CLOSE(frd[2], 34.0f, 1e-3), "全链路 FRD z=+34（北半球 z 必须为正）");

        // TiltCompass：平放（accel 朝下），磁北在机头 -> yaw = 0
        Eigen::Vector3f accel(0.0f, 0.0f, 9.79f);
        Eigen::Vector3f mag(frd[0], frd[1], frd[2]);
        Eigen::Vector3f ypr = bfs::TiltCompass(accel, mag);
        CHECK(fabsf(ypr(0)) < 1e-3, "机头朝北 -> yaw=0");
        CHECK(fabsf(ypr(1)) < 1e-3, "平放 -> pitch=0");
        CHECK(fabsf(ypr(2)) < 1e-3, "平放 -> roll=0");
    }
    // 机头朝东平放：北在左侧 -> FRD = (0, -34, 34)（NED 航向约定东=+90°）
    {
        Eigen::Vector3f accel(0.0f, 0.0f, 9.79f);
        Eigen::Vector3f mag(0.0f, -34.0f, 34.0f);
        Eigen::Vector3f ypr = bfs::TiltCompass(accel, mag);
        CHECK(CLOSE(ypr(0), 3.14159265f / 2.0f, 1e-3), "机头朝东 -> yaw=+90°(NED)");
    }

    if (g_fail == 0)
    {
        printf("=== test_mag_parse: 全部通过 ===\n");
        return 0;
    }
    printf("=== test_mag_parse: %d 项失败 ===\n", g_fail);
    return 1;
}
